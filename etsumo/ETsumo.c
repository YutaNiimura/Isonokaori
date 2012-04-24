/**
 ******************************************************************************
 **	ファイル名 : ETsumo.c
 **
 **	概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cプログラム
 **
 ** 注記 : ET相撲攻略プログラム
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */

/* It is necessary to adjust following parameters for each robots/environment */
#define GYRO_OFFSET  602 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE	 530 /* 白色の光センサ値 */
#define LIGHT_BLACK	 740 /* 黒色の光センサ値 */

#define TAIL_ANGLE_STAND_UP 107 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
/* Bluetooth palam */
#define DEVICE_NAME       "ET0"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */
#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */

/* 関数プロトタイプ宣言 */
static int sonar_alert(int alert_distance);
static void tail_control(signed int angle);
static int remote_start(void);
static void robot_init(void);
static void robot_monitor(void);
int myabs(int n);

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];

//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize(){
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); /* 完全停止用モータエンコーダリセット */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth通信初期化 */
}

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate(){
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
//	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
}

//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値 : なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain){
	signed char forward = 0;      /* 前後進命令 */
	signed char turn = 0;         /* 旋回命令 */
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	unsigned short state = 0;
	unsigned short buf = 0;
	unsigned short cnt1 = 0, cnt2 = 0, cnt3 = 0;
	unsigned short sw1 = 1, sw2 = 1, sw3 = 1;

	ecrobot_status_monitor("ECrobotti");
	ecrobot_set_bt_device_name(DEVICE_NAME);

	while(1){
		robot_monitor();
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1){
			robot_init();
			if(buf == 0){
				switch(state){
				case 0:
					state = 1;
					break;
				case 1:
					state = 0;
					break;
				}
			}
			buf = 1;
		}
		else{
			buf = 0;
		}

		switch(state){
		case 0:
			tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

			if (remote_start() == 1){	/* リモートスタート */
				state = 1;
				robot_init();
				break;
			}
			nxt_motor_set_speed(NXT_PORT_C, 0, 1); /* 左モータ停止 */
			nxt_motor_set_speed(NXT_PORT_B, 0, 1); /* 右モータ停止 */

			systick_wait_ms(10); /* 10msecウェイト */
			break;

		case 1:
			tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

			if (sonar_alert(50) == 1){ /* 障害物検知 */
				forward = 30;	/* 障害物を検知した場合前進 */
				turn = 0;
				sw1 = 0; sw2 = 1;
				cnt1 = cnt2 = 0;
			}
			else {
				forward = 0;	/* その場で旋回して障害物を探索 */
				if (++cnt1 == 125){	/* 約500msec毎に旋回・停止切り替え */
					sw1 = myabs(sw1-1);
					cnt1 = 0;
				}
				switch (sw1){
				case 0:
					turn = 20 * sw2;
					if (++cnt2 == 1500){	/* 約180°毎に回転方向を反転 */
						sw2 *= -1;
						cnt2 = 0;
					}
					break;
				case 1:
					turn = 0;
					break;
				default:
					turn = 0;
					break;
				}
			}

			switch(sw3){
			case 0:	/* エッジ検出 */
//				if (ecrobot_get_light_sensor(NXT_PORT_S3) <= LIGHT_WHITE){
				if (ecrobot_get_gyro_sensor(NXT_PORT_S1) > GYRO_OFFSET+100
					|| ecrobot_get_gyro_sensor(NXT_PORT_S1) < GYRO_OFFSET-100){
					sw3 = 1;
				}
				break;
			case 1:	/* 180°旋回 */
				forward = 0;
				turn = 60;
				if (++cnt3 == 420){
					sw3 = 2;
					sw1 = 0; sw2= 1;
					cnt1 = cnt2 = cnt3 = 0;
				}
				break;
			case 2: /* エッジ脱出 */
				forward = 20;
				turn = 0;
				if (++cnt3 == 420){
					sw3 = 0;
					cnt3 = 0;
				}
				break;
			}
			

			/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
			balance_control(
				(float)forward,								 /* 前後進命令(+:前進, -:後進) */
				(float)turn,									 /* 旋回命令(+:右旋回, -:左旋回) */
				(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* ジャイロセンサ値 */
				(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
				(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
				(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
				(float)ecrobot_get_battery_voltage(),			 /* バッテリ電圧[mV] */
				&pwm_L,										 /* 左モータPWM出力値 */
				&pwm_R);									 /* 右モータPWM出力値 */
			nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */

			/* fail safe */
			if (ecrobot_get_gyro_sensor(NXT_PORT_S1) > GYRO_OFFSET+200
				|| ecrobot_get_gyro_sensor(NXT_PORT_S1) < GYRO_OFFSET-200){
				state = 0;
			}

			systick_wait_ms(4); /* 4msecウェイト */
			break;
		}
	}
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : alert_distance  (超音波センサによる障害物検知距離[cm])
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(int alert_distance){
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4){ /* 約40msec周期毎に障害物検知  */
		/*
		 * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
		 * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
		 */
		distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
		if ((distance <= alert_distance) && (distance >= 0)){
			alert = 1; /* 障害物を検知 */
		}
		else {
			alert = 0; /* 障害物無し */
		}
		counter = 0;
	}

	return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(signed int angle){
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* 比例制御 */
	/* PWM出力飽和処理 */
	if (pwm > PWM_ABS_MAX){
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX){
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}

//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
static int remote_start(void){
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++){
		rx_buf[i] = 0; /* 受信バッファをクリア */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0){
		/* 受信データあり */
		if (rx_buf[0] == CMD_START){
			start = 1; /* 走行開始 */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* 受信データをエコーバック */
	}

	return start;
}

//*****************************************************************************
// 関数名 : robot_init
// 引数 : 無し
// 返り値 : 無し
// 概要 : 走行体倒立振子制御初期化
//*****************************************************************************
static void robot_init(void){
	balance_init();						/* 倒立振子制御初期化 */
	nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */
}

//*****************************************************************************
// 関数名 : robot_monitor
// 引数 : 無し
// 返り値 : 無し
// 概要 : NXT内部状態データを液晶に表示．
//*****************************************************************************
static void robot_monitor(void){
	static unsigned int counter = 0;
	if (++counter == 500/4){
		ecrobot_status_monitor("ECrobotti");
		counter = 0;
	}
}

//*****************************************************************************
// 関数名 : myabs
// 引数 : n(int型数値データ)
// 返り値 : nの絶対値
// 概要 : 引数で与えられたint型変数の絶対値を返す。
//*****************************************************************************
int myabs(int n){
	if (n < 0){
		n = n * -1;
	}
	return n;
}
