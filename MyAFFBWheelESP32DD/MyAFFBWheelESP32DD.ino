/*****************************************************************************
*
*	MyAFFBWheelESP32DD.ino -- ポータブルFFBホイール(ESP32-S2用)
*
*	ビルドの方法
*	・Arduino IDEでボードタイプをESP32S2 Dev Moduleを選択
*
*	rev1.0	2023/08/20	initial revision by	Toshi
*	rev2.0	2024/11/03	ブラシレスモータによるダイレクトドライブ
*	rev3.0	2024/12/28	ボードをArduino Pro MicroからESP32-S2に変更
*
*****************************************************************************/
#define MOTORTEST 0	// モーターの接続テスト(アクセルペダルで右回転)
#define BUGPRINT 0	// デバッグ用変数の表示
#define EPRINT 0	// ESP-NOW送受信状態の表示
#define RPRINT 0	// PC側から受信したレポートを表示
#define VPRINT 0	// ステア回転速度と回転加速度の表示
#define FPRINT 0	// フォースの表示

#include <string.h>
#include <EEPROM.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <USB.h>
#include <USBHID.h>
#include <driver/uart.h>	// シリアルピン設定のため必要
#include "PwmTable.h"		// PWM出力テーブル
#include "hidDescriptor.h"	// HIDディスクリプタ

/********* 定数とマクロ *********/
#define PIN_M1 12	// モーターM1
#define PIN_M2 13	// モーターM2
#define PIN_M3 14	// モーターM3
#define PIN_SDA 37	// I2C SDA
#define PIN_SCL 39	// I2C SCL
#define PIN_SW1 0	// タクトSW1(ピンが出ていないので基板から直に引き出す)
#define PIN_SW2 18	// タクトSW2
#define PIN_SW3 33	// タクトSW3
#define PIN_SW4 35	// タクトSW4
#define PIN_TX 9	// デバッグ用シリアル出力
#define PIN_DOUT 11	// 動作確認用デジタル出力
#define DOUT1 digitalWrite(PIN_DOUT, 1);
#define DOUT0 digitalWrite(PIN_DOUT, 0);

#define APPID 0x88	// ESP-NOW(MyProtocol):アプリ固有値(通信が確立したら+1)
#define ACK 0x11	// ESP-NOW(MyProtocol):親機の正常応答

// グローバル変数
USBHID HID;				// USB-HIDデバイス
int16_t InitialRawAngle;// 初期角度
int16_t Ad0;			// アクセルペダルA/D値
int16_t Ad1;			// ブレーキペダルA/D値
int16_t AccelMin;		// アクセルペダルA/D値min
int16_t AccelMax;		// アクセルペダルA/D値max
int16_t BrakeMin;		// ブレーキペダルA/D値min
int16_t BrakeMax;		// ブレーキペダルA/D値max
bool fReqAck0;			// 子機へACK送信リクエスト
bool fReqAck1;

// ESP-NOW用MACアドレス
uint8_t MacAddr0[6];	// UNIT0 MAC
uint8_t MacAddr1[6];	// UNIT1 MAC
int32_t bug,bug1,bug2,bug3,bug4,bug5;

// EEPROM構造体
struct EEPstruct
{
	int16_t InitialRawAngle;
	int16_t AccelMin;
	int16_t AccelMax;
	int16_t BrakeMin;
	int16_t BrakeMax;
} EEPobj;

// 送信するハンコンデータを格納する構造体
struct wheelData
{
	int16_t axis[8]={-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768};
	uint32_t buttons;
} WheelData;
#define AXIS_WHEEL 0
#define AXIS_ACC 1
#define AXIS_BRAKE 2
#define AXIS_CLUTCH 3
#define AXIS_AUX1 4
#define AXIS_AUX2 5
#define AXIS_AUX3 6
#define AXIS_AUX4 7

// プロトタイプ宣言
int32_t SpringForce(int16_t force, int16_t angle);
int32_t DamperForce(int16_t velocity);
int32_t InertiaForce(int16_t accel);
int32_t EndstopForce(int16_t angle);
void PwmForce(int16_t force, int16_t angle, bool finit);
int16_t Round585(int16_t data);
int16_t GetRawAngle();
int16_t GetSteer(int16_t rawangle, int16_t* iniangle, bool finit,
					int16_t* angle, int16_t* wheelv, int16_t* wheela);
long map2(long x, long in_min, long in_max, long out_min, long out_max);
void Filter(float* filt, float dat, float fc, uint32_t dt);
void AddOnTime(bool flag, int16_t* ontime);

/*----------------------------------------------------------------------------
	Wheel用カスタムHIDデバイスクラス(サンプルのCustomHIDDevice.inoを参考に)
----------------------------------------------------------------------------*/
class CustomHIDDevice : public USBHIDDevice
{
private:
	int16_t force;	// 受け取ったフォース
	bool fenable;	// フォースイネイブルフラグ
public:
	// コンストラクタ
	CustomHIDDevice(void)
	{
		static bool initialized = false;
		if (!initialized)
		{
			initialized = true;
			HID.addDevice(this, sizeof(wheelHIDDescriptor)); // デバイス登録
		}
	}
	// 開始処理
	void begin(void)
	{
		HID.begin();
	}
	// 受け取り済みのフォースを返す
	int16_t getforce()
	{
		return (int16_t)constrain(force, -16383, 16383);
	}
	// PCにコントローラー側のデータを送信
	bool send()
	{
    	return HID.SendReport(0x01, (uint8_t*)&WheelData, sizeof(WheelData));
	}
	// PCからフォースを受け取るためのコールバック
	void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len)
	{
		switch (report_id)
		{
			case 4:		// PeriodicForce
				// Periodicフォースのオフセット(変化の中心値)
				force = fenable ? (buffer[3] | buffer[4] << 8) : 0;
				break;
			case 5:		// ConstantForce
				// Constantフォース
				force = fenable ? (buffer[1] | buffer[2] << 8) : 0;
				break;
			case 10:	// EffectOperation
				fenable = (buffer[1] == 1);	// 1=Start
				if (!fenable)	// 開始状態でないなら
				{
					force = 0;	// フォースは0に
				}
				break;
			case 12:	// Device Control
				force = 0;	// フォースを0に初期化
				break;
			default:
				break;
		}
		#if RPRINT
		rp("_onOutput", report_id, buffer, len);
		#endif //RPRINT
	}
	// ↓このコールバックがないとPCにデバイスが出現しない
	uint16_t _onGetDescriptor(uint8_t *buffer)
	{
		memcpy(buffer, wheelHIDDescriptor, sizeof(wheelHIDDescriptor));
		return sizeof(wheelHIDDescriptor);
	}
	// ↓↓これらのコールバックがないと正常作動しない
	void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len)
	{
		#if RPRINT
		rp("_onSetFeature", report_id, buffer, len);
		#endif //RPRINT
	}
	uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len)
	{
		#if RPRINT
		rp("_onGetFeature", report_id, buffer, len);
		#endif //RPRINT
		return len;
	}
	#if RPRINT
	// デバッグ用受信レポート表示
	void rp(char* s, uint8_t report_id, const uint8_t *buffer, uint16_t len)
	{
		Serial1.print(s);
		Serial1.printf(" ID[%d] ", report_id);
		for (int16_t i = 0; i < len - 1;i++)
		{
			Serial1.printf("%02X,", buffer[i]);
		}
		Serial1.printf("%02X\n", buffer[len - 1]);
	}
	#endif //RPRINT
} Wheel;	// Wheelオブジェクト

/*----------------------------------------------------------------------------
	セットアップ
----------------------------------------------------------------------------*/
void setup()
{
	// デバッグ用にシリアル出力ピンを設定
	uart_set_pin(UART_NUM_1, PIN_TX, UART_PIN_NO_CHANGE,
									 UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
	Serial1.begin(115200);

    EEPROM.begin(sizeof(EEPstruct));	// ESPはサイズ初期化が必要
	EepRead();							// ステアセンターデータを読み込む
	if (AccelMax - AccelMin < 100 || BrakeMax - BrakeMin < 100)	// データNG？
	{
		// デフォルト値を書き込む
		AccelMin = BrakeMin = 40;
		AccelMax = BrakeMax = 1000;
		EepWrite();
	}

	// ピンモードの設定
	pinMode(PIN_DOUT, OUTPUT);
	pinMode(PIN_M1, OUTPUT);
	pinMode(PIN_M2, OUTPUT);
	pinMode(PIN_M3, OUTPUT);
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_SW4, INPUT_PULLUP);
	ledcAttach(PIN_M1, 39062, 10);	// 0～1024→11bitだと出力が0のまま
	ledcAttach(PIN_M2, 39062, 10);
	ledcAttach(PIN_M3, 39062, 10);

	Wire.begin(PIN_SDA, PIN_SCL);	// エンコーダーAS5600読み取り用
	Wire.setClock(400000L);			// I2C Fast-mode(400kHz)に

	WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);	// 送信パワーミニマム
	WiFi.mode(WIFI_STA);	// ステーションモード
	WiFi.disconnect();		// いったん切断

	// ESP-NOWイニシャライズ
	if (esp_now_init() == 0)
	{
		Serial1.println("ESPNow Init Success");
	}
	else
	{
		Serial1.println("ESPNow Init Failed");
		ESP.restart();	// 再起動
	}
	// ESP-NOWコールバック登録
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);

	Wheel.begin();	// ホイールHID開始
	USB.begin();	// USB開始

	// ESP32-S2の場合、通常のloop()処理ではloopの外側で
	// 2秒ごとに5msのディレイが入ってしまうので自前で回す
	while (1)
	{
		yield();
		loop();
	}
}
/*----------------------------------------------------------------------------
	メインループ
----------------------------------------------------------------------------*/
#define STEER_MAX 5120	// ステア最大左右切れ角±450deg x 4096 / 360
#define PDELTA 5		// 1ms当たりのペダル戻し量
#define DEBOUNCETIME 10	// チャタリング除去待ち時間[ms]
void loop()
{
	int16_t rawangle;			// AS5600の角度生データ0～4095
	static int16_t angle;		// 初期値を0とした角度データ0～4095
	int16_t contangle;			// 初期値を0とした連続した角度データ±0～
	static int16_t wheelv;		// ステア回転速度
	static int16_t wheela;		// ステア回転加速度
	bool fsw1, fsw2, fsw3, fsw4;// スイッチ入力
	static bool fsetting;		// イニシャルセッティング開始条件
	static bool fsettingz;
	static bool freqsetting;	// イニシャルセッティング要求中
	uint32_t switchs;			// スイッチ入力を束ねた物
	static uint32_t switchsz;
	int16_t accel, brake;		// アクセル＆ブレーキ量-32768～32767
	static int16_t clutch;		// クラッチアナログ量
	static int16_t sidebrake;	// サイドブレーキアナログ量
	int32_t rxforce, force, force1, force2, force3, endforce;// 各フォース
	int16_t targetforce;		// 最終的な目標フォース
	static uint32_t tz0, tz1;
	static int16_t amin, amax, bmin, bmax;

DOUT0
	CheckSendAck();	// ESP-NOW ACK要求チェックと送信

	// アクセル、ブレーキ量を-32768～32767に変換する
	accel = map2(Ad0, AccelMin, AccelMax, -32768, 32767);
	brake = map2(Ad1, BrakeMin, BrakeMax, -32768, 32767);

	// SWを読む
	fsw1 = !digitalRead(PIN_SW1);
	fsw2 = !digitalRead(PIN_SW2);
	fsw3 = !digitalRead(PIN_SW3);
	fsw4 = !digitalRead(PIN_SW4);
	switchs = fsw4 << 3 | fsw3 << 2 | fsw2 << 1 | fsw1;	// SW情報を束ねる
	fsetting = fsw2 && fsw3 && fsw4;	// イニシャルセッティング要求開始条件

	// AS5600の角度を取得
	rawangle = GetRawAngle();	// この処理に250usほどかかる

	// ステア角度に変換し、ステア回転速度と加速度を計算
	contangle = GetSteer(rawangle, &InitialRawAngle, freqsetting,
						&angle, &wheelv, &wheela);

	// ゲーム側に渡すデータを用意
	WheelData.axis[AXIS_WHEEL] = map2(contangle,
									-STEER_MAX, STEER_MAX, -32767, 32767);
	WheelData.axis[AXIS_ACC]  = accel;
	WheelData.axis[AXIS_BRAKE] = brake;
	// クラッチとサイドブレーキはアナログ軸にも同時に出力する
	// ゲーム側でスイッチとアナログどちらでも選択可能にするため
	WheelData.axis[AXIS_CLUTCH] = map2(clutch, 0, 1023, -32768, 32767);
	WheelData.axis[AXIS_AUX1] = map2(sidebrake, 0, 1023, -32768, 32767);

	// SWのチャタリング除去(デバウンス処理)
	if (switchsz != switchs)	// SWに変化があったら
	{
		tz1 = millis();			// 変化した時刻
		switchsz = switchs;		// 変化した時のSWの状態を保持
	}
	else if (millis() - tz1 >= DEBOUNCETIME)	// 変化後に時間が経過？
	{
		tz1 = millis() - (DEBOUNCETIME + 1);	// 次回からは毎回

		WheelData.buttons = switchs;	// ホイールに伝えるSW情報をセット

		// SW1をクラッチに割り当てる前提
		if (fsw1)	// クラッチSWオン？
		{
			clutch = 1023;	// 目いっぱい踏んでいることにする
		}
		else if (clutch)
		{
			clutch -= PDELTA;	// クラッチを徐々に離す
			if (clutch < 0)
			{
				clutch = 0;
			}
		}
		// サイドブレーキSWオン？
		sidebrake = fsw4 ? 1023 : 0;

		// イニシャルセッティング要求開始した瞬間？
		if (!fsettingz && fsetting)
		{
			Serial1.println("イニシャルセッティング開始");
			freqsetting = true;	// イニシャルセッティング要求開始
			amin = bmin = 10000;
			amax = bmax = 0;
		}
		// イニシャルセッティング要求終了した瞬間？
		else if (fsettingz && !fsetting)
		{
			if (amax > 200)		// イニシャル期間中にアクセルが踏まれた？
			{
				AccelMin = amin;
				AccelMax = amax;
				Serial1.printf("アクセルmin:%d max:%d\n", amin, amax);
			}
			if (bmax > 200)		// イニシャル期間中にブレーキが踏まれた？
			{
				BrakeMin = bmin;
				BrakeMax = bmax;
				Serial1.printf("ブレーキmin:%d max:%d\n", bmin, bmax);
			}
			EepWrite();				// EEPROMにイニシャル値を書き込む
			freqsetting = false;	// イニシャルセッティング要求解除
			Serial1.println("イニシャルセッティング終了");
		}
		fsettingz = fsetting;
	}

	// イニシャルセッティング期間なら
	if (freqsetting)
	{
		// A/D値の最小最大を更新
		amin = min(Ad0, amin);
		bmin = min(Ad1, bmin);
		amax = max(Ad0, amax);
		bmax = max(Ad1, bmax);
	}

	// PCから受け取ったフォースデータを取得
	rxforce = Wheel.getforce();

	// 受信フォースが0の時ゆるやかにセンター付近に戻すためのフォース
	force1 = SpringForce(rxforce, contangle);	// スプリングフォース

	// 据え置き型のハンコンをエミュレートするためのフォース
	force2 = DamperForce(wheelv);	// ダンパーフォース
	force3 = InertiaForce(wheela);	// イナーシャフォース

	force = rxforce + force1 + force2 + force3;	// 通常のフォース

	// ステア左右端でのフォース
	endforce = EndstopForce(contangle);	// エンドストップフォース
 	force += endforce;

	#if MOTORTEST
	// アクセル量に応じてステアを右回転
	targetforce = -map2(accel, -32768, 32767, 0, 16383);
	#else
	// 最終的な指示フォース(-16383～16383)
	targetforce = (int16_t)constrain(force, -16383, 16383);
	#endif //MOTORTEST

	// フォースをPWM出力
	PwmForce(targetforce, angle, freqsetting);

	// 定時処理
	if (millis() - tz0 >= 10)	// 10[ms]
	{
		tz0 = millis();

		#if BUGPRINT
		Serial1.printf("%d,%d,%d,%d,%d,%d\n",bug,bug1,bug2,bug3,bug4,bug5);
		#endif //BUGPRINT
		#if VPRINT
		Serial1.printf("%d,%d\n",wheelv, wheela);
		#endif //VPRINT
		#if FPRINT
		Serial1.printf("%d,%d,%d,%d,%d\n",
						rxforce, force1, force2, force3, targetforce);
		#endif //FPRINT
	}
DOUT1
	// ハンコンデータをPCに送る
	// この処理は次の送信タイミング(1ms)まで待たされるのでループの最後に置く
	Wheel.send();
}
/*----------------------------------------------------------------------------
	受信したフォースが0の時のステア回転角度に比例したスプリングフォース
	書式 ret = SpringForce(int16_t force, int16_t angle)

	int32_t	ret;	フォース
	int16_t force;	受信したフォース
	int16_t angle;	連続した舵角 ±0～
----------------------------------------------------------------------------*/
#define SPRINGC 10			// スプリングゲイン
#define SPRINGMAX 4000		// ゆっくり回る程度のフォース
#define SPRINGTIME 1000		// スプリングフォースが発生するまでの時間[ms]
int32_t SpringForce(int16_t force, int16_t angle)
{
	static int16_t tim0;

	AddOnTime(force == 0, &tim0);	// 受信フォースなし時間

	if (tim0 < SPRINGTIME)	// 受信フォース0が連続していないなら
	{
		return 0;	// スプリングフォースなし
	}

	return constrain((int32_t)angle * SPRINGC, -SPRINGMAX, SPRINGMAX);
}
/*----------------------------------------------------------------------------
	ステア回転速度に比例したダンパーフォース
	書式 ret = DamperForce(int16_t velocity)

	int32_t	ret;		フォース
	int16_t velocity;	ステア回転速度[deg/s]
----------------------------------------------------------------------------*/
#define DAMPC 10		// ダンパーゲイン
#define DAMPDEAD 5		// ステア回転速度不感帯
#define EFFECTMAX 8192	// 最大補正フォース
int32_t DamperForce(int16_t velocity)
{
	// 不感帯処理
	if (velocity >= DAMPDEAD)
	{
		velocity -= DAMPDEAD;
	}
	else if (velocity <= -DAMPDEAD)
	{
		velocity += DAMPDEAD;
	}
	else
	{
		velocity = 0;
	}

	return constrain((int32_t)velocity * DAMPC, -EFFECTMAX, EFFECTMAX);
}
/*----------------------------------------------------------------------------
	ステア回転加速度に比例したイナーシャフォース
	書式 ret = InertiaForce(int16_t accel);

	int32_t	ret;		フォース
	int16_t accel;		ステア回転加速度[deg/s^2]/10
----------------------------------------------------------------------------*/
#define INERTIAC 15		// イナーシャゲイン
#define INERTIADEAD 50	// ステア加速度不感帯
int32_t InertiaForce(int16_t accel)
{
	// 不感帯処理
	if (accel >= INERTIADEAD)
	{
		accel -= INERTIADEAD;
	}
	else if (accel <= -INERTIADEAD)
	{
		accel += INERTIADEAD;
	}
	else
	{
		accel = 0;
	}
	return constrain((int32_t)accel * INERTIAC, -EFFECTMAX, EFFECTMAX);
}
/*----------------------------------------------------------------------------
	ステア左右端でのフォース
	書式 ret = EndstopForce(int16_t angle)

	int32_t	ret;	フォース ±32767(逆方向のフォースを打ち消せるよう2倍)
	int16_t angle;	連続した舵角 ±0～
----------------------------------------------------------------------------*/
#define ENDSTOP_WIDTH 128	// 少し手前から立ち上げる
int32_t EndstopForce(int16_t angle)
{
	int16_t absangle;
	int32_t force;

	absangle = abs(angle);
	force = map2(absangle, STEER_MAX - ENDSTOP_WIDTH, STEER_MAX, 0, 32767);
	if (angle < 0)
	{
		force = -force;
	}
	return force;
}
/*----------------------------------------------------------------------------
	ForceをPWM出力	Forceが正だとステアが左に回転
	書式 void PwmForce(int16_t force, int16_t angle, bool finit)

	int16_t force;	指示フォース ±16383
	int16_t angle;	舵角 0～4095
	bool finit;		イニシャライズ(センタリング)要求
----------------------------------------------------------------------------*/
#define PWM_MAX  1024	// 最大PWM指示値
void PwmForce(int16_t force, int16_t angle, bool finit)
{
	int16_t index1, index2, index3;
	int16_t tbl1, tbl2, tbl3;
	int16_t out1, out2, out3;
	int16_t absforce, phase;
	int32_t pwm1, pwm2, pwm3;

	absforce = abs(force);

	if (force > 0)
	{
		phase = -147;	// -90deg
	}
	else if (force < 0)
	{
		phase = 147;	// 90deg
	}
	else
	{
		phase = 0;
	}

	// 1相目の配列インデックス
	index1 = Round585(angle % 585 + phase);

	if (finit)	// センタリング中なら
	{
		index1 = 0;			// 0degで
		absforce = 16383;	// 最大フォース
	}
	index2 = Round585(index1 + 195);	// 1相目+120deg
	index3 = Round585(index1 + 390);	// 1相目+240deg

	// 各位相でのPWM最大値を正弦波ベースのテーブルから取得
	tbl1 = PwmTable585[index1];	// 0～32767
	tbl2 = PwmTable585[index2];
	tbl3 = PwmTable585[index3];

	// フォースに応じたPWM値
	pwm1 = (int32_t)tbl1 * absforce / 16383;	// 0～32767
	pwm2 = (int32_t)tbl2 * absforce / 16383;
	pwm3 = (int32_t)tbl3 * absforce / 16383;

	// PWMで指示できる最大値までに抑える
	out1 = (int16_t)(pwm1 * PWM_MAX / 32767); // 0～PWM_MAX
	out2 = (int16_t)(pwm2 * PWM_MAX / 32767);
	out3 = (int16_t)(pwm3 * PWM_MAX / 32767);

	// 念のためリミット
	out1 = constrain(out1, 0, PWM_MAX);
	out2 = constrain(out2, 0, PWM_MAX);
	out3 = constrain(out3, 0, PWM_MAX);

	// PWM出力
	ledcWrite(PIN_M1, out1);
	ledcWrite(PIN_M2, out2);
	ledcWrite(PIN_M3, out3);
}
// 0～584に丸める
int16_t Round585(int16_t data)
{
	if (data >= 585)
	{
		data -= 585;
	}
	else if (data < 0)
	{
		data += 585;
	}
	return data;
}
/*----------------------------------------------------------------------------
	AS5600の角度を取得
	書式 ret = GetRawAngle();

	int16_t ret;	生の角度 0～4095
----------------------------------------------------------------------------*/
int16_t GetRawAngle()
{
	uint8_t msb4, lsb8;
	static int16_t angle;

	Wire.beginTransmission(0x36);	// AS5600アドレス
	Wire.write(0x0C);				// レジスタ指定
	Wire.endTransmission(false);	// 送信終了

	if (Wire.requestFrom(0x36, 2) == 2)	// 2バイト受信リクエスト
	{
		msb4 = Wire.read();					// 上位4bit
		lsb8 = Wire.read();					// 下位8bit
		angle = (0x0F & msb4) << 8 | lsb8;	// 全12bit
	}
	return angle;
}
/*----------------------------------------------------------------------------
	ステア角度に変換
	書式 ret = GetSteer(int16_t rawangle, int16_t* iniangle, bool finit,
					int16_t* angle, int16_t* wheelv, int16_t* wheela);

	int16_t ret;		初期値を0とした連続角度データ
	int16_t rawangle;	生角度
	int16_t* iniangle;	イニシャル生角度
	bool finit;			イニシャライズ(センタリング)要求
	int16_t* angle;		初期値を0として0 ～ 4095に正規化した角度データ
	int16_t* wheelv;	ステア回転速度
	int16_t* wheela;	ステア回転加速度
----------------------------------------------------------------------------*/
int16_t GetSteer(int16_t rawangle, int16_t* iniangle, bool finit,
					int16_t* angle, int16_t* wheelv, int16_t* wheela)
{
	int16_t angle0, dv;
	static int16_t angle0z;
	static int16_t rotate;
	static int16_t cangle, canglez;
	static int32_t vz;
	uint32_t micro;
	int32_t dt, velocity, accel;
	static uint32_t timez;
	static float filtv, filtg;
	static bool f2nd;

	micro = micros();
	dt = micro - timez;		// 前回からの経過時間[μs]
	timez = micro;

	if (finit)	// センタリング中なら
	{
		*iniangle = rawangle;		// イニシャル角度を設定
		rotate = 0;					// 回転数累積をクリア
	}
	angle0 = rawangle - *iniangle;	// イニシャル角度を差し引く
	if (angle0 < 0)
	{
		angle0 += 4096;	// 初期値を0として0 ～ 4095に正規化
	}

	// 右に1周した？
	if (angle0z > 3072 && angle0 >= 0 && angle0 < 1024)
	{
		rotate++;	// 回転数++
	}
	// 左に1周した？
	else if (angle0z >= 0 && angle0z < 1024 && angle0 > 3072)
	{
		rotate--;	// 回転数--
	}

	// 初期状態を0とした連続した角度データ
	cangle = rotate * 4096 + angle0;

	// 0割防止
	if (dt > 0)
	{
		// 回転速度と回転加速度を求める
		dv = cangle - canglez;	// 角度変化分
		if (!f2nd)
		{
			f2nd = true;
			dv = 0;
		}
		if (abs(dv) < 16383)	// 1周していないならアップデート
		{
			// 回転速度[deg/s]
			velocity = (int32_t)dv * (360L * 1000000L / 4096L) / dt;
			Filter(&filtv, (float)velocity, 10.0, dt);
			// 回転加速度[deg/s^2]/10
			accel = (velocity - vz) * 100000L / dt;
			Filter(&filtg, (float)accel, 1.0, dt);
		}
		canglez = cangle;
		vz = velocity;
	}
	angle0z = angle0;	// 現在の角度をメモリ
	*angle = angle0;	// 初期値を0として0 ～ 4095に正規化した角度データ
	*wheelv = (int16_t)filtv;
	*wheela = (int16_t)filtg;
	return cangle;		// 初期値を0とした連続角度データ
}
/*----------------------------------------------------------------------------
	ESP-NOW ACK要求チェックと送信
----------------------------------------------------------------------------*/
void CheckSendAck()
{
	static uint8_t send = ACK;

	if (fReqAck0)	// CH0からACK送信要求？
	{
		esp_now_send(MacAddr0, &send, 1);	// ACK送信
		fReqAck0 = false;
	}
	if (fReqAck1)	// CH1からACK送信要求？
	{
		esp_now_send(MacAddr1, &send, 1);	// ACK送信
		fReqAck1 = false;
	}
}
/*----------------------------------------------------------------------------
	ESP-NOW 送信コールバック
	書式 void OnDataSent(const uint8_t *mac, esp_now_send_status_t status);

	uint8_t* mac;					MACアドレス(6バイト)
	esp_now_send_status_t status;	送信ステータス
----------------------------------------------------------------------------*/
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
	char macStr[18];

	snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	#if EPRINT
	Serial1.print("Sent to: ");
	Serial1.println(macStr);
	#endif //EPRINT
}
/*----------------------------------------------------------------------------
	ESP-NOW受信コールバック
	書式 void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data,
						int data_len)

	esp_now_recv_info_t *info;	ESPNOWパケット情報
	uint8_t* data;				データ本体
	int data_len;				データ長
----------------------------------------------------------------------------*/
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data,
						int data_len)
{
	char macStr[18];
	uint8_t mac[6];

	memcpy(mac, info->src_addr, 6);	// 子機MACをコピー
	#if EPRINT
	snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial1.print("Recv from: ");
	Serial1.println(macStr);
	#endif //EPRINT

	// 子機UNITからの受信？
	if (data_len == 4 && (data[0] == APPID || data[0] == APPID + 1))
	{
		if (data[1] == 0)		// アクセルペダル？
		{
			Ad0 = (data[3] << 8) | data[2];
			if (data[0] == APPID)	// 接続確立前か？
			{
				memcpy(MacAddr0, mac, 6);	// 子機MACをコピー
				// ESP32はピアリストへの登録が必要
				if (!esp_now_is_peer_exist(MacAddr0))	// 未登録なら
				{
					esp_now_peer_info_t info;
					memset(&info, 0, sizeof(info));
					memcpy(info.peer_addr, MacAddr0, 6);
					esp_now_add_peer(&info);	// リストに登録
				}
				fReqAck0 = true;	// ACK送信要求
			}
		}
		else if (data[1] == 1)	// ブレーキペダル？
		{
			Ad1 = (data[3] << 8) | data[2];
			if (data[0] == APPID)	// 接続確立前か？
			{
				memcpy(MacAddr1, mac, 6);	// 子機MACをコピー
				// ESP32はピアリストへの登録が必要
				if (!esp_now_is_peer_exist(MacAddr1))	// 未登録なら
				{
					esp_now_peer_info_t info;
					memset(&info, 0, sizeof(info));
					memcpy(info.peer_addr, MacAddr1, 6);
					esp_now_add_peer(&info);	// リストに登録
				}
				fReqAck1 = true;	// ACK送信要求
			}
		}
	}
}
/*----------------------------------------------------------------------------
	EEP書き込み処理
----------------------------------------------------------------------------*/
void EepWrite()
{

	EEPobj.InitialRawAngle = InitialRawAngle;
	EEPobj.AccelMin = AccelMin;
	EEPobj.AccelMax = AccelMax;
	EEPobj.BrakeMin = BrakeMin;
	EEPobj.BrakeMax = BrakeMax;

	EEPROM.put<EEPstruct>(0, EEPobj);
	EEPROM.commit();	// ESPの場合必要
}
/*----------------------------------------------------------------------------
	EEP読出し処理
----------------------------------------------------------------------------*/
void EepRead()
{
	EEPROM.get<EEPstruct>(0, EEPobj);

	InitialRawAngle = EEPobj.InitialRawAngle;
	AccelMin = EEPobj.AccelMin;
	AccelMax = EEPobj.AccelMax;
	BrakeMin = EEPobj.BrakeMin;
	BrakeMax = EEPobj.BrakeMax;
}
/*----------------------------------------------------------------------------
	最大最少リミット付きmap関数 out_min <= out_maxであること
----------------------------------------------------------------------------*/
int32_t map2(int32_t x, int32_t in0, int32_t in1,
									int32_t out_min, int32_t out_max)
{
	int32_t ret;

	ret = map(x, in0, in1, out_min, out_max);
	ret = constrain(ret, out_min, out_max);
	return ret;
}
/*----------------------------------------------------------------------------
	フィルタ
	書式 void Filter(float* filt, float dat, float fc, uint32_t dt);

	float filt;		入力(1サンプル前)→出力
	float dat;		入力(今回)
	float fc;		フィルタ定数(0～)
	uint32_t dt;	インターバル[us]
----------------------------------------------------------------------------*/
void Filter(float* filt, float dat, float fc, uint32_t dt)
{
	float fact;

	fact = fc * (float)dt / 1e5 / 2.0;
	if (fact > 1.0) fact = 1.0;
	*filt = (1.0 - fact) * *filt + fact * dat;
}
/*----------------------------------------------------------------------------
	フラグのオン時間の累積
	書式 void AddOnTime(bool flag, int16_t* ontime)

	bool flag;			フラグ
	int16_t* ontime;	オン時間
----------------------------------------------------------------------------*/
#define	TIMEMAX 30000
void AddOnTime(bool flag, int16_t* ontime)
{
	if (flag)							// オンしてるなら
	{
		if (*ontime < TIMEMAX)
		{
			(*ontime)++;				// オン時間++
		}
	}
	else
	{
		*ontime = 0;
	}
}
/*** end of "MyAFFBWheelESP32DD.ino" ***/
