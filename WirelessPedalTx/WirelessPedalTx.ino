/*****************************************************************************
*
*	WirelessPedalTx.ino -- ワイヤレスペダル送信側 for ESP-01
*
*	・ペダルのアナログ値をESP-NOWで送信
*
*	rev1.0	2024/12/19	initial revision by	Toshi
*
*****************************************************************************/
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

/********* 定数とマクロ *********/
#define CH 0		// 0:アクセルペダル 1:ブレーキペダル
#define APPID 0x88	// アプリ固有値(接続が確立したら+1)
#define ACK 0x11	// 親機からの正常応答

#define DEBUG0 0	// 自身のMAC表示
#define DEBUG1 0	// esp-now作動状況表示
#define DEBUG2 0	// A/D読み取り値表示
#define DEBUG3 0	// SLEEP関連表示
#define DEBUG4 0	// SLEEP関連パルス出力

/********* グローバル変数 *********/
bool fTxComplete;		// 送信完了フラグ
uint32_t TxErrTime;		// 送信できなかった経過時間[s]x10

// ESP-NOW用MACアドレス
uint8_t MacAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};	// ブロードキャスト
uint8_t AppId = APPID;

/********* プロトタイプ宣言 *********/
void OnWakeUp(void);
void BeginSleep(void);
void OnDataSent(uint8_t *mac, uint8_t status);
void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t data_len);

/*----------------------------------------------------------------------------
	セットアップ
----------------------------------------------------------------------------*/
void setup()
{
	pinMode(0, OUTPUT);		// GPIO0ピン流れ込み側
	digitalWrite(0, 0);

	#if DEBUG4
	pinMode(2, OUTPUT);		// GPIO2ピンから電圧を出す(デバッグ用)
	#endif // DEBUG4

	#if DEBUG0 || DEBUG1 || DEBUG2 || DEBUG3
	// ハードウェアシリアル
	Serial.begin(115200);
	#endif // DEBUG0 || DEBUG1 || DEBUG2 || DEBUG3

	WiFi.setOutputPower(0);	// 送信パワーミニマム
	WiFi.mode(WIFI_STA);	// ステーションモード
	WiFi.disconnect();		// いったん切断

	#if DEBUG0
	Serial.printf("\n\nSTA MAC: %s\n", WiFi.macAddress().c_str());
	#endif // DEBUG0

	// ESP-NOWイニシャライズ
	if (esp_now_init() == 0)
	{
		#if DEBUG1
		Serial.println(F("ESPNow Init Success"));
		#endif // DEBUG1
	}
	else
	{
		#if DEBUG1
		Serial.println(F("ESPNow Init Failed"));
		#endif // DEBUG1
		ESP.restart();	// 再起動
	}
	// ESPがステーションモードならESP-NOWの役割はコントローラーとする
	esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
	// ESP-NOWコールバック登録
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);
}
/*----------------------------------------------------------------------------
	メインループ
----------------------------------------------------------------------------*/
#define TXINTERVAL 12		// 11以下ではスリープに入らない
#define AUTOOFFTIME 1800	// 自動オフ時間[s]x10
#define TIMEOUT 10			// タイムアウト検知時間[s]x10
void loop()
{
	uint8_t send[4];
	static uint16_t ad, count;
	static uint32_t t0, t1;

	t0 = millis();				// ループ開始時刻

	#if DEBUG4
	digitalWrite(2, 0);			// 動作確認用0出力
	#endif // DEBUG4

	#if 1
	ad = analogRead(A0);		// アナログリード
	if (ad > 1023) ad = 1023;	// 1024まで返されるので
	#else
	ad++;						// 受信側デバッグ用
	#endif

	send[0] = AppId;
	send[1] = CH;
	send[2] = ad & 0xFF;
	send[3] = (ad >> 8) & 0x03;
	esp_now_send(MacAddr, send, 4);	// 送信(4バイト)

	#if DEBUG2
	Serial.println(ad);
	#endif // DEBUG2

	while (millis() - t0 < TXINTERVAL)	// 時間が来るまで
	{
		esp_yield();					// システム側の処理を行う
		if (fTxComplete)				// 送信終了した？
		{
			fTxComplete = false;		// フラグを下ろす
			if (AppId != APPID)			// 接続が確立した後は
			{
				#if DEBUG4
				digitalWrite(2, 1);		// 動作確認用1出力
				#endif // DEBUG4
				BeginSleep();			// スリープ開始
				break;					// スリープ後はwhileを抜ける
			}
		}
	}

	// 定時処理
	if (millis() - t1 >= 100)	// 0.1秒経過ごとに
	{
		t1 = millis();
		TxErrTime++;	// 送信できなかった時間を+1
		if (++count >= 10)
		{
			count = 0;	// 点滅用カウンタ
		}
	}
	// タイムアウトしていないなら
	if (TxErrTime <= TIMEOUT)
	{
		pinMode(0, OUTPUT);	// 連続点灯
	}
	else
	{
		pinMode(0, (count <= 6) ? OUTPUT : INPUT);	// 点滅
	}

	// 親機がオフの場合のオートパワーオフ
	if (TxErrTime >= AUTOOFFTIME)// 送信できなかった時間が規定を超えた？
	{
		pinMode(0, INPUT);	// GPIO0ピンフロート
		ESP.deepSleep(0);	// ディープスリープ開始
		delay(100);
	}
}
/*----------------------------------------------------------------------------
	スリープ復帰時のコールバック
----------------------------------------------------------------------------*/
void OnWakeUp() 
{
	wifi_fpm_close();						// 強制スリープ不可
	wifi_set_opmode_current(STATION_MODE);	// ステーションモードで
	wifi_station_connect();					// 再度接続

	#if DEBUG3
	Serial.print("Wake Up millis = ");
	Serial.println(millis());
	Serial.flush();
	#endif // DEBUG3
}
/*----------------------------------------------------------------------------
	スリープ開始 LIGHT_SLEEPを指示しているがMODEM_SLEEP_Tにしかならない？
----------------------------------------------------------------------------*/
void BeginSleep() 
{
	wifi_station_disconnect();
	wifi_set_opmode_current(NULL_MODE);		// wifi_set_opmodeだと100msかかる
	wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);	// スリープ種類指示
	wifi_fpm_open();						// 強制スリープを可能に
	wifi_fpm_set_wakeup_cb(OnWakeUp);		// コールバックを登録  
	wifi_fpm_do_sleep(TXINTERVAL * 1000L);	// スリープ開始
	delay(TXINTERVAL - 1);					// 待ちを入れないとスリープしない
}
/*----------------------------------------------------------------------------
	ESP-NOW 送信コールバック
----------------------------------------------------------------------------*/
void OnDataSent(uint8_t *mac, uint8_t status)
{
	#if DEBUG1
	char macstr[18];

	snprintf(macstr, sizeof(macstr), "%02X:%02X:%02X:%02X:%02X:%02X",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial.print(F("Sent to: "));
	Serial.print(macstr);
	Serial.print(F("  status:"));
	Serial.println(status);
	#endif // DEBUG1

	if (AppId != APPID && status == 0)	// 相手が受信した？
	{
		TxErrTime = 0;	// 送信できなかった時間クリア
	}

	fTxComplete = true;	// 送信完了フラグをセット
}
/*----------------------------------------------------------------------------
	ESP-NOW受信コールバック
----------------------------------------------------------------------------*/
void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t data_len)
{
	#if DEBUG1
	char macstr[18];

	snprintf(macstr, sizeof(macstr), "%02X:%02X:%02X:%02X:%02X:%02X",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial.print(F("Recv from: "));
	Serial.println(macstr);
	#endif // DEBUG1

	if (data_len == 1 && data[0] == ACK)	// ACK?
	{
		memcpy(MacAddr, mac, 6);	// 次回送信先は親機のmacアドレス
		AppId = APPID + 1;			// IDを変更しておく
		#if DEBUG1
		Serial.println("*** Get MAC ***");
		#endif // DEBUG1
	}
}
/*** end of "WirelessPedalTx.ino" ***/
