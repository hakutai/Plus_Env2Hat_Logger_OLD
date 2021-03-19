/*
    note: need add library Adafruit_BMP280 from library manage
    Github: https://github.com/adafruit/Adafruit_BMP280_Library
*/

/**
 * @file Plus_Env2Hat.ino
 * 
 * @note
 * @version
 *   2021/03/16 Version 1.00
 *   2021/03/17 Version 1.01　ログをシリアルモニタへ送信機能
 * 
*/

#define   DEBUG_MODE
#define   DEVICE_NAME         "M5StickLogger"

#include <M5StickCPlus.h>
#include <math.h>
#include "SHT3X.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


/*---------------------------------------------------------
 * 各センサー用変数
 */
SHT3X                       sht3x;              // 温湿度センサー
Adafruit_BMP280             bmp280;             // 気圧センサー

/*---------------------------------------------------------
 * 大域変数　
 *          処理速度を上げるためにローカル変数や関数呼び出しにスタックを極力使わない
 */
        // センサー情報
float       temperature   = 0.0;      // 温度
float       humidity      = 0.0;      // 湿度
float       pressure      = 0.0;      // 気圧
float       altitude      = 0.0;      // 高度
int         wbgtIndex     = 0.0;      // 暑さ指数（Wet Bulb Globe Temperature）
int         discomfortIndex = 0.0;    // 不快指数（discomfort index）

RTC_DateTypeDef           rtcDate;    // 年月日
RTC_TimeTypeDef           rtcTime;    // 日時秒

        // 動作設定用
#define WAKEUP_TIME       SLEEP_MIN(30)         // ディープ休眠時間
#define DEMOTIME          5000                  // デモモードの画面切り替え時間（5000ミリ秒）

RTC_DATA_ATTR uint8_t     scrnMode      = 0;    // LCD表示内容　(0:温度・湿度,1:気圧・高度,2:方位,3:履歴）
RTC_DATA_ATTR uint8_t     lcdDirection  = 1;    // LCDの向き　 1 or 3  
RTC_DATA_ATTR uint8_t     lcdBrightness = 9;    // LCDの明るさ 7 to 15
RTC_DATA_ATTR uint32_t    defaultPowerOffTime = 20000;      // スリープ時間
RTC_DATA_ATTR uint8_t     resumeOn      = false;  // レジューム（スリープ時の表示に戻る）
uint32_t                  demoMode      = 0;    // デモモード　0:オフ,>0:オン（切替までのミリ秒）

uint32_t    powerOffTime = defaultPowerOffTime;    // ディープスリープへ移行する時間

        // 画面用　ダブルバッファー他
int16_t       scrnWidth,scrnHeight;   // スクリーン（LCD)縦横
TFT_eSprite   lcdDblBuf = TFT_eSprite(&M5.Lcd);   // ダブルバッファー

uint32_t      update_time   = 0;                  // LCD書き換え間隔管理

/*
 * 気圧変動の確認用にスローメモリーへ保存する構造体、日付・気温・湿度・気圧を３時間ごとに保存
 */
typedef struct PressArray_ {            //  RTCメモリへ保存構造体
  int      day;                         //    日
  int      minutes;                     //    分
  int      pressure;                    //    気圧
  int      temperature;                 //    温度
  int      humidity;                    //    湿度
} PressArray;

#define MAX_PRESSARRAY   48             // 配列のサイズ
#define CLEAR_PRESSARRAY {0,0,0,0,0}    // 構造体初期化値
#ifdef DEBUG_MODE
RTC_DATA_ATTR PressArray presAry[MAX_PRESSARRAY] = { { 17, 00, 1005, 23, 36 }, //  0:00
                                                     { 17, 30, 1005, 23, 37 }, //  0:30
                                                     { 17, 00, 1006, 23, 37 }, //  1:00
                                                     { 17, 30, 1006, 23, 37 }, //  1:30 
                                                     { 17, 00, 1006, 23, 37 }, //  2:00
                                                     { 17, 30, 1006, 22, 37 }, //  2:30 
                                                     { 17, 00, 1006, 22, 37 }, //  3:00
                                                     { 17, 30, 1006, 22, 37 }, //  3:30
                                                     { 17, 00, 1007, 22, 37 }, //  4:00
                                                     { 17, 30, 1007, 22, 38 }, //  4:30 
                                                     { 17, 00, 1007, 22, 38 }, //  5:00
                                                     { 17, 30, 1008, 22, 38 }, //  5:30 
                                                     { 17, 00, 1008, 22, 38 }, //  6:00
                                                     { 17, 30, 1008, 22, 38 }, //  6:30 
                                                     { 17, 00, 1008, 22, 38 }, //  7:00
                                                     { 17, 30, 1009, 22, 38 }, //  7:30 
                                                     { 17, 00, 1009, 22, 38 }, //  8:00
                                                     { 17, 30, 1009, 22, 38 }, //  8:30
                                                     { 17, 00, 1009, 22, 39 }, //  9:00
                                                     { 17, 30, 1013, 22, 38 }, //  9:30
                                                     { 17, 00, 1010, 22, 36 }, // 10:00
                                                     { 17, 30, 1010, 24, 33 }, // 10:30
                                                     { 17, 00, 1010, 27, 22 }, // 11:00
                                                     { 17, 30, 1010, 27, 20 }, // 11:30
                                                     { 17, 10, 1009, 27, 19 }, // 12:00
                                                     { 17, 37, 1009, 28, 19 }, // 12:30
                                                     { 17, 00, 1009, 28, 18 }, // 13:00
                                                     { 17, 30, 1009, 27, 19 }, // 13:30
                                                     { 17, 00, 1009, 27, 18 }, // 14:00
                                                     { 17, 30, 1009, 27, 19 }, // 14:30
                                                     { 17, 00, 1009, 27, 19 }, // 15:00
                                                     { 17, 30, 1009, 27, 20 }, // 15:30
                                                     { 17, 00, 1010, 29, 19 }, // 16:00
                                                     { 16, 30, 1003, 28, 28 }, // 16:30
                                                     { 16, 00, 1003, 27, 31 }, // 17:00
                                                     { 16, 30, 1003, 26, 33 }, // 17:30
                                                     { 16, 00, 1004, 26, 34 }, // 18:00
                                                     { 16, 49, 1004, 26, 35 }, // 18:30
                                                     { 16, 00, 1004, 27, 35 }, // 19:00
                                                     { 16, 30, 1005, 25, 35 }, // 19:30
                                                     { 16, 00, 1006, 24, 34 }, // 20:00
                                                     { 16, 30, 1003, 23, 34 }, // 20:30
                                                     { 16, 00, 1004, 23, 35 }, // 21:00
                                                     { 16, 30, 1004, 23, 36 }, // 21:30
                                                     { 16, 00, 1004, 23, 36 }, // 22:00
                                                     { 16, 30, 1004, 23, 36 }, // 22:30
                                                     { 16, 00, 1005, 23, 36 }, // 23:00
                                                     { 16, 30, 1005, 23, 36 }, // 23:30
                                                    };
#else
RTC_DATA_ATTR PressArray presAry[MAX_PRESSARRAY] = { CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                    };
#endif

/*
 * 気圧関連
 */
#define   DEFAULT_SEALEVEL        1013.25     // 標準気圧
#define   SEALEVEL_TEMPERATURE    15          // 海面温度
RTC_DATA_ATTR float     seaLevelPressure = DEFAULT_SEALEVEL;      // 設定海面気圧
uint16_t  kAltitude = 0;                      // 標高校正用カウンタ

/*
 * 記録表示
 */
int       logListOffset = 0;

/*
 * BLE 関係
 */
#define SERVICE_UUID                   "181a"                                        // Environment Sensing UUID(0000181a-0000-1000-8000-00805f9b34fb)
#define CHARACTERISTIC_UUID            "156f7abe-87c8-11eb-8dcd-0242ac130003"        // Generate https://www.uuidgenerator.net/

BLEServer         *pBLEServer          = NULL;
BLECharacteristic *pBLECharacteristic  = NULL;
bool              bleConnected          = false;
bool              oldBleConnected       = false;
#define           BLEDEVICE_NAME         "StickLogger"
#ifdef DEBUG_MODE
RTC_DATA_ATTR uint8_t bleDeviceNumber  = 1;             // 0:BLE off / 1～7:Number   BLEDEVICE_NAME + bleDeviceNumber
#else
RTC_DATA_ATTR uint8_t bleDeviceNumber  = 0;             // 0:BLE off / 1～7:Number   BLEDEVICE_NAME + bleDeviceNumber
#endif

struct _BLEDataPacket {   // Bluetoothで送信するデータ
  uint16_t        day;            // 日付         month << 8 | day
  uint16_t        time;           // データ時刻    hh << 8 | mm
  int16_t         pressure;       // 気圧         int(pres *  10)
  int16_t         temperature;    // 温度         int(temp * 100)
  int16_t         humidity;       // 湿度         int(temp * 100)
  uint16_t        voltage;        // 電圧         int(volt * 100)
} bleDataPacket;  

/*
 * 電源監視用
 */
uint8_t     extPW = false;                    /* 電源使用 true:外部電源 / false:内部電源 */
float       pwVolt;                           // Power Voltage
float       pwCurt;                           // Power Current

/*===========================================================================
 * 汎用変数
 */
int16_t     i,j;                 // カウンター
int16_t     x,y;                 // 座標用
char        tmpStr[40];          // 汎用文字列

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * 各種　一行計算式
 * 
 */
//  熱中症指数、不快指数の算出
#define CalcWBGTIndex(temp,humi)              (temp * (0.62 + humi * 0.005))
#define CalcdiscomfortIndex(temp,humi)        (0.81 * temp + 0.01 * humi * (0.99 * temp - 14.3) + 46.3)
//  標高ー＞気圧変換
#define   AltToPres(altitude)                 (pressure*pow((1-0.0065*altitude/(SEALEVEL_TEMPERATURE+273.2)),5.254))
// 時間からPressArrayのインデックスを求
#define CalcPressArrayIndex(hours,minutes)    (hours * 2 + int(minutes / 30))

// RGB565 24bit -> 16bit 色決定後に定数に直すこと
#define ToRGB565(r,g,b) ((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3))

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * M5StickC-PlusのSetSleepはバグ有り　正常動作のSetSleep 2021/02/20
 * 
 */
void xSetSleep(void)          //version M5stick-C plus
{
  M5.Axp.Write1Byte(0x31, M5.Axp.Read8bit(0x31) | ( 1 << 3)); // short press to wake up
  M5.Axp.Write1Byte(0x90, M5.Axp.Read8bit(0x90) | 0x07); // GPIO1 "low noise" otherwise it goes to download mode when waking up
  M5.Axp.Write1Byte(0x12, M5.Axp.Read8bit(0x12) & ~(1<<1) & ~(1<<2) & ~(1<<3)); // Disable all outputs but DCDC1 (DCDC3 = NC, LDO2 = LCD-BL, LDO3 = LCD-Logic)
}
uint8_t           wakeUpCause;      // 起動理由 esp_sleep_get_wakeup_cause()の返り値


/*
 * BLE 関係　関数
 */
void BLE_Setup();               // BLEデバイス初期化

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      bleConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      bleConnected = false;
    }
};



/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 *＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * SETUP
 */
void setup() {

  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0,26,100000);
  M5.Lcd.setRotation(lcdDirection);
  M5.Lcd.fillScreen(BLACK);
  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
  scrnWidth  = M5.Lcd.width();
  scrnHeight = M5.Lcd.height();

    // LCDのダブルバッファーの用意
  lcdDblBuf.createSprite(M5.Lcd.width(),M5.Lcd.height());
  lcdDblBuf.setSwapBytes(false);
  
  pinMode(M5_BUTTON_HOME, INPUT);
  setCpuFrequencyMhz(80);             // CPUを20MHzで駆動 -> BLEを使用するため80Mhzに変更

  if (!bmp280.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }
    /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */

    /* BLE SETUP */
  BLE_Setup();  

  if (!resumeOn) scrnMode = 0;        // スクリーンの復帰
  if (demoMode != 0) demoMode = millis() + DEMOTIME;
  
  /*　電源ソースを調べる */
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;

  /* ディープスリープ設定（ボタンA押下で起床 */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0);  

  /* 起動理由の格納 */
  wakeUpCause = esp_sleep_get_wakeup_cause();

  if (wakeUpCause != ESP_SLEEP_WAKEUP_TIMER) {
    i = CalcPressArrayIndex(rtcTime.Hours,rtcTime.Minutes);

    bleDataPacket.day         = rtcDate.Month << 8 | presAry[i].day;
    bleDataPacket.time        = rtcTime.Hours << 8 | presAry[i].minutes;
    bleDataPacket.pressure    = (int)(presAry[i].pressure * 10.);
    bleDataPacket.temperature = (int)(presAry[i].temperature * 100.);
    bleDataPacket.humidity    = (int)(presAry[i].humidity * 100.);
    bleDataPacket.voltage     = (int)(pwVolt * 100.);
    pBLECharacteristic->setValue((uint8_t*)&bleDataPacket, sizeof(bleDataPacket));
    pBLECharacteristic->notify();
    delay(10);
  }
}

/*******************************************************
 *  
 */
void BMM150_Calibrate();
enum ScrnMode { TEMPHUMI, LOGLIST, PRESVOLT };
void SetupMenu();                 // 設定
void DispTempHumi();              // 温度・湿度　デカ表示
void Dispog();             // 温度・湿度　デカ表示
void DispLogList();               // 記録表示
/*******************************************************
 *  LOOP
 */

void loop() {

  M5.update();
  if (M5.BtnA.wasPressed()) {
    scrnMode = (scrnMode + 1) & 0x01;
    lcdDblBuf.fillSprite(BLACK);
    update_time = 0;
    powerOffTime = defaultPowerOffTime + millis();
  }
  if (scrnMode == TEMPHUMI) {
    if (M5.BtnB.isPressed()) {
      SetupMenu();
      powerOffTime = defaultPowerOffTime + millis();
    }    
  }
  if (demoMode > 0) {
    if (millis() > demoMode) {
      scrnMode = (scrnMode + 1) & 0x01;
      demoMode = millis() + DEMOTIME;    
    }
  }

  if (scrnMode == LOGLIST) {
    if (M5.BtnB.isPressed()) {
      logListOffset++;
      if (logListOffset > 6) logListOffset = 0;
      powerOffTime = defaultPowerOffTime + millis();
    }    
  }
  
  if (scrnMode == PRESVOLT) {
    if (M5.BtnB.isPressed()) {              // 標高の設定
      Serial.printf("Btn B press  %d\r\n",kAltitude);
      if (kAltitude <= 1) seaLevelPressure = DEFAULT_SEALEVEL;
      else                seaLevelPressure = AltToPres(-100 * (kAltitude / 2 - 1));     // 再描画間隔が短いと操作性悪い、２で割って0.5間隔で更新
      Serial.printf("Btn B press  %d  %lf\r\n",kAltitude,seaLevelPressure);
      kAltitude++;
      powerOffTime = UINT32_MAX;
    } else {
      if (kAltitude > 1) powerOffTime = defaultPowerOffTime + millis();
      kAltitude = 0;
    }
  }
  
  /*--- 電源ソースを調べる ---*/
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;
  else              extPW = false;

  if (millis() > update_time) {
    update_time = millis() + 500;          // 500ms間隔で再描画
    
    if(sht3x.get()==0){                     // 温度・湿度の取得
      temperature = sht3x.cTemp;
      humidity = sht3x.humidity;
      wbgtIndex = (int)CalcWBGTIndex(temperature,humidity);
      discomfortIndex = (int)CalcdiscomfortIndex(temperature,humidity);
    }

    pressure       = bmp280.readPressure() / 100;   //Pa -> hPa
    altitude       = bmp280.readAltitude(seaLevelPressure);

    M5.Rtc.GetData(&rtcDate);
    M5.Rtc.GetTime(&rtcTime);

    if (int(wbgtIndex) >= 31) {     // 31
      M5.Lcd.fillScreen(BLACK);
      lcdDblBuf.pushSprite(0,0); 
      M5.Beep.tone(2000);
    } else {
      M5.Beep.mute();
    }

    M5.Lcd.startWrite();
    switch (scrnMode) {
      case TEMPHUMI: 
              DispTempHumi();                // 温度・湿度表示
              break;
      case LOGLIST:
              DispLogList();
              break;      
      default:
              DispLogList();
    }

    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
    M5.Lcd.endWrite();
    

    i = CalcPressArrayIndex(rtcTime.Hours,rtcTime.Minutes);
    /*--- 気圧をスローメモリーに保存 0,3,6,9,12,15,18,21時の３時間ごとに保存 ---*/
    if (presAry[i].day != rtcDate.Date) {
      presAry[i].day          = rtcDate.Date;
      presAry[i].minutes      = rtcTime.Minutes;
      presAry[i].temperature  = (int)temperature;
      presAry[i].humidity     = (int)humidity;
      presAry[i].pressure     = (int)pressure;

      // BLE送信
      bleDataPacket.day         = rtcDate.Month << 8 | rtcDate.Date;
      bleDataPacket.time        = rtcTime.Hours << 8 | rtcTime.Minutes;
      bleDataPacket.pressure    = (int)(pressure * 10.);
      bleDataPacket.temperature = (int)(temperature * 100.);
      bleDataPacket.humidity    = (int)(humidity * 100.);
      bleDataPacket.voltage     = (int)(pwVolt * 100.);
      pBLECharacteristic->setValue((uint8_t*)&bleDataPacket, sizeof(bleDataPacket));
      pBLECharacteristic->notify();
      delay(10);
    }

  /*--- DeepSleep の設定 
          バッテリーまたはdeepsleepから起動した場合は再びdeepsleepへ
  */
    if (!extPW || (wakeUpCause == ESP_SLEEP_WAKEUP_TIMER)) {  
      if (millis() > powerOffTime) {     // 20sec
        i = int(rtcTime.Minutes / 30 + 1) * 30 - rtcTime.Minutes;
          //---ここからは AXP192::DeepSleep(uint64_t time_in_us)の必要部分の抜き出し
         xSetSleep();
         esp_sleep_enable_timer_wakeup(SLEEP_MIN(i));       // 次の計測時刻までスリープ
         esp_deep_sleep(SLEEP_MIN(i));
          //--- ここまで
      }
    }
  }

  delay(50);
}
/*=====================================================================
 * Functions
 */
/*
 * BLE Setup
 */
 void BLE_Setup() {

  sprintf(tmpStr,"%s%d",BLEDEVICE_NAME,bleDeviceNumber);
  BLEDevice::init(tmpStr);

      // Create BLE-Server
  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
  BLEService *pBLEService = pBLEServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
  pBLECharacteristic = pBLEService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
                    
  pBLECharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pBLEService->start();

  // Start advertising
  BLEAdvertising *pBLEAdvertising = BLEDevice::getAdvertising();
  pBLEAdvertising->addServiceUUID(SERVICE_UUID);
  pBLEAdvertising->setScanResponse(false);
  pBLEAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
 }
/*=====================================================================================================
 * メニュー表示
 * 　　スクリーンの向き
 * 　　スクリーンの明るさ
 * 　　スリープへの時間
 * 　　日時
 *  設定で速度関係ないからローカル変数を使用
 */
void SetupMenu() {
  uint16_t      idxMenu = 0;
  uint16_t      btnAwasPressed;   //単純に関数嫌だから呼ぶの
  uint16_t      menuExit = false;
                               // 1  2  3  4  5  6  7  8  9 10 11 12
  uint16_t      monthDay[12] = { 31,29,31,30,31,30,31,31,30,31,30,31 };
  uint32_t      autoExitTime;

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextFont(1);
  lcdDblBuf.fillSprite(BLACK);

  M5.Rtc.GetData(&rtcDate);
  M5.Rtc.GetTime(&rtcTime);

  autoExitTime = defaultPowerOffTime + millis();

  do {
    lcdDblBuf.setTextColor(TFT_WHITE, TFT_BLACK);
      //-- LCDの向き
    lcdDblBuf.setCursor(0,0);     lcdDblBuf.printf("Scrn Direction ");
    lcdDblBuf.setCursor(176,0);   
    if (lcdDirection == 1)     lcdDblBuf.printf(" LEFT");
    else                       lcdDblBuf.printf("RIGHT");
      //-- LCDの明るさ
    lcdDblBuf.setCursor(0,20);    lcdDblBuf.printf("Scrn Brightness ");
    lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",lcdBrightness);
      //-- スリープ時間
    lcdDblBuf.setCursor(0,40);    lcdDblBuf.printf("Sleep Time ");
    lcdDblBuf.setCursor(210,40);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
      //-- レジューム（スクリーンの復帰）
    lcdDblBuf.setCursor(0,60);    lcdDblBuf.printf("Resume ");
    lcdDblBuf.setCursor(200,60);  
    if (resumeOn) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
      //-- 時刻
    lcdDblBuf.setCursor(0,80);    lcdDblBuf.printf("DT ");
    lcdDblBuf.setCursor(40,80);   
      lcdDblBuf.printf("%4d/%02d/%02d %02d:%02d",
                          rtcDate.Year,rtcDate.Month,rtcDate.Date,rtcTime.Hours,rtcTime.Minutes);
      //-- ログ出力
    lcdDblBuf.setCursor(0,100); lcdDblBuf.printf("Log Output");
    lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");
      //-- デモモード
/*    lcdDblBuf.setCursor(0,100);    lcdDblBuf.printf("DemoMode");
    lcdDblBuf.setCursor(200,100);
    if (demoMode) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
*/      //-- RETURN
    lcdDblBuf.setCursor(0,120);   lcdDblBuf.printf("RETURN");

    M5.update();
    if (btnAwasPressed = M5.BtnA.wasPressed()) autoExitTime = defaultPowerOffTime + millis();
    if (M5.BtnB.wasPressed()) {
      if (++idxMenu > 10) idxMenu = 0;
      autoExitTime = defaultPowerOffTime + millis();
    }

    lcdDblBuf.setTextColor(TFT_BLACK, TFT_CYAN);
    switch (idxMenu) {
      case 0:     //スクリーンの向き
                lcdDblBuf.setCursor(176,0);
                if (btnAwasPressed) {
                  lcdDirection == 1 ? lcdDirection = 3 : lcdDirection = 1;
                  lcdDblBuf.fillSprite(BLACK);
                  M5.Lcd.setRotation(lcdDirection);
                }
                if (lcdDirection == 1)  lcdDblBuf.printf(" LEFT");
                else                    lcdDblBuf.printf("RIGHT");
                break;
      case 1:     //スクリーンの明るさ
                if (btnAwasPressed) {
                  if (++lcdBrightness > 15) lcdBrightness = 7;
                  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
                }
                lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",lcdBrightness);
                break;
      case 2:     //スリープ時間
                if (btnAwasPressed) {
                  defaultPowerOffTime += 10000;
                  if (defaultPowerOffTime > 60000) defaultPowerOffTime = 10000;
                }
                lcdDblBuf.setCursor(210,40);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
                break;
      case 3:     //レジューム
                if (btnAwasPressed) {
                  resumeOn = !resumeOn;
                }
                lcdDblBuf.setCursor(200,60);  
                if (resumeOn) lcdDblBuf.printf(" ON");
                else          lcdDblBuf.printf("OFF");
                break;
      case 4:     //年
                if (btnAwasPressed) {
                  if (++rtcDate.Year > 2030) rtcDate.Year = 2021;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40,80);   
                lcdDblBuf.printf("%4d",rtcDate.Year);
                break;
      case 5:     //月
                if (btnAwasPressed) {
                  if (++rtcDate.Month > 12) rtcDate.Month = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 5,80);   
                lcdDblBuf.printf("%02d",rtcDate.Month);
                break;
      case 6:     //日
                if (btnAwasPressed) {
                  if (++rtcDate.Date > monthDay[rtcDate.Month-1]) rtcDate.Date = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 8,80);   
                lcdDblBuf.printf("%02d",rtcDate.Date);
                break;
      case 7:     //時
                if (btnAwasPressed) {
                  if (++rtcTime.Hours > 23) rtcTime.Hours = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 11,80);   
                lcdDblBuf.printf("%02d",rtcTime.Hours);
                break;
      case 8:     //分
                if (btnAwasPressed) {
                  if (++rtcTime.Minutes > 59) rtcTime.Minutes = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 14,80);   
                lcdDblBuf.printf("%02d",rtcTime.Minutes);
                break;
#ifdef DEBUG_MODE
      case 9:     // ログ出力
                lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");
                if (btnAwasPressed) {
                  M5.Lcd.setTextSize(2);
                  M5.Lcd.setTextFont(1);
                  for (i = 0; i < MAX_PRESSARRAY; i++) {
                    Serial.printf("{ %2d, %2d, %4d, %2d, %2d },  // %2d:%02d\r\n",
                      presAry[i].day,presAry[i].minutes,presAry[i].pressure,presAry[i].temperature,presAry[i].humidity,
                      int(i/2),i%2*30);
                    
                    M5.Lcd.setCursor(160,100); M5.Lcd.printf("%6d",i);
                  }
                }
                lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");
                break;
#else
      case  9:    //デモモード
                if (btnAwasPressed) {
                  if (demoMode != 0) demoMode = 0;
                  else               demoMode = millis() + DEMOTIME;
                }
                lcdDblBuf.setCursor(200,100);  
                if (demoMode != 0) lcdDblBuf.printf(" ON");
                else               lcdDblBuf.printf("OFF");
                break;
#endif
      default:
                lcdDblBuf.setCursor(0,120);   lcdDblBuf.printf("RETURN");
                menuExit = false;
                if (btnAwasPressed) menuExit = true;
                break;
    }
    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
                
    if (millis() > autoExitTime) menuExit = true;


    delay(100);
  } while (!menuExit);
}
/*==================================================================
 * 温度・湿度　デカ表示
 */
void DispTempHumi() {
  // 背景
  if (wbgtIndex >= 31)      lcdDblBuf.fillSprite(ToRGB565(192,0,0));
  else if (wbgtIndex >= 28) lcdDblBuf.fillSprite(TFT_OLIVE);
  else                      lcdDblBuf.fillSprite(TFT_DARKCYAN);

  // 気温
  x = 5; y = 1;
  sprintf(tmpStr,"%.1f ",temperature);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+5,y+6,4);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,x,y,4);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setCursor(160,3); lcdDblBuf.printf("C");

  x = 165; y = 23;
  sprintf(tmpStr,"%2d ",wbgtIndex);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+3,y+4,4);
  if (wbgtIndex < 25)      lcdDblBuf.setTextColor(TFT_CYAN);
  else if (wbgtIndex < 28) lcdDblBuf.setTextColor(TFT_GREENYELLOW);
  else if (wbgtIndex < 31) lcdDblBuf.setTextColor(TFT_YELLOW);
  else             lcdDblBuf.setTextColor(TFT_RED);
  lcdDblBuf.drawString(tmpStr,x,y,4);
    // 暑さ指数上昇
  i = CalcPressArrayIndex(rtcTime.Hours - 1,0);       // 一時間前と比較
  if (i < 0) i += MAX_PRESSARRAY;
  if (presAry[i].temperature != 0) {
    j = CalcWBGTIndex(presAry[i].temperature,presAry[i].humidity);
    lcdDblBuf.setCursor(165,23,4);
    lcdDblBuf.setTextColor(TFT_RED);
    if (j - wbgtIndex < 0)  lcdDblBuf.printf("     ^");   // 前の時間帯より指数上昇
    else                    lcdDblBuf.printf("      ");
  }
  
  // 湿度
  x = 5; y = 67;
  sprintf(tmpStr,"%.1f ",humidity);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+5,y+6,4);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,x,y,4);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setCursor(160,67); lcdDblBuf.printf("%%");

  x = 165; y = 25;
  sprintf(tmpStr,"%2d",discomfortIndex);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+3,y+69,4);
  if (discomfortIndex < 76)      lcdDblBuf.setTextColor(TFT_CYAN);
  else if (discomfortIndex < 80) lcdDblBuf.setTextColor(TFT_GREEN);
  else if (discomfortIndex < 85) lcdDblBuf.setTextColor(TFT_GREENYELLOW);
  else if (discomfortIndex < 90) lcdDblBuf.setTextColor(TFT_YELLOW);
  else             lcdDblBuf.setTextColor(TFT_LIGHTGREY,TFT_RED);
  lcdDblBuf.drawString(tmpStr,x,y+65,4);

  // 時刻
  x = 15; y = 40;
  sprintf(tmpStr,"%2d:%02d",rtcTime.Hours,rtcTime.Minutes);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));         // 影
  lcdDblBuf.drawString(tmpStr,x+8,y+9,4);           
  lcdDblBuf.drawString(tmpStr,x+10,y+11,4);

  lcdDblBuf.setTextColor(TFT_DARKGREEN);              // 輪郭
  lcdDblBuf.drawString(tmpStr,x+2,y,4);
  lcdDblBuf.drawString(tmpStr,x,y+2,4);
 
  lcdDblBuf.setCursor(x,y,4);
  lcdDblBuf.setTextColor(TFT_GREEN);
  lcdDblBuf.drawString(tmpStr,x,y,4);
  
}

/*============================================================================
 * 記録表示
 */
void DispLogList() {
  lcdDblBuf.fillSprite(ToRGB565(128,128,64));

  lcdDblBuf.setTextSize(1);
  lcdDblBuf.setCursor(6,0,2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.printf("  d      Time   %4dhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
    // 幅広にし見やすく
    lcdDblBuf.setCursor(7,0,2);
    lcdDblBuf.printf("  d      Time   %4dhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
    // 表示位置の算出　現在の時刻を最下行にする
  j = CalcPressArrayIndex(rtcTime.Hours,rtcTime.Minutes) - logListOffset * 8;
  lcdDblBuf.setTextSize(2);
  for (i = 7; i >= 0; i--,j--) {
    if (j < 0) j += MAX_PRESSARRAY;
    if (i % 2 == 0) lcdDblBuf.setTextColor(TFT_YELLOW);
    else            lcdDblBuf.setTextColor(TFT_WHITE);
    lcdDblBuf.setCursor(3,15+ 15*i,1);
    lcdDblBuf.printf("%2d %2d:%02d%5d%3d%3d",presAry[j].day,int(j / 2),presAry[j].minutes,presAry[j].pressure,presAry[j].temperature,presAry[j].humidity);
      // 幅広にし見やすく
      lcdDblBuf.setCursor(4,15+ 15*i,1);
      lcdDblBuf.printf("%2d %2d:%02d%5d%3d%3d",presAry[j].day,int(j / 2),presAry[j].minutes,presAry[j].pressure,presAry[j].temperature,presAry[j].humidity);
  }
}
