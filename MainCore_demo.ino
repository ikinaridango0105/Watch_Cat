///////////////////////////////////////////////////////////////////////////////
//メインコア
///////////////////////////////////////////////////////////////////////////////
#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <GNSS.h>
#include <RTC.h>
#include <DHT.h> 
#include <Audio.h>
#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <Camera.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DNNRT.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <GNSS.h>
#include <GNSSPositionData.h>
#include <QZQSM.h>
#include <HttpGs2200.h>  // GainSpan HTTP clientライブラリをインクルード
#include <TelitWiFi.h>   // Telit WiFiモジュール用ライブラリをインクルード
#include <Watchdog.h>

SpGnss Gnss;
SDClass theSD;
DNNRT dnnrt;
File myFile;
File nnbfile;
AudioClass *theAudio;

/*コア間通信*/
int8_t snddata;
int8_t sndid = 100;

/*コア間共有メモリ*/
#define MSG_SIZE 500
int32_t *share_addr;

/*GPS+RTC*/
#define MY_TIMEZONE_IN_SECONDS (9 * 60 * 60) // JST
int8_t time_set = 1;
int8_t next_time;

/*災危*/
const int HISTORY_COUNT = 20;
QZQSM s_history[HISTORY_COUNT];
int s_index = 0;

/*超音波センサ*/
#define trigPin 0  //trigを9番ピンに割り当て
#define echoPin 1  //echoを8番ピンに割り当て
float Duration = 0;  // 計測した時間
float Distance = 0;  // 距離

/*温度センサDHT22*/
#define DHT_PIN 22             
#define DHT_MODEL DHT11      
DHT dht(DHT_PIN, DHT_MODEL); 

/*赤外線受信*/
#define maxLen 500
#define rxPinIR 2                        //pin D2 or D3 on standard arduinos. (other pins may be available on More mordern modules like MEga2560, DUE, ESP8266, ESP32)
volatile unsigned int irBuffer[maxLen];  //stores timings - volatile because changed by ISR
volatile unsigned int x;                 //Pointer thru irBuffer - volatile because changed by ISR

/*登録済み赤外線信号ファイル読み込み*/
static String elements[MSG_SIZE];
static int received_elements_num = 0;

/*LED*/
int LED_PIN_RG = 15;    // デジタルピン3番に赤・緑色LEDを割り当てる(変数)
int LED_PIN_W = 14;   // デジタルピン2番に白色LEDを割り当てる(変数)
int LED_PIN_B = 7;
float fRG = 0.5;

/*LCD*/
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS -1
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

/*Audio*/
bool ErrEnd = false;

/*画像認識*/
#define DNN_IMG_W 28
#define DNN_IMG_H 28
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 104
#define CAM_CLIP_Y 0
#define CAM_CLIP_W 112
#define CAM_CLIP_H 224
#define LINE_THICKNESS 5
uint8_t buf[DNN_IMG_W*DNN_IMG_H];
DNNVariable input(DNN_IMG_W*DNN_IMG_H);
static uint8_t const label[2] = {0, 1};

/*割り込み処理用トリガ*/
const byte interruptPin4 = 4;
const byte interruptPin5 = 5;
const byte interruptPin6 = 6;
//const byte interruptPin7 = 7;
volatile byte home = HIGH;
volatile byte set = LOW;
volatile byte health = LOW;
volatile byte hazard = LOW;

/*flag一覧*/
float old_temp;                               //
float old_time;
int8_t old_hour;
int8_t old_min;
volatile int8_t flag_dnnrt = false;
volatile bool flag_here = false;          //1：人を検出
volatile bool flag_alert = false;         //1：室内が危険状態
volatile bool flag_power = false;         //1：エアコンがON状態
volatile bool flag_signal = false;        //1：赤外線受信開始            
volatile bool flag_audio_stop = false;    //1：Audioの再生中

bool flag_gnss_down = true;               //1：gnssの開始

int8_t flag_random_voice = 0;             //ランダム音声再生
int8_t flag_music = 0;                    //音楽再生

volatile bool flag_light = false;

int8_t count_p = 0;            //人認識　閾値カウント
int8_t count_m = 0;            //人認識　連続時間カウント(min)
int8_t count_distance = 0;    //障害物検知

///////////////////////////////////////////////////////////////////////////////
//4番ピンの割込み処理
///////////////////////////////////////////////////////////////////////////////
void function4() {
  if (set == HIGH) {
    //赤外線受信開始
    flag_signal = true;

  } else if (health == HIGH) {
    //ホーム画面に移行
    delay(200);
    home_lcd();
    RTC.end();
    //パラメータリセット
    home = HIGH;
    health = LOW;

  } else if (hazard == HIGH) {
    //ホーム画面に移行
    delay(200);
    home_lcd();
    RTC.end();
    //パラメータリセット
    home = HIGH;
    hazard = LOW;

  }else if (home == HIGH) {
    //setup画面に移行
    delay(200);
    set_lcd();
    //パラメータリセット
    set = HIGH;
    home = LOW;
  
  }

}

///////////////////////////////////////////////////////////////////////////////
//5番ピンの割込み処理
///////////////////////////////////////////////////////////////////////////////
void function5() {
  if (set == HIGH) {
    ///////////////////
    //人認識(set View)
    ///////////////////
    if (flag_dnnrt == 0) {
      //開始
      flag_dnnrt = 1;
    } else if (flag_dnnrt == 2) {
      //終了
      flag_dnnrt = 3;
    }

  } else if (health == HIGH) {
    //ランダムおしゃべり
    flag_random_voice = 1;

  } else if (hazard == HIGH) {
    //1/fゆらぎ
    if (flag_light==false) {
      flag_light = true;
    } else if (flag_light==true) {
      flag_light = false;
    }

  } else if (home == HIGH) {
    //見守りモードに移行
    delay(200);
    Health_lcd();
    RTC.begin();
    //パラメータリセット
    old_time = 0;
    old_temp = 0;
    health = HIGH;
    home = LOW;
  }
}

///////////////////////////////////////////////////////////////////////////////
//6番ピンの割込み処理
///////////////////////////////////////////////////////////////////////////////
void function6() {
  if (set == HIGH) {
    //ホーム画面に移行
    delay(200);
    home_lcd();
    home = HIGH;
    set = LOW;
  } else if (health == HIGH) {
    //割り当てなし
    
  } else if (hazard == HIGH) {
    //音楽再生
    flag_music = 1;

  } else if (home == HIGH) {
    //寄り添いモードに移行
    delay(200);
    Hazard_lcd();
    RTC.begin();
    setup_QZSS();
    //パラメータリセット
    old_time = 0;
    old_hour = 0;
    old_min = 0;
    hazard = HIGH;
    home = LOW;
  }
}

///////////////////////////////////////////////////////////////////////////////
//GPSから時刻取得を行う関数
///////////////////////////////////////////////////////////////////////////////
void get_time() {
  if (Gnss.waitUpdate()) {
    Serial.println("gps try...");
    SpNavData  NavData;

    // Get the UTC time
    Gnss.getNavData(&NavData);
    SpGnssTime *time = &NavData.time;

    //RTCの設定
    if (time->year>2000){
      Serial.println("gps get!!");
      time_set = 0;
      RtcTime now = RTC.getTime();
      RtcTime gps(time->year, time->month, time->day,
                  time->hour, time->minute, time->sec, time->usec * 1000);
#ifdef MY_TIMEZONE_IN_SECONDS
      // Set the time difference
      gps += MY_TIMEZONE_IN_SECONDS;
#endif
      int diff = now - gps;
      if (abs(diff)>=1){
        RTC.setTime(gps);
      }
    }
  }
}

void get_time_line(int year, int month, int day, int hour, int minute, int sec, int usec) {
  //RTCの設定
  RtcTime now = RTC.getTime();
  RtcTime gps(year, month, day,
              hour, minute, sec, usec * 1000);
  int diff = now - gps;
  if (abs(diff)>=1){
    RTC.setTime(gps);
  }
}

///////////////////////////////////////////////////////////////////////////////
//災危通報を受信する関数
///////////////////////////////////////////////////////////////////////////////
void setup_QZSS() {
  /* Initialize GNSS */
  if (Gnss.begin()) {
    Serial.println("begin error!");
  }

  /* Select satellite system */
  Gnss.select(GPS);
  Gnss.select(QZ_L1CA);
  Gnss.select(QZ_L1S);

  /* Set interval */
  Gnss.setInterval(1);

  /* Start GNSS */
  if (Gnss.start(COLD_START)) {
    Serial.println("start error!");
  }
}

String loop_QZSS() {
  /* Check update. */
  if (Gnss.waitUpdate(1000)) {
    RtcTime now;

    // Get time and position data with QZQSM
    SpNavData NavData;
    Gnss.getNavData(&NavData);

    // Check if UTC time is acquired
    SpGnssTime *time = &NavData.time;
    if (time->year >= 2000) {
      // Convert SpGnssTime to RtcTime
      RtcTime gps(time->year, time->month, time->day,
                  time->hour, time->minute, time->sec, time->usec * 1000);
      // Convert UTC to JST time
      gps += 9 * 60 * 60;

      // Compare with the current time
      now = RTC.getTime();
      int diff = now - gps;
      if (abs(diff) >= 1) {
        RTC.setTime(gps);
      }
    }

    // Get DC report
    void *handle = Gnss.getDCReport();
    if (handle) {
      QZQSM report;
      now = RTC.getTime();
      report.SetYear(now.year());
      report.Decode(((struct cxd56_gnss_dcreport_data_s*)handle)->sf);

      // Check if the report is new
      bool reported = false;
      for (int i = 0; i < HISTORY_COUNT; i++) {
        if (s_history[i] == report) {
          reported = true;
          break;
        }
      }

      // If new report, store it and return
      if (!reported) {
        char datetime[64];
        snprintf(datetime, sizeof(datetime),
                 "================================ %04d/%02d/%02d %02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
        Serial.println(datetime);
        Serial.println(report.GetReport());

        s_history[s_index] = report;
        s_index = (s_index + 1) % HISTORY_COUNT;

        // Return the report string
        return report.GetReport();
      }
    }
  }
  // Return empty string if no new report is found
  return "";
}

///////////////////////////////////////////////////////////////////////////////
//赤外線受信を行う関数
///////////////////////////////////////////////////////////////////////////////
void GetSignal(String file) {
  x = 0;
  int8_t signal_wait = 1;
  
  attachInterrupt(digitalPinToInterrupt(rxPinIR), rxIR_Interrupt_Handler, CHANGE);

  while (signal_wait) {
    Serial.println("now wait");
    delay(3000);
    if (x) {
      //受信確認用LED
      digitalWrite(LED2, HIGH);
      Serial.println("get!!");
      detachInterrupt(digitalPinToInterrupt(rxPinIR));

      //SDカード書き込み
      myFile = theSD.open(file, FILE_WRITE);
      if (myFile) {
        for (int i = 1; i < x; i++) {
          myFile.print(irBuffer[i] - irBuffer[i - 1]);
          myFile.print(",");
        }
        myFile.println();
      }
      myFile.close();

      digitalWrite(LED2, LOW);
      signal_wait = 0;
    }
  }

}

void rxIR_Interrupt_Handler() {
  if (x > maxLen) return;    //ignore if irBuffer is already full
  irBuffer[x++] = micros();  //just continually record the time-stamp of signal transitions
}

///////////////////////////////////////////////////////////////////////////////
//登録した赤外線信号をファイルから読み込む関数
///////////////////////////////////////////////////////////////////////////////
void LoadSignal(String file, int number) {
  unsigned int beginIndex;

  Serial.println("load now");
  myFile = theSD.open(file);
  String line = myFile.readStringUntil("¥n");
  myFile.close();

  beginIndex = 0;
  for (received_elements_num = 0; received_elements_num < MSG_SIZE; received_elements_num++) {
    if (received_elements_num != (MSG_SIZE - 1)) {
      unsigned int endIndex;
      endIndex = line.indexOf(',', beginIndex);
      if (endIndex != -1) {
        elements[received_elements_num] = line.substring(beginIndex, endIndex);
        share_addr[received_elements_num] = elements[received_elements_num].toInt();
        Serial.println(share_addr[received_elements_num]);
        beginIndex = endIndex + 1;
      } else {
        break;
      }
    } else {
      elements[received_elements_num] = line.substring(beginIndex);
      share_addr[received_elements_num] = elements[received_elements_num].toInt();
    }
    
  }

  Serial.println(received_elements_num);
  Serial.println("load finish");
  delay(100);

  SendSignal(number);
}

///////////////////////////////////////////////////////////////////////////////
//赤外線送信を行う関数
///////////////////////////////////////////////////////////////////////////////
void SendSignal(int8_t snddata) {
  int8_t rcvid;
  MP.Send(sndid, snddata, 1);
  MP.RecvTimeout(MP_RECV_BLOCKING);
  MP.Recv(&rcvid, &snddata, 1);
}

void SendSignal_ver2(int8_t snddata) {
  int8_t rcvid;
  MP.Send(sndid, snddata, 2);
  MP.RecvTimeout(MP_RECV_BLOCKING);
  MP.Recv(&rcvid, &snddata, 2);
}

///////////////////////////////////////////////////////////////////////////////
//エアコン動作時間を管理する関数
///////////////////////////////////////////////////////////////////////////////
//10分後の時刻を求める関数
float TimePlus10min(float time) {
  if (time >= 50) {
    time -= 50;
  } else if (time < 50) {
    time += 10;
  }
  return time;
}

///////////////////////////////////////////////////////////////////////////////
//画像認識を行う関数
///////////////////////////////////////////////////////////////////////////////
//DNNRTの開始
void DnnrtStart() {
  theCamera.begin();
  File nnbfile = theSD.open("model.nnb");
  dnnrt.begin(nnbfile);
  theCamera.startStreaming(true, CamCB);
  Serial.println("DNNRT start!");
}

void DnnrtStart_LCD() {
  theCamera.begin();
  File nnbfile = theSD.open("model.nnb");
  dnnrt.begin(nnbfile);
  theCamera.startStreaming(true, CamCB_LCD);
  Serial.println("DNNRT_LCD start!");
}

//DNNRTの終了
void DnnrtStop() {
  theCamera.startStreaming(false, CamCB);
  dnnrt.end();
  theCamera.end();
}

void DnnrtStop_LCD() {
  theCamera.startStreaming(false, CamCB_LCD);
  dnnrt.end();
  theCamera.end();
}

void CamCB(CamImage img) {
  //画像データの加工
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small
                     , CAM_CLIP_X, CAM_CLIP_Y
                     , CAM_CLIP_X + CAM_CLIP_W -1
                     , CAM_CLIP_Y + CAM_CLIP_H -1
                     , DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()){
    return;
  }

  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* tmp = (uint16_t*)small.getImgBuff();

  float *dnnbuf = input.data();
  float f_max = 0.0;
  for (int n = 0; n < DNN_IMG_H*DNN_IMG_W; ++n) {
    dnnbuf[n] = (float)((tmp[n] & 0x07E0) >> 5);
    if (dnnbuf[n] > f_max) f_max = dnnbuf[n];
  }
  
  /* normalization */
  for (int n = 0; n < DNN_IMG_W*DNN_IMG_H; ++n) {
    dnnbuf[n] /= f_max;
  }
  
  String gStrResult = "?";
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);//入力画像がいずれかに該当する確率
  int index = output.maxIndex();
  
  if (index < 2) {
    gStrResult = String(label[index]) + String(":") + String(output[index]);
  } else {
    gStrResult = String("?:") + String(output[index]);
  }

  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565); 
  
  //シリアルモニタ確認用
  Serial.println(gStrResult);

  //人を認識する閾値設定
  if (index == 1 && flag_here == false) {
    Serial.println(count_p);
    count_p++;
  } else if (count_p >= 10 && flag_here == false) {
    Serial.println("you are here!!");
    flag_here = true;
    count_p = 0;
  }

}

void CamCB_LCD(CamImage img) {
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small
                     , CAM_CLIP_X, CAM_CLIP_Y
                     , CAM_CLIP_X + CAM_CLIP_W -1
                     , CAM_CLIP_Y + CAM_CLIP_H -1
                     , DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()){
    putStringOnLcd("Clip and Reize Error:" + String(err), ILI9341_RED);
    return;
  }

  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* tmp = (uint16_t*)small.getImgBuff();

  float *dnnbuf = input.data();
  float f_max = 0.0;
  for (int n = 0; n < DNN_IMG_H*DNN_IMG_W; ++n) {
    dnnbuf[n] = (float)((tmp[n] & 0x07E0) >> 5);
    if (dnnbuf[n] > f_max) f_max = dnnbuf[n];
  }
  
  /* normalization */
  for (int n = 0; n < DNN_IMG_W*DNN_IMG_H; ++n) {
    dnnbuf[n] /= f_max;
  }
  
  String gStrResult = "?";
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  
  if (index < 10) {
    gStrResult = String(label[index]) + String(":") + String(output[index]);
  } else {
    gStrResult = String("?:") + String(output[index]);
  }
  Serial.println(gStrResult);

  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff(); 

  tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 224);
  putStringOnLcd(gStrResult, ILI9341_GREEN);

  //人を認識する閾値設定
  if (health) { 
    if (index == 1 && flag_here == false) {
      Serial.println(count_p);
      count_p++;
    }
    if (count_p >= 10 && flag_here == false) {
      Serial.println("you are here!!");
      flag_here = true;
      count_p = 0;
      delay(3000);
      //DnnrtStop_LCD();

    }
  }
}

void putStringOnLcd(String str, int color) {
  int len = str.length();
  tft.fillRect(0,224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - len/2*12;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

///////////////////////////////////////////////////////////////////////////////
//audio用関数(mp3)
///////////////////////////////////////////////////////////////////////////////
void audio_mp3(String file) {
  initializeAudio();

  openAudioFile(file);

  checkMemory();

  playAudio();
  stopAudio();

  checkMemory();
}

void audio_attention_cb(const ErrorAttentionParam *atprm) {
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
  {
    ErrEnd = true;
  }
}

void initializeAudio() {
  theAudio = AudioClass::getInstance();
  theAudio->begin(audio_attention_cb);
  theAudio->setRenderingClockMode(AS_CLKMODE_NORMAL);
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);
}

bool openAudioFile(String fileName) {
  myFile = theSD.open(fileName);
  if (!myFile)
  {
    Serial.println("File open error");
    return false;
  }

  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_AUTO, AS_CHANNEL_STEREO);
  if (err != AUDIOLIB_ECODE_OK)
  {
    Serial.println("Player0 initialize error");
    return false;
  }

  err = theAudio->writeFrames(AudioClass::Player0, myFile);
  if (err != AUDIOLIB_ECODE_OK && err != AUDIOLIB_ECODE_FILEEND)
  {
    Serial.println("File read error");
    myFile.close();
    return false;
  }

  return true;
}

void playAudio() {
  theAudio->setVolume(120);
  theAudio->startPlayer(AudioClass::Player0);

  while (true)
  {
    int err = theAudio->writeFrames(AudioClass::Player0, myFile);

    if (err == AUDIOLIB_ECODE_FILEEND)
    {
      Serial.println("Main player file end");
      break;
    }
    if (err || ErrEnd)
    {
      Serial.printf("Error code: %d\n", err);
      break;
    }

    usleep(40000);
  }
}

void stopAudio() {
  theAudio->stopPlayer(AudioClass::Player0);
  myFile.close();
  theAudio->setReadyMode();
  theAudio->end();
}

void checkMemory() {
  int usedMem, freeMem, largestFreeMem;

  MP.GetMemoryInfo(usedMem, freeMem, largestFreeMem);

  MPLog("Used:%4d [KB] / Free:%4d [KB] (Largest:%4d [KB])\n",
        usedMem / 1024, freeMem / 1024, largestFreeMem / 1024);
}

///////////////////////////////////////////////////////////////////////////////
//LCD用関数
///////////////////////////////////////////////////////////////////////////////
void load_lcd() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(30, 100);
  tft.print("Now Loading...");
}

void line_lcd_step1() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(30, 100);
  tft.print("LINE contact...");
}

void line_lcd_step2() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(30, 100);
  tft.print("LINE check...");
}

void home_lcd() {
  int i;
  int w = tft.width();
  int h = tft.height();

  //塗りつぶし
  tft.fillScreen(ILI9341_BLACK);

  uint16_t Navy = tft.color565(0, 0, 128);
  uint16_t GREEN = tft.color565(0, 128, 0);

  tft.fillRect(0,0,w,h/4,GREEN);
  tft.fillRect(0,h/4,w,h/4,Navy);
  tft.fillRect(0,h/4*2,w,h/4,ILI9341_MAGENTA);

  //フレーム
  //横線
  for (i=0; i<3; i++) tft.drawLine(0, h/4+i, w, h/4+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*2+i, w, h/4*2+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*3+i, w, h/4*3+i, ILI9341_WHITE);

  //アイコン
  //Setup
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8-5);
  tft.print("->");
  tft.setTextSize(4);
  tft.setCursor(w/5, h/16);
  tft.print("Setup");

  //for Health
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8*3-5);
  tft.print("->");
  tft.setTextSize(4);
  tft.setCursor(w/5, h/16*5);
  tft.print("for Health");

  //for Hazard
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8*5-5);
  tft.print("->");
  tft.setTextSize(4);
  tft.setCursor(w/5, h/16*9);
  tft.print("for Hazard");

  //home
  uint16_t Skin = tft.color565(210, 160, 120);
  tft.fillRect(w/12*5+16,h-30,25,25,ILI9341_WHITE);
  tft.fillTriangle(w/2-2,h/16*13,w/12*5+3,h-30,w/12*7,h-30,ILI9341_RED);
  tft.fillRect(w/2,h-23,10,18,Skin);
  tft.fillRect(w/2+7,h-15,2,2,ILI9341_BLACK);
}

void set_lcd() {
  int i;
  int w = tft.width();
  int h = tft.height();
  
  //塗りつぶし
  tft.fillScreen(ILI9341_BLACK);
  uint16_t GREEN = tft.color565(0, 128, 0);

  tft.fillRect(0,0,w,h/4,GREEN);
  tft.fillRect(0,h/4,w,h/4,GREEN);
  tft.fillRect(0,h/4*2,w,h/4,GREEN);

  //フレーム
  //横線
  for (i=0; i<3; i++) tft.drawLine(0, h/4+i, w, h/4+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*2+i, w, h/4*2+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*3+i, w, h/4*3+i, ILI9341_WHITE);

  //アイコン
  //for Health
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8-5);
  tft.print("->");
  tft.setTextSize(4);
  tft.setCursor(w/5, h/16);
  tft.print("set Signal");

  //for Hazard
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8*3-5);
  tft.print("->");
  tft.setTextSize(4);
  tft.setCursor(w/5, h/16*5);
  tft.print("set View");

  //ホームアイコン
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(w/24, h/8*5-5);
  tft.print("->");
  int x = 75;
  int y = 60;
  uint16_t Skin = tft.color565(210, 160, 120);
  tft.fillRect(w/12*5+16-x,h-30-y,25,25,ILI9341_WHITE);
  tft.fillTriangle(w/2-2-x,h/16*13-y,w/12*5+3-x,h-30-y,w/12*7-x,h-30-y,ILI9341_RED);
  tft.fillRect(w/2-x,h-23-y,10,18,Skin);
  tft.fillRect(w/2+7-x,h-15-y,2,2,ILI9341_BLACK);

  //setup
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(w/3, h/16*13);
  tft.print("Setup");

}

void str_lcd(String str) {
  tft.fillScreen(ILI9341_BLACK);
  
  int w = tft.width();
  int h = tft.height();
  //for ir
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(w/3, h/5*2);
  tft.print(str);
}

void info_lcd(float temp, RtcTime now) {
  if (old_temp != temp) {
    int w = tft.width();
    int h = tft.height();

    //室温
    //int temp2 = round(temp);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(7);
    tft.setCursor(w/3-10, h/4);
    tft.print(old_temp,1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(w/3-10, h/4);
    tft.print(temp,1);
    tft.setCursor(w/4*3+35,h/4-3);
    tft.print("c");
    tft.setTextSize(3);
    tft.setCursor(w/4*3+20,h/4+1);
    tft.print("o");

    old_temp = temp;

  } else if (old_time != now.day()) {
    old_time = now.day();
    int w = tft.width();
    int h = tft.height();

    //日付
    tft.fillRect(0,h/4*3+5,w,h/4-5,ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(4);
    tft.setCursor(w/16*2, h/16*13);
    tft.print(now.year());
    tft.print("/");
    if (now.month()<10) tft.print(0);
    tft.print(now.month());
    tft.print("/");
    if (now.day()<10) tft.print(0);
    tft.print(now.day());
  }
}

void info_lcd2(RtcTime now) {
  if (old_min != now.minute()) {
    int w = tft.width();
    int h = tft.height();

    //室温
    //int temp2 = round(temp);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(7);
    tft.setCursor(w/3-10, h/4);
    if (old_hour<10) tft.print(0);
    tft.print(old_hour,1);
    tft.print(":");
    if (old_min<10) tft.print(0);
    tft.print(old_min);

    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(w/3-10, h/4);
    if (now.hour()<10) tft.print(0);
    tft.print(now.hour());
    tft.print(":");
    if (now.minute()<10) tft.print(0);
    tft.print(now.minute());

    old_hour = now.hour();
    old_min = now.minute();
  }
  if (old_time != now.day()) {
    old_time = now.day();
    int w = tft.width();
    int h = tft.height();

    //日付
    tft.fillRect(0,h/4*3+5,w,h/4-5,ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(4);
    tft.setCursor(w/16*2, h/16*13);
    tft.print(now.year());
    tft.print("/");
    if (now.month()<10) tft.print(0);
    tft.print(now.month());
    tft.print("/");
    if (now.day()<10) tft.print(0);
    tft.print(now.day());

    old_time = now.day();
  }
}

void earthquake_lcd() {  
  int i;
  int w = tft.width();
  int h = tft.height();
  //黒色背景
  tft.fillScreen(ILI9341_BLACK);
  //緊急地震速報
  for (i=0;i<20;i++) {
    for (int l=0;l<20;l++) tft.drawLine(l+i*40,0,l+i*40,h,ILI9341_YELLOW);
  }
  tft.fillRect(0,h/6,w,h/6*4,ILI9341_BLACK);
  //earthquake
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(w/9, h/5*2);
  tft.print("earthquake");

}

void Health_lcd() {
  int i;
  int w = tft.width();
  int h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  uint16_t Navy = tft.color565(0, 0, 100);
  tft.fillRect(0,0,w/4,h/4*3,Navy);

  //フレーム
  //縦線
  for (i=0; i<3; i++) tft.drawLine(w/4+i, 0, w/4+i, h/4*3, ILI9341_WHITE);
  //横線
  for (i=0; i<3; i++) tft.drawLine(0, h/4+i, w/4+i, h/4+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*2+i, w/4+i, h/4*2+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*3+i, w, h/4*3+i, ILI9341_WHITE);

  //アイコン
  //home
  //ホームアイコン
  int x = 120;
  int y = 185;
  uint16_t Skin = tft.color565(210, 160, 120);
  tft.fillRect(w/12*5+16-x,h-30-y,25,25,ILI9341_WHITE);
  tft.fillTriangle(w/2-2-x,h/16*13-y,w/12*5+3-x,h-30-y,w/12*7-x,h-30-y,ILI9341_RED);
  tft.fillRect(w/2-x,h-23-y,10,18,Skin);
  tft.fillRect(w/2+7-x,h-15-y,2,2,ILI9341_BLACK);

  //ON信号
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(4);
  tft.setCursor(w/32+5, h/8*3-10);
  //tft.print("ON");

  //ランダムおしゃべり
  x = 0;
  y = -63;
  tft.fillRoundRect(13+x,h/4*2+12+y,53,32,3,ILI9341_WHITE);
  for (i=0;i<3;i++) tft.drawRoundRect(13+i+x,h/4*2+12+i+y,53-i*2,32-i*2,3,ILI9341_GREEN);
  tft.fillTriangle(18+x,h/4*2+32+12+y,28+x,h/4*2+32+12+y,20+x,h/4*2+32+24+y,ILI9341_WHITE);
  for (i=0;i<3;i++) tft.drawTriangle(18+i+x,h/4*2+32+12+y,28-i+x,h/4*2+32+12+y,20+x,h/4*2+32+24-i+y,ILI9341_GREEN);
  for (i=0;i<5;i++) tft.drawLine(21+x,h/4*2+32+8+i+y,25+x,h/4*2+32+8+i+y,ILI9341_WHITE);

  //文字
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(w/32+3+x, h/8*5-13+y);
  tft.print(" ? ");

  //OFF信号
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  for (i=0;i<5;i++) tft.drawLine(0+i,h/4*2,w/4,h/4*3-i,ILI9341_WHITE);
  

}

void Hazard_lcd() {
  int i;
  int w = tft.width();
  int h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0,0,w/4,h/4*3,ILI9341_MAGENTA);

  //フレーム
  //縦線
  for (i=0; i<3; i++) tft.drawLine(w/4+i, 0, w/4+i, h/4*3, ILI9341_WHITE);
  //横線
  for (i=0; i<3; i++) tft.drawLine(0, h/4+i, w/4+i, h/4+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*2+i, w/4+i, h/4*2+i, ILI9341_WHITE);
  for (i=0; i<3; i++) tft.drawLine(0, h/4*3+i, w, h/4*3+i, ILI9341_WHITE);

  //アイコン
  //home
  //ホームアイコン
  int x = 120;
  int y = 185;
  uint16_t Skin = tft.color565(210, 160, 120);
  tft.fillRect(w/12*5+16-x,h-30-y,25,25,ILI9341_WHITE);
  tft.fillTriangle(w/2-2-x,h/16*13-y,w/12*5+3-x,h-30-y,w/12*7-x,h-30-y,ILI9341_RED);
  tft.fillRect(w/2-x,h-23-y,10,18,Skin);
  tft.fillRect(w/2+7-x,h-15-y,2,2,ILI9341_BLACK);

  //ON信号
  tft.setCursor(w/32+2+13, h/8*3-12);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("*");
  tft.setCursor(w/32+20+13, h/8*3);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("*");

  //OFF信号
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(w/32-3, h/8*5-13);
  tft.print("");

  //♪の描写
  x = 30;
  y = 30;
  for (i=0;i<3;i++) tft.fillCircle(w/32-3+x+i, h/8*5-13+y,8,ILI9341_BLACK);
  for (i=0;i<3;i++) tft.drawLine(w/32-3+x+8+i,h/8*5-13+y,w/32-3+x+8+i,h/8*5-13-8,ILI9341_BLACK);


}

///////////////////////////////////////////////////////////////////////////////
//温度センサ用関数（wbgt算出）
///////////////////////////////////////////////////////////////////////////////
int dht_wbgt(float temp, float hum) {
  int wbgt = 0;
  //wbgt換算表より
  if (temp>=28 && hum>=75) wbgt = 1;
  if (temp>=29 && hum>=70) wbgt = 1;
  if (temp>=30 && hum>=65) wbgt = 1;
  if (temp>=31 && hum>=60) wbgt = 1;
  if (temp>=32 && hum>=55) wbgt = 1;
  if (temp>=33 && hum>=50) wbgt = 1;
  if (temp>=34 && hum>=45) wbgt = 1;
  if (temp>=35) wbgt = 1;

  return wbgt;
}

///////////////////////////////////////////////////////////////////////////////
//超音波センサ用関数
///////////////////////////////////////////////////////////////////////////////
void distance() {
  // 距離計測処理
  digitalWrite(trigPin, LOW);              
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);             
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  Duration = pulseIn(echoPin, HIGH);      
  Duration = Duration / 2;                        //計測時間を片道分に
  Distance = Duration * 340 * 100 / 1000000;      //音速に直して，単位cm
  
  if (Distance < 10) { // 距離が15cm未満の場合
    count_distance++;
    Serial.println(Distance);
    Serial.println(count_distance);
  }else{
    Serial.print("距離: ");
    Serial.print(Distance);
    Serial.println(" cm");
    count_distance = 0;
  }
  if (count_distance>1) {
    Serial.println("前が見えないにゃ");
    audio_mp3("Sound12.mp3");
    count_distance = 0;
  }
  
  delay(500);
}

///////////////////////////////////////////////////////////////////////////////
//LED用関数
///////////////////////////////////////////////////////////////////////////////
void LEDf() {
  float lastRG = fRG;
  //float lastGreen = fGreen;
  //float lastBlue = fBlue;

  fRG = fluctuate(fRG);
  //fGreen = fluctuate(fGreen);220 - (int)(interpolat(fRG, lastRG, i) * 80)
  //fBlue = fluctuate(fBlue);

  for (int i = 1; i <= 80; i++)
  {
    analogWrite(LED_PIN_RG, 255);
    analogWrite(LED_PIN_W, 255 - (int)(interpolate(fRG, lastRG, i) * 255));  //カメラ撮影用
    //analogWrite(LED_PIN_W, 255 - (int)(interpolate(fRed, lastRed, i) * 255));  //カメラ撮影用
    //analogWrite(LED_PIN_W, 220 - (int)(interpolate(fRG, lastRG, i) * 80));  //実際に見るとき用
    delay(20);
  }
}

//1/fゆらぎ
//滑らかな変化にする
float interpolate(float next, float prev, int percent) {
  return (next * percent / 100 + (prev * (100 - percent)) / 100);
}

float fluctuate(float x) {
  //1/fゆらぎ
  if (x < 0.5)
  {
    x = x + 2 * x * x;
  }
  else
  {
    x = x - 2 * (1.0 - x) * (1.0 - x);
  }
  //極端な値の排除
  if (x < 0.05 || x > 0.95)
  {
    x = random(100, 900) / 1000.0;
  }

  return (x);
}

///////////////////////////////////////////////////////////////////////////////
//LINE送信関数
///////////////////////////////////////////////////////////////////////////////
void line_send(int num) {
  //SubCore2でLINE送信
  audio_mp3("Sound13.mp3");
  delay(100);
  MP.begin(2);
  checkMemory();
  delay(3000);
  SendSignal_ver2(num);
  delay(3000);
  MP.end(2);
  checkMemory();
  //LINE送信完了後HWリセット
  //再起動記録用ファイル
  //myFile = theSD.open("reset.txt", FILE_WRITE);
  if (health == HIGH) myFile = theSD.open("reset_health.txt", FILE_WRITE);
  if (hazard == HIGH) myFile = theSD.open("reset_hazard.txt", FILE_WRITE);
  myFile.print(0);
  myFile.print(",");
  myFile.println();
  myFile.close();
  //再起動
  Watchdog.begin();
  Watchdog.start(100);
  delay(1000);
  Watchdog.kick();
}

///////////////////////////////////////////////////////////////////////////////
//setup関数
///////////////////////////////////////////////////////////////////////////////
void setup() {
  int i;
  Serial.begin(115200);
  while (!Serial);

  //SDカード設定
  while (!theSD.begin()) {}

  //LCD待機画面
  if (theSD.exists("reset.txt")) {
    line_lcd_step2();
  } else {
    load_lcd();
  }
  //サブコア起動
  MP.begin(1);

  //共有メモリ
  uint32_t *addr = (uint32_t *)MP.AllocSharedMemory(MSG_SIZE);
  if (!addr) {
    printf("Error1: out of memory\n");
    return;
  }

  printf("SharedMemory Address=@%08lx\n", (uint32_t)addr);  // 先頭アドレスの表示
  share_addr = &addr[0];                                    // 共有メモリの先頭アドレスをグローバルで保持
  for (i = 0; i < MSG_SIZE; i++) {                          // 共有メモリを0x0000で初期化
    addr[i] = 0x0000;
  }

  //サブコアへ共有メモリのアドレスを通知
  int8_t msgid = 10;
  MP.Send(msgid, addr, 1);  

  //サブコアより受信確認
  void *raddr;
  MP.Recv(&msgid, &raddr, 1);

  //共有メモリを確認
  for (i = 0; i < MSG_SIZE; i++) {
    if (addr[i] != 0x0000) {
      printf("Error: @%08lx\n", (uint32_t)&addr[i]);
      while (1);
    }
  }
  printf("SharedMemory[1] Check: OK!\n");

  //SDカード設定
  //while (!theSD.begin()) {}

  //DHT11
  dht.begin();

  //超音波センサ
  pinMode(echoPin, INPUT);   // エコーピンを入力に
  pinMode(trigPin, OUTPUT);  // トリガーピンを出力に

  //LED
  randomSeed(analogRead(A0));
  analogWrite(LED_PIN_RG,0);
  analogWrite(LED_PIN_W,0);
  analogWrite(LED_PIN_B,0);

  //割込み用設定
  pinMode(interruptPin4, INPUT_PULLUP);
  pinMode(interruptPin5, INPUT_PULLUP);
  pinMode(interruptPin6, INPUT_PULLUP);
  //pinMode(interruptPin7, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptPin4), function4, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin5), function5, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin6), function6, FALLING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin7), function7, FALLING);

  //赤外線信号ファイルの読み込み
  if (theSD.exists("cool.txt")) {
    LoadSignal("cool.txt",3);
  }
  if (theSD.exists("heat.txt")) {
    LoadSignal("heat.txt",4);
  }
  if (theSD.exists("off.txt")) {
    LoadSignal("off.txt",5);
  }

  //デバック用
  //RTC
  RTC.begin(); // RTCを初期化
  RtcTime compiledDateTime(__DATE__, __TIME__); // コンパイル時の日時を取得
  RTC.setTime(compiledDateTime); // RTCに日時を設定
  RTC.end();

  //謎のメモリ安定化作業
  if (theSD.exists("reset_health.txt")) {
    //for Healthの再起動
    audio_mp3("Sound1.mp3");
    theSD.remove("reset_health.txt");
    Health_lcd();
    RTC.begin();
    Serial.println("reboot");
    //再起動時の時刻を記録する必要があるかも
    flag_power = true;
    home = LOW;
    health = HIGH;

  } else {
    //起動音
    Serial.println("start!!");
    audio_mp3("Sound2.mp3");
    home_lcd();
  }
}

void loop() {  
  //***************************************************//
  //ホーム画面(home == HIGH)
  //***************************************************//
  if (home == HIGH) {
    ///////////////////
    //他機能との干渉防止のgps終了
    ///////////////////
    if (flag_gnss_down==false) {
      Serial.println("gnss stop");
      Gnss.stop();
      Gnss.end();
      delay(500);
      flag_gnss_down = true;
    }

  }

  //***************************************************//
  //セットモード(set == HIGH)
  //***************************************************//
  if (set == HIGH) {
    ////////////////////
    //赤外線受信・保存
    ////////////////////
    if (flag_signal) {
      //登録済みの信号を削除
      if (theSD.exists("cool.txt")) {
        theSD.remove("cool.txt");
      }
      if (theSD.exists("heat.txt")) {
        theSD.remove("heat.txt");
      }
      if (theSD.exists("off.txt")) {
        theSD.remove("off.txt");
      }

      //受信開始
      tft.fillScreen(ILI9341_BLACK);
      str_lcd("wait");
      audio_mp3("Sound3.mp3");
      
      //冷房信号の登録
      audio_mp3("Sound4.mp3");
      str_lcd("push!!");
      GetSignal("cool.txt");
      LoadSignal("cool.txt", 3);
      str_lcd("wait");
      
      //停止信号の登録
      audio_mp3("Sound5.mp3");
      str_lcd("push!!");
      GetSignal("off.txt");
      LoadSignal("off.txt", 5);
      str_lcd("wait");      

      //暖房信号の登録
      audio_mp3("Sound6.mp3");
      str_lcd("push!!");
      GetSignal("heat.txt");
      LoadSignal("heat.txt", 4);
      str_lcd("wait");

      //信号確認
      str_lcd("check");
      audio_mp3("Sound7.mp3");
      audio_mp3("Sound8.mp3");
      SendSignal(2);
      delay(5000);
      SendSignal(0);
      delay(5000);
      SendSignal(2);
      delay(5000);
      SendSignal(1);
      delay(5000);
      SendSignal(2);
      delay(5000);

      //赤外線を送信するとLCDがフリーズするため一度リセット
      tft.begin();
      tft.setRotation(3);
      set_lcd();

      flag_signal = false;

    }
    ////////////////////
    //人認識
    ////////////////////
    if (flag_dnnrt == 1) {
      DnnrtStart_LCD();
      flag_dnnrt = 2;
      
    } else if (flag_dnnrt == 3) {
      DnnrtStop_LCD();
      flag_dnnrt = 0;
      checkMemory();
      set_lcd();
      
    }
  }

  //***************************************************//
  //見守りモード(health == HIGH)
  //***************************************************//
  if (health == HIGH) {
    if (flag_alert == false) {
      /////////////////
      //時刻取得
      /////////////////
      if (time_set) {
        if (flag_gnss_down==true) {
          Serial.println("gps start");
          Gnss.begin();
          Gnss.start();
          delay(500);
          flag_gnss_down = false;
        }
        get_time();
      }

      ////////////////
      //画面更新
      ////////////////
      float Humidity = dht.readHumidity();
      float Temperature = dht.readTemperature();
      RtcTime now = RTC.getTime();
      info_lcd(Temperature, now);
      if (flag_alert == false && now.second()%2 == 0) distance();

      //////////////////
      //エアコン動作前
      //////////////////
      if (flag_power == false && Temperature>=28) {
        //WGBT基準で危険温度であれば人認識開始
        if (flag_gnss_down == false) {
          RTC.end();
          Gnss.stop();
          Gnss.end();
          flag_gnss_down = true;
          Serial.println("GNSS stop");
        }
        //暑さをLEDで表現
        analogWrite(LED_PIN_RG,255);
        analogWrite(LED_PIN_W,0);
        analogWrite(LED_PIN_B,0);
        //室温異常を音声通知
        audio_mp3("Sound14.mp3");
        delay(2000);
        //人認識開始
        DnnrtStart_LCD();
        flag_alert = true;
      
      }

      ///////////////////
      //エアコン動作後
      ///////////////////
      if (flag_power == true) {
        //earthquake_lcd();
        //audio_mp3("Sound17.mp3");
        //delay(10000);

      }
    }
    
    //////////////////////
    //人認識動作中
    //////////////////////
    if (flag_alert == true && flag_power == false) {
      if (flag_here) {
        //人認識終了
        DnnrtStop_LCD();
        flag_here = false;
        //冷房動作開始
        audio_mp3("Sound15.mp3");
        SendSignal(0);
        flag_alert = false;
        flag_power = true;
        old_time = 0;
        old_temp = 0;
        //LCDリセット
        tft.begin();
        tft.setRotation(3);
        //冷房動作開始のLINE通知
        line_lcd_step1();
        line_send(0);

        //冷房動作開始時刻を記録
        //RtcTime time = RTC.getTime();
        //float start = time.minute();

        //再確認時刻を設定
        //next_time = TimePlus10min(start);
      }
    } 

    ////////////////////
    //ランダムおしゃべり
    ////////////////////
    if (flag_random_voice>0) {
      int v = random(1, 100) % 3;
      Serial.println(v);
      if (v==0) audio_mp3("Sound9.mp3");
      if (v==1) audio_mp3("Sound10.mp3");
      if (v==2) audio_mp3("Sound11.mp3");

      flag_random_voice = 0;

    }

    //WGBT基準で熱中症の危険がある場合は人認識を開始。さらに人を検出した場合は冷房ON
    /*
    if (((Temperature>=28 && Humidity>=75) || (Temperature>=29 && Humidity>=70) || (Temperature>=30 && Humidity>=65) || (Temperature>=31 && Humidity>=60) || 
        (Temperature>=32 && Humidity>=50) || (Temperature>=33 && Humidity>=45) || (Temperature>=34 && Humidity>=40) || (Temperature>=35) || (flag_alert == true)) && flag_power == false){ 
      //人認識開始
      if (flag_alert == false) {
        DnnrtStart();
        flag_alert = true;
      }

      //温度が閾値を下回った場合
      if (Temperature<=26) {
        DnnrtStop();
        flag_here = false;
        flag_alert = false;
      }

      //人検出で冷房ON
      if (flag_here == true) {
        //人認識終了
        DnnrtStop();
        flag_here = false;

        //冷房動作開始の音声
        //audio("sound.wav");

        //冷房ON信号を送信
        SendSignal(0);
        flag_alert = false;
        flag_power = true;

        //冷房動作開始時刻を記録
        RtcTime time = RTC.getTime();
        float start = time.minute();

        //再確認時刻を設定
        next_time = TimePlus10min(start);
      }
    }

    //エアコン動作開始後10分おきに人検出。60分間検出されない場合エアコンOFF
    if (flag_power == true) {
      RtcTime time = RTC.getTime();
      float now_hour = time.hour();
      float now_minute = time.minute();

      //10分おきの人検出
      if (next_time == now_minute) {
        //人認識を開始
        if (count_m == 0) DnnrtStart();
        next_time = TimePlus10min(now_minute);
        
        //定刻でエアコンOFF
        if (now_hour == 18) {
          //人認識を終了
          DnnrtStop();
          flag_here = false;
          //エアコンOFF
          SendSignal(2);
          flag_power = false;
        }

        //時間カウント
        count_m++;
      }

      //人が検出された場合はカウントを0に戻す
      if (flag_here == true) {       
        //人認識を終了
        DnnrtStop();
        flag_here = false;
        count_m = 0;

        //人を検知したのでついでに話しかける
        //audio_mp3("にゃ～.mp3");

      }

      //連続して60分間人が検出されなかった場合はエアコンOFF
      if (count_m >= 6) {
        //人認識を終了
        DnnrtStop();
        flag_here = false;

        //エアコンOFFを知らせる音声
        //audio_mp3(".mp3");

        //エアコンをOFF
        SendSignal(2);
        flag_power = false;
      }
    }
    */

  }

  //***************************************************//
  //寄り添いモード(hazard == HIGH)
  //***************************************************//
  if (hazard == HIGH) {
    //メイン機能は災危通報を受信して事前に地震の到来を警告
    ///////////////////
    //時刻取得
    ///////////////////
    if (time_set) {
      if (flag_gnss_down==true) {
        Serial.println("gps start");
        Gnss.begin();
        Gnss.start();
        delay(500);
        flag_gnss_down = false;
      }
      get_time();
    }
    ///////////////////
    //災危通報
    ///////////////////
    String report = loop_QZSS();
    if (report.length() > 0) {
      // "緊急地震速報"が含まれている場合、flagを1に設定
      if (report.indexOf("緊急地震速報") >= 0) {
        line_send(3);
      }
      // "津波"が含まれている場合、flagを2に設定
      else if (report.indexOf("津波") >= 0) {
        line_send(4);
      }

    }

    ///////////////////
    //画面更新
    ///////////////////
    RtcTime now = RTC.getTime();
    info_lcd2(now);

    ///////////////////
    //ゆらぎ
    ///////////////////
    if (flag_light) {
      LEDf();
    } else if (flag_light == false) {
      analogWrite(LED_PIN_RG,0);
      analogWrite(LED_PIN_W,0);
      analogWrite(LED_PIN_B,0);
    }

    //////////////////
    //音楽再生
    //////////////////
    if (flag_music) {
      audio_mp3("Music2.mp3");
      flag_music = 0;

    }

  }

}
