#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <RTC.h>
#include <HttpGs2200.h>  // GainSpan HTTP clientライブラリをインクルード
#include <TelitWiFi.h>   // Telit WiFiモジュール用ライブラリをインクルード

#define LINETOKEN "i3gqr7Zs3AFbsJsrdJrVV7FhcLQVEiDenodJWpT8scz"  //グループLINEトークン
#define AP_SSID "iPhone (330)"
#define PASSPHRASE "ovk7nnopm9q7f"
#define CONSOLE_BAUDRATE 115200  // シリアル通信のボーレートを定義
#define HTTP_SRVR_IP "notify-api.line.me"
#define HTTP_PORT "443"               // HTTPSだから443
#define HTTP_POST_PATH "/api/notify"  // LINE Notify APIのパス
#define ROOTCA_FILE "CERTS/GlobalSign.crt"

char sendData[100]; // POSTデータを格納するための文字配列
const uint16_t RECEIVE_PACKET_SIZE = 1500;          // 受信パケットサイズの定義
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };  // 受信データを格納する配列

//notify-api.line.me のルート認証 SD使えないから貼る。
const uint8_t root_ca[] = {
"-----BEGIN CERTIFICATE-----\n"
"MIIDXzCCAkegAwIBAgILBAAAAAABIVhTCKIwDQYJKoZIhvcNAQELBQAwTDEgMB4G\n"
"A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjMxEzARBgNVBAoTCkdsb2JhbFNp\n"
"Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDkwMzE4MTAwMDAwWhcNMjkwMzE4\n"
"MTAwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMzETMBEG\n"
"A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI\n"
"hvcNAQEBBQADggEPADCCAQoCggEBAMwldpB5BngiFvXAg7aEyiie/QV2EcWtiHL8\n"
"RgJDx7KKnQRfJMsuS+FggkbhUqsMgUdwbN1k0ev1LKMPgj0MK66X17YUhhB5uzsT\n"
"gHeMCOFJ0mpiLx9e+pZo34knlTifBtc+ycsmWQ1z3rDI6SYOgxXG71uL0gRgykmm\n"
"KPZpO/bLyCiR5Z2KYVc3rHQU3HTgOu5yLy6c+9C7v/U9AOEGM+iCK65TpjoWc4zd\n"
"QQ4gOsC0p6Hpsk+QLjJg6VfLuQSSaGjlOCZgdbKfd/+RFO+uIEn8rUAVSNECMWEZ\n"
"XriX7613t2Saer9fwRPvm2L7DWzgVGkWqQPabumDk3F2xmmFghcCAwEAAaNCMEAw\n"
"DgYDVR0PAQH/BAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFI/wS3+o\n"
"LkUkrk1Q+mOai97i3Ru8MA0GCSqGSIb3DQEBCwUAA4IBAQBLQNvAUKr+yAzv95ZU\n"
"RUm7lgAJQayzE4aGKAczymvmdLm6AC2upArT9fHxD4q/c2dKg8dEe3jgr25sbwMp\n"
"jjM5RcOO5LlXbKr8EpbsU8Yt5CRsuZRj+9xTaGdWPoO4zzUhw8lo/s7awlOqzJCK\n"
"6fBdRoyV3XpYKBovHd7NADdBj+1EbddTKJd+82cEHhXXipa0095MJ6RMG3NzdvQX\n"
"mcIfeg7jLQitChws/zyrVQ4PkX4268NXSb7hLi18YIvDQVETI53O9zJrlAGomecs\n"
"Mx86OyXShkDOOyyGeMlhLxS67ttVb9+E7gUJTb0o2HLO02JQZR7rkpeDMdmztcpH\n"
"WD9f\n"
"-----END CERTIFICATE-----\n"
};


TelitWiFi gs2200;                   // Telit WiFiモジュールのインスタンスを作成
TWIFI_Params gsparams;              // WiFi接続のパラメータを格納する構造体
HttpGs2200 theHttpGs2200(&gs2200);  // HTTPクライアントのインスタンスを作成
HTTPGS2200_HostParams hostParams;   // ホストパラメータの構造体
//SDClass theSD;                      // SDカード操作のためのインスタンス

void line_set() {
  //LINE通知用設定
  /* GS2200のSPIアクセスを初期化 */
  Init_GS2200_SPI_type(iS110B_TypeC);

  /* ATコマンドライブラリバッファを初期化 */
  gsparams.mode = ATCMD_MODE_STATION;                // モードをステーションに設定
  gsparams.psave = ATCMD_PSAVE_DEFAULT;              // デフォルトのパラメータを設定
  if (gs2200.begin(gsparams)) {                      // GS2200の初期化を試みる
    Serial.println("GS2200の初期化に失敗しました");  // 初期化失敗のログ
    while (1);  // 失敗時は無限ループ
  }

  /* GS2200をAPに接続 */
  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {  // SSIDとパスフレーズを使用して接続
    Serial.println("APへの接続に失敗しました");        // 接続失敗のログ
    while (1);  // 失敗時は無限ループ
  }

  hostParams.host = (char *)HTTP_SRVR_IP;  // ホストのIPアドレスを設定
  hostParams.port = (char *)HTTP_PORT;     // ポートを設定
  theHttpGs2200.begin(&hostParams);        // HTTPクライアントを初期化

  Serial.println("HTTPセキュアクライアントを開始します");  // クライアント起動のログ

  /* HTTPヘッダーを設定 */
  theHttpGs2200.config(HTTP_HEADER_AUTHORIZATION, "Bearer " LINETOKEN);                 // 認証情報を設定
  theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");                       // 転送エンコーディングを設定
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/x-www-form-urlencoded");  // コンテンツタイプを設定
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);                                 // ホストヘッダーを設定

  // サーバーに接続する前にSDカード上のファイルから証明書を設定
  char time_string[128];        // 時間を格納するための文字列配列
  RtcTime rtc = RTC.getTime();  // 現在のRTC時間を取得
  //時間をフォーマットして文字列に格納
  snprintf(time_string, sizeof(time_string), "%02d/%02d/%04d,%02d:%02d:%02d", rtc.day(), rtc.month(), rtc.year(), rtc.hour(), rtc.minute(), rtc.second());

  // 証明書を設定
  const int root_ca_size = sizeof(root_ca);
  theHttpGs2200.set_cert((char *)"TLS_CA", time_string, 0, 1, (uint8_t*)root_ca, root_ca_size);
}

void line_send(String message) {
  bool result = false;                                             // 処理結果の初期値
  theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");  // 転送エンコーディングを設定

  String query = String("message=") + String(message);  // queryにmessageを設定

  // POSTリクエストを送信する
  Serial.println("POSTリクエストを送信中...");                 // メッセージを表示
  result = theHttpGs2200.post(HTTP_POST_PATH, query.c_str());  // queryをPOSTデータとして送信

  if (false == result) {                                   // リクエストが失敗した場合
    Serial.println("POSTリクエストの送信に失敗しました");  // エラーメッセージ
    return;                                                // 次の処理へ
  }

  do {                                                             // レスポンスを待機
    result = theHttpGs2200.receive(5000);                          // 5秒間レスポンスを受信
    if (result) {                                                  // 成功した場合
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);  // 受信データを読み込む
      Serial.print("受信データ: ");                                // データを表示
      Serial.println((char *)(Receive_Data));
    } else {  // レスポンスがなかった場合
      Serial.println("AT+HTTPSENDが完了しました");
    }
  } while (result);                                // 成功している限り繰り返す
  result = theHttpGs2200.end();                    // リクエストを終了
  Serial.println("POSTリクエストが終了しました");  // メッセージを表示

  delay(2000);
}


void setup()
{
  MP.begin();
  Serial.begin(115200);
  while (!Serial);
  MPLog("subcore2 start");

  //RTC
  RTC.begin();                                   // RTCを初期化
  RtcTime compiledDateTime(__DATE__, __TIME__);  // コンパイル時の日時を取得
  RTC.setTime(compiledDateTime);                 // RTCに日時を設定
}

void loop()
{
  int      ret;
  int8_t   rcvid;
  uint32_t rcvdata;

  //***************************************************//
  //受信待機
  //***************************************************//
  MP.RecvTimeout(MP_RECV_BLOCKING);
  MP.Recv(&rcvid, &rcvdata);

  //LINE送信
  if (rcvdata == 0) {
    MPLog("line set start");
    line_set();
    line_send("冷房が起動しました");
    RTC.end();
    MPLog("line send complete");
    MP.Send(rcvid, 1);
  } else if (rcvdata == 1) {
    MPLog("line set start");
    line_set();
    line_send("暖房が起動しました");
    RTC.end();
    MPLog("line send complete");
    MP.Send(rcvid, 1);
  } else if (rcvdata == 2) {
    MPLog("line set start");
    line_set();
    line_send("起床を確認しました");
    RTC.end();
    MPLog("line send complete");
    MP.Send(rcvid, 1);
  } else if (rcvdata == 3) {
    MPLog("line set start");
    MP.Send(rcvid, 1);
    MP.RecvTimeout(MP_RECV_BLOCKING);
    MP.Recv(&rcvid, &str);
    line_set();
    line_send(str);
    RTC.end();
    MPLog("line send complete");
    MP.Send(rcvid, 1);
  }
}