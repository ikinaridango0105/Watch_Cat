///////////////////////////////////////////////////////////////////////////////
//サブコア1
///////////////////////////////////////////////////////////////////////////////
#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <IRremote.h>

IRsend irsend;

/*コア間共有メモリ*/
#define MSG_SIZE 500
int32_t *share_addr;

/*赤外線信号*/
#define data_size 500
unsigned int signal_cool[data_size];
unsigned int signal_heat[data_size];
unsigned int signal_off[data_size];

void setup()
{
  MP.begin();

  //共有メモリ
  int8_t msgid;
  uint32_t *addr;
  MP.Recv(&msgid, &addr);
  share_addr = &addr[0];
  MP.Send(msgid, addr);
}

void loop()
{
  int khz = 38;
  int      i;
  int      ret;
  int8_t   rcvid;
  uint32_t rcvdata;

  //***************************************************//
  //受信待機
  //***************************************************//
  MP.RecvTimeout(MP_RECV_BLOCKING);
  MP.Recv(&rcvid, &rcvdata);

  //IR送信
  if (rcvdata == 0){
    //冷房ON信号の送信
    irsend.sendRaw(signal_cool, sizeof(signal_cool) / sizeof(signal_cool[0]), khz);
    MPLog("send cool\n");
    MP.Send(rcvid, 1);

  } else if (rcvdata == 1){
    //暖房ON信号の送信
    irsend.sendRaw(signal_heat, sizeof(signal_heat) / sizeof(signal_heat[0]), khz);
    MPLog("send heat\n");
    MP.Send(rcvid, 1);

  } else if (rcvdata == 2){
    //停止信号の送信
    irsend.sendRaw(signal_off, sizeof(signal_off) / sizeof(signal_off[0]), khz);
    MPLog("send stop\n");
    MP.Send(rcvid, 1);

  } else if (rcvdata == 3) {
    //冷房信号の登録
    for (i = 0; i < MSG_SIZE; i++) {
      signal_cool[i] = share_addr[i];
    }
    //共有メモリの初期化
    for (i = 0; i < MSG_SIZE; i++) {// 共有メモリを初期化
      share_addr[i] = 0;
    }
    MPLog("cool set complete!!");
    MP.Send(rcvid, 1);

  } else if (rcvdata == 4) {
    //暖房信号の登録
    for (i = 0; i < MSG_SIZE; i++) {
      signal_heat[i] = share_addr[i];
    }
    //共有メモリの初期化
    for (i = 0; i < MSG_SIZE; i++) {// 共有メモリを初期化
      share_addr[i] = 0;
    }
    MPLog("heat set complete!!");
    MP.Send(rcvid, 1);

  } else if (rcvdata == 5) {
    //停止信号の登録
    for (i = 0; i < MSG_SIZE; i++) {
      signal_off[i] = share_addr[i];
    }
    //共有メモリの初期化
    for (i = 0; i < MSG_SIZE; i++) {// 共有メモリを初期化
      share_addr[i] = 0;
    }
    MPLog("off set complete!!");
    MP.Send(rcvid, 1);

  }

}
