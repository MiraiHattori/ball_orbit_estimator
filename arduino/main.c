/*  タイマ0 : 8bit  : 0~255
  タイマ1 : 16bit : 0~65535

  1MHzで動作すると仮定して, 1クロック 0.000001sec
  なので、
    タイマ1 : 0.000001*65536 = 65.5ms　までOK
*/

/*  高速PWMモード
  ICR1 : 最大値
  OCR1A (データシートのpin配置ではOC1A) : 真ん中

    |
   ICR1  |-----------------------
    |    /＼
    |   /   ＼         /
  OCR1A|------＼------/-----------
    | /         ＼   /
    |/            ＼/

  出力波形(PB1)
    |
    |--________________--___
    |

  CTCモード : 最大値はICR1
  通常モード : 最大値はOCR1A
*/

/* タイマ1をつかう */
#include <avr/io.h>
#include <stdio.h>

int main(void)
{
  // Hz
  int pointgrey_trigger_hz = 20;
  // 分周
  int clock_division;

  //制御レジスタA
  TCCR1A = 0b10000010;  // 10:コンペアマッチAでLOW,10:高速PWM動作
  if (4 <= pointgrey_trigger_hz && pointgrey_trigger_hz < 31) {
      clock_division = 64;
      //制御レジスタB
      TCCR1B = 0b00011011;  // 11:高速PWM動作, 011:分周1/64 <-- もともと16MHzなので，これで0.25MHz
  } else if (31 <= pointgrey_trigger_hz && pointgrey_trigger_hz < 245) {
      clock_division = 8;
      //制御レジスタB
      TCCR1B = 0b00011010;  // 11:高速PWM動作, 010:分周1/8 <-- もともと16MHzなので，これで2MHz
  } else if (pointgrey_trigger_hz <= 245) {
      clock_division = 1;
      //制御レジスタB
      TCCR1B = 0b00011000;  // 11:高速PWM動作, 000:分周1/1 <-- もともと16MHzなので，これで16MHz
  } else {
      printf("FrameRate %d is not valid.\n", pointgrey_trigger_hz);
      return -1;
  }

  //最大値 (16bit?<65535)
  if (16000000 / clock_division % pointgrey_trigger_hz == 0) {
      ICR1 = 16000000 / clock_division / pointgrey_trigger_hz - 1;
  } else {
      printf("FrameRate %d is not dividable.\n", pointgrey_trigger_hz);
      return -1;
  }

  // HIGHの時間(クロック数)
  OCR1A = 16000000 * 0.5 / clock_division / pointgrey_trigger_hz - 1;


  // ICR1 = 20000;  // High時間0.01s
  // 2MHzのクロックで40000カウントしたら立ち上がる 2MHz/40000_50Hz
  // OCR1A = 10000;  // 0から数える. High時間 0.005ms <-- duty比を決める(ICR1の半分ならduty比0.5)
  // OCR1AはPB1ピン

  DDRB = 0b11111110;  // portB : 0bit目のみ入力

  PORTB = 0b00000001;  // PB0のプルアップ抵抗を有効に.
  while (1)
  {
  }
  return 0;
}
