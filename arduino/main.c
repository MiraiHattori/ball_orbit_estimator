/*  �^�C�}0 : 8bit  : 0~255
  �^�C�}1 : 16bit : 0~65535

  1MHz�œ��삷��Ɖ��肵��, 1�N���b�N 0.000001sec
  �Ȃ̂ŁA
    �^�C�}1 : 0.000001*65536 = 65.5ms�@�܂�OK
*/

/*  ����PWM���[�h
  ICR1 : �ő�l
  OCR1A (�f�[�^�V�[�g��pin�z�u�ł�OC1A) : �^��

    |
   ICR1  |-----------------------
    |    /�_
    |   /   �_         /
  OCR1A|------�_------/-----------
    | /         �_   /
    |/            �_/

  �o�͔g�`(PB1)
    |
    |--________________--___
    |

  CTC���[�h : �ő�l��ICR1
  �ʏ탂�[�h : �ő�l��OCR1A
*/

/* �^�C�}1������ */
#include <avr/io.h>
#include <stdio.h>

int main(void)
{
  // Hz
  int pointgrey_trigger_hz = 20;
  // ����
  int clock_division;

  //���䃌�W�X�^A
  TCCR1A = 0b10000010;  // 10:�R���y�A�}�b�`A��LOW,10:����PWM����
  if (4 <= pointgrey_trigger_hz && pointgrey_trigger_hz < 31) {
      clock_division = 64;
      //���䃌�W�X�^B
      TCCR1B = 0b00011011;  // 11:����PWM����, 011:����1/64 <-- ���Ƃ���16MHz�Ȃ̂ŁC�����0.25MHz
  } else if (31 <= pointgrey_trigger_hz && pointgrey_trigger_hz < 245) {
      clock_division = 8;
      //���䃌�W�X�^B
      TCCR1B = 0b00011010;  // 11:����PWM����, 010:����1/8 <-- ���Ƃ���16MHz�Ȃ̂ŁC�����2MHz
  } else if (pointgrey_trigger_hz <= 245) {
      clock_division = 1;
      //���䃌�W�X�^B
      TCCR1B = 0b00011000;  // 11:����PWM����, 000:����1/1 <-- ���Ƃ���16MHz�Ȃ̂ŁC�����16MHz
  } else {
      printf("FrameRate %d is not valid.\n", pointgrey_trigger_hz);
      return -1;
  }

  //�ő�l (16bit?<65535)
  if (16000000 / clock_division % pointgrey_trigger_hz == 0) {
      ICR1 = 16000000 / clock_division / pointgrey_trigger_hz - 1;
  } else {
      printf("FrameRate %d is not dividable.\n", pointgrey_trigger_hz);
      return -1;
  }

  // HIGH�̎���(�N���b�N��)
  OCR1A = 16000000 * 0.5 / clock_division / pointgrey_trigger_hz - 1;


  // ICR1 = 20000;  // High����0.01s
  // 2MHz�̃N���b�N��40000�J�E���g�����痧���オ�� 2MHz/40000_50Hz
  // OCR1A = 10000;  // 0���琔����. High���� 0.005ms <-- duty������߂�(ICR1�̔����Ȃ�duty��0.5)
  // OCR1A��PB1�s��

  DDRB = 0b11111110;  // portB : 0bit�ڂ̂ݓ���

  PORTB = 0b00000001;  // PB0�̃v���A�b�v��R��L����.
  while (1)
  {
  }
  return 0;
}
