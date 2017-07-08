#include <eRCaGuy_Timer2_Counter.h>

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

#define CHANNEL_COUNT     6
#define CHANNEL_OFFSET    4

#define PULSE_DEFAULT     1000
#define PULSE_MIN_DOUBLE  1500
#define PULSE_MAX_DOUBLE  4500

#define PACKET_SIZE       25

uint32_t  rise[CHANNEL_COUNT], fall[CHANNEL_COUNT], pulse;
uint16_t  channel[CHANNEL_COUNT],
          genOne[CHANNEL_COUNT], genTwo[CHANNEL_COUNT], genThree[CHANNEL_COUNT], genFour;
uint8_t   packet[PACKET_SIZE], i;

void interrupt() {

  if (arduinoPinState) {
    rise[arduinoInterruptedPin - CHANNEL_OFFSET] = timer2.get_count();

  } else {
    fall[arduinoInterruptedPin - CHANNEL_OFFSET] = timer2.get_count();
  }

}

void setup() {

  timer2.setup();

  Serial.begin(100000, SERIAL_8E2);

  for (i = 0; i < CHANNEL_COUNT; i++) {
    channel[i] = genOne[i] = genTwo[i] = genThree[i] = PULSE_DEFAULT;
    rise[i] = fall[i] = 0;
  }

  packet[0] = 0x0F;
  for (i = 1; i < PACKET_SIZE; i++) {
    packet[i] = 0;
  }

  for (i = CHANNEL_OFFSET; i < CHANNEL_OFFSET + CHANNEL_COUNT; i++) {
    pinMode(i, INPUT);
    enableInterrupt(i, interrupt, CHANGE);
  }

}

void loop() {

  for (i = 0; i < CHANNEL_COUNT; i++) {

    pulse = fall[i] - rise[i];

    if (pulse < PULSE_MIN_DOUBLE) {
      pulse = PULSE_MIN_DOUBLE;
    
    } else if (pulse > PULSE_MAX_DOUBLE) {
      pulse = PULSE_MAX_DOUBLE;
    }

    // genFour = ((uint16_t) (pulse >> 1) - 880) * 8 / 5;
    genFour = (uint16_t) ((pulse << 2) / 5 - 1408);
    
    channel[i] = (genOne[i] + genTwo[i] + genThree[i] + genFour) >> 2;
    
    genOne[i]   = genTwo[i];
    genTwo[i]   = genThree[i];
    genThree[i] = genFour;
  }

  packet[1] = (uint8_t) (channel[0]);
  packet[2] = (uint8_t) (channel[0] >> 8  | channel[1] << 3);
  packet[3] = (uint8_t) (channel[1] >> 5  | channel[2] << 6);
  packet[4] = (uint8_t) (channel[2] >> 2);
  packet[5] = (uint8_t) (channel[2] >> 10 | channel[3] << 1);
  packet[6] = (uint8_t) (channel[3] >> 7  | channel[4] << 4);
  packet[7] = (uint8_t) (channel[4] >> 4  | channel[5] << 7);
  packet[8] = (uint8_t) (channel[5] >> 1);
  packet[9] = (uint8_t) (channel[5] >> 9);

  //  packet[9]  = (uint8_t) (channel[5]  >> 9   | channel[6]  << 2);
  //  packet[10] = (uint8_t) (channel[6]  >> 6   | channel[7]  << 5);
  //  packet[11] = (uint8_t) (channel[7]  >> 3);
  //  packet[12] = (uint8_t) (channel[8]);
  //  packet[13] = (uint8_t) (channel[8]  >> 8   | channel[9]  << 3);
  //  packet[14] = (uint8_t) (channel[9]  >> 5   | channel[10] << 6);
  //  packet[15] = (uint8_t) (channel[10] >> 2);
  //  packet[16] = (uint8_t) (channel[10] >> 10  | channel[11] << 1);
  //  packet[17] = (uint8_t) (channel[11] >> 7   | channel[12] << 4);
  //  packet[18] = (uint8_t) (channel[12] >> 4   | channel[13] << 7);
  //  packet[19] = (uint8_t) (channel[13] >> 1);
  //  packet[20] = (uint8_t) (channel[13] >> 9   | channel[14] << 2);
  //  packet[21] = (uint8_t) (channel[14] >> 6   | channel[15] << 5);
  //  packet[22] = (uint8_t) (channel[15] >> 3);

  Serial.write(packet, PACKET_SIZE);

  delay(11);

}
