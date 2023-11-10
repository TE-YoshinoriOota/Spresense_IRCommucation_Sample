#define RECV_PIN  (21)
#define MONITOR_PIN  (20)

#define FETCH_INTERVAL (1600)

#define BIT_BUFF_SIZE (16)
#define BYTE_BUFF_SIZE (512)

#define PH0  (0)
#define PH1  (1)
#define PSZ  (2)
#define CS0  (3)
#define CS1  (4)
#define CS_SIZE (2)
#define HEADER_SIZE (CS1+1)

const bool interrupt_debug = false;

/* Timer interrupt handler */
bool bFetch = false;
unsigned int  read_recv_pin() {
  bFetch = true;
  return FETCH_INTERVAL;
}

/* Hardware interrupt handler */
bool bPowerOn = true;
void waiting_for_start_bit() {
  // Note: it takes 800msec from voltage falling to firing software interrupt
  if (interrupt_debug) digitalWrite(MONITOR_PIN, HIGH);
  detachInterrupt(RECV_PIN);
  uint32_t interval;
  if (bPowerOn) {  // the counter measure for changing the interrupt timing. I don't know why but the first interrupt takes 800usec. 
    interval = FETCH_INTERVAL;
    bPowerOn = false;
  } else {
    interval = FETCH_INTERVAL*1.5-100;  // 100 microseconds is the processing time for detachInterrupt and attachTimerInterrupt
  }
  attachTimerInterrupt(read_recv_pin, interval);      
  if (interrupt_debug) digitalWrite(MONITOR_PIN, LOW);
}


void setup() {
  pinMode(RECV_PIN, INPUT_PULLUP);
  if (interrupt_debug) pinMode(MONITOR_PIN, OUTPUT);
  attachInterrupt(RECV_PIN, waiting_for_start_bit, FALLING);
}


void loop() {

  static bool bPinSetting = false;
  static int bit_counter = 0;
  static int data_counter = 0;
  static int last_byte_value = 0;

  static char bit_array[BIT_BUFF_SIZE] = {0};
  static char byte_array[BYTE_BUFF_SIZE] = {0};
  static char output_data[BYTE_BUFF_SIZE] = {0};

  if (bFetch) {
    bFetch = false;

    if (interrupt_debug) digitalWrite(MONITOR_PIN, HIGH);
    if (!bPinSetting) pinMode(RECV_PIN, INPUT_PULLUP);

    bit_array[bit_counter++] = digitalRead(RECV_PIN);

    /* all bit are stored in the bit array */
    if (bit_counter == 9) {

      /* check the stop bit */
      if (bit_array[8] == 1) {
        /* construct byte data */
        int byte_value = 0;
        for (int n = 0; n < 8; ++n) {
          byte_value |= bit_array[n] << n;
        }
        //printf("0x%02X\n", byte_value);

        /* store the data to byte_array */
        byte_array[data_counter++] = byte_value;

        // the below condition may not reach but put this for a fail-safe.
        if (data_counter >= BYTE_BUFF_SIZE) {
          printf("byte_array overflow\n");
          data_counter = 0; 
        }

        /* waiting for the next header */
        if (data_counter > 2 && byte_array[data_counter-2] == 'U' &&  byte_array[data_counter-1] == 'Z') {
    
          /* check if the last bytes have already been stored */
          if (data_counter > HEADER_SIZE && byte_array[PH0] == 'U' && byte_array[PH1] == 'Z') {
            delayMicroseconds(100); // countermeasure: timing adjustment

            /* get the payload size */
            uint16_t length = byte_array[PSZ];

            /* get the checksum data */
            uint16_t csum = (byte_array[CS1] << 8) | byte_array[CS0];

            /* check the data with the sachecksum */
            uint16_t bcc = calc_crc(&byte_array[0], length);
            if (bcc == csum) {
              for (int n = 0; n < length; ++n) {
                output_data[n] = byte_array[n + HEADER_SIZE];
              }

              // output the data
              printf("%s\n", &output_data[0]);

            } else {
              // to do: error handling
              printf("Checksum Error: 0x%04X 0x%04X\n",  bcc, csum);
            }

            /* the message has been processed. clear the buffer */
            data_counter = 0;
            memset(byte_array, 0, BYTE_BUFF_SIZE); 

            /* restore the current header */
            byte_array[data_counter++] = last_byte_value; // 'U'
            byte_array[data_counter++] = byte_value;  // 'Z'
          }
        }
        last_byte_value = byte_value;
      } else {
        /* stop bit (bit_array[8]) is not 1. need an error handling. */
        printf("stop bit error\n");
      } 
      bit_counter = 0;
      bPinSetting = false;
      detachTimerInterrupt();
      attachInterrupt(RECV_PIN, waiting_for_start_bit, FALLING); // switch the hardware interrupt to wait the start bit.
    }
    digitalWrite(MONITOR_PIN, LOW);
  }

}


uint16_t calc_crc(char *msg_ptr, uint16_t data_length) {
  uint16_t crc = 0x0000;
  uint16_t header_length = HEADER_SIZE - CS_SIZE; // without checksum_data
  char *data_ptr = msg_ptr + HEADER_SIZE;
  for (uint16_t i = 0; i < header_length; i++) crc ^= *(msg_ptr++);
  for (uint16_t i = 0; i < data_length;   i++) crc ^= *(data_ptr++);
  return crc;
}

