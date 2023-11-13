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
bool bStartBit = false;
void waiting_for_start_bit() {
  bStartBit = true;
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

  static char bit_array[BIT_BUFF_SIZE] = {0};
  static char byte_array[BYTE_BUFF_SIZE] = {0};
  static char output_data[BYTE_BUFF_SIZE] = {0};
  
  // Hardware Interrupt Handling (StartBit detection)
  if (bStartBit) {
    if (interrupt_debug) digitalWrite(MONITOR_PIN, HIGH);
    detachInterrupt(RECV_PIN);

    uint32_t interval;
    static bool bPowerOn = true; // the counter measure for changing the interrupt timing. I don't know why but the first interrupt takes 800usec. 
    if (bPowerOn) {  
      interval = FETCH_INTERVAL;
      bPowerOn = false;
    } else {
      interval = FETCH_INTERVAL*1.5-100;  // 100 microseconds is the processing time for detachInterrupt and attachTimerInterrupt
    }
    attachTimerInterrupt(read_recv_pin, interval);      
    if (interrupt_debug) digitalWrite(MONITOR_PIN, LOW);   
    bStartBit = false;
    return;
  }

  // Timer Interrupt Handling (To fetch bits)
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
        // printf("[%d] %c\n", data_counter, byte_value);

        /* store the data to byte_array */
        byte_array[data_counter++] = byte_value;

        // the below condition may not reach but put this for a fail-safe.
        if (data_counter >= BYTE_BUFF_SIZE) {
          printf("byte_array overflow\n");
          data_counter = 0; 
        }

        /* waiting for the next header */
        static char *header = NULL;
        static bool bByteRecording = false;
        if (data_counter > PH1 && byte_array[data_counter-2] == 'U' &&  byte_array[data_counter-1] == 'Z') {
          header = &byte_array[data_counter-2];
          bByteRecording = true;
        }

        static int packet_counter = 2; // 'U','Z'
        if (bByteRecording) {
          ++packet_counter;
          if (packet_counter > HEADER_SIZE) {

            /* get the payload size */
            uint16_t length = header[PSZ];

            if (packet_counter > HEADER_SIZE + length) {

              /* get the checksum data */
              uint16_t csum = (header[CS1] << 8) | header[CS0];
              /* check the data with the sachecksum */
              uint16_t bcc = calc_crc(&header[PH0], length);
              if (bcc == csum) {

                for (int i = 0; i < length; ++i) {
                  output_data[i] = header[HEADER_SIZE + i];
                }

                // output the data
                printf("%s\n", &output_data[0]);

              } else {
                // to do: error handling
                printf("Checksum Error: 0x%04X 0x%04X\n",  bcc, csum);
              }

              /* the message has been processed. clear the buffer */
              data_counter = 0;
              packet_counter = 2;
              header = NULL;
              bByteRecording = false;
              memset(byte_array, 0, BYTE_BUFF_SIZE); 
            }
          }
        }
      } else {
        /* stop bit (bit_array[8]) is not 1. need an error handling. */
        printf("stop bit error\n");
      } 
      bit_counter = 0;
      memset(bit_array, 0, BIT_BUFF_SIZE);
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

