# Spresense_IRCommucation_Sample
This is a sample IR communication program for the Sony Spresense Board.  

# The electrical characteristics
## TX Signal
This protocol is based on UART. A data bit is expressed by IR blinking on a 40 kHz carrier wave. From the RX system point of view, the signal on the IR carrier wave is "0" and the non-active is "1". 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide0.PNG" width="700" />

The TX system may use any IR LED, but you could refer to [a datasheet of the IR LED made by Vishay Semiconductors](https://www.vishay.com/docs/81011/tsal6400.pdf). The example circuit is shown below for a reference. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide1.PNG" width="700" />

## RX Signal
The IR diode for the RX system can only receive IR blinking on the carrier wave. The below figure shows the output signal of the RX system. As you can see, receiving the carrier wave is "0" and no-signal is "1". You can choose any frequency of the carrier wave though, it depends on the IR diode characteristics. I decided to use an IR diode having 40 kHz sensitivity. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide2.PNG" width="700" />

You could refer to [a datasheet of the IR diode made by Vishay Semiconductors](https://www.vishay.com/docs/82459/tsop48.pdf) for a good IR diode device. The example circuit is shown below for a reference.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide3.PNG" width="700" />


# Protocol
## Sending a byte
The protocol is based on UART 8N1. The start bit is "0" and the stop bit is "1". The byte data consists of 8 bits. The bit length of the signal is 1600 (TBD) microseconds.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide4.PNG" width="700" />

## Payload structure
The message has a header to recognize the top of the dataset. The first byte is 'U' and the second byte is 'Z'. The third byte is the length of the data payload. The fourth and fifth bytes consist of the checksum of the message without the checksum value itself.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide6.PNG" width="700" />


| Symbol | Value |
|--------|-------|
| PH0 | 'U' |
| PH1 | 'Z' |
| PSZ | Content Size |
| CS0 | Checksum LSB |
| CS1 | Checksum MSB |
| Dn  | Data Contents |

# Software structure
## The TX software
### ■ setup payload and PWM function
In the setup function, set up the payload and initialize the PWM system. The PWM0 of D06 pin on Spresense Extension Board can be used for output 40kHz carrier wave stably. 
```
void setup() {

  /* setup message header */
  message[PH0] = 'U';
  message[PH1] = 'Z';
  message[PSZ] = data_length;

  /* set the payload data that starts from 'a' to 'z' */
  char c = 'a';
  for (int n = 0; n < data_length; ++n) {
    message[n + HEADER_SIZE] = c++;
  }

  /* calc and set the checksum of the message */
  uint16_t csum = calc_crc(&message[0], data_length);
  message[CS0] = (uint8_t)(csum & 0x00ff);
  message[CS1] = (uint8_t)((csum & 0xff00) >> 8);
  printf("Checksum: 0x%04X\n", csum);
  printf("Checksum: 0x%02X 0x%02X \n", message[CS1], message[CS0]);
  /* Open the PWM device for reading */
  fd = open(pwm_devpath, O_RDONLY);
  if (fd < 0) {
    printf("error: failed to open pwm0 device\n");
    return;
  }

  /* Configure the characteristics of the pulse train */
  info.frequency = frequency;
  info.duty = duty;

  ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
}
```
### ■ send the payload in the loop function
The sendChar function disassembles the byte data to send a bit on the IR transmission. For sending the bit "0",  the PWM0 runs for 1600 microseconds, for sending the bit "1" the PWM0 stops for 1600 microseconds.

```
void loop() {
  for (char n = 0; n < msg_length; ++n) {
    sendChar(message[n]);
  }
  sleep(1);
}

void sendChar(const char c) {
  // start bit
  ioctl(fd, PWMIOC_START, 0);
  delayMicroseconds(INTERVAL); // 0

  for (int n = 0; n < 8; ++n) {
    uint8_t bit = (c >> n) & 0x01;
    if (bit == 1) {
      // send bit 1
      ioctl(fd, PWMIOC_STOP, 0);
      delayMicroseconds(INTERVAL);       
    } else if (bit == 0) {
      // send bit 0
      ioctl(fd, PWMIOC_START, 0);
      delayMicroseconds(INTERVAL);  
    } else {
      MPLog("Fatal Error! %d\n", bit);
    }
  }

  // stop bit
  ioctl(fd, PWMIOC_STOP, 0);
  delayMicroseconds(INTERVAL*2);  // STOP_BITx2
}
```

### ■ checksum function
The checksum is calculated by exclusive OR using the message header and the data contents.

```
uint16_t calc_crc(char *msg_ptr, uint16_t data_length) {
  uint16_t crc = 0x0000;
  uint16_t header_length = HEADER_SIZE - CS_SIZE; // without checksum_data
  char *data_ptr = msg_ptr + HEADER_SIZE;
  for (uint16_t i = 0; i < header_length; i++) crc ^= *(msg_ptr++);
  for (uint16_t i = 0; i < data_length;   i++) crc ^= *(data_ptr++);
  return crc;
}
```


## The RX software.
The RX system waits for the start bit by monitoring the GPIO pin like D20 for example. Once the monitored pin is falling the signal by a start bit, the hardware interrupt fires for starting the timer interrupt with 1600 microseconds. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide5.PNG" width="700" />

### ■ setup interrupt handlers
In the setup function, the hardware interrupt is set to detect a start bit. The interrupt handler is "waiting_for_sart_bit" which is turning on the bStartBit. When the bStartBit turns true, the timer interrupt starts to fetch each bit of the 8N1. The timer interrupt handler is read_recv_pin which is turning on the bFetch.

```
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
```

### ■ decode bit data to byte data
In the loop function, store each bit and convert it into byte data. When the "bStartBit" turns into "true", set the timer interrupt to fetch each bit. However, there is a problem just after the system bootup. The expectation is that the interrupt should be just after dropping the voltage, but the interrupt is delayed for 800 microseconds just after the power-on. So, I put the bPowerOn flag as the countermeasure for the problem.  Please look into the code for the detail

When the "bFetch" turns into "true" by the timer interrupt, read the value of the RECV_PIN and store it in the "bit_array". When the stop bit is detected, the bit data compiles to the byte data and it is stored in the "byte_array". After all bytes of the data have been read that is specified by the content length of "PSZ", the data is copied to the "output_data" array.

```
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
    static bool bPowerOn = true; // the countermeasure for changing the interrupt timing. I don't know why but the first interrupt takes 800usec. 
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
              /* check the data with the checksum */
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
```
