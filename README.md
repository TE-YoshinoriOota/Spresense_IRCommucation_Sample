# Spresense_IRCommucation_Sample
This is a sample IR communication program for the Sony Spresense Board.  

# The electrical characteristics
## TX Signal
This protocol is based on UART. A data bit is expressed by IR blinking on a 40 kHz carrier wave. Please note that sending the IR carrier wave is "0" and the non-active is "1". 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide0.PNG" width="700" />

The TX system may use any IR LED, but you could refer to [a datasheet of the IR LED made by Vishay Semiconductors](https://www.vishay.com/docs/81011/tsal6400.pdf). The example circuit is shown below for a reference. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide1.PNG" width="700" />

## RX Signal
The IR diode can only receive IR blinking on a carrier wave. The below figure shows the relationship between the TX wave coming from the TX system and the output signal of the IR diode. You can choose the frequency though, I decided on an IR diode having 40 kHz sensitivity. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide2.PNG" width="700" />

The R system may use any IR diode, but you could refer to [a datasheet of the IR diode made by Vishay Semiconductors](https://www.vishay.com/docs/82459/tsop48.pdf) The example circuit is shown below for a reference.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide3.PNG" width="700" />


# Protocol
## Sending a byte
The protocol is based on UART 8N1. The start bit is 0 and the stop bit is 1. The byte data consists of 8 bits. The bit length of the signal is 1600 (TBD) microseconds.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide4.PNG" width="700" />

## Payload structure
The message has a header to recognize the top of the dataset. The first byte is 'U' and the second byte is 'Z'. The third byte is the length of the data payload. The fourth and fifth bytes consist of the checksum of the message without the checksum value.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide5.PNG" width="700" />


| Symbol | Value |
|--------|-------|
| PH0 | 'U' |
| PH1 | 'Z' |
| PSZ | Content Size |
| CS0 | Checksum LSB |
| CS1 | Checksum SB |
| Dn  | Data Contents |

# Software structure
## The TX software
### ■ setup payload and PWM function
In the setup function, initialize the payload and the pwm. The PWM0 of D06 pin of Spresense is used for output 40kHz carrier wave stably. 
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
The sendChar function disassembles the byte data to send the bit on the IR transmission. Please note that the bit "0" starts PWM0 for 1600 microseconds and the bit1 or the stop bit stops PWM0 for 1600 microseconds.

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
The RX system waits for the start bit by monitoring the GPIO pin like D20 for example. Once the monitored pin is falling the signal by the start bit, a hardware interrupt occurs, starting a timer interrupt of 1600 microseconds. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/blob/main/resources/slide6.PNG" width="700" />

### ■ setup interrupt handlers
In the setup function, the hardware interrupt is set to detect falling the voltage. For the interrupt handling, there has been a problem just after the power-on. The expectation is that the interrupt should be just after dropping the voltage, but the problem is that the interrupt is delayed just after the power-on. So, I put the  countermeasure in "waiting_for_start_bit" that is the interrupt routine. In the interrupt routine, the hardware interrupt is detached and switched to the timer interrupt to fetch the bits after the start bit detected.

```
/* Timer interrupt Handler */
bool bFetch = false;
unsigned int  read_recv_pin() {
  bFetch = true;
  return FETCH_INTERVAL;
}

/* Hardware interrupt Handler */
bool bPowerOn = true;
void waiting_for_start_bit() {
  // Note: it takes 800msec from voltage falling to firing software interrupt
  detachInterrupt(RECV_PIN); /* detach the hardware interrupt */
  uint32_t interval;
  if (bPowerOn) {  // the countermeasure for changing the interrupt timing. I don't know why but the first interrupt takes 800usec. 
    interval = FETCH_INTERVAL;
    bPowerOn = false;
  } else {
    interval = FETCH_INTERVAL*1.5-100;  // 100 microseconds is the processing time for detachInterrupt and attachTimerInterrupt
  }
  attachTimerInterrupt(read_recv_pin, interval);  /* switch to the timer interrupt */   
}

void setup() {
  pinMode(RECV_PIN, INPUT_PULLUP);
  attachInterrupt(RECV_PIN, waiting_for_start_bit, FALLING);
}
```

### ■ decode bit data to byte data
In the loop function, store each bit and convert it into byte data. When "bFetch" turns into "true", read a bit of data from RECV_PIN and store the value in "bit_array". When the stop bit is detected, the bit data compiles to byte data. Then, the byte data is stored in "byte_array". All byte data has been read that is specified by "PSZ", then the data is copied to "output_data".

```
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

        /* store the data to byte_array */
        byte_array[data_counter++] = byte_value;

        /* waiting for the next header */
        if (data_counter > 2 && byte_array[data_counter-2] == 'U' &&  byte_array[data_counter-1] == 'Z') {
    
          /* check if the last bytes have already been stored */
          if (data_counter > HEADER_SIZE && byte_array[PH0] == 'U' && byte_array[PH1] == 'Z') {

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

              // put a user function here to pass the decoded data to a user application.
              // put just the "print out" function here for the tentative
              printf("%s\n", &output_data[0]);

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
      } 
      bit_counter = 0;
      bPinSetting = false;
      detachTimerInterrupt(); 
      attachInterrupt(RECV_PIN, waiting_for_start_bit, FALLING); // switch the hardware interrupt to wait the start bit.
    }
  }
}
```
