# Spresense_IRCommucation_Sample
This is a sample IR communication program for the Sony Spresense Board.  

# The electrical characteristics
## TX Signal
This protocol is based on UART. A data bit is expressed by IR blinking on a 40 kHz carrier wave. Please note that sending the IR carrier wave is "0" and the non-active is "1". 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/6256c05b-0c80-4fa7-b88e-65bb8e3edf78" width="700" />

The TX system may use any IR LED, but you could refer to [a datasheet of the IR LED made by Vishay Semiconductors](https://www.vishay.com/docs/81011/tsal6400.pdf). The example circuit is shown below for a  reference. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/a6f67425-6e45-46a9-9f07-9e3d420bb90b" width="700" />

## RX Signal
The IR diode can only receive IR blinking on a carrier wave. The below figure shows the relationship between the TX wave coming from the TX system and the output signal of the IR diode. You can choose the frequency though, I decided on an IR diode having 40 kHz sensitivity. 

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/b393e8c7-4f30-4758-85f3-5a44821b0295" width="700" />

The R system may use any IR diode, but you could refer to [a datasheet of the IR diode made by Vishay Semiconductors](https://www.vishay.com/docs/82459/tsop48.pdf) The example circuit is shown below for a reference.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/28683a5d-7d7f-4f8e-9c06-fe052083b184" width="700" />


# Protocol
## Sending a byte
The protocol is based on UART 8N1. The start bit is 0 and the stop bit is 1. The byte data consists of 8 bits. The bit length of the signal is 1600 (TBD) microseconds.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/a4f1a5d2-c826-4fe4-9187-3200a3d7ed2a" width="700" />

## Payload structure
The message has a header to recognize the top of the dataset. The first byte is 'U' and the second byte is 'Z'. The third byte is the length of the data payload. The fourth and fifth bytes consist of the checksum of the message without the checksum value.

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/b04015c5-4d41-4f34-ae4b-11b6c4212e97" width="700" />


| Symbol | Value |
|--------|-------|
| PH0 | 'U' |
| PH1 | 'Z' |
| PSZ | Payload Size |
| CS0 | Checksum LSB |
| CS1 | Checksum SB |
| Dn  | Data Content |

# Software structure
## The TX software
The TX system uses PWM0 of D06 pin of Spresense to output 40kHz carrier wave stably. The bit "0" or the start bit starts PWM0 for 1600 microseconds and the bit "1" or the stop bit stops PWM0 for 1600 microseconds.

```
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

  // end bit
  ioctl(fd, PWMIOC_STOP, 0);
  delayMicroseconds(INTERVAL*2);  // STOP_BITx2
}
```



The checksum is calculated by exclusive OR using the message header and the data payload.

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

<img src="https://github.com/TE-YoshinoriOota/Spresense_IRCommucation_Sample/assets/14106176/836f6b23-0046-41da-ae4a-4f870ec7f117" width="700" />



