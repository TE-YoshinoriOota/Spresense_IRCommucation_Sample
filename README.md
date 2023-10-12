# Spresense_IRCommucation_Sample
This is a sample program of IR  communication for Spresense Board. This sample consists of the TX routine and RX routine.

# The electrical characteristics
## TX Signal
This protocol is based on UART. A data bit is expressed by IR blinking on a 40 kHz carrier wave. Please note that the active that is sending the IR signal is "0" and the non-active is "1". 

The TX system may use any IR LED, but you could refer to [a datasheet of the IR LED made by Vishay Semiconductors](https://www.vishay.com/docs/81011/tsal6400.pdf). The example circuit is as follows for your reference.


## RX Signal
The IR diode only can receive IR blinking on a carrier wave. The below figure shows the relationship between the TX wave coming from the TX system and the output signal of the IR diode. You can choose the frequency though, I chose an IR diode having 40 kHz sensitivity. 


The R system may use any IR diode, but you could refer to [a datasheet of the IR diode made by Vishay Semiconductors](https://www.vishay.com/docs/82489/tsop322.pdf) The example circuit is as follows for your reference.


# Protocol
## Sending a byte
The protocol is based on UART 8N1. The start bit is 0 and the stop bit is 1. The byte data consists of 8 bits. The bit length of the signal is 1600 (TBD) microseconds.

## Payload structure
The message has a header to recognize the top of the dataset. The first byte is 'U' and the second byte is 'Z'. The third byte is the length of the data payload. The fourth and fifth bytes consist of the checksum of the message without the checksum value.


# Software structure
## TX system
The TX system uses PWM0 of D06 pin of Spresense to output 40kHz carrier wave stably. The bit "0" or the start bit starts PWM0 for 1600 microseconds and the bit "1" or the stop bit stops PWM0 for 1600 microseconds.

The checksum is calculated by exclusive OR using the message header and the data payload.


## RX system.
The RX system waits for the start bit by monitoring the GPIO pin like D20 for example. Once the monitored pin is falling the signal by the start bit, a hardware interrupt occurs that starts a timer interrupt of 1600 microseconds. 

