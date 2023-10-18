#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>   
#include <nuttx/timers/pwm.h>
#include <MP.h>

static uint32_t frequency = 40000;
static uint16_t duty = 0x7fff; 

char pwm_devpath[] = "/dev/pwm0";
int fd;
struct pwm_info_s info;


#define PH0    (0)
#define PH1    (1)
#define PSZ  (2)
#define CS0  (3)
#define CS1  (4)
#define CS_SIZE (2)
#define HEADER_SIZE (CS1+1)

const int data_length = 26;
const int msg_length = data_length + HEADER_SIZE;
char message[msg_length] = {0};

#define INTERVAL (1600)


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


void loop() {
  for (char n = 0; n < msg_length; ++n) {
    sendChar(message[n]);
    // printf("%c", message[n]);
  }
  // printf("\n");

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

  // end bit
  ioctl(fd, PWMIOC_STOP, 0);
  delayMicroseconds(INTERVAL*2);  // STOP_BITx2

}


uint16_t calc_crc(char *msg_ptr, uint16_t data_length) {
  uint16_t crc = 0x0000;
  uint16_t header_length = HEADER_SIZE - CS_SIZE; // without checksum_data
  char *data_ptr = msg_ptr + HEADER_SIZE;
  for (uint16_t i = 0; i < header_length; i++) crc ^= *(msg_ptr++);
  for (uint16_t i = 0; i < data_length;   i++) crc ^= *(data_ptr++);
  return crc;
}
