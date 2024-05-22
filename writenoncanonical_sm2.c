/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include "linklayer2.h"

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP = FALSE;

#define Start 0
#define Flag_rec 1
#define A_rec 2
#define C_rec 3
#define BCC_rec 4
#define Stop 5
#define State_stop 9

/*typedef struct sm 
{
  int state;
};*/

int main(int argc, char **argv)
{

  int fd, c, res;
  struct termios oldtio, newtio;
  unsigned char buf[5], bufua[5];
  int i, sum = 0, speed = 0;
  if ((argc < 2) ||
      ((strcmp("/dev/ttyS10", argv[1]) != 0) &&
       (strcmp("/dev/ttyS11", argv[1]) != 0)))
  {
    printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
    exit(1);
  }

  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */

  fd = open(argv[1], O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror(argv[1]);
    exit(-1);
  }

  memset(&newtio, 0 ,sizeof(newtio)); //configurar serial port
  if (tcgetattr(fd, &oldtio) == -1)
  { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 1;  /* blocking read until 5 chars received */

  /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1)
  {
    perror("tcsetattr");
    exit(-1);
  }

  printf("New termios structure set (WRITE)\n");

  unsigned char BCC;

  BCC = 0x01 ^ 0x07;

  buf[0] = 0x5c;
  buf[1] = 0x01;
  buf[2] = 0x07;
  buf[3] = BCC;
  buf[4] = 0x5c;


  res = write(fd, buf, 5); //escreve no fd o que está no buf
  printf("%d bytes written to SP (SET)\n", res);

  // struct sm ua_rec;

  unsigned char BCCrec, bufrec[5];
  int send;
  unsigned char byte;
  int state = Start;

while(state != State_stop)
{
    switch (state)
    {
      case Start:
        printf("Start_Read\n");
        res = read(fd, buf, 1);

        if (buf[0] == 0x5c)
        {
          bufrec[0] = 0x5c;
          state = Flag_rec;
        }
        else
        {
          state = Start;
        }
        break;
      
      case Flag_rec:
        printf("Flag_Rec\n");
        res = read(fd, buf, 1);

        if (buf[0] == 0x01)
        {
          bufrec[1] = 0x01;
          //printf("0x%02x\n", bufrec[1]);
          state = A_rec;
        }
        else if(buf[0]== 0x5c)
        {
          state = Flag_rec;
        }
        else
        {
          state = Start;
        }
        break;
      
      case A_rec: 
        printf("A_Rec\n");
        res = read(fd, buf, 1);

        if (buf[0] == 0x06)
        {
          bufrec[2] = 0x06;
          state = C_rec;
        }
        else if(buf[0] == 0x5c)
        {
          state = Flag_rec;
        }
        else
        {
          state = Start;
        }
        break;
      
      case C_rec:
        printf("C_Rec\n");
        res = read(fd, buf, 1);

        if (buf[0] == (bufrec[1]^bufrec[2]))
        {
          bufrec[3] = buf[0];
          state = BCC_rec;
        }
        else if(buf[0] == 0x5c)
        {
          state = Flag_rec;
        }
        else
        {
          state = Start;
        }
        break;

      case BCC_rec:
        printf("BCC_Rec\n");
        res = read(fd, buf, 1);
        if (buf[0] == 0x5c)
        {
          bufrec[4] = 0x5c;
          state = Stop;
        }
        else
        {
          state = Start;
        }
        break; 
      
      case Stop:
        printf("Stop\n");
        for (int i =0; i<5; i++)
        {     
            printf("%d, 0x%02x\n", i, bufrec[i]);
        }
        printf("UA received\n");
        printf("-----------------------------------------------------------------------\n");
        bufrec[2] = 0x06;
        bufrec[3] = 0x01^0x06;
        state = 7;
        break;
    }
}

unsigned char data[10];

  data[0] = 0x01;
  data[1] = 0x02;
  data[2] = 0x03;
  data[3] = 0x04;
  data[4] = 0x5d; //FLAG
  data[5] = 0x06;
  data[6] = 0x07;
  data[7] = 0x5c;//FLAG
  data[8] = 0x09;
  data[9] = 0x5c; //FLAG

  int dataSize = 10;           //tamanho do buffer definido por nos para já

  llwrite(fd, data, dataSize);

  sleep(1);

  if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
  {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);
  return 0;
}
