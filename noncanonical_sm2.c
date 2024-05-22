/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include "linklayer2.h"

#define BAUDRATE B9600
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

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

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];
    int state = Start;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS10", argv[1])!=0) &&
          (strcmp("/dev/ttyS11", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    memset(&newtio, 0 ,sizeof(newtio)); 

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set (READ)\n");
        
    //struct sm set_rec;

    unsigned char BCCrec, bufrec[5], BCC;
    int send;

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

        if (buf[0] == 0x07)
        {
          bufrec[2] = 0x07;
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
        printf("SET received\n");
        bufrec[2] = 0x06;
        bufrec[3] = 0x01^0x06;
        send = write(fd, bufrec, 5);
        printf("%d bytes written to SP (UA)\n", send);
        printf("-----------------------------------------------------------------------\n");
        state = State_stop;
        break;
    }
}
    unsigned char bufrec2[255]; //bufrec2 vai receber dados
    llread(fd, bufrec2);

    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
