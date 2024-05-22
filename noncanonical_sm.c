/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

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
#define State_stop 7

/*typedef struct sm 
{
  int state;
};*/

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

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

    printf("New termios structure set\n");

//    while (STOP==FALSE) {       /* loop for input */
        
    //struct sm set_rec;

    unsigned char BCCrec, bufrec[5], BCC;
    int send;
    
    int state = Start;

while(state != State_stop)
{
    switch (state)
    {
      case Start:
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
        res = read(fd, buf, 1);

        if (buf[0] == 0x08)
        {
          bufrec[2] = 0x08;
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
        for (int i =0; i<5; i++)
        {     
            printf("%d, 0x%02x\n", i, bufrec[i]);
        }
        bufrec[2] = 0x06;
        bufrec[3] = 0x03^0x06;
        send = write(fd, bufrec, 5);
        printf("%d bytes written\n", send);
        state = State_stop;
        break;
    }
}

  int state2 = Start;
  unsigned char bufrec2[26];

  while (state2 != State_stop)
  {
    switch (state2)
    {
    case Start:
      read(fd, buf, 1);
      if (buf[0] == 0x5c)
      {
        bufrec[0] = 0x5c;
        state2 = Flag_rec;
      }
      else
      {
        state2 = Start;
      }
      break;
    
    case Flag_rec:
      read(fd, buf, 1);
      printf("Flag recebida: 0x%02x\n", bufrec[1]);
      if (buf[0] == 0x01)
      {
        bufrec[1] = 0x01;
        state2 = A_rec;
      }
      else if (buf[0] == 0x5c)
      {
        state2 = Flag_rec;
      }
      else
      {
        state2 = Start;
      }
      break;
    
    case A_rec:
      read(fd, buf, 1);
      printf("A recebido: 0x%02x\n", bufrec[2]);
      if (buf[0] == 0x00)
      {
        bufrec[2] = 0x00;
        state2 = C_rec;
      }
      else if (buf[0] == 0x5c)
      {
        state2 = Flag_rec;
      }
      else
      {
        state2 = Start;
      } // implementar os diferentes tipos de C recebidos
      break;
    
    case C_rec:
      read(fd, buf, 1);
      printf("C recebido: 0x%02x\n", bufrec[3]);
      if (buf[0] == (bufrec[1]^bufrec[2]))
      {
        bufrec[3] = buf[0];
        state2 = BCC_rec;
      }
      else if (buf[0] == 0x5c)
      {
        state2 = Flag_rec;
      }
      else
      {
        state2 = Start;
      }
      break;
    
    case BCC_rec:
      while (int i = 0; i<22; i++)
      {
        read(fd, buf, 1);
        int j = 0;
        if (buf[0] == 0x5c)
        {
          unsigned char BCC2 = bufrec2[j-1];
          bufrec2[j-1]=0;
          j--;

          unsigned char check = bufrec2[0];

          for (int i = 1; i < j; i++)
          {
            check ^= bufrec2[i];
          }

          if (check == BCC2)
          {
            state2 = Stop;
          }
          else
          {
            printf("isto não é suposto acontecer para já\n");
          }
        }
        else
        {
          bufrec2[j] = buf[0];
          j++;
        }
      }
      break;
      
    default:
      break;
    }

    
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
