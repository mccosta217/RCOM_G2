/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyS1"
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

/*typedef struct sm 
{
  int state;
};*/

int main(int argc, char** argv)
{

    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[5], bufua[5];
    int i, sum = 0, speed = 0;
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

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
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

 
    unsigned char BCC;

    BCC = 0x03^0x08;

    buf[0] = 0x5c;
    buf[1] = 0x01;
    buf[2] = 0x08;
    buf[3] = BCC;
    buf[4] = 0x5c;

    /*testing*/
    //buf[25] = '\n';

    res = write(fd,buf,5);
    printf("%d bytes written\n", res);

    
   // struct sm ua_rec;

    unsigned char BCCrec, bufrec[5];
    int send;

    int state = Start;

while(state != 7)
{
    switch (state)
    {
      case Start:
        res = read(fd, buf, 1);
        buf[res]=0;
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
        if (buf[0] == 0x03)
        {
          bufrec[1] = 0x03;
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
            printf("0x%02x\n", bufrec[i]);
        }
        bufrec[2] = 0x06;
        bufrec[3] = 0x01^0x06;
        /*send = write(fd, bufrec, 5);
        printf("%d bytes written\n", send);*/
        state = 7;
        break;
    }
  }    
    
    
    
    /*int ua;
    ua = read(fd, bufua, 5);
    bufua[ua] = 0;

    unsigned char BCCua;
    BCCua = bufua[1]^bufua[2];

    if (BCCua == bufua[3])
    {
        for (int i = 0; i<5;i++)
        {
            printf("0x%x\n", bufua[i]);
        }
    }

    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */
    sleep(1);

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
