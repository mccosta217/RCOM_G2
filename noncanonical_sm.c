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

while(state != 7)
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
          printf("0x%02x\n", bufrec[1]);
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
        state = 7;
        break;
    }
}

/*        res = read(fd,buf,255);    returns after 5 chars have been input 
        buf[res]=0;                so we can printf... 
      
        


      BCCrec = buf[1]^buf[2];  
     
      if (BCCrec == buf[3]) 
      {
        for (int i =0; i<5; i++)
        {     
            printf("0x%x\n", buf[i]);
        }
        BCC = 0x01^0x06;
        bufrec[0] = 0x5c;
        bufrec[1] = 0x01;
        bufrec[2] = 0x06;
        bufrec[3] = BCC;
        bufrec[4] = 0x5c;

        send = write(fd, bufrec, 5);
        printf("%d bytes written\n", send);  
      }
      else
      {
        printf("erro");
      }    
      
    
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */ 
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
