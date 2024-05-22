#ifndef LINKLAYER
#define LINKLAYER

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define Start 0
#define Flag_rec 1
#define A_rec 2
#define C_rec 3
#define BCC_rec 4
#define Stop 5
#define ESCAPE 6
#define Dados 7
#define State_stop 9
#define UA_wait 10
#define UA_A 11
#define UA_C 12
#define UA_BCC 13
#define BCC2_state 14


unsigned char tx = 0;
unsigned char rx = 1;

int fd;


typedef struct linkLayer{
    char serialPort[50];
    int role; //defines the role of the program: 0==Transmitter, 1=inf_rec
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;

//ROLE
#define NOT_DEFINED -1
#define TRANSMITTER 0
#define RECEIVER 1
//#define inf_rec 1


//SIZE of maximum acceptable payload; maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

//CONNECTION deafault values
#define BAUDRATE_DEFAULT B38400
#define MAX_RETRANSMISSIONS_DEFAULT 3
#define TIMEOUT_DEFAULT 4
#define _POSIX_SOURCE 1 /* POSIX compliant source */

//MISC
#define FALSE 0
#define TRUE 1

//bool alarm_flag; 
int alarm_counter = 0;
int alarm_on = 0;

/*void alarm_handler(int signal)
{
  printf("Alarm handler\n");
  //alarm_flag = true;
  alarm_counter--;
  alarm_on = 1;
}*/

void send_supervision(int fd, unsigned char A, unsigned char C){
  int i;
  unsigned char frame[5] = {0x5c, A, C, A ^ C, 0x5c};
  i = write(fd, frame,5);
  if(i <= 0){
    printf("Erro a enviar Supervision Frame\n");
  }else{
    printf("Supervision frame enviada\n");
  }

  for (int e = 0; e < 5; e++) {
    printf("0x%02x\n", frame[e]);
    }
}

// Opens a connection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer connectionParameters)
{
 // int alarm_on = 1;
 // int retries = connectionParameters.numTries;
 // int timeout = connectionParameters.timeOut;
  int role = connectionParameters.role;
 // alarm_counter = retries;

  int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror(connectionParameters.serialPort);
    return -1;
  }
  struct termios oldtio, newtio;

  if (tcgetattr(fd, &oldtio) == -1)
  { /* save current port settings */
    perror("tcgetattr");
    return -1;
  }

  memset(&newtio, 0, sizeof(newtio));

  newtio.c_cflag = BAUDRATE_DEFAULT | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &newtio) == -1)
  {
    perror("tcsetattr");
    return -1;
  }

  if (role == TRANSMITTER)
  {
    unsigned char buf[5];
    unsigned char BCC;
    BCC = 0x01 ^ 0x07;

    buf[0] = 0x5c;
    buf[1] = 0x01;
    buf[2] = 0x07;
    buf[3] = BCC;
    buf[4] = 0x5c;
    
        write(fd, buf, 5);
        printf("%d bytes written to SP (SET)\n", 5);

    unsigned char BCCrec, bufrec[5];
    int state = Start, res;

   // while(alarm_counter > 0)
   // {
    /*  if (alarm_on)
      {
        (void) signal(SIGALRM, alarm_handler); //definimos que depois do timeout é suposto ir para o alarm_handler
        write(fd, buf, 5);
        printf("%d bytes written to SP (SET)\n", 5);
        alarm(timeout);
        alarm_on = 0;
     }*/

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
            return fd;
            state = State_stop;
            break;
        }
      }
    }
    //printf("Connection failed, limit reached\n");
    //alarm(0);
  

  else if(role == rx)
  {
    unsigned char buf[5];
    unsigned char BCCrec, bufrec[5];
    int state = Start, res, send;

    while(state != State_stop)
    {
      switch (state)
      {
        case Start:
          //printf("Start_Read\n");
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
          //printf("Flag_Rec\n");
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
          //printf("A_Rec\n");
          res = read(fd, buf, 1);

          if (buf[0] == 0x07)
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
          //printf("C_Rec\n");
          res = read(fd, buf, 1);

          if (buf[0] == (0x01^0x07))
          {
            bufrec[3] = (bufrec[1]^bufrec[2]);
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
          //printf("BCC_Rec\n");
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
          //printf("Stop\n");
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
    return fd;
  }
  return -1;
}

int S_type; 

// Sends data in infwith size infSize
int llwrite(unsigned char* buf, int bufSize) //buf só tem dados
{
  if(bufSize == 1){
    if(buf[0] == 0x00) //acabar envio
      write(fd, buf, 1);
      printf("ACABAR ENVIO AQUI\n");
   return 1;   
  }

  int a;
  int buff_stuffed = bufSize; //variável criada para calcular o tamanho total do buffer depois do stuffing
  int I_type = 0;
  printf("llwrite: BufSize: %d\n", bufSize);
  printf("buf[0]: %d\n", buf[0]);

  if(buf == NULL || bufSize == 0){
    fprintf(stderr, "Invalid data or size\n");
    return -1;
  }  

  for (int i = 0; i < bufSize; i++)
  {
    if (buf[i] == 0x5c || buf[i] == 0x5d)
    {
      buff_stuffed++;
    }
  }

  char buf2[buff_stuffed + 6]; // flag + A + C + BCC1 + BCC2 + flag + D(sizeof(buf)) = 6 + buff_stuffed

  memset(buf2, 0, (buff_stuffed + 6));
  
  buf2[0] = 0x5c;
  buf2[1] = 0x01;

  if(I_type == 0){ 
    buf2[2] = 0x80; //10000000
  }else{
    buf2[2] = 0xC0; //11000000
  }
  
  char BCC1 = buf2[1] ^ buf2[2];
  buf2[3] = BCC1;
 
  memcpy(buf2+4, buf, bufSize); //F A C BCC1 DADOS     falta BCC2 e F
  printf("BUF2[4]/////////////////////////////////: %d\n", buf2[4]);

//XOR dos dados todos (buf)
  unsigned char BCC2 = buf[0];

  for (int i = 1; i < bufSize; i++)
  {
    BCC2 = BCC2 ^ buf[i];
  }

buf2[4 + buff_stuffed] = BCC2;
buf2[5 + buff_stuffed] = 0x5c;


//bit stuffing e colocar os dados no buf2
  int j = 4; //dados começam em buf2[4]
  for (int i = 0; i < bufSize; i++)
  {
    if (buf[i] == 0x5c)
    {
      buf2[j] = 0x5d;
      buf2[j + 1] = 0x7c;
      j = j + 2;
    }
    else if (buf[i] == 0x5d)
    {
      buf2[j] = 0x5d;
      buf2[j + 1] = 0x7d;
      j = j + 2;
    }
    else
    {
      //if(i >= MAX_PAYLOAD_SIZE){
      buf2[j] = buf[i];
      j++;
      //break;
      
      //}
    }
  }

  printf("j: %d\n",j);
  //for(int l=0; l < buff_stuffed + 6; l++){
  //printf("buf2[%d]: 0x%02x\n", l,buf2[l]);
  //}


  a = write(fd, buf2, (buff_stuffed + 6)); //buf2 com bit stuffing (INFORMATION FRAME ENVIADA PARA A SP)
  if(a < 0){
    printf("Erro ao enviar Informatio Frame\n");
  }else{
    printf("Information Frame enviada\n");
  }
  
  I_type = 1 - I_type; 


/////////////////// SM de receção da SUPERVISION frame //////////////////
unsigned char superv[255];
int state = Start;
unsigned char superv_rec[255];
unsigned char resposta;
int res;
while(state != State_stop)
{
    switch (state)  
    {
      case Start:
        //printf("Start_Read\n");
        res = read(fd, superv, 1);
        printf("res: 0x%02x\n",superv[0]);

        if (superv[0] == 0x5c)
        {
          superv_rec[0] = 0x5c;
          state = Flag_rec;
        }
        else
        {
          state = Start;
        }
        break;
      
      case Flag_rec: //problema aqui (loop)
        printf("Flag_Rec\n");
        res = read(fd, superv, 1);
        printf("res: 0x%02x\n",superv[0]);

        if (superv[0] == 0x01)
        {
          superv_rec[1] = 0x01;
          state = A_rec;
        }
        else if(superv[0]== 0x5c)
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
        res = read(fd, superv, 1);  //RR=1 -> 0x11  OU RR=0 -> 0x01
                                    //REJ=1 -> 0x15 OU REJ=0 -> 0x05
        printf("C recebido: 0x%02x\n", superv[0]);
        if (superv[0] == 0x01 || superv[0] == 0x11) 
        {
            resposta = superv[0];
            superv_rec[2] = superv[0];
            printf("ACEITE(RR)\n");
            state = C_rec;
            break;

          //O QUE ACONTECE?
        }
        else if(superv[0] = 0x15 || superv[0] == 0x05){ //Se C = REJ
          resposta = superv[0];
          printf("REJEITADA (REJ)\n");
          state = C_rec;
        }
        else
        {
          state = Start;
        }
        break;
      
      case C_rec:
        printf("C_Rec\n");
        res = read(fd, superv, 1);
        printf("BCC: 0x%02x\n", superv[0]);
        printf("A ^ C = 0x%02x\n",superv_rec[1]^superv_rec[2]);
        if (superv[0] == (superv_rec[1]^superv_rec[2]))
        {
          printf("BCC: 0x%02x\n", superv[0]);
          superv_rec[3] = superv[0];
          state = BCC_rec;
        }
        else if(superv[0] == 0x5c)
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
        res = read(fd, superv, 1);
        printf("res: 0x%02x\n",superv[0]);
        if (superv[0] == 0x5c)
        {
          superv_rec[4] = 0x5c;
          state = Stop;
        }
        else
        {
          state = Start;
        }
        break; 
      
      case Stop:
        printf("Stop\n");
        for (int i =0; i<5; i++) //RESPOSTA DO RECEIVER (Ex: RR)
        {     
            printf("%d, 0x%02x\n", i, superv_rec[i]);
        }

        printf("RR/REJ received\n");
        
        printf("-----------------------------------------------------------------------\n");
        state = State_stop;
        break;
    }
sleep(1);
}
return a;
}

int reenv = 0;

int llread(unsigned char* packet) {
    int state2 = Start;
    unsigned char inf;
    //int S_type; 
    unsigned char inf_rec[5];
    unsigned char sup_send[5];
    unsigned char valorI;
    int packet_size = sizeof(packet);
    int bytes_read = 0;
    int j = 0;
    unsigned char C, A;

    if(packet == NULL) {
        printf("Erro ao alocar memória\n");
        return -1;
    }
    printf("Inicio LLREAD\n");
    while (state2 != State_stop) {
      //printf("Dentro do while\n");
        switch (state2) {
            case Start: //state 0
              
                printf("Dentro do switch (START)\n");
                read(fd, &inf, 1);
                if(inf == 0x00){ //acabou de ler
                  printf("ACABOU DE LER O FICHEIRO\n");
                  return -1;
                }
                printf("INF: 0x%02x\n",inf);
                if (inf == 0x5c) { //se for flag
                    printf("start entrei na flag\n");
                    inf_rec[0] = 0x5c;
                    state2 = Flag_rec;
                } else if(reenv == 1){
                  state2 = BCC_rec;
                }
                else {
                    state2 = Start;
                }
                break;

            case Flag_rec: //state 1
                read(fd, &inf, 1);
                printf("inf: 0x%02x\n", inf);
                if (inf == 0x01) {
                    inf_rec[1] = 0x01;
                    state2 = A_rec;
                } else if (inf == 0x5c) {
                    state2 = Flag_rec;
                } else {
                    state2 = Start;
                }
                printf("F recebida: 0x%02x\n", inf_rec[0]);
                break;

            case A_rec:
                read(fd, &inf, 1);
                printf("inf: 0x%02x\n", inf);
                if (inf == 0x80) { //I=0 -> 0x80 
                    printf("INF = 0x80\n");
                    inf_rec[2] = inf;
                    S_type = 1;
                    valorI = 0x80;
                    state2 = C_rec;
                } else if (inf == 0xC0) { // I=1 -> 0xC0
                    printf("INF = 0xC0\n");
                    inf_rec[2] = inf;
                    S_type = 0;
                    valorI = 0xC0;
                    state2 = C_rec;
                } else if (inf == 0x5c) {
                    state2 = Flag_rec;
                } else {
                    state2 = Start;
                } 
                printf("A recebido: 0x%02x\n", inf_rec[1]);
                break;

            case C_rec:
                read(fd, &inf, 1);
                printf("inf: 0x%02x\n", inf);
                
                if (inf == (inf_rec[1] ^ inf_rec[2])) {
                    inf_rec[3] = inf;
                    state2 = BCC_rec;
                    printf("BCC certo\n");
                } else if (inf == 0x5c) {
                    state2 = Flag_rec;
                } else {
                    state2 = Start;
                }
                printf("C recebido: 0x%02x\n", inf_rec[2]);
                break;

            case BCC_rec: //DESTUFFING, AQUI SÃO SÓ LIDOS DADOS, BCC2 E FLAG FINAL
                printf("BCC recebido\n");
                while (1) {
                    printf("Tou a ler\n");

                    int res = read(fd, &inf, 1);
                    if (res <= 0) {
                        perror("Erro ao ler o file descriptor");
                        return -1;
                    }

                    if (inf == 0x5d) { //ESCAPE
                        state2 = ESCAPE; 
                        //printf("VIM PARA O ESCAPE NO BCC_rec = 0X5d (0x%02x)\n", inf);   
                        break;
                    }
                    if (inf == 0x5c) { //SE ENCONTRAR FLAG É PORQUE ACABOU
                        state2 = BCC2_state; 

                        break;
                  } else { // colocar dados no packet
                        if (j >= MAX_PAYLOAD_SIZE) {
                            printf("LLREAD: Enviei 1 packet\n");
                            reenv = 1;
                            int bytes_read = j;
                            state2 = BCC2_state;
                            break;
                        }
                        
                          printf("j = %d, dados: 0x%02x\n",j, inf);
                          packet[j] = inf;
                          j++; // j é o current byte
                        
                        }
                }
              break;

            case ESCAPE:
                read(fd, &inf, 1);
                //printf("Entrei no ESCAPE\n");
                //read(fd, &inf, 1);
                if (inf == 0x7c) {
                    packet[j] = 0x5c;
                    j++;
                 
                } else if (inf == 0x7d) {
                    packet[j] = 0x5d;
                    j++;
                   
                }
                state2 = BCC_rec;
                break;
            
            case BCC2_state:
                        printf("BCC2_State\n");
                        unsigned char BCC2 = packet[j-1];
                        packet[j-1] = '\0';
                        j--;

                        unsigned char check = 0;
                        //printf("PACKET[0]: 0x%02x\n", packet[0]);
                        for (int k = 0; k < j; k++) { // Corre os dados todos e faz XOR
                            //printf("check: 0x%02x\n",check);
                            check ^= packet[k];
                            //printf("check: 0x%02x\n",check);
                        }

                        printf("CHECK = 0x%02x e BCC2 = 0x%02x\n", check, BCC2);
                        if (check == BCC2) { // Se BCC2 estiver correto envia RR 
                            state2 = Stop;
                            printf("BCC2 correto\n");

                            if(S_type == 0){
                              C = 0x01;
                              printf("RR(C) = 0x01;\n");
                            }else if(S_type == 1){
                               C = 0x11 ;
                               printf("RR(C)= 0x11;\n");
                            }
                            send_supervision(fd, 0x01, C);
                           // printf("Supervision frame:\n");
                            
                            S_type = 1 - S_type;
                            state2=Stop;

                            printf("Supervision frame enviada (RR)\n");
                        } else { // BCC errado envia REJ
                            if (valorI == 0x80) { // I = 0 -> REJ = 1
                                C = 0x15; // REJ 1 -> 0x15

                            } else if (valorI == 0xC0) { // I = 1 -> REJ = 0
                                C = 0x05; // REJ 0 -> 0x05
                            }

                            send_supervision(fd, 0x01, C);
                            S_type = 1 - S_type;
                            printf("ERRO na transmissão (REJ).\n");
                            state2=Stop;
                        }
              break;  
          
            case Stop:
                printf("\n");
                printf("LLREAD: STOP\n");
                printf("LLREAD: Dados recebidos no packet:\n");
                printf("Quantidade de dados: %d\n", j);
                for (int z = 0; z < j; z++) {
                    printf("%d, 0x%02x\n", z, packet[z]);
                }
                printf("\n");
                state2 = State_stop;
                break;

            default:
                break;
        }
            sleep(1);
        }
    
    
    return j; // retorna o número de bytes lidos
}

// Closes previously opened connection; if showStatistics==TRUE, link layer should print statistics in the console on close
int llclose(linkLayer connectionParameters, int showStatistics)
{
  //alarm_on = 1;
  unsigned char buf[5] = {0x5c, 0x01, 0x0A, 0x01^0x0A, 0x5c};
  unsigned char reg[5];
  int b;
  int state = Start;
  int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror(connectionParameters.serialPort);
    return -1;
  }
  
  if (connectionParameters.role == TRANSMITTER)
  {
      b = write(fd, buf, 5); //Envia DISC
      printf("%d bytes written to SP (DISC)\n", b);

    while (state != State_stop)
    {   
      //Recebe outro DISC
        switch (state)
        {
          case Start:
            read(fd, reg, 1);
            if (reg[0] == 0x5c)
            {
              //printf("Start\n");
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case Flag_rec:
            read(fd, reg, 1);
            //printf("Flag_rec\n");
            if (reg[0] == 0x03)
            {
              state = A_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case A_rec:
            read(fd, reg, 1);
            //printf("A_Rec\n");
            if (reg[0] == 0x0A)
            {
              state = C_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case C_rec:
            read(fd, reg, 1);
            //printf("C_rec\n");
            if (reg[0] == (0x03 ^ 0x0A))
            {
              state = BCC_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case BCC_rec:
            read(fd, reg, 1);
            //printf("BCC_rec\n");
            if (reg[0] == 0x5c)
            {
              state = Stop;
            }
            else
            {
              state = Start;
            }
            break;
          case Stop:
            printf("DISC received\n");
            printf("-----------------------------------------------------------------------\n");
            buf[0] = 0x5c;
            buf[1] = 0x01;
            buf[2] = 0x06;
            buf[3] = buf[1] ^ buf[2];
            buf[4] = 0x5c;
            write(fd, buf, 5);
            printf("%d bytes written to SP (UA)\n", 5);
            close (fd);
            return 0;
          default:
            break;
        }
      }
    }
  

  if (connectionParameters.role == rx)
  {
    while(1)
    {
        switch(state) 
        {
          case Start:
            read(fd, reg, 1);
            //printf("Start\n");
            if (reg[0] == 0x5c)
            { 
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case Flag_rec:
            read(fd, reg, 1);
            //printf("Flag_rec\n");
            if (reg[0] == 0x01)
            {
              state = A_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case A_rec:
            read(fd, reg, 1);
            //printf("A_rec\n");
            if (reg[0] == 0x0A)
            {
              state = C_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case C_rec:
            read(fd, reg, 1);
            //printf("C_rec\n");
            if (reg[0] == (0x01 ^ 0x0A)) //0x03?
            {
              state = BCC_rec;
            }
            else if (reg[0] == 0x5c)
            {
              state = Flag_rec;
            }
            else
            {
              state = Start;
            }
            break;
          case BCC_rec:
            read(fd, reg, 1);
            //printf("BCC_REC\n");
            if (reg[0] == 0x5c)
            {
            int a;
            printf("DISC received\n");
            printf("-----------------------------------------------------------------------\n");
            unsigned char discrec[5] = {0x5c, 0x03, 0x0A, 0x03^0x0A, 0x5c};
            a = write(fd, discrec, 5); //Envia DISC para o transmissor
          
            if(a <= 0){
              printf("Erro, DISC nao enviado para o transmissor\n");
            }else{
              printf("DISC foi enviado para o transmissor\n");
            }
              state = UA_wait;
            }
            else
            {
              state = Start;
            }
            break;
          case UA_wait:
             // Espera pelo UA do transmissor
             read(fd, reg, 1);
            if(reg[0] == 0x5c) state = UA_A;
            break;

          case UA_A:
            read(fd, reg, 1);;
            if(reg[0] == 0x01) state = UA_C;
            break;
          case UA_C:
            read(fd, reg, 1);
            if(reg[0] == 0x06) state = UA_BCC;
            break;
          case UA_BCC:
            read(fd, reg, 1);
            if(reg[0] == 0x01 ^ 0x06){
              printf("Recetor: UA recebido\n");
              printf("A fechar conexão\n");
              close(fd);
              return 0;

            } 
            break;
        
          default:
            break;
                
        }
      }
    }    
        
  }



#endif
