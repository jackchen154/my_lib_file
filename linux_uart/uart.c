#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义exit()*/
#include<unistd.h>     /*Unix 标准函数定义read(),write()*/
//#include<sys/types.h> 
//#include<sys/stat.h>   
#include<fcntl.h>      /*文件控制定义open(),fcntl()*/
#include<termios.h>    /*PPSIX 终端控制定义*/
//#include<errno.h>      /*错误号定义*/
//#include<string.h>
#define uchar unsigned char 
#define FALSE  -1
#define TRUE   0
#define Time_out 5 //串口超时时间设置


/*******************************************************************
* 名称：UART0_Open
* 功能：打开串口并返回串口设备文件描述
* 入口参数：fd:文件描述符
          port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：正确返回为1，错误返回为0
*******************************************************************/
int UART0_Open(int fd,const char* port)
{
  fd = open( port, O_RDWR|O_NDELAY|O_NOCTTY);//
  if (fd < 0)
  {
    printf("Can not open:%s\n",port);
    perror("Error");
    return(FALSE);
  }
  printf("%s:open success!\n",port);

  //恢复串口为阻塞状态                               
  if(fcntl(fd, F_SETFL, 0) < 0)
  {
     printf("set block failed!\n");
     return(FALSE);
  }     
            
  return fd;
}

/*******************************************************************
* 名称：UART0_Close
* 功能：关闭串口并返回串口设备文件描述
* 入口参数：fd:文件描述符     
* 出口参数：void
*******************************************************************/
void UART0_Close(int fd)
{
    close(fd);
}
 


/*******************************************************************
* 名称：UART0_Init
* 功能：设置串口数据位，停止位和效验位
* 入口参数：fd:串口文件描述符
*         speed:串口速度
*         flow_ctrl:数据流控制
*         databits:数据位(取值为5,6,7，8)
*         stopbits:停止位(值为1或者2)
*         parity:效验类型(取值为N=无校验位,E=偶校验,O=奇校验,S=空格)
          例子：UART0_Init(fd,0,115200,'N',8,1);
*出口参数：正确返回为1，错误返回为0
*******************************************************************/
int UART0_Init(int fd,int flow_ctrl,int speed,int parity,int databits,int stopbits)
{
  unsigned int   i;
  uchar speed_sta;
  int   speed_arr[] = { B115200,B57600,B38400,B19200, B9600, B4800, B2400, B1200, B300};
  int   name_arr[] =  {  115200, 57600, 38400, 19200,  9600,  4800,  2400,  1200,  300};
         
  struct termios options; 
   /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。
     若调用成功，函数返回值为0，若调用失败，函数返回值为1.
   */
   if(tcgetattr(fd,&options)!=0)
   {
      perror("SetupSerial 1");    
      return(FALSE); 
   }
   printf("UART_init:");
   //设置串口输入波特率和输出波特率
   for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
   {
      if(speed == name_arr[i])
      {             
         cfsetispeed(&options, speed_arr[i]); 
         cfsetospeed(&options, speed_arr[i]); 
         printf("%d-",name_arr[i]);
         speed_sta=1;
	 break; 
      }
   }     
   if(speed_sta==0)
   {
     printf("\nSpeed value[%d] is wrong!\n",speed);
     return (FALSE); 
   }     
   
   //修改控制模式，保证程序不会占用串口
   options.c_cflag |= CLOCAL;
   //修改控制模式，使得能够从串口中读取输入数据
   options.c_cflag |= CREAD;
  
   //设置数据流控制
   switch(flow_ctrl)
   {      
       case 0 ://不使用流控制
              options.c_cflag &= ~CRTSCTS;
              break;        
       case 1 ://使用硬件流控制
              options.c_cflag |= CRTSCTS;
              break;
       case 2 ://使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;
   }

    //设置校验位
    switch (parity)
    {  
       case 'n':
       case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB; 
                 options.c_iflag &= ~INPCK; 
                 printf("N-");   
                 break; 
       case 'o':  
       case 'O'://设置为奇校验    
                 options.c_cflag |= (PARODD | PARENB); 
                 options.c_iflag |= INPCK;
                 printf("O-");             
                 break; 
       case 'e': 
       case 'E'://设置为偶校验  
                 options.c_cflag |= PARENB;       
                 options.c_cflag &= ~PARODD;       
                 options.c_iflag |= INPCK;
                 printf("E-");       
                 break;
       case 's':
       case 'S': //设置为空格 
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 printf("S-"); 
                 break; 
        default:  
                 printf("\nParity bit[%c] is not support!\n",parity);    
                 return (FALSE); 
    } 
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {  
       case 5    :
                     options.c_cflag |= CS5;
                     printf("5-");
                     break;
       case 6    :
                     options.c_cflag |= CS6;
                     printf("6-");
                     break;
       case 7    :    
                     options.c_cflag |= CS7;
                     printf("7-");
                     break;
       case 8:    
                     options.c_cflag |= CS8;
                     printf("8-");
                     break;  
       default:   
                 printf("\nData bit[%d] is not support\n",databits);
                 return (FALSE); 
    }

    // 设置停止位 
    switch (stopbits)
    {  
       case 1:   
                 options.c_cflag &= ~CSTOPB;                  
                 printf("1\n");
                 break; 
       case 2:   
                 options.c_cflag |= CSTOPB; 
                 printf("2\n");
                 break; 
      
       default:   
                printf("\nStop bit[%d] is not support\n",stopbits); 
                return (FALSE);
    }
   
  //修改输出模式，原始数据输出
  options.c_oflag &= ~OPOST;
  
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
  //options.c_lflag &= ~(ISIG | ICANON);
   
    //设置等待时间和最小接收字符
    //options.c_cc[VTIME] = 50; /* 读取一个字符等待1*(1/10)s */  
    //options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */
   
    //刷清收发缓存区内的数据
    tcflush(fd,TCIFLUSH);
   
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)  
    {
       perror("com set error!\n");  
       return (FALSE); 
    }
    return (TRUE); 
}

/*******************************************************************
* 名称：UART0_Recv
* 功能：串口接收(带超时检测功能)，接收的数据保存在输入所指的数组
* 入口参数：  fd:串口文件描述符   
            *rcv_buf:接收到数据后存放的位置
            data_len：需要接收的个数
                     
* 出口返回： >0:实际接收到的个数
           -1：接收超时(接收超时时间为Time_out这个宏定义)
*******************************************************************/
int UART0_Recv(int fd, uchar *rcv_buf,int data_len)
{
    int len,fs_sele;
    struct timeval time;
    
    fd_set fs_read;  
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
   
    time.tv_sec = Time_out;
    time.tv_usec = 0;
   
    
    fs_sele = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sele)
       {
              len = read(fd,rcv_buf,data_len);
	      //printf("len = %d fs_sele = %d\n",len,fs_sele);
              tcflush( fd, TCIFLUSH );
              return len;
       }
       else
       {
	      printf("time out!\n");
              return FALSE;
       }     
}

/*******************************************************************
* 名称：IR_learn
* 功能：IR按键学习和学习码值发送
* 入口参数：  fd:串口文件描述符   
            mode:0为学习，1为学习码发送
            num:学习码值0x00~0x64,共可以设置101个值
                     
* 出口参数： 0：设置成功
           1：设置失败
           2：接收超时(接收超时时间为5s)
           3:串口发送失败
           4：参数错误
*******************************************************************/
int IR_learn(int fd,uchar mode,uchar num)//模式设置
{
  uchar learn[]={0x00,0x00,0x00,0x00,0x00};
  uchar receive_buf=0x00,len,i;
  if(mode==0)learn[0]=0x88;
  if(mode==1)learn[0]=0x86;
  if(num>=0&&num<=100)//参数判断
  {
    learn[1]=num;
    learn[4]=learn[0]^learn[1]^learn[2]^learn[3];
    printf("send learn data is:");  
    for(i=0;i<5;i++)
    printf("%x ",learn[i]);
    printf("  ");

    len = write(fd,learn,5);
    if (len == 5 ) 
    {
       printf("send sucess!\n");
       //return len;
    }
    else   
    {               
      tcflush(fd,TCOFLUSH);
      return 3;
    } 
 
    if(UART0_Recv(fd,&receive_buf,1)<0)
    {
       printf("接收超时！请检查模块是否连接正常。\n");
       return 2;
    }   
    else
    {
      if(receive_buf==0x89)
      { 
        printf("learn成功\n"); 
        return 0;
      }
      else if(receive_buf==0xe0)
      { 
        printf("learn失败\n"); 
        return 1;
      }
      else printf("模块数据错误!\n");
    }
   }
   else
   printf("学习码值参数错误！\n");
   
   return 4;
}

/*******************************************************************
* 名称：IR红外模式设置
* 功能：将IR的模式和参数通过串口发送
* 入口参数：  fd:串口文件描述符   
            mode:设置的模式
            value：设置的参数
                     
* 出口参数： 0：设置成功
           1：设置失败
           2：接收超时(接收超时时间为5s)
           3:串口发送失败
*******************************************************************/
int IR_control(int fd,uchar mode,uchar value)//模式设置
{
  uchar control[]={0x00,0x00,0x08,0x08,0x00};
  uchar receive_buf=0x00,len,i;
  control[0]=mode;
  control[1]=value;
  control[4]=control[0]^control[1]^control[2]^control[3];
  printf("send data is:");
  
  for(i=0;i<5;i++)
  printf("%x ",control[i]);
  printf("  ");
  len = write(fd,control,5);
  if (len == 5 ) 
  {
     printf("send sucess!\n");
     //return len;
  }
  else   
  {               
    tcflush(fd,TCOFLUSH);
    return 3;
  }
 
    if(UART0_Recv(fd,&receive_buf,1)<0)
    {
       printf("接收超时！请检查模块是否连接正常。\n");
       return 2;
    }   
    else
    {
      if(receive_buf==0x89)
      { 
        printf("模式设置成功\n"); 
        return 0;
      }
      if(receive_buf==0xe0)
      { 
        printf("模式设置失败\n"); 
        return 1;
      }
    }
    
    printf("模块数据错误!\n");
    return 4;   

}

 
 
//*
int main(int argc, char **argv)
{
    int fd=0; //文件描述符
    fd = UART0_Open(fd,"/dev/ttyS1"); //打开串口，返回文件描述符
    if(fd<0)
    { 
      printf("open port fail!\n");
      exit(1);
    }

    if(UART0_Init(fd,0,38400,'N',8,1)<0)
    {
       printf("UART_Init fail!\n");
       exit(1); 
    }

     while (1) //循环读取数据
     {  
       IR_control(fd,0x06,0x1e);
       //IR_learn(fd,1,10);	
       //sleep(1);             
      }
     UART0_Close(fd);
}//*/



