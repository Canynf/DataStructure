#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/un.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <termios.h>

pthread_t m_thread;//通讯线程标识符ID
int m_ExitThreadFlag;//退出数据接收线程标志
int m_fd;
int m_DatLen;//串口数据接收长度
char DatBuf[500];//串口数据接收Buffer

typedef struct port_info {
	int baud_rate;
	int port_fd;
	char parity;
	char stop_bit;
	char flow_ctrl;
	char data_bits;
}*pport_info;

int get_baud_rate(unsigned long int baud_rate);
int set_port(pport_info p_info);


int OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity, int rt);
int ClosePort();//关闭串口并释放相关资源
int WritePort(const char* Buf, int len);//向串口写数据
void Flush();
int ReceiveThreadFunc(void* lparam);// 串口数据接收线程

int main(int argc, char *argv[])//argc用来存放命令行参数的个数，argv指针数组
{
	int  	ret;
	int     portno,	baudRate;//串口号，波特率
	char    Buf[10], BufC[10];
	int i1;
	
	
	// 解析命令行参数：串口号   波特率
	if( argc > 1 )
	{
		portno = atoi( argv[1] );//将字符数组转换成一个整数并返回结果，如果不能转换成 int 或者为空字符串，那么将返回 0
	}
	else
	{
		portno = 2;
	}
	if( argc > 2 )
	{
		baudRate = atoi( argv[2]);//波特率
	}
	else
	{
		baudRate = 9600;
	}

	printf( "port:%d baudrate:%d\n", portno, baudRate);
	
	
	ret = OpenPort(portno, baudRate, '8', '1', 'N',0);//根据输入串口参数打开串口，并创建串口数据接收线程
	if(ret<0 )
	{
		printf( "serial open fail\n");
		return -1;
	}

	Buf[0] = 0x31;
	Buf[1] = 0x32;
	Buf[2] = 0x33;
	Buf[3] = 0x34;
	Buf[4] = 0x35;
	Buf[5] = 0x36;
	Buf[6] = 0x37;
	Buf[7] = 0x38;
	Buf[8] = 0x39;
	
	
	BufC[0] = 'A';
	BufC[1] = 'B';
	BufC[2] = 'C';
	BufC[3] = 'D';
	BufC[4] = 'E';
	BufC[5] = 'F';
	BufC[6] = 'G';
	BufC[7] = 'H';
	BufC[8] = 'I';
	


	//进入主循环，这里每隔1s输出一个提示信息
	/*for( i1=0; i1<10000;i1++)
	{
		sleep(2);
		//printf( "%d \n", i1+1);
		if(-1==WritePort( argv[3], 9 ))
				printf("write error\n");
		
	}*/
	printf("Input:");
	char ch[128] = {'\0'};
	while(1)
	{
		
		scanf("%s", ch);
		
		if(-1==WritePort( ch, strlen(ch) ))
				printf("write error\n");
		
		memset(ch, 0,128);//用来对一段内存空间全部设置为某个字符，一般用在对定义的字符串进行初始化，ch为数组，将0赋给ch，128为数组的长度
		tcflush(1, TCIFLUSH);//清空输入、输出缓冲区，TCIFLUSH 　刷新收到的数据但是不读
		sleep(1);
	}
	//ClosePort( );
	return 0;
}


int OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity, int rt)//串口号，波特率，数据位，停止位，奇偶性
{
	char				portname[20];
	struct port_info 	info;
	pthread_attr_t 		attr;
	int					res;

	sprintf( portname, "/dev/ttyUSB%d", PortNo );
	if((m_fd = open(portname,O_RDWR | O_NOCTTY | O_NONBLOCK))==-1)	//O_RDWR 读写模式，若所有欲核查的权限都通过了检查则返回0，表示成功，只要有一个权限被禁止则返回-1
	{
		perror("Cannot open the desired port");
		return -1;
	}

    //
    // Fill in the device control block.
    //
 	// 修改相关参数
	info.baud_rate = baudrate;
	info.parity = parity;
	info.data_bits = databits;
 	info.flow_ctrl = '0';
	info.port_fd = m_fd;
	info.stop_bit = stopbits;

	if(set_port(&info)==-1 )
	{
		printf( "set port fail\n");
		return -1;
	}


	// 创建串口的接收线程
	res = pthread_attr_init(&attr);//对线程属性初始化
	if( res!=0 )
	{
		printf("Create attribute failed\n" );
	}
	// 设置线程绑定属性
	res = pthread_attr_setscope( &attr, PTHREAD_SCOPE_SYSTEM );//PTHREAD_SCOPE_SYSTEM，表示与系统中所有线程一起竞争CPU时间
	// 设置线程分离属性
	res += pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );//在分离属性下，一个线程结束时立即释放所占用的系统资源
	if( res!=0 )
	{
		printf( "Setting attribute failed\n" );
	}
	res = pthread_create( &m_thread, &attr, (void *(*) (void *))&ReceiveThreadFunc, NULL );//创建一个新的线程，并将线程加入当前进程，&m_thread线程标识符;&attr:设置线程属性;
                                                                                               //start_routine:线程运行函数的起始地址;arg：运行函数的参数
	if( res!=0 )
	{
		close( m_fd );
		return -1;
	}

	pthread_attr_destroy( &attr );//销毁一个目标结构，并且使它在重新初始化之前不能重新使用
	return 0;
}

int ReceiveThreadFunc(void* lparam)
{
	//定义读事件集合
	fd_set fdRead;
	int ret;
	struct timeval	aTime;

	while( 1 )
	{
        //收到退出事件，结束线程
		if(m_ExitThreadFlag )
		{
			break;
		}

		FD_ZERO(&fdRead);//将fdRead的所有位置零（在对文件描述符集合进行设置前，必须对其进行初始化）
		FD_SET(m_fd,&fdRead);//用于在文件描述符集合中增加一个新的文件描述符

		aTime.tv_sec = 0;
		aTime.tv_usec = 300000;

		ret = select( m_fd+1,&fdRead,NULL,NULL,&aTime );//程序会停在select这里等待，直到被监视的文件句柄有一个或多个发生了状态改变， m_fd是需要监视的最大的文件描述符值，
                                                                //fdRead为需要检测的可读文件描述符的集合，&aTime用于描述一段时间长度，如果在这个时间内，需要监视的描述符没有事件发生则函数返回，返回值为0
		//printf( "select ret = %d\n", ret);

		if (ret < 0 )
		{
			//关闭串口
			ClosePort( );
			break;
		}

		if (ret > 0)
		{
			//判断是否读事件
			if (FD_ISSET(m_fd,&fdRead))//满足可读条件
			{
				//data available, so get it!
				m_DatLen = read( m_fd, DatBuf, 100 );// m_fd为文件描述符，用来指向要操作的文件的文件结构体，DatBuf指一块内存空间，100为希望读取的字节数，返回值表示实际读到的字节数
				// 对接收的数据进行处理，
				if( m_DatLen > 0 )
				{
					printf("Read：   %s\n", DatBuf);
					Flush();
				}
				// 处理完毕
			}
		}
	}

	printf( "ReceiveThreadFunc finished\n");
	pthread_exit( NULL );
	return 0;
}


int get_baud_rate(unsigned long int baud_rate)
{
	switch (baud_rate) {
	case 0:
		return B0;
	case 50:
		return B50;
	case 75:
		return B75;
	case 110:
		return B110;
	case 134:
		return B134;
	case 150:
		return B150;
	case 200:
		return B200;
	case 300:
		return B300;
	case 600:
		return B600;
	case 1200:
		return B1200;
	case 1800:
		return B1800;
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	default:
		return -1;
	}
}

int set_port(pport_info p_info)
{
	struct termios new_opt;
	int baud_rate;
	int status;

	//get the current config -> new_opt
	tcgetattr(p_info->port_fd, &new_opt);
	bzero(&new_opt, sizeof(new_opt));

	//convert baud rate -> baud_flag
	baud_rate = get_baud_rate(p_info->baud_rate);

	tcflush(p_info->port_fd, TCIOFLUSH);
	//setup input/output baudrate
	cfsetispeed(&new_opt, baud_rate);
	cfsetospeed(&new_opt, baud_rate);
	status = tcsetattr(p_info->port_fd, TCSANOW, &new_opt);
	if (status != 0)
	{
		perror("tcsetattr::set baud rate failed\n");
		return -1;
	}

	//修改控制模式，保证程序不会占用串口
	new_opt.c_cflag |= CLOCAL;

	//修改控制模式，使得能够从串口读取输入数据
	new_opt.c_cflag |= CREAD;

	new_opt.c_cflag |= HUPCL;
	//setup control flow
	switch (p_info->flow_ctrl)
	{
	case '0':
		//no control-flow
		new_opt.c_cflag &= ~CRTSCTS;
		break;
	case '1':
		//hardware control-flow
		new_opt.c_cflag |= CRTSCTS;
		break;
	case '2':
		new_opt.c_iflag |= IXON | IXOFF | IXANY;
		break;
	}
	//printf("c_cflag(no ctl-flow) = %x\r\n", new_opt.c_cflag);

	//setup bit size
	new_opt.c_cflag &= ~CSIZE;
	switch (p_info->data_bits)
	{
	case '5':
		new_opt.c_cflag |= CS5;
		break;
	case '6':
		new_opt.c_cflag |= CS6;
		break;
	case '7':
		new_opt.c_cflag |= CS7;
		break;
	case '8':
		new_opt.c_cflag |= CS8;
		break;
	default:
		new_opt.c_cflag |= CS8;
	}
	//printf("c_cflag |= CS8 => %x\r\n", new_opt.c_cflag);

	//setup parity
	switch (p_info->parity)
	{
	case 'n':
	case 'N':
		new_opt.c_cflag &= ~PARENB; /* Clear parity enable */
		new_opt.c_iflag &= ~INPCK; /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		new_opt.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		new_opt.c_iflag |= INPCK; /* Disable parity checking */
		break;
	case 'e':
	case 'E':
		new_opt.c_cflag |= PARENB; /* Enable parity */
		new_opt.c_cflag &= ~PARODD; /* 转换为偶效验*/
		new_opt.c_iflag |= INPCK; /* Disable parity checking */
		break;
	case 'S':
	case 's': /*as no parity*/
		new_opt.c_cflag &= ~PARENB;
		new_opt.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported parity\n");
		return -1;
	}
	//printf("c_cflag &=~PARENB => %x\r\n", new_opt.c_cflag);

	//setup stop-bit
	if (p_info->stop_bit == '2') {
		new_opt.c_cflag |= CSTOPB;
	} else {
		new_opt.c_cflag &= ~CSTOPB;
	}
	//printf("c_cflag &=~CSTOPB => %x\r\n", new_opt.c_cflag);

	/* Set input parity option */
	if ((p_info->parity != 'n') || (p_info->parity != 'N'))
	{
		new_opt.c_iflag |= INPCK;
	}

	//修改输出模式：原始数据输出(raw 模式)
	new_opt.c_lflag &= ~(ICANON | ECHO | ISIG); /*Input*/
	new_opt.c_oflag &= ~OPOST; /*Output*/

	//修改控制字符：读取字符的最少个数为1
	new_opt.c_cc[VMIN] = 1;

	//修改控制字符：读取第一个字符的超时时间为1×100ms
	new_opt.c_cc[VTIME] = 1;

	//试图去掉在接收时必须收到'\n'才返回的问题
	//忽略输入的回车
	//new_opt.c_iflag |= IGNCR;
	//new_opt.c_iflag &= ~(IXON|IXOFF|IXANY);

	//如果发生数据溢出，接收数据，但是不再读取
	tcflush(p_info->port_fd, TCIFLUSH);

	status = tcsetattr(p_info->port_fd, TCSANOW, &new_opt);
	if (status != 0)
	{
		perror("Cannot set the serial port parameters");
		return -1;
	}

	return status;
}

int WritePort(const char *buf, int len)
{
	if (m_fd == 0) return -1;
	int sendlen = 0;
	sendlen = write(m_fd, buf, len);//m_fd是文件描述符（write所对应的是写，即就是1），buf是一个需要写入的字符串，len是每次写入的字节数

	if (sendlen == len)
	{
		return sendlen;
	}
	else
	{
		//如果出现溢出情况
		tcflush(m_fd, TCOFLUSH);//用于清空输入，输出缓冲区，m_fd为打开的串口描述符，TCOFLUSH为刷新写入的数据但是不传送，也就是把输入缓冲区清空
		return -1;
	}
}

void Flush()
{
	tcflush(m_fd, TCIFLUSH);//TCIFLUSH为刷新收到的数据但是不读，也就是将输出缓冲器清空
}

int ClosePort()
{
	m_ExitThreadFlag = 1;
	close(m_fd);
	printf("close serial port\n");
	return 0;
}
