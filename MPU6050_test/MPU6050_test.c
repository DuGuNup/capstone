
#define F_CPU 16000000UL
#define sbi(PORTX,bitX) PORTX|=(1<<bitX)
#define cbi(PORTX,bitX) PORTX&=~(1<<bitX)
#define tbi(PORTX,bitX) PORTX^=(1<<bitX)

#define FS_SEL 131


#include<avr/io.h> 
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>

 
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char addressr);
void USART_Transmit(unsigned char tx_data);
void USART_Transmit_init4(int data);
void get_raw_data();
void calibrate();

volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
volatile double bas_a_x,bas_a_y,bas_a_z;
volatile double bas_g_x,bas_g_y,bas_g_z;
volatile double a_x,a_y,a_z;
volatile double a_x_MAX,a_y_MAX,a_z_MAX;
volatile double a_x_min,a_y_min,a_z_min;
volatile double g_x,g_y,g_z;
volatile double g_x_tmp,g_y_tmp,g_z_tmp;
volatile double g_x_sum,g_y_sum,g_z_sum;
volatile double las_angle_gx,las_angle_gy,las_angle_gz;
volatile double angle_ax,angle_ay,angle_az;
volatile double angle_gx,angle_gy,angle_gz;
volatile double roll,pitch,yaw;
volatile double alpha;

int main()
{  
//**********초기설정 시작**********
	//UART, 
	UCSR1A = 0x00;
	UCSR1B = 0b00011000; 
	UCSR1C = (3<<UCSZ00);  
	UBRR1H = 0;
	UBRR1L = 103;		//9600

	//TWI(I2C)
	TWCR = (1<<TWEN);
	TWBR = 12;	 		//400khz

	//TIMER0
	TCCR0 = (1<<CS02)|(1<<CS01);	//256 분주 
	TCNT0 = 256-125;				//125 번 => 0.002s
	TIMSK = (1<<TOIE0);

	//MPU6050 init
	twi_write(0x6B, 0x00); 	//sleep 끔
	twi_write(0x1A, 0x05); 	//DLPF 10Hz

	calibrate();

	int Fall_Lv_1 = 1;
	int Fall_Lv_2 = 1;
	int Crash_Lv_1 = 1;
	//int acc_cou = 0;
	int det_btn = 0;
	int det_cnt = 0;
	char Alert = 64;

	SREG = 0x80;
	
	USART_Transmit('\n');
	USART_Transmit('\r');
	USART_Transmit('S');

	g_x_tmp = 0;
	g_y_tmp = 0;
	g_z_tmp = 0;

	g_x_sum = 0;
	g_y_sum = 0;
	g_z_sum = 0;

	a_x_MAX = 0;
	a_y_MAX = 0;
	a_z_MAX = 0;

	a_x_min = 0;
	a_y_min = 0;
	a_z_min = 0;
//**********초기설정 끝**********
	while(1)
	{ 
		get_raw_data();

		las_angle_gx = roll;	//최근값 누적
		las_angle_gy = pitch;
		las_angle_gz = yaw;
//값 받아오기
		temp = (a_x_h<<8) | a_x_l;
		a_x = temp;
		temp = (a_y_h<<8) | a_y_l;
		a_y = temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z = temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x = temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y = temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z = temp;

		g_x = (g_x - bas_g_x)/FS_SEL;
		g_y = (g_y - bas_g_y)/FS_SEL;
		g_z = (g_z - bas_g_z)/FS_SEL;

		g_x_tmp = g_x_sum;
		g_y_tmp = g_y_sum;
		g_z_tmp = g_z_sum;

		g_x_sum = g_x_tmp + g_x;
		g_y_sum = g_y_tmp + g_y;
		g_z_sum = g_z_tmp + g_z;
		// 누적 오차 제거용 코드(일정값 이하 삭제)
		if(g_x < 10 && g_x > -10 && g_x_sum < 1000)
		{
			g_x_sum = 0;
		}
		if(g_y < 10 && g_y > -10 && g_y_sum < 1000)
		{
			g_y_sum = 0;
		}
		if(g_z < 10 && g_z > -10 && g_z_sum < 1000)
		{
			g_z_sum = 0;
		}
		//여기까지
		
		angle_ax = atan(-1.000*a_y/sqrt(pow(a_x,2) + pow(a_z,2)))*180/3.141592;
		angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2)))*180/3.141592;

		angle_gx = g_x*dt + las_angle_gx;
		angle_gy = g_y*dt + las_angle_gy;
		angle_gz = g_z*dt + las_angle_gz;

		dt = 0.000;

		alpha = 0.96;
		roll = alpha*angle_gx + (1.000 - alpha)*angle_ax;
		pitch = alpha*angle_gy + (1.000 - alpha)*angle_ay;
		yaw = angle_gz;

		//USART_Transmit('\n');
		//USART_Transmit('\r');
		//USART_Transmit('L');	
		//USART_Transmit('\0');

		
		
		
		//_delay_ms(1000);
		/*
		if(a_x_min > a_x)
		{
			a_x_min = a_x;
			acc_cou = 1;
		}
		
		if(a_y_min > a_y)
		{
			a_y_min = a_y;
			acc_cou = 1;
		}
		
		if(a_z_min > a_z)
		{
			a_z_min = a_z;
			acc_cou = 1;
		}

		if(a_x_MAX < a_x)
		{
			a_x_MAX = a_x;
			acc_cou = 1;
		}
		
		if(a_y_MAX < a_y)
		{
			a_y_MAX = a_y;
			acc_cou = 1;
		}
		
		if(a_z_MAX < a_z)
		{
			a_z_MAX = a_z;
			acc_cou = 1;
		}		


		if(acc_cnt)
		{
			USART_Transmit('\t');
			USART_Transmit('M');
			USART_Transmit('\t');
			USART_Transmit_init4(a_x_MAX);
			USART_Transmit('\t');
			USART_Transmit_init4(a_y_MAX);
			USART_Transmit('\t');
			USART_Transmit_init4(a_z_MAX);
			USART_Transmit('\n');
			USART_Transmit('\r');
			USART_Transmit('\t');
			USART_Transmit('m');
			USART_Transmit('\t');
			USART_Transmit_init4(a_x_min);
			USART_Transmit('\t');
			USART_Transmit_init4(a_y_min);
			USART_Transmit('\t');
			USART_Transmit_init4(a_z_min);
			USART_Transmit('\n');
			USART_Transmit('\r');
			acc_cou = 0;
		} 가속수치 플마 20000 기준점
		*/

		/*
		USART_Transmit_init4(g_x);
		USART_Transmit('\t');
		USART_Transmit_init4(g_y);
		USART_Transmit('\t');
		USART_Transmit_init4(g_z);
		USART_Transmit('\t');
		*/
		/*
		USART_Transmit_init4(g_x_sum);
		USART_Transmit('\t');
		USART_Transmit_init4(g_y_sum);
		USART_Transmit('\t');
		USART_Transmit_init4(g_z_sum);
		USART_Transmit('\t');
		USART_Transmit('\n');
		*/

		//사고 감지 코드
		if((g_y_sum > 5000 || g_z_sum > 5000 || g_y_sum < -5000 || g_z_sum < -5000) && Fall_Lv_1)
		{
			Alert = Alert + 1; //알레트에 +0001
			det_btn = 1;
			Fall_Lv_1 = 0;
		}		

		if((g_y_sum > 15000 || g_z_sum > 15000 || g_y_sum < -15000 || g_z_sum < -15000) && Fall_Lv_2)
		{
			Alert = Alert + 2; //알레트에 +0010
			det_btn = 1;
			Fall_Lv_2 = 0;
		}

		if((a_y > 20000 || a_z > 20000 || a_y < -20000 || a_z < -20000) && Crash_Lv_1)
		{
			Alert = Alert + 4; //알레트에 +0100
			det_btn = 1;
			Crash_Lv_1 = 0;
		}
		/*
		A 전복
		C 추락
		D 충돌
		E 충돌 후 전복
		G 충돌 후 추락
		*/



		/*
		USART_Transmit_init4(roll);
		USART_Transmit('\t');
		USART_Transmit_init4(pitch);
		USART_Transmit('\t');
		USART_Transmit_init4(yaw);
		USART_Transmit('\n');
		USART_Transmit('\r');
		*/

		if(det_btn)
		{
			det_cnt++;
		}
	
		
		if(det_cnt > 1000)
		{
			USART_Transmit(Alert);
			break;
		}

		_delay_ms(10);

	} 

} 


ISR(TIMER0_OVF_vect)	//0.002s
{
	dt += 0.002;

	TCNT0 = 256-125;
}




void calibrate()	//초기값 읽기 
{
	int cal = 10;

	for(int i=0; i<cal; i++)	//평균 
	{
		get_raw_data();
	
		temp = (a_x_h<<8) | a_x_l;
		a_x += - temp;
		temp = (a_y_h<<8) | a_y_l;
		a_y += - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z += temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x += temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y += temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z += temp;

		_delay_ms(100);
	}	
	
	a_x /= cal;
	a_y /= cal;
	a_z /= cal;
	g_x /= cal;
	g_y /= cal;
	g_z /= cal;

	bas_a_x = a_x;	//초기 값으로 저장 
	bas_a_y = a_y;
	bas_a_z = a_z;
	bas_g_x = g_x;
	bas_g_y = g_y;
	bas_g_z = g_z;

}


void get_raw_data()
{
	a_x_h = twi_read(0x3B);		//x축 가속도
	a_x_l = twi_read(0x3C);
	a_y_h = twi_read(0x3D);		//y축 가속도 
	a_y_l = twi_read(0x3E);		
	a_z_h = twi_read(0x3F);		//z축 가속도 
	a_z_l = twi_read(0x40);		
	g_x_h = twi_read(0x43);		//x축 각속도 
	g_x_l = twi_read(0x44);		
	g_y_h = twi_read(0x45);		//y축 각속도 
	g_y_l = twi_read(0x46);		
	g_z_h = twi_read(0x47);		//z축 각속도 
	g_z_l = twi_read(0x48);		
}




void twi_write(unsigned char address,unsigned char data)
{ 
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT))); 
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	TWDR = data; 				 //data 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송  

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP
} 

 

unsigned char twi_read(char address)
{ 
	unsigned char data;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT))); 
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//Repeat START

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

	TWDR = 0b11010001;			 //AD(1101000)+R(1) 
	TWCR = (1<<TWINT)|(1<<TWEN); //전송 

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림 

	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림 

	data = TWDR; 

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP

	return data; 
}



void USART_Transmit(unsigned char tx_data)
{ 
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = tx_data; 
}


void USART_Transmit_init4(int data)
{
	if(data < 0)
	{
		data = -data;
		USART_Transmit('-');
	}
	else
		USART_Transmit(' ');

	int temp = 0;
	temp = data/10000;
	USART_Transmit(temp+48);
	temp = (data%10000)/1000;
	USART_Transmit(temp+48);
	temp = (data%1000)/100;
	USART_Transmit(temp+48);
	temp = (data%100)/10;	
	USART_Transmit(temp+48);
	temp = data%10; 
	USART_Transmit(temp+48);
}
