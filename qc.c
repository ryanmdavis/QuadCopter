/*******************************
	* include file for some QC commands.
	***************************/

//Define register values
#define GyroAddr			0xD0			//MPU-6050 3-axis Gyro and Accel.  AD0 pin is @ 0V
#define GyroRead			0b00000001
#define GyroWrite			0b00000000
	
//Read Addresses copied from: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

//Power management byte and bit: for setting sleep mode.  Set bit to 0 to disable sleep mode?
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_PWR1_SLEEP_BIT          6

//Define the various QC states
#define WAITING_FOR_CMD			0b00000000		//waiting for a qc command
#define	RECEIVED_CMD			0b00000001		//received "qc-", now wait for the rest of a command
#define RETURN_MOTION_DATA		0b00000010		
#define INTERPRETING_QC_CMD		0b00000100		//Third bit QCSR should be set if QC is interpreting previous command
#define PID_MODE				0b00001000
#define RETURN_MOTOR_DUTY_CYCLE	0b00010000
#define KILL_MOTORS				0b00100000

//Define PWM constants:
#define MIN_PWM_DUTY_CYCLE	1000			//min duration of the pulse in usec
#define MAX_PWM_DUTY_CYCLE	2000			//max duration of the pulse in usec

//calibration constants for gyro and accelerometer
#define LSB_PER_DPS			131
#define LSB_PER_G			16384
#define LSB_PER_C			340
#define T_Y_INTERCEPT		3653

//define the vectors that describe roll, pitch, yaw, lift
volatile const int16_t pitch[4] = {1,0,-1,0};
volatile const int16_t roll[4] = {0,1,0,-1};
volatile const int16_t yaw[4] = {1,-1,1,-1};
volatile const int16_t lift[4] = {1,1,1,1};


//variable to count overflows of TCNT0
volatile uint8_t tcnt0_overflow_num;

void BLINK0(uint8_t num_blinks)
{
	int i;
	for (i = 0;i<num_blinks;i++){
		PORTB ^= 1 << PINB0;
		_delay_ms(200);
		PORTB ^= 1 << PINB0;
		_delay_ms(200);
		}
	return;
}

void BLINK1(uint8_t num_blinks)
{
	int i;
	DDRB|=0b00000010;
	for (i = 0;i<num_blinks;i++){
		PORTB ^= 1 << PINB1;
		_delay_ms(200);
		PORTB ^= 1 << PINB1;
		_delay_ms(200);
		}
	return;
}

void sendStringViaUART(const char * array_ptr,uint8_t array_size)
{
	uint8_t i;
	if (array_size < 1) array_size = strlen((const char *)array_ptr);
	for (i = 0;i < array_size; i = i + 1)
	{
		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
		UDR0 = array_ptr[i];
	}
	return;
}

void sendIntVectorViaUART(uint8_t * array_ptr,uint8_t array_size)
{
	uint8_t i;
	for (i = 0;i < array_size; i = i + 1)
	{
		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
		UDR0 = (char)array_ptr[i];
	}
	return;
}

/**
 *void sendInt16VectorViaUART(int16_t * array_ptr,uint8_t array_size)
 *{
 *	uint8_t i;
 *	for (i = 0;i < array_size; i = i++)
 *	{
 *		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
 *		UDR0 = (char)array_ptr[i]>>8;
 *		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
 *		UDR0 = (char)array_ptr[i]&0xff;
 *	}
 *	return;
 *}
 */

int16_t iabs(int16_t num)
{
	if (num >=0) return num;
	else return -num;
}

uint8_t findMSBLoc(int16_t number,uint8_t base)
{
	uint8_t msb = 0;
	while ((int32_t)iabs(number)>((int32_t)pow((double)base,(double)msb))) msb++;
	return msb;
}

void sendInt16VectorViaUART(int16_t * array_ptr,uint8_t array_size,uint8_t base)
{
	uint8_t i;
	int16_t this_int16;
	uint8_t num_bits;
	char * bin_string = NULL;

	for (i = 0;i < array_size; i = i + 1)
	{
		this_int16 = array_ptr[i];
		num_bits = findMSBLoc(this_int16,base); //hangs here
		if ((base == 10)&&(this_int16<0)) num_bits+=1; //need room for minus sign
		bin_string = (char *)malloc(sizeof(char)*(num_bits+1));
		itoa(array_ptr[i],bin_string,base);
		sendStringViaUART((const char *)bin_string,0);
		sendStringViaUART(":",0);
		free(bin_string);
		//sendStringViaUART("5",0);
	}
	return;
	
}
void sendUint16VectorViaUART(uint16_t * array_ptr,uint8_t array_size,uint8_t base)
{
	uint8_t i;
	char * bin_string = "                ";
	for (i = 0;i < array_size; i = i + 1)
	{
		itoa(array_ptr[i],bin_string,base);
		sendStringViaUART((const char *)bin_string,0);
		sendStringViaUART(":",0);
	}
	return;
}
void sendInt8VectorViaUART(int8_t * array_ptr,uint8_t array_size,uint8_t base)
{
	uint8_t i;
	char * bin_string = "        ";
	for (i = 0;i < array_size; i = i + 1)
	{
		itoa(array_ptr[i],bin_string,base);
		sendStringViaUART((const char *)bin_string,0);
		sendStringViaUART(":",0);
		
	}
	return;
}

void sendUint8VectorViaUART(uint8_t * array_ptr,uint8_t array_size,uint8_t base)
{
	uint8_t i;
	char * bin_string = "        ";
	for (i = 0;i < array_size; i = i + 1)
	{
		itoa(array_ptr[i],bin_string,base);
		sendStringViaUART((const char *)bin_string,0);
		sendStringViaUART(":",0);
		
	}
	return;
}

void sendInt16VectorViaUARTAsChars(int16_t * int_vector,uint8_t array_size)
{
	if (!array_size) array_size = sizeof(int_vector)/sizeof(int_vector[0]);
	for (int i = 0;i < array_size; i = i + 1)
	{
		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
		UDR0 = (char)(int_vector[i]>>8);
		while(!(UCSR0A & (1<<UDRE0)) | (PORTA & (1 << PINA1)));
		UDR0 = (char)(int_vector[i]&0xff);
		
	}
}

int16_t bitwiseUintToInt(uint8_t high,uint8_t low)
{
	uint16_t unsigned16;
	unsigned16 = ((uint16_t)high)*256 + (uint16_t)low;
	//unsigned16 = 16385;
	if ((unsigned16&0x8000) == 0) return (int16_t)unsigned16;
	else return (int16_t)(unsigned16&0x7fff) - 32768;
}

void reset(void)
{
	//reset gyro
	//reset bluetooth
	//reset mcu last
}


	/*****************************************************************
	*input: 
	*		k: pid constants
	*		xa,xt: actual and target roll,pitch,yaw or lift (rpyl)
	*output:
	*		correction: the intenisty of the rpyl correction.  This number
	*		will be multiplied by a rpyl vector.
	******************************************************************/
void pid(int16_t * k,int16_t xa,int16_t xt, int16_t * correction)
{
	//use inputs to find correction.  correction will be multipled by the appropriate rpyl vector
}

void pidPitch(int16_t * k,int16_t pa_p,int16_t pt_p,int16_t * m)
{
	int16_t correction=0;
	int16_t dp; //delta pitch
	pid(k,pa_p,pt_p,&correction);
	addToVect(1,m,correction,pitch);
}

void setMotorDutyCycle(int16_t * m_p)
{
	OCR1A = m_p[0];
	OCR1B = m_p[1];
	OCR3A = m_p[2];
	OCR3B = m_p[3];
}

void addToVect(int16_t vect1_coeff,int16_t * vect1,int16_t vect2_coeff,int16_t * vect2)
{
	if ((sizeof(vect1)/sizeof(vect1[0])) != (sizeof(vect2)/sizeof(vect2[0]))) sendStringViaUART("ERROR adding vectors",0);
	else{
		for (uint8_t i = 0;i < sizeof(vect1)/sizeof(vect1[0]);i++) vect1[i] = vect1_coeff*vect1[i]+vect2_coeff*vect2[i];
		}
}

/****************************************************************************
	*btod - bit to dps - conversts from dps/LSB to 100*dps and returns dps
	*assumes that gyro is set to +/-dps range
	*input:
	*		gyro_bit - gyro rate in bits (unsigned integer 16bit)
	*output:
	*		gyro rate in 100*degrees per second
	*************************************************************************/
int16_t btod(int16_t gyro_bit)
{
	return (int16_t)((100*(int32_t)gyro_bit)/LSB_PER_DPS);
}

/****************************************************************************
	*btos - bit to dps - conversts from dps/LSB to 1000*g
	*assumes that gyro is set to +/-dps range
	*input:
	*		gyro_bit - gyro rate in bits (unsigned integer 16bit)
	*output:
	*		gyro rate in 100*degrees per second
	*************************************************************************/
int16_t btoa(int16_t a_bit)
{
	return (int16_t)((1000*(int32_t)a_bit)/LSB_PER_G);
}

/****************************************************************************
	*btot - bit to degrees celcius
	*input:
	*		gyro_bit - gyro rate in bits (unsigned integer 16bit)
	*output:
	*		gyro rate in 100*degrees celsius
	*************************************************************************/
int16_t btoc(int16_t t_bit)
{
	return (int16_t)((100*(int32_t)t_bit)/LSB_PER_C + T_Y_INTERCEPT);
}

/****************************************************************************
	*btod - bit to dps - conversts from dps/LSB to 100*dps in form of array
	*assumes that gyro is set to +/-dps range
	*input:
	*		gyro_dps - ptr to gyro rates in 100*degrees per second.  Array must be
	*					3 ints long
	*		gyro_bit - ptr to gyro rates in bits (unsigned integer 16bit).  Array must
						be 3 ints long
	*************************************************************************/
void btoda(int16_t * gyro_dps,int16_t * gyro_bit)
{
	for (uint8_t i = 0;i<3;i++) gyro_dps[i] = btod(gyro_bit[i]);
}

/****************************************************************************
	*btog - bit to g's - conversts from dps/LSB to 1000*units of gravity
	*assumes that gyro is set to +/- 2g range
	*input:
	*		gyro_dps - ptr to gyro rates in 100*degrees per second
	*		gyro_bit - ptr to gyro rates in bits (unsigned integer 16bit)
	*************************************************************************/
void btoga(int16_t * a_g,int16_t * a_b)
{
	for (uint8_t i = 0;i<3;i++) a_g[i] = btoa(a_b[i]);
}

/******************************************************************************
	*multiply dps by seconds to get degrees:
	*input:
	*	pointers to motion data(holding dps) and angle
	***************************************************************************/
void integrateRotation(int16_t * motion_data_sign_ptr,int16_t * angle)
{
	//char * time_string = "       ";
	uint16_t time = timeSinceLastMotionRead(); //time in microseconds
	//angle is in degrees * 100
	for (uint8_t i =0;i < 3;i++) angle[i] += (int16_t)(((int32_t)motion_data_sign_ptr[i+4]*(int32_t)time)/1e6);
	//sendStringViaUART("time: ",0);
	//itoa(time,time_string,10);
	//sendStringViaUART(time_string,0);
	//sendStringViaUART("\n",0);
	

}

void initQCTimer(void)
{
	//set prescaler of 8-bit timer 0 to 64.  Gives counter update of 8 us. ~2ms to go from 
	//BOTTOM to TOP
	TCCR0B |= (1<<CS01)|(1<<CS00); 
	TIMSK0 |= (1<<TOIE0); //call interrupt when TCNT0 his BOTTOM
}

//return time in microseconds since last time this function was called
int16_t timeSinceLastMotionRead(void)
{
	int16_t time;
	time = ((int16_t)tcnt0_overflow_num*0xff+TCNT0)*8;
	tcnt0_overflow_num=0;
	TCNT0=0;
	return time;
}

//Function sets all angles to zero:
void zeroAngles(int16_t * angle) 
	{
	for (uint8_t i = 0;i<3;i++) angle[i] = 0;
	sendStringViaUART("Angles Zeroed\n/",0);
	}