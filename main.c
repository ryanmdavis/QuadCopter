/*********
	* This code uses Peter Fluery's I2C library
	*/

//physical shock/accel seems to freeze the microcontroller.  MAybe put a sponge underneath it?
//problem maybe went away when I used battery to power it

//not sure if wdt isr is actually running, no blinks.  you only get one timeout period before reset goes.  

//Libraries:
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <compat/twi.h>
#include <i2cmaster.h>
#include <twimaster.c>
#include <math.h>
#include <main.h>
#include <avr/wdt.h>
#include <qc.c>
#include <util/atomic.h>


int main(void)
{
	// incase of watchdog reset, turn off watchdog during startup code
	if (!(MCUSR&(1<<WDRF))) writeChar2EEPROM((char)0,0);
	MCUSR=0;
	wdt_disable(); //disable the wdt incase reset just occured
	_delay_ms(30);
	
	DDRB |= 0b00001111;
	PORTB |= 0b00000011;

	//initialize UART buffer
	uart_buffer_bt = (char *)malloc(sizeof(char));
	uart_buffer_xbee = (char *)malloc(sizeof(char)*8);
	stick_pos_xbee = (volatile uint8_t *)malloc(sizeof(uint8_t)*4);
	for (uint8_t i=0;i<4;i++) stick_pos_xbee[i] = 128;
	for (uint8_t i=1;i<7;i++) uart_buffer_xbee[i]=(char)((uint8_t)'a'+i);
	initUartBuffer();
	
	//initialize pointeres to command segments
	cmd_seg_loc = malloc(sizeof(char *));

	//initialize QCSR1 to "waiting for qc command"
	QCSR1|=MOTORS_OFF|PID_MODE;
	//enable interrupts
	sei();

	//initialize pin registers
	DDRA |= (1 << PINA0); //Set pinA0 to be Request to Send.  (O) 
	DDRA &= ~(1 << PINA1);  //Set PINA1 to be Clear to Send.  (I) I.e. if CTS is low, then send.  If CTS is high, then wait

	
	//initialize PWM
	initPWM();
	
	//sends pwm of zero throttle to esc of all motors
	killMotors();
	
	//initialize while loop and XBEE timers
	initQCTimer();
	volatile uint32_t time_since_last_xbee=0;
	
	// allocate memory to store real time motion data:
	// motion_data_sign_ptr has units of 1000*g (accel) and 10^-4rad per second (rotation)
	int16_t * motion_data_sign_ptr = (int16_t *)malloc(7*sizeof(int16_t));
	gyro_zero_signed_ptr = (int16_t *)malloc(sizeof(int16_t)*3);
	for (uint8_t i=0;i<3;i++) gyro_zero_signed_ptr[i]=0;
	
	// alocate memory to store motion data in physical units:
	//int16_t * gyro_dps = (int16_t *)malloc(3*sizeof(int16_t));  //in 100*dps
	//int16_t * a_g = (int16_t *)malloc(3*sizeof(int16_t));		//g's times 1000
	
	//declare pid and motor variables:
	int16_t * delta_angle = malloc(3*sizeof(int16_t));
	int32_t * angle = malloc(3*sizeof(int32_t));
	int16_t * dd_angle = malloc(3*sizeof(int16_t)); //degrees per secod per second
	int16_t * d_angle = malloc(3*sizeof(int16_t));	//degrees per second
	int16_t * d_angle_tgt = malloc(3*sizeof(int16_t)); //target rotation rate, i.e. joystick position
	int16_t * m_p = malloc(4*sizeof(int16_t)); //pointer to motor duty cycle
	for (uint8_t i = 0;i<3;i++) {delta_angle[i]=0; angle[i]=0; dd_angle[i]=0; d_angle[i]=0; d_angle_tgt[i]=0; motion_data_sign_ptr[i]=0;}
	for (uint8_t i = 0;i<4;i++) {m_p[i]=0;}

	
	//Declare rotation matrix:  Use int32 because int16 will only go up to 2^15/100 ~= 330 degrees
	//Rows are r_i,p_i,y_i, cols are r_f,p_f,y_f
	int32_t R[3][3] = {{ANGLE_UNITS_INV,0,0},{0,ANGLE_UNITS_INV,0},{0,0,ANGLE_UNITS_INV}};

	
	//This rotation matrix maps x,y,z from gyro axes onto r,p,y of qc axes
	//Rxp[r,p,y][x,y,z]
	/*int16_t Rxp[3][3] = 	{{707,707,0},
							 {-707,707,0},
							 {0,0,1000}};*/
	int16_t Rxp[3][3] = 	{{-707,-707,0},
							 {707,-707,0},
							 {0,0,1000}};
	//int16_t Rxp[3][3] = {{RXP_FACTOR,0,0},{0,RXP_FACTOR,0},{0,0,RXP_FACTOR}};

	//Declare matrix of pid values. rows are pid, 	cols are rpy
	//int16_t K[3][3] = {{17,17,17},{23,23,23},{30,30,30}};  //see excel sheet for latest value (was 17,23,30)
	int16_t K[3][3] = {{17,17,17},{23,23,23},{30,30,30}};  //see excel sheet for latest value (was 17,23,30)

	
	
	//Declare array of "amount" of rpyl 
	int16_t * rpyl = malloc(4*sizeof(int16_t));
	rpyl[0] = rpyl[1] = rpyl[2] = 0;
	rpyl[3] = 0;

	//declare timer variable
	uint16_t * dt_p = malloc(sizeof(uint16_t));
	int16_t * dt_hz = malloc(sizeof(int16_t));
	uint32_t * time_since_last_renorm = malloc(sizeof(uint32_t));
	*time_since_last_renorm=0;

	//disable sleep mode of gyro and zero the gyros
	if (writeTWI(GyroAddr,MPU6050_RA_PWR_MGMT_1,1)) sendStringViaUART("Gyro wakeup failed",0);
	writeTWI(GyroAddr,MPU6050_CONFIG,3); 	//set DLPF to 42 Hz
	determineGyroZeros(DEFAULT_GYRO_ZERO_REPS,0,&Rxp);
	zeroAngles(delta_angle,angle,&R,rpyl,0);
	
	//initialize sign of life timer
	initSignsOfLifeTimer();
	
	//initialize external interupts
	initInter();
	
	//initialize UART.  UBRR = 12 correspondds to a Baud Rate of 38.4kHz with error of 0.2%
	initUART(12,12); //baud rates of 38.4kbs (BT) and 38.4kbs (XBEE)
	
	//initialize counter of main while loop
	uint8_t while_loop_count=0;

	//initialize watchdog timer and set first EEPROM byte to zero
	initWatchdog();
	while(1)
	{	
		wdt_reset(); //resets watchdog timer
		while_loop_count++;
		if (while_loop_count>=30)
		{
			while_loop_count=0;
			PORTB ^= 1<<PINB0;
			
		}
		/****************************************************
		/Workflow for controlling QC motion:
		*****************************************************/		
		readMotionData(motion_data_sign_ptr,&Rxp); //10^-4 rad/s and g's
        addStickPos(d_angle_tgt,motion_data_sign_ptr);
		calcDeltaAngle(motion_data_sign_ptr,delta_angle,dt_p); //delta angle in 10^-4 rad
		updateR(&R,delta_angle);
		calcAngularMotion(&R,angle,d_angle,dd_angle,motion_data_sign_ptr); //angle,dps, dpsps all in 10ths of degree (plus time units)
		if (QCSR1 & PID_MODE) pid(rpyl,&K,angle,d_angle,dd_angle,m_p);
		governMotorDutyCycle(m_p);
		if (QCSR1 & MOTORS_OFF) setMotorDutyCycle((int16_t *)m_p_off); else setMotorDutyCycle(m_p);
		renormalizeR(dt_p,time_since_last_renorm,&R);
		
		/****************************************************
		/Workflow for receiving and interpreting QC commands:
		*****************************************************/
		checkUARTBufferForCmd();
		//if qc command has be received then interpret it:
		if ((QCFCR & RECEIVED_BT_CMD)){ //this if statement takes ~10us to evaluate (not the functions, just checking QCSR1)
			findCommandSegments((const char *)uart_buffer_bt);
			interpretCommand(m_p,delta_angle,&R,angle,d_angle,dd_angle,d_angle_tgt,motion_data_sign_ptr,rpyl,&K,time_since_last_renorm,&Rxp,dt_p);
			clearUARTBuffer();
			QCFCR&=~(RECEIVED_BT_CMD|RECEIVING_BT_CMD); //allow qc to receive the next BT command
			}
		
		//if AT command was detected then clear the uart buffer
		if ((QCFCR & RECEIVED_AT_CMD)){ //this if statement takes ~10us to evaluate (not the functions, just checking QCSR1)
			clearUARTBuffer();
			QCFCR&=~(RECEIVED_AT_CMD|RECEIVING_AT_CMD); //allow qc to receive the next BT command
			}		
			
		if (QCFCR&RECEIVED_XBEE_CMD){
			setStickPositionXBee(d_angle_tgt,rpyl);
			QCFCR&=~RECEIVED_XBEE_CMD;
			}
		
		//If python is expecting motion data, then send it:
		if (QCSR1 & RETURN_MOTION_DATA){
			for (uint8_t i = 0;i<3;i++) motion_data_sign_ptr[i+4] = (int16_t)angle[i];
			//for (uint8_t i = 0;i<3;i++) motion_data_sign_ptr[i+4] = (int16_t)dd_angle[i];
			//sendInt16VectorViaUART(motion_data_sign_ptr,7,10);
			sendInt16VectorViaUARTAsChars(motion_data_sign_ptr,7);
			sendStringViaUART("/",0);
			toggleReturnMotionDataState();
			}
		
		if (QCSR1 & RETURN_MOTOR_DUTY_CYCLE){
			sendInt16VectorViaUARTAsChars(m_p,4);
			//sendInt16VectorViaUART(m_p,4,10);
			sendStringViaUART("/",0);
			toggleReturnMotorDutyCycleState();
			}
		
		if (QCSR1 & KILL_MOTORS)	{
			for (int i = 0;i<4;i++) m_p[i] = getMotorPWMOCValue(100); //if kill switch pressed put all motors to zero
			QCSR1 |= MOTORS_OFF;
			}
			
		if (QCSR1 & RETURN_QC_RATE) {
			sendStringViaUART("QC cycle rate: ",0);
			*dt_hz = 1e6/(*dt_p);
			sendInt16VectorViaUART(dt_hz,1,10);
			sendStringViaUART(" Hz \n/",0);
			toggleRetQCRate();
			}
		
		//Toggling the kill switch based on the lift joystick position
		if ((stick_pos_xbee[LIFT] < JOY_KILL_THRESH_LO) && !(QCSR2 & KILL_SWITCH_DOWN)) 
			{
			toggleMotorsOff(0);
			if (!(QCSR2 & GYRO_ZEROED)) determineGyroZeros(DEFAULT_GYRO_ZERO_REPS,0,&Rxp);
			QCSR2 |= GYRO_ZEROED;
			zeroAngles(delta_angle,angle,&R,rpyl,0);
			QCSR2|=KILL_SWITCH_DOWN;
			//BLINK0(2);
			}
			
		if (stick_pos_xbee[LIFT] > JOY_KILL_THRESH_HI) QCSR2&=(~KILL_SWITCH_DOWN);
		
		// make sure we are still hearing from XBEE
		time_since_last_xbee=timeSinceLastXBEE();
		PORTB|=(1<<PINB1);
		if ((time_since_last_xbee > XBEE_TIME_OUT_DELAY) || (QCFCR&XBEE_TIMED_OUT))
			{
			QCFCR|=XBEE_TIMED_OUT;
			QCSR1|=MOTORS_OFF;
			stopXBEETimer();
			PORTB&=~(1<<PINB1);
			}

		PORTA &= ~(1 << PINA0); //Set PINA0 to be zero, i.e. ready to receive next bluetooth byte
	}

	return 1;
}

ISR(USART0_RX_vect)  //UART receive complete interrupt
{
	//variables
	char received_byte;

	PORTA |= (1 << PINA0); //During this ISR, MC is NOT ready to receive another UART byte

	//Wait until reception is complete
	while(!(UCSR0A & (1<<RXC0)));
	received_byte = (char)UDR0;

	//only write data to uart_buffer_bt if QC is not still interpreting the previous command
	if ((!(QCFCR & RECEIVED_BT_CMD))&&(!(QCFCR & RECEIVED_AT_CMD)))
	{
		//PORTB ^= 1 << PINB0;
		uart_buffer_bt_size++; //!!!do not move this call into the next line, because it does not work!!!
		if (uart_buffer_bt_size > uart_buffer_bt_size_max){
			uart_buffer_bt_size_max++;
			if (uart_buffer_bt_size_max>MAX_UART_BUFFER_SIZE)
			{
				clearUARTBuffer();
			}
			else changeUartBuffSize(BT,uart_buffer_bt_size_max);
			}
		uart_buffer_bt[uart_buffer_bt_size-1] = (char)received_byte;
	}

}

ISR(USART1_RX_vect)  //XBEE UART receive complete interrupt
{
	char received_byte;
	//Wait until reception is complete
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		while(!(UCSR1A & (1<<RXC1)));
		received_byte = (char)UDR1;
	
		uart_buffer_xbee[6]=uart_buffer_xbee[5];
		uart_buffer_xbee[5]=uart_buffer_xbee[4];
		uart_buffer_xbee[4]=uart_buffer_xbee[3];
		uart_buffer_xbee[3]=uart_buffer_xbee[2];
		uart_buffer_xbee[2]=uart_buffer_xbee[1];
		uart_buffer_xbee[1]=uart_buffer_xbee[0];
		uart_buffer_xbee[0]=(char)received_byte;
	
		if ((uart_buffer_xbee[0]=='q')&&(uart_buffer_xbee[6]=='\n')) 
			{
			for (uint8_t i = 0;i<4;i++) stick_pos_xbee[i] = (uint8_t)uart_buffer_xbee[i+1];
			
			//If XBEE connection was timed out
			if (QCFCR&XBEE_TIMED_OUT)
			{
				initXBEETimer();
				QCFCR&=~XBEE_TIMED_OUT; //Turn off timeout flag, since it was on
			}
			QCFCR|=RECEIVED_XBEE_CMD;
			resetXBEETimer();
			}
	}
}

//This interrupt vector handles the kill switch
ISR(INT2_vect)
{
	sendStringViaUART("qc killed!!!!\nuart_buffer was: ",0);
	sendStringViaUART((const char *)uart_buffer_bt,0);
	writeQCR();
	killMotors();
	toggleKillMotors();
	QCFCR=0;
	for (uint8_t i=0; i < uart_buffer_xbee_size_max;i++) uart_buffer_xbee[i] = '*';
	for (uint8_t i=0; i < uart_buffer_bt_size_max;i++) uart_buffer_bt[i] = '*';
	BLINK0(2);
}

//called when TCNT0 overflows.  Lets mcu know how many overflows occur between cycles of main while loop
ISR(TIMER0_OVF_vect)
{
	tcnt0_overflow_num+=1;
}

//for checking when the last signal from XBEE was recorded
ISR(TIMER2_OVF_vect)
{
	tcnt2_overflow_num+=1;
}

ISR(WDT_vect)
{
	//kill all of the motors
	OCR1A=0;
	OCR1B=0;
	OCR3A=0;
	OCR3B=0;
	QCSR1|=MOTORS_OFF;
	TCCR1B=0;//turn off clock sources
	TCCR3B=0;

	
	//set the flag that shows that an wdt timeout has occured
	char uart_buffer_size_char=readCharFromEEPROM(0);
	if ((uint8_t)uart_buffer_size_char == 0) //only write to EEPROM if there is not already an error there
	{
		for (uint8_t uart_char_num=0;uart_char_num<uart_buffer_bt_size;uart_char_num++)
		{
			wdt_reset();
			writeChar2EEPROM((char)uart_buffer_bt[uart_char_num],uart_char_num+1);
		}
		writeChar2EEPROM((char)uart_buffer_bt_size,0);
	}
	
	//hang here until system reset
	while(1);
}