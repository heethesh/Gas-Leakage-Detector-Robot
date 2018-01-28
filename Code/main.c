/*********************************************************************************************************/

//                                         	  FILE LEVEL COMMENTS                                        //

/*********************************************************************************************************/

// IMPORTANT NOTE: PLEASE REFER TO ATTACHED JPG FOR PIPELINE AREA AND VALVE NUMBERING USED IN THIS PROGRAM

/*
* Team ID:			eYRC-GD#672
* Author List: 		Heethesh Vhavle
* Filename: 		eYRC-GD-672.c
* Theme: 			Gas Leakage Detection (eYRC 2015)

* Functions: 		Indicator Functions: buzzer_pin_config(), buzzer_on(), lcd_port_config(), led_pin_config(),
					buzzer_off(), buzz(), led_red(), led_grn(), led_blu(), led_off()

					ADC Functions: adc_pin_config(), adc_init(), ADC_Conversion(unsigned char),
					print_sensor(char, char, unsigned char), Sharp_GP2D12_estimation(unsigned char)

					Timer Functions: timer1_init(), timer3_init(), timer4_init(), timer5_init(), start_timer4(),
					stop_timer4(), ISR(TIMER3_OVF_vect), ISR(TIMER4_OVF_vect)

					Motion Functions: motion_pin_config(), left_encoder_pin_config(), right_encoder_pin_config(),
					left_position_encoder_interrupt_init(), right_position_encoder_interrupt_init(), ISR(INT4_vect),
					ISR(INT5_vect), velocity(unsigned char, unsigned char), motion_set(unsigned char), forward(), 
					back(), left(), right(), soft_left(), soft_left_2(), soft_right(), soft_right_2(), stop(),
					angle_rotate(unsigned int), linear_distance_mm(unsigned int), forward_mm(unsigned int),
					back_mm(unsigned int), left_degrees(unsigned int), right_degrees(unsigned int), turn(int),
					soft_left_degrees(unsigned int), soft_right_degrees(unsigned int), soft_left_2_degrees(unsigned int),
					soft_right_2_degrees(unsigned int)

					Servomotor Functions: servo1_pin_config(), servo2_pin_config(), servo3_pin_config(),
					servo_1(unsigned char), servo_2(unsigned char), servo_3(unsigned char), servo_1_free(), servo_2_free(),
					servo_3_free(), color_arm_up(), color_arm_down(), color_arm_90(), servo_3_front(), servo_3_left(),
					servo_3_45(), servo_3_back(), servo_initial_positions()
					
					Color Sensor Functions: color_sensor_pin_config(), color_sensor_pin_interrupt_init(),
					ISR(INT0_vect), filter_red(), filter_green(), filter_blue(), red_read(), green_read(), blue_read(), 
					color_sensor_scaling(), color_sensor_power_down(), read_color(), display_color(), get_color(int)

					Line Follower Functions: read_line(), display_line(), enable_flags(), clear_count(), search_line(),
					line_conditions(), follow_line(), line_follower(), correct_left(int), correct_right(int)

					Wall Follower Functions: read_sharp(), display_sharp(), check_wall(), follow_left_wall(int),
					follow_right_wall(int), follow_right_wall2()

					Drop Mechanism Functions: drop_align_front_wall(), drop_align1(), drop_magnet()
					
					Detection Functions: gas_detection(), valve_detection(), check_gas(int), check_valve(int)

					Navigation Functions: pipeline_navigation(), enter_pipeline(int, int), exit_pipeline(int), go_to_valve(int),
					close_known_valve(), close_known_valve_2(), close_remaining_valve(), navigate_back(), navigate_start()
					
					Initiation Functions: port_init(), init_devices(), display_status() 

					Main Function: main()					   
					 
* Global Variables:	current_PA, gas_pos[], val_pos[], half_walls_map[], valve_known, gas_known, total_detects,
					valve_remaining, magnets_dropped, wall_present, buzzer_flag, Left, Center, Right, line_cond, 
					left_flag, right_flag, node_flag, turn_flag, correct_left_flag, correct_right_flag, left_turn, 
					right_turn, node, t4_flag, t4_sec, t4_count, en_left, en_right, en_node, ShaftCountLeft, Degrees,
					ShaftCountRight, ADC_Value, sharp_1, sharp_2, sharp1, sharp2, pulse, red, blue, green, left_max,
					right_max, left_mid, right_mid
*/


/*********************************************************************************************************/

//                                         HEADERS AND DECLARATIONS                                      //

/*********************************************************************************************************/

//Header Files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "lcd.c"

//Function Prototypes
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void read_line();
void line_conditions();
void stop_timer4();
unsigned char ADC_Conversion(unsigned char);


/*********************************************************************************************************/

//                                             GLOBAL VARIABLES                                          //

/*********************************************************************************************************/

int buzzer_flag = 0; 							//Flag to indicate buzzer status (on/off)

unsigned int ADC_Value, sharp_1, sharp_2; 		//Converted Sharp sensor distance value in mm scale
unsigned char sharp1, sharp2;					//ADC input Sharp sensor distance value

unsigned char left_max = 254, right_max = 255;	//Maximum velocity
unsigned char left_mid = 254, right_mid = 255;	//Medium velocity (Line Follower Velocity)

unsigned long int ShaftCountLeft = 0; 			//To keep track of left position encoder 
unsigned long int ShaftCountRight = 0; 			//To keep track of right position encoder
unsigned int Degrees; 							//To accept angle in degrees for turning

volatile unsigned long int pulse = 0, red, blue, green; //To store pulses read by the color sensor

int t4_flag = 0;								//Flag to indicate whether Timer 4 is counting or not
int t4_sec = 0;									//Number of seconds passed after Timer 4 is enabled
int t4_count;									//Number of required seconds till which Timer 4 needs to count

int Left = 0, Center = 0, Right = 0;			//To store the ADC white line sensor values
int line_cond = 0;								//To store the type of line condition
int en_left = 1, en_right = 1, en_node = 1;		//Flags to enable/disable detection of turns/nodes
int left_flag = 0, right_flag = 0, node_flag;	//Flags used to indicate if turn/node is detected
int left_turn = 0, right_turn = 0, node = 0;	//To store the number/count of turns/nodes detected
int correct_left_flag = 1, correct_right_flag = 1; //Flags to enable/disable correction of position on line

int wall_present = 0; 							//To store the half wall configuration // 1 -> Left // 2 -> Right // 3-> Both // 0 -> None
int half_walls_map[] = {7,7,7,7};				//To store half wall configuration of all pipeline areas //7 -> Default value

/*
COLOR CODES:

0 -> Black
1 -> Red
2 -> Green
3 -> Blue
5 -> Closed Valve
>7 -> Default Initial Value
*/

int current_PA = 0;					//To store the current Pipeline Area index/number
int gas_pos[] = {8,8,8,8};			//To store the color code of gas leakages of PA-0 to PA-3 (see above)
int valve_pos[] = {9,9,9,9};		//To store the color code of control valves of D0 to D3 (see above)

int valve_known = -1;				//Flag to indicate whether position of valve of same color previously known or not
int gas_known = 0;					//Flag to indicate whether type of gas leakage already detected or not
int total_detects = 0;				//To store total number of valid gas leakages detected

int magnets_dropped = 0;			//To store the number of valves closed/magnets dropped
int valve_remaining = 0;			//Flag to indicate whether the valve (D0) needs to be closed at the end of the run or not


/*********************************************************************************************************/

//                                                INDICATORS                                             //

/*********************************************************************************************************/

/*
* Function Name: 	buzzer_pin_config		
* Input: 			None
* Output: 			None
* Logic: 			Function to initialize Buzzer connected to PORTC 3
* Example Call:		buzzer_pin_config();
*/
void buzzer_pin_config (void)
{
 	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 	PORTC = PORTC & 0xF7;	//Setting PORTC 3 logic low to turnoff buzzer
}


/*
* Function Name: 	buzzer_on	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch buzzer on, PORTC 3 pin is set high
* Example Call:		buzzer_on();
*/
void buzzer_on (void)
{
 	unsigned char port_restore = 0;
 	port_restore = PINC;
 	port_restore = port_restore | 0x08;
 	PORTC = port_restore;
}


/*
* Function Name: 	buzzer_off	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch buzzer off, PORTC 3 pin is set low
* Example Call:		buzzer_off();
*/
void buzzer_off (void)
{
 	unsigned char port_restore = 0;
 	port_restore = PINC;
 	port_restore = port_restore & 0xF7;
 	PORTC = port_restore;
}


/*
* Function Name: 	buzz		
* Input: 			time_delay (int) -> Time duration between buzzer on and off
* Output: 			None
* Logic: 			Buzzer is set to on for required time period and then switched off
* Example Call:		buzz(1000);
*/
void buzz(int time_delay)
{
	buzzer_on();
	_delay_ms(time_delay);
	buzzer_off();
}


/*
* Function Name: 	lcd_port_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to configure LCD port (PORT C)
* Example Call:		lcd_port_config();
*/
void lcd_port_config (void)
{
 	DDRC = DDRC | 0xF7; 	//All the LCD pin's direction set as output
 	PORTC = PORTC & 0x80; 	//All the LCD pins are set to logic 0 except PORTC 7
}


/*
* Function Name: 	led_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to configure LED port (PORT J)
* Example Call:		led_pin_config();
*/
void led_pin_config (void)
{
 	DDRJ = DDRJ | 0xFF;		
 	PORTJ = PORTJ & 0x00;		
}


/*
* Function Name: 	led_red	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch on Red LED, PORTJ 2 High
* Example Call:		led_red();
*/
void led_red()
{
 	unsigned char port_restore = 0;
 	port_restore = PINJ;
 	port_restore = port_restore | 0x04;
 	PORTJ = port_restore;
}


/*
* Function Name: 	led_grn	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch on Green LED, PORTJ 3 High
* Example Call:		led_grn();
*/
void led_grn()
{
 	unsigned char port_restore = 0;
 	port_restore = PINJ;
 	port_restore = port_restore | 0x08;
 	PORTJ = port_restore;
}


/*
* Function Name: 	led_blu	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch on Blue LED, PORTJ 4 High
* Example Call:		led_blu();
*/
void led_blu()
{
 	unsigned char port_restore = 0;
 	port_restore = PINJ;
 	port_restore = port_restore | 0x10;
 	PORTJ = port_restore;
}


/*
* Function Name: 	led_off	
* Input: 			None
* Output: 			None
* Logic: 			Function to switch off RGB LED, PORTJ 2-3-4 Low
* Example Call:		led_red();
*/
void led_off()
{
 	PORTJ &= 0xE0;
}


/*********************************************************************************************************/

//                                               ADC FUNCTIONS                                           //

/*********************************************************************************************************/

/*
* Function Name: 	adc_pin_config 	
* Input: 			None
* Output: 			None
* Logic: 			PORTS F AND K are configured to enable ADC conversion
* Example Call:		adc_pin_config ();
*/
void adc_pin_config (void)
{
 	DDRF  = 0x00; 
 	PORTF = 0x00;
 	DDRK  = 0x00;
 	PORTK = 0x00;
}


/*
* Function Name: 	adc_init	
* Input: 			None
* Output: 			None
* Logic: 			The registers required for ADC conversion are configured
* Example Call:		adc_init();
*/
void adc_init()
{
	ADCSRA = 0x00;	//Initialise ADC Control and Status register A
	ADCSRB = 0x00;	//MUX5 = 0
	ADMUX  = 0x20;	//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR   = 0x80;	//Enabling the Analog comparator interrupt
	ADCSRA = 0x86;	//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


/*
* Function Name: 	ADC_Conversion	
* Input: 			Ch (unsigned char) -> Channel Number

					ACD CH.     PORT        Sensor
                  	0 			PF0 		Battery Voltage
                  	1 			PF1 		White line sensor 3
                  	2 			PF2 		White line sensor 2
                  	3 			PF3 		White line sensor 1
                  	4 			PF4 		IR Proximity analog sensor 1
                  	5 			PF5 		IR Proximity analog sensor 2
                  	6 			PF6 		IR Proximity analog sensor 3
                  	7 			PF7 		IR Proximity analog sensor 4
                  	8 			PK0 		IR Proximity analog sensor 5
                  	9 			PK1			Sharp IR range sensor 1
                  	10 			PK2 		Sharp IR range sensor 2
                  	11 			PK3 		Sharp IR range sensor 3
                  	12 			PK4 		Sharp IR range sensor 4
                  	13 			PK5 		Sharp IR range sensor 5

* Output: 			a (unsigned char) -> Returns corresponding ADC value
* Logic: 			Reads data from the sensors and performs analog to digital
 					conversion by using two registers ADCSRA and ADCSRB
* Example Call:		value = ADC_Conversion(11);
*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; 		//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


/*
* Function Name: 	print_sensor	
* Input: 			row (char) -> LCD row
					column(char) -> LCD column
					channel (unsigned char) -> ADC Channel Number
* Output: 			None
* Logic: 			Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
* Example Call:		print_sensor(1, 1, 11);
*/
void print_sensor(char row, char coloumn, unsigned char channel)
{
	
	ADC_Value = (int)(ADC_Conversion(channel));
	lcd_print(row, coloumn, ADC_Value, 3);
}


/*
* Function Name: 	Sharp_GP2D12_estimation	
* Input: 			adc_reading (unsigned char) -> The digital value reading of Sharp sensor
* Output: 			distanceInt (unsigned int) -> Actual distance in mm
* Logic: 			Function to linearise ADC reading value to mm scale
* Example Call:		sharp = Sharp_GP2D12_estimation(ADC_value);
*/
// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


/*********************************************************************************************************/

//                                            TIMER FUNCTIONS                                            //

/*********************************************************************************************************/

/*
* Function Name: 	timer1_init	
* Input: 			None
* Output: 			None
* Logic: 			Function to initiate timer 1 for Servo control
					- TIMER1 initialization in 10 bit fast PWM mode  
					- Prescale:256
					- WGM: 7) PWM 10bit fast, TOP=0x03FF
					- Actual value: 52.25Hz 
* Example Call:		timer1_init();
*/
void timer1_init(void)
{
	TCCR1B = 0x00; 	//stop
	TCNT1H = 0xFC; 	//Counter high value to which OCR1xH value is to be compared with
	TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
	OCR1AH = 0x03;	//Output compare Register high value for servo 1
	OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
	OCR1BH = 0x03;	//Output compare Register high value for servo 2
	OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
	OCR1CH = 0x03;	//Output compare Register high value for servo 3
	OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
	ICR1H  = 0x03;	
	ICR1L  = 0xFF;
	TCCR1A = 0xAB; 	/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
	TCCR1C = 0x00;
	TCCR1B = 0x0C; 	//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


/*
* Function Name: 	timer3_init	
* Input: 			None
* Output: 			None
* Logic: 			Function to initiate timer 3 for Line Sensor Readings
					- TIMER3 initialize - prescale:256
					- WGM: 0) Normal, TOP=0xFFFF
					- Desired value: 100Hz
					- Actual value:  100.000Hz (0.0%)
* Example Call:		timer3_init();
*/
void timer3_init(void)
{
	TCCR3B = 0x00; 	//stop
	TCNT3  = 0xFDC0;// 0.01s
	OCR3AH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR3AL = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR3BH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR3BL = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR3CH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR3CL = 0x00; 	//Output Compair Register (OCR)- Not used
	ICR3H  = 0x00; 	//Input Capture Register (ICR)- Not used
	ICR3L  = 0x00; 	//Input Capture Register (ICR)- Not used
	TCCR3A = 0x00; 
	TCCR3C = 0x00;
	TCCR3B = 0x04; 	//Prsecaler 256 1-0-0
}


/*
* Function Name: 	timer4_init	
* Input: 			None
* Output: 			None
* Logic: 			Function to initiate timer 4 for Timing Operations
					- TIMER4 initialize - prescale:256
					- WGM: 0 Normal, TOP=0xFFFF
					- Desired value: 10Hz
					- Actual value:  10.000Hz (0.0%)
* Example Call:		timer4_init();
*/
void timer4_init(void)
{
	TCCR4B = 0x00; 	//stop
	TCNT4  = 0xE980;// 0.1s
	OCR4AH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR4AL = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR4BH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR4BL = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR4CH = 0x00; 	//Output Compair Register (OCR)- Not used
	OCR4CL = 0x00; 	//Output Compair Register (OCR)- Not used
	ICR4H  = 0x00; 	//Input Capture Register (ICR)- Not used
	ICR4L  = 0x00; 	//Input Capture Register (ICR)- Not used
	TCCR4A = 0x00; 
	TCCR4C = 0x00;
	TCCR4B = 0x04; 	//Prsecaler 256 1-0-0
}


/*
* Function Name: 	timer5_init	
* Input: 			None
* Output: 			None
* Logic: 			Function to initiate timer 5
					- Timer 5 initialized in PWM mode for velocity control
					- Prescale:256
					- PWM 8bit fast, TOP=0x00FF
					- Timer Frequency:225.000Hz
* Example Call:		timer5_init();
*/
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


/*********************************************************************************************************/

//                                           MOTION FUNCTIONS                                            //

/*********************************************************************************************************/

/*
* Function Name: 	motion_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to configure PORT A and PORT L to enable robot's motion
* Example Call:		motion_pin_config();
*/
void motion_pin_config (void) 
{
 	DDRA  = DDRA | 0x0F;
 	PORTA = PORTA & 0xF0;
 	DDRL  = DDRL | 0x18;   	//Setting PL3 and PL4 pins as output for PWM generation
 	PORTL = PORTL | 0x18; 	//PL3 and PL4 pins are for velocity control using PWM.
}


/*
* Function Name: 	left_encoder_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to configure INT4 (PORTE 4) pin as input for the left position encoder
* Example Call:		left_encoder_pin_config();
*/
void left_encoder_pin_config (void)
{
 	DDRE  = DDRE & 0xEF;  	//Set the direction of the PORTE 4 pin as input
 	PORTE = PORTE | 0x10; 	//Enable internal pull-up for PORTE 4 pin
}


/*
* Function Name: 	right_encoder_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call:		right_encoder_pin_config();
*/
void right_encoder_pin_config (void)
{
 	DDRE  = DDRE & 0xDF;  	//Set the direction of the PORTE 4 pin as input
 	PORTE = PORTE | 0x20; 	//Enable internal pull-up for PORTE 4 pin
}


/*
* Function Name: 	velocity	
* Input: 			left_motor (unsigned char) -> Left motor velocity
					right_motor (unsigned char) -> Right motor velocity
* Output: 			None
* Logic: 			PWM is used. 255 (0xFF)is set as the maximum value of the timer. When the count of the timer exceeds 
					255 the timer overflows. The TCNT and OCR5_n registers are compared and on match the timer overflows.
					For 100% Duty cycle, the value is 255. Hence to vary the duty cycle we can give any value in the range 0-255. 
* Example Call:		velocity(255,255);
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


/*
* Function Name: 	left_position_encoder_interrupt_init	
* Input: 			None
* Output: 			None
* Logic: 			Interrupt 4 enable
* Example Call:		left_position_encoder_interrupt_init();
*/
void left_position_encoder_interrupt_init (void)
{
 	cli(); 					//Clears the global interrupt
 	EICRB = EICRB | 0x02; 	// INT4 is set to trigger with falling edge
 	EIMSK = EIMSK | 0x10; 	// Enable Interrupt INT4 for left position encoder
 	sei();   				// Enables the global interrupt 
}


/*
* Function Name: 	right_position_encoder_interrupt_init	
* Input: 			None
* Output: 			None
* Logic: 			Interrupt 5 enable
* Example Call:		right_position_encoder_interrupt_init();
*/
void right_position_encoder_interrupt_init (void)
{
 	cli(); 					//Clears the global interrupt
 	EICRB = EICRB | 0x08; 	//INT5 is set to trigger with falling edge
 	EIMSK = EIMSK | 0x20; 	//Enable Interrupt INT5 for right position encoder
 	sei();   				//Enables the global interrupt 
}


/*
* Function Name: 	ISR	
* Input: 			INT5_vect
* Output: 			None
* Logic: 			ISR for right position encoder, increment right shaft position count
* Example Call:		Called automatically by interrupt
*/
ISR(INT5_vect)  
{
 	ShaftCountRight++;  //Increment right shaft position count
}


/*
* Function Name: 	ISR	
* Input: 			INT4_vect
* Output: 			None
* Logic: 			ISR for left position encoder, increment left shaft position count
* Example Call:		Called automatically by interrupt
*/
ISR(INT4_vect)
{
 	ShaftCountLeft++;  	//Increment left shaft position count
}


/*
* Function Name: 	motion_set	
* Input: 			Direction (unsigned char) -> Direction HEX

					- 0x06 --> Forward	
                    - 0x09 --> Backward
                    - 0x05 --> Left
                    - 0x0A --> Right
                    - 0x04 --> Soft Left
                    - 0x02 --> Soft right
                    - 0x01 --> Soft Left 2 (Reverse logic of soft left)
                    - 0x08 --> Soft Right 2 (Reverse logic of soft right)
                    - 0x00 --> Stop

* Output: 			None
* Logic: 			Assigns the motor to move in specified direction. The motor consists of two inputs.
					Based on polarity of the inputs, it rotates clockwise or anticlockwise.
* Example Call:		motion_set(0x01);
*/
void motion_set (unsigned char Direction)
{
 	unsigned char PortARestore = 0;
	
 	Direction &= 0x0F; 			// removing upper nibbel for the protection
 	PortARestore = PORTA; 		// reading the PORTA original status
 	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 	PortARestore |= Direction; 	// adding lower nibbel for forward command and restoring the PORTA status
 	PORTA = PortARestore; 		// executing the command
}


/*
* Function Name: 	forward	
* Input: 			None
* Output: 			None
* Logic: 			Both wheels forward
* Example Call:		forward();
*/
void forward (void) 
{
  	motion_set (0x06);
}


/*
* Function Name: 	back	
* Input: 			None
* Output: 			None
* Logic: 			Both wheels backward
* Example Call:		back();
*/
void back (void) 			
{
  	motion_set(0x09);
}


/*
* Function Name: 	left	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel backward, Right wheel forward
* Example Call:		left();
*/
void left (void)			
{
  	motion_set(0x05);
}


/*
* Function Name: 	right	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel forward, Right wheel backward
* Example Call:		right();
*/
void right (void) 		
{
  	motion_set(0x0A);
}


/*
* Function Name: 	soft_left	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel stationary, Right wheel forward
* Example Call:		soft_left();
*/
void soft_left (void) 		
{
 	motion_set(0x04);
}


/*
* Function Name: 	soft_right	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel forward, Right wheel is stationary
* Example Call:		soft_right();
*/
void soft_right (void) 		
{
 	motion_set(0x02);
}


/*
* Function Name: 	soft_left_2	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel backward, right wheel stationary
* Example Call:		soft_left_2();
*/
void soft_left_2 (void) 	
{
 	motion_set(0x01);
}


/*
* Function Name: 	soft_right_2	
* Input: 			None
* Output: 			None
* Logic: 			Left wheel stationary, Right wheel backward
* Example Call:		soft_right_2();
*/
void soft_right_2 (void) 	
{
 	motion_set(0x08);
}


/*
* Function Name: 	stop 	
* Input: 			None
* Output: 			None
* Logic: 			Both wheels stop
* Example Call:		stop():
*/
void stop (void)
{
  	motion_set (0x00);
}


/*
* Function Name: 	angle_rotate	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot moves 4.090 degress per count. Hence, it calculates required shaft count by 
					dividing by 4.090 for moving by specified degrees.
* Example Call:		angle_rotate(180);
*/
void angle_rotate(unsigned int Degrees)
{
 	float ReqdShaftCount = 0;
 	unsigned long int ReqdShaftCountInt = 0;

 	ReqdShaftCount = (float) Degrees/ 4.090; //Division by resolution to get shaft count
 	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 	ShaftCountRight = 0; 
 	ShaftCountLeft = 0; 

 	while (1)
 	{
  		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  		break;
 	}
 	stop(); //Stop robot
}


/*
* Function Name: 	linear_distance_mm	
* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance
* Output: 			None
* Logic: 			The robot moves 5.338mm per pulse. Hence,it calculates required shaft count by dividing by 
					5.338 for moving to a specified distance.
* Example Call:		linear_distance_mm(100);
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
 	float ReqdShaftCount = 0;
 	unsigned long int ReqdShaftCountInt = 0;

 	ReqdShaftCount = DistanceInMM / 5.338; //Division by resolution to get shaft count
 	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	  
 	ShaftCountRight = 0;
 	while(1)
 	{
  		if(ShaftCountRight > ReqdShaftCountInt)
  		{
  			break;
  		}
 	} 
 	stop(); //Stop robot
}


/*
* Function Name: 	forward_mm	
* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance
* Output: 			None
* Logic: 			The robot moves forward for the specified distance
* Example Call:		forward_mm(100);
*/
void forward_mm(unsigned int DistanceInMM)
{
 	forward();
 	linear_distance_mm(DistanceInMM);
}


/*
* Function Name: 	back_mm	
* Input: 			DistanceInMM (unsigned int) -> Traverse by specified distance
* Output: 			None
* Logic: 			The robot moves backward for the specified distance
* Example Call:		back_mm(100);
*/
void back_mm(unsigned int DistanceInMM)
{
 	back();
 	linear_distance_mm(DistanceInMM);
}


/*
* Function Name: 	left_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot rotates left for the specified degrees
* Example Call:		left_degrees(90);
*/
void left_degrees(unsigned int Degrees) 
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
 	left(); //Turn left
 	angle_rotate(Degrees);
}


/*
* Function Name: 	right_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot rotates right for the specified degrees
* Example Call:		right_degrees(90);
*/
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
 	right(); //Turn right
 	angle_rotate(Degrees);
}


/*
* Function Name: 	soft_left_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot takes a soft left for the specified degrees
* Example Call:		soft_left_degrees(90);
*/
void soft_left_degrees(unsigned int Degrees)
{
 	// 176 pulses for 360 degrees rotation 2.045 degrees per count
 	soft_left(); //Turn soft left
 	Degrees=Degrees*2;
 	angle_rotate(Degrees);
}


/*
* Function Name: 	soft_right_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot takes a soft right for the specified degrees
* Example Call:		soft_right_degrees(90);
*/
void soft_right_degrees(unsigned int Degrees)
{
 	// 176 pulses for 360 degrees rotation 2.045 degrees per count
 	soft_right();  //Turn soft right
 	Degrees=Degrees*2;
 	angle_rotate(Degrees);
}


/*
* Function Name: 	soft_left_2_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot takes a reverse soft left for the specified degrees
* Example Call:		soft_left_2_degrees(90);
*/
void soft_left_2_degrees(unsigned int Degrees)
{
 	// 176 pulses for 360 degrees rotation 2.045 degrees per count
 	soft_left_2(); //Turn reverse soft left
 	Degrees=Degrees*2;
 	angle_rotate(Degrees);
}


/*
* Function Name: 	soft_right_2_degrees	
* Input: 			Degrees (unsigned int) -> Rotate by specified degrees
* Output: 			None
* Logic: 			The robot takes a reverse soft right for the specified degrees
* Example Call:		soft_right_2_degrees(90);
*/
void soft_right_2_degrees(unsigned int Degrees)
{
 	// 176 pulses for 360 degrees rotation 2.045 degrees per count
 	soft_right_2();  //Turn reverse soft right
 	Degrees=Degrees*2;
 	angle_rotate(Degrees);
}


/*
* Function Name: 	turn	
* Input: 			mode (int) -> Operation mode of the function
* Output: 			None
* Logic: 			mode = 0 -> Turn Left
         			mode = 1 -> Turn Right
* Example Call:		turn(1);
*/
void turn(int mode)
{
	if(mode==0) 	 left_degrees(88);
	else if(mode==1) right_degrees(90);
}


/*********************************************************************************************************/

//                                          SERVOMOTOR FUNCTIONS                                         //

/*********************************************************************************************************/

/*
* Function Name: 	servo1_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Configure PORTB 5 pin for servo motor 1 operation
* Example Call:		servo1_pin_config();
*/
void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}


/*
* Function Name: 	servo2_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Configure PORTB 6 pin for servo motor 2 operation
* Example Call:		servo2_pin_config();
*/
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}


/*
* Function Name: 	servo3_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Configure PORTB 7 pin for servo motor 3 operation
* Example Call:		servo3_pin_config();
*/
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}


/*
* Function Name: 	servo_1	
* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate
* Output: 			None
* Logic: 			Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
* Example Call:		servo_1(60);
*/
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
  	PositionPanServo = ((float)degrees / 1.86) + 35.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}


/*
* Function Name: 	servo_2	
* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate
* Output: 			None
* Logic: 			Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
* Example Call:		servo_2(100);
*/
void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}


/*
* Function Name: 	servo_3	
* Input: 			degrees (unsigned char) -> The angle by which the servo should rotate
* Output: 			None
* Logic: 			Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
* Example Call:		servo_3(160);
*/
void servo_3(unsigned char degrees)
{
 	float PositionServo = 0;
 	PositionServo = ((float)degrees / 1.86) + 35.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionServo;
}


/* 
Servo_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. 
This function can be used to reduce the power consumption of the motor if it is holding load against the gravity.
*/

/*
* Function Name: 	servo_1_free	
* Input: 			None
* Output: 			None
* Logic: 			Makes servo 1 free rotating
* Example Call:		servo_1_free():
*/


void servo_1_free (void)
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; //Servo 1 off
}


/*
* Function Name: 	servo_2_free	
* Input: 			None
* Output: 			None
* Logic: 			Makes servo 2 free rotating
* Example Call:		servo_2_free();
*/
void servo_2_free (void)
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; //Servo 2 off
}


/*
* Function Name: 	servo_3_free	
* Input: 			None
* Output: 			None
* Logic: 			Makes servo 3 free rotating
* Example Call:		servo_3_free();
*/
void servo_3_free (void)
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; //Servo 3 off
}


/*
* Function Name: 	color_arm_up	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 2 by 210 degrees which maintains the arm mechanism in upright position for gas detection
* Example Call:		color_arm_up();
*/
void color_arm_up()
{
	_delay_ms(50);
	servo_2(210);
	_delay_ms(250);
	servo_2_free();
	_delay_ms(50);	
}


/*
* Function Name: 	color_arm_down	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 2 by 60 degrees which brings the arm mechanism down for valve detection
* Example Call:		color_arm_down();
*/
void color_arm_down()
{
	_delay_ms(50);
	servo_2(60);
	_delay_ms(250);
	servo_2_free();
	_delay_ms(50);	
}


/*
* Function Name: 	color_arm_90	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 2 by 100 degrees which brings the arm mechanism to 90 degrees for central walls detection
* Example Call:		color_arm_90();
*/
void color_arm_90()
{
	_delay_ms(50);
	servo_2(100);
	_delay_ms(200);
	servo_2_free();
	_delay_ms(50);	
}


/*
* Function Name: 	servo_3_front	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 3 by 12 degrees which orients the sharp sensor mounted on the servo to the front
* Example Call:		servo_3_front():
*/
void servo_3_front()
{
	_delay_ms(50);
	servo_3(12);
	_delay_ms(400);
	servo_3_free();
	_delay_ms(50);
}


/*
* Function Name: 	servo_3_left	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 3 by 93 degrees which orients the sharp sensor mounted on the servo to the left
* Example Call:		servo_3_left():
*/
void servo_3_left()
{
	_delay_ms(50);
	servo_3(93);
	_delay_ms(250);
	servo_3_free();
	_delay_ms(50);
}


/*
* Function Name: 	servo_3_45	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 3 by 40 degrees which orients the sharp sensor mounted on the servo to an angle of 45 degrees
* Example Call:		servo_3_45();
*/
void servo_3_45()
{
	_delay_ms(50);
	servo_3(40);
	_delay_ms(250);
	servo_3_free();
	_delay_ms(50);	
}


/*
* Function Name: 	servo_3_back	
* Input: 			None
* Output: 			None
* Logic: 			Function to move the servo 3 by 188 degrees which orients the sharp sensor mounted on the servo to the back
* Example Call:		servo_3_back():
*/
void servo_3_back()
{	
	_delay_ms(50);
	servo_3(188);
	_delay_ms(400);
	servo_3_free();
	_delay_ms(50);
}


/*
* Function Name: 	servo_initial_positions	
* Input: 			None
* Output: 			None
* Logic: 			Function to orient all the servos to their initial positions
* Example Call:		servo_initial_positions();
*/
void servo_initial_positions()
{
	_delay_ms(50);
	color_arm_90();
	servo_3_left();

	_delay_ms(50);		
	servo_1(0);
	_delay_ms(1000);
	servo_1_free();
}


/*********************************************************************************************************/

//                                          COLOR SENSOR FUNCTIONS                                       //

/*********************************************************************************************************/

/*
* Function Name: 	color_sensor_pin_config	
* Input: 			None
* Output: 			None
* Logic: 			Function to initialise color sensor PORTD
* Example Call:		color_sensor_pin_config();
*/
void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; 	//Set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;	//Enable internal pull-up for PORTD 0 pin
}


/*
* Function Name: 	color_sensor_pin_interrupt_init	
* Input: 			None
* Output: 			None
* Logic: 			Function to initialise color sensor interrupts
* Example Call:		color_sensor_pin_interrupt_init();
*/
void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); 					//Clears the global interrupt
	EICRA = EICRA | 0x02; 	//INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; 	//Enable Interrupt INT0 for color sensor
	sei(); 					//Enables the global interrupt
}


/*
* Function Name: 	ISR	
* Input: 			INT0_vect
* Output: 			None
* Logic: 			ISR for color sensor
* Example Call:		Called automatically by the interrupt
*/
ISR(INT0_vect)
{
	pulse++; //Increment on receiving pulse from the color sensor
}


/*
* Function Name: 	filter_red
* Input: 			None
* Output: 			None
* Logic: 			Used to select red filter
* Example Call:		filter_red();
*/
void filter_red(void)  
{
	//Filter Select - red filter
	PORTD = PORTD & 0xBF; 	//set S2 low
	PORTD = PORTD & 0x7F; 	//set S3 low
}


/*
* Function Name: 	filter_green	
* Input: 			None
* Output: 			None
* Logic: 			Used to select green filter
* Example Call:		filter_green():
*/
void filter_green(void)
{
	//Filter Select - green filter
	PORTD = PORTD | 0x40; 	//set S2 High
	PORTD = PORTD | 0x80; 	//set S3 High
}


/*
* Function Name: 	filter_blue	
* Input: 			None
* Output: 			None
* Logic: 			Used to select blue filter
* Example Call:		filter_blue():
*/
void filter_blue(void)	
{
	//Filter Select - blue filter
	PORTD = PORTD & 0xBF; 	//set S2 low
	PORTD = PORTD | 0x80; 	//set S3 High
}


/*
* Function Name: 	color_sensor_scaling	
* Input: 			None
* Output: 			None
* Logic: 			Function to scale the output frequency to 2%
* Example Call:		color_sensor_scaling():
*/
void color_sensor_scaling()
{
	//Output Scaling 2% from datasheet
	PORTD = PORTD & 0xEF; 	//set S0 low
	PORTD = PORTD | 0x20; 	//set S1 high
}


/*
* Function Name: 	color_sensor_power_down	
* Input: 			None
* Output: 			None
* Logic: 			Function to disable the  color sensor when not in use
* Example Call:		color_sensor_power_down():
*/
void color_sensor_power_down()
{	
	//Power Down
	PORTD = PORTD & 0xEF; 	//set S0 low
	PORTD = PORTD & 0xDF; 	//set S1 low
}


/*
* Function Name: 	red_read	
* Input: 			None
* Output: 			None
* Logic: 			Function to read the pulses received from the red filter
* Example Call:		red_read():
*/
void red_read(void)
{
	//Red
	filter_red(); 		//select red filter
	pulse=0; 			//reset the count to 0
	_delay_ms(100); 	//capture the pulses for 100 ms or 0.1 second
	red = pulse;  		//store the count in variable called red
}


/*
* Function Name: 	green_read	
* Input: 			None
* Output: 			None
* Logic: 			Function to read the pulses received from the green filter
* Example Call:		green_read():
*/
void green_read(void)
{
	//Green
	filter_green(); 	//select green filter
	pulse=0; 			//reset the count to 0
	_delay_ms(100); 	//capture the pulses for 100 ms or 0.1 second
	green = pulse;  	//store the count in variable called green
}


/*
* Function Name: 	blue_read	
* Input: 			None
* Output: 			None
* Logic: 			Function to read the pulses received from the blue filter
* Example Call:		blue_read():
*/
void blue_read(void)
{
	//Blue
	filter_blue(); 		//select blue filter
	pulse=0; 			//reset the count to 0
	_delay_ms(100); 	//capture the pulses for 100 ms or 0.1 second
	blue = pulse;  		//store the count in variable called blue
}


/*
* Function Name: 	read_color	
* Input: 			None
* Output: 			None
* Logic: 			Function to read all the color pulses successively
* Example Call:		read_color();
*/
void read_color()
{
	red_read();
	green_read();
	blue_read();
}


/*
* Function Name: 	display_color	
* Input: 			None
* Output: 			None
* Logic: 			Function to display color pulses on LCD
* Example Call:		display_color():
*/
void display_color()
{
	lcd_print(1,1,red,3);
	lcd_print(1,5,green,3);
	lcd_print(1,9,blue,3);
}


/*
* Function Name: 	get_color	
* Input:			mode (int) -> Operation mode of the function
* Output:			color_code (int) -> Returns the detected color code
					
					Color Codes:
					0 -> Black
					1 -> Red
					2 -> Green
					3 -> Blue

* Logic:			- mode = 1 -> Gas Leakage Color Detection
					- mode = 0 -> Control Valve Color Detection
					- If mode = 1, check color of central walls (black or white). If central walls are white, 
					  it's a valid leakage zone. Then move color sensor arm up, move slightly inside the leakage zone
					  and detect the corresponding color based on the different pulses obtained
					- If mode = 0, detect the corresponding color of valve, based on the different pulses obtained
					- Indicate the color of valve/gas leakage using RGB LED
					- Return the color code and power down color sensor
* Example Call:		gas_color = get_color(1);
*/
int get_color(int mode)
{	
	int color_code, leakage_zone = 0; //leakage_zone = 0 for No Leakage Zone, else 1

	led_off();
	color_sensor_scaling(); //2% Scaling
	read_color();			//Read all color pulses
	
	//Check if mode = 1 for gas leakage detection
	if(mode)
	{	
		//Check if central walls are white
		if(red>25 && green>25 && blue>25) 
		{	
			//No black chart paper -> Detect LED color
			color_arm_up();
			leakage_zone = 1;	
		}

		//Black chart paper present -> No Leakage Zone
		else
		{
			color_arm_up();
			color_sensor_power_down();
				
			return 0; //Return 0 -> No Leakage Zone			
		}
	}
	
	//If gas leakage zone, move slightly inside the zone for better color detection
	if(leakage_zone)
	{	
		forward_mm(50);
		soft_left_degrees(10);
		_delay_ms(100);
	}
	
	//Color Detection of valves and LEDs in Gas Leakage Zones
	if(leakage_zone || mode==0)
	{	
		//Special condition for Green LED Detection
		if(red>30 && red<85 && green>30 && green<75 && blue>30 && blue<85 && abs(green-blue)<=8 && abs(green-red)<=8)
	    {	
			color_code = 2;	
			led_grn();		//Switch on green LED
	    }

		//Red color condition for valves and LEDs
	    else if (red>=26 && red>green && red>blue)
	    {
			color_code = 1;	
			led_red();		//Switch on red LED
	    }

		//Green color condition for valves and LEDs
	    else if (green>=21 && green>red && green>blue)
	    {
			color_code = 2;	
			led_grn();		//Switch on green LED
	    }

		//Blue color condition for valves and LEDs
	    else if (blue>=19 && blue>=red && blue>green)
	    {
			color_code = 3;	
			led_blu();		//Switch on blue LED
	    }

		//Else black valve detected condition
	    else
	    {
			color_code = 0;	//Black
	    }
	}

	color_sensor_power_down();	

	return color_code; //Return color code 
}


/*********************************************************************************************************/

//                                       SENSOR READING FUNCTIONS                                        //

/*********************************************************************************************************/

/*
* Function Name:	ISR 	
* Input: 			TIMER3_OVF_vect			
* Output: 			None
* Logic:			- Timer 3 ISR is used to update Line Sensor Readings at 100Hz
					- Based on readings, determine the line condition
* Example Call:		Called automatically by timer interrupt
*/
ISR(TIMER3_OVF_vect)
{
	read_line();
	line_conditions();
	TCNT3 = 0xFDC0;	 
}


/*
* Function Name: 	ISR	
* Input: 			TIMER4_OVF_vect	
* Output: 			None
* Logic:			- Timer 4 ISR is used for timing operations at a resolution of 10Hz
					- t4_sec is incremented every 0.1 sec until it is equal to required t4_count
* Example Call:		Called automatically by timer interrupt
*/
ISR(TIMER4_OVF_vect)
{
	TCNT4 = 0xE980;
	t4_sec++; //Increment every 0.1 sec
	
	if(t4_sec==t4_count && t4_flag==1) //If required count is reached stop timer 4
	stop_timer4();
}
 

/*
* Function Name: 	start_timer4	
* Input: 			count (int) -> Required number of time units in 0.1 sec to count
* Output: 			None
* Logic:			- Enable Timer 4 overflow interrupt
					- Initialise Timer 4 flags (global variables)
* Example Call:		start_timer4(20); -> This will count for 2 sec and stop
*/
void start_timer4(int count)
{
	t4_flag = 1;	//Indicate timer 4 is enabled
	t4_sec = 0;
	t4_count = count+1;
	TIMSK4 = 0x01; //Timer 4 overflow interrupt enable
}


/*
* Function Name: 	stop_timer4	
* Input: 			None
* Output: 			None
* Logic:			- If t4_sec reaches required t4_count, stop timer 4
					- Disable Timer 4 overflow interrupt
					- Also used for turning off buzzer if it is on
* Example Call:		stop_timer4();
*/
void stop_timer4()
{
	t4_flag = 0;
	t4_sec = 0;
	TIMSK4 = 0x00; //Timer4 overflow interrupt disable

	//Switch off buzzer if it is on
	if(buzzer_flag)
	{
		buzzer_off();
		buzzer_flag = 0;
	}
}


/*
* Function Name:	enable_flags 	
* Input: 			l_flag (int) -> 0 - Disable / 1 -> Enable
					n_flag (int) -> 0 - Disable / 1 -> Enable
					r_flag (int) -> 0 - Disable / 1 -> Enable 
* Output: 			None
* Logic:			- Function to enable/disable left/right turn or node detection
* Example Call:		enable_flags(0,1,0); -> Detect only nodes, ignore left and right turns
*/
void enable_flags(int l_flag, int n_flag, int r_flag)
{
	en_left  = l_flag;
	en_right = r_flag;
	en_node  = n_flag;
}


/*
* Function Name: 	read_sharp	
* Input: 			mode (int) -> Operation mode of the function		
* Output: 			Returns sharp_1 or sharp_2 distance value
* Logic:			- Read front and right Sharp sensor values, linearise them to mm scale
					- mode = 0 -> Return front Sharp sensor value
					- mode = 1 -> Return right Sharp sensor value
					- mode = 2 -> Only read and store both Sharp sensor values as global variables
* Example Call:		distance = read_sharp(1); -> Returns right Sharp sensor value
*/
int read_sharp(int mode)
{
	sharp_1 = sharp_2 = 0;

	sharp1 	= ADC_Conversion(11);						
	sharp_1 = (unsigned int)(Sharp_GP2D12_estimation(sharp1)/1.17);

	sharp2 	= ADC_Conversion(13);						
	sharp_2 = (unsigned int)(Sharp_GP2D12_estimation(sharp2)/2.55);

	if(mode==0)			return sharp_1;
	else if(mode==1) 	return sharp_2;
	else				return 0;
}


/*
* Function Name: 	display_sharp	
* Input: 			None
* Output: 			None
* Logic:			Display Sharp sensor values on LCD
* Example Call:		display_sharp();
*/
void display_sharp()
{
	lcd_print(2, 1, sharp_1, 3);
	lcd_print(2, 5, sharp_2, 3);
}


/*********************************************************************************************************/

//                                         LINE FOLLOWER FUNCTIONS                                       //

/*********************************************************************************************************/

/*
* Function Name: 	read_line	
* Input: 			None
* Output: 			None
* Logic:			Read left, center and right white line sensor values and store in global variables
* Example Call:		read_line();
*/
void read_line()
{
	Left   = (int)ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center = (int)ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right  = (int)ADC_Conversion(1);	//Getting data of Right WL Sensor
}


/*
* Function Name: 	display_line	
* Input: 			None		
* Output: 			None
* Logic:			Display line sensor readings on LCD
* Example Call:		display_line();
*/
void display_line()
{	
	lcd_print(1, 1, Left, 	3);
	lcd_print(1, 5, Center, 3);
	lcd_print(1, 9, Right, 	3);	
}


/*
* Function Name: 	clear_count 	
* Input: 			None
* Output: 			None
* Logic:			Clear all line follower flags and turn/node counts
* Example Call:		clear_count();
*/
void clear_count()
{
	node = 0;
	left_turn = 0;
	right_turn = 0;

	node_flag = 0;
	left_flag = 0;
	right_flag = 0;

	correct_left_flag = 1;
	correct_right_flag = 1;
}


/*
* Function Name: 	line_conditions	
* Input: 			None
* Output: 			None
* Logic:			- Based on the position of robot on line, the line condition flag is set
					- NOTE: This functions only checks the condition and sets the line follower flags, doesn't follow the line
					- Line following and Line detection are done seperately, in order to increase the sampling rate of the 
					  sensors so that the sensors do not miss out nodes and turns when the robot is moving at higher velocities
* Example Call:		line_conditions();	
*/
void line_conditions()
{	
	// Nodes Condition
	if(Center>=127 || Left>=127 || Right>=127 || (Center>=120 && (Left>=120 || Right>=120)))// 1 1 1
	{	
		line_cond = 1;
		node_flag = 1;
	}
	
	//Right Turn Condition
	else if((Center>12 && Center<130) && (Right>12 && Right<130) && Left<20)//0 1 1 
	{	
		line_cond = 2;
		right_flag = 1;
	}
	
	//Left Turn Condition
	else if((Center>12 && Center<130) && (Left>12 && Left<130) && Right<20)// 1 1 0
	{	
		line_cond = 3;
		left_flag = 1;
	}
	
	//Straight Line Condition
	else if(Center>=10 && Left<10 && Right<10) // 0 1 0
	{		
		line_cond = 4;	
	}
	
	//Straight Line Condition (Robot moved slightly towards right)
	else if(Left>=10 && Center<10 && Right<10)// 1 0 0
	{
		line_cond = 5;
	}
	
	//Straight Line Condition (Robot moved slightly towards left)
	else if(Right>=10 && Center<10 && Left<10)// 0 0 1
	{
		line_cond = 6;
	}
	
	//No Line Condition
	else if(Center<10 && Left<10 && Right<10)// 0 0 0 
	{	
		line_cond = 7;
	}
}


/*
* Function Name:   	correct_left	
* Input: 			mode (int) -> Operation mode of the function			
* Output: 			None	
* Logic:			- mode = 1 -> Keep rotating left till center sensor see line
					- mode = 2 -> Keep rotating left till left sensor see line
* Example Call:		correct_left(1);
*/
void correct_left(int mode)
{
	stop();
	_delay_ms(200);
	while(1)	
	{		
		left();	
		
		//Stop if center sensor sees line
		if(Center>12 && mode==1)
		{	
			stop();
			return;
		}

		//Stop if left sensor sees line
		else if(Left>12 && mode==2)
		{
			left_degrees(12);
			stop();
			return;
		}
	}
}


/*
* Function Name:   	correct_right	
* Input: 			mode (int) -> Operation mode of the function			
* Output: 			None	
* Logic:			- mode = 1 -> Keep rotating right till center sensor see line
					- mode = 2 -> Keep rotating right till right sensor see line
* Example Call:		correct_right(2);
*/
void correct_right(int mode)
{
	stop();
	_delay_ms(200);
	while(1)	
	{		
		right();	
		
		//Stop if center sensor sees line
		if(Center>12 && mode==1)
		{	
			stop();
			return;
		}

		//Stop if right sensor sees line
		else if(Right>12 && mode==2)
		{
			right_degrees(14);
			stop();
			return;
		}
	}
}


/*
* Function Name: 	follow_line	
* Input: 			None
* Output: 			None
* Logic:			- Follow the line based on the line conditions and turn flags set
					- Line following and Line detection are done seperately, in order to increase the 
					  sampling rate of the sensors so that the sensors do not miss out nodes and turns
					  when the robot is moving at higher velocities
					- To avoid duplicate detection of nodes and turns, after the first detection, timer 4 is enabled for
					  a short duration (0.5 sec). As long as the timer 4 is running, duplicate detections will be discarded.
* Example Call:		follow_line();
*/
void follow_line()
{	
	velocity(left_mid, right_mid);
	
	//If timer 4 is on ignore all nodes and turns detected
	if(t4_flag == 1)
	{
		node_flag = 0;
		left_flag = 0;
		right_flag = 0;
	}

	//Nodes
	if(node_flag == 1)
	{	
		node++; 					//Increment node count
		if(!en_node)				//If node detection is disabled, skip node operation	
		{	
			node_flag = 0;
			start_timer4(5);		//Start timer 4 for 0.5 sec to avoid duplicate detection
			return;
		}

		forward();
		node_flag = 0;
		start_timer4(5); 			//Start timer 4 for 0.5 sec to avoid duplicate detection
	}
	
	//Right Turn
	else if(right_flag == 1)
	{	
		right_turn++;				//Increment right turn count
		if(!en_right)				//If right turn detection is disabled, skip turn operation
		{	
			if(en_node) node++; 	//Count as node, in case node is detected as right turn when en_right is disabled
			correct_right_flag = 0; //Avoid right correction of robot for a short duration
			right_flag = 0;
			start_timer4(5);		//Start timer 4 for 0.5 sec to avoid duplicate detection
			return;
		}
		
		forward_mm(65);				//Move slightly forward before turning
		correct_right(2);			//Turn right till right line sensor sees the line
		_delay_ms(200);
		
		right_flag = 0;
		start_timer4(5);			//Start timer 4 for 0.5 sec to avoid duplicate detection
	}
	
	//Left Turn
	else if(left_flag == 1)
	{	
		//buzz(50);
		left_turn++;				//Increment left turn count
		if(!en_left)				//If left turn detection is disabled, skip turn operation
		{	
			if(en_node) node++;		//Count as node, in case node is detected as left turn when en_left is disabled
			correct_left_flag = 0;	//Avoid left correction of robot for a short duration
			left_flag = 0;
			start_timer4(5);		//Start timer 4 for 0.5 sec to avoid duplicate detection
			return;
		}
		
		forward_mm(65);				//Move slightly forward before turning
		correct_left(2);			//Turn left till left line sensor sees the line
		_delay_ms(200);

		left_flag = 0;
		start_timer4(5);			//Start timer 4 for 0.5 sec to avoid duplicate detection
	}
	
	// Straight Path
	else if(line_cond == 4)	forward(); //Just move forward
	
	//Move slightly left to center the robot on line
	else if(line_cond == 5 && correct_left_flag)	
	{	
		stop();
		_delay_ms(150);
		left_degrees(7);
		_delay_ms(150);
	}

	//Move slightly right to center the robot on line
	else if(line_cond == 6 && correct_right_flag)
	{
		stop();
		_delay_ms(150);
		right_degrees(7);
		_delay_ms(150);
	}

	//If no line is found, just move forward
	else if(line_cond == 7)	forward();
	else forward();
}


/*
* Function Name: 	line_follower	
* Input: 			left_no (int)  -> Follow line till required number of left turns
					node_no (int)  -> Follow line till required number of nodes
					right_no (int) -> Follow line till required number of right turns
* Output: 			None
* Logic:			Function to follow the line upto to a specific number of turns/nodes
* Example Call:		line_follower(0,0,2); -> Follows the line and stops after 2 right turns are encountered
*/
void line_follower(int left_no, int node_no, int right_no)
{	
	clear_count();	//Clear line follower flags
	TIMSK3 = 0x01; 	//Enable Line Sensor Readings	
	
	//Follow line till required number of nodes/turns are encountered
	if(node_no>0) 		while(node<node_no) 		follow_line();
	else if(left_no>0) 	while(left_turn<left_no) 	follow_line();
	else if(right_no>0) while(right_turn<right_no) 	follow_line();

	stop();
	TIMSK3 = 0x00;	//Disable Line Sensor Readings		
}


/*
* Function Name: 	search_line
* Input: 			mode (int) -> Operation mode of the function	
* Output: 			None
* Logic:			- Keeps moving forward until a line is seen by the WL sensors
					- mode = 0 -> Takes left turn after line found
					- mode = 1 -> Takes right turn after line found
* Example Call:		search_line();
*/
void search_line(int mode)
{	
	clear_count();
	TIMSK3 = 0x01; 	//Enable Line Sensor Readings
	
	velocity(left_max, right_max);
	forward();

	while(1)
	{	
		//Stop if line is found
		if(Left>100 || Center>100 || Right>100)	
		{
			stop();
			TIMSK3 = 0x00;
			break;
		}
		forward();
	}

	forward_mm(65);	//Move slightly forward before turning
	
	TIMSK3 = 0x01;	//Enable Line Sensor Readings
	if(mode==0) 	 correct_left(1); 	//Turn Left
	else if(mode==1) correct_right(1);	//Turn Right	
	TIMSK3 = 0x00;	//Disable Line Sensor Readings
}


/*********************************************************************************************************/

//                                         WALL FOLLOWER FUNCTIONS                                       //

/*********************************************************************************************************/

/*
* Function Name:	check_wall	
* Input: 			None
* Output: 			None
* Logic: 			- Reads Sharp sensor values on both sides and stores half wall configuration in global 
					  variable wall_present 1 -> Left // 2 -> Right // 3-> Both // 0 -> None
* Example Call:		check_wall();	
*/
void check_wall()
{
	read_sharp(2);	//Read both Sharp sesnor values

	if(sharp_1 < 150)				 wall_present = 1; 	//Left Wall Present
	if(sharp_2 < 150)				 wall_present = 2;	//Right Wall Present
	if(sharp_1<150 && sharp_2<150)	 wall_present = 3;	//Bothe Walls Present
	if(sharp_1>=150 && sharp_2>=150) wall_present = 0;	//None Present
}


/*
* Function Name: 	follow_left_wall	
* Input: 			mm (int) -> Distance in mm at which the wall should be followed
* Output: 			None
* Logic: 			- Follows a wall on left at a distance 'mm'
					- If distance < mm, move slightly right
					- If distance > mm, move slightly left
* Example Call:		follow_left_wall(100);
*/
void follow_left_wall(int mm)
{	
	if(sharp_1>(mm-5) && sharp_1<(mm+5)) //Go straight
	{
		velocity(left_max, right_max);
		forward();
	}
	else if(sharp_1 <= (mm-5))			//Move right slightly
	{
		velocity(255,230);
		forward();
	}
	else if(sharp_1 >= (mm+5))			//Move left slightly		
	{
		velocity(230,255);
		forward();
	}				
}


/*
* Function Name: 	follow_right_wall	
* Input: 			mm (int) -> Distance in mm at which the wall should be followed
* Output: 			None
* Logic: 			- Follows a wall on right at a distance 'mm'
					- If distance > mm, move slightly right
					- If distance < mm, move slightly left
* Example Call:		follow_right_wall(100);
*/
void follow_right_wall(int mm)
{
	if(sharp_2>(mm-3) && sharp_2<(mm+5)) //Go straight
	{
		velocity(left_max, right_max);
		forward();
	}
	else if(sharp_2 <= (mm-3))			//Move left slightly
	{
		velocity(230,255);
		forward();
	}
	else if(sharp_2>=(mm+5) && sharp_2<=(mm+30))
	{
		velocity(255,230);				//Move right slightly
		forward();
	}
	else if(sharp_2 > (mm+30))
	{
		velocity(255,200);				//Move right little more
		forward();
	}
}


/*
* Function Name:	follow_right_wall2 	
* Input: 			None		
* Output: 			None
* Logic: 			Follows a wall on right at 90mm till the central wall in front is detected at 125mm
* Example Call:		follow_right_wall2()	
*/
void follow_right_wall2()
{	
	stop();
	_delay_ms(100);

	servo_3_45(); //Rotate Servo 3 to 45 degrees to detect central walls, which are kept at an angle
				  //NOTE: Sharp sensor 1 is mounted on Servo 3	
	while(1)
	{
		if(read_sharp(0) <= 125) //Read Sharp sensor 1
		{	
			stop();
			return;
		}		
		follow_right_wall(90);	//Follow right wall at 90mm						
	}
}

/*********************************************************************************************************/

//                                         DROP MECHANISM FUNCTIONS                                      //

/*********************************************************************************************************/


/*
* Function Name: 	drop_align_front_wall	
* Input: 			None
* Output: 			None
* Logic: 			Align the robot with the front wall before taking a turn near the control valves
* Example Call: 	drop_align_front_wall();	
*/
void drop_align_front_wall()
{
	forward_mm(50);	//Move forward 50mm	
	_delay_ms(200);

	TIMSK3 = 0x01;
	correct_left(1); //Rotate left till center line sensor sees line
	TIMSK3 = 0x00;
}


/*
* Function Name:	drop_align1 	
* Input: 			None
* Output: 			None
* Logic: 			Align the robot with the wall behind at 250mm
* Example Call:		drop_align1();
*/
void drop_align1()
{
	servo_3_back(); //Rotate Servo 3/Sharp sensor 1 back
	_delay_ms(50);

	right_degrees(12); //Turn right slightly
	_delay_ms(150);
	
	//Align the robot at a distance of 250mm with the wall at the back
	int mm = 250;
	read_sharp(2);
	if(sharp_1<(mm-1)) 		forward_mm(mm - sharp_1);
	else if(sharp_1>(mm+1)) back_mm(sharp_1 - mm);
}


/*
* Function Name:	drop_magnet() 	
* Input: 			None
* Output: 			None
* Logic: 			- Align the robot before dropping
					- Based on the number of magnets dropped already, rotate Servo 1 by 60 degrees, on which 
					  the dropping mechanism is mounted, to drop the magnets successively in the control valves
					- Remove the closed valve from the val_pos list after dropping magnet
* Example Call:		drop_magnet() 	
*/
void drop_magnet()
{
	drop_align1();				//Align the robot with the wall behind
	_delay_ms(200);
	
	//Drop Magnet 1
	if(magnets_dropped==0)
	{		
		servo_1(40);			//Rotate to align slot 1 with the opening
		_delay_ms(1000);
		servo_1_free();		
	}
	
	//Drop Magnet 2
	else if(magnets_dropped==1)
	{
		servo_1(102);			//Rotate to align slot 2 with the opening
		_delay_ms(1000);
		servo_1_free();
	}
	
	//Drop Magnet 3
	else if(magnets_dropped==2)
	{
		servo_1(170);			//Rotate to align slot 3 with the opening
		_delay_ms(1000);
		servo_1_free();
	}

	servo_3_left();				//Rotate Servo 3/Sharp sensor 1 to left
	magnets_dropped++;			//Increment global variable
	valve_pos[current_PA] = 5;	//Remove the current closed valve from the val_pos list //5 -> Valve Closed
}


/*********************************************************************************************************/

//                                      GAS/VALVE DETECTION FUNCTIONS                                    //

/*********************************************************************************************************/

/*
* Function Name: 	gas_detection()	
* Input: 			None
* Output: 			gas_color (int) -> Returns the color code of the LED in gas leakage zone
* Logic: 			- Check whether black chart present or not (leakage/no leakage zone), then detect color of LED
					- If Gas Leakage is detected start the buzzer for 1 second (switched off by timer 4)
					- Move back slightly and take an U-Turn, switch
* Example Call:		color = gas_detection();	
*/
int gas_detection()
{	
	int gas_color;
	
	velocity(left_max, right_max);

	color_arm_90();				//Rotate color sensor arm perpendicular to central walls to check black/white
	gas_color = get_color(1);	//Store the detected LED color //Coressponding RGB LED is also lit

	if(gas_color)
	{
		soft_right_2_degrees(10);
		buzzer_on();
		buzzer_flag = 1;
		start_timer4(10);		//Buzz for 1 sec
	}
	
	back_mm(80);				//Go back slightly before taking U-Turn

	_delay_ms(200);
	left_degrees(176);			//Take U-Turn

	servo_3_front();			//Rotate Servo 3/Sharp sensor 1 front
	led_off();					//Turn off RGB LED

	return gas_color;			//Return Gas color code
}


/*
* Function Name:	valve_detection 	
* Input: 			None
* Output: 			valve_color (int) -> Returns the color code of the valve detected
* Logic: 			- Moves the color sensor arm down to detect control valves' color
					- Rotate robot slightly for better detection
					- Align with the front wall before taking a turn (Alignment for dropping magnets later)		
* Example Call:		color = valve_detection();	
*/
int valve_detection()
{
	int valve_color;
	
	color_arm_down();			//Moves color sensor arm down to detect control valves' color

	soft_right_degrees(5);		//Rotate robot slightly for better detection
	valve_color = get_color(0);	//Store the detected color //Coressponding RGB LED is also lit
	
	_delay_ms(200);
	soft_left_2_degrees(5);
	
	color_arm_up();				//Moves color sensor back up to default
	drop_align_front_wall();	//Align with the front wall before taking a turn (Alignment for dropping magnets later)	
	
	_delay_ms(100);
	led_off();					//Turn off RGB LED

	return valve_color;			//Return Valve color code
}


/*
* Function Name: 	check_gas	
* Input: 			gcolor (int) -> Color code of gas leakage to be processed
* Output: 			None
* Logic: 			- Processes the current gas color and checks if position of valve of same color is known previously
					- If it was detected one step previously, valve_known = 1
					- If it was detected two steps previously, valve_known = 2 or if it's far it'll be closed at the last
					- If valve not detected previously, move on to next pipepline area, valve_known = 0
* Example Call:		check_gas(1);
*/
void check_gas(int gcolor)
{
	gas_pos[current_PA] = gcolor;					//Store the color code of the current gas leakage in gas_pos list
	if(gcolor>=1 && gcolor<=3) total_detects++;		//Increment total detects, if valid gas leakage

	valve_known = 0;
	
	if(current_PA==0 || gcolor==0)	return;			//If it's the first pipeline, valve is not detected previously

	if(valve_pos[current_PA - 1] == gcolor)			//Valve detected one step back
	valve_known = 1;

	if(valve_pos[0] == gcolor && current_PA >= 2) 	//Valve detected two or three steps back at D0
	valve_remaining = 1;							//Close the valve at last instead of closing it now

	if(valve_pos[1] == gcolor && current_PA == 3)	//Valve detected two steps back at D1
	valve_known = 2;								//Go back and close the valve known itself
}


/*
* Function Name: 	check_valve	
* Input: 			vcolor (int) -> Color code of control valve to be processed
* Output: 			None
* Logic: 			- Processes the current valve color and checks if gas of same color is detected previously
					- If gas is detected previously, gas_known flag = 1, else 0
* Example Call:		check_valve(2);
*/
void check_valve(int vcolor)
{
	valve_pos[current_PA] = vcolor;		//Store the color code of the current valve in valve_pos list
	gas_known = 0;
	
	if(vcolor==0) return;               //If dummy valve, return

	for(int i=0; i<=3; i++)				//Search the gas_pos list, if gas of same color is detected already
	{
		if(gas_pos[i] == vcolor) 
		{
			gas_known = 1;				//If matched is found, gas_known flag = 1
			break;
		}
	}
}


/*********************************************************************************************************/

//                                         NAVIGATION FUNCTIONS                                          //

/*********************************************************************************************************/

/*
* Function Name: 	pipeline_navigation	
* Input: 			None
* Output: 			half_wall (int) -> Returns the hall wall configuration code of the pipeline
					// 1 -> Left // 2 -> Right // 3-> Both // 0 -> None
* Logic: 			Function to navigate inside the pipeline area
					- First, just go forward until a wall on either side is found
					- If a wall on left is detected, follow the wall till the fixed wall on right side is detected
					- If a wall on right is detected, move Sharp sensor 1 to front and follow the wall till the
					  central walls are detected
					- With the help of timer 4, based on how long the wall is present, the half wall configuration
					  can be mapped, which could be useful for quick navigation in future
* Example Call:		half_wall_config = pipeline_navigation();
*/
int pipeline_navigation()
{	
	int half_wall = 1;
	servo_3_left(); 			//Rotate Sharp sensor 1 to left
	
	start_timer4(15);			//Start timer 4
	check_wall();				//Check presence of walls on both sides
	
	if(wall_present==0)			//No wall present
	{
		while(1)
		{	
			check_wall();
			if(wall_present>0)	//Move forward till a wall on either side is found
			{	
				if(t4_flag==0)	//If no wall is detected within the specified time, no half walls present
				half_wall = 0;
				break;
			}

			velocity(left_max, right_max);
			forward();
		}
	}

	if(wall_present==2 || wall_present==3) 	//Wall on right side present 
	{
		if(t4_flag) 			//Wall detected within specified time peroid, store the half wall map
		half_wall = wall_present;

		follow_right_wall2();	//Follow right wall till central walls are found
		return half_wall;
	}

	else if(wall_present==1)	//Wall on left side present		
	{
		if(t4_flag)
		half_wall = 1;			//Wall detected within specified time peroid, store the half wall map

		while(1)
		{	
			check_wall();		//Follow left wall till fixed wall on right side is found
			if(wall_present==2 || wall_present==3) 
			{	
				if(t4_flag)		
				half_wall = 3;
				break;			//Wall on right side found, break loop
			}
			follow_left_wall(120); //Else, follow left wall at 120mm
		}
		follow_right_wall2();	//Follow right wall till central walls are found
	}

	return half_wall;			//Return the half wall configuration of the current pipeline area
}


/*
* Function Name: 	enter_pipeline	
* Input: 			mode (int) -> Direction of function operation
								  mode = 0 -> Enter from left side (anti-clockwise navigation)
								  mode = 1 -> Enter from right side (clockwise navigation)
* Output: 			skip_mode (int) -> 0, if gas detection is to be done
									   1, skip gas detection in the pipeline area
* Logic: 			Function to navigate and enter the next pipeline area
					- Follows the black line until an opening is detected to enter the next pipeline
					  within the specified time, else follow the line till the next node and enter pipeline
					- If within the specified time an opening is detected (i.e. half wall not present), 
					  enter the pipeline from here itself (shortcut)
					- Based on direction mode, take a left or right turn and prepare for next pipeline navigation
					- If skip_mode = 1, it will skip the gas detection routine in the current pipeline and go straight out
* Example Call:	 	enter_pipeline(1,0); -> Enter from right side and perform gas detection routine in current pipeline
*/
void enter_pipeline(int mode, int skip_mode)
{
	int flag=1, opening_detected = 0;

	clear_count();					//Clear line follower flags
	enable_flags(!mode,0,mode);		//Enable line detection flags based on direction mode

	start_timer4(20);				//Start timer 4
	TIMSK3 = 0x01; 					//Enable Line Sensor Readings

	_delay_ms(100);

	while(1)
	{	
		if(right_turn>0 || left_turn>0)	//If a right/left is detected, no half walls openings were found
		{	
			flag = 0;
			stop_timer4();			//Stop timer 4

			enable_flags(0,1,0);	//Enable only node detection
			break;
		}

		if(t4_flag && flag)			//Timer 4 is still running and turns not yet detected
		{
			TIMSK3 = 0x00;
			if(read_sharp(mode)>200) opening_detected = 1;	//If Sharp sensor detects an opening, stop line following
			TIMSK3 = 0x01;
				
			if(opening_detected)	//Opening detected, i.e. a half wall was not present
			{	
				stop();
				TIMSK3 = 0x00;		//Disable Line Sensor Readings	
				
				velocity(left_max, right_max);
				forward_mm(180-(mode*40));	//Move forward slightly before turning			
				_delay_ms(200);

				turn(mode);			//Turn left or right based on direction mode
				_delay_ms(200);

				if(skip_mode)		//If skip mode is on, go out straight to search for line
				search_line(mode);	

				else
				{	
					if(current_PA) 	//Store half wall configuration 
					half_walls_map[current_PA] = 0;

					forward_mm(355); //Move forward to the center of the pipeline area

					if(read_sharp(0)<200 && current_PA) //Store half wall configuration
					half_walls_map[current_PA] = 1;

					_delay_ms(200);
					turn(mode);		//Turn left or right based on direction mode
				}	
				return;
			}
		}
		follow_line();				//No opening detected yet, follow line
	}
	
	//No opening detected, follow line till next node and skip gas detection	
	if(skip_mode)
	{
		line_follower(0,1,0); 		//Follow line till one node is encountered
		
		//Follow line till next left/right turn based on direction mode	
		enable_flags(!mode,0,mode);
		line_follower(!mode,0,mode);
	}

	//No opening detected, follow line till next node and preapare for pipeline navigation
	else
	{
		line_follower(0,1,0);		//Follow line till one node is encountered

		velocity(left_max, right_max);
		forward_mm(100);			//Move forward before turning

		_delay_ms(200);
		turn(mode);					//Turn left or right based on direction mode
	}
}


/*
* Function Name: 	exit_pipeline	
* Input: 			mode (int) -> Direction of function operation
								  mode = 0 -> Exit from left side (anti-clockwise navigation)
								  mode = 1 -> Exit from right side (clockwise navigation)
* Output: 			None
* Logic: 			Function to exit from the current pipeline area
					- Follows the wall until an opening is detected to exit the current pipeline
					  within the specified time, else follow the wall till the black line is found
					- If within the specified time an opening is detected (i.e. half wall not present),
					  turn left/right based on direction mode and search for the outer black line and turn again
					- Else follow the wall till its lost, search for outer black line, and follow the line
* Example Call:		exit_pipeline(0); -> Exit pipeline from the left side
*/
void exit_pipeline(int mode)
{	
	if(!mode) servo_3_left(); 	//If direction mode is left, move Sharp sensor 1 to left for wall follower

	start_timer4(20);			//Start timer 4

	while(1)
	{
		if(read_sharp(mode)>240)
		{
			stop();				
			_delay_ms(100);
			
			if(t4_flag==1)		//Opening detected in the specified time
			{
				stop_timer4();
				velocity(left_max, right_max);
				forward_mm(180-(mode*40));	//Move foward
			
				_delay_ms(200);
				turn(mode);		//Turn left or right based on direction mode
				_delay_ms(200);				
			}
			
			search_line(mode);	//Search for outer black line and turn left or right based on direction mode
			break;
		}
		
		//Opening not yet detected, keep following the wall
		if(mode==0)	follow_left_wall(117);	//Follow left wall at 117mm
		else		follow_right_wall(87);  //Follow right wall at 87mm
	}
}


/*
* Function Name: 	go_to_valve	
* Input: 			mode (int) -> Direction of function operation
								  mode = 0 -> Anti-clockwise navigation
								  mode = 1 -> Clockwise navigation	
* Output: 			None
* Logic: 			Function to navigate to the next control valve
					- Follow line with left/right turn detection enabled based on mode
					- Follow line till front Sharp sensor detects a wall at 200mm
					- Move forward slightly for valve color detection
* Example Call:		go_to_valve(1);
*/
void go_to_valve(int mode)
{
	clear_count();	
	enable_flags(!mode,0,mode); //Enable left/right turn detection based on mode

	servo_3_front();			//Move Sharp sensor 1 to front

	while(1)
	{	
		TIMSK3 = 0x00;
		read_sharp(2);			//Read Sharp sensor value (Disable line sensor reading while doing so to avoid error)
		TIMSK3 = 0x01;

		if(sharp_1<=200)		//If front wall detected at 200mm, stop line follower			
		{
			stop();
			TIMSK3 = 0x00;
			break;
		}
		follow_line();			//Else, continue line following
	}

	velocity(left_max, right_max);
	forward_mm(45);				//Move forward slightly for valve color detection
	_delay_ms(100);
}


/*
* Function Name: 	close_known_valve	
* Input: 			None
* Output: 			None
* Logic: 			Function to navigate one step back to close a known valve
					- Navigates to the control valve in anti-clockwise direction and closes it
* Example Call:		close_known_valve();		
*/
void close_known_valve()
{
	servo_3_front();	//Move Sharp sensor 1 to front
	go_to_valve(0);		//Navigate to the control valve in anti-clockwise direction

	_delay_ms(200);		//Robot is now facing the valve. take U-Turn
	left_degrees(90);	//Turn 90 degrees left first
						//Then the next 90 degrees turning is done by drop_magnet()
	drop_magnet();		//Drop the magnet and close the valve
}


/*
* Function Name: 	close_known_valve_2	
* Input: 			None
* Output: 			None
* Logic: 			Function to navigate two steps back to close a known valve and returns back
					This function is called only in one special configuration where a valve is present at D1 and its
					corressponding gas leakge is detected later at PA-3
					- Navigates to the control valve in anti-clockwise direction and closes it
					- Navigates back to one step before from initial position, from where it started
* Example Call:		close_known_valve_2();	
*/
void close_known_valve_2()
{
	enable_flags(0,0,1);		//Follow line till one left turn is encountered
	line_follower(0,0,1);
	
	if(half_walls_map[2] == 0)	//If no walls were present in PA-2
	enter_pipeline(0,1);		//Take a shortcut, i.e. enter PA-2 in skip mode from left side

	else						//Else follow the line till two left turns are encountered
	{
		enable_flags(1,0,0);
		line_follower(2,0,0);
	}

	close_known_valve();		//Navigate one more step back to close the known valve at D1

	//Navigate back to initial position
	if(half_walls_map[2] == 0) 	//Enter pipeline in skip mode from right if HW not present in PA-2
	enter_pipeline(1,1);

	else						//Else just use line follower to navigate back one step
	{
		enable_flags(0,0,1);	//Follow the line till two right turns are encountered
		line_follower(0,0,2);
	}

	enable_flags(1,0,0);
	line_follower(1,0,0);
}


/*
* Function Name: 	navigate_back()	
* Input: 			None		
* Output: 			None
* Logic: 			Function to navigate one step back after closing a known valve
* Example Call:		navigate_back();		
*/
void navigate_back()
{
	if(half_walls_map[current_PA] == 0) 
	enter_pipeline(1,1);		//If no half walls present in current PA, enter from right in skip mode

	else						
	{
		enable_flags(0,0,1);	//Follow the line till two right turns are encountered
		line_follower(0,0,2);
	}	
}


/*
* Function Name: 	close_remaining_valve		
* Input: 			None
* Output: 			None
* Logic: 			Function to navigate to close the last remaining valve at D0 at the end
					- Navigate to the valve D0
					- Close the valve at D0
* Example Call:		close_remaining_valve();
*/
void close_remaining_valve()
{
	if(half_walls_map[0] == 0) 	//If no half walls present in PA-0, enter from right in skip mode
	enter_pipeline(1,1);

	else						//Else follow the line till two right turns are encountered
	{
		enable_flags(0,0,1);
		line_follower(0,0,2);
	}

	go_to_valve(1);				//Navigate to the valve
	drop_align_front_wall();	//Align with the front wall

	drop_magnet();				//Close the valve

	_delay_ms(100);
	forward_mm(20);

	_delay_ms(100);
	left_degrees(90);

	enable_flags(1,0,0);		//Navigate to the start node using line follower
	line_follower(1,0,0);

	enable_flags(0,1,0);
	line_follower(0,1,0);
}


/*
* Function Name: 	navigate_start	
* Input: 			None
* Output: 			None
* Logic: 			Function to navigate back to start after closing all the valves
					- If valve_remaining flag = 1, close the remaining valve at D0 
					- Else use line follower to reach start node
* Example Call:		navigate_start();
*/
void navigate_start()
{
	if(valve_remaining)			//If valve_remaining flag = 1, close the remaining valve at D0
	close_remaining_valve(); 

	else						//Else navigate to start node
	{
		enable_flags(0,0,1);
		line_follower(0,0,1);

		enable_flags(0,1,0);
		line_follower(0,1,0);
	}
	
	velocity(left_max, right_max);
	forward_mm(80);				//Move forward slightly
	_delay_ms(200);

	turn(valve_remaining);		//Turn left/right based on valve_remaining flag
	_delay_ms(200);				//i.e., Based on the direction robot was coming from before

	enable_flags(0,1,0);		//Follow the small strip of black line till Start/Stop sign
	line_follower(0,1,0);

	forward_mm(100);			//Move forward slightly
}


/*
* Function Name: 	display_status()	
* Input: 			None
* Output: 			None
* Logic: 			To display the variuos status flags on LCD
* Example Call:		display_status()	
*/
void display_status()
{
	lcd_wr_command(0x01);	//Clear LCD

	// ROW 1
	lcd_print(1, 1, current_PA+1, 1);	//Current pipeline area
	
	for(int i=0; i<=3; i++)	lcd_print(1, (i+3),  half_walls_map[i], 1); //  3 -  7	//Half walls map
	for(int i=0; i<=3; i++)	lcd_print(1, (i+8),  gas_pos[i], 		1);	//  8 - 11	//Gas leakage colors
	for(int i=0; i<=3; i++)	lcd_print(1, (i+13), valve_pos[i], 		1);	// 13 - 16	//Valve colors
	
	// ROW 2
	lcd_print(2, 1, total_detects, 1);
	lcd_print(2, 3, magnets_dropped, 1);
	lcd_print(2, 5, valve_remaining, 1);	
}


/*********************************************************************************************************/

//                                         INITIATION FUNCTIONS                                          //

/*********************************************************************************************************/

/*
* Function Name: 	port_init()	
* Input: 			None
* Output: 			None
* Logic: 			Function to initialize all the ports
* Example Call:		port_init();	
*/
void port_init()
{	
	buzzer_pin_config();		//Buzzer pin configuration
	lcd_port_config();			//LCD pin configuration
	led_pin_config();			//LED pin configuration

	adc_pin_config();			//ADC pin configuration
	color_sensor_pin_config();	//Color sensor pin configuration	

	motion_pin_config();		//Motion pin configuration
	left_encoder_pin_config(); 	//Left encoder pin configuration
	right_encoder_pin_config(); //Right encoder pin configuration
	
	servo1_pin_config(); 		//Configure PORTB 5 pin for Servomotor 1 operation
 	servo2_pin_config(); 		//Configure PORTB 6 pin for Servomotor 2 operation 
 	servo3_pin_config(); 		//Configure PORTB 7 pin for Servomotor 3 operation  		
}


/*
* Function Name: 	init_devices	
* Input: 			None
* Output: 			None
* Logic: 			Function to initiate all the sensors, actuators and interrupts
* Example Call:		init_devices();
*/
void init_devices(void)
{
 	cli(); 			//Clears the global interrupts

	port_init();	//Initiate all ports
	adc_init();		//Initiate ADC registers
	color_sensor_pin_interrupt_init(); //Initiate color sensor interrupts
	
	//Initiate timers
	timer1_init();	
	timer3_init();	
	timer4_init();
	timer5_init();
	
	//Initiate position encoder interrupts
	left_position_encoder_interrupt_init();
 	right_position_encoder_interrupt_init();

	sei();   		//Enables the global interrupts
}


/*********************************************************************************************************/

//                                               MAIN FUNCTION                                           //

/*********************************************************************************************************/

//IMPORTANT NOTE: PLEASE REFER TO ATTACHED JPG FOR PIPELINE AREA AND VALVE NUMBERING USED IN THIS PROGRAM

/*
* Function Name: 	main()	
* Input: 			None
* Output: 			None
* Logic: 			Function to detect Gas Leakages and close their respective Control Valves
					- Function contains 4 iterations for each pipeline process
					- First the robot navigates through the  pipeline area using wall follower algorithm
					- If it is a Gas Leakage Zone, type of gas leakage is detected using color sensor
					- Exit the pipeline area
					- If the Valve for the respective gas leakage is known already,
					  	- If it was detected one step previously, close the valve and return back
					  	- If it was detected two steps previously, close the valve and return back
					  	- If it was detected two/three steps previously at D0, it'll be closed at the last
					  	- If valve was not detected previously, move on to next valve detection
					- Navigate to the next control valve and detect the color
					- Process the detected color to check if gas of same color is detected already
					- Close the valve if the gas leakage is already detected by dropping magnets in the control valve
					- Navigate to the next pipeline area either using line follower algorithm or if half wall in
					  the next pipeline area is absent, take a shortcut entry
					- If it is the fourth pipeline area, navigate to start node
						- If valve is remaining to be closed at D0, close and navigate to start node
						- End process iteration
					- Else increment pipeline area number/index and iterate entire process till completion
					- Buzz for 5 seconds to inidicate end of run
* Example Call:		Called automatically by the Microcontroller
*/
int main()
{
	//Inititate everything	
	init_devices();
	lcd_set_4bit();
	lcd_init();

	color_sensor_power_down(); 			//Power down color sensor
	servo_initial_positions();			//Rotate servos to initial positions
	
	int g_color, v_color, half_wall;	//Variables to store color and half wall codes

	// START
	velocity(left_max, right_max);		//Set maximum velocity
	forward_mm(70);						//Go forward slightly

	enable_flags(0,1,0);				//Follow small strip of black line till start node
	line_follower(0,1,0);	

	while(1)
	{	
		//--------------------------------------//
		//    PIPELINE ENTRY - GAS DETECTION    //
		//--------------------------------------//

		display_status();
		
		if(total_detects<=3)								//If 3 or less than 3 leakages are detected, perform gas detection routine
		{	
			if(current_PA) enter_pipeline(1,0);				//Enter the next Pipeline from right
			
			//Pipeline Navigation
			half_wall = pipeline_navigation();				//Store Half Wall Configuration for PA-0
			if(!current_PA) half_walls_map[0] = half_wall; 	
			
			//Gas Detection
			g_color = gas_detection();						//Check the Gas Leakage color
			check_gas(g_color);								//Process the detected color to check if valve of same color is known
		}

		//Skip Gas Detection Routine
		else												//3 valid leakages already detected, skip the fourth No Leakage Zone
		{	
			navigate_back();								//Skips the fourth pipeline area (PA-3)
			navigate_start();
			break;
		}
		
		//--------------------------------------//
		//  VALVE DETECTION - DROPPING MAGNETS  //
		//--------------------------------------//

		display_status();
		
		if(valve_known==0)									//Valve not known, continue to next pipeline area
		{
			exit_pipeline(1);								//Exit pipeline from right
		}

		else if(valve_known)
		{
			exit_pipeline(0);								//Exit pipeline from left
			if(valve_known == 1) 	close_known_valve();	//Valve known 1 step back, go close and come back
			if(valve_known == 2)	close_known_valve_2();	//Valve known 2 steps back, go close and come back
			navigate_back();
		}
		
		go_to_valve(1);										//Navigate to the next control valve
		v_color = valve_detection();						//Detect the color of the control valve
		check_valve(v_color);								//Process the detected color to check if gas of same color is detected

		if(gas_known) drop_magnet();						//If gas leakage is already detected, close the valve
		else right_degrees(12);

		gas_known = 0;
		valve_known = -1;

		//--------------------------------------//		
		//   NAVIGATION TO NEXT PIPELINE/START  //
		//--------------------------------------//

		display_status(); 

		if(current_PA==3)									//If it is the fourth pipeline, navigate to start node
		{	
			navigate_start();								//If any valve is remaining to be closed, close and navigate to start
			break;											//End process iteration
		}	
		
		else current_PA++;									//Else increment pipeline area number/index and iterate entire process
	}

	// END
	display_status();
	buzz(5000);												//Buzz for 5 seconds to indicate end of run
}
