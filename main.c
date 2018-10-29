//Initializes essential libraries for the program
#include "msp.h"
#include "msp432.h"
#include <stdio.h>


////////////////////////////////////////////////////////////////

#include "msp.h"


/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	// Configure 5.7 as Timer A2.2 output
	    P5->SEL0 |= (BIT7);
	    P5->SEL1 &= ~(BIT7);
	    P5->DIR |= (BIT7);

	    //Need to configure both red led and green led
TIMER_A2->CCR[0] = 30000-1; //This is the PWM period   NEED 20 MS   30000-1
TIMER_A2->CCTL[2] = 0xE0;   //CCR4 reset/set mode
TIMER_A2->CTL  = 0b0000001001010100;    //use SMCLK, count up, clear TAOR register

	    while(1)
	    {
	     int PWM;

	        PWM = 1;


	        if(PWM == 1)
	                   {

	                              TIMER_A2->CCR[2] = 3000;//need 2 ms duty cyle   3000

	                   }

__delay_cycles(10000);

	        PWM = 2;

	        //rotate to 90 degree
	        if(PWM == 2)
	        {

	            TIMER_A2->CCR[2] = 1500;//need 20 ms duty cyle, 0 is 1 ms 1500

	        }
 __delay_cycles(10000);
	    }
}



////////////////////////////////////////////////////////////////

/**************************
 *  Authors: Kelly Schultz and Nathan Gruber
 *  Class: EGR 226 902
 *  Instructor: Professor Zuidema
 *  Date: 10/22/18
 *  Assignment: Midterm Project
 *  Descripiton: Midterm Project, Rev 2 
 *  EGR 226 - Fall 2018 Introduction to Digital Logic Tools
 *  Design a control system using a numeric Keypad and 16x4 LCD. 
 *  The heart of this system will be the TI MSP432 Launchpad microcontroller board. 
 *  The microcontroller will control the LCD display menu functions using keypad entry.
 *
 **************************/

//Initializes systick timer function
void SysTick_Init(void);

//Initializes functions necessary to facilitate LCD functionality 
void LCD_init(void);
void delay_micro(unsigned microsec);
void delay_ms (unsigned ms);
void PulseEnablePin(void);
void pushNibble (uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);
void PrintMenu(void);

//Initializes functions of FSM for DOOR, MOTOR, RED LED, GREEN LED, BLUE LED
void PrintMenu(void);
void DoorSubmenu(void);
void PrintDoorOpen(void);
void PrintDoorClosed(void);
void MotorSubmenu(void);
void LightSubmenu(void);
void PrintRED(void);
void PrintGREEN(void);
void PrintBLUE(void);
void error_message(void);
void PinEnables(void);

//Initializes functions to accept and return keypad entry values
int read_keypad();
void write_result(int value);
int collect_input(int value);
void reset(void);

int PWMRed=0; //global variable for red PWM value
int PWMBlue=0; //global variable for blue PWM value
int PWMGreen=0; //global variable for green PWM value
int LEDFlag=1; //global variable for LEDFlag

int GreenDoorFlag=0; //global variable for green led
int RedDoorFlag=0; //global variable for red led



//Delcares and defines states of FSM
enum states{
    DEFAULT,//Default state

    //SECTION OF DOOR
    DOOR,//switch do garage door, RETURN TO DEFAULT WHEN PROMPTED
    OPEN, //Open door for state door, RETURN TO DOOR
    CLOSE, //Close door for state door, RETURN TO DOOR

    //SECTION OF MOTOR
    MOTOR, //Select PWM for MOTOR, RETURN TO DEFAULT WHEN PROMPTED

    //SECTION OF LIGHTS
    LIGHTS, //Swith to lights, RETURN TO DEFAULT WHEN PROMPTED
    RED, //switch to RED, control PWM, RETURN TO LGIHTS
    GREEN,  //Switch to GREEN, control PWM, RETURN TO LGIHTS
    BLUE,  //Switch to BLUE, control PWM, RETURN TO LGIHTS
};

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    
    //Funciton calls initialization of pins
    PinEnables();
    int i=0;

    //Function initializes the LCD
    LCD_init();
    
    //Both functions move the cursor to the first line and clears the screen for displaying text
    commandWrite(0x0F);
    commandWrite(0x0C);

    //Sets first state of FSM to DEFAULT state
    enum states state= DEFAULT;

       //This value ensures proper operation of the keypad
       int value=-1;
    
//While loop of the entire FSM
while(1)
{      switch (state)
        {
            //Default State
            case DEFAULT:
        
                commandWrite(0x01); //clears LCD
                PrintMenu(); //Prints the LCD menu to the user
                value= read_keypad(); //Accepts the incoming value of the keypad selected by the user
        
                while(value==-1) //if value is -1 the keypad stays in its default state, accepting values from the user
                {
                    value= read_keypad();
                }
        
                if(value!=-1)//If the value is not -1. IE a button is pressed by the user, that is between 1 and 3, a menu selection will be made.
                {
                    //printf("%d", value);
                    
                    //If 1 is selected, FSM navigates to the DOOR menu
                    if(value == 1)
                        state= DOOR;
                    
                    //If 2 is selected, FSM navigates to the MOTOR menu
                    if(value== 2)
                        state= MOTOR;
                    
                    //If 3 is selected, FSM navigates to the LIGHTS menu
                    if(value== 3)
                        state= LIGHTS;
                }
                break;
            
            //DOOR state
            case DOOR:
        
                commandWrite(0x01); //clears LCD
                DoorSubmenu(); //calls the function to display the door submenu to the user
                value=read_keypad(); //Accepts the incoming value of the keypad selected by the user
        
                while(value==-1)
                {
                    value=read_keypad(); //if value is -1 the keypad stays in its default state, accepting values from the user
                }
        
                //If a value is selected, IE if the user selects a value from 1 to 2 the FSM will navigate to the according menu
                if(value!=-1)
                {
                    //If 1 is pressed, the garage door will open
                    if(value==1)
                        state= OPEN;
                    
                    //If 2 is pressed, the garage door will close
                    if(value==2)
                        state=CLOSE;
                    
                    //If the asteric key is pressed, which is a value of 10, the user will return the FSM to the DEFAULT menu
                    if(value==10)
                        state=DEFAULT;
                }
                break;

            //If the user selects 1, OPEN in the DOOR menu, the user will direct the FSM to this state
            case OPEN:
        
                commandWrite(0x01); //clears LCD
                PrintDoorOpen(); //prints to the user "DOOR OPEN"

                //turn on GREEN LED (P5.2), turn off RED LED (P5.5)
                GreenDoorFlag=1;
                RedDoorFlag=0;
                P5->OUT &= ~(BIT5);
                P5->OUT |= (BIT2);

                //Door Open, sets pulse width modulation accordingly 
                TIMER_A2->CCR[2] = 1500;
                
                //Accepts and stores values from the keypad
                value=read_keypad();
        
                //While value is -1, keypad awaits input from the user
                while(value==-1)
                {
                    value=read_keypad();
                }
        
                //If a value is selected, greater than -1, the door opens
                if(value!=-1)
                {
                    //If an asterik key is pressed, a value of 10, the user is returned to the DOOR menu
                    if(value==10)
                        state=DOOR;
                    
                   //If the user selects a value that is anything other than 10, the state returns to OPEN
                    if(value!=10)
                        state= OPEN;
                }
                break;
        
            //If the user selects 2 in the door menu, the FSM will navigate to the CLOSE menu
            case CLOSE:
        
                commandWrite(0x01); //clears LCD
                PrintDoorClosed();//Prints to the user, "DOOR CLOSED"
        
                //turn off GREEN LED (P5.2), turn on RED LED (P5.5)
                GreenDoorFlag=0;
                RedDoorFlag=1;
                P5->OUT |= (BIT5);
                P5->OUT &= ~(BIT2);

                //DOOR is closed, pulse width modulation is set accordingly
                TIMER_A2->CCR[2] = 3000;

                //Accepts and stores keypad values from the keypad
                value=read_keypad();
        
                //While the value is -1 the program remains in a state accepting values
                while(value==-1)
                {
                    value=read_keypad();
                }
        
                //If any value is pressed, the FSM will change accordingly 
                if(value!=-1)
                {
                    //If an asteric is pressed the FSM will return to the door menu
                    if(value==10)
                        state=DOOR;
                    
                    //If a value is pressed other than an asteric key, the door will close
                    if(value!=10)
                        state= CLOSE;
                 }
                break;
        
            //FSM is navigated to MOTOR menu
            case MOTOR:
                commandWrite(0x01); //clears LCD
                MotorSubmenu(); //prints to the user the motor submenu
       
                int PWM_DCmotor=0; //PWM is set accordingly so that the motor remains off until the user selects a value
        
                value=read_keypad(); //Calls function to accept keypad presses from the user

                PWM_DCmotor= get_value(); //Sets the pulse width modulation value to the integer value entered by the user

                if(PWM_DCmotor >=0 && PWM_DCmotor<=100) //This sets parameters so that the program will only accept values for the motor between 0 and 100
                {
                    //if the DC motor is set to zero then the program will stay at zero
                    if(PWM_DCmotor == 0)
                        TIMER_A2->CCR[1] = 0; //0 needs to be set to 0 instead of 0 minus 1.
                    
                    //If any other value is entered that is greater than 0, the pulse width modulation will be set accordingly
                    else
                        TIMER_A2->CCR[1] = (PWM_DCmotor/100.0) * (37500);  // all other inputs scale by multiply by 10 and subtracting 1.  10% is 99, 50% is 499, 100% is 999.
                }

                //If a value is entered outside of 0 to 100 the program will display an error message to the user
                else
                     error_message();
        
                //While nothing is pressed the keypad will wait for the user to enter a value
                while(value==-1)
                   {
                      value=read_keypad();
                   }
            
                //If any value is entered greater than -1, the FSM will accept and store the values accordinly 
                if(value!=-1)
                   {
                      //If the asterik key is pressed, the user will return the FSM to the default state
                      if(value==10)
                          state=DEFAULT;
                    
                      //If any other key is pressed, the state will remain in MOTOR and accept commands accordingly
                      if(value!=10)
                          state= MOTOR;
                   }
                 break;

            //State for LIGHTS
            case LIGHTS:
        
                commandWrite(0x01); //clears LCD
                LightSubmenu(); //Prints to the user the light submenu
                value=read_keypad(); //Keypad function accepts keypad presses and stores them to the value function
        
                //while nothing is pressed on the key pad, the program will await instructions
                while(value==-1)
                {
                    value=read_keypad();
                }
                
                //If the user presses a key the FSM will change the state accordingly to the input provided the values are between 1 and 3.
                if(value!=-1)
                {
                    //If 1 is pressed the FSM will navigate to the state for the Red LED
                    if(value==1)
                        state=RED;
                    
                    //If 2 is pressed the FSM will navigate to the state for the BLUE LED
                    if(value==2)
                        state=BLUE;
                    
                    //If 3 is pressed the FSM will navigate to the state for the Green LED
                    if(value==3)
                        state=GREEN;
                    
                    //if an asterik is pressed the user will return the FSM to the DEFAULT menu
                    if(value==10)
                        state=DEFAULT;
                }
                break;
        
             //FSM navigates to the RED state
             case RED:
        
                commandWrite(0x01); //clears LCD
                PrintRED(); //Prints to the user the RED LED menu options
                //Get PWM Value
                //Red PWM LED is P7.4 and TA1.4
        
                value=read_keypad(); //sets value to any keypad entry made by the user

                PWMRed= get_value(); //Sets the pulse width modulation to the value entered by the user
        
                //Sets parameters so that the program will only accept values between 0 and 100
                if(PWMRed >=0 && PWMRed<=100)
                {
                    //If a zero is entered, the pulse width modulation will be set to zero
                    if(PWMRed == 0)
                        TIMER_A1->CCR[4] = 0; //0 needs to be set to 0 instead of 0 minus 1.
                    
                    //If a value is entered above 0 and under 101 the program will set the pulse width modulation accordingly 
                    else
                        TIMER_A1->CCR[4] = PWMRed * 10 - 1;  // all other inputs scale by multiply by 10 and subtracting 1.  10% is 99, 50% is 499, 100% is 999.
                }

                //If any other value is pressed besides values between 0 and 100, the user is met with an error message
                else
                    error_message();
        
                //If nothing is pressed, the program will sit and wait for the user to enter a value
                while(value==-1)
                {
                    value=read_keypad();
                }
        
                //If an asterik is pressed the program will return to the LIGHTS main menu
                if(value!=-1)
                {
                    if(value==10)
                        state=LIGHTS;
                }
                break;
        
            //State for the BLUE LED
            case BLUE:
        
                commandWrite(0x01); //clears LCD
                PrintBLUE(); //Prints to the user the BLUE LED menu

                //Get PWM value
                //Blue PWM LED is P7.6 and TA1.2
                value=read_keypad(); //Reads and stores any value from the user in VALUE
                PWMBlue= get_value(); //Sets the pulse width modulation to the entry by the user
        
                if(PWMBlue >=0 && PWMBlue<=100)
                {
                    if(PWMBlue == 0)
                        TIMER_A1->CCR[2] = 0; //0 needs to be set to 0 instead of 0 minus 1.
                    else
                        TIMER_A1->CCR[2] = PWMBlue * 10 - 1;  // all other inputs scale by multiply by 10 and subtracting 1.  10% is 99, 50% is 499, 100% is 999
                }
                
                //If any other value is pressd an error message will be displaye to the user
                else
                    error_message();
                
                //If any other value is pressed, the value will be stored to value
                while(value==-1)
                {
                    value=read_keypad();
                }
                
                //If an asterik is pressed, the user will be returned to the LIGHTS menu
                if(value!=-1)
                {
                    if(value==10)
                        state=LIGHTS;
                }
                break;
        
            //State for GREEN LED
            case GREEN:
        
                commandWrite(0x01); //clears LCD
                PrintGREEN(); //Prints GREEN LED menu to the user

                //Get PWM value
                //Green PWM LED is P7.5 and TA1.3
                value=read_keypad(); //stores keypad pressed from the user into value
                PWMGreen= get_value(); //sets the pulse width modulation for the LED when the vlaue is entered 
        
                //Sets parameters so that only values between 0 and 100 will be accepted
                if(PWMGreen >=0 && PWMGreen<=100)
                {
                    //If zero is entered, the pulse width modulation will be set to zero
                    if(PWMGreen == 0)
                        TIMER_A1->CCR[3] = 0; //0 needs to be set to 0 instead of 0 minus 1.
                    
                    //Pulse width modulation will be set to any value provided is is between 1 and less than 101.
                    else
                        TIMER_A1->CCR[3] = PWMGreen * 10 - 1;  // all other inputs scale by multiply by 10 and subtracting 1.  10% is 99, 50% is 499, 100% is 999
                }
        
                //if the user enters a value greater than 100, an error message will display
                else
                    error_message();
        
                //When nothing is pressed, the pogram will wait for keypad entries
                while(value==-1)
                {
                    value=read_keypad();
                }
        
                //If an asterik is entered, a value of 10, the FSM will return to the LIGHTS State
                if(value!=-1)
                {
                    if(value==10)
                        state=LIGHTS;
                }

                break;
        
            /*state LIGHTSOFF:
            {
            }
            state LIGHTSON:
            {
            }*/

        }
    }

}

//Function initializes all pins for various hardware components
void PinEnables(void)
{

    SysTick_Init();
    P3->SEL0 &= ~(BIT2|BIT3); //sets P3.2 and P3.3 as GPIO
    P3->SEL1 &= ~(BIT2|BIT3);   //sets P3.2 and P3.3 as GPIO
    P3->DIR |= (BIT2|BIT3); //sets P3.2 and P3.3 as OUTPUT
    //P3->OUT &= ~(BIT2|BIT3); //sets P3.2 and P3.3 to 0 for RS RW =0

    //LCD
    P2->SEL0 &= ~(BIT4|BIT5|BIT6|BIT7); //sets (DB4-DB7) P2.4, P2.5, P2.5, P2.6, P2.7 as GPIO
    P2->SEL1 &= ~(BIT4|BIT5|BIT6|BIT7);
    P2->DIR |= (BIT4|BIT5|BIT6|BIT7); //sets pins 4.4-4.7 to OUTPUT


    P6->SEL0 &=~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
    P6->SEL1 &=~BIT4; //sets P6.4 to GPIO (ENABLE PIN)
    P6->DIR |= BIT4; //sets as output
    P6->OUT &= ~BIT4; //sets Enable pin to 0 initially

    //Pin enables for the Keypad
    P4->SEL0 &= ~(BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); //sets up P4 with 1-7
    P4->SEL1 &= ~(BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); // sets up P4 with 1-7
    P4->DIR &= ~(BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); // sets up P4 with 1-7
    P4->REN |= (BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); //enable internal resistor on P4 1-7
    P4->OUT |=  (BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); // make the internal resistor pull up to 3.3V (default state is a 1 now)

    //Pin enables for the door open/closed lights GREEN LED (P5.2) and  RED LED (P5.5)
    P5->SEL0 &= ~(BIT2|BIT5); //initializes red LED and green LED
    P5->SEL1 &= ~(BIT2|BIT5);   //initializes red LED and green LED
    P5->DIR |= (BIT2|BIT5);     //initializes red LED and green LED
    P5->OUT &= ~(BIT2|BIT5); //initializes LEDs to be off

    //Pin enables for Button 2.3 which is to be used for turning on and off the LEDs and is structured as an interrupt
    P2->SEL0 &= ~(BIT3);   //button connected to P2.3
    P2->SEL1 &= ~(BIT3);   //button connected to P2.3
    P2->DIR  &= ~(BIT3);   //sets to input
    P2->REN  |=  (BIT3);   //enables internal resistors
    P2->OUT  |=  (BIT3);   //sets output
    P2->IE   |=  (BIT3);   //enables interrupt

    //Pin enables for PWM LEDs (Referencing code from Zuidema In-class example Week 5 part 1)
    P7->SEL0 |= (BIT4|BIT5|BIT6); //sets SEL0=1;
    P7->SEL1 &= (BIT4|BIT5|BIT6);  //SEL1 = 0. Setting SEL0=1 and SEL1=0 activates PWM function
    P7->DIR |= (BIT4|BIT5|BIT6);    // Set pins as  PWM output.
    P7->OUT &= ~(BIT4|BIT5|BIT6);

    //Timer A
    TIMER_A1->CCR[0] = 999;  //1000 clocks = 0.333 ms.  This is the period of everything on Timer A1.  0.333 < 16.666 ms so the on/off shouldn't
                                 //be visible with the human eye.  1000 makes easy math to calculate duty cycle.  No particular reason to use 1000.

    TIMER_A1->CCTL[2] = 0b0000000011100000;  //reset / set compare.   Duty Cycle = CCR[1]/CCR[0].
    TIMER_A1->CCR[2] = 0;  //P7.6 initialize to 0% duty cycle
    TIMER_A1->CCTL[3] = 0b0000000011100000;
    TIMER_A1->CCR[3] = 0;  //P7.5 intialize to 0% duty cycle
    TIMER_A1->CCTL[4] = 0b0000000011100000;
    TIMER_A1->CCR[4] = 0;  //7.4 0% duty cycle

    //The next line turns on all of Timer A1.  None of the above will do anything until Timer A1 is started.
    TIMER_A1->CTL = 0b0000001000010100;

    //DOOR MOTOR CONFIGURATION AND TIMER
    //Configure 5.7 as Timer A2.2 output
    P5->SEL0 |= (BIT7);
    P5->SEL1 &= ~(BIT7);
    P5->DIR |= (BIT7);

    //Initializes both red led and green led
    TIMER_A2->CCR[0] = 30000-1; //This is the PWM period   NEED 20 MS   30000-1
    TIMER_A2->CCTL[2] = 0xE0;   //CCR4 reset/set mode
    TIMER_A2->CTL  = 0b0000001001010100;    //use SMCLK, count up, clear TAOR register

    //DC MOTOR SET UP
    P5->SEL0 |= (BIT6);
    P5->SEL1 &= ~(BIT6);
    P5->DIR |= (BIT6);

    TIMER_A2->CCR[0] = 37500-1; //This is the PWM period   3,000,000/75,000 = 40 Hz
    TIMER_A2->CCR[1] = 0;
    TIMER_A2->CCTL[1] = 0xE0;   //CCR1 reset/set mode
    TIMER_A2->CTL  = 0b0000001001010100;    //use SMCLK, count up, clear TAOR register

    //BUTTON 1.6 and BUTTON 1.7 (Button 1.6 corresponds to lights, Button 1.7 corresponds to motor)
    P1-> SEL0 &= ~(BIT6|BIT7);
    P1 -> SEL1 &= ~(BIT6|BIT7);
    P1 -> DIR &= ~(BIT6|BIT7);
    P1 -> REN |= (BIT6|BIT7);
    P1->OUT |= (BIT6|BIT7);
    P1->IE |= (BIT6|BIT7);
    P1->IES |= (BIT6|BIT7);
    
    //NVIC_Enable allows for the interrupt to be preformed
    NVIC_EnableIRQ(PORT1_IRQn);
    
}

//Interupt function for LED and DC Motor
void PORT1_IRQHandler()
{
    //Button 1.6 is for Lights
    //Button 1.7 is for DC motor
    if(P1->IFG & BIT6)
//    if(buttonP16_pressed())
    {
        //If push button is pressed, OR Bit 5
        if(RedDoorFlag==1)
        {
            //OR's the bits at 5, turning on and off LED lights.
            P5->OUT ^= (BIT5);
        }

        //If push button is pressed, OR Bit 2
        if(GreenDoorFlag==1)
        {
            P5->OUT ^= BIT2;
        }
        
        //If push button is pressed, turn off RED, BLUE, and GREEN LED
        if(LEDFlag==1)
        {
            P1 -> IFG &= ~BIT6;
            LEDFlag=0;
            TIMER_A1->CCR[4] = 0; //RED LED off
            TIMER_A1->CCR[2] = 0; //BLUE LED off
            TIMER_A1->CCR[3] = 0; //Green LED off
        }

        else //if(LEDFlag==0)
        {
            P1 -> IFG &= ~BIT6;
            LEDFlag=1;
            
            //Initializes all the timer A at CCR4 functions 
            TIMER_A1->CCR[4] = PWMRed * 10 - 1;  // RED LED on at previous Duty Cycle
            TIMER_A1->CCR[2] = PWMBlue * 10 - 1;  //BLUE LED on at previous Duty Cycle
            TIMER_A1->CCR[3] = PWMGreen * 10 - 1;  //Green LED on at previous Duty Cycle

        }
    }
    
    //if(buttonP17_pressed())
    if(P1->IFG & BIT7)
    {
        P1 -> IFG &= ~BIT7;
        TIMER_A2->CCR[1] = 0; //Stops DC Motor from Running

    }

}

//Function that prints main menu to user
void PrintMenu(void)
{
    commandWrite(0x0C); //Prints to line 1 of LCD
    int i;
    char line1[]= "      Menu      ";
    char line2[]= "1. Door         ";
    char line3[]= "2. Motor        ";
    char line4[]= "3. Lights       ";

     //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0); //Prints to line 2 of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }
    
     commandWrite(0x90); //Prints to line 3 of the LCD
     delay_ms(100); //Delay
    
      //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line3[i]);
     }

     commandWrite(0xD0); //Prints to line 4 of LCD
     delay_ms(100); //Delay
    
      //Prints characters of Array
     for(i=0; i<16; i++)
     {
          dataWrite(line4[i]);
      }
}

//Function that prints Door Submenu to user
void DoorSubmenu(void)
{
    commandWrite(0x01); //Clears the LCD screen
    commandWrite(0x0C); //Prints to first line of LCD
    
    int i;
    char line1[]= "   Door Menu    ";
    char line2[]= "1. Door Open     ";
    char line3[]= "2. Door Closed      ";

     //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0); //Prints to second line of LCD
    delay_ms(100);//Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }

     commandWrite(0x90); //Prints to third line of LCD
     delay_ms(100);//Delay
    
     //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line3[i]);
     }

}

//Prints the motor submenu text to the user
void MotorSubmenu(void)
{
    commandWrite(0x01);
    commandWrite(0x0C); //Prints to first line of LCD
    
    int i;
    char line1[]= "   Motor Menu    ";
    char line2[]= "Enter speed:    ";

     //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0); //Prints to second line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }
}

//Prints the Light submenu to the user
void LightSubmenu(void)
{
    commandWrite(0x01);
    commandWrite(0x0C); //Prints to first line of LCD
    
    int i;
    char line1[]= "  Light Submenu    ";
    char line2[]= "1. Red LED           ";
    char line3[]= "2. Blue LED           ";
    char line4[]= "3. Green LED           ";

     //Prints characters of Array
     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0); //Prints to second line of LCD
    delay_ms(100); //Dealy
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }
    commandWrite(0x90); //Prints to third line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line3[i]);
    }

    commandWrite(0xD0); //Prints to fourth line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line4[i]);
     }
}

//Prints text to the user for DOOR open options
void PrintDoorOpen(void)
{
    commandWrite(0x01);
    
    int i;
    char option[]= "    Door Open    ";
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}

//Prints text to user for DOOR closed option
void PrintDoorClosed(void)
{
    commandWrite(0x01);
    
    int i;
    char option[]= "  Door Closed   ";
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}

//Prints the red led submenu to the user
void PrintRED(void)
{
    commandWrite(0x01);
    
    int i;
    char option[]= "    Red LED     ";
    char prompt[]= "Enter Brightness";
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
    
    commandWrite(0xC0); //Prints to second line of LCD
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(prompt[i]);
    }

}

//Prints the BLUE led option menu to the user
void PrintBLUE(void)
{
    commandWrite(0x01);
    
    int i;
    char option[]= "    Blue LED     ";
    char prompt[]= "Enter Brightness";
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
    
    commandWrite(0xC0); //Prints to second line of LCD
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(prompt[i]);
    }
}

//Prints the Green LED menu to the user
void PrintGREEN(void)
{
    commandWrite(0x01);
    
    int i;
    char option[]= "    Green LED     ";
    char prompt[]= "Enter Brightness";
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
    
    commandWrite(0xC0); //Prints to second line of LCD
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(prompt[i]);
     }
}

//This function goes through the entire initialization sequence as shown in Figure 4
void LCD_init(void)
{
    //P3->OUT &= ~BIT2;    //P3.2 is RS, set to 0 because sending command

    commandWrite(0x03);   //3 in HEX
    delay_ms(100);  //waits 100 ms
    commandWrite(0x03);   //3 in HEX
    delay_micro(200);   //waits 200 microseconds
    commandWrite(0x03); //3 in HEX
    delay_ms(100);  //waits 100 ms
    
    commandWrite(0x02); //2 in HEX
    delay_micro(100); //waits 100 microseconds
    commandWrite(0x02); //2 in HEX
    delay_micro(100); ///waits 100 microseconds

    commandWrite(0x08); //8 in HEX
    delay_micro(100); //waits 100 microseconds
    commandWrite(0x0F); //HEX 0F MOD
    delay_micro(100); //waits 100 microseconds
    commandWrite(0x01); //01 HEX
    delay_micro(100); //waits 100 microseconds
    commandWrite(0x06); //HEX 06
    delay_ms(10); //waits 10 microseconds

    //INitialization complete according to Figure 4 in prelab
}

//Use the SysTick Timer peripheral to generate a delay in microseconds
//must be able to generate delays between 10 and 100 microseconds
void delay_micro(unsigned microsec)
{
    SysTick ->LOAD = ((microsec*3)-1); //1ms count down to 0
    SysTick ->VAL =0; //any write to CVR clears it and COUNTFLAG in CSR

    //wait for flag to be SET (Timeout happened)
    while((SysTick -> CTRL & 0x00010000) ==0);

}

//uses the SysTick timer peripheral to generate a delay in milliseconds
//function must be able to generate a delay of at least 60 ms
void delay_ms (unsigned ms)
{
    SysTick ->LOAD = ((ms*3000)-1); //1microsecond count down to 0
    SysTick ->VAL =0; //any write to CVR clears it and COUNTFLAG in CSR

    //wait for flag to be SET (Timeout happened)
    while((SysTick -> CTRL & 0x00010000) ==0);

}

//Sequence the Enable (E) pin as shown in Figure 6
void PulseEnablePin(void)
{
    P6->OUT &= ~BIT4; //sets enable pin to LOW
    delay_micro(100); //waits 10 microseconds

    P6 ->OUT |= BIT4; //sets enable pin to HIGH
    delay_micro(100); //waits 10 microseconds

    P6->OUT &= ~BIT4; //sets enable pin to LOW
    delay_micro(100);
}

//Pushes 1 nibble onto the data pins and pulses the Enable pin
void pushNibble (uint8_t nibble)
{
    P2->OUT &= ~(BIT4|BIT5|BIT6|BIT7); //clears values
    P2->OUT |= ((nibble & 0x0F)<<4);
   // delay_micro(100);

    PulseEnablePin();
}

//Pushes the most significant 4 bits of the byte onto the data pins by calling the pushNibble() function
//Then pushes the least significant 4 bits onto the data pins by calling the pushNibble() function again
void pushByte(uint8_t byte)
{
    uint8_t temp;
    temp= ((byte & 0xF0)>>4);

    //MOST SIGNIFICANT
    pushNibble(temp);

    //LEAST SIGNIFICANT
    temp= (byte & 0x0F);
    pushNibble(temp);
    delay_micro(100);

}

//write one byte of COMMAND by calling the pushByte() function with the COMMAND parameter
void commandWrite(uint8_t command)
{
    //RW to zero
    P3->OUT &= ~(BIT2); //pulls RS pin LOW (expects instructions)

    //RS to zero
    P3 ->OUT &=~(BIT3);
    pushByte(command);
    delay_ms(100);
}

//writes one byte of DATA by calling the pushByte() function within the DATA parameter
void dataWrite(uint8_t data)
{
    P3->OUT |= BIT2; //pulls RS pin HIGH (expects data)
    pushByte(data);

}

//Initalizes the systic timer 
void SysTick_Init(void)
{
    SysTick -> CTRL=0; //disable SysTick during setup
    SysTick -> LOAD= 0x00FFFFFF; //maximum reload value
    SysTick -> VAL= 0; //any write to current value clears it
    SysTick -> CTRL= 0x00000005; //enable SysTIck, CPU clk, no interrupts
}

//fucntion that allows the user to enter keypad presses and debounces each keypad press
int read_keypad()
{
    int value = -1;
    
    P4-> OUT &= ~BIT3; ///sets column 0 to 0
    P4-> DIR |= BIT3;
    {
        delay_ms(30);
        if(!(P4->IN & BIT7)) //Row 0, This is the number 1 on the keypad, value of 1
        {
            while(!((P4->IN & BIT7) == BIT7)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
               delay_ms(30);
               //printf("nothing is pressed\n");
            }

            value=1;
        }

        if(!(P4->IN & BIT6))  //Row 1, this is the number 4 on the keypad, value of 4
        {
            while(!((P4->IN & BIT6) == BIT6)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

            value=4;
        }
        if(!(P4->IN & BIT5))  //Row 2, this is the number 7 on the keypad, value of 7
        {
            while(!((P4->IN & BIT5) == BIT5)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

            value=7;
        }
        if(!(P4->IN & BIT4)) //Row 3, this is the asteric on the keypad, value of 10
        {
            while(!((P4->IN & BIT4) == BIT4)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

            value=10;
        }
            reset(); //Resets the pins so that the program can read for additional key pad presses.
    }

        P4-> OUT &= ~BIT2; ///sets column 1 to 0 SETS TO OUTPUT
        P4-> DIR |= BIT2;
    {
        delay_ms(30);
        if(!(P4->IN & BIT7)) //Row 0, This is the number 2 on the keypad, value of 2
        {
            while(!((P4->IN & BIT7) == BIT7)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

            value=2;
        }

        if(!(P4->IN & BIT6)) //Row 1,  This is the number 5 on the keypad, value of 5
        {
            while(!((P4->IN & BIT6) == BIT6)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

           value=5;
        }

        if(!(P4->IN & BIT5)) //Row 2, This is the number 8 on the keypad, value of 8
        {
            while(!((P4->IN & BIT5) == BIT5)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

            value=8;
         }

        if(!(P4->IN & BIT4)) //Row 3, This is the number 0 on the keypad, value of 0
        {
            while(!((P4->IN & BIT4) == BIT4)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
            {
                delay_ms(30);
                //printf("nothing is pressed\n");
            }

           value=0;
        }

        reset(); //Resets the pins so that the program can read for additional key pad presses.
    }

    P4-> OUT &= ~BIT1; ///sets column 2 to 0 SETS TO OUTPUT
    P4-> DIR |= BIT1;

    {
       delay_ms(30);
       if(!(P4->IN & BIT7)) //Row 0,  This is the number 3 on the keypad, value of 3
       {
           while(!((P4->IN & BIT7) == BIT7)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
           {
               delay_ms(30);
               //printf("nothing is pressed\n");
           }

           value=3;
       }

       if(!(P4->IN & BIT6)) //Row 1, This is the number 6 on the keypad, value of 6
       {
           while(!((P4->IN & BIT6) == BIT6)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
           {
               delay_ms(30);
               //printf("nothing is pressed\n");
           }

           value=6;
       }

       if(!(P4->IN & BIT5)) //Row 2, This is the number 9 on the keypad, value of 9
       {
           while(!((P4->IN & BIT5) == BIT5)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
           {
               delay_ms(30);
               //printf("nothing is pressed\n");
           }

           value=9;
       }

       if(!(P4->IN & BIT4)) //Row 3, This is the pound sign on the keypad, value of 12
       {
           while(!((P4->IN & BIT4) == BIT4)) //Causes the program to wait until the button is released to record a value. also prevents button bouncing entirely.
           {
               delay_ms(30);
               //printf("nothing is pressed\n");
           }

          value=12;
       }
       reset(); //Resets the pins so that the program can read for additional key pad presses.
    }


    //printf("Value = %d\n", value);

    //Return the value calculated by the formula above to be displayed to the user
    return value;
}

//Reset function that resets all rows and pins
void reset(void)
{
    P4->DIR &= ~(BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
    P4->OUT |= (BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); // make the internal resistor pull up to 3.3V (default state is a 1 now)
}

//function that writes the value to the user
void write_result(int result)
{
    int i=0;
    int j=0;
    char buffer[50];
    int pin[3];
    
    pin[0]=0;
    pin[1]=0;
    pin[2]=0;

   if(result < 0) //If nothing is pressed the user will be identified as such
    {

    }

    if(result <= 9 && result >= 0 ) //what ever number, 0 through 9 is pressed, will be returned to the user as such
    {
        //printf("The button pressed is %d \n" , result);
        sprintf(buffer, "%d", result);
        commandWrite(0x01);
        for(i=0; i<1; i++)   //prints the result to the LCD
            {
            dataWrite(buffer[i]);
            commandWrite(0x80+i);
            }
    }

    if(result == 10) //If the formula returns a 10, this means the asterisk button has been pressed
    {
        //printf("Button Pressed:* \n");
        sprintf(buffer, "Invalid: *");
        commandWrite(0x01);
        for(i=0; i<10; i++)   //prints result to LCD
            dataWrite(buffer[i]);
    }

    if(result == 12) //If the formula returns a 12, the pound sign will be returned.
       {
           //printf("Button Pressed:# \n");

       }

}

//Function that collects and stores 4 digits into an array. The array is set up so that no matter how many buttons are pressed the returned
// 4 digit array thats returned will always be the last 4 digits pressed. An asterick will return an invalid entry
int collect_input(int value)
{
   int pin=0;
   int i=0;
   char buffer[50];

   static int pincode[2]; //potentially 3?

    if(value == 12) //When the pound sign is pressed, the program will return the last 4 entered digits
    {
       //printf("\n\nthe 3 digit pin code is %d%d%d\n\n", pincode[0], pincode[1], pincode[2]);
        //printf("Button Pressed:# \n");
        commandWrite(0x01);
        sprintf(buffer, "Value:%d%d%d", pincode[0], pincode[1], pincode[2]);
        for(i=0; i<9; i++)   //prints result to LCD
            dataWrite(buffer[i]);
        pin= (pincode[0]*100 + pincode[1]*10 + pincode[2]*1);
        printf("PIN %d", pin);

        //printf("Value: %d%d%d", pincode[0], pincode[1], pincode[2]);
        pincode[0]=0;
        pincode[1]=0;
        pincode[2]=0;
    }
    
    if(value== 10) //If the Asteric Symbol is pressed, nothing will be returned
    {
        printf("\nINVALID ENTRY\n");
    }
    
if((value <=9) && (value >= 0))//If an entry is pressed 0-9, the number will be stored in the program.
{
    int i;
    int j=0;
    for(i=0; i<2; i++)
    {
        pincode[i] =pincode[i+1];
    }
    pincode[2] = value;

}
    //pin= (pincode[0]*100 + pincode[1]*10 + pincode[2]*1);
    //printf("PIN %d", pin);
    return pin;

}

//Function that retreives the value from the keypad
int get_value(void)
{
    int value=0;
    int code=0;
    char buffer[50];
    int i=0;

    while(value!= 12)
            {
        
            value = read_keypad(); //inputs the calculated value from the keypad into the int value to be used in other parts of the program.
           
            write_result(value);//Will print the result to the user
            
            code=collect_input(value); //stores the value to
        
            }

    sprintf(buffer, "Input: %d%%          ", code);

    commandWrite(0xC0); //moves cursor to second line
    
     //Prints characters of Array
    for(i=0; i<16; i++)
            dataWrite(buffer[i]);
        return code;
}

//Prints an error message to the value when improperly entering a numerical value into the keypad
void error_message(void)
{
    int i=0;
    char line1[]= "Invalid Entry      ";
    char line2[]= "Valid #: 1-100     ";
    char line3[]= "Return to Menu     ";
    char line4[]= "By entering *      ";

    commandWrite(0x01);

     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line1[i]);
     }

    commandWrite(0xC0);//Prints to second line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }

    commandWrite(0x90); //Prints to third line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line3[i]);
    }

    commandWrite(0xD0); //Prints to third line of LCD
    delay_ms(100); //Delay
    
     //Prints characters of Array
    for(i=0; i<16; i++)
    {
        dataWrite(line4[i]);
    }

}
