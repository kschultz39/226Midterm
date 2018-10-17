#include "msp.h"
#include "msp432.h"
#include <stdio.h>


/**
 * main.c
 */
void SysTick_Init(void);


void LCD_init(void);
void delay_micro(unsigned microsec);
void delay_ms (unsigned ms);
void PulseEnablePin(void);
void pushNibble (uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);
void PrintMenu(void);

void PrintMenu(void);
void DoorSubmenu(void);
void PrintDoorOpen(void);
void PrintDoorClosed(void);
void MotorSubmenu(void);
void LightSubmenu(void);
void PrintRED(void);
void PrintGREEN(void);
void PrintBLUE(void);
void PinEnables(void);

int read_keypad();
void write_result(int value);
int collect_input(int value);
void reset(void);


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

    //RUN OUTSIDE STATE MACHINE AS IF STATEMENTS TO CONTROL LIGHTS OFF AND ON
    //LIGHTSOFF,  //turns the lights off
    //LIGHTSON    //Turns lights on
};

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    PinEnables();
    int i=0;
    LCD_init();
    commandWrite(0x0F);
    commandWrite(0x0C);

   enum states state= DEFAULT;
   /*PrintMenu();
   delay_ms(1000);
   DoorSubmenu();
   delay_ms(1000);
   PrintDoorOpen();
   delay_ms(1000);
   PrintDoorClosed();
   delay_ms(1000);
   MotorSubmenu();
   delay_ms(1000);
   LightSubmenu();
   delay_ms(1000);
   PrintRED();
   delay_ms(1000);
   PrintGREEN();
   delay_ms(1000);
   PrintBLUE();
*/
       int value=-1;
while(1)
{      switch (state)
        {
        case DEFAULT:
                commandWrite(0x01); //clears LCD
                PrintMenu();
                value= read_keypad();
                while(value==-1)
                {
                    value= read_keypad();
                }
                if(value!=-1)
                {
                    printf("%d", value);
                    if(value == 1)
                        state= DOOR;
                    if(value== 2)
                        state= MOTOR;
                    if(value== 3)
                        state= LIGHTS;
                }
                break;
            case DOOR:
                commandWrite(0x01); //clears LCD
                DoorSubmenu();
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==1)
                        state= OPEN;
                    if(value==2)
                        state=CLOSE;
                    if(value==10)
                        state=DEFAULT;
                }
                break;

            case OPEN:
                commandWrite(0x01); //clears LCD
                PrintDoorOpen();
                //turn on led, turn off other
                //rotate servo 90*
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==10)
                        state=DEFAULT;
                    if(value!=10)
                        state= OPEN;
                }
                break;
            case CLOSE:
                commandWrite(0x01); //clears LCD
                PrintDoorClosed();
                //turn off led, turn on other
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==10)
                        state=DEFAULT;
                    if(value!=10)
                        state= CLOSE;
                 }
                break;
            case MOTOR:
                commandWrite(0x01); //clears LCD
                MotorSubmenu();
                //PWM
                value=read_keypad();
                while(value==-1)
                   {
                      value=read_keypad();
                   }
                if(value!=-1)
                   {
                      if(value==10)
                          state=DEFAULT;
                      if(value!=10)
                          state= MOTOR;
                   }
                 break;

            case LIGHTS:
                commandWrite(0x01); //clears LCD
                LightSubmenu();
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==1)
                        state=RED;
                    if(value==2)
                        state=BLUE;
                    if(value==3)
                        state=GREEN;
                    if(value==10)
                        state=DEFAULT;
                }
                break;
            case RED:
                commandWrite(0x01); //clears LCD
                PrintRED();
                //Get PWM Value
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==10)
                        state=DEFAULT;
                }
                break;
            case BLUE:
                commandWrite(0x01); //clears LCD
                PrintBLUE();
                //Get PWM value
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==10)
                        state=DEFAULT;
                }
                break;
            case GREEN:
                commandWrite(0x01); //clears LCD
                PrintGREEN();
                //Get PWM value
                value=read_keypad();
                while(value==-1)
                {
                    value=read_keypad();
                }
                if(value!=-1)
                {
                    if(value==10)
                        state=DEFAULT;
                }

                break;
            /*state LIGHTSOFF:
            {
            }
            state LIGHTSON:
            {
            }*/
        }

    /*int value=0;
    int code=0;
    char buffer[50];

         while(1)
         {
            value = read_keypad(); //inputs the calculated value from the keypad into the int value to be used in other parts of the program.
            write_result(value);//Will print the result to the user


             code=collect_input(value); //stores the value to

         }*/

}
}
void PinEnables(void)
{

    SysTick_Init();
    P3->SEL0 &= ~(BIT2|BIT3); //sets P3.2 and P3.3 as GPIO
    P3->SEL1 &= ~(BIT2|BIT3);   //sets P3.2 and P3.3 as GPIO
    P3->DIR |= (BIT2|BIT3); //sets P3.2 and P3.3 as OUTPUT
    //P3->OUT &= ~(BIT2|BIT3); //sets P3.2 and P3.3 to 0 for RS RW =0

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
}
void PrintMenu(void)
{
    commandWrite(0x0C);
    int i;
    char line1[]= "      Menu      ";
    char line2[]= "1. Door         ";
    char line3[]= "2. Motor        ";
    char line4[]= "3. Lights       ";

     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }

     commandWrite(0x90);
     delay_ms(100);
     for(i=0; i<16; i++)
     {
         dataWrite(line3[i]);
     }

     commandWrite(0xD0);
     delay_ms(100);
     for(i=0; i<16; i++)
     {
          dataWrite(line4[i]);
      }
}
void DoorSubmenu(void)
{
    commandWrite(0x01);
    commandWrite(0x0C);
    int i;
    char line1[]= "   Door Menu    ";
    char line2[]= "1. Door Open     ";
    char line3[]= "2. Door Closed      ";

     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }

     commandWrite(0x90);
     delay_ms(100);
     for(i=0; i<16; i++)
     {
         dataWrite(line3[i]);
     }


}
void MotorSubmenu(void)
{
    commandWrite(0x01);
    commandWrite(0x0C);
    int i;
    char line1[]= "   Motor Menu    ";
    char line2[]= "Enter motor speed:    ";


     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }
}

void LightSubmenu(void)
{
    commandWrite(0x01);
    commandWrite(0x0C);
    int i;
    char line1[]= "  Light Submenu    ";
    char line2[]= "1. Red LED           ";
    char line3[]= "2. Blue LED           ";
    char line4[]= "3. Green LED           ";


     for(i=0; i<16; i++)
     {
         dataWrite(line1[i]);
     }

    commandWrite(0xC0);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line2[i]);
    }
    commandWrite(0x90);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line3[i]);
    }

    commandWrite(0xD0);
    delay_ms(100);
    for(i=0; i<16; i++)
    {
        dataWrite(line4[i]);
     }
}

void PrintDoorOpen(void)
{
    commandWrite(0x01);
    int i;
    char option[]= "    Door Open    ";
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}
void PrintDoorClosed(void)
{
    commandWrite(0x01);
    int i;
    char option[]= "  Door Closed   ";
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}

void PrintRED(void)
{
    commandWrite(0x01);
    int i;
    char option[]= "    Red LED     ";
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}

void PrintBLUE(void)
{
    commandWrite(0x01);
    int i;
    char option[]= "    Blue LED     ";
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
    }
}
void PrintGREEN(void)
{
    commandWrite(0x01);
    int i;
    char option[]= "    Green LED     ";
    for(i=0; i<16; i++)
    {
        dataWrite(option[i]);
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

void SysTick_Init(void)
{
    SysTick -> CTRL=0; //disable SysTick during setup
    SysTick -> LOAD= 0x00FFFFFF; //maximum reload value
    SysTick -> VAL= 0; //any write to current value clears it
    SysTick -> CTRL= 0x00000005; //enable SysTIck, CPU clk, no interrupts
}








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

        printf("Value: %d%d%d", pincode[0], pincode[1], pincode[2]);
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
    pin= (pincode[0]*100 + pincode[1]*10 + pincode[2]*1);
    return pin;

}
