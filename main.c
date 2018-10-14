#include "msp.h"
#include "msp432.h"


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
    LIGHTSOFF,  //turns the lights off
    LIGHTSON    //Turns lights on
};

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    PinEnables();
    int i=0;
    LCD_init();
    commandWrite(0x0F);
    commandWrite(0x0C);

    //enum states state= DEFAULT;
   PrintMenu();
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


    /*while(1)
    {
        switch (state)
        {
            state DEFAULT:
            {
                PrintMenu();
                value= read_keypad();
                if(value ==1)
                    state= DOOR;
                if(value==2)
                    state= MOTOR;
                if(value==3)
                    state= LIGHTS;
                if(value==0 || value>=4 && value<=12)
                    state= DEFAULT;
            }
            state DOOR:
            {
            }
            state OPEN:
            {
            }
            state CLOSED:
            {
            }
            state MOTOR:
            {
            }
            state LIGHTS:
            {
            }
            state RED:
            {
            }
            state BLUE:
            {
            }
            state GREEN
            {
            }
            state LIGHTSOFF:
            {
            }
            state LIGHTSON:
            {
            }
        }
    }
*/




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
