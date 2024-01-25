/*
 * File:   CAN_example.c
 * Author: Tom Stenvall
 *
 * PIC16F877A example code for using the multi MCP2515 library
 * 
 */

// PIC16F877A CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


#include <xc.h>

#include "spi.h"
#include "mcp2515.h"

#define CAN0_INT PORTBbits.RB4
#define CAN1_INT PORTBbits.RB5

//Example IDs for filters
#define FILTER_ID_1 0xA1
#define FILTER_ID_2 0xB2


void main(void) {
    
    
    //Initialise SPI master mode (SPI mode 2) with the frequency of FOSC/4
    SPI_ini(SPI_SPEED_FOSC_4);
    
    
    TRISBbits.TRISB4 = 1;       //Set MCP2515 interrput input pin to high impedance mode (input)
    OPTION_REGbits.nRBPU = 0;   //Enable pullups for PORTB (Interrupt pin of the MCP2515 is open-drain)
    
    //Make 2 instances of the Mcp2515 type
    Mcp2515 can_0;
    Mcp2515 can_1;
    
    //Create pointers to the Mcp2515 instances
    //(or just reference the instance every time you call a MCP2515 function)
    Mcp2515* can0 = &can_0;
    Mcp2515* can1 = &can_1;
    
    //Attach SS/CS pins to the Mcp2515 can instances to a port and a pin number
    MCP2515_attach(can0, PORT_D, 2);
    MCP2515_attach(can1, PORT_D, 3);
    
    //Initialise MCP2515s to the required CAN bus speeds, enable/disable rollover from RX0 to RX1 if RX0 is full
    MCP2515_ini(can0, BUS_SPEED_95KBPS, ROLLOVER_ENABLE);
    //MCP2515_ini(can1, BUS_SPEED_95KBPS);
    
    //NOTE: filters 0-1 is associated with buffer 0 and filters 2-5 is associated with buffer 1
    MCP2515_setfilter(can0, 0, FILTER_ID_1);
    MCP2515_setfilter(can0, 2, FILTER_ID_1); // Message with this ID will be received to buffer 1 ^^
    
    MCP2515_disable_filters(can1);    //All messages will be received
    
    //Enable specific interrupts to drive the MCP2515 interrupt pin
    MCP2515_interrupt_enable(can0, RX0I | RX1I);   //Can 0 interrupts on reception to receive buffer 0 and 1
    MCP2515_interrupt_enable(can1, RX0I);          //Can1 interrupt on reception to receive buffer 1 
    
    
    MCP2515_setclkout(can0, CLKOUT_1);  //Set clkout to FOSC/1 (8MHz in this case), it will be driving the PIC16F877A and CAN1-MCP2515 clocks
    MCP2515_setmode(can0, MODE_NORMAL); //Configuration done, set can0-MCP2515 to normal operation mode
    
    MCP2515_setclkout(can1, CLKOUT_OFF);
    MCP2515_setmode(can1, MODE_NORMAL);
    
    while(1){
        
        if(!CAN0_INT){ //INT fallen -> go to read
            
            Mcp2515_rx_status rx_status = MCP2515_check_rx(can0);
            
            Can_frame received_frame; //Create a CAN frame to pass to the read function
            
            /*
             *  It can also be decided what buffer needs to be read just by the filterhit value
             *  (remember to handle rollover condition! (rx_status.rollover) )
             */
            
            //Read buffers, clear interrupts
            if(rx_status.rx0_full){
                MCP2515_read_rx(can0, &received_frame, 0);
                MCP2515_interrupt_clear(can0,RX0I); 
            }
            if(rx_status.rx1_full){
                MCP2515_read_rx(can0, &received_frame, 1);
                MCP2515_interrupt_clear(can0,RX1I);
            }
            
            
            //Send something back if we got a frame with id 0xA1
            if(received_frame.id == 0xA1){
                
                Can_frame newframe;
                
                newframe.id = 0xAA;
                newframe.len = 4; //Data length of 4
                
                //Making up some data here
                newframe.data[0] = 0x1F;
                newframe.data[1] = 0xA1;
                newframe.data[2] = 0xF1;
                newframe.data[3] = 0x1A;
                
                MCP2515_load_tx(can0, &newframe, 0); //Load the new frame to TX buffer 0
                MCP2515_load_tx(can0, &received_frame, 1); //Load the received frame to TX buffer 1
                
                //Start requesting transmission of TX0 and TX1 on the bus
                MCP2515_rts(can0, 0);
                MCP2515_rts(can0, 1);
            }
            
            /*
             * DO other stuff with the received data
             */
        }
        
        //Same stuff for CAN1
        if(!CAN1_INT){
            Mcp2515_rx_status rx_status = MCP2515_check_rx(can1);
            
            Can_frame received_frame;
            if(rx_status.rx0_full){
                MCP2515_read_rx(can1, &received_frame, 0);
                MCP2515_interrupt_clear(can1,RX0I); 
            }
            if(rx_status.rx1_full){
                MCP2515_read_rx(can1, &received_frame, 1);
                MCP2515_interrupt_clear(can1,RX1I);
            }
            
            /*
             * DO stuff with the received data
             */
        }
        
    }
    
    return;
}
