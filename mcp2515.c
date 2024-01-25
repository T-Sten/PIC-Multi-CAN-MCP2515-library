/*
 * File:   mcp2515.c
 * Author: tomst
 *
 * Created on January 8, 2024, 10:45 PM
 */


#include <xc.h>
#include "mcp2515.h"


//SPI instruction bytes
#define INS_RESET 0b11000000
#define INS_BITMOD 0b00000101
#define INS_READ 0b00000011
#define INS_WRITE 0b00000010
#define INS_LOADTX 0b01000000
#define INS_RTS 0b10000000

#define INS_READRX 0b10010000
#define INS_RXSTATUS 0b10110000

//Register definitions
#define REG_CNF1 0x2A
#define REG_CNF2 0x29
#define REG_CNF3 0x28

#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_CANCTRL 0x0F

#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

//Filter registers
#define REG_RXF0SIDH 0x00
#define REG_RXF0SIDL 0x01

#define REG_RXF1SIDH 0x04
#define REG_RXF1SIDL 0x05

#define REG_RXF2SIDH 0x08
#define REG_RXF2SIDL 0x09

#define REG_RXF3SIDH 0x10
#define REG_RXF3SIDL 0x11

#define REG_RXF4SIDH 0x14
#define REG_RXF4SIDL 0x15

#define REG_RXF5SIDH 0x18
#define REG_RXF5SIDL 0x19

//Filter mask registers
#define REG_RXM0SIDH 0x20
#define REG_RXM0SIDL 0x21
#define REG_RXM1SIDH 0x24
#define REG_RXM1SIDL 0x25

//Macros for setting the correct SS/CS pin low
#define SS_HIGH *self->_ss_pin_port |= (self->_ss_pin_bitmask);
#define SS_LOW *self->_ss_pin_port &= ~(self->_ss_pin_bitmask);



void MCP2515_attach(Mcp2515* self, SS_PIN_PORT ss_port,const uint8_t ss_pin){
    
    //calculate bitmask corresponding to pin number
    self->_ss_pin_bitmask = (uint8_t)(0x01 << ss_pin);
    
    //set corresponding TRIS bit to 0 (output) for SS pin
    switch(ss_port){
        
        case PORT_A:
            TRISA &= ~(self->_ss_pin_bitmask);
            self->_ss_pin_port = &PORTA;
        break;
        
        case PORT_B:
            TRISB &= ~(self->_ss_pin_bitmask);
            self->_ss_pin_port = &PORTB;
        break;
        
        case PORT_C:
            TRISC &= ~(self->_ss_pin_bitmask);
            self->_ss_pin_port = &PORTC;
        break;
        
        case PORT_D:
            TRISD &= ~(self->_ss_pin_bitmask);
            self->_ss_pin_port = &PORTD;
        break;
        
    }
    
    SS_HIGH;
}

void MCP2515_ini(Mcp2515* self, BUS_SPEED bus_speed, uint8_t enable_rollover){
    MCP2515_reset(self);
    
    uint8_t cnf1,cnf2,cnf3;
    
    switch(bus_speed){

        case BUS_SPEED_95KBPS:
        cnf1 =  SETTING_95KBPS_CNF1;
        cnf2 = SETTING_95KBPS_CNF2;
        cnf3 = SETTING_95KBPS_CNF3;
        break;                
    }
    
    MCP2515_set_reg(self, REG_CNF1, cnf1);
    MCP2515_set_reg(self, REG_CNF2, cnf2);
    MCP2515_set_reg(self, REG_CNF3, cnf3);
    
    MCP2515_set_reg(self, REG_RXB0CTRL | (enable_rollover << 2), 0xFF);
    MCP2515_set_reg(self, REG_RXB1CTRL, 0xFF);
}

void MCP2515_reset(Mcp2515* self){
    SS_LOW;
    SPI_transfer(INS_RESET);
    SS_HIGH;
}

uint8_t MCP2515_get_reg(Mcp2515* self, uint8_t reg_address){
    SS_LOW;
    SPI_transfer(INS_READ);
    SPI_transfer(reg_address);
    uint8_t recieved = SPI_transfer(0);
    SS_HIGH;
    return recieved;
}

void MCP2515_set_reg(Mcp2515* self, uint8_t reg_address, uint8_t data){
    SS_LOW;
    SPI_transfer(INS_WRITE);
    SPI_transfer(reg_address);
    SPI_transfer(data);
    SS_HIGH;
}

void MCP2515_load_tx(Mcp2515* self, Can_frame* frame, uint8_t buffer){
    SS_LOW;
    SPI_transfer(INS_LOADTX | (buffer * 2)); 
    
    //Set 11 bit SID (pg 20)
    SPI_transfer((uint16_t)frame->id >> 3);
    SPI_transfer((frame->id & 0b111) << 5);
    
    //Dummy bytes to skip EID registers, since we're here we are sending SID
    SPI_transfer(0);
    SPI_transfer(0);
    
    SPI_transfer(frame->len);
    
    //Set data registers
    for(int i=0;i<frame->len;i++){
        SPI_transfer((uint8_t)frame->data[i]);
    }
    SS_HIGH;
}

void MCP2515_load_tx_ext(Mcp2515* self, Can_extended_id_frame* frame, uint8_t buffer){
    SS_LOW;
    SPI_transfer(INS_LOADTX | (buffer * 4)); //pg 69
    
    //Dummy byte to skip SIDH register
    SPI_transfer(0);
    
    //Set extended id and extended ID bit pg 20,21
    SPI_transfer((frame->id >> 16 & 0b11) | 0b1000);
    SPI_transfer(frame->id >> 8 & 0xFF);
    SPI_transfer(frame->id & 0xFF);
    
    //Set or do not set RTR bit
    if(frame->rtr){
        SPI_transfer(frame->len | 0b1000000);
    }else{
        SPI_transfer(frame->len);
    }

    //Set data registers
    for(int i=0;i<frame->len;i++){
        SPI_transfer((uint8_t)frame->data[i]);
    }
    SS_HIGH;
}

void MCP2515_read_rx(Mcp2515* self, Can_frame* frame, uint8_t buffer){
    
    uint8_t ins;
    if(buffer == 0){ins = INS_READRX;}
    else{ins = INS_READRX | 0b10;}

    SS_LOW;
    SPI_transfer(ins);
    frame->id = (((uint16_t)SPI_transfer(0)) << 3);
    frame->id |= SPI_transfer(0) >> 5;
    
    //skip the extended id registers
    SPI_transfer(0);
    SPI_transfer(0);

    frame->len = SPI_transfer(0);
    
    for(int i=0; i < frame->len; i++){
        frame->data[i] = SPI_transfer(0);
    }
    SS_HIGH;
   
}

Mcp2515_rx_status MCP2515_check_rx(Mcp2515* self){

    SS_LOW;
    SPI_transfer(INS_RXSTATUS);
    uint8_t status_byte = SPI_transfer(0);
    SS_HIGH;
    
    Mcp2515_rx_status status;
    status.rx0_full = (status_byte >> 6) & 0b1;
    status.rx1_full = (status_byte >> 7) & 0b1;
    status.extended_id = (status_byte >> 4) & 0b1;
    status.remote_frame = (status_byte >> 3) & 0b1;
    status.filterhit = (status_byte & 0b111);
    
    
    //Handle rollover condition
    if(status.filterhit > 5){
        status.filterhit -= 5;
        status.rollover = 1;
    }
    else{
        status.rollover = 0;
    }
    
    return status;
}

void MCP2515_rts(Mcp2515* self, uint8_t buffer){
    SS_LOW;
    SPI_transfer(INS_RTS | (0x01 << buffer));
    SS_HIGH;
}

void MCP2515_bitmod(Mcp2515* self, uint8_t reg_address, uint8_t mask, uint8_t data){
    SS_LOW;
    SPI_transfer(INS_BITMOD);
    SPI_transfer(reg_address);
    SPI_transfer(mask);
    SPI_transfer(data);
    SS_HIGH;
}

void MCP2515_setfilter(Mcp2515* self, uint8_t filter_n, uint8_t id){
    
    //Filters 0-1 is associated with RX buffer 0 and filters 2-5 with RX buffer 1
    uint8_t buffer_n = 0;
    if(filter_n > 1){
        buffer_n = 1;
    }
            
    
    //Make id bytes
    uint8_t idh = id >> 3;
    uint8_t idl = (id & 0b111) << 5;
    
    //Get id reg addresses
    uint8_t fil_reg_idh = 0, fil_reg_idl = 0;
    
    switch(filter_n){
        case 0:
            fil_reg_idh =  REG_RXF0SIDH;
            fil_reg_idl =  REG_RXF0SIDL;
            break;
        case 1:
            fil_reg_idh =  REG_RXF1SIDH;
            fil_reg_idl =  REG_RXF1SIDL;
            break;
        case 2:
            fil_reg_idh =  REG_RXF2SIDH;
            fil_reg_idl =  REG_RXF2SIDL;
            break;
        case 3:
            fil_reg_idh =  REG_RXF3SIDH;
            fil_reg_idl =  REG_RXF3SIDL;
            break;
        case 4:
            fil_reg_idh = REG_RXF4SIDH;
            fil_reg_idl = REG_RXF4SIDL;
            break;
        case 5:
            fil_reg_idh = REG_RXF5SIDH;
            fil_reg_idl = REG_RXF5SIDL;
            break;
    }
    
    //Write filters
    MCP2515_set_reg(self, fil_reg_idh, idh);
    MCP2515_set_reg(self, fil_reg_idl, idl);
    
    //Get buffer control and mask register address for correct buffer
    uint8_t rxctrl = 0, rxmh = 0, rxml = 0;
    if(buffer_n < 1){
        rxctrl = REG_RXB0CTRL; 
        rxmh = REG_RXM0SIDH;
        rxml = REG_RXM0SIDL;
    }
    else{
        rxctrl = REG_RXB1CTRL; 
        rxmh = REG_RXM1SIDH;
        rxml = REG_RXM1SIDL;                
    }
    
    
    //Turn filters on
    MCP2515_set_reg(self, rxctrl, 0x00);
    
    //Disable masking
    MCP2515_set_reg(self, rxmh, 0xFF);
    MCP2515_set_reg(self, rxml, 0xFF); 
    
}

void MCP2515_disable_filters(Mcp2515* self){
    MCP2515_set_reg(self, REG_RXB0CTRL, 0x60);
    MCP2515_set_reg(self, REG_RXB1CTRL, 0x60);
}

void MCP2515_setmode(Mcp2515* self, MCP_MODE mode){
    
    MCP2515_bitmod(self, REG_CANCTRL, 0xE0, mode);
}

void MCP2515_setclkout(Mcp2515* self, MCP_CLKOUT clkout){
    
    MCP2515_bitmod(self, REG_CANCTRL, 0x07, clkout);
}


void MCP2515_interrupt_enable(Mcp2515* self, MCP_INTERRUPT mcp_interrupt){
    MCP2515_set_reg(self,REG_CANINTF, 0x00); //clear interrupts
    MCP2515_bitmod(self, REG_CANINTE, mcp_interrupt, mcp_interrupt);
}

void MCP2515_interrupt_clear(Mcp2515* self, MCP_INTERRUPT mcp_interrupt){
    MCP2515_bitmod(self, REG_CANINTF, mcp_interrupt, 0x00);
}
