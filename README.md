# PIC-Multi-CAN-MCP2515-library
Library for implementation of multiple MCP2515 CAN controllers for most 8-bit PIC MCUs with MSSP module

## Calculating the CAN speed
This library currently has CNF registers settings for 95kbps and 500kbps at 8MHz clockspeed.
Refer to the MCP2515 datasheet pages 39-43 for calculating CNF1, CNF2, CNF3 registers or use the following formula:

baudrate = 1 / ((TQtot * 2 * (BRP + 1) / Fosc)<br>

TQtot = Sync + Prop+1 + Phase1+1 + Phase2+1<br>

BRP (Baudrate prescaler) value: 0-63<br>

bit time quantum (Tq) count values:<br>
Sync: 1 (fixed)<br>
Prop: 1-8<br>
Phase1: 1-8<br>
Phase2: 2-8<br>

Registers:<br>
BRP: CNF1<5:0><br>
Prop: CNF2<2:0><br>
Phase1: CNF2<5:3><br>
Phase2: CNF3<2:0><br>

NOTE: if CNF2 bit 7 (BTLMODE) is clear  Phase2 Tq will be the greater of Phase1 or 2Tq
