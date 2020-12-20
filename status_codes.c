0x00	No errors
0x08	START condition transmitted

0x10	Repeated START condition transmitted

0x18	SLA+W transmitted; ACK received

0x20	SLA+W transmitted; NAK received

0x28	Data byte transmitted; ACK received

0x30	Data byte transmitted; NAK received

0x38	Arbitration lost in SLA; or NAK received

0x40	SLA+R transmitted; ACK received

0x48	SLA+R transmitted; NAK received

0x50	Data received; ACK has been returned

0x58	Data received; NAK has been returned

0xE0	Arbitration lost

0xE1	Arbitration lost in START

0xE2	Arbitration lost in STOP

0xE3	Arbitration lost in read ACK

0xE4	Arbitration lost in read NAK

0xE5	Arbitration lost in write

0xF8	Unknown error

0xFF	Illegal START or STOP condition
