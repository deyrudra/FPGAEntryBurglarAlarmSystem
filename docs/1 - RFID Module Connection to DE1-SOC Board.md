## RFID Module Connection to DE1-SOC Board

**References:**

1. Here is a document covering RIFD MCRC522 with SPI: https://opencoursehub.cs.sfu.ca/bfraser/grav-cms/cmpt433/links/files/2024-student-howtos/UsingSPItoDriveAnMFRC522_RFIDReader.pdf 

	- 3.4.2 Executing a Transceive on Page 13/18
	- 3.5 Obtaining a UID on Page 16/18

2. Here is DE1-SOC Documentation: https://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/DE1-SoC_User_manualv.1.2.2_revE.pdf

- GPIO  (2x20) Pins Reference in on Page 29 and 30.



**RFID Module Connection to DE1-SOC Board**

RFID RC522 PINs Reference

- SDA: GPIO0[1]
- SCK: GPIO0[2]
- MOSI: GPIO0[3]
- MISO: GPIO0[4]
- GND: External Power Supply GND
- 3.3V: External Power Supply 3.3V



**RFID Module obtaining a UID**

1. We, the programmer, lodge the **REQA** command into the reader’s **FIFO Buffer**. 
   - Refer to `1` section **3.3.4** 
2. We, the programmer, execute the **TRANSCEIVE** command.
   - Refer to `1` section **3.4.2** 
3. The reader transmits the **REQA** command, probing for tags within range.
4. A tag receives the command. It sends an ATQA back to the reader, signaling that it is ready to receive a **UID** request.
5. We, the programmer, lodge the **ANTICOLLISION** command into the **FIFO Buffer**.
   - Refer to `1` section **3.3.4**, except replace the **REQA** (0x26) command with **ANTICOLLISION** (0x93 0x20)
     - You can see that **ANTICOLLISION** takes 2 bytes (16 bits) of address, where normally it is 8 bits. To implement this refer to `1` section **3.6** in the **Hint**s section.
       - Recall that **ANTICOLLISION’s** command code (0x93 0x20) is 2 bytes long. To handle this, you need to write to the `FIFODataReg` register two times: 0x93 first, followed by 0x20.
6. We, the programmer, execute the **TRANSCEIVE** command.
   - Refer to `1` section **3.4.2** 
7. The reader transmits the **ANTICOLLISION** command to the selected tag. **ANTICOLLISION** means nothing more than “GIVE ME YOUR UID”.
8. The selected tag receives the command. It sends its 4-byte UID, and its BCC, back to the reader.
9. We, the programmer, retrieve the UID and BCC from the **FIFO Buffer**.
   - Refer to `1` section **3.4.4** 



**What I have to do in Verilog**

1. Create a module which formats a Register Address (`module rfid_formatRegAddr (regAddr)`)
   - MFRC522 will not accept 0x01 (0000 0001) as a valid address. You must format it a certain way.
   - Refer to `1` section **3.3.4** to format the address right.
2. Create a module which writes to a Register Address (`module rifd_writeReg (regAddr, value)`)
   - Refer to `1` section **3.4.1** to write this module.
3. Create the final module which contains all of **"RFID Module obtaining a UID"** (`module rfid_obtainUID (UID)`)
   - Refer to "**RFID Module obtaining a UID**" to write this module.
