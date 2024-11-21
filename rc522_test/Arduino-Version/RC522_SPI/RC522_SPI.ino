#include <SPI.h>
// Define pins for the RC522 module
#define RST_PIN 9  // Reset pin
#define SS_PIN 10  // Slave select pin

// RC522 Instance Variables
byte _chipSelectPin = SS_PIN;
byte _resetPowerDownPin = RST_PIN;
#define MFRC522_SPICLOCK (4000000u)  // set to 4MHz, max is 10MHz

// Status Codes
enum StatusCode : byte {
  STATUS_OK,                 // Success
  STATUS_ERROR,              // Error in communication
  STATUS_COLLISION,          // Collission detected
  STATUS_TIMEOUT,            // Timeout in communication.
  STATUS_NO_ROOM,            // A buffer is not big enough.
  STATUS_INTERNAL_ERROR,     // Internal error in the code. Should not happen ;-)
  STATUS_INVALID,            // Invalid argument.
  STATUS_CRC_WRONG,          // The CRC_A does not match
  STATUS_MIFARE_NACK = 0xff  // A MIFARE PICC responded with NAK.
};


typedef struct {
		byte		size;			// Number of bytes in the UID. 4, 7 or 10.
		byte		uidByte[10];
		byte		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
	} Uid;

Uid uid;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;
  SPI.begin();
  // module rc_init()
  // RC522 Initialization
  rc_init();
  // endmodule
  Serial.println("Place your card near the reader...");
}

void loop() {
  // put your main code here, to run repeatedly:

  if (!rc_picc_isNewKeyPresent()) {
    return;
  } 

  // // Select one of the cards
  if (!PICC_ReadCardSerial()) {
    return;
  }

  // Print UID to Serial Monitor
  Serial.print("Card UID: ");
  for (byte i = 0; i < uid.size; i++) {
    Serial.print(uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(uid.uidByte[i], HEX);
  }
  Serial.println();


}

void rc_init() {
  bool hardReset = false;
  // Set the chipSelectPin as digital output, do not select the slave yet
	pinMode(_chipSelectPin, OUTPUT);
	digitalWrite(_chipSelectPin, HIGH);
	
	// If a valid pin number has been set, pull device out of power down / reset state.
	if (_resetPowerDownPin != UINT8_MAX) {
		// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
		pinMode(_resetPowerDownPin, INPUT);
	
		if (digitalRead(_resetPowerDownPin) == LOW) {	// The MFRC522 chip is in power down mode.
			pinMode(_resetPowerDownPin, OUTPUT);		// Now set the resetPowerDownPin as digital output.
			digitalWrite(_resetPowerDownPin, LOW);		// Make sure we have a clean LOW state.
			delayMicroseconds(2);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
			digitalWrite(_resetPowerDownPin, HIGH);		// Exit power down mode. This triggers a hard reset.
			// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
			delay(50);
			hardReset = true;
		}
	}

  if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		rc_reset();
	}

  // call rc_writeRegister() to reset rest of the registers
  // Reset Baud Rates
  rc_writeRegister(0x12 << 1, 0x00);  // TxModeReg = 0x12, 0x00 to reset
  rc_writeRegister(0x13 << 1, 0x00);  // RxModeReg = 0x13, 0x00 to reset

  // Reset Module Width
  rc_writeRegister(0x24 << 1, 0x26);  // 0x24 = modWidthReg, 0x26 to reset

  // Uncomment if needed, dont know what this does.
  rc_writeRegister(0x2A << 1, 0x80);
  rc_writeRegister(0x2B << 1, 0xA9);
  rc_writeRegister(0x2C << 1, 0x03);
  rc_writeRegister(0x2D << 1, 0xE8);

  rc_writeRegister(0x15 << 1, 0x40);  // TxASKReg = 0x15
  rc_writeRegister(0x11 << 1, 0x3D);  // TxSelReg = 0x16
  rc_antennaOn();                     // Reset turns antenna off, turn it on now.
}


// This resets the the RC522 module.
void rc_reset() {
  rc_writeRegister(0x01 << 1, 0x0F);  // 0x01 is CommandReg, 0x0F is Soft Reset

  uint8_t count = 0;
  do {
    delay(50);
  } while ((rc_readRegister(0x01 << 1) & (1 << 4)) && (++count) < 3);  // Checks if the 4th bit from CommandReg (PowerDown bit) is equal to 1. If it is, then count is increased by 1.
  // This operation takes a maximum of 3 x 50ms.
}

// This writes to a register using the register address and the value wanting to be pushed to the register.
void rc_writeRegister(byte reg, byte value) {                                // reg must be formatted.
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));  // MSBFIRST = 1
  digitalWrite(_chipSelectPin, LOW);                                         // Setting chip select to 0 to select RC522.

  SPI.transfer(reg);    // Transfer reg address (already formatted) through MOSI
  SPI.transfer(value);  // Transfer reg address (already formatted) through MISO

  digitalWrite(_chipSelectPin, HIGH);  // Setting chip select to 1, so that we aren't selecting RC522 any longer
  SPI.endTransaction();                // Stop SPI bus
}

void rc_writeRegister(byte reg,     ///< The register to write to. One of the PCD_Register enums.
                      byte count,   ///< The number of bytes to write to the register
                      byte *values  ///< The values to write. Byte array.
) {
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));  // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                                         // Select slave
  SPI.transfer(reg);                                                         // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  for (byte index = 0; index < count; index++) {
    SPI.transfer(values[index]);
  }
  digitalWrite(_chipSelectPin, HIGH);  // Release slave again
  SPI.endTransaction();                // Stop using the SPI bus
}



byte rc_readRegister(byte reg) {
  byte value;
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));  // MSBFIRST = 1
  digitalWrite(_chipSelectPin, LOW);                                         // Setting chip select to 0 to select RC522.

  SPI.transfer(0x80 | reg);            // MSB == 1 (0x80) is for reading. LSB is not used in address.
  value = SPI.transfer(0);             // Read the value back. Send 0 to stop reading.
  digitalWrite(_chipSelectPin, HIGH);  // Setting chip select to 1, so that we aren't selecting RC522 any longer
  SPI.endTransaction();                // Stop SPI bus
  return value;
}

void rc_readRegister(byte reg,      ///< The register to read from. One of the PCD_Register enums.
                     byte count,    ///< The number of bytes to read
                     byte *values,  ///< Byte array to store the values in.
                     byte rxAlign = 0   ///< Only bit positions rxAlign..7 in values[0] are updated.
) {
  if (count == 0) {
    return;
  }
  //Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | reg;                                                 // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;                                                            // Index in values array.
  SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));  // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                                         // Select slave
  count--;                                                                   // One read is performed outside of the loop
  SPI.transfer(address);                                                     // Tell MFRC522 which address we want to read
  if (rxAlign) {                                                             // Only update bit positions rxAlign..7 in values[0]
    // Create bit mask for bit positions rxAlign..7
    byte mask = (0xFF << rxAlign) & 0xFF;
    // Read value and tell that we want to read the same address again.
    byte value = SPI.transfer(address);
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask);
    index++;
  }
  while (index < count) {
    values[index] = SPI.transfer(address);  // Read value and tell that we want to read the same address again.
    index++;
  }
  values[index] = SPI.transfer(0);     // Read the final byte. Send 0 to stop reading.
  digitalWrite(_chipSelectPin, HIGH);  // Release slave again
  SPI.endTransaction();                // Stop using the SPI bus
}



void rc_antennaOn() {
  byte value = rc_readRegister(0x14 << 1);      // 0x14 for TxControlReg
  if ((value & 0x03) != 0x03) {                 // Checking the two least significant bits, if they are 1.
    rc_writeRegister(0x14 << 1, value | 0x03);  // 0x14 for TxControlReg, setting the two least significant bits to 1 to turn antenna on.
  }
}



bool rc_picc_isNewKeyPresent() {  // checks if there is a NEW key
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  // Reset baud rates
  rc_writeRegister(0x12 << 1, 0x00);  // TxModeReg = 0x12, 0x00 to reset
  rc_writeRegister(0x13 << 1, 0x00);  // RxModeReg = 0x13, 0x00 to reset
  // Reset ModWidthReg
  rc_writeRegister(0x24 << 1, 0x26);  // 0x24 = modWidthReg, 0x26 to reset

  // rc_writeRegister(0x0D << 1, 0x07);
  // rc_writeRegister(0x0A << 1, 0x26);

  // rc_writeRegister(0x01 << 1, 0x0C);
  // PCD_SetRegisterBitMask(0x0D << 1, 0x80);

  // byte read = rc_readRegister(0x0A << 1);

  // Serial.println(read);

  StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);

  return (result == STATUS_OK || result == STATUS_COLLISION);
}

StatusCode PICC_RequestA(byte *bufferATQA, byte *bufferSize) {  //  Returns true if a PICC responds to PICC_CMD_REQA.
  return PICC_REQA_or_WUPA(0x26, bufferATQA, bufferSize);       // 0x26 = REQA Command
}



void PCD_ClearRegisterBitMask(byte reg,  ///< The register to update. One of the PCD_Register enums.
                              byte mask  ///< The bits to clear.
) {
  byte tmp;
  tmp = rc_readRegister(reg);
  rc_writeRegister(reg, tmp & (~mask));  // clear bit mask
}

StatusCode PCD_TransceiveData(byte *sendData,  ///< Pointer to the data to transfer to the FIFO.
                              byte sendLen,    ///< Number of bytes to transfer to the FIFO.
                              byte *backData,  ///< nullptr or pointer to buffer if data should be read back after executing the command.
                              byte *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                              byte *validBits = nullptr,
                              byte rxAlign = 0,
                              bool checkCRC = false) {
  byte waitIRq = 0x30;  // RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(0x0C, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

StatusCode PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize) {
  byte validBits;
  StatusCode status;

  if (bufferATQA == nullptr || *bufferSize < 2) {  // The ATQA response is 2 bytes long.
    return STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(0x0E << 1, 0x80);  // 0x0E = CollReg	// ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;                              // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
  // status = STATUS_OK;
  if (status != STATUS_OK) {
    return status;
  }
  if (*bufferSize != 2 || validBits != 0) {  // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  }
  return STATUS_OK;
}  // End PICC_REQA_or_WUPA()

StatusCode PCD_CommunicateWithPICC(byte command,     ///< The command to execute. One of the PCD_Command enums.
                                   byte waitIRq,     ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                   byte *sendData,   ///< Pointer to the data to transfer to the FIFO.
                                   byte sendLen,     ///< Number of bytes to transfer to the FIFO.
                                   byte *backData,   ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                   byte *backLen,    ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                   byte *validBits,  ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                   byte rxAlign,     ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                   bool checkCRC     ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits;  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  rc_writeRegister(0x01 << 1, 0x00);               // Stop any active command.
  rc_writeRegister(0x04 << 1, 0x7F);               // Clear all seven interrupt request bits
  rc_writeRegister(0x0A << 1, 0x80);               // FlushBuffer = 1, FIFO initialization
  rc_writeRegister(0x09 << 1, sendLen, sendData);  // Write sendData to the FIFO
  rc_writeRegister(0x0D << 1, bitFraming);         // Bit adjustments
  rc_writeRegister(0x01 << 1, command);            // Execute the command
  if (command == 0x0C) {
    PCD_SetRegisterBitMask(0x0D << 1, 0x80);  // StartSend=1, transmission of data starts
  }

  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
  // automatically starts when the PCD stops transmitting.
  //
  // Wait here for the command to complete. The bits specified in the
  // `waitIRq` parameter define what bits constitute a completed command.
  // When they are set in the ComIrqReg register, then the command is
  // considered complete. If the command is not indicated as complete in
  // ~36ms, then consider the command as timed out.
  const uint32_t deadline = millis() + 36;
  bool completed = false;

  do {
    byte n = rc_readRegister(0x04 << 1);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & waitIRq) {                    // One of the interrupts that signal success has been set.
      completed = true;
      break;
    }
    if (n & 0x01) {  // Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
    yield();
  } while (static_cast<uint32_t>(millis()) < deadline);

  // 36ms and nothing happened. Communication with the MFRC522 might be down.
  if (!completed) {
    return STATUS_TIMEOUT;
  }

  // Stop now if any errors except collisions were detected.
  byte errorRegValue = rc_readRegister(0x06 << 1);  // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13) {                       // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }

  byte _validBits = 0;

  // If the caller wants data back, get it from the MFRC522.
  if (backData && backLen) {
    byte n = rc_readRegister(0x0A << 1);  // Number of bytes in the FIFO
    if (n > *backLen) {
      return STATUS_NO_ROOM;
    }
    *backLen = n;                                      // Number of bytes returned
    rc_readRegister(0x09 << 1, n, backData, rxAlign);  // Get received data from FIFO
    _validBits = rc_readRegister(0x0C << 1) & 0x07;    // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (validBits) {
      *validBits = _validBits;
    }
  }

  // Tell about collisions
  if (errorRegValue & 0x08) {  // CollErr
    return STATUS_COLLISION;
  }

  // Perform CRC_A validation if requested.
  if (backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if (*backLen == 1 && _validBits == 4) {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0) {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
    if (status != STATUS_OK) {
      return status;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
      return STATUS_CRC_WRONG;
    }
  }

  return STATUS_OK;
}

void PCD_SetRegisterBitMask(byte reg,  ///< The register to update. One of the PCD_Register enums.
                            byte mask  ///< The bits to set.
) {
  byte tmp;
  tmp = rc_readRegister(reg);
  rc_writeRegister(reg, tmp | mask);  // set bit mask
}

StatusCode PCD_CalculateCRC(byte *data,   ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                            byte length,  ///< In: The number of bytes to transfer.
                            byte *result  ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
) {
  rc_writeRegister(0x01 << 1, 0x00);          // Stop any active command. CommandReg = 0x01 << 1, PCD_Idle = 0x00
  rc_writeRegister(0x05 << 1, 0x04);          // Clear the CRCIRq interrupt request bit, DivIrqReg = 0x05 << 1,
  rc_writeRegister(0x0A << 1, 0x80);          // FlushBuffer = 1, FIFO initialization, FIFOLevelReg = 0x0A << 1
  rc_writeRegister(0x09 << 1, length, data);  // Write data to the FIFO, FIFODataReg = 0x09 << 1
  rc_writeRegister(0x01 << 1, 0x03);          // Start the calculation, CommandReg = 0x01 << 1, PCD_CalcCRC = 0x03

  // Wait for the CRC calculation to complete. Check for the register to
  // indicate that the CRC calculation is complete in a loop. If the
  // calculation is not indicated as complete in ~90ms, then time out
  // the operation.
  const uint32_t deadline = millis() + 89;

  do {
    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    byte n = rc_readRegister(0x05 << 1);
    if (n & 0x04) {                       // CRCIRq bit set - calculation done
      rc_writeRegister(0x01 << 1, 0x00);  // Stop calculating CRC for new content in the FIFO.
      // Transfer the result from the registers to the result buffer
      result[0] = rc_readRegister(0x22 << 1);  //CRCResultRegL = 0x22 << 1
      result[1] = rc_readRegister(0x21 << 1);  //CRCResultRegH =  0x21 << 1
      return STATUS_OK;
    }
    yield();
  } while (static_cast<uint32_t>(millis()) < deadline);

  // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
  return STATUS_TIMEOUT;
}

StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	StatusCode result;
	byte count;
	byte checkBit;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(0x0E << 1, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = 0x93;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = 0x95;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = 0x97;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = 0x88;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			rc_writeRegister(0x0D << 1, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = rc_readRegister(0x0E << 1); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == 0x88) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == 0x88) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
}

bool PICC_ReadCardSerial() {
	StatusCode result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
} // End 