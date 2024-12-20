#include <LiquidCrystal.h>
#include <SPI.h>


// LCD Display
#define RSPin 2
#define EnablePin 3
#define DS4 4
#define DS5 5
#define DS6 0
#define DS7 1

#define LCDColumns 16
#define LCDRows 2

LiquidCrystal lcd(RSPin, EnablePin,DS4,DS5,DS6,DS7);


// FPGA SPI
const int MISO_FPGA = 9;
const int MOSI_FPGA = 8;
const int SCK_FPGA = 7;
const int SS_FPGA = 6;

volatile bool ssActive = false;
volatile uint8_t receivedData = 0;
 // Data to send to master (example)


// RFID SPI
#define SS_PIN 10  // Slave select pin

// RC522 Instance Variables
byte _chipSelectPin = SS_PIN;
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

byte misoData = 0x00;
bool sendUID = false;
int count = 0;
Uid currentUID;
Uid noUID;

String systemState = "DISENGAGED"; // Default System Disengaged
String prevState;

void setup()
{

  // Serial.begin(9600);

  lcd.begin(LCDColumns, LCDRows); //Configure the LCD
  lcd.setCursor(0,0);
  

  // Configure FPGA SPI Pins
  pinMode(MISO_FPGA, OUTPUT);
  pinMode(MOSI_FPGA, INPUT);
  pinMode(SCK_FPGA, INPUT);
  pinMode(SS_FPGA, INPUT);
  digitalWrite(SS_FPGA, HIGH);  // Enable Pull-up Resistor on SS


  // Configure RFID SPI Pins
  SPI.begin();
  rc_init();


  // Reset Registers/Data
  uid = noUID;
  

  // Startup Message
  lcd.clear();             // Clear previous value (4 spaces for old data)
  lcd.setCursor(0, 0);
  lcd.print("ENTRY & BURGLAR ");
  lcd.setCursor(0, 1);           // Reset to counter position
  lcd.print("  ALARM SYSTEM");


  delay(3000); // Delay 3 seconds.
}

void loop()
{
  while (digitalRead(SS_FPGA) == HIGH);

  // SPI communication: receive data and respond

  if (receivedData == 0xAA) {
    misoData = 0x00;
    count = 0;
    sendUID = true;
    currentUID = uid;
  }

  if (sendUID == true) {
    misoData = currentUID.uidByte[count];
    if (count == 4) {
      misoData = 0x00;
      sendUID = false;
      currentUID = noUID;
      uid = noUID;
      count = 0;
    }
    count++;
  }

  if (receivedData == 0xBA) {
    systemState = "DISENGAGED";
  }

  if (receivedData == 0xBB) {
    systemState = "ENGAGED";
  }

  if (receivedData == 0xBC) {
    systemState = "COUNTDOWN";
  } 

  if (receivedData == 0xBD) {
    systemState = "ALARM";
  }

  // Serial.print("HEX: ");
  // Serial.print(misoData, HEX);
  // Serial.print("   Binary: ");
  // Serial.println(misoData, BIN);


  receivedData = 0; // Clear the received data buffer

  for (int bit = 7; bit >= 0; --bit) {
    // Wait for the clock to go HIGH
    while (digitalRead(SCK_FPGA) == LOW);

    // Read data from MOSI line
    if (digitalRead(MOSI_FPGA)) {
      receivedData |= (1 << bit);
    }

    // Write data to MISO line
      if (bitRead(misoData, bit)) {
        digitalWrite(MISO_FPGA, HIGH);
      }
      else {
        digitalWrite(MISO_FPGA, LOW);
      }

    // Wait for the clock to go LOW
    while (digitalRead(SCK_FPGA) == HIGH);
  }


  // RFID KEY

  if (!rc_picc_isNewKeyPresent()) {
  } 
  else {
    if (!PICC_ReadCardSerial()) {
    }
    else {
    }
  }

  if (prevState != systemState) {
    prevState = systemState;
    lcd.clear();             // Clear previous value (4 spaces for old data)
    lcd.setCursor(0, 0);
    lcd.print("STATE:");
    if (systemState != "DISENGAGED") {
      lcd.print(" ");
    }
    lcd.print(systemState);
    lcd.setCursor(0, 1); 
    lcd.print("KEY1: SET ALARM");
    lcd.setCursor(0, 1);           // Reset to counter position
  }


}






void rc_init() {
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

  rc_antennaOn();
}

void rc_antennaOn() {
  byte value = rc_readRegister(0x14 << 1);      // 0x14 for TxControlReg
  if ((value & 0x03) != 0x03) {                 // Checking the two least significant bits, if they are 1.
    rc_writeRegister(0x14 << 1, value | 0x03);  // 0x14 for TxControlReg, setting the two least significant bits to 1 to turn antenna on.
  }
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


bool rc_picc_isNewKeyPresent() {  // checks if there is a NEW key
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  // Reset baud rates
  rc_writeRegister(0x12 << 1, 0x00);  // TxModeReg = 0x12, 0x00 to reset
  rc_writeRegister(0x13 << 1, 0x00);  // RxModeReg = 0x13, 0x00 to reset
  // Reset ModWidthReg
  rc_writeRegister(0x24 << 1, 0x26);  // 0x24 = modWidthReg, 0x26 to reset

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
  } while (true);

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

  return STATUS_OK;
}

void PCD_SetRegisterBitMask(byte reg,  ///< The register to update. One of the PCD_Register enums.
                            byte mask  ///< The bits to set.
) {
  byte tmp;
  tmp = rc_readRegister(reg);
  rc_writeRegister(reg, tmp | mask);  // set bit mask
}


StatusCode PICC_Select(	Uid *uid,
											byte validBits
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	StatusCode result;
	byte count;
	byte checkBit;
	byte index;
	byte uidIndex;
	int8_t currentLevelKnownBits;
	byte buffer[9];
	byte bufferUsed;
	byte rxAlign;
	byte txLastBits;
	byte *responseBuffer;
	byte responseLength;
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(0x0E << 1, 0x80);
	
	uidComplete = false;
	while (!uidComplete) {
    buffer[0] = 0x93;
  
		currentLevelKnownBits = validBits;
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}

		index = 2; 

		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0);
		if (bytesToCopy) {
			byte maxBytes = 4; 
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
		}
		selectDone = false;
		while (!selectDone) {

				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;
				buffer[1]		= (index << 4) + txLastBits;
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			
			rxAlign = txLastBits;										
			rc_writeRegister(0x0D << 1, (rxAlign << 4) + txLastBits);
			
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) {
				byte valueOfCollReg = rc_readRegister(0x0E << 1);

				count			= currentLevelKnownBits % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); 
				buffer[index]	|= (1);
			}
			else { 
				if (currentLevelKnownBits >= 32) { 
					selectDone = true;

				}
				else { 

					currentLevelKnownBits = 32;

				}
			}
		} 

		

		index	=  2; 
		bytesToCopy	= 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[count] = buffer[index++];
		}
		uidComplete = true;


	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 4;

	return STATUS_OK;
}

bool PICC_ReadCardSerial() {
	StatusCode result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
} // End 