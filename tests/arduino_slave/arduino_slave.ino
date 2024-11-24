#include<SPI.h>
volatile int i = 0;
byte myArray[2];

void setup()
{
  Serial.begin(9600);
  pinMode(SS, INPUT_PULLUP);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, INPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();  //allows SPI interrupt
}

void loop(void)
{
  // if (i == 2)
  // {
  //   uint16_t x = (uint16_t)myArray[0] << 8 | (uint16_t)myArray[1];
  //   Serial.print("Received 16-bit data item from Master: ");
  //   Serial.println(x, HEX);
  //   i = 0;
  //   Serial.println("=============================================");
  // }
}

ISR(SPI_STC_vect) // Interrupt routine function
{
  myArray[i] = SPDR;
  Serial.println("Byte received");
  Serial.print("Binary: ");
  Serial.print(myArray[i], BIN);
  Serial.print(", HEX: ");
  Serial.println(myArray[i], HEX);
  i++;
}