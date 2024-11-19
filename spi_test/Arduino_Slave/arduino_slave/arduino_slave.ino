#include<SPI.h>
volatile int i = 0;
byte receivedData;

void setup()
{
  Serial.begin(9600);
  pinMode(SS, INPUT_PULLUP);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, INPUT);
  SPCR |= _BV(SPE);
  // SPCR = (1 << SPE) | (1 << MSTR);
  SPI.attachInterrupt();  //allows SPI interrupt
}

void loop(void)
{

}

ISR (SPI_STC_vect)   //Inerrrput routine function
{
  receivedData = SPDR;
  Serial.println("Byte received");
  Serial.print("Binary: ");
  Serial.print(receivedData, BIN);
  Serial.print(", HEX: ");
  Serial.println(receivedData, HEX);
  i++;
}