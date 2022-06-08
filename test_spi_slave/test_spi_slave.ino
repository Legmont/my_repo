#include <SPI.h>

#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_CS   15
static const int spiClk = 240000000; // 1 MHz
SPIClass * hspi = NULL;

char buff[]="Hello Slave\n";

//byte buff[] = {0xAA, 0xBB, 0xAA, 0x01, 0x89, 0xAB, 0xCD, 0xEF};

void setup() {
Serial.begin(9600);
hspi = new SPIClass(HSPI);
hspi->begin();
hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS); //SCLK, MISO, MOSI, SS
pinMode(HSPI_CS, OUTPUT); //HSPI SS
}

void loop() {
 for(int i=0; i<sizeof buff; i++)
 {
  SPI.transfer(buff[i]);
  Serial.println(buff[i]);
//https://www.makerguides.com/master-slave-spi-communication-arduino/
 }
}
// Interrupt function
ISR (SPI_STC_vect) 
{
 char c = SPDR;        // read byte from SPI Data Register
  if (i < sizeof(buff)) {
    str [i++] = c; // save data in the next index in the array buff
    if ( (c == '\r') || (c == '\n') || (c=='\0') ) //check for the end of the word
      pin = true;
  }
}
