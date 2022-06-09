// Initialsierung der benötigten Bibliotheken
#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"





#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
// Pinbelegung für die Kommunikation
// Kann nach belieben gewählt werden
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14
#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2


#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST

#else
#define RCV_HOST    SPI2_HOST

#endif
#define BUFFER_SIZE 64

WORD_ALIGNED_ATTR uint8_t sendbuf[BUFFER_SIZE];    // Sendebuffer
WORD_ALIGNED_ATTR uint8_t recvbuf[BUFFER_SIZE];    // Empfangsbuffer
spi_slave_transaction_t t;    // Objekt für die SPI-Kommunikation
int counter=0;                // counter um den Buffer auszulesen
char Typ;                     // Einlesen des Identifiers
float wert=0;                 // Wert für das zwischenspeichern der Empfangenen Daten         
esp_err_t ret;                // Wird zur Überprüfung ob Daten Empfangen wurden benötigt



// Wird aufgerufen nachdem die Empfangenen Daten verarbeitet wurden 
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    // Handshake-Line auf High setzen
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));  
}

//Wird aufgerufen nachdem Daten Empfangen wurden
void my_post_trans_cb(spi_slave_transaction_t *trans) 
{   // Handshake-Line auf Low
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
    
}

void print_buffer() {  
    while (counter<BUFFER_SIZE-4)  // solange counter nicht alle Werte im Buffer ausgegeben hat
    {
        char array[sizeof(float)];   // Array für das zwischenspeichern der Messwerte
        Typ=(char)(recvbuf[counter]);  // Identifier wird aus dem Buffer in Typ eingelesen
    
    // switch case mit dem Identifier zur Identifizierung des Messwerts
    
    Serial.println(wert);
    //Typ = toUpperCase(Typ);
    Serial.println(Typ);
   switch (Typ)
   {
    case 'F':  // falls Flow
        Serial.print("Float Wert: "); // Ausgabe Flow
        for (int i=0;i<sizeof(float);i++) 
        {
            // einzelne Bytes des Messwerts in Array einlesen
            array[i]=(char)(recvbuf[i+1+counter]);
           
        }
        // einzelne Bytes in float Wert kopieren
        memcpy(&wert,array, sizeof(float));
        Serial.println(wert); // Ausgabe des Messwerts
        counter=counter+6;  // counter inkrementieren für den nächsten Messwert
        break;
    case 'P':  // gleiches Vorgehen für Messwert
        Serial.print("Pressure Wert: ");
        for (int i=0;i<sizeof(float);i++)
        {
            array[i]=(char)(recvbuf[i+1+counter]);
           
        }
        Serial.println(wert);
        memcpy(&wert,array, sizeof(float));
         counter=counter+6;
        break;
    case 'O':  // gleiches Vorgehen für Oxygen Wert
        Serial.print("Oxygen Wert: ");
        for (int i=0;i<4;i++)
        {
            array[i]=(char)(recvbuf[i+1+counter]);
           
        }
        memcpy(&wert,array, sizeof(float));
        Serial.println(wert);
        counter=counter+6;

        break;  
    case 'V':
        Serial.print("Volumen Wert: ");
        Serial.println(wert);
        counter=counter+6;

        break;          
    default:  // Falls der gelesene Identifier nicht mit den definierten übereinstimmt
    Serial.println("ERRORR");  // Ausgabe Error
    counter=counter+6;  
    break;
    

   }
    wert=0;  // Wert nach jedem Durchlauf auf Null setzen
    
   }
   	
   
}

void setup()
{
Serial.begin(115200);   // Serial-Monitor starten 
}

// Loop
void loop()
{
    Serial.print("AAAAAAAAAAAAAAAAAAAAA");
    // Konfiguration SPI-Bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Konfoguration des Slave Mode
   spi_slave_interface_config_t slvcfg;
        slvcfg.mode=0;
        slvcfg.spics_io_num=GPIO_CS;
        slvcfg.queue_size=3;
        slvcfg.flags=0;
        slvcfg.post_setup_cb=my_post_setup_cb;
        slvcfg.post_trans_cb=my_post_trans_cb;
    
   

    //Konfiguration Handshake-Line
    gpio_config_t io_conf;
        io_conf.intr_type=GPIO_INTR_DISABLE;
        io_conf.mode=GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask=(1<<GPIO_HANDSHAKE);
    

    //Handshake-Line als Output definieren
    gpio_config(&io_conf);
    // enable Pull-Ups
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);

    //Slave wird initialisiert
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg,SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);

    
    memset(recvbuf, 0, BUFFER_SIZE);   // Empfangsbuffer auf 0 setzen
    pinMode(64,OUTPUT);
    memset(&t, 0, sizeof(t));         // Transaktionsbuffer auf Null setzen
  
    // Endlosschleife 
    while(1)
     {

        
        // send und receive Buffer den Transaktionsbuffer zuweisen
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;
        t.length=BUFFER_SIZE*8;
       

    
        // Daten in t empfangen
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        // Aufruf der Funktion print_Buffer
        print_buffer();
        Serial.println(" ");
        // counter im Anschluss auf 0 setzen
        counter=0;
   
    }
    
}
