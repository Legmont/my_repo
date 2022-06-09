#include <Arduino.h>
#include <AMS.h>
#include <AMS.cpp>
#include <SFM3000CORE.h>
#include <iostream>
using namespace std;

// Ekicioglu SPI
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "xtensa/core-macros.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <lwip/sockets.h>
//#include <lwip/dns.h>
//#include <lwip/netdb.h>
//#include <lwip/igmp.h>
//#include "xtensa/core-macros.h"

//#include <esp_wifi.h>
//#include <esp_system.h>
//#include <esp_event.h>
//#include <nvs_flash.h>
#include <soc/rtc_periph.h>
#include <driver/spi_master.h>
//#include <esp_log.h>
//#include <esp_spi_flash.h>

#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <soc/cpu.h>



/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

#elif CONFIG_IDF_TARGET_ESP32C3
#define GPIO_HANDSHAKE 3
#define GPIO_MOSI 7
#define GPIO_MISO 2
#define GPIO_SCLK 6
#define GPIO_CS 10

#elif CONFIG_IDF_TARGET_ESP32S3
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 11
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 10

#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2


#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST HSPI_HOST
#define USEDPIN 26
#define BUFFER_SIZE 62
#else
#define SENDER_HOST SPI2_HOST

#endif

 esp_err_t ret;
 spi_device_handle_t handle;


int counter=0;
float xK=0;
uint8_t sendbuf[BUFFER_SIZE] = {0};
uint8_t recvbuf[BUFFER_SIZE] = {0};
gpio_num_t GPIO_HANDSHAKe = GPIO_NUM_2;
void set_buffer_Flow(float Wert) 
{
  
  sendbuf[4+counter]='F';
  char array[sizeof(float)];
  memcpy(array, &Wert, sizeof(float));
  for(int i=0;i<4;i++)
  {
    sendbuf[i+counter+5]=array[i];
  }
  counter=counter+6;
  
}

void set_buffer_Oxygen(float Wert2) 
{
  
  sendbuf[4+counter]='O';
  char array[sizeof(float)];
  memcpy(array, &Wert2, sizeof(float));
  for(int i=0;i<4;i++)
  {
    sendbuf[i+counter+5]=array[i];
  }
  counter=counter+6;
  
}
void set_buffer_Pressure(float Wert2) 
{
  
  sendbuf[4+counter]='P';
char array[sizeof(float)];
  memcpy(array, &Wert2, sizeof(float));
  for(int i=0;i<4;i++)
  {
    sendbuf[i+counter+5]=array[i];
  }
  counter=counter+6;
  
}

static xQueueHandle rdySem;


static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime;
    uint32_t currtime=XTHAL_GET_CCOUNT();
    uint32_t diff=currtime-lasthandshaketime;
    if (diff<240000) return; //ignore everything <1ms after an earlier irq
    lasthandshaketime=currtime;

    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
   
}

spi_transaction_t t;
int spi_flag = 0;
// ENDE SPI


////// DEFINES //////
#define laenge 3000
const int shutoff =26;              ////// GPIO SETUP - SHUT OFF VALVE //////

////// TOUCHPIN //////
#define TOUCHPIN 4
int touchVal = 100;
int a = 0;

////// PWM SETUP - FOOD VALVE //////
const int pwmPin = 27;
const int freq = 800;
const int pwmCh = 0;
const int resolution = 8;
double pwm_remapped = 0;
double pwm_alt = 0;

////// REGLER SETUP //////****************************************************************************
// Regelkreis
double e_raw = 0.0;
double x_raw = 0.0;
double w_peep = 5.0;
double w_pinsp = 20.0;            // Soll - WErt in mbar
double e = 0;                   // Regelabweichung = w - x
double x = 0;                   // Rückführgröße aus Sensor - gemappt auf -1..1 für Druckberich: 0mbar..40mbar..80mbar
double w = 0;
double w_1 = (w_peep - 40.0)/40.0; //-0.7;          // Soll - Wert - Eingabe als -1..1 --> 0 entspricht 40mbar
double w_2 = (w_pinsp - 40.0)/40.0;
long int cntr = 0;              // Counter für Anzahl Messwerte
double regler_out = 0;          // Regler Ausgang

// P - Regler

// PI Regler stats: kp = 2, ki 0.05, i_out 0.15 bei 1 bar
// Pi: kp=1.5, ki=1 bei 1,9bar
// I - Regler

double delta_t = 0.01196;//0.0094598;     // Zeit zwischen 2 Messpunkten in ms mal 1000 für sekunden
double I_sum = 0;               // Summe Funktionswerte für Integrierer
double I_out = 0;               // Reglerausgang I_sum * Ki     

//*****************************************************************************************************
////// FLOW SENSOR VARIABLEN //////
SFM3000CORE senseFlow(0x40);
float offset = 32000.0;      // offset wird gesetzt mit 32000
float scale = 140.0;       // Scale für Luft beträgt 140
float sfm_flow = 0;        // Sensor output 
float mw_sfm = 0.0;
float mem_sfm[5000];
#define delta_sfm 4.0

////// AMS DRUCKSENSOR SETUP //////
AMS ams_sens(6915, 0x28,-100, 100);   // 4,7k recommended as pullup, // (sensor_fam, adress, min_pressure, max_pressure)
float ams_pressure;             // Druck in mbar
double ams_mapped;              // Druck gemapped auf -1..1 
float mem_ams[5000];              // Array für gleitenden Mittelwert - Druck
int ams_cntr = 0;               // Array Pointer MW Druck
float mw_ams = 0;               // Mittelwert Druck
float mem_pwm [5000];

////// Zeiten //////
#define Tinsp 2000
#define Texsp 3000
clock_t peep_start = 0;
clock_t pinsp_start = 0;
//clock_t start_mode = 0;

////// Flags //////
bool regler_an = 0;
bool flag_start = 0;
bool flag_peep = 0;
bool flag_pinsp = 0;
bool flag_sptn = 0;

////// SWITCH CASE SETUP //////
enum cmd_State {mState_1, mState_2, mState_3, mState_4, mState_5};
cmd_State state = mState_1;

// blaue lung: 
// p = 2.5bar, kp = 1.0, ki = 0.1

////// Reglerverstärker //////
const double Kp = 3.0;          // Verstärkungsfaktor P-Regler
const double Ki = 0.2; //10;//0.05;//60;//5;          // Verstärkungsfaktor I-Regler
const double Kd = 0.0;// 0.03;//0.5;          // Verstärkungsfaktor D-Regler

#define iLim 0.25
double I_limit (double soll)
{
  if (soll <= 35.0)
  {
    return 5.0;
  }
  else
  {
    return 7.0;
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  //senseFlow.init(); //S+
  Serial.println("D");
  pinMode(shutoff, OUTPUT);
  digitalWrite(shutoff, LOW);

  ledcSetup(pwmCh, freq, resolution);
  ledcAttachPin(pwmPin, pwmCh);

  clock_t peep_start = 0;
  clock_t pinsp_start = 0;
  
  if (ams_sens.Available()==false)
  {
      Serial.println("AMS nicht initialisiert");    
  }
  else
  {
    Serial.println("Sensoren initialisiert!");
  }
// SPI ALP
//Configuration for the SPI device on the other side of the bus
      
    // SPI ENDE
}
void loop()
{
  if(spi_flag == 0)
  {
    spi_flag=1;
    //SETUP 
        //Configuration for the SPI device on the other side of the bus
    spi_bus_config_t buscfg={

        .mosi_io_num=GPIO_MOSI,

        .miso_io_num=GPIO_MISO,

        .sclk_io_num=GPIO_SCLK,

        .quadwp_io_num=-1,

        .quadhd_io_num=-1

    };
    static spi_device_interface_config_t devcfg;
    devcfg.command_bits=0;
    devcfg.address_bits=0;
    devcfg.clock_speed_hz=4000000;
    devcfg.duty_cycle_pos=128;
    devcfg.mode=0;
    devcfg.spics_io_num=GPIO_CS;
    devcfg.cs_ena_posttrans=3;
    devcfg.queue_size=3;

          //GPIO config for the handshake line.
    gpio_config_t io_conf;
        io_conf.intr_type=GPIO_INTR_POSEDGE,
        io_conf.mode=GPIO_MODE_INPUT;
        io_conf.pull_up_en=GPIO_PULLUP_ENABLE;
        io_conf.pin_bit_mask=(1<<GPIO_HANDSHAKE);

     

    
    memset(&t, 0, BUFFER_SIZE);

       //Create the semaphore.
    rdySem=xSemaphoreCreateBinary();

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKe, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKe, gpio_handshake_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(SENDER_HOST, &devcfg, &handle);
    assert(ret==ESP_OK);
    
    
    xSemaphoreGive(rdySem);
    

  }
  state =mState_3; //S+
  switch(state)
  {
    case mState_1: // Trigger Touch pos. Flanke
          {
            touchVal = touchRead(TOUCHPIN);
            if (touchVal < 50)
            {
              state = mState_2;
              Serial.println("State 1 DONE");
            }
            

            break;
          }
    case mState_2: // Trigger Touch neg. Flanke
          {
            touchVal = touchRead(TOUCHPIN);
            if (touchVal > 50)
            {
              state = mState_3;
              //Serial.println("State 2 DONE");
              //Serial.println(clock());
              //start_mode = clock();
            }
            break;
          }
    case mState_3: // Zustand für Sensorik 
          {
            //Serial.print(clock());
            //Serial.println("State3_ENTERED");
            ////// Wenn Regler zuvor aus, dann erst PEEP anfahren
            if (regler_an == 0)
            { 
              w = w_1;
              regler_an = 1;
              flag_peep = 1;
            /*  if(clock() >= start_mode + 1000) 
              {
                regler_an = 1;
              }              
            */
            }
            
            ////// Sensorik auslesen und in Array speichern //////
            //ams_pressure=ams_sens.readPressure();           // Drucksensor auslesen
            //sfm_flow = ((senseFlow.getValue()-offset) / scale);

            ams_pressure = 5.4;
            set_buffer_Pressure(ams_pressure);
            sfm_flow = 0.7; //S+
            Serial.println("A");
            mem_ams[ams_cntr] = ams_pressure;
            mem_sfm[ams_cntr] = sfm_flow;
            set_buffer_Flow(sfm_flow);

            
            /*xK++;
            if(xK>=50)
            {
              xK= 0;
            }*/
            if(counter >= 60)
            {
              counter=0;
              t.tx_buffer=sendbuf;
              t.rx_buffer=recvbuf;
              t.length=sizeof(sendbuf)*8;
              xSemaphoreTake(rdySem, portMAX_DELAY);
              ret=spi_device_transmit(handle, &t);
              Serial.println("Sendit");
            }
            ams_cntr++;
            //Serial.print(ams_pressure);
            //Serial.print(" ");
            //Serial.print(sfm_flow);
            //Serial.print(" ");
/*            if(ams_cntr < 20)
            {
              mem_ams[ams_cntr] = ams_pressure;
              mem_sfm[ams_cntr] = sfm_flow;               // Sensorwert [mbar] in Array abspeichern
              mem_pwm[ams_cntr] = pwm_alt;
              ams_cntr++;;
              cntr++;
            }
*/
            ////// Mittelwertberechnung Druck und Flow /////// -> immer 20 aktuelleste Werte gemittelt
            if (ams_cntr >= 20)
            {
              for (int i = 0; i <= 20; i++) 
              {
                mw_ams = mw_ams + mem_ams[i];
              }
              mw_ams = mw_ams / 21.0;

              for (int j = 0; j <= 20; j++)
              {
                mw_sfm = mw_sfm + mem_sfm[j];
              }
              mw_sfm = mw_sfm / 21.0;
              ams_cntr = 0;
            }
          
            //Serial.print(mw_ams);
            //Serial.print(" ");
            //Serial.println(mw_sfm);
          
            ////// Sensorwert skalieren und Regelabweichung berechnen //////          
            x = ams_mapped = ((double)ams_pressure - 40.0)/40.0;  // Hier Drucksensor von -1..0..1, P= 0..40..80mbar, AP=40mbar
            e = w - (x); 
            //ams_cntr++;

            if (flag_peep == 1)
            {
              state = mState_4;
              flag_pinsp = 0;
            }
            else if (flag_pinsp == 1)
            {
              state = mState_5;
              flag_peep = 0;
            }
            

          /*  if(clock() >= start_mode + 1000)
            {
              if(w = w_peep)
              {
                state = mState_4;
              }
              else 
              {
                state = mState_5;
              }
              //state = mState_4;
            }
            else
            {
              state = mState_3;
            }
          */ 
          
            //Serial.println("State3_LEFT");          
            break;
          }
    case mState_4: // Zustand für Ventilansteuerung PEEP niveau
          { //Serial.println("State4_entered"); 
            
            if ((I_out <=iLim) && (e > 0))
            {
              I_sum = I_sum + e*delta_t;
            }
            else if((I_out >= -iLim) && (e < 0))
            { 
              I_sum = I_sum + e*delta_t;                                  // Berechnung der Summe aller Abweichungen
            }

            /*  if ( ((mw_e)*40.0) > (5.0))// ( (w*0.3)*(w*0.3) ))//( ((28.0-40)/40)*((28.0-40)/40) ) )           // Quadrate zum einfachen Bilden des Betrags: z.B. (e)^2 > (z.B. 5 mBar)^2
                {
                    I_sum = 0.0;
                }
            */
            I_out = 0.5 * I_sum;  // Hier ki geändert für regelung auf 5 mbar !! von 0.2 auf 0.5

            if(I_out>iLim)
            {
              I_out=iLim;
            }
            else if(I_out < -iLim)
            {
              I_out= -iLim;
            }

            ////// Regler Ausgang berechnen und steuern ////// 
            regler_out = (Kp * e) + (I_out);          // Reglerausgang: P-Regler * I-Regler * D-Regler
            
            ////// Wenn Druck zu gering, langsam inkrementieren
            if(ams_pressure <= 2.0)                                  // Grund Druck sicherstellen: wenn P<8mbar, dann erst aufbauen 
            {
              //pwm_remapped = 120;     // "pwm_alt" ist DIE PWM-Größe schlechthin. Siehe auch oben beim seriellen printen.
              pwm_alt = pwm_alt + 2.0; //110.0;
              I_sum = 0;                          //Wichtig: //I-Summe auf Null halten, bis der Regler überhaupt einsetzt. Somit wird ein unnötiges (und hinderliches!!) Wind-Up am Beginn unterbunden!
              ledcWrite(pwmCh, pwm_alt);
            }
            else                                               // Wenn Druck i.O, dann Reglerausgang auf Ventil geben
            {     
              ////// Regler_out skalieren auf 8-bit PWM
              pwm_remapped = (regler_out * 195.0) + 55.0;//+ 170;
              //pwm_alt = pwm_remapped;

              ////// Wenn delta zu hoch dann in 5er Schritten //////
              if((pwm_remapped - pwm_alt) > 5.0)              // Während PWM Differenz > 5, inkrementiere pwm um max. 5
              {
                pwm_alt = pwm_alt + 5.0;                      // Runterschalten ist doch(!) :-) begrenzt
              }
              else if((pwm_remapped - pwm_alt) < -5.0)        // Während PWM Differenz < 5, dekrementiere pwm um max. 5
              {
                pwm_alt = pwm_alt - 5.0;                   
              }
              else
              {
                pwm_alt = pwm_remapped;                  // wenn Betrag der PWM-Differenz <= 5, dann direkte PWM-Ausgabe auf Ventil
              }

              ////// Wenn PWM außerhalb 8-bit, dann korrigieren //////
              if(pwm_alt > 255.0)                                // Kontrolle, ob PWM remapped im Bereich 8-bit PWM
              {
                pwm_alt = 255.0;
              }
              else if(pwm_alt < 0.0)
              {
                pwm_alt = 0.0;
              }

              ////// PWM auf Pin schalten //////
              ledcWrite(pwmCh, pwm_alt);                   // Ausgabe der PWM. Nur hier. Jawohl!
            }

//***********////// Wenn FV versucht mehr als nur "zu" zumachen, dann SV auf. 
            if((e < 0.1) && (pwm_alt <=65))  //(e < 0) && 
            { 
              if(mw_ams >= 5.5)
              {
                //Serial.println("SV auf");
                digitalWrite(shutoff, HIGH);
              }
            }
            else if(mw_ams < w_peep) // pwm_alt > 80) //ams_pressure <= w_peep
            {
              //Serial.println("SV zu");
              digitalWrite(shutoff, LOW);
            }
            

            if (flag_start == 0)
            {
              peep_start = clock();
              flag_start = 1;
            }
            
            ////// Spontanatmung ermöglichen ab Erreichen des PEEP - Levels //////
            if((mw_ams >= 4.5) && (mw_ams <= 5.5))
            {
              
              if(mw_sfm >= delta_sfm)
              {
                //flag_sptn = 1;
                flag_peep = 0;
                flag_pinsp = 1;
                w = w_2;
                flag_start = 0;
                state = mState_3;
              }
            }
          
            if (clock() >= peep_start + Texsp)
            {
              flag_peep = 0;
              flag_pinsp = 1;
              w = w_2;
              flag_start = 0;
              state = mState_3;
            }
            else
            {
              state = mState_3;
            }

          /*  //cntr ++;
            if (cntr >= laenge)
            {
              state = mState_5;
              Serial.println (clock());
            }
            else
            { 
              state = mState_3;
            }
          */
         //Serial.println("State4_left"); 
          break;
          }
    case mState_5: // Zustand für Ventilansteuerung PINSP niveau
          {
            //Serial.println("State5_entered"); 
            if ((I_out <=iLim) && (e > 0))
            {
              I_sum = I_sum + e*delta_t;
            }
            else if((I_out >= -iLim) && (e < 0))
            { 
              I_sum = I_sum + e*delta_t;                                  // Berechnung der Summe aller Abweichungen
            }

        /*  if ( ((mw_e)*40.0) > (5.0))// ( (w*0.3)*(w*0.3) ))//( ((28.0-40)/40)*((28.0-40)/40) ) )           // Quadrate zum einfachen Bilden des Betrags: z.B. (e)^2 > (z.B. 5 mBar)^2
            {
                I_sum = 0.0;
            }
        */
            I_out = Ki * I_sum;

            if(I_out>iLim)
            {
              I_out=iLim;
            }
            else if(I_out < -iLim)
            {
              I_out= -iLim;
            }

            ////// Regler Ausgang berechnen und steuern ////// 
            regler_out = (Kp * e) + (I_out);          // Reglerausgang: P-Regler * I-Regler * D-Regler
            
            ////// Wenn Druck zu gering, langsam inkrementieren
            if(ams_pressure <= 5.0)                                  // Grund Druck sicherstellen: wenn P<8mbar, dann erst aufbauen 
            {
              //pwm_remapped = 120;     // "pwm_alt" ist DIE PWM-Größe schlechthin. Siehe auch oben beim seriellen printen.
              pwm_alt = pwm_alt + 2.0; //110.0;
              I_sum = 0;                          //Wichtig: //I-Summe auf Null halten, bis der Regler überhaupt einsetzt. Somit wird ein unnötiges (und hinderliches!!) Wind-Up am Beginn unterbunden!
              ledcWrite(pwmCh, pwm_alt);
            }
            else                                               // Wenn Druck i.O, dann Reglerausgang auf Ventil geben
            {     
              ////// Regler_out skalieren auf 8-bit PWM
              pwm_remapped = (regler_out * 195.0) + 55.0;//+ 170;
              //pwm_alt = pwm_remapped;

              ////// Wenn delta zu hoch dann in 5er Schritten //////
              if((pwm_remapped - pwm_alt) > 5.0)              // Während PWM Differenz > 5, inkrementiere pwm um max. 5
              {
                pwm_alt = pwm_alt + 5.0;                   // Runterschalten ist doch(!) :-) begrenzt
              }
              else if((pwm_remapped - pwm_alt) < -5.0)              // Während PWM Differenz < 5, dekrementiere pwm um max. 5
              {
                pwm_alt = pwm_alt - 5.0;                   
              }
              else
              {
                pwm_alt = pwm_remapped;                  // wenn Betrag der PWM-Differenz <= 5, dann direkte PWM-Ausgabe auf Ventil
              }

              ////// Wenn PWM außerhalb 8-bit, dann korrigieren //////
              if(pwm_alt > 255.0)                                // Kontrolle, ob PWM remapped im Bereich 8-bit PWM
              {
                pwm_alt = 255.0;
              }
              else if(pwm_alt < 0.0)
              {
                pwm_alt = 0.0;
              }

              ////// PWM auf Pin schalten //////
              ledcWrite(pwmCh, pwm_alt);                   // Ausgabe der PWM. Nur hier. Jawohl!
            }

            if (flag_start == 0)
            {
              pinsp_start = clock();
              flag_start = 1;
            }

//***********////// Ausatmen ermöglichen, wenn Überdruck von 1% mbar auf Pinsp - Level //////
            //if(flag_sptn == 1) //
            
            /*  // Variante um auf pinsp zurückzuregeln, geht nicht da 2/1 Wege ventil, hartes ein und ausschalten
              if(mw_ams >= w_pinsp + 3.0)
              {
                digitalWrite(shutoff,HIGH);
                if(mw_ams <= w_pinsp)
                {
                  digitalWrite(shutoff, LOW);
                }
              }
            */

              if(mw_ams >= w_pinsp + w_pinsp * 0.1)
              {
                flag_peep = 1;
                w = w_1;
                flag_pinsp = 0;
                flag_sptn = 0;
                flag_start = 0;
                state = mState_3;
              }
          
            if (clock() >= pinsp_start + Tinsp)
            {
              flag_peep = 1;
              w = w_1;
              flag_pinsp = 0;
              flag_start = 0;
              state = mState_3;
            }
            else
            {
              state = mState_3;
            }

            //Serial.println("State5_left");                    
            break;
          }
    /*case mState_6: // Zustand für Ausgabe des Arrays
          { 
            ledcWrite(pwmCh, 0);
            digitalWrite(shutoff,HIGH);
            //Serial.println(clock());
            //delay(2000);
            
            
            for ( int i = 0; i< ams_cntr; i++)
            {
              Serial.print(mem_ams[i]);
              Serial.print(" ");
              Serial.print(mem_sfm[i]);
              Serial.print(" ");
              Serial.println(mem_pwm[i]);
            }
            ams_cntr = 0;
            digitalWrite(shutoff,LOW);
            Serial.print("Anzahl Messpunkte: "); 
            Serial.println(cntr);
            //Serial.println(I_limit(w_raw));
            Serial.println("State 5 DONE");
            cntr=0;                                               // counter nullen
            state = mState_1;                                     // Sprung in state 1, warte auf Toucheingabe
            break;
          }
    */
  }

  /*
  zweite statemaschine erstellen
  den selben taster abfragen wie in 1. state machine, flag setzen, sobald 1. state getriggert


  // hier nächstes state machine
  //zb sollwert auf trigger ändern 
  */
}
