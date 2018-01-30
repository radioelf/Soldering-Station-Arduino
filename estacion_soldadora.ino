/*************************************************************************************
Estación de-soldadora, control de desoldador JBC (24V), soldador Hakko T12 (24V 75W) y soldador Weller RT 1 (12V 40W)

Basado en:
https://github.com/ConnyCola/SolderingStation
https://github.com/FlyGlas/WMRP
http://dangerousprototypes.com/forum/viewtopic.php?f=56&t=5264

---------------------------------------------------------------------------------------
termopares 
B->100°C 0.033mV, 500°C 1.24184 mV
S->100°C 0.645mV, 500°C 4.23329 mV
R->100°C 0.647mV, 500°C 4.47126 mV
N->100°C 2.774mV, 500°C 16.7478 mV ->JBC
K->100°C 4.096mV, 500°C 20.6442 mV 
T->100°C 4.278mV, 400°C 20.8719 mV
J->100°C 5.269mV, 500°C 27.3926 mV
E->100°C 6.319mV, 500°C 37.0053 mV

D NO estándar->100°C 1.145mV, 300°C 4.286mV, 400°C 6,129, 500°C 8,077mV -soldador Weller 12V RT1
C NO estándar->100°C 1.451mV
G NO estándar->100°C 0.344mV

T12 desconocido -> V = 0.00001 × (T * T) + 0,0162 × T -0.6534, obtenemos la tensión conociendo la temperatura
T =(-0.0162 + sqrt(0.00026244 + -4 * 0.00001 *(-(0.6534 + ((float)ADC * (mV_ADC - OFFSET)  / GANANCIA))))) /(2 * 0.00001)

----------------------------------------------------------------------------------------
funciones de las librerías TFT:
tft.setRotation(1)                             orientación pantalla, 1->horizontal, 0-> vertical
tft.setCursor (x0,y0,)                         posiciona el cursor en las posición X0, Y0
tft.print("texto")                             escribe el texto indicado
tft.print(X, DEC)                              escribe el valor de las variable x en decimal
tft.setTextSize(3)                             tamaño del texto a 3,  1 = 5x8, 2 = 10x16 3 = 15x24 
tft.setTextColor(color)                        color del texto
tft.setTextWrap(true)                          true-> sigue en la siguiente linea
tft.fillScreen(color)                          borra la pantalla con el color indicado
tft.fillRect (x0,y0, W6, H4, color)            rellena un recuadro de color del punto x0,y0 a con una anchura de 6 y una altura de 4
tft.drawPixel(x0,y0, color)                    pinta el pixel indicado
tft.drawFastHLine(x0,y0, L, color)             dibuja una línea horizontal de la longitud L
tft.drawFastVLine(x0,y0, H, color)             dibuja una línea vertical de la longitud H
tft.fillCircle(x, y, radio, color)             dibuja un circulo relleno
tft.drawCircle(x, y, radio, color)             dibuja el contorno de un circulo
tft.drawRect(X0, Y0, X1, Y1,color)             dibuja un rectángulo
int X = tft.width(),                           devuelve la anchura de la pantalla 
int X = tft.height(),                          devuelve la altura

128*160 
El punto (0,0) es la esquina superior izquierda y el punto (160,128) la esquina inferior derecha. (vertical)

    Y->          
  X0-Y0         X0-Y128
  X *-------------*
  | |             |
  V //           //
    |             |
    *-------------*               
  X160-Y0       X160-Y128  

color:
ST7735_BLACK   0x0000
ST7735_BLUE    0x001F
ST7735_RED     0xF800
ST7735_GREEN   0x07E0
ST7735_CYAN    0x07FF
ST7735_MAGENTA 0xF81F
ST7735_YELLOW  0xFFE0
ST7735_WHITE   0xFFFF
ST7735_GRAY    0xCCCC
ST7735_ORANGE  0xFA60 

*******************************************************************************************/
#include <SPI.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <eRCaGuy_NewAnalogRead.h>        // http://electricrcaircraftguy.blogspot.com/ libreria de sobremuestreo ADC (es necesario tener ruido en ADC)

#include "iron.h"
#include "stationLOGO.h"

#define VERSION "0.7"		                  // versión

// definición de los pines para control de la pantalla (bus SPI)
#define BLpin         47                  // retroiluminación 
#define dc            48                  // D/C
#define rst           49                  // reset del display
// SPI por hardware
//#define miso        50                  // SDO, entrada de datos del master
//#define mosi 	      51	                // SDI, salida de datos del master
//#define sclk        52                  // SCK, pulso sincronización
#define cs_tft        53	                // Selección LCD

#include <Adafruit_GFX.h>                 // Core graphics library V 1.0.1 
#include <Adafruit_ST7735.h>              // Hardware-specific library, driver IC:  ST7735

#define STANDBY       5                   // standby para de/soldador
#define LED_R         6                   // led bicolor estado rojo
#define LED_V         7                   // led bicolor estado verde 
#define ZUMBADOR      8                   // zumbador indicación
#define PWMpin        9                   // PWM de/soldador TIMER 2
#define BOMBA_PUL     10                  // pulsador bomba desoldador
#define BOMBA         11                  // relé bomba de vació del desoldador 
#define ADC_V         A0                  // ADC_V0 entrada analógica tensión de alimentación
#define ADC_T         A1                  // entrada analógica termopar soldador
#define ADC_T_OFFSET  A2                  // entrada analógica sensor TC1047, ambiente OFFSET
#define ADC_I         A3                  // entrada analógica sensor ACS712T-5, intensidad (0A =512, 185mV por Amperio-5A)
#define LED_ADC       4                   // indica lectura temperatura ADC

#define ENCODER_CLK   2                   // pin del encoder pin 2 (INT0) para CLK 
#define ENCODER_DT    3                   // pin del encoder pin 3 (INT1) para B
#define PULSADOR      18                  // pulsador encoder (INT5)

#define V_24          0
#define V_12          1
#define desoldador    0
#define soldador      1
#define OFF           0
#define ON            1

#define ERROR_OFFSET 0.318                 // error offset
#define GANANCIA_AMP 471                   // ganancia  amplificador 
// factores conversión
#define mV_ADC       4.8876                // 0-5V, ADC 0-1023, mV = adc* mV_ADC
#define mV_ADC_12b   1.12219               // 0-5V, ADC 0-4092 (12bits)
#define V_ADC        0.0048876             // 0-5V, ADC 0-1023, V = adc* V_ADC
// modulo ACS712
#define ACSoffset    2500                  // 0A = 2.5V->2500->512 (1024/2)
#define mV_Amp       185                   // 185 para modulo de 5A, 100 para modulo de 20A, y 66 para modulo de 30A
// tipo de sensor de tenperatura de los soldadores                                        
#define JBC          1                     // termopar tipo E
#define RT1          2                     // termopar tipo D
#define T12          3                     // termopar tipo desconocodo ->V = 0.00001 × (T * T) + 0,0162 × T -0.6534
// intensidades en Amp. según tipo de soldador
#define I_T12        2.15                  // intensidad de 2.27A a 24V
#define I_JBC        2.70                  // intensidad de 2.96A a 24V    
#define I_RT1        3.75                  // intensidad de 3.89 a 12V
                                                           
#define ST7735_GREY    0xCCCC              // color gris NO incluido en librería
#define ST7735_ORANGE  0xFA60              // color naranja NO incluido en librería
// Creamos una instancia del objeto Adafruit_ST7735 que llamamos  tft
Adafruit_ST7735 tft = Adafruit_ST7735(cs_tft, dc, rst);  // hardware SPI

byte bitsOfResolution = 12;               // resolución de 12 bis para sobremuestreo  ADC
unsigned long numSamplesToAvg = 4;        // número de muestras de 12 bits

static float TempC_Emf[11];               // array para los coeficientes  de temperatura a tensión
static float Emf_TempC[10];               // array para los coeficientes  de tensión a temperatura
byte  longitud_TempC_Emf =0;              // longitud del araray con datos validos que contiene los coeficientes de temperatura a tensión
byte  longitud_Emf_TempC =0;              // longitud del araray con datos validos que contiene los coeficientes de tensión a temperatura

// Variables para la librería del PID
double Setpoint, Input, Output;
float KP_S = 2.089;                       // valores iniciales para PID control soldador 24V
float KI_S = 0.417;
float KD_S = 2.615;

float KP__S = 0.5;                        // valores iniciales para PID control soldador 12V
float KI__S = 0.2;    
float KD__S = 0.50;    

float KP_D = 12.62;                        // valores iniciales para PID control desoldador 
float KI_D = 2.39;
float KD_D = 16.85;

byte    PWM_DIV   = 7;     		            // por defecto TIMER2: 0x04->64->489 Hz, 0x07->1024-> 30 Hz
int     EEPROM_direccion  = 0;            // posición inicial de la EEPROM
int     OK_EEPROM = 50;                   // ultima posición de la EEPROM con datos              

boolean tension_12_24 = V_24;             // soldador de 12 o 24V 
int     tension = 0; 

int     pwm_max = 230;                    // PWM de 0 a 230
int     out_pwm = 0;                      // salida PWM
int     posicion_encoder = 0;             // variable posición virtual encoder inicial 
boolean de_soldador = soldador;           // iniciamos de/soldador (true->soldador, false->desoldador)
byte    TIPO   = T12;                     // tipo de sonda T12 (soldador->defecto)
byte    MEMORIA = 0;                      // memoria de temperaturas (NO implementado)
boolean ON_OFF_zumbador = true;           // permiso de indicación sonora por zumbador
boolean menu = false;                     // No menú
boolean DEBUGGER = OFF;                    // modo debugger ON/OFF 
boolean standby = OFF;                    // modo standby ON/OFF 
boolean inicio = false;
boolean OK = false;
boolean INTRO =true;
float   intensidad = 0;                    // intensidad en A

int   Temp_Max = 450;                     // temperatura máxima 450 °C
int   Temp_Standby = 180;                 // temperatura en modo standby
byte  Temp_Min = 100;                     // temperatura minina (0-255)
int   ini_temp = 280;                     // temperatura inicial de trabajo
int   mem_SB =ini_temp;                   // memoria temperatura de trabajo/standby ON/OFF 
int   Tem__Setpoint = ini_temp;           // temperatura de Setpoin 
int   actual_temperatura = 0;             // temperatura punta de-soldador
int   actual_temperatura_tft =0;          // temperatura media para la gestión del tft

int   media =0;
byte  ciclo_media = 0;
byte  error_consumo =0;
byte  Tiempo_stop = 30;                   // tiempo en minutos máximo en Standby (se apagará y indica por zumbador) 
byte  ciclo_error = 0;                    // número de errores por temperatura alta o error en lectura

unsigned long espera_standby;
unsigned long ciclo_espera =0;

union Float_Byte{
  float datoF;
  byte  datoB[4];
} unionFB;
union Integer_Byte{
  int  datoI;
  byte datoB[2];
} unionIB;

PID myPID(&Input, &Output, &Setpoint, KP_S, KI_S, KD_S, DIRECT);

//Modificar frecuencia PWM para pin 9 TIMER 2 y XT de 16000000 Hz 
//Para el Timer/Counter 2
//Config.   Divisor   Frecuencia PWM
//0x00            -        0
//0x01            1   31,333 Hz
//0x02            8    3,916 Hz
//0x03           32      979 Hz
//0x04           64      489 Hz-> defecto
//0x05          128      244 Hz
//0x06          256      122 Hz
//0x07         1024       30 Hz
//TCCR2B = TCCR2B & 0b11111000 | 0x07;->30Hz 

//******************************FUNCIONES*****************************************************
// Interrupción por pulsación selector encoder
//--------------------------------------------------------------------------------------------
void pulsacion(){
  delayMicroseconds(1000);                      // duración mínima del pulso de 1ms             
  if (digitalRead(PULSADOR) == false){
    menu =true;
  }
}
//********************************************************************************************
// Interrupción por encoder
//--------------------------------------------------------------------------------------------
void encoder(){
static unsigned long  periodo = 0;
unsigned long   ms_inicio = millis();
// si la interrupción se produce antes de 10 ms despreciamos (rebote)
  if ((ms_inicio - periodo) > 10) {
    (digitalRead(ENCODER_DT) ==false) ? posicion_encoder= (posicion_encoder+ 1): posicion_encoder= (posicion_encoder- 1);
  }
  periodo = ms_inicio;
}
//*******************************************************************************************
// Reinicio por software
//-------------------------------------------------------------------------------------------
void(*  soft_reset) (void) = 0;
//*******************************************************************************************
// Calculamos los valores KP, Ki y Kd (basado en código Marlin https://github.com/MarlinFirmware/Marlin )
//------------------------------------------------------------------------------------------
// Obtenemos los valores Kp, Ki y Kd para el PID realizando 10 ciclos 
boolean PID_autotune(float temp, byte num_PID){
boolean calentar = true;
unsigned long temp_millis, t1, t2;                    
long t_high = 0, t_low = 0, bias, d;
int ciclos =0, soft_pwm =0;
byte enfriado =0;
float  input = 0.0, Ku, Tu, Kp, Ki, Kd, max_ = 0, min_ = 10000, current_temperature;
  if (DEBUGGER){
    Serial.println("Autotune PID  iniciado");  
  }
   digitalWrite(LED_V, LOW);
  digitalWrite(LED_R, HIGH);                               // rojo ON, calentando
  analogWrite(PWMpin, 100);
  do{
    delay(100);
    current_temperature = getTemperatura(0, 100);          // despues del menu() el PWM se encuentra a OFF
  }while(current_temperature < temp);                      // esperamos atener la temperatura de PID
  analogWrite(PWMpin, 0);                                  // PWM OFF
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREY);
  tft.setCursor(0,90);
  tft.print("ENFRIANDO...");
  if (ON_OFF_zumbador) zumbador_ON (50);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_V, HIGH);                               // verde ON, enfriando
  do{
    delay(250);
    current_temperature = getTemperatura(0, 0);
    Serial.println(current_temperature);
    if (++ciclos >= 720){                                 // máximo 3 minutos
      if (DEBUGGER){
        Serial.println("FALLO! tiempo enfriamiento excesivo");
      }
      return (false);
    }
    if (ciclos == 1){
      if (DEBUGGER){
        Serial.println("Enfriamiento..");
      }
    }
    if (current_temperature < 100) enfriado++;
    
  }while (enfriado < 4);                                   // esperamos a que baje la temperatura
  if (ON_OFF_zumbador) zumbador_ON (50);
  digitalWrite(LED_R, HIGH);                               // verde +rojo-> naranja, RUN auto PID
  ciclos =0;
  tft.fillRect (0, 90, 118, 20, ST7735_BLACK); 
  tft.setTextSize(2);
  tft.setCursor(40,90);
  tft.print(ciclos);
  soft_pwm = bias = d = (pwm_max)/2;              
  analogWrite(PWMpin, soft_pwm);                           // pwm a 1/2 de pwm_max
  temp_millis = millis();                                  // obtenemos los milisegundos pasados desdé que se inicio la alimentación
  t1 = temp_millis;
  t2 = temp_millis;
  for(;;){
    if (soft_pwm !=0) delay(50);                           // esperamos si estamos calentando (en cada lectura de temperatura paramos PWM)
    current_temperature =getTemperatura(0, soft_pwm);
    if(current_temperature != Temp_Max+100) {              // comprobamos que tenemos una lectura sin error
      input = current_temperature;
      max_ =max (max_ ,input);
      min_ =min (min_ ,input);
      if(calentar == true && input > temp) {
        if(millis() - t2 > 5000) { 
          calentar=false;
          soft_pwm = (bias - d) >> 1;
          analogWrite(PWMpin, soft_pwm);
          t1 =millis();
          t_high=t1 - t2;
          max_ =temp;
        }
      }
      if(calentar == false && input < temp) {
        if(millis() - t1 > 5000) {
          calentar =true;
          t2 =millis();
          t_low =t2 - t1;
          if(ciclos > 0) {                                // ciclos 1, 2, 3...
            long max_pow = pwm_max;
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(max_pow)-20);
            d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;
            if (DEBUGGER){
              Serial.println(" bias: "); Serial.print(bias);
              Serial.println(" d: "); Serial.print(d);
              Serial.println(" min: "); Serial.print(min_);
              Serial.println(" max: "); Serial.print(max_);
            }
            if(ciclos > 2) {                                // ciclos 3, 4 y 5
              Ku = (4.0 * d) / (3.14159 * (max_ -min_) /2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              if (DEBUGGER){
                Serial.println(" Ku: "); Serial.print(Ku);
                Serial.println(" Tu: "); Serial.print(Tu);
              }
              Kp = 0.6 * Ku;
              Ki = 2 * Kp / Tu;
              Kd = Kp * Tu / 8;
              if (DEBUGGER){
                Serial.println(" PID tipico ");
                Serial.println(" Kp: "); Serial.print(Kp);
                Serial.println(" Ki: "); Serial.print(Ki);
                Serial.println(" Kd: "); Serial.print(Kd);
              }
            }
          }
          soft_pwm = (bias + d) >> 1;
          analogWrite(PWMpin, soft_pwm);
          ciclos ++;
          tft.fillRect (40, 90, 118, 20, ST7735_BLACK); 
          tft.setCursor(40,90);
          tft.print(ciclos);
          min_ =temp;
        }
      } 
    }
    else {                                                    // errores en autotune PID
      if (DEBUGGER){
        Serial.println("FALLO! Autotune PID: error en lectura temperatura");
      }
      tft.fillRect (0, 90, 118, 20, ST7735_BLACK); 
      tft.setCursor(0,90);
      tft.setTextSize(1);
      tft.print("ERROR 1: Autotune PID");
      if (ON_OFF_zumbador) zumbador_ON (500);
      delay(3000);
      return (false);
    }
    if(input > (max_ + 100)) {
      if (DEBUGGER){
        Serial.println("FALLO! Autotune PID: temperatura demasiado alta");
      }
      tft.fillRect (0, 90, 118, 20, ST7735_BLACK); 
      tft.setCursor(0,90);
      tft.setTextSize(1);
      tft.print("ERROR 2: Autotune PID");
      if (ON_OFF_zumbador) zumbador_ON (500);
      delay(3000);
      return (false);
    }
    if(millis() - temp_millis > 2000) {                         // cada 2 segundos  
      if (DEBUGGER){
        Serial.print(input);   
        Serial.print(" @:");
        Serial.println((int)soft_pwm);     
      }  
      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > 1200000L) {       // tiempo máximo  =1200000L = 2 minutos
      if (DEBUGGER){
        Serial.println("FALLO! Autotune PID: tiempo excesivo");
      }
      tft.fillRect (0, 90, 118, 20, ST7735_BLACK); 
      tft.setCursor(0,90);
      tft.setTextSize(1);
      tft.print("ERROR 3: Autotune PID");
      if (ON_OFF_zumbador) zumbador_ON (500);
      delay(3000);
      return (false);
    }
    if(ciclos > 10) {                                          // 10 ciclos
      if (DEBUGGER){
        Serial.println("Autotune PID ");
      }
      if (num_PID == 1){                                       // soldador 12V
        KP__S = Kp;
        KI__S = Ki;
        KD__S = Kd; 
        if (DEBUGGER){
          Serial.print("soldador RT1"); 
        }
      }
      if (num_PID == 2){                                       // soldador 24V
        KP_S = Kp;
        KI_S = Ki;
        KD_S = Kd;
        if (DEBUGGER){
          Serial.print("soldador T12");
        }
      }
      if (num_PID == 3){                                       // desoldador 24V
        KP_D = Kp;
        KI_D = Ki;
        KD_D = Kd;
        if (DEBUGGER){
          Serial.print("desoldador JBC");
        }
      }
      if (DEBUGGER){
        Serial.println(" finalizado!");
        Serial.print("KP ");
        Serial.print(Kp);
        Serial.print(" KI ");
        Serial.print(Ki);
        Serial.print(" KD ");
        Serial.println(Kd);
      }
      if (ON_OFF_zumbador) zumbador_ON (50);
      delay(250);
      if (ON_OFF_zumbador) zumbador_ON (50);
      tft.fillRect (0, 90, 118, 20, ST7735_BLACK); 
      tft.setCursor(0,90);
      tft.setTextSize(1);
      tft.print("Autotune PID OK");
      tft.setTextSize(1);
      delay(3000);
      return (true);
    }
  }
}
//********************************************************************************************
// Programación EEPROM
//--------------------------------------------------------------------------------------------
void program_EEPROM(int direccion){
byte posicion =0;
  (direccion == 50) ? EEPROM_direccion =0: EEPROM_direccion =direccion;    
  if(EEPROM_direccion <25){
    for (EEPROM_direccion; EEPROM_direccion <36; EEPROM_direccion++){
      switch (EEPROM_direccion)  {
         case 0:                          // soldador 12V
          unionFB.datoF = KP__S;
         break;
         case 4:
          unionFB.datoF = KI__S;
         break;
         case 8:
          unionFB.datoF = KD__S;
         break;
         case 12:                          // soldador 24V
          unionFB.datoF = KP_S;
         break;
         case 16:
          unionFB.datoF = KI_S;
         break;
         case 20:
          unionFB.datoF = KD_S;
         break;
         case 24:                         // desoldador 24V
          unionFB.datoF = KP_D;
         break;
         case 28:
          unionFB.datoF = KI_D;
         break;
         case 32:
          unionFB.datoF = KD_D;
         break;       
        }
        EEPROM.update(EEPROM_direccion, unionFB.datoB[posicion++]);
        if (posicion ==4){
          if (direccion ==50) posicion =0; else return; // si hemos escrito 4 bytes y NO se envió escritura completa salimos      
        }
      }
    }
    switch (EEPROM_direccion) {
         case 36:
            EEPROM.update(36, MEMORIA);                 // en posicion 36 número de memoria
            if (direccion == 50) EEPROM_direccion =37;
         case 37:
            unionIB.datoI = Temp_Max;
            EEPROM.update(37, unionIB.datoB[0]);        // en posicion 37
            EEPROM.update(38, unionIB.datoB[1]);        // y 38 temperatura máxima
            if (direccion == 50) EEPROM_direccion =39;
         case 39:
            EEPROM.update(39, Temp_Min);                // en posicion 39 temperatura mínima
            if (direccion == 50) EEPROM_direccion =40;
         case 40:
            unionIB.datoI = Temp_Standby;
            EEPROM.update(40, unionIB.datoB[0]);        // en posicion 40
            EEPROM.update(41, unionIB.datoB[1]);        // y 41 temperatura standby
            if (direccion == 50) EEPROM_direccion =42;
         case 42:
            EEPROM.update(42, Tiempo_stop);             // en posicion 42, tiempo en minutos de Standby para  STOP
            if (direccion == 50) EEPROM_direccion =43; 
         case 43:
            EEPROM.update(43, de_soldador);             // en posicion 43 el soldador habilitado true= de/soldador 1, false = de/soldador 2
            if (direccion == 50) EEPROM_direccion =44;
         case 44:
            EEPROM.update(44, DEBUGGER);                // en posicion 44  DEBUGGER ON/OFF     
            if (direccion == 50) EEPROM.update(50,direccion);// en la posicion 50 guardamos el valor 50
          break;
    } 
    if (DEBUGGER){
      if (direccion == 50){
        Serial.println("Programacion completa de la EEPROM"); 
      }else{
        Serial.print("Direccion ");
        Serial.print(EEPROM_direccion);
        Serial.println(" de la EEPROM programada");
      }
    }
}
//********************************************************************************************
// Leer EEPROM
//--------------------------------------------------------------------------------------------
float leer_EEPROM(int posicion_rx){
  switch (posicion_rx) {
    case 0:
      unionFB.datoF = KP__S ;
      unionFB.datoB[0] =  EEPROM.read(0);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(1);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(2);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(3);
      delay(5);
      KP_S = unionFB.datoF;
    break;
    case 4:
      unionFB.datoF = KI__S ;
      unionFB.datoB[0] =  EEPROM.read(4);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(5);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(6);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(7);
      delay(5);
      KI_S = unionFB.datoF;
    break;
    case 8:
      unionFB.datoF = KD__S ;
      unionFB.datoB[0] =  EEPROM.read(8);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(9);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(10);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(11);
      delay(5);
      KD_S = unionFB.datoF;
    break;
    case 12:
      unionFB.datoF = KP_S ;
      unionFB.datoB[0] =  EEPROM.read(12);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(13);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(14);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(15);
      delay(5);
      KP_S = unionFB.datoF;
    break;
    case 16:
      unionFB.datoF = KI_S ;
      unionFB.datoB[0] =  EEPROM.read(16);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(17);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(18);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(19);
      delay(5);
      KI_S = unionFB.datoF;
    break;
    case 20:
      unionFB.datoF = KD_S ;
      unionFB.datoB[0] =  EEPROM.read(20);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(21);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(22);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(23);
      delay(5);
      KD_S = unionFB.datoF;
    break;
    case 24:
      unionFB.datoF = KP_D ;
      unionFB.datoB[0] =  EEPROM.read(24);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(25);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(26);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(27);
      delay(5);
      KP_D = unionFB.datoF;
    break;
    case 28:
      unionFB.datoF = KI_D ;
      unionFB.datoB[0] =  EEPROM.read(28);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(29);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(30);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(31);
      delay(5);
      KI_D = unionFB.datoF;
    break;
    case 32:
      unionFB.datoF = KD_D ;
      unionFB.datoB[0] =  EEPROM.read(32);
      delay(5);
      unionFB.datoB[1] =  EEPROM.read(33);
      delay(5);
      unionFB.datoB[2] =  EEPROM.read(34);
      delay(5);
      unionFB.datoB[3] =  EEPROM.read(35);
      delay(5);
      KD_D = unionFB.datoF;
    break;
    case 36:
      MEMORIA = EEPROM.read(36);          
    break;
    case 37:
      unionIB.datoI = Temp_Max ;
      unionIB.datoB[0] =  EEPROM.read(37);
      delay(5);
      unionIB.datoB[1] =  EEPROM.read(38);
      delay(5);
      Temp_Max = unionIB.datoI;
    break;
    case 39:
      Temp_Min = EEPROM.read(39);
      delay(5);
    break;
    case 40:
      unionIB.datoI = Temp_Standby;
      unionIB.datoB[0] =  EEPROM.read(40);
      delay(5);
      unionIB.datoB[1] =  EEPROM.read(41);
      delay(5);
      Temp_Standby = unionIB.datoI;
    break;
    case 42:
      Tiempo_stop = EEPROM.read(42);
      delay(5);
    break;
    case 43:
      //de_soldador = EEPROM.read(43);
      delay(5);
    break;
    case 44:
      DEBUGGER = EEPROM.read(44);
      delay(5);
    break;
    case 50:
      OK_EEPROM = EEPROM.read(50);
      delay(5);;
    break;
  }
}
//*************************************************************************************
// Gestión RGB color
//-------------------------------------------------------------------------------------
uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
//******************************************************************************************
// Gestión zumbador
//------------------------------------------------------------------------------------------
void zumbador_ON(byte duracion) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_V, LOW); 
    digitalWrite(ZUMBADOR, HIGH);
    delay(duracion);
    digitalWrite(ZUMBADOR, LOW);
}
//********************************************************************************************
// calculo polinomio
//--------------------------------------------------------------------------------------------
float PolyEval(float x, float *coeficiente, unsigned int longitud) {
  float pol = 0.0;
  for (int i = longitud; i >= 0; i--){
    pol = pol * x + coeficiente[i];
  }
  return pol;
}
//********************************************************************************************
// Leemos temperatura ambiente y soldador 
//--------------------------------------------------------------------------------------------
float getTemperatura(boolean debug, int pwm_run){
float temp_ambiente =0, offset =0, temp=0, miliV =0, temperatura_punta_absoluta =0; 
long adc_temperatura_punta_relativa;
int espera =400;
  analogWrite(PWMpin, 0);                                                 // PWM OFF
  digitalWrite(LED_ADC, ON);                                              // inicio lectura
  if (debug == false)
	  espera =50;                                                           // autoPID ON
  do{											                                                // esperamos a NO tener consumo en soldador
	  delay(1); 
	  if(--espera ==0){
	    if (debug){
        Serial.println("ERROR: consumo en lectura");
      }	
      digitalWrite(LED_ADC, OFF);                                         // fin lectura  
	    return (Temp_Max+100);
    }
  }while (analogRead(ADC_I)  > 513);                                      // 512 =0A                                                                
  delay(5);	    
   miliV =(analogRead(ADC_T_OFFSET) * mV_ADC);                            // leemos el sensor de temperatura y obtenemos la tensión en mV
   miliV = 600;
   if (miliV >500){
    temp_ambiente = ((miliV-500) /10);                                   // pasamos los mV a grados, 500mv =0°C , +1 °C = + 10mv
    temp_ambiente= 13.443;
// aplicamos el polinomio de temperatura a voltaje
    for (byte x =longitud_TempC_Emf; x >0; x--){                          // recorremos el array de la posición 10 a la 1
      offset = (offset + TempC_Emf[x]) * temp_ambiente;
    }
    offset = TempC_Emf[0] + offset;                                       // tensión equivalente al termopar relativa a la temperatura del sensor TC1047
    if (debug){ 
      Serial.print("Temperatura ambiente: "); 
      Serial.println((float)temp_ambiente,2);
    }
   }else{
    if (debug){
      Serial.println("ERROR, temperatura demasiado baja");  
    }
    digitalWrite(LED_ADC, OFF);                                           // fin lectura
    return (Temp_Max+100);
   } 
   //adc_temperatura_punta_relativa =analogRead(ADC_T);                    // obtenemos el valor analógico 10 bits
   adc_temperatura_punta_relativa = adc.newAnalogRead(ADC_T);              // ADC 12 bit con sobremuestreo
   if (debug){
    Serial.print("Calculo para"); 
   }
   if (TIPO == T12){
    temperatura_punta_absoluta =((-0.0162 + sqrt(0.00026244 + -4 * 1e-05 *(-(0.6534 + ((float)adc_temperatura_punta_relativa * (mV_ADC_12b - ERROR_OFFSET)  / GANANCIA_AMP))))) /(2 * 1e-05)) + temp_ambiente;
    if (debug){
     Serial.println(" -T12");
    }
   }else{                                                                   // usamos termopar conocido
    temperatura_punta_absoluta = 1000 * PolyEval((float)adc_temperatura_punta_relativa * mV_ADC_12b  / GANANCIA_AMP + PolyEval((float)temp_ambiente, TempC_Emf, longitud_TempC_Emf), Emf_TempC, longitud_Emf_TempC);
    temperatura_punta_absoluta =(temperatura_punta_absoluta /1000);         // pasamos a grados
   }
   if (TIPO == JBC){                                                        // corrección 
     temperatura_punta_absoluta =temperatura_punta_absoluta * 0.872;
   }
   if (debug){ 
    Serial.print("Temperatura soldador: "); 
    Serial.print ((float)temperatura_punta_absoluta,2); 
    Serial.println(" grados");
   }
   if(temperatura_punta_absoluta >Temp_Max +50){
    if (debug){
      Serial.println("ERROR, fuera de rango!");  
    }
    digitalWrite(LED_ADC, OFF);                                               // fin lectura
    return (Temp_Max+100);
   }   
   digitalWrite(LED_ADC, OFF);                                                // fin lectura
   analogWrite(PWMpin, pwm_run);                                              // restauramos de nuevo el PWM
   return (temperatura_punta_absoluta); 
}
//*************************************************************
// inicia pantalla                           
//*************************************************************
void inicia_tft(){
  inicio =false;
  tft.fillScreen(ST7735_BLACK);                                               // borrar, rellenar de negro
  tft.drawBitmap(2,1,stationLOGO1,124,47,ST7735_GREY);                        // imprimimos logo estación
  tft.drawBitmap(3,3,stationLOGO1,124,47,ST7735_YELLOW);    
  tft.drawBitmap(3,3,stationLOGO2,124,47,Color565(254,147,52)); 
  tft.drawBitmap(3,3,stationLOGO3,124,47,Color565(255,78,0)); 
  if(INTRO){
    delay(500); 
    tft.drawBitmap(15,50,iron,100,106,ST7735_GREY);                           // imprimimos en display el logo del soldador
    tft.drawBitmap(17,52,iron,100,106,ST7735_YELLOW);
    delay(500);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_GREY);
    tft.setCursor(70,130);
    tft.print(VERSION);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(72,132);
    tft.print(VERSION);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_GREY);
    tft.setCursor(103,0);
    tft.print("v");
    tft.print(VERSION);
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(104,1);
    tft.print("v");
    tft.print(VERSION);
    delay(2500);
  }
   tft.fillRect(0,47,128,125,ST7735_BLACK);
   tft.setTextColor(ST7735_WHITE);
   tft.setTextSize(1);
   tft.setCursor(1,84);
   tft.print("actual"); 
   tft.setCursor(1,129);
   tft.print("selec.");
   tft.setTextSize(2);
   tft.setCursor(80,144);
   tft.print("   %");
   tft.setTextSize(1);
   tft.setCursor(1,151);                                                  
   tft.print("pwm");
   tft.setTextSize(2);
}
//******************************************************************************************
// Escritura de texto en display
//------------------------------------------------------------------------------------------
void testdrawtext(char *text, uint16_t color){
   tft.setCursor(0, 0);
   tft.setTextColor(color);
   tft.setTextWrap(true);                                            // Si el texto no cabe lo pasamos a la siguiente linea
   tft.print(text);
}
//*******************************************************************************************
// Escritura de la temperatura seleccionada, actual,  y valor PWM en pantalla  
//-------------------------------------------------------------------------------------------
void writeHEATING(int tempSOLL, int tempVAL, int pwmVAL){
  //static int d_tempSOLL = 2;                                       // filtro
  static int tempSOLL_OLD = 10;
  static int tempVAL_OLD  = 10;
  static int pwmVAL_OLD = 10;
  
  pwmVAL = map(pwmVAL, 0, 250, 0, 100); 
  tft.setTextSize(5);
// temperatura real
  if (tempVAL_OLD != tempVAL){
    tft.setCursor(30,57);
    tft.setTextColor(ST7735_BLACK);
    //tft.print(tempSOLL_OLD);   
// el primer dígito es distinto
    if ((tempVAL_OLD/100) != (tempVAL/100)){ tft.print(tempVAL_OLD/100);}else tft.print(" ");   
    if (((tempVAL_OLD/10)%10) != ((tempVAL/10)%10) ){ tft.print((tempVAL_OLD/10)%10 );} else tft.print(" ");   
    if ( (tempVAL_OLD%10) != (tempVAL%10) ) tft.print(tempVAL_OLD%10 );  
    tft.setCursor(30,57);
    if (tempVAL >= Temp_Max+100 || tempVAL < 0){
      tft.print("   ");
      tft.setTextColor(ST7735_RED);
      tft.setCursor(30,57);
      tft.print("ERR");                                             // error en lectura
      analogWrite(PWMpin, 0);
      pwmVAL =0;
    }else{
      tft.setTextColor(ST7735_WHITE);   
      if (tempVAL < 100) tft.print(" ");
      if (tempVAL <10) tft.print(" ");
      int tempDIV = round(float(tempSOLL - tempVAL)*8.5);           // calculamos el color según diferencia temperatura con setpoint
      tempDIV = tempDIV > 254 ? tempDIV = 254 : tempDIV < 0 ? tempDIV = 0 : tempDIV;
      tft.setTextColor(Color565(tempDIV, 255-tempDIV, 0));
      if (standby) tft.setTextColor(ST7735_CYAN);                   // color en modo standby
      tft.print(tempVAL);
      tempVAL_OLD = tempVAL; 
    }    
  }
// temperatura seleccionada SETPOINT
   if (tempSOLL_OLD != tempSOLL){
    tft.setCursor(30,102);
    tft.setTextColor(ST7735_BLACK); 
// el primer dígito es distinto
    if ((tempSOLL_OLD/100) != (tempSOLL/100)){ tft.print(tempSOLL_OLD/100);} else tft.print(" ");   
    if ( ((tempSOLL_OLD/10)%10) != ((tempSOLL/10)%10) ){ tft.print((tempSOLL_OLD/10)%10 );} else tft.print(" ");   
    if ( (tempSOLL_OLD%10) != (tempSOLL%10) ) tft.print(tempSOLL_OLD%10 );    
// escribir nuevo valor en color blanco
    tft.setCursor(30,102);
    tft.setTextColor(ST7735_WHITE);
    if (tempSOLL < 100) tft.print(" ");
    if (tempSOLL <10) tft.print(" ");   
    tft.print(tempSOLL);
    tempSOLL_OLD = tempSOLL; 
  }
 // valor PWM  
  tft.setTextSize(2);
  if (pwmVAL_OLD != pwmVAL){                                        // si el valor de PWM es distinto al anterior actualizamos
    tft.setCursor(80,144);
    tft.setTextColor(ST7735_BLACK);  
// el primer dígito es distinto?
    if ((pwmVAL_OLD/100) != (pwmVAL/100)){ tft.print(pwmVAL_OLD/100);} else tft.print(" ");
    if ( ((pwmVAL_OLD/10)%10) != ((pwmVAL/10)%10) ){ tft.print((pwmVAL_OLD/10)%10 );} else tft.print(" ");
    if ( (pwmVAL_OLD%10) != (pwmVAL%10) ) tft.print(pwmVAL_OLD%10 );
    tft.setCursor(80,144);
    tft.setTextColor(ST7735_WHITE);
    if (pwmVAL < 100) tft.print(" ");
    if (pwmVAL <10) tft.print(" ");
    tft.print(pwmVAL);
    pwmVAL_OLD = pwmVAL;
  }
}
//*************************************************************************************
// gestión menú configuración en tft
//*************************************************************************************
void menu_linia(byte linea){
  if (linea ==0){
    tft.fillScreen(ST7735_BLACK);                         // borrar, rellenar de negro
    tft.setTextWrap(true);                                // salto de linea ON
    tft.fillRect (0, 0, 128, 8, ST7735_BLUE);             // rellena del punto x=0, y=0  y 128 de largo por 8 de ancho
    tft.setTextSize(1);                                   // tamaño del texto a 1, cada 21 caracteres saltamos de linea
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(0, 0);
    tft.print(" Menu  configuracion");
    tft.setCursor(0, 145);
    tft.print("Tension: ");
    tft.print((int)(analogRead(ADC_V) * 0.0309));
    tft.print("V");
    if (TIPO == RT1) tft.print("-- RT1"); else TIPO == JBC  ? tft.print("-- JBC"): tft.print("-- T12");
  }else{
    tft.fillRect (0, 145, 128, 10, ST7735_BLACK);         // borramos linea tensión, 128 de largo por 10 de ancho
    tft.fillRect (0, 9, 10, 151, ST7735_BLACK);           // borramos columna, 10 de largo por 151 de ancho
  }
  tft.setCursor(0, 20);
  if (linea ==1){
    tft.setTextColor(ST7735_BLUE); 
    tft.print("->"); 
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-");
  }
  tft.setCursor(11, 20);
  tft.print("Zumbador ON/OFF");
  tft.setCursor(0, 40);
  if (linea ==2){
    tft.setTextColor(ST7735_BLUE); 
    tft.print("->"); 
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-");
  }
  tft.setCursor(11, 40);
  tft.print("DEBUGGER ON/OFF");
  tft.setCursor(0, 60);
  if (linea ==3){
    tft.setTextColor(ST7735_BLUE); 
    tft.print("->");
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-");
  }
  tft.setCursor(11, 60);
  tft.print("Auto ajuste PID");
  tft.setCursor(0, 80);
  if (linea ==4){
    tft.setTextColor(ST7735_BLUE); 
    tft.print("->");
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-");
  }
  tft.setCursor(11, 80);
  tft.print("Temp. maxima");
  tft.setCursor(0, 100);
  if (linea ==5){
    tft.setTextColor(ST7735_BLUE); 
    tft.print("->");
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-");
  }
  tft.setCursor(11, 100);
  tft.print("Temp. minima");
  tft.setCursor(0, 120);
  if (linea ==6){
    tft.setTextColor(ST7735_BLUE);
    tft.print("->");  
  }else{
    tft.setTextColor(ST7735_WHITE);
    tft.print("-"); 
  }
  tft.setCursor(11, 120);
  tft.print("Tiempo Standby"); 
}
//*************************************************************************************
// Gestión menu                                                                           
void menu_config(){
long pulsos_encoder =9;
byte espera =0;
boolean actualiza =true;
  if (ON_OFF_zumbador) zumbador_ON (50);
  menu =false;
  out_pwm =0;
  analogWrite(PWMpin, out_pwm);
  posicion_encoder = pulsos_encoder;                              // posicion inicial del encoder    
    do{                                                          
      espera++;
      delay(100);
      if (espera ==30){                                           // comprobamos si tenemos 30 ciclos de 100ms pulsando  
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 80);
        tft.setTextSize(1);
        tft.setTextColor(ST7735_RED);
        tft.setTextSize(1);
        tft.print("STOP estacion pulsar para Re-iniciar");
        if (DEBUGGER){
          Serial.println(" Estacion en STOP"); 
        }                    
        if (ON_OFF_zumbador) zumbador_ON (50);
        while(!digitalRead(PULSADOR)){}                           // esperamos que se libere el pulsador
        do{
        delay(100);
        }while(digitalRead(PULSADOR));                            // esperamos que se pulsa para iniciar
        soft_reset();
      }
    }while (!digitalRead(PULSADOR));                              // si se esta manteniendo la pulsacion
    espera =0;
//------------------------------MENU INICIAL-------------------------------------      
    if (ON_OFF_zumbador) zumbador_ON (50); 
    do{
      switch (pulsos_encoder){
        case 9:
          if (actualiza){ 
            actualiza =false;
            if (DEBUGGER){
              Serial.println("Menu configuracion");
            }
            menu_linia(0);
          }
          break;
//------------------------MENU ZUMBADOR ON/OFF-----------------------------------------
        case 10:
          if (actualiza){ 
            actualiza =false;
            menu_linia(1);
            if (DEBUGGER){
              Serial.println("1-Zumbador ON/OFF");
              Serial.println("Pulsar para cambiar ");
            }
          }
          do{
            delay(100);
            if (pulsos_encoder != posicion_encoder){                        // si tenemos pulsos del encoder
            if (posicion_encoder < 10)  posicion_encoder = 15;
            espera =100; 
            }
            if (menu ==true){                                               // si pulsamos encoder
              while(!digitalRead(PULSADOR)){}                               // esperamos que se libere el pulsador
              delay(100);
              menu =false;
              tft.fillScreen(ST7735_BLACK);                                 // borrar, rellenar de negro
              tft.setCursor(0, 80);
              tft.setTextSize(1);
              tft.setTextColor(ST7735_RED);
              if (ON_OFF_zumbador == ON){
                ON_OFF_zumbador = OFF; 
                tft.print("Zumbador OFF");
                if (DEBUGGER){
                  Serial.println("Zumbador en OFF");
                }
              }else{
                ON_OFF_zumbador = ON;
                tft.print("Zumbador ON ");
                if (DEBUGGER){
                  Serial.println("Zumbador en ON");
                }
                zumbador_ON (50);
                delay(3000);
              }
              posicion_encoder = 9;                                         // acceso al menu inicial
              espera =100;
            }
          }while (++espera < 50); 
          if (espera !=50) espera =0;                                       // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder;
          break;
//---------------------MENU DEBUGGER ON/OFF-------------------------------    
        case 11:
          if (actualiza){ 
            actualiza =false;          
            menu_linia(2); 
            if (DEBUGGER){
              Serial.println("2-Menu DEBUGGER ON/OFF ");
              Serial.println("Pulsar para cambiar");
            }  
          }
          do{
            delay(100);
            if (pulsos_encoder != posicion_encoder) espera =100;          // si tenemos pulsos del encoder          
            if (menu ==true){                                             // si pulsamos encoder
              tft.fillScreen(ST7735_BLACK);                               // borrar, rellenar de negro
              tft.setCursor(5, 80);
              tft.setTextSize(1);
              tft.setTextColor(ST7735_RED);
              while(!digitalRead(PULSADOR)){}                             // esperamos que se libere el pulsador
              delay(100);
              menu =false;
              if (DEBUGGER == ON){
                tft.print("DEBUGGER en OFF");
                DEBUGGER = OFF; 
                Serial.println("DEBUGGER en OFF");
              }else{
                DEBUGGER = ON;
                tft.print("DEBUGGER en ON");
                if (DEBUGGER){
                  Serial.println("DEBUGGER en ON");
                }
              }
              program_EEPROM(44);
              if (ON_OFF_zumbador) zumbador_ON (50);
              delay(3000);
              soft_reset();
            }
          }while (++espera < 50); 
          if (espera !=50) espera =0;                                     // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder;
          break;
//-----------------------MENU CALCULO PID DE-SOLDADOR-----------------------------------
        case 12:
          if (actualiza){ 
            actualiza =false;          
            menu_linia(3);
            if (DEBUGGER){
              Serial.println("3-Menu Auto ajuste PID ");
              Serial.println("Pulsar para iniciar..");
            }
          }
          do{
           delay(100);
           if (pulsos_encoder != posicion_encoder)  espera =100;          // si tenemos pulsos del encoder
             if (menu ==true){                                            // si pulsamos encoder
                tft.fillScreen(ST7735_BLACK);                             // borrar, rellenar de negro
                tft.setCursor(0,80);
                tft.setTextSize(1);
                tft.setTextColor(ST7735_RED);
              while(!digitalRead(PULSADOR)){}                             // esperamos que se libere el pulsador
              delay(100);
              menu =false;
             if (tension_12_24 == V_12){
              tft.print("PID soldador 12V..");          
              if (PID_autotune(150, 1)){                                  // calculo valores PID para soldador de 12V
                if (DEBUGGER){
                  Serial.print("PID soldador 12v ajustado a Kp, Ki, Kd: ");
                  Serial.print(KP__S);
                  Serial.print(", ");
                  Serial.print(KI__S);
                  Serial.print(", ");
                  Serial.println(KD__S);
                }
                  program_EEPROM(0);
                  program_EEPROM(4);
                  program_EEPROM(8);
                  tft.fillRect (0, 80, 128, 88, ST7735_BLACK);
                  tft.print("PID soldador 12V OK");
                  delay(3000);
                  soft_reset();
              }else{
                tft.fillScreen(ST7735_RED);
                tft.setTextColor(ST7735_BLACK);
                tft.print("Error: PID soldador 12V..");
                if (DEBUGGER){
                  Serial.println("ERROR: fallo auto PID soldador 12V");
                }
                if (ON_OFF_zumbador) zumbador_ON (250);
                delay(3000);
                soft_reset();
              }                
            }else{
              if (de_soldador == soldador){
                tft.print("PID soldador 24V..");
                if (PID_autotune(150, 2)){                                  // calculo valores PID soldador 24V
                  if (DEBUGGER){
                    Serial.print("PID soldador 24v ajustado a Kp, Ki, Kd: ");
                    Serial.print(KP_S);
                    Serial.print(", ");
                    Serial.print(KI_S);
                    Serial.print(", ");
                    Serial.println(KD_S);
                  }
                  program_EEPROM(12);
                  program_EEPROM(16);
                  program_EEPROM(20);
                  tft.fillRect (0, 80, 128, 88, ST7735_BLACK);
                  tft.print("PID soldador 24V OK");
                  delay(3000);
                  soft_reset();
                }else{
                  tft.fillScreen(ST7735_RED);
                  tft.setTextColor(ST7735_BLACK);
                  tft.print("Error: PID soldador 24V..");
                  if (DEBUGGER){
                    Serial.println("ERROR: fallo auto PID soldador 24V");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (250);
                  delay(3000);
                  soft_reset();
                }
              }else{
                tft.print("PID desoldador 24V..");
                if (PID_autotune(150, 3)){                                  // calculo valores PID desoldador 24V
                  if (DEBUGGER){
                    Serial.print("PID desoldador 24V ajustado a Kp, Ki, Kd: ");
                    Serial.print(KP_D);
                    Serial.print(", ");
                    Serial.print(KI_D);
                    Serial.print(", ");
                    Serial.println(KD_D);
                  }
                  program_EEPROM(24);
                  program_EEPROM(28);
                  program_EEPROM(32);  
                  tft.fillRect (0, 80, 128, 88, ST7735_BLACK);
                  tft.print("PID desoldador 24V OK");
                  if (ON_OFF_zumbador) zumbador_ON (50);
                  delay(3000);
                  soft_reset();   
                }else{
                  tft.fillScreen(ST7735_RED);
                  tft.setTextColor(ST7735_BLACK);
                  tft.print("Error: PID desoldador 24V..");
                  if (DEBUGGER){
                    Serial.println("ERROR: fallo auto PID desoldador");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (250);
                  delay(3000);
                  soft_reset();
                }   
               }
             }
           }    
          }while (++espera < 50);
          if (espera !=50) espera =0;                                     // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder;
        break;
//--------------------------------MENU TEMPERATURA MÁXIMA------------------------------
        case 13: 
          if (actualiza){ 
            actualiza =false;
            menu_linia(4);
            if (DEBUGGER){
              Serial.println("4-Menu Temp. maxima");
              Serial.println("Pulsar para cambiar");
            }
          }
          do{
           delay(100);
           if (pulsos_encoder != posicion_encoder) espera =100;           // si tenemos pulsos del encoder           
           if (menu ==true){
            while(!digitalRead(PULSADOR)){}                               // esperamos que se libere el pulsador
            delay(100);
            menu =false;
            posicion_encoder = Temp_Max;                                  // posicion inicial = a la temperatura mÁxima actual
            pulsos_encoder = Temp_Max;
            tft.fillScreen(ST7735_BLACK);                                 // borrar, rellenar de negro
            tft.setTextColor(ST7735_RED);
            tft.setTextSize(1);
            tft.setCursor(0,40);
            tft.print("grados centigrados");
            tft.setTextSize(2);
            tft.setCursor(5,80);
            tft.print(Temp_Max);
            tft.setTextSize(1);
            espera =0;
            do{
              delay(100);           
              if (posicion_encoder != pulsos_encoder){
                espera =0;
                pulsos_encoder = posicion_encoder;
                Temp_Max = pulsos_encoder;
                tft.fillRect (0, 80, 128, 88, ST7735_BLACK);
                tft.setTextSize(2);
                tft.setCursor(5,80);
                tft.print(Temp_Max);
                tft.setTextSize(1);
                if (DEBUGGER){
                  Serial.println(Temp_Max);
                }
                if (Temp_Max > 480){
                  Temp_Max = 480;                                        // temperatura máxima 480
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.fillScreen(ST7735_BLACK);                          // borrar, rellenar de negro
                  tft.setCursor(0,80);
                  tft.setTextSize(1);
                  tft.setTextColor(ST7735_RED);
                  tft.print("Maximo alcanzado!");
                  if (DEBUGGER){
                    Serial.println("Maximo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
                if (Temp_Max < (Temp_Min+20)){
                  Temp_Max = (Temp_Min+20);                             // temperatura máxima como mínimo  20 mas que la mínima 
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.setCursor(0,80);
                  tft.setTextSize(1);
                  tft.print("Minimo alcanzado!");
                  if (DEBUGGER){
                    Serial.println("Minimo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
              }             
              if (menu ==true){                                         // si pulsamos salimos y programamos
                while(!digitalRead(PULSADOR)){}                         // esperamos que se libere el pulsador
                delay(100);
                menu =false;
                tft.fillRect (0, 80, 128, 96, ST7735_BLACK);
                tft.setCursor(0,80);
                tft.setTextSize(1);
                tft.print("Maximo a:");
                tft.print(Temp_Max);
                if (DEBUGGER){
                  Serial.print("Temp. maxima a:");
                  Serial.println(Temp_Max);
                }
                posicion_encoder = 9;
                program_EEPROM(37);
                if (ON_OFF_zumbador) zumbador_ON (50);
                delay(3000);
                espera =100;         
              }
              }while (espera < 30);
           }   
          }while (++espera < 50); 
          if (espera !=50) espera =0;                                   // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder;
        break;
//-------------------------MENU TEMPERATURA MÍNIMA--------------------------------------------
        case 14:
          if (actualiza){ 
            actualiza =false;
            menu_linia(5); 
            if (DEBUGGER){
              Serial.println("5-Menu Temp. minima");
              Serial.println("Pulsar para cambiar");
            }
          }
          do{
           delay(100);
           if (pulsos_encoder != posicion_encoder) espera =100;          
           if (menu ==true){
            while(!digitalRead(PULSADOR)){}                         // esperamos que se libere el pulsador
            delay(100);
            menu =false;
            posicion_encoder =Temp_Min;
            pulsos_encoder =Temp_Min;
            tft.fillScreen(ST7735_BLACK);                           // borrar, rellenar de negro
            tft.setTextColor(ST7735_RED);
            tft.setTextSize(1);
            tft.setCursor(0,40);
            tft.print("grados centigrados");
            tft.setTextSize(2);
            tft.setCursor(5,80);
            tft.setTextSize(1);
            tft.print(Temp_Min);
            if (DEBUGGER){
              Serial.println("grados centigrados");
            }
            espera =0;
            do{
              delay(100);
              if (posicion_encoder != pulsos_encoder){
                espera =0;
                pulsos_encoder = posicion_encoder;
                Temp_Min = pulsos_encoder;
                tft.fillRect (0, 80, 128, 88, ST7735_BLACK);
                tft.setTextSize(2);
                tft.setCursor(5,80);
                tft.print(Temp_Min);
                tft.setTextSize(1);
                if (DEBUGGER){
                  Serial.println(Temp_Min);
                }
                if (Temp_Min > (Temp_Max-20)){
                  Temp_Min = (Temp_Max-20);
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.setTextSize(1);
                  tft.setCursor(0,80);
                  tft.print("Maximo alcanzado!");
                  tft.print(Temp_Min);
                  if (DEBUGGER){
                    Serial.println("Maximo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
                if (Temp_Min < 100){
                  Temp_Min = 100;
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.setTextSize(1);
                  tft.setCursor(0,80);
                  tft.print("Minimo alcanzado! ");
                  tft.print(Temp_Min);
                  if (DEBUGGER){
                    Serial.println("Minimo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
              }            
              if (menu ==true){                                 // si pulsamos salimos y programamos
                while(!digitalRead(PULSADOR)){}                 //esperamos que se libere el pulsador
                delay(100);
                menu =false;
                tft.fillRect (0, 80, 128, 96, ST7735_BLACK);
                tft.setTextSize(1);
                tft.setCursor(0,80);
                tft.print("Temp. minimo a:");
                tft.print(Temp_Min);
                if (DEBUGGER){
                  Serial.print("Temp. minimo a:");
                  Serial.println(Temp_Min);
                }
                posicion_encoder = 9;
                pulsos_encoder = posicion_encoder;
                program_EEPROM(39);
                if (ON_OFF_zumbador) zumbador_ON (50);
                delay(3000);
                espera =100;                
              }
              }while (espera < 30);
           }   
          }while (++espera < 50); 
          if (espera !=50) espera =0;                 // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder;
        break;
 //---------------------------MENU TIEMPO MÁXIMO EN STANDBY---------------------------------------
        case 15: 
          if (actualiza){ 
            actualiza =false;
            menu_linia(6);
            if (DEBUGGER){
              Serial.println("6-Menu Tiempo Standby");
              Serial.println("Pulsar para cambiar");
            }
          }
          do{
           delay(100);
           if (pulsos_encoder != posicion_encoder){
            if (posicion_encoder > 15)  posicion_encoder = 10;
            espera =100;
           }
           if (menu ==true){
            while(!digitalRead(PULSADOR)){}         //esperamos que se libere el pulsador
            delay(100);
            menu =false;
            posicion_encoder =Tiempo_stop;
            pulsos_encoder =Tiempo_stop;
            tft.fillScreen(ST7735_BLACK);                    // borrar, rellenar de negro          
            tft.setTextColor(ST7735_RED);
            tft.setCursor(0,40);
            tft.setTextSize(1);
            tft.print("minutos de 1 a 255");
            tft.setCursor(5,80);
            tft.setTextSize(2);
            tft.print(Tiempo_stop);
            tft.setTextSize(1);
            if (DEBUGGER){
              Serial.println("minutos de 1 a 255");
            }
            espera =0;
            do{
              delay(100);
              if (posicion_encoder != pulsos_encoder){
                espera =0;
                pulsos_encoder = posicion_encoder;
                Tiempo_stop = pulsos_encoder;
                tft.fillRect (0, 80, 128, 96, ST7735_BLACK);
                tft.setCursor(5,80);
                tft.setTextSize(2);
                tft.print(Tiempo_stop);
                tft.setTextSize(1);
                if (DEBUGGER){
                  Serial.println(Tiempo_stop);
                }
                if (Tiempo_stop > 255){
                  Tiempo_stop = 255;                        // tiempo máximo, 255 minutos
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.setCursor(0,80);
                  tft.setTextSize(1);
                  tft.print("Maximo alcanzado!");
                  if (DEBUGGER){
                    Serial.println("Maximo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
                if (Tiempo_stop < 1){
                  Tiempo_stop = 1;                          // tiempo mínimo, 1 minuto
                  tft.fillRect (5, 80, 128, 96, ST7735_BLACK);
                  tft.setCursor(0,80);
                  tft.setTextSize(1);
                  tft.print("Minimo alcanzado!");
                  if (DEBUGGER){
                    Serial.println("Minimo alcanzado!");
                  }
                  if (ON_OFF_zumbador) zumbador_ON (50);
                }
              }           
              if (menu ==true){                             // si pulsamos salimos y programamos en la EEPROM
                while(!digitalRead(PULSADOR)){}             // esperamos que se libere el pulsador
                delay(100);
                menu =false;
                tft.fillRect (0, 80, 128, 96, ST7735_BLACK);
                tft.setCursor(0,80);
                tft.setTextSize(1);
                tft.print("STOP a:");
                tft.print(Tiempo_stop);
                tft.print(" minutos");
                if (DEBUGGER){
                  Serial.print("Ajustado a:");
                  Serial.print(Tiempo_stop);
                  Serial.println(" minutos");
                }
                program_EEPROM(40);
                if (ON_OFF_zumbador) zumbador_ON (50);
                delay(3000);
                espera =100;
                posicion_encoder = 9;
              }
            }while (++espera < 30);
           }   
          }while (++espera < 50); 
          if (espera !=50) espera =0;                       // solo ponemos a cero si NO es fin de tiempo inactivo
          actualiza =true;
          pulsos_encoder = posicion_encoder; 
        break;
      }
//------------------------------ESPERA SALIDA MENU----------------------------------------------------------
      delay(100);            
      if (pulsos_encoder != posicion_encoder){              // si tenemos pulsos del encoder
        espera =0;
        actualiza =true;
        if (posicion_encoder > 15)  posicion_encoder = 1;
        if (posicion_encoder < 10)  posicion_encoder = 15;
        pulsos_encoder = posicion_encoder;
      }else{
        espera++;                                          // mientras se espera sin rotación del encoder
      }
    }while (espera < 50 );
    posicion_encoder =0;
    delay(500); 
    inicio =false;
    menu =false;
}
//***********************************************************************************************
// Configuración
//***********************************************************************************************
void setup(void){
int contador;
byte ciclo =0, I_Soldador;
  inicio = false;
  pinMode(BLpin, OUTPUT);                             // pin BL como salida
  digitalWrite(BLpin, LOW);                           // pin BL a 0
  pinMode(LED_ADC, OUTPUT);                           // pin CONTROL como salida
  digitalWrite(LED_ADC, OFF);                         // lectura de temperatura OFF                    
  pinMode(LED_R, OUTPUT);                             // pin del led rojo como salida 
  digitalWrite(LED_R, HIGH);                          // rojo ON
  pinMode(LED_V, OUTPUT);                             // pin del led verde como salida 
  digitalWrite(LED_V, HIGH);                          // verde ON (rojo+verde=naranja)
  pinMode(ZUMBADOR, OUTPUT);
  pinMode(BOMBA_PUL, INPUT_PULLUP);                   // configura al pin2 como pin de entrada y habilita la resistencia interna en pull-up

  ADC_prescaler_t ADCSpeed = ADC_FAST;
  adc.setADCSpeed(ADCSpeed);                          // sobremuestreso ADC a alta velocidad
  adc.setBitsOfResolution(bitsOfResolution);          // resolución en la lectura
  adc.setNumSamplesToAvg(numSamplesToAvg);            // múmero de muestras  con la resolución del sobremuestreo

  TCCR2B = (TCCR2B & 0b11111000) | PWM_DIV;           // config. frecuencia PWM

  tft.initR(INITR_BLACKTAB);                          // inicializamos pantalla
  SPI.setClockDivider(SPI_CLOCK_DIV4);                // SPI ON, 4 MHz (half speed), MSBFIRST, SPI_MODE0
  tft.setRotation(0);                                 // 0 - vertical, 1 - horizontal
  tft.setTextWrap(true);                                    // si el texto no cabe lo pasamos a la siguiente linea
  tft.fillScreen(ST7735_BLACK); 
  
  delay(50);
  digitalWrite(BLpin, HIGH);                          // retroalimentación  ON
  DEBUGGER =EEPROM.read(44);
  if (DEBUGGER){
    Serial.begin(115200);
    Serial.println("Modo Debugger habilitado");
  }
  delay(50);
// gestionamos tensión 12 o 24V
  tension = analogRead(ADC_V);                       // despreciamos la primera lectura
  delay(150);
  tension = (int)(analogRead(ADC_V) *0.0309);
  if (DEBUGGER){                                    
      Serial.print("Tension de alimentacion: ");
      Serial.print(tension);
      Serial.println("V");
    }
  (tension <16) ? tension_12_24 = V_12: tension_12_24 = V_24;    // obtenemos tensión de soldador                              
// comprobamos que tenemos un soldador conectado y el tipo 
  I_Soldador =0;
  analogWrite(PWMpin, 255);
  delay(10);
  intensidad = (((analogRead(ADC_I) * mV_ADC) - ACSoffset) / mV_Amp);
  analogWrite(PWMpin, 0);
  if (DEBUGGER){
    Serial.print("Consumo del soldador de : ");
    Serial.print(intensidad);
    Serial.println("A");
  }
  if (intensidad >I_T12){I_Soldador ++;}                                    // T12 2.27A
  if (intensidad >I_JBC){I_Soldador ++;}                                    // JBC 2.96A
  if (intensidad >I_RT1){I_Soldador ++;}                                    // RT1 3.89A
  while(I_Soldador ==0){                                                    // si no tenemos consumo                   
    if (ciclo ==0){
      testdrawtext ("!ERROR! De/Solsador NO conectado, apagar y conectar", ST7735_WHITE);
      digitalWrite(LED_V, LOW);
      if (DEBUGGER){
        Serial.println("ERROR! DE/Soldador NO conectado");
        Serial.println("apagar y conectar");
      }
    }
    if (++ciclo ==255){
      digitalWrite(ZUMBADOR, HIGH);
      delay(500);
      digitalWrite(ZUMBADOR, LOW);
      ciclo =1;                      
    }
    digitalWrite(LED_V, LOW); 
    delay(100);      
    (digitalRead(LED_R) == true) ? digitalWrite(LED_R, LOW): digitalWrite(LED_R, HIGH); // parpadeo rojo
  }
  de_soldador = soldador;
  if (tension_12_24 == V_12){
    TIPO = RT1;
    if (DEBUGGER){
        Serial.println("Detectado soldador WELLER RT1");
      }
  }
  else{
    if (digitalRead(BOMBA_PUL) == false){                                    // si se encuentra el pulsador de vacio pulsado
      TIPO = JBC;
      de_soldador = desoldador;
      pwm_max = 255;
      if (DEBUGGER){
        Serial.println("Detectado desoldador JBC");
      }
    }else{
      TIPO = T12;
      if (DEBUGGER){
        Serial.println("Detectado soldador HAKOO T12");
      }
    }
    //if (I_Soldador ==2){
    //  TIPO = JBC;
    //  de_soldador = desoldador;
    //  if (DEBUGGER){
    //    Serial.println("Detectado desoldador JBC");
    //  }
    //}
    //if (I_Soldador ==1){
    //  TIPO = T12;
    //  if (DEBUGGER){
    //    Serial.println("Detectado soldador HAKOO T12");
    //  }
    //}
  }
//********************************************************************************************
// escribimos y leemos EEPROM 
// leemos la posicion 50 de la EEPROM para conocer si tenemos datos
  if (EEPROM.read(50) != OK_EEPROM){                  // datos OK->50, si NO tenemos datos validos en la EEPROM
    if (DEBUGGER){
      Serial.println("Guardando configuracion en EEPROM");
    }
    program_EEPROM(50);                               // guardamos toda la configuración de defecto en EEPROM 
  }
//******************************************************************************************
// leer toda la configuración de la EEPROM 
  for (contador =0; contador < OK_EEPROM; contador++){
    leer_EEPROM(contador);                            // leemos toda la configuración de la EEPROM
  }
  if (DEBUGGER){
    Serial.println("Configuracion leida de la EEPROM:");
    Serial.print ("KP S 24V: ");
    Serial.print((float)(KP_S),3);
    Serial.print (", KI S 24V: ");
    Serial.print((float)(KI_S),3);
    Serial.print (", KD S 24V: ");
    Serial.println((float)(KD_S),3);
    
    Serial.print ("KP S 12V: ");
    Serial.print((float)(KP__S),3);
    Serial.print (", KI S 12V: ");
    Serial.print((float)(KI__S),3);
    Serial.print (", KD S 12V: ");
    Serial.println((float)(KD__S),3);
    
    Serial.print ("KP D 24V: ");
    Serial.print((float)(KP_D),3);
    Serial.print (", KI D 24V: ");
    Serial.print((float)(KI_D),3);
    Serial.print (" ,KD D 24V: ");
    Serial.println((float)(KD_D),3);
    
    Serial.print ("Tipo: ");
    switch (TIPO)  {
      case 1:
        Serial.println("JBC");
      break;
      case 2:
        Serial.println("RT1");
      break;
      case 3:
        Serial.println("T12");
      break;
    }
    Serial.print ("Temperatura maxima: ");
    Serial.print((int)(Temp_Max));
    Serial.print (", Temperatura minina: ");
    Serial.print((byte)(Temp_Min));
    Serial.print (", Temperatura standbay: ");
    Serial.println((int)(Temp_Standby));
    
    Serial.print ("Tiempo STOP: ");
    Serial.println((byte)(Tiempo_stop));   
    
    Serial.print ("Habilitado: ");
    if (tension_12_24 == V_12){                              // soldador 12v
      Serial.println("Soldador Weller 12V");       
    }else{        
      if (de_soldador == soldador) 
        Serial.println("Soldador HAKKO T12 24V");
      else
        Serial.println("Desoldador JBC 24V");  
    }     
  } 
//******************************************************************************************
    switch (TIPO) {
      case 1:                                               // tipo N
// coeficientes para termopar tipo N 0 °C a 1300 °C-> desoldador JBC
    TempC_Emf[0] = 0.000000000000E+00; // tipo N
    TempC_Emf[1] = 0.259293946010E-01;
    TempC_Emf[2] = 0.157101418800E-04;
    TempC_Emf[3] = 0.438256272370E-07;
    TempC_Emf[4] = -0.252611697940E-09;
    TempC_Emf[5] = 0.643118193390E-12;
    TempC_Emf[6] = -0.100634715190E-14;
    TempC_Emf[7] = 0.997453389920E-18;
    TempC_Emf[9] = -0.608632456070E-21;
    TempC_Emf[9] = 0.208492293390E-24;
    TempC_Emf[10] = -0.306821961510E-28;
    longitud_TempC_Emf =10;
    
    Emf_TempC[0] = 0.00000E+00;
    Emf_TempC[1] = 3.86896E+01;
    Emf_TempC[2] = -1.08267E+00;
    Emf_TempC[3] = 4.70205E-02;
    Emf_TempC[4] = -2.12169E-06; 
    Emf_TempC[5] = -1.17272E-04;
    Emf_TempC[6] = 5.39280E-06;
    Emf_TempC[7] = -7.98156E-08;
    Emf_TempC[8] = 0.00000E+00;
    Emf_TempC[9] = 0.00000E+00;
    longitud_Emf_TempC = 9;
// creamos el control PID con los punteros Input y Output, y los valores kp, ki, kd para el desoldador JBC
        myPID.SetTunings(KP_D, KI_D, KD_D);               // ajustamos valores para el desoldador  
          if (DEBUGGER){
            Serial.println("PID configurado para desoldador JBC tipo-E");
        }
      break;
      case 2:                                             // tipo D
// coeficientes para termopar tipo D -0 °C a 783 °C -> soldador Weller RT1
        TempC_Emf[0] =  0.0000000E+00;
        TempC_Emf[1] =  9.5685256E-3;
        TempC_Emf[2] =  2.0592621E-5;
        TempC_Emf[3] =  -1.8464576e-8;
        TempC_Emf[4] =  7.9498033E-12;
        TempC_Emf[5] =  -1.4240735E-15;
        longitud_TempC_Emf =5;

        Emf_TempC[0] = 7.407745239E-1;
        Emf_TempC[1] =  9.827743947E+1;
        Emf_TempC[2] = -1.223274978E+1;
        Emf_TempC[3] = 1.930483337E+0;
        Emf_TempC[4] = -1.825482304E-1;
        Emf_TempC[5] = 9.140368153E-3;
        Emf_TempC[6] = -1.851682378E-4;
        longitud_Emf_TempC =6; 

// creamos el control PID con los punteros Input y Output, y los valores kp, ki, kd para el soldador 12V
        myPID.SetTunings(KP__S, KI__S, KD__S);  
        if (DEBUGGER){
          Serial.println("PID configurado para soldador Weller RT1 tipo-D");
        }
      break;
      case 3: 
// creamos el control PID con los punteros Input y Output, y los valores kp, ki, kd para el soldador T12
        TIPO = T12;
        myPID.SetTunings(KP_S, KI_S, KD_S);                 // ajustamos valores para el soldador 
        if (DEBUGGER){
          Serial.println("PID configurado para soldador HAKKO T12");
        }
    }
//**************************************************************************************
  // inicializamos 
  tft.fillScreen(ST7735_BLACK);                             // borrar, rellenar de negro
  tft.setCursor(0, 0);
  testdrawtext ("   Inicializando...\nEstacion De-soldadora\nPantalla de 120 x 160\n   Version V0.7\n", ST7735_WHITE);
  tft.setCursor(0, 80);
  tft.setTextSize(1);
  if (tension_12_24 == V_12 &&  TIPO ==RT1){
    tft.setTextColor(ST7735_RED);
    tft.print("Soldador  RT1 12V");
  }else{
    if (TIPO ==T12){
      tft.setTextColor(ST7735_BLUE);
      tft.print("Soldador  T12 24V");
    }else{
      tft.setTextColor(ST7735_YELLOW);
      tft.print("Desoldador JBC 24V");
    }
  } 
// posición inicial encoder a 0
  posicion_encoder =0;                                      // encoder en posición 0
  pinMode(STANDBY, INPUT_PULLUP);                           // pull-up ON en el pin STANDBY_S
  pinMode(PULSADOR, INPUT_PULLUP);                          // pull-up ON en el pin PULSADOR encoder
  pinMode(PWMpin, OUTPUT);                                  // pin salida PWM del soldador como salida
  
  analogWrite(PWMpin, out_pwm);                             // PWM de-soldador a 0
  pinMode(BOMBA, OUTPUT);
  digitalWrite(BOMBA, LOW);                                 // pin del bomba desoldador como salida y a 0
 
  if (ON_OFF_zumbador) zumbador_ON (50);
  delay(3000);
  inicia_tft();
// Definimos los parámetros de control PID
  myPID.SetOutputLimits(0, pwm_max);                        // valores mínimo y máximo para el PID
  myPID.SetMode(AUTOMATIC);                                 // PID en automático
  myPID.SetSampleTime(100);                                 // ejecución del PID cada 100ms

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_V, LOW); 
  actual_temperatura = ((int)getTemperatura(DEBUGGER, 0));  // PWM  OFF
  writeHEATING(ini_temp, actual_temperatura, 0);
}
//****************************************************************************************
//Ejecución cíclica
//----------------------------------------------------------------------------------------
void loop() {
int diferencia =0, consumo =0;
  tension = (int)(analogRead(ADC_V) * 0.0309);                   // obtenemos la tension de alimentacion
  if (DEBUGGER){
    Serial.print("Tension de: ");
    Serial.print(tension);
    Serial.println("V");
  }
  if (tension_12_24 ==V_12 && de_soldador ==soldador){           // comprobamos que la tension es la correcta para el soldador de 12v    
    if (tension < 15){   
      if (DEBUGGER){
        Serial.println("Soldador de 12V, con tension correcta");
      }                             
    }else{
      out_pwm =0;
      analogWrite(PWMpin, out_pwm);
      digitalWrite(LED_V, LOW);
      digitalWrite(LED_R, HIGH);                                  // led rojo en ON
      if (DEBUGGER){
        Serial.println("ERROR: soldador de 12V, con tension >15V");
      }
      do{                                               
        if (ON_OFF_zumbador) zumbador_ON (500);
        delay(2000);
        tension = (int)(analogRead(ADC_V)* 0.3);                  // solo salimos cuando la tension es menor de 15V
      }while(tension > 15); 
      soft_reset();   
    }
  }
// gestión menu configuración  
  if (menu){
    if (digitalRead(BOMBA_PUL) == false) digitalWrite(BOMBA, LOW);
    menu_config();                                                // gestión menu de configuración
    inicia_tft();
    analogWrite(PWMpin, out_pwm);                                 // restauramos PWM
    writeHEATING(ini_temp-1, actual_temperatura_tft-1, out_pwm);  // actualizamos en tft 
  }
// gestión bomba vació desoldador  
  if (de_soldador == desoldador){                                 // si tenemos activo el desoldador
    if (digitalRead(BOMBA_PUL) == false){
      delay(10);                                                  // eliminamos rebotes
      if (digitalRead(BOMBA_PUL) == false) digitalWrite(BOMBA, HIGH); 
    }else{
      digitalWrite(BOMBA, LOW);
    }
  }
// gestión ON/OFF standby  
  if (digitalRead(STANDBY) == false){                              // comprobamos si secuencia en standby
    tft.setCursor(2,55);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_RED);                                  // indicamos estado standby, rojo  en standby
    tft.print("SB"); 
    if (DEBUGGER){
      Serial.println("En STANDBY");
    }
    if (standby == OFF){                                           // si estuvimos en trabajo
      standby = ON;                                 
      mem_SB = ini_temp;
      if (TIPO == RT1)  Temp_Standby = 150;                        // para el soldador RT1 Standby 150 grados
      if (TIPO == JBC)  Temp_Standby = 250;
      ini_temp = Temp_Standby;
      Tem__Setpoint = Temp_Standby;
    }
    if (ciclo_espera == 0){
      espera_standby =millis();                                    // obtenemos el tiempo actual de furncionamiento
      ciclo_espera =espera_standby;
    }else{
      ciclo_espera = millis(); 
      if ((ciclo_espera - espera_standby) > (Tiempo_stop *60000)){
        out_pwm =0;
        analogWrite(PWMpin, out_pwm);      
        if (DEBUGGER){
          Serial.print("Se sobre paso el tiempo de STANDBY: ");  
          Serial.print(Tiempo_stop);
          Serial.println(" minutos, pulsar para reiniciar");
        }
        do{
          tft.setCursor(2,55);
          tft.setTextSize(2);
          tft.setTextColor(ST7735_YELLOW);                               
          tft.print("SB");
          delay(400);
          if (ON_OFF_zumbador) zumbador_ON (100);
          tft.setCursor(2,55);    
          tft.setTextColor(ST7735_BLUE);                               
          tft.print("SB");
          delay(500);
        }while(!menu && digitalRead(STANDBY) == false);
        menu =0;
        ciclo_espera =0;
      }
    }  
  }else{ 
    ciclo_espera =0;
    if (DEBUGGER){
      Serial.println("En trabajo");
    }
    if (standby == ON){                                             // si estuvimos en standby
      standby = OFF;  
      ini_temp =mem_SB;  
      Tem__Setpoint = ini_temp;   
      espera_standby =0;                                   
      OK =false;
    }
    tft.setCursor(2,55);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("SB");                                                // indicamos estado standby, blanco en trabajo 
  }
// gestión modificación temperatura de trabajo
    if (posicion_encoder !=0){                                      // comprobamos si tenemos pulsos del encoder
      OK =false;
      if (posicion_encoder >0){                                     // estamos incrementando
        if (ini_temp >= Temp_Max){
          ini_temp = Temp_Max;
          if (DEBUGGER){
            Serial.println("Temperatura maxima alcanzada!");
          }
          if (ON_OFF_zumbador) zumbador_ON (250);      
        }else ini_temp = (ini_temp + posicion_encoder);             // incrementamos
      }else{                                                        // decrementamos
        if (ini_temp <= Temp_Standby){
          ini_temp = Temp_Standby;
          if (DEBUGGER){
            Serial.println("Temperatura minima igual a standby!");
          }
          if (ON_OFF_zumbador) zumbador_ON (250); 
        }else{
          if (posicion_encoder < 0){
            ini_temp = ini_temp + posicion_encoder;                // decrementamos, valor en negativo
          }else{
            ini_temp = ini_temp - posicion_encoder;                // decrementamos, valor en positivo  
          } 
        }
      }  
      posicion_encoder =0;
      Tem__Setpoint = ini_temp;
      writeHEATING(ini_temp, actual_temperatura, Output);          // actualizamos en tft
    }
// gestión temperatura
  Setpoint = Tem__Setpoint;                                         
  Input = getTemperatura(DEBUGGER, out_pwm);                       // leemos ta temperatura para obtener el input para PID  
  actual_temperatura =(int)Input;                                  // para gestión display tipo int 
// seguridad  
  if (actual_temperatura > (Tem__Setpoint+25 && standby == OFF)){  // si  temperatura alta y NO standby
    if (++ciclo_error > 4 && out_pwm !=0){
      out_pwm =0;
      analogWrite(PWMpin, out_pwm);
      digitalWrite(LED_V, LOW);
      digitalRead(LED_R) == true ? digitalWrite(LED_R, LOW): digitalWrite(LED_R, HIGH);
        writeHEATING(ini_temp, actual_temperatura, out_pwm);      // actualizamos en tft 
      if (DEBUGGER){
        Serial.print("Temperatura muy alta! ");
        Serial.println(actual_temperatura);
      }
    }
  }
  if (actual_temperatura == (Temp_Max+100)){                      // error en lectura temperatura
    out_pwm =0;
    analogWrite(PWMpin, out_pwm);
    digitalWrite(LED_V, LOW);
    digitalWrite(LED_R, HIGH);                                   // led rojo en ON
    if (ON_OFF_zumbador) zumbador_ON (250);
    delay(500);
    if (ON_OFF_zumbador) zumbador_ON (250);
    if (DEBUGGER){
      Serial.println("Error en lectura ");
    }
  }else{ 
    ciclo_error = 0;
    if (Input < Setpoint+25){                                    // control temperatura ON
      diferencia =(Tem__Setpoint - actual_temperatura);  
      if (DEBUGGER){
        Serial.print("Setpoint: ");
        Serial.println(Tem__Setpoint);
      }
      if(diferencia > 100){                                      // comprobamos la diferencía entre la temperatura actual y setpoint    
        analogWrite(PWMpin,250);                                 // si es mayor de 100 grados NO usamos PID
        out_pwm = 250;
        delay (diferencia);
        if (DEBUGGER){
          Serial.println("Contol PID OFF");
        }
      }else{                                                     // control temperatura por PID
        myPID.Compute();                                         // en el puntero Output obtenemos el valor para PWM 
        (actual_temperatura > (Tem__Setpoint+50))? out_pwm =0: out_pwm =Output;
        analogWrite(PWMpin, out_pwm);                            // ajustamos valor PWM 
        if (DEBUGGER){
          Serial.println("Contol PID ON");
        }
      }
      if (DEBUGGER){
        Serial.print("PWM a: ");
        Serial.println(out_pwm);
      }
    }else{                                                       // control temperatura OFF
      out_pwm = 0;
      analogWrite(PWMpin, out_pwm);
      if (DEBUGGER){
        Serial.println("PWM OFF");
      }
    }
    if (ciclo_media ==5 || ciclo_media ==10){                    // 500ms, 1000ms
      if (ciclo_media == 10) ciclo_media = 0;
      if (actual_temperatura > (Tem__Setpoint-5)){      
        if (actual_temperatura > (Tem__Setpoint+5)){
          digitalWrite(LED_R, LOW);
          (digitalRead(LED_V) == true) ? digitalWrite(LED_V, LOW): digitalWrite(LED_V, HIGH);// parpadeo verde
        }else{
          digitalWrite(LED_R, LOW);
          digitalWrite(LED_V, HIGH);                             // temperatura OK  
        }    
      }else{
        digitalWrite(LED_V, LOW);
        (digitalRead(LED_R) == true) ? digitalWrite(LED_R, LOW): digitalWrite(LED_R, HIGH); // parpadeo rojo
      }
    }
    if (++ciclo_media == 10){
      actual_temperatura_tft = (media + actual_temperatura) / 10;       
      media = 0;
      if (DEBUGGER){
        Serial.print("Temperatura media: ");
        Serial.println (actual_temperatura_tft);
        tension = (int)(analogRead(ADC_V) *0.0309);                      
        Serial.print("Tension de alimentacion: ");
        Serial.print(tension);
        Serial.println("V");
      }
    }else{
      media = media + actual_temperatura;
      writeHEATING(ini_temp, actual_temperatura_tft, out_pwm);   // actualizamos en tft 
      if (OK ==false){                                           // temperatura alcanzada correctamente
        if (actual_temperatura_tft >=Tem__Setpoint){
          if (ON_OFF_zumbador) zumbador_ON (100);
          OK =true;
        }
      }
    }
  }
  delay(50);
  if (out_pwm > 200){                              
    consumo = analogRead(ADC_I);  
    if (consumo < 520){
      if (++error_consumo ==150){                                // es necesario realizar la lectura en el periodo positivo del PWM
        analogWrite(PWMpin, 0);
        if (ON_OFF_zumbador) zumbador_ON (500);
        if (DEBUGGER){
          Serial.println("!ERROR! De/Solsador SIN consumo");
        }
        tft.fillScreen(ST7735_BLACK);                            // borramos pantalla
        tft.setTextSize(1);
        testdrawtext ("!ERROR! De/Solsador SIN consumo", ST7735_WHITE);
        digitalWrite(LED_V, LOW);
        while (true){                                           // bucle infinito!! 
          delay(100);      
          (digitalRead(LED_R) == true) ? digitalWrite(LED_R, LOW): digitalWrite(LED_R, HIGH); // parpadeo rojo
        }
      }
    }else{
      error_consumo =0;  
    }
  }
  delay(50);
  if (DEBUGGER){
    Serial.println("----------------------------------------------------");
  }
  if (!inicio){                                        // solo ejecutamos una vez
    tft.setCursor(0,97);    
    tft.setTextSize(2);
    if (tension_12_24 == V_12 &&  TIPO ==RT1){
      tft.setTextColor(ST7735_RED);
      tft.print("RT1");
    }else{
      if (TIPO ==T12){
        tft.setTextColor(ST7735_BLUE);
        tft.print("T12");
      }else{
        tft.setTextColor(ST7735_YELLOW);
        tft.print("JBC");
      }
    }
    inicio = true;
    INTRO =false;
    OK =false;
    intensidad =0;
    if (DEBUGGER){
      Serial.println("***************** INICIALIZADO *********************");
    }
    if (ON_OFF_zumbador){
      if (TIPO >=1)
      zumbador_ON (50);
      if (TIPO >=2)
      delay(300);
      zumbador_ON (50);
      if (TIPO ==3)
      delay(300);
      zumbador_ON (50);
    }
    attachInterrupt(0, encoder, FALLING);                     // habilitamos interrupción 0 CLK encoder, dispara en el flanco de bajada
    attachInterrupt(5, pulsacion, FALLING);                   // habilitamos interrupción 5 del pulsador del encoder, dispara en el flanco de bajada
  }
}
