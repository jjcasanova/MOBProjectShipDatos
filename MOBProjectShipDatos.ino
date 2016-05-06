#include "Arduino.h"
#include <VirtualWire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;        // pines del gps bee a 3,5v, otros gps pueden ir a 5v
static const uint32_t GPSBaud = 9600;
double alfa;
double bravo;
double charlie;
double delta;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// emisor transmite por el pin 12 y se conecta a 5v

void setup()
{

  //Iniciamos el Serial y la comunicacion por radio
    Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.println("Radio encendida");
  vw_setup(2000);  Serial.println(F("latitud   longitud         alfa         bravo"));

}
void loop()

{
//funcion que define datos sacados de gps y atribucion de datos a variables alfa y bravo
printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
 printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
 charlie =(gps.location.lat());
  delta = (gps.location.lng());
  alfa = charlie * 10000000; bravo = (delta+60)*10000000;              // llega a 030ºl E y coge casi toda la costa española, ojo a Canarias, verificar en receptor
                                                                     // con estos parametros se evitan decimales y negativos

double victor=gps.speed.kmph(); Serial.print('\n'); Serial.println("velocidad"); Serial.println(victor,8);
double romeo=gps.course.deg();  Serial.println("rumbo"); Serial.println(romeo,8);
double golf = victor*100;Serial.println("golf"); Serial.println(golf,8);       // maximo 4000 para motora
double hotel= romeo*10000;Serial.println("hotel"); Serial.println(hotel,8);

Serial.print("    "); Serial.print(charlie,6); Serial.print("    "); Serial.print(delta,6); Serial.print("    "); Serial.print(alfa,0); Serial.print("    "); Serial.print(bravo,0);
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No recibe GPS verificar cableado"));
    Serial.println();                    //Ojo aqui todos los print son para el minitor mientras se verifica que se programa, luego no sirve para nada




//Funcion para enviar los mensajes alfa y bravo

{

//cadena para latitud
String Alfa=String(alfa,0); // lo paso a un String con 0 decimales
int longitudALFA=Alfa.length()+1; // calculo la longitud del String y lo paso a un entero
char msg[longitudALFA]; // declaro una nueva matriz de caracteres
Alfa.toCharArray(msg,longitudALFA); // le paso un array de caracteres de longitud de la cadena
vw_send((uint8_t *)msg, strlen(msg)); // se pasan los valores al sensor
vw_wait_tx();  //Se hace la transmisión de datos
delay(200);

//cadena para longitud
String Bravo=String(bravo,0); // lo paso a un String con 0 decimales
int longitudBRAVO=Bravo.length()+1; // calculo la longitud del String y lo paso a un entero
//char msg[longitudBRAVO]; // declaro una nueva matriz de caracteres
Bravo.toCharArray(msg,longitudALFA); // le paso un array de caracteres de longitud de la cadena
vw_send((uint8_t *)msg, strlen(msg)); // se pasan los valores al sensor
vw_wait_tx();  //Se hace la transmisión de datos
delay(1000); //Se espera un segundo

//cadena para velocidad
String Golf=String(golf,0); // lo paso a un String con 0 decimales
int longitudGOLF=Golf.length()+1; // calculo la longitud del String y lo paso a un entero
//char msg[longitudGOLF]; // declaro una nueva matriz de caracteres
Golf.toCharArray(msg,longitudGOLF); // le paso un array de caracteres de longitud de la cadena
vw_send((uint8_t *)msg, strlen(msg)); // se pasan los valores al sensor
vw_wait_tx();  //Se hace la transmisión de datos
delay(200);

//cadena para mi rumbo
String Hotel=String(hotel,0); // lo paso a un String con 0 decimales
int longitudHOTEL=Hotel.length()+1; // calculo la longitud del String y lo paso a un entero
//char msg[longitudHOTEL]; // declaro una nueva matriz de caracteres
Hotel.toCharArray(msg,longitudHOTEL); // le paso un array de caracteres de longitud de la cadena
vw_send((uint8_t *)msg, strlen(msg)); // se pasan los valores al sensor
vw_wait_tx();  //Se hace la transmisión de datos
delay(200);

}


}






// estas funciones hacen que el GPS funcione y verifican datos. Por eso hay que conectar Tx y Rx
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
