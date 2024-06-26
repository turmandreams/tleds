#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>

#include <FS.h>
#include <SD.h>

#define SD_CLK  18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 4

#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite bufferscreen = TFT_eSprite(&tft);

boolean pinta=true;
boolean sdcard=false;

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240


#define TFT_MISO 19 // (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   14  // Chip select control pin
#define TFT_DC   27  // Data Command control pin
#define TFT_RST  33  // Reset pin (could connect to RST pin)
#define TFT_BL   32  


#include "SPIFFS.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

/////////////////////////
//////Espectrometro//////
/////////////////////////

#include "arduinoFFT.h"    


const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);



int pinadc=36;
int pindata=16;

///////////////////////////
//////////////////////////

//SPIClass spiSD = SPIClass(VSPI);

#include "AnimatedGIF.h"

#include "ColorConverterLib.h"

#include <FastLED_NeoPixel.h>

#define DATA_PIN 16
#define NUM_LEDS 4096
#define BRIGHTNESS 50

int numpixeles=256;

CRGB leds[NUM_LEDS];
FastLED_NeoPixel_Variant strip(leds, NUM_LEDS);


AnimatedGIF gif;

String carpeta="/";
int numerogifs=0;

static File FSGifFile; // temp gif file holder
static File GifRootFolder; // directory listing

#define eeprominicio 0

#define card_cs 2

const char* ssid = "TLEDS";
const char* password = "1234567890";

WiFiServer server(80);
WiFiClient client;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



///Valores Efectos////

int tipo[255];
int ngifs[255];
String gifs[255];
String texto[255];
int velocidad[255];
int loopss[255];
int tolerancia[255];
int brillo[255];
int saturacion[255];
int rojo[255];
int verde[255];
int azul[255];
int transicion[255];

String cadena="";
String configuracion="";

boolean espera=false;

int numpantalla=0;

int numefectos=0;

int screensizex=320;
int screensizey=240;

uint16_t pantalla[64][32];
uint16_t pantalla2[64][32];

int pantallasizex=16;
int pantallasizey=16;

int esquina=0;
int serpentina=0;
int rotar=0;

int spritesizex=16;
int spritesizey=16;

int gifsizex=0;
int gifsizey=0;

float escalax;
float escalay;

int pantallax;
int pantallay;

uint8_t R,G,B;

boolean siguiente=false;

static int xOffset = 0;
static int yOffset = 0;

int lastFile = -1;

const int maxLoopsDuration  =  3000; // ms, max cumulated time after the GIF will break loop
const int maxGifDuration    = 30000; // ms, max GIF duration

char GifComment[256];

union Float_Byte{
  float datoF;
  byte  datoB[4];
}unionFB;

union Int_Byte{
  int datoI;
  byte  datoB[2];
}unionIB;

int addr=eeprominicio;

int segundos=0;
int minutos=0;
int horas=0;

int dia=0;
int mes=0;
int ano=0;

int contadorframe=0;

boolean borrar=false;
int numborrar=0;


float valoradc=0.0;

TaskHandle_t Task1;  // declaramos para poder ejecutar en otro core


void IRAM_ATTR onTimer() {     
    portENTER_CRITICAL_ISR(&timerMux);

    segundos++;
    if(segundos>=60) { 
        segundos=0;
        minutos++;
        if(minutos>=60) { 
            minutos=0;
            horas++;          
            if(horas>=24) {
                horas=0;
                dia++;
                int diamax=31;
                if((mes==4)||(mes==6)||(mes==9)||(mes==11)){ diamax=30; }
                else if(mes==2){ diamax=28; }

                if(dia>diamax){
                    dia=1;
                    mes++;
                    if(mes>12) { ano++; }
                }                
            }
        }
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}


void inicializatira(){

  if(pindata==2){
      strip.begin(FastLED.addLeds<WS2812B,2,GRB>(leds,NUM_LEDS));
  }else if(pindata==4){
      strip.begin(FastLED.addLeds<WS2812B,4,GRB>(leds,NUM_LEDS));
  }else if(pindata==5){
      strip.begin(FastLED.addLeds<WS2812B,5,GRB>(leds,NUM_LEDS));
  }else if(pindata==12){
      strip.begin(FastLED.addLeds<WS2812B,12,GRB>(leds,NUM_LEDS));
  }else if(pindata==13){
      strip.begin(FastLED.addLeds<WS2812B,13,GRB>(leds,NUM_LEDS));
  }else if(pindata==14){
      strip.begin(FastLED.addLeds<WS2812B,14,GRB>(leds,NUM_LEDS));
  }else if(pindata==15){
      strip.begin(FastLED.addLeds<WS2812B,15,GRB>(leds,NUM_LEDS));
  }else if(pindata==16){
      strip.begin(FastLED.addLeds<WS2812B,16,GRB>(leds,NUM_LEDS));
  }else if(pindata==17){
      strip.begin(FastLED.addLeds<WS2812B,17,GRB>(leds,NUM_LEDS));
  }else if(pindata==18){
      strip.begin(FastLED.addLeds<WS2812B,18,GRB>(leds,NUM_LEDS));
  }else if(pindata==19){
      strip.begin(FastLED.addLeds<WS2812B,19,GRB>(leds,NUM_LEDS));
  }else if(pindata==21){
      strip.begin(FastLED.addLeds<WS2812B,21,GRB>(leds,NUM_LEDS));
  }else if(pindata==22){
      strip.begin(FastLED.addLeds<WS2812B,22,GRB>(leds,NUM_LEDS));
  }else if(pindata==23){
      strip.begin(FastLED.addLeds<WS2812B,23,GRB>(leds,NUM_LEDS));
  }else if(pindata==25){
      strip.begin(FastLED.addLeds<WS2812B,25,GRB>(leds,NUM_LEDS));
  }else if(pindata==26){
      strip.begin(FastLED.addLeds<WS2812B,26,GRB>(leds,NUM_LEDS));
  }else if(pindata==27){
      strip.begin(FastLED.addLeds<WS2812B,27,GRB>(leds,NUM_LEDS));
  }else if(pindata==32){
      strip.begin(FastLED.addLeds<WS2812B,32,GRB>(leds,NUM_LEDS));
  }else if(pindata==33){
      strip.begin(FastLED.addLeds<WS2812B,33,GRB>(leds,NUM_LEDS));
  }
  
}

void tftclear(){

   if(pinta){ 
        tft.fillRect(0,0,DISPLAY_WIDTH,DISPLAY_HEIGHT,0);
   }
   
}

void analizacadena(){

       String d="";
              
       int pos1=cadena.indexOf("@");

       int pos2=cadena.indexOf(";",pos1+1);
       d= cadena.substring(pos1+1,pos2);
       numefectos=d.toInt(); 

       pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
       numpantalla=d.toInt(); 
          
       for(int i=0;i<numefectos;i++){

          if(i!=0) { 
             pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
             transicion[i-1]=d.toInt(); 
          }
                    
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          tipo[i]=d.toInt(); 
          
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          ngifs[i]=d.toInt(); 
          
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          d.replace("%20"," ");texto[i]=d; 

          if((tipo[i]==3)||(tipo[i]==4)){
              int pos3=d.indexOf("(");
              int pos4=d.indexOf("/",pos3+1);
              String d2 = d.substring(pos3+1,pos4);dia=d2.toInt();//Serial.println(dia);
              
              pos3=pos4+1;
              pos4=d.indexOf("/",pos3);
              d2 = d.substring(pos3,pos4);mes=d2.toInt();//Serial.println(mes);            
              
              pos3=pos4+1;
              pos4=d.indexOf(")",pos3);
              d2 = d.substring(pos3,pos4);ano=d2.toInt();//Serial.println(ano);            

              pos3=d.indexOf(")(");
              pos4=d.indexOf(":",pos3+1);
              d2 = d.substring(pos3+2,pos4);horas=d2.toInt();//Serial.println(horas);
              
              pos3=pos4+1;
              pos4=d.indexOf(":",pos3);
              d2 = d.substring(pos3,pos4);minutos=d2.toInt();//Serial.println(minutos);            
              
              pos3=pos4+1;
              pos4=d.indexOf(")",pos3);
              d2 = d.substring(pos3,pos4);segundos=d2.toInt();//Serial.println(segundos);            
                
          }
          
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          velocidad[i]=d.toInt(); if(velocidad[i]==0) { velocidad[i]=1; }
          
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          loopss[i]=d.toInt(); 

          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          tolerancia[i]=d.toInt(); 
   
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          brillo[i]=d.toInt(); 
   
          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          saturacion[i]=d.toInt(); 

          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          rojo[i]=d.toInt(); 

          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          verde[i]=d.toInt(); 

          pos1=pos2;pos2=cadena.indexOf(";",pos1+1);d = cadena.substring(pos1+1,pos2);
          azul[i]=d.toInt();           

       }

  
}

void analizaconfiguracion(){

     String d="";
     
     int pos1=configuracion.indexOf("?"); 

     int pos2=configuracion.indexOf(";",pos1+1);
     d=configuracion.substring(pos1+1,pos2);
     pantallasizex=d.toInt(); 
     if(pantallasizex>64) { pantallasizex=64;}

     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     pantallasizey=d.toInt(); 
     if(pantallasizey>64) { pantallasizey=64;}

     spritesizex=pantallasizex;
     spritesizey=pantallasizey;

     numpixeles=pantallasizex*pantallasizey;

     strip.updateLength(numpixeles);
     
     bufferscreen.deleteSprite();
     bufferscreen.createSprite(spritesizex,spritesizey);
     
     //Serial.print(spritesizex);Serial.print(",");Serial.println(spritesizey);
     
     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     esquina=d.toInt(); 
     
     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     serpentina=d.toInt(); 

     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     rotar=d.toInt(); 

     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     pindata=d.toInt(); 

     inicializatira();
     
     pos1=pos2;pos2=configuracion.indexOf(";",pos1+1);d = configuracion.substring(pos1+1,pos2);
     pinadc=d.toInt(); 

     //Serial.print(pindata);Serial.print(",");Serial.println(pinadc);
     
}

boolean init_sd(){
  /*
    spiSD.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    pinMode(SD_CS,OUTPUT);
    digitalWrite(SD_CS,LOW);
    if (!SD.begin(SD_CS, spiSD, 4000000)){
        Serial.println("SD Card Mount Failed");
        return false;
    }else{
        Serial.println("SD Card Mounted");      
        return true;
    }
    */  
}

void cargargifsSD(){

  int intentos=0;
  Serial.println("Se va leer la SD");  
  while(intentos<10) {    
      boolean sdcargada=init_sd();
      if(sdcargada){ break; }
           
      Serial.println(".");
      intentos++;              
      delay(50);       
  }

  intentos=0;
  while(intentos<10) {
      Serial.println("Buscando GIFs");
      
      numerogifs = getGifInventory(carpeta); 
      if(numerogifs!=0){ break; }
      
      intentos++;              
      delay(50);       

  } 
  
}

void cargargifsSPIFFS(){

  int intentos=0;
  Serial.println("Se va cargar SPIFFS");  
  while(intentos<10) {    
      boolean spiffscargada=SPIFFS.begin();
      if(spiffscargada){ break; }
           
      Serial.println(".");
      intentos++;              
      delay(50);       
  }

  if(intentos<10){ Serial.println("Se ha cargardo SPIFFS correctamente.");  }
  else{ Serial.println("Fallo al cargar SPIFFS.");  }
  
  intentos=0;
  while(intentos<10) {
      Serial.println("Buscando GIFs");
      
      numerogifs = getGifInventory(carpeta); 
      if(numerogifs!=0){ break; }
      
      intentos++;              
      delay(50);       

  } 
  
}

void guardaconfiguracion(){

    File archivo = SPIFFS.open("/config.txt","w");
      
    if (archivo) {     

        archivo.println(configuracion);
        archivo.println(cadena);        
        
    }else{
        Serial.println("No se ha podido escribir config.txt");
    }


    archivo.close();


   
}

void cargaconfiguracion(){

    File archivo = SPIFFS.open("/config.txt","r");
          
    if (archivo) {     
            
        configuracion="";
        while (archivo.available()) {
            char c=archivo.read();
            if(c=='\n') { break; }

            if(c!='\r'){ configuracion+=c; }          
        }

        //Serial.println(configuracion);

        
        analizaconfiguracion();

        cadena="";
        while (archivo.available()) {
            char c=archivo.read();
            if(c=='\n') { break; }
            if(c!='\r'){ cadena+=c; }          
        }

        //Serial.println(cadena);
        analizacadena();        
        
        
    }else{
        Serial.println("No se ha podido cargar config.txt");
    }


    archivo.close();
    
}


void moveback(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    for(int jj=0;jj<pantallasizex;jj++){       
      
      int ini=millis();
      
      for(int j=0;j<pantallasizey;j++){
        
          for(int i=1;i<pantallasizex;i++){
                                        
              pantalla[i-1][j]=pantalla[i][j];                  
              pantalla[i][j]=0;
              if(siguiente){ return;} 
              
          }
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
}

void moveback2(){

    for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla2[i][j]=pantalla[i][j];
              pantalla[i][j]=0;
          }
    }
    
    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    for(int jj=0;jj<pantallasizex;jj++){       
      
      int ini=millis();
      
      for(int j=0;j<pantallasizey;j++){

          int ii=(pantallasizex-1-jj);
          for(int i=0;i<=jj;i++){                                        
              pantalla[ii][j]=pantalla2[i][j];                           
              ii++;    
              if(siguiente){ return;}           
          }
          
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
    
}

void moveforward(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    for(int jj=0;jj<pantallasizex;jj++){       
      
      int ini=millis();
      
      for(int j=0;j<pantallasizey;j++){
        
          for(int i=(pantallasizex-1);i>0;i--){
                                        
              pantalla[i][j]=pantalla[i-1][j];              
              pantalla[i-1][j]=0;
              if(siguiente){ return;} 
          }
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
}

void moveforward2(){

    for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla2[i][j]=pantalla[i][j];
              pantalla[i][j]=0;
          }
    }
    
    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    for(int jj=pantallasizex;jj>=0;jj--){       
      
      int ini=millis();
      
      for(int j=0;j<pantallasizey;j++){

          int ii=0;
          for(int i=jj;i<pantallasizex;i++){                                        
              pantalla[ii][j]=pantalla2[i][j];                           
              ii++;    

              if(siguiente){ return;} 
          }
          
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
}

void fadetoblack(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    float brillo=0.95;

    for(int jj=0;jj<20;jj++){       
      
      int ini=millis();
      
      for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              
              GetRGBFrom16Bit(pantalla[i][j]); 
            
              int R03=(int)R*brillo;
              int G03=(int)G*brillo;
              int B03=(int)B*brillo;
              
              pantalla[i][j]=rgb565(R03,G03,B03);

              if(siguiente){ return;} 
              
          }
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
}

void fadetoblack2(){

    
    for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla2[i][j]=pantalla[i][j];
          }
    }
    
    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }
  
    float brillo=0.0;

    for(int jj=0;jj<20;jj++){       
      
      int ini=millis();
      
      for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              
              GetRGBFrom16Bit(pantalla2[i][j]); 
            
              int R03=(int)R*brillo;
              int G03=(int)G*brillo;
              int B03=(int)B*brillo;
              
              pantalla[i][j]=rgb565(R03,G03,B03);

              if(siguiente){ return;}               
          }
      }

      brillo+=0.05;

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }     
      
    }
}


void lines(){

     for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla2[i][j]=pantalla[i][j];
          }
    }
    
    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    
    for(int jj=0;jj<pantallasizex;jj++){             
      
      int ini=millis();      
      for(int j=0;j<pantallasizey;j+=2){        
          for(int i=1;i<pantallasizex;i++){                                        
              pantalla[i-1][j]=pantalla[i][j];                  
              pantalla[i][j]=0;
              if(siguiente){ return;}              
          }
      }

      for(int j=1;j<pantallasizey;j+=2){        
          for(int i=(pantallasizex-1);i>0;i--){                                        
              pantalla[i][j]=pantalla[i-1][j];              
              pantalla[i-1][j]=0;
              if(siguiente){ return;} 
          }
      }

      actualizarleds();
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }          
    }
}


void lines2(){

    for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla2[i][j]=pantalla[i][j];
              pantalla[i][j]=0;
          }
    }
    
    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    int jjj=pantallasizex;
    
    for(int jj=0;jj<pantallasizex;jj++){       
      
      int ini=millis();
      
      for(int j=0;j<pantallasizey;j+=2){
          int ii=(pantallasizex-1-jj);
          for(int i=0;i<=jj;i++){                                        
              pantalla[ii][j]=pantalla2[i][j];                           
              ii++;    
              if(siguiente){ return;}           
          }         
      }

      for(int j=1;j<pantallasizey;j+=2){
          int ii=0;
          for(int i=jjj;i<pantallasizex;i++){                                        
              pantalla[ii][j]=pantalla2[i][j];                           
              ii++;    
              if(siguiente){ return;} 
          }          
      }

      actualizarleds();
      
      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }      

      jjj--;
      
    }

    for(int i=0;i<pantallasizex;i++){
          for(int j=0;j<pantallasizey;j++){
              pantalla[i][j]=pantalla2[i][j];              
          }
    }
    actualizarleds();
    
}


void tfttopantalla(){
  
  for(int i=0;i<pantallasizex;i++){
      for(int j=0;j<pantallasizey;j++){
          pantalla[i][j]=0;
      }
  }
  
  
   escalax= ((float)spritesizex) / (float)pantallasizex;
   escalay= ((float)spritesizey) / (float)pantallasizey;
     
   for(int y=0;y<spritesizey;y++){

      for(int x=0;x<spritesizex;x++){
    
          uint16_t color=bufferscreen.readPixel(x,y); 

          //if(color!=0){ Serial.println(color); }
          
          GetRGBFrom16Bit(color); 
          
          int Rojo1=R;
          int Verde1=G;
          int Azul1=B;
    
          int px=x/escalax;
          int py=y/escalay;
    
          float panx=(((float)x)/escalax)-px;
    
          
          if(panx<0.5){
                  
              GetRGBFrom16Bit(pantalla[px][py]); 
              int R02=R;int G02=G;int B02=B;
        
              int R03=(Rojo1+R02)/2;
              int G03=(Verde1+G02)/2;
              int B03=(Azul1+B02)/2;
        
              pantalla[px][py]=rgb565(R03,G03,B03);
    
          }
          
          //Serial.print(pantallax);Serial.print(",");Serial.print(pantallay);Serial.print(" - ");Serial.print(px);Serial.print(",");Serial.println(py);

      }
  }
  
}


void espectrometro2(){

  //Serial.println("Espectrometro");
  
  int tiempo=(10000)/velocidad[numpantalla];
  if(tiempo>1000){ tiempo=1000; }
   
  uint16_t samples = 64;

  double vReal[samples];
  double vImag[samples];

  uint16_t color2=rgb565(255,255,255);
  if(pinta) { tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2); }
  color2=rgb565(0,0,0);
  bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);

  int angulo=0;

  int RR=random(50,150);
  int GG=random(50,150);
  int BB=random(50,150);

  color2=rgb565(RR,GG,BB);

  int contador=0;
          
  for(int jj=0;jj<loopss[numpantalla];jj++){
        
        for(int j=0;j<100;j++){            
            
            int ini=millis();
            
            for(int i=0;i<samples;i++){
                vReal[i] = analogRead(pinadc);
                vImag[i] = 0;
                delayMicroseconds(100);
                if(siguiente){ return;} 
            }

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();


            int maximo=0;
            
            for(int i=(samples/2)-1;i>=6;i--){               
                int p=(int)vReal[i];
                if(p>maximo){ maximo=p; }
                if(siguiente){ return;} 
            }

            int ty=spritesizey/2;

            if(maximo>4096) { maximo=4096; }
            
            //Serial.print("TY : ");Serial.println(ty);
            //Serial.print("Maximo : ");Serial.println(maximo);
  
            int r=map(maximo,0,4096,0,ty);   
            if(r<3) { r=3; }
            
            float radian = ((float)(angulo)*0.0174533);
            float x=sin(radian)*r;x+=(spritesizex/2.0);            
            float y=cos(radian)*r;y+=(spritesizey/2.0);

            int x2=(spritesizex/2)-1;
            int y2=(spritesizey/2)-1;

            float x3=sin(radian)*ty;x3+=(spritesizex/2.0);            
            float y3=cos(radian)*ty;y3+=(spritesizey/2.0);

            bufferscreen.drawLine((int)x,(int)y,(int)x3,(int)y3,0);
            
            bufferscreen.drawLine(x2,y2,(int)x,(int)y,color2);

            contador++;
            
            if(contador>4) {
                contador=0;
                RR=random(50,150);
                GG=random(50,150);
                BB=random(50,150);
                color2=rgb565(RR,GG,BB);                
            }
                                            
            angulo+=7; 
            if(angulo>=360) { angulo-=360; }

            if(pinta){ bufferscreen.pushSprite(0,0);}
            tfttopantalla();
            actualizarleds();
  
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }              

        }
  
    }             
  
}



void espectrometro(){

  //Serial.println("Espectrometro");
  
  int tiempo=(10000)/velocidad[numpantalla];
  if(tiempo>1000){ tiempo=1000; }
   
  uint16_t color2=rgb565(255,255,255);

  if(pinta){  
    tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);
  }

  int RR[pantallasizex];
  int GG[pantallasizex];
  int BB[pantallasizex];

  for(int i=0;i<pantallasizex;i++){
      RR[i]=random(50,150);
      GG[i]=random(50,150);
      BB[i]=random(50,150);    
  }

  for(int jj=0;jj<loopss[numpantalla];jj++){

        for(int j=0;j<100;j++){

            color2=rgb565(0,0,0);
            bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);
  
            int ini=millis();
            
            for(int i=0;i<samples;i++){
                vReal[i] = analogRead(pinadc);
                vImag[i] = 0;
                delayMicroseconds(100);
                if(siguiente){ return;} 
            }

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();

            int x=pantallasizex-1;  
            
            for(int i=(samples/2)-1;i>=0;i--){               
                color2=rgb565(RR[i],GG[i],BB[i]);
                
                int p=map((int)vReal[i],0,4096,pantallasizey,0);
                bufferscreen.fillRect(x,p,1,pantallasizey-p,color2);               
                x--;
                if(x<0){ break; }                
                if(siguiente){ return;} 
            }

            if(pinta){bufferscreen.pushSprite(0,0);}
            
            tfttopantalla();
            actualizarleds();
  
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }         

        }
  
    }             
  
}

void test(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2); }

    color2=rgb565(rojo[numpantalla],verde[numpantalla],azul[numpantalla]);
    bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);

    if(pinta){ bufferscreen.pushSprite(0,0); }

    strip.clear();
       
    for(int jj=0;jj<loopss[numpantalla];jj++){

      for(int i=0;i<numpixeles;i++){

            int ini=millis();

            strip.setPixelColor(i, strip.Color(rojo[numpantalla],verde[numpantalla],azul[numpantalla]));

            strip.show();
             
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
            
      }
                
    }
  
}


void colorescena(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2); }

    color2=rgb565(rojo[numpantalla],verde[numpantalla],azul[numpantalla]);
    bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);


    contadorframe=0;
       
    for(int jj=0;jj<loopss[numpantalla];jj++){

            int ini=millis();
            
            if(pinta){ bufferscreen.pushSprite(0,0); }
            tfttopantalla();

             if((contadorframe==0)&&(numpantalla!=0)){            
                if(transicion[numpantalla-1]==1){ //FADE TO BLACK  
                    fadetoblack2();    
                }else if(transicion[numpantalla-1]==2){ //MOVE FORDWARD
                    moveforward2();    
                }else if(transicion[numpantalla-1]==3){ //MOVE BACK
                    moveback2();    
                }else if(transicion[numpantalla-1]==4){ //LINES
                    lines2();    
                }                    
            }

            contadorframe++;
    
            actualizarleds();
  
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
            
    }
  
}

void battle(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);  }

    strip.clear();
    
    for(int jj=0;jj<loopss[numpantalla];jj++){

        bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
        if(pinta){ bufferscreen.pushSprite(0,0); }

           
        for(int i=5;i>0;i--){
    
            bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
            if(pinta){ bufferscreen.pushSprite(0,0); }

            int r=random(0,255);
            int g=random(0,255);
            int b=random(0,255);
            
            color2=rgb565(r,g,b); 
            
            bufferscreen.setTextColor(color2);
            String t="";t+=i;
            if(pantallasizey>=16){  bufferscreen.drawString(t,1,1,2);  }
            else { bufferscreen.drawString(t,1,1,1);  }

            if(pinta){ bufferscreen.pushSprite(0,0); }
    
            if(siguiente){ return;}
     
            tfttopantalla();
            actualizarleds();
            delay(1000);
            
        }

        
        int puntos1=numpixeles/2;
        int puntos2=numpixeles/2;

       bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
       if(pinta){ bufferscreen.pushSprite(0,0); }
       

        for(int j=0;j<100;j++){
            
            int ini=millis();

            for(int i=0;i<samples;i++){
                vReal[i] = analogRead(pinadc);
                vImag[i] = 0;
                delayMicroseconds(100);
                if(siguiente){ return;} 
            }

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();

            int x=pantallasizex-1;

            double contador1=0;
            double contador2=0;
                                    
            for(int i=(samples/2)-1;i>=0;i--){               

                if(x>=(pantallasizex/2)){ contador2+=vReal[i];}
                else{ contador1+=vReal[i];}
                x--;
                if(x<0){ break; }                
                if(siguiente){ return;} 
            }


            if(contador1>contador2){ puntos1++;puntos2--;}
            else if(contador1<contador2){ puntos2++;puntos1--;}
           
            x=0;
            int y=pantallasizey-1;
            int ay=-1;
            color2=rgb565(0,0,255);  

            for(int i=0;i<puntos1;i++){
                  bufferscreen.fillRect(x,y,1,1,color2);
                  y+=ay;
                  if((y>=pantallasizey)||(y<0)){ ay=-ay;y+=ay;x++;}
            }

            
            x=pantallasizex-1;
            y=pantallasizey-1;
            ay=-1;
            color2=rgb565(255,0,0);  

            for(int i=0;i<puntos2;i++){
                  bufferscreen.fillRect(x,y,1,1,color2);
                  y+=ay;
                  if((y>=pantallasizey)||(y<0)){ ay=-ay;y+=ay;x--;}
            }

            
            if(pinta){ bufferscreen.pushSprite(0,0); }
            if(siguiente){ return;}
    
            tfttopantalla();
            actualizarleds();
            
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
               
        }

        String t="";

        int r=random(0,255);
        int g=random(0,255);
        int b=random(0,255);
        
        if(puntos1==puntos2) {  t="TIE"; }
        else if(puntos1>puntos2) {  t="PLAYER 1 WIN";r=0;g=0;b=255; }
        else { t="PLAYER 2 WIN";r=255;g=0;b=0; }
                     
                    
        color2=rgb565(r,g,b);            
        muestratexto(t,1,color2);    
               
    }
  
}


void explosion(){
    
    uint16_t color2=rgb565(100,100,100);
    
    for(int i=0;i<3;i++){

        bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
        tfttopantalla();actualizarleds();delay(200);

        bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);
        tfttopantalla();actualizarleds();delay(200);
      
    }
  
}

void pong(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);  }

    strip.clear();

    color2=rgb565(100,100,100); 

    for(int jj=0;jj<loopss[numpantalla];jj++){

        bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
        if(pinta){ bufferscreen.pushSprite(0,0); }

           
        for(int i=5;i>0;i--){
    
            bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
            if(pinta){ bufferscreen.pushSprite(0,0); }
            
            bufferscreen.setTextColor(color2);
            String t="";t+=i;
            if(pantallasizey>=16){  bufferscreen.drawString(t,1,1,2);  }
            else { bufferscreen.drawString(t,1,1,1);  }

            if(pinta){ bufferscreen.pushSprite(0,0); }
    
            if(siguiente){ return;}
     
            tfttopantalla();
            actualizarleds();
            delay(1000);
            
       }

        
       int puntos1=0;
       int puntos2=0;

       bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
       if(pinta){ bufferscreen.pushSprite(0,0); }

       int py=2;

       int boty=2;
        
       int bx=0;
       int by=2;
       int ax=1;
       int ay=1;

       for(int j=0;j<200;j++){
            
            int ini=millis();

            bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
             
            int maximo=0;
            for(int i=0;i<samples;i++){
                int val = analogRead(pinadc);
                if(val>maximo) { maximo=val; }
                delayMicroseconds(10);
                if(siguiente){ return;} 
            }

            //Serial.println(maximo);
            if(maximo>1600){ 
                py++; 
                if(py>=pantallasizey){ py=-2; }
            }

            boty+=random(-1,2);

            if(boty<0) { boty=0;}
            else if(boty>pantallasizey-3) { boty=pantallasizey-3;}
            

            bufferscreen.fillRect(0,py,1,3,color2);  ///Pintamos jugador

            bufferscreen.fillRect(pantallasizey-1,boty,1,3,color2);  ///Pintamos bot

            ///Pintamos la pelota

            bufferscreen.fillRect(bx,by,2,2,color2);

            by+=ay;

            if(by<0) { ay=-ay; } 
            else if(by+1>=(pantallasizey-1)) { ay=-ay; } 

            bx+=ax;     

            
            if(bx+2<0) { ax=-ax; puntos2++;explosion();bx=1;} 
            else if(bx-2>=(pantallasizex-1)) { ax=-ax; puntos1++;explosion();bx=pantallasizex-2;} 
                            
            if((bx+1)<=1) {   ///Rebote con player         
              
                if((by>=py)&&(by<=py+2)){ ax=-ax; bx=1; }                              
                if(((by+1)>=py)&&((by+1)<=py+2)){ ax=-ax; bx=1; }                              
                
            }

            if(bx>=pantallasizex-2) {   ///Rebote con bot         
                            
                if((by>=boty)&&(by<=boty+2)){ ax=-ax; bx=pantallasizex-2; }                              
                if(((by+1)>=boty)&&((by+1)<=boty+2)){ ax=-ax; bx=pantallasizex-2; }                              
                
            }
            
                                  
            if(pinta){ bufferscreen.pushSprite(0,0); }
            if(siguiente){ return;}
    
            tfttopantalla();
            actualizarleds();
            
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
               
        }

        String t="";

        if(puntos1==puntos2) {  t="TIE"; }
        else if(puntos1>puntos2) {  t="PLAYER WIN"; }
        else { t="BOT WIN"; }
                           
        muestratexto(t,1,color2);    
               
    }
  
}

void runescena(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t samples = 64;

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);  }

    int r=random(0,255);
    int g=random(0,255);
    int b=random(0,255);
    
    color2=rgb565(r,g,b);
    bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);

    if(pinta){ bufferscreen.pushSprite(0,0); }

    strip.clear();
    
    for(int jj=0;jj<loopss[numpantalla];jj++){

        for(int j=0;j<100;j++){

            int ini=millis();
            
            for(int i=numpixeles-1;i>0;i--){
                 uint32_t color=strip.getPixelColor(i-1);                  
                 strip.setPixelColor(i,color);                      
            }            

            int maximo=0;
            for(int i=0;i<samples;i++){
                int val = analogRead(pinadc);
                if(val>maximo) { maximo=val; }
                delayMicroseconds(10);
                if(siguiente){ return;} 
            }

            //Serial.println(maximo);
            if(maximo>1600){
                r=random(0,255);  g=random(0,255);  b=random(0,255);
                uint32_t color=strip.Color(r,b,g); strip.setPixelColor(3,color);              
                color=strip.Color(r/2,b/2,g/2); strip.setPixelColor(2,color);              
                color=strip.Color(r/4,b/4,g/4); strip.setPixelColor(1,color);              
                color=strip.Color(r/8,b/8,g/8); strip.setPixelColor(0,color);              
            }else{
                strip.setPixelColor(0,strip.Color(0,0,0));                                      
            }

            strip.show();
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
               
        }        
    }
  
}

void runmescena(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    uint16_t samples = 64;

    uint16_t color2=rgb565(255,255,255);

    if(pinta){  tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);  }

    color2=rgb565(0,0,0);
    bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);

    if(pinta){ bufferscreen.pushSprite(0,0); }

    strip.clear();
    
    for(int jj=0;jj<loopss[numpantalla];jj++){

        for(int j=0;j<100;j++){

            int ini=millis();

            int x=pantallasizex-1;
            int y=pantallasizey-1;

            int matriz[pantallasizex][pantallasizey];

            for(int iii=0;iii<pantallasizex;iii++){
                for(int jjj=0;jjj<pantallasizey;jjj++){
                    matriz[iii][jjj]=0;
                }
            }
            
            int direccion=0; 

            int num=0;
            while(num<(numpixeles-1)){

                  if(direccion==0){
                      if((matriz[x][y-1]==0)&&((y-1)>=0)){

                          uint16_t color=bufferscreen.readPixel(x,y-1); 
                          bufferscreen.fillRect(x,y,1,1,color);
                          matriz[x][y]=1;
                          y--;                      
                          num++;
                      }else{
                          direccion=1;
                      }
                  }else if(direccion==1){
                      if((matriz[x-1][y]==0)&&((x-1)>=0)){
                          uint16_t color=bufferscreen.readPixel(x-1,y); 
                          bufferscreen.fillRect(x,y,1,1,color);
                          matriz[x][y]=1;            
                          x--;          
                          num++;
                      }else{
                          direccion=2;
                      }
                  }else if(direccion==2){
                      if((matriz[x][y+1]==0)&&((y+1)<pantallasizey)){
                          uint16_t color=bufferscreen.readPixel(x,y+1); 
                          bufferscreen.fillRect(x,y,1,1,color);
                          matriz[x][y]=1;            
                          y++;          
                          num++;
                      }else{
                          direccion=3;
                      }
                  }else if(direccion==3){
                      if((matriz[x+1][y]==0)&&((x+1)<pantallasizex)){
                          uint16_t color=bufferscreen.readPixel(x+1,y); 
                          bufferscreen.fillRect(x,y,1,1,color);
                          matriz[x][y]=1;            
                          x++;          
                          num++;
                      }else{
                          direccion=0;
                      }
                  }                                
            }
            
            /*
            for(int i=numpixeles-1;i>0;i--){
                 uint32_t color=strip.getPixelColor(i-1);                  
                 strip.setPixelColor(i,color);                      
            }            
            */
            
            int maximo=0;
            for(int i=0;i<samples;i++){
                int val = analogRead(pinadc);
                if(val>maximo) { maximo=val; }
                delayMicroseconds(10);
                if(siguiente){ return;} 
            }

            Serial.println(maximo);
            

            //maximo=random(0,1800);

            
            if(maximo>1600){
              
                int r=random(0,255);  int g=random(0,255);  int b=random(0,255);

                uint16_t color=rgb565(r,g,b);
                bufferscreen.fillRect((spritesizex/2)-1,(spritesizey/2),1,1,color);
                
                color=rgb565(r/2,g/2,b/2);
                bufferscreen.fillRect((spritesizex/2)-1,(spritesizey/2)-1,1,1,color);
                
                color=rgb565(r/4,g/4,b/4);
                bufferscreen.fillRect(spritesizex/2,(spritesizey/2)-1,1,1,color);

                color=rgb565(r/8,g/8,b/8);
                bufferscreen.fillRect(spritesizex/2,spritesizey/2,1,1,color);
                
                
            }else{
                
                bufferscreen.fillRect((spritesizex/2)-1,(spritesizey/2),1,1,0);               
                
            }


            if(pinta){ bufferscreen.pushSprite(0,0); }
            tfttopantalla();
            actualizarleds();
            
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
               
        }        
    }
  
}

void matrix(){

    int tiempo=(10000)/velocidad[numpantalla];
    if(tiempo>1000){ tiempo=1000; }

    float posy[spritesizex];
    float vy[spritesizex];
    int longitud[spritesizex];
    
    for(int i=0;i<spritesizex;i++){ 
      posy[i]=(float)random(0,spritesizex); 
      vy[i]=(float)(random(10,40)/10); 
      longitud[i]=random(2,20);
    }

    uint16_t color2=rgb565(255,255,255);

    if(pinta){
      tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);
    }

    contadorframe=0;
       
    for(int jj=0;jj<loopss[numpantalla];jj++){

        for(int j=0;j<100;j++){

            int ini=millis();
            
            float r=rojo[numpantalla]*0.5;
            float g=verde[numpantalla]*0.5;
            float b=azul[numpantalla]*0.5;
              
            color2=rgb565((int)r,(int)g,(int)b);
            bufferscreen.fillRect(0,0,spritesizex,spritesizey,color2);

            int x=140;
            int y=0;
      
            for(int ii=0;ii<spritesizex;ii++){  
              
              posy[ii]+=vy[ii];
              if(posy[ii]>=spritesizey) {  
                  posy[ii]=0; 
                  vy[ii]=(float)(random(10,40)/10);  
                  longitud[ii]=random(2,20);
              }              

              float porcentaje=1.0+((float)tolerancia[numpantalla]/100.0);
              
              for(int iii=0;iii<longitud[ii];iii++){
                
                  float r=rojo[numpantalla]*porcentaje;   if(r>255) { r=255; }
                  float g=verde[numpantalla]*porcentaje;  if(g>255) { g=255; }
                  float b=azul[numpantalla]*porcentaje;   if(b>255) { b=255; }
              
                  color2=rgb565((int)r,(int)g,(int)b);

                  int py=((int)posy[ii])-iii;

                  if(py<0){ py=spritesizey+py; }
                  
                  bufferscreen.fillRect(ii,py,1,1,color2);                              
                  
                  porcentaje-=(((float)tolerancia[numpantalla]/50.0)/(float)longitud[ii]);if(porcentaje<0) { porcentaje=0; }

                  if(siguiente){ return;} 
                  
              }
              
              x+=5;
              
            }

            if(pinta){ bufferscreen.pushSprite(0,0); }
            tfttopantalla();

             if((contadorframe==0)&&(numpantalla!=0)){            
                if(transicion[numpantalla-1]==1){ //FADE TO BLACK  
                    fadetoblack2();    
                }else if(transicion[numpantalla-1]==2){ //MOVE FORDWARD
                    moveforward2();    
                }else if(transicion[numpantalla-1]==3){ //MOVE BACK
                    moveback2();    
                }else if(transicion[numpantalla-1]==4){ //LINES
                    lines2();    
                }                    
            }

            contadorframe++;
    
            actualizarleds();
  
            while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    
               
        }        
    }
  
}

void muestratexto(String t,int loop,uint16_t color){

  uint16_t color2=rgb565(255,255,255);
  if(pinta){
      tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);
  }
  
  int len=t.length()+2;

  int tiempo=(10000)/velocidad[numpantalla];
  if(tiempo>1000){ tiempo=1000; }

  //Serial.print("Tiempo : ");Serial.println(tiempo);
  
  int yy=0;
  
  for(int jj=0;jj<loop;jj++){
  
    int px=spritesizex+10;
  
    for(int ii=0;ii<(len*8);ii++){

      int ini=millis();
  
      bufferscreen.fillRect(0,0,spritesizex,spritesizey,0);
            
      if(pinta){
          color2=rgb565(255,255,255);
          tft.fillRect(0,0,spritesizex+1,spritesizey+1,color2);
          tft.fillRect(0,0,spritesizex,spritesizey,0);  
      }

      bufferscreen.setTextColor(color);
      if(pantallasizey>=16){  bufferscreen.drawString(t,px,yy+1,2);  }
      else { bufferscreen.drawString(t,px,yy+1,1);  }

      if(pinta){ bufferscreen.pushSprite(0,0); }
    
      px--;    
     
      if(siguiente){ return;}
     
      tfttopantalla();
      actualizarleds();

      while((millis()-ini)<tiempo) { if(siguiente){ return;} delay(1);wdt(); }                    

    
    }

    if(siguiente){ return;}
    
  }
  
}

void GetRGBFrom16Bit(const uint16_t color) {

    double pr = ((color >> 11) & 0x1F) / 31.0; // red   0.0 .. 1.0
    double pg = ((color >> 5) & 0x3F) / 63.0;  // green 0.0 .. 1.0
    double pb = (color & 0x1F) / 31.0;         // blue  0.0 .. 1.0
    
    R=(int)(255*pr);
    G=(int)(255*pg);
    B=(int)(255*pb);

    /*
    if(G>(R+5)){
        Serial.print("G : ");Serial.print(G);
        Serial.print(", R : ");Serial.println(R);    
        delay(50);            
    }
    */
    
}


uint16_t rgb565(int r, int g, int b) {

  int r565 = map(r,0,255,0,31);
  int g565 = map(g,0,255,0,63);
  int b565 = map(b,0,255,0,31);


  uint16_t rgb_565 = (r565 << 11) | (g565 << 5) | b565;

  // Serial.println(r565);

  return (rgb_565);

}


int getGifInventory(String basePath) {

  
  int amount = 0;
  if(sdcard){  GifRootFolder = SD.open(basePath); }
  else { GifRootFolder = SPIFFS.open(basePath); }
  
  if(!GifRootFolder){  Serial.println("Failed to open directory"); return 0; }
  if(!GifRootFolder.isDirectory()){ Serial.println("Not a directory"); return 0; }

  File file = GifRootFolder.openNextFile();

  while( file ) {
    if(!file.isDirectory()) {
      String archivo=file.name();

      if(archivo.indexOf(".gif")!=-1){
          gifs[amount]=file.name();      
          amount++;      
      }
      
      file.close();
    }
    file = GifRootFolder.openNextFile();
  }
  
  GifRootFolder.close();
  Serial.print("Encontrados ");Serial.print(amount);Serial.println(" GIFs");
  
  return amount;
  
}




int deleteGif(int pos) {

    String archivo="";
    
    if(gifs[pos].indexOf("/")==-1){ archivo+="/"; }
    
    archivo+=gifs[pos];      

    Serial.println("Vamos a borrar");
    
    if(sdcard){ SD.remove(archivo); }
    else{ SPIFFS.remove(archivo); } 
          
    Serial.println("Se ha borrado");
    
}

void wdt(){
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    yield(); 
}

void cerrarcliente(){
  client.flush();
  client.stop();   
//  client.stopAll();     
}

void cerrarconexiones(){
  bool cliente=true;

  while(cliente) {    
      delay(100);  
      client=server.available();
      cliente=false;
      
      if(client){
        if(client.connected()){
          if(client.available()){ 
              cliente=true;             
              cerrarcliente();             
          } 
        }
      }
  } 
  
}

void webprincipal(){

     

      String s=F("<html><head><meta http-equiv=\"Content-Type\" content=\"text/html; charset=windows-1252\">\n<script>\n\naddEventListener(\"resize\",actualizatamano);\naddEventListener(\"keypress\", function(event) {  \n  if (event.key === \"Enter\") {    \nevent.preventDefault();\n    play(0);\n  }\n});\n\nvar numefectos=0;\n\nvar tamano=60;\n\nvar cadena=\"");client.print(s);
     
      s=cadena;client.print(s);
      
      s=F("\";\n\nfunction get(peticion){\n\nip=location.host;\n\nif(window.XMLHttpRequest){xmlhttp=new XMLHttpRequest();}\nelse{xmlhttp=new ActiveXObject(\"Microsoft.XMLHTTP\");}\n\nxmlhttp.onreadystatechange=function(){\nif(xmlhttp.responseText!=\"\" && xmlhttp.readyState==4 && xmlhttp.status==200){\ndatorecogido=xmlhttp.responseText;\n}else if(xmlhttp.status==404){alert(\"FALLO COMUNICACION !!\");}\n}\n\n//alert(peticion);\n\nxmlhttp.open(\"GET\",\"http://\"+ip+\"/\"+peticion,true);\nxmlhttp.send();\n\n\n}\n\n\n\nfunction play(num){\n\n//alert(\"Se van a enviar los datos !!!\");\n\nvar datos=\"@\"+numefectos+\";\"+num+\";\";\n\nfor(var i=1;i<=numefectos;i++){\n\nif(i!=1){\ndatos+=document.getElementById(\"transicion\"+(i-1)).value+\";\";\n}\n\ndatos+=document.getElementById(\"seleccion\"+i).value+\";\";\n\nif(document.getElementById(\"gifs\"+i)!=null){\ndatos+=document.getElementById(\"gifs\"+i).value+\";\";\n}else{\ndatos+=\"0;\";\n}\n\nif(document.getElementById(\"texto\"+i)!=null){\ndatos+=document.getElementById(\"texto\"+i).value+\";\";\n}else{\ndatos+=\"0;\";\n}\n\ndatos+=document.getElementById(\"tvelocidad\"+i).value+\";\";\ndatos+=document.getElementById(\"tloops\"+i).value+\";\";\ndatos+=document.getElementById(\"ttolerancia\"+i).value+\";\";\ndatos+=document.getElementById(\"tbrillo\"+i).value+\";\";\ndatos+=document.getElementById(\"tsaturacion\"+i).value+\";\";\ndatos+=document.getElementById(\"trojo\"+i).value+\";\";\ndatos+=document.getElementById(\"tverde\"+i).value+\";\";\ndatos+=document.getElementById(\"tazul\"+i).value+\";\";\n\n}\n\nget(datos);\n\n}\n\nfunction elementFromHtml(html){\n\nconst template=document.createElement(\"template\");\ntemplate.innerHTML=html.trim();\nreturn templa");client.print(s);
      s=F("te.content;\n\n}\n\n\nfunction crearselecgifs(num){\n\nconst codigo=elementFromHtml(\"<select id='gifs\"+num+\"' style='padding:5px;background:#edf2ff;width:200px'>");client.print(s);
           
      numerogifs = getGifInventory(carpeta); 
      s="";
      for(int i=0;i<numerogifs;i++){
          s+="<option value='"; s+=i; s+="'>";
          s+=gifs[i];        
          s+="</option>";
      }
      client.print(s);   
      
      s=F("</select>\");\n\ndocument.getElementById(\"imagentexto\"+num).appendChild(codigo);\n}\n\n\nfunction eliminarefecto(num){\n\n\n\nfor(i=num;i<numefectos;i++){\n\nif(i==numefectos){ break; }\n\n\nif(i!=1){document.getElementById(\"transicion\"+(i-1)).value=document.getElementById(\"transicion\"+i).value;  }\n\ndocument.getElementById(\"seleccion\"+i).value=document.getElementById(\"seleccion\"+(i+1)).value;\n\n\nactivardesactivar(i);\n\n\nvar imagen=document.getElementById(\"gifs\"+(i+1));\nvar texto=document.getElementById(\"texto\"+(i+1));\n\nif(imagen!=null) { document.getElementById(\"gifs\"+i).value=imagen.value; }\n\nif(texto!=null) {document.getElementById(\"texto\"+i).value=texto.value; }\n\ndocument.getElementById(\"velocidad\"+i).value=document.getElementById(\"velocidad\"+(i+1)).value;\ndocument.getElementById(\"tvelocidad\"+i).value=document.getElementById(\"tvelocidad\"+(i+1)).value;\n\ndocument.getElementById(\"loops\"+i).value=document.getElementById(\"loops\"+(i+1)).value;\ndocument.getElementById(\"tloops\"+i).value=document.getElementById(\"tloops\"+(i+1)).value;\n\ndocument.getElementById(\"tolerancia\"+i).value=document.getElementById(\"tolerancia\"+(i+1)).value;\ndocument.getElementById(\"ttolerancia\"+i).value=document.getElementById(\"ttolerancia\"+(i+1)).value;\n\ndocument.getElementById(\"brillo\"+i).value=document.getElementById(\"brillo\"+(i+1)).value;\ndocument.getElementById(\"tbrillo\"+i).value=document.getElementById(\"tbrillo\"+(i+1)).value;\n\ndocument.getElementById(\"saturacion\"+i).value=document.getElementById(\"saturacion\"+(i+1)).value;\ndocument.getElementById(\"tsaturacion\"+i).value=document.getElementById(\"tsaturacion\"+(i+1)).value;\n\ndocument.getElementById(\"rojo\"+i).value=document.getElementById(\"rojo\"+(i+1)).value;\ndocument.getElementById(\"trojo\"+i).value=document.getElementById(\"trojo\"+(i+1))");client.print(s);
      s=F(".value;\n\ndocument.getElementById(\"verde\"+i).value=document.getElementById(\"verde\"+(i+1)).value;\ndocument.getElementById(\"tverde\"+i).value=document.getElementById(\"tverde\"+(i+1)).value;\n\ndocument.getElementById(\"azul\"+i).value=document.getElementById(\"azul\"+(i+1)).value;\ndocument.getElementById(\"tazul\"+i).value=document.getElementById(\"tazul\"+(i+1)).value;\n\n\nactualizavalor(\"azul\",i,0);\n\n\n}\n\n\n\n\n\n\nvar efecto=document.getElementById(\"dive\"+numefectos);\nefecto.remove();\n\nnumefectos--;\n\nif(numefectos!=0){\nvar tran=document.getElementById(\"divt\"+numefectos);\ntran.remove();\n}\n\n\n\n\n}\n\n\n\nfunction creartransicion() {\n\n\nconst codigo=elementFromHtml(\"<div id='divt\"+numefectos+\"'><br><table width=40% style='border-radius:20px;background-color:#FAFAFF;cursor:pointer;'><tr align=center><td width=20%></td><td width=30% style='font-size:170%;font-weight:bold'>TRANSITION \"+numefectos+\"</td><td width=50%><select id='transicion\"+numefectos+\"' style='padding:5px;background:#edf2ff'><option value='0'>NONE</option><option value='1'>FADE TO BLACK</option><option value='2'>MOVE FORWARD</option><option value='3'>MOVE BACK</option><option value='4'>LINES</option></select><td><tr></table><div>\");\n\n\ndocument.getElementById(\"efectos\").appendChild(codigo);\n\n\n        \n}\n\nfunction actualizatamano(){\n\n//alert(window.innerHeight+\" - \"+window.innerWidth);\n\nif(window.innerWidth<=1600) { tamano=100; }\nelse{ tamano=(1600*100)/window.innerWidth; }\n\n\nfor(i=1;i<=numefectos;i++){\nif(document.getElementById(\"tablaefecto\"+i)!=null){\ndocument.getElementById(\"tablaefecto\"+i).width=tamano+\"%\";\n}\n}\n\n        document.getElementById(\"tablaplay\").width=tamano+\"%\";\n\n}\n\nfunction crearefecto(num) {\n\nif(num==-1) { \nnum=numefectos; \n}\n\nif(num==numefectos){\nif(numefectos!=0){ creartransicion(); }\nnumefectos++;\nnum=numefectos; \n}\n\nconst codigo=elementFromHtml(\"<div id='dive\"+num+\"'><br><table id='tablaefecto\"+num+\"' width='60%' style='border-radius:20px;background-color:#FAFAFF;padding:20px'><tr align='left'><td witdh='10%' st");client.print(s);
      s=F("yle='font-size:170%;font-weight:bold;cursor:pointer;' onclick='play(\"+(num-1)+\")'>SCREEN \"+num+\"</td><td witdh='20%'><select id='seleccion\"+num+\"' onchange='activardesactivar(\"+num+\")' style='padding:5px;background:#edf2ff'><option value='0'></option><option value='1'>GIF PLAYER</option><option value='2'>TEXT</option><option value='3'>DATE</option><option value='4'>TIME</option><option value='5'>MATRIX</option><option value='6'>SPECTROMETER (M)</option><option value='7'>SPECTROMETER 2 (M)</option><option value='8'>RUN (M)</option><option value='9'>RUNM (M)</option><option value='10'>BATTLE (M)</option><option value='11'>PONG (M)</option><option value='12'>TEST</option><option value='13'>COLOR</option></select></td><td witdh='30%' align=center><div id='imagentexto\"+num+\"' width=100%><input type='text' id='texto\"+num+\"' size='30' disabled></div><br><br><table style='background-color:#fff;border:1px solid black;border-collapse:collapse' width=25% id='color\"+num+\"'><tr><td><br><br><br></td></tr></table></td><td witdh='35%'><table width=100%><tr><td width=45%>SPEED (%):</td><td width=10%><input type='text' id='tvelocidad\"+num+\"' size=2 value='100' onchange='actualizavalor(\\\"velocidad\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='velocidad\"+num+\"' value='100' min='0' max='200' onchange='actualizavalor(\\\"velocidad\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>LOOPS (N):</td><td width=10%><input type='text' id='tloops\"+num+\"' size=2 value='1' onchange='actualizavalor(\\\"loops\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='loops\"+num+\"' value='1' min='1' max='10' onchange='actualizavalor(\\\"loops\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>TOLERANCE (M):</td><td width=10%><input type='text' id='ttolerancia\"+num+\"' size=2 value='0' onchange='actualizavalor(\\\"tolerancia\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='tolerancia\"+num+\"' value='0' min='0' max='100' onchange='");client.print(s);
      s=F("actualizavalor(\\\"tolerancia\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>BRIGHTNESS (%):</td><td width=10%><input type='text' id='tbrillo\"+num+\"' size=2 value='50' onchange='actualizavalor(\\\"brillo\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='brillo\"+num+\"' value='50' min='0' max='100' onchange='actualizavalor(\\\"brillo\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>SATURATION (%):</td><td width=10%><input type='text' id='tsaturacion\"+num+\"' size=2 value='50' onchange='actualizavalor(\\\"saturacion\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='saturacion\"+num+\"' value='50' min='0' max='100' onchange='actualizavalor(\\\"saturacion\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>RED (0-255):</td><td width=10%><input type='text' id='trojo\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"rojo\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='rojo\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"rojo\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>GREEN (0-255):</td><td width=10%><input type='text' id='tverde\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"verde\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='verde\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"verde\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>BLUE (0-255):</td><td width=10%><input type='text' id='tazul\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"azul\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='azul\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"azul\\\",\"+num+\",1)' disabled></td></tr></table></td><td witdh='5%' onclick='eliminarefecto(\"+num+\")' valign='top' style='cursor:pointer;'><table style='background-color: lightgrey;'><tr><td style='background-color:white;font-family: Arial Rounded MT Bold'><b>&nbsp;X&nbsp;</b></td></tr></table></td></tr></table><div>\"");client.print(s);
      s=F(");\n\n\ndocument.getElementById(\"efectos\").appendChild(codigo);\n\nactualizatamano();\nactualizavalor(\"verde\",num,1);\n}\n\n\nfunction establecevalor(dato,num,valor){\n\ndocument.getElementById(dato+num).value=valor;\ndocument.getElementById(\"t\"+dato+num).value=valor;\n\n}\n\nfunction deshabilita(dato,num,deshabilitar){\n\ndocument.getElementById(dato+num).disabled = deshabilitar;\ndocument.getElementById(\"t\"+dato+num).disabled = deshabilitar;\n\n}\n\nfunction activardesactivar(num){\n\nvar opcion=document.getElementById(\"seleccion\"+num).value;\n\nvar imagentexto = document.getElementById(\"imagentexto\"+num);\n\nvar imagen = document.getElementById(\"gifs\"+num);\nvar texto = document.getElementById(\"texto\"+num);\n\nif(imagen!=null){ imagentexto.removeChild(imagen); }\nif(texto!=null){ imagentexto.removeChild(texto); }\n\nif(opcion==0){\n\nvar input = document.createElement(\"input\");input.setAttribute(\"type\",\"text\");input.setAttribute(\"id\",\"texto\"+num);input.setAttribute(\"size\",\"30\");input.setAttribute(\"disabled\",\"\");\nimagentexto.appendChild(input);\n\ndeshabilita(\"velocidad\",num,true);\ndeshabilita(\"loops\",num,true);\ndeshabilita(\"tolerancia\",num,true);\ndeshabilita(\"brillo\",num,true);\ndeshabilita(\"saturacion\",num,true);\ndeshabilita(\"rojo\",num,true);\ndeshabilita(\"verde\",num,true);\ndeshabilita(\"azul\",num,true);\n\nestablecevalor(\"rojo\",num,255);\nestablecevalor(\"verde\",num,255);\nestablecevalor(\"azul\",num,255);\nactualizavalor(\"verde\",num,1);\n\n}else if(opcion==1){ //GIF\n\ncrearselecgifs(num);\n\ndeshabilita(\"velocidad\",num,false);\ndeshabilita(\"loops\",num,false);\ndeshabilita(\"tolerancia\",num,false);\ndeshabilita(\"brillo\",num,false);\ndeshabilita(\"saturacion\",num,false);\ndeshabilita(\"rojo\",num,true);\ndeshabilita(\"verde\",num,true);\ndeshabilita(\"azul\",num,true);\n\n}else if((opcion==2)||(opcion==3)||(opcion==4)||(opcion==5)){ //TEXTO , FECHA , HORA \n\nvar input = document.createElement('input');input.setAttribute('type',\"text\");input.setAttribute('id',\"texto\"+num);input.setAttribute('size',\"30\");\nimagentexto.appendChild(input);\n");client.print(s);
      s=F("\ndeshabilita(\"velocidad\",num,false);\ndeshabilita(\"loops\",num,false);\ndeshabilita(\"brillo\",num,true);\ndeshabilita(\"saturacion\",num,true);\ndeshabilita(\"rojo\",num,false);\ndeshabilita(\"verde\",num,false);\ndeshabilita(\"azul\",num,false);\n\nif((opcion==3)||(opcion==4)){\n\ndeshabilita(\"tolerancia\",num,true);\n\nvar currentdate = new Date();\nvar datetime = \"(\" + currentdate.getDate() + \"/\" +(currentdate.getMonth()+1)+ \"/\" + currentdate.getFullYear() + \")(\" + currentdate.getHours() + \":\" + currentdate.getMinutes() + \":\" + currentdate.getSeconds()+\")\";\n\ndocument.getElementById(\"texto\"+num).value=datetime;\n\n\n}else if(opcion==5){\n\ndeshabilita(\"tolerancia\",num,false);\n\nestablecevalor(\"tolerancia\",num,50);\n\nestablecevalor(\"rojo\",num,0);\nestablecevalor(\"verde\",num,128);\nestablecevalor(\"azul\",num,0);\nactualizavalor(\"verde\",num,1);\n\n}\n\n}else if((opcion>=6)&&(opcion<=11)){\n\nvar input = document.createElement('input');input.setAttribute('type',\"text\");input.setAttribute('id',\"texto\"+num);input.setAttribute('size',\"30\");\nimagentexto.appendChild(input);\n\ndeshabilita(\"velocidad\",num,false);\ndeshabilita(\"loops\",num,false);\ndeshabilita(\"tolerancia\",num,false);\ndeshabilita(\"brillo\",num,false);\ndeshabilita(\"saturacion\",num,false);\ndeshabilita(\"rojo\",num,true);\ndeshabilita(\"verde\",num,true);\ndeshabilita(\"azul\",num,true);\n\n}else if((opcion>=12)&&(opcion<=13)){\n\nvar input = document.createElement('input');input.setAttribute('type',\"text\");input.setAttribute('id',\"texto\"+num);input.setAttribute('size',\"30\");\nimagentexto.appendChild(input);\n\ndeshabilita(\"velocidad\",num,false);\ndeshabilita(\"loops\",num,false);\ndeshabilita(\"tolerancia\",num,true);\ndeshabilita(\"brillo\",num,true);\ndeshabilita(\"saturacion\",num,true);\ndeshabilita(\"rojo\",num,false);\ndeshabilita(\"verde\",num,false);\ndeshabilita(\"azul\",num,false);\n}\n\n}\n\nfunction actualizavalor(dato,num,tipo){\nif(tipo==1){ document.getElementById(\"t\"+dato+num).value=document.getElementById(dato+num).value; }\nelse{ document.getElementById(dato+num).value=document.g");client.print(s);
      s=F("etElementById(\"t\"+dato+num).value; }\n\nif((dato==\"rojo\")||(dato==\"verde\")||(dato==\"azul\")){\ndocument.getElementById(\"color\"+num).style=\"background-color:rgb(\"+document.getElementById(\"rojo\"+num).value+\",\"+document.getElementById(\"verde\"+num).value+\",\"+document.getElementById(\"azul\"+num).value+\");border:1px solid black;border-collapse:collapse;\";\n}\n}\n\n\n\nfunction inicializa(){\n\nif(cadena==\"\"){\n\ncrearefecto(-1);\n\n}else{\n   \n   cadena=cadena.replace(\"%20\",\" \");\n   \n   var pos1=-1;\n       \n       pos1=cadena.indexOf(\"@\",pos1+1); if(pos1==-1) { crearefecto(-1); }\n\n       var pos2=cadena.indexOf(\";\",pos1+1);\n\n   var d=cadena.substring(pos1+1,pos2);\n\n       var numefectos=parseInt(d); \n   \n   pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n       var numpantalla=parseInt(d); \n\n   var transicion=0;\n   var tipo=0;\n   var ngifs=0;\n   var texto=\"\";\n   var velocidad=1;\n   var loops=1;\n   var tolerancia=0;\n   var brillo=0;\n   var saturacion=0;\n   var rojo=0;\n   var verde=0;\n   var azul=0;\n          \n       for(var i=0;i<numefectos;i++){\n\n  crearefecto(i);\n\n  if(i!=0) { \n pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n transicion=parseInt(d); \n  }\n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  tipo=parseInt(d); \n  \n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  ngifs=parseInt(d); \n  \n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  d.replace(\"%20\",\" \");texto=d; \n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  velocidad=parseInt(d); if(velocidad==0) { velocidad=1; }\n  \n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  loops=parseInt(d); \n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  tolerancia=parseInt(d); \n   \n  pos1=pos2;pos");client.print(s);
      s=F("2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  brillo=parseInt(d); \n   \n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  saturacion=parseInt(d); \n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  rojo=parseInt(d); \n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  verde=parseInt(d); \n\n  pos1=pos2;pos2=cadena.indexOf(\";\",pos1+1);d = cadena.substring(pos1+1,pos2);\n  azul=parseInt(d);          \n  \n\n//  alert(transicion+\",\"+tipo+\",\"+ngifs+\",\"+texto+\",\"+velocidad+\",\"+loops+\",\"+tolerancia+\",\"+brillo+\",\"+saturacion+\",\"+rojo+\",\"+verde+\",\"+azul);\n\n\n  if(document.getElementById(\"transicion\"+i)!=null){ \ndocument.getElementById(\"transicion\"+i).value=transicion; \n  }\n\n  document.getElementById(\"seleccion\"+(i+1)).value=tipo; \n\n  activardesactivar(i+1);\n\n  if(document.getElementById(\"gifs\"+(i+1))!=null){\n  document.getElementById(\"gifs\"+(i+1)).value=ngifs;\n  }\n\n  if(document.getElementById(\"texto\"+(i+1))!=null){\n     document.getElementById(\"texto\"+(i+1)).value=texto;\n  }\n\n   document.getElementById(\"velocidad\"+(i+1)).value=velocidad;\n   document.getElementById(\"tvelocidad\"+(i+1)).value=velocidad;\n\n  document.getElementById(\"loops\"+(i+1)).value=loops;\n   document.getElementById(\"tloops\"+(i+1)).value=loops;\n\n  document.getElementById(\"tolerancia\"+(i+1)).value=tolerancia;\n   document.getElementById(\"ttolerancia\"+(i+1)).value=tolerancia;\n\n  document.getElementById(\"brillo\"+(i+1)).value=brillo;\n   document.getElementById(\"tbrillo\"+(i+1)).value=brillo;\n\n  document.getElementById(\"saturacion\"+(i+1)).value=saturacion;\n   document.getElementById(\"tsaturacion\"+(i+1)).value=saturacion;\n\n    document.getElementById(\"rojo\"+(i+1)).value=rojo;\n   document.getElementById(\"trojo\"+(i+1)).value=rojo;\n\n  document.getElementById(\"verde\"+(i+1)).value=verde;\n");client.print(s);
      s=F("  document.getElementById(\"tverde\"+(i+1)).value=verde;\n  \n  document.getElementById(\"azul\"+(i+1)).value=azul;\n  document.getElementById(\"tazul\"+(i+1)).value=azul;\n\n  actualizavalor(\"azul\",(i+1),0);\n\n  transicion=0;\n  tipo=0;\n  ngifs=0;\n  texto=\"\";\n  velocidad=1;\n  loopss=1;\n  tolerancia=0;\n  brillo=0;\n  saturacion=0;\n  rojo=0;\n  verde=0;\n  azul=0;\n       }\n\n}\n\n}\n\n</script>\n<style>\nbody{\nbackground-image: linear-gradient(to right, #0799bf, #d5e7FF);\nfont-family: \"Courier New\", monospace;\n}\nh1{\n  border-radius:20px;\n  background-image: linear-gradient(to right,#6666FF,#5555FF);\n  color: #0000FF;\n  font-size:300%;\n}\n\n</style>\n\n</head>\n\n<body onload=\"inicializa();\">\n<center>\n\n<table><tbody><tr><td><h1>&nbsp;&nbsp;&nbsp;TLEDS&nbsp;&nbsp;<a href=\"http://192.168.4.1/config\" style=\"text-decoration:none;cursor:pointer;color:#0000FF;\">&#10049;</a><a href=\"http://192.168.4.1/gifs\" style=\"text-decoration:none;cursor:pointer;color:#0000FF;\">&#10064;</a>&nbsp;&nbsp;</h1></td></tr></tbody></table>\n\n<div id=\"efectos\"></div>\n\n<br><br>\n\n<table id=\"tablaplay\" width=\"83.33333333333333%\" align=\"center\"><tbody><tr>\n<td width=\"15%\"><br></td>\n<td width=\"10%\"><table width=\"100%\" style=\"border-radius:20px;background-color:#FAFAFF;cursor:pointer;\" onclick=\"crearefecto(-1)\"><tbody><tr align=\"center\"><td width=\"100%\" style=\"font-size:600%\">+</td><td></td></tr><tr></tr></tbody></table></td>\n<td width=\"5%\"><br></td>\n<td width=\"10%\"><table width=\"10%\" style=\"border-radius:20px;background-color:#FAFAFF;cursor:pointer;\" onclick=\"play(0)\"><tbody><tr align=\"center\"><td width=\"100%\" style=\"font-size:600%\">PLAY</td><td></td></tr><tr></tr></tbody></table></td>\n\n</tr></tbody></table>\n\n</center>\n\n\n\n\n\n</body></html>");client.print(s);
      s=F("");



  
}


void webconfig(){

      
      String s=F("<html><head><style>body{background-image: linear-gradient(to right, #0799bf, #d5e7FF);font-family:'Courier New',monospace;}</style><script>addEventListener(\"keypress\",function(event){if (event.key===\"Enter\"){event.preventDefault();salvar();}});function salvar(){ var datos=\"?\";datos+=document.getElementById(\"ledsx\").value+\";\";datos+=document.getElementById(\"ledsy\").value+\";\";datos+=document.getElementById(\"firstpixel\").value+\";\";if(document.getElementById(\"serpentine\").checked){datos+=\"1;\";}else{datos+=\"0;\";}if(document.getElementById(\"rotate\").checked){datos+=\"1;\";}else{datos+=\"0;\";}datos+=document.getElementById(\"pindata\").value+\";\";datos+=document.getElementById(\"pinadc\").value+\";\";window.location.href=\"./\"+datos;}</script></head><body><center><table style='border-radius:20px;background-color:#FAFAFF;padding:20px'>");client.print(s);

      s=F("<tr><td><br></td></tr>");client.print(s);
      s=F("<tr><td><h1><a href='/' style='text-decoration:none;cursor:pointer;color:black;'>&#10094;</a>&nbsp;&nbsp;&nbsp;LED MATRIX&nbsp;&nbsp;</h1></td></tr>");client.print(s);
      s=F("<tr><td><br></td></tr>");client.print(s);
      s+="<tr><td align=center><b>W :&nbsp;&nbsp;</b><input type='text' id='ledsx' size='5' value='";
      s+=pantallasizex;
      s+="'/></td></tr>";      
      s+="<tr><td align=center><b>H :&nbsp;&nbsp;</b><input type='text' id='ledsy' size='5' value='";
      s+=pantallasizey;
      s+="'/></td></tr>";
      client.print(s);

      s="<tr><td><br></td></tr>";

      s+="<tr><td><b>FIRST PIXEL :&nbsp;&nbsp;</b><select id='firstpixel'>";

      if(esquina==0){   s+="<option value=0 selected>LEFT-UP</option>";  }
      else{   s+="<option value=0>LEFT-UP</option>";  }

      if(esquina==1){   s+="<option value=1 selected>LEFT-DOWN</option>";  }
      else{   s+="<option value=1>LEFT-DOWN</option>";  }

      if(esquina==2){   s+="<option value=2 selected>RIGHT-UP</option>";  }
      else{   s+="<option value=2>RIGHT-UP</option>";  }

      if(esquina==3){   s+="<option value=3 selected>RIGHT-DOWN</option>";  }
      else{   s+="<option value=3>RIGHT-DOWN</option>";   }

      s+="</select></td></tr>";client.print(s);
      
      s="<tr><td><br></td></tr>";

      if(serpentina==0){  s+="<tr><td align=center><b>SERPENTINE :&nbsp;&nbsp;</b><input type='checkbox' id='serpentine'/></td></tr>";  }
      else{   s+="<tr><td align=center><b>SERPENTINE :&nbsp;&nbsp;</b><input type='checkbox' id='serpentine' checked/></td></tr>"; }

      s+="<tr><td><br></td></tr>";
      
      if(rotar==0){  s+="<tr><td align=center><b>ROTATE :&nbsp;&nbsp;</b><input type='checkbox' id='rotate'/></td></tr>";  }
      else{   s+="<tr><td align=center><b>ROTATE :&nbsp;&nbsp;</b><input type='checkbox' id='rotate' checked/></td></tr>"; }
      
      client.print(s);

      s=F("<tr><td><br></td></tr>");client.print(s);
      s+="<tr><td align=center><b>PIN DATA :&nbsp;&nbsp;</b><input type='text' id='pindata' size='5' value='";
      s+=pindata;
      s+="'/></td></tr>";      
      s+="<tr><td align=center><b>PIN ADC :&nbsp;&nbsp;</b><input type='text' id='pinadc' size='5' value='";
      s+=pinadc;
      s+="'/></td></tr>";
      client.print(s);
      

      s=F("<tr><td><br><br></td></tr>");client.print(s);

      s=F("<tr><td align=center><table style='background-color:#FFFFFF;border:1px solid;border-collapse:collapse;font-size:2vw;font-family:\"Courier New\",monospace;cursor:pointer;' onclick='salvar();'><tr><td valign=middle><b>SAVE</b></td></tr></table></td></tr>");client.print(s);

      s=F("<tr><td><br></td></tr>");client.print(s);
             
      s=F("</table></center></body></html>");client.print(s);

   
}

void websubida(){

      
      String s=F("<html><style>body{background-image: linear-gradient(to right, #0799bf, #d5e7FF);font-family:'Courier New',monospace;}</style><body><center><table style='border-radius:20px;background-color:#FAFAFF;padding:20px'>");client.print(s);

      s=F("<tr><td colspan=2><h1><a href='/' style='text-decoration:none;cursor:pointer;color:black;'>&#10094;</a>&nbsp;&nbsp;&nbsp;GIFS UPLOAD</h1></td></tr>");
      
      numerogifs = getGifInventory(carpeta); 
            
      for(int i=0;i<numerogifs;i++){
          s+="<tr><td>&nbsp;&nbsp;&nbsp;&nbsp;";
          s+=gifs[i];        
          s+="&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td><td>&nbsp;&nbsp;&nbsp;&nbsp;<a href='/gifs?";
          s+=i;                  
          s+="' style='text-decoration:none;cursor:pointer;color:black;'>&#10008;</a>&nbsp;&nbsp;&nbsp;&nbsp;</td></tr>";
      }
      
      s+="<tr><td colspan='2' align=center><br><br><FORM action='/fupload' method='post' enctype='multipart/form-data'><input class='buttons' type='file' name='fupload' id = 'fupload' value='' onchange='form.submit()'></form></td></tr>";
      
      client.print(s); 
         
      s=F("</center></body></html>");client.print(s);

      
}



void clienteweb(){
  
  //Serial.println("Se ha recibido una peticion WEB");
  String s="";
  String req="";
  int contador=0;
  while(!client.connected()) {delay(1);contador++;if(contador>1000){return;}}
  contador=0;
  while(!client.available()) {delay(1);contador++;if(contador>1000){return;}}

  char c;
  do{          
    c = client.read();
    req+=c;          
  }while(c!='\n');

  Serial.println(req);

  if(req.indexOf("/ ")!=-1){

     webprincipal();


  }else if(req.indexOf("/?")!=-1){

     int pos1=-1;
     String d="";
       
     //Serial.println("Se recogen los valores !!");
      
     pos1=req.indexOf("?"); 
     int pos2=req.indexOf(" ",pos1+1);

     configuracion=req.substring(pos1,pos2);

     guardaconfiguracion();
      
     analizaconfiguracion();
          
     webconfig();


  }else if(req.indexOf("/gifs?")!=-1){

      int pos=req.indexOf("/gifs?");pos+=6;
      int pos2=req.indexOf(" ",pos);

      String numero=req.substring(pos,pos2);

      numborrar=numero.toInt();
      

      s=F("<html><head><script>setTimeout(function(){location.href='./gifs',12000} );</script></head><body></body></html>");client.print(s); 
      client.print("\r\n\r\n");        
      cerrarcliente(); 

      borrar=true;  
      siguiente=true;    
      deleteGif(numborrar);    
      borrar=false;  
      
  }else if(req.indexOf("/gifs ")!=-1){

      websubida();
            

  }else if(req.indexOf("/fupload ")!=-1){

      int tamanoarchivo=0;
      
      String info="";
      char c;
      int sigue=0;
      while(sigue<2){
          c=client.read();
          info+=c;
          if(c=='\n'){
              if(info.indexOf("\r\n\r\n")!=-1){ 
                sigue++;
                if(sigue<2){ 
                    //Serial.println("");
                    //Serial.println(info);
                    //Serial.println("");
                    int pos=info.indexOf("Content-Length: ");
                    if(pos!=-1){
                        int pos2=info.indexOf("\r\n",pos);
                        String dato=info.substring(pos+16,pos2);
                        tamanoarchivo=dato.toInt();                        
                    }                          
                    info="";  
                }
              }
          }                    
      }

      tamanoarchivo-=187;
      Serial.print("Tamano : ");
      Serial.println(tamanoarchivo);                        
                        
      //Serial.println("");
      //Serial.println(info);
      //Serial.println("");

      String nombrearchivo="";
      
      int pos=info.indexOf("filename=\"");
       if(pos!=-1){
          pos+=10;
          int pos2=info.indexOf("\"",pos);
          nombrearchivo=info.substring(pos,pos2);          
      }  

      Serial.print("Nombre Archivo : ");
      Serial.println(nombrearchivo);         

      String dirarchivo="/";
      dirarchivo+=nombrearchivo;
      
      File archivo;
      
      if(nombrearchivo!="") {
          if(sdcard){ archivo = SD.open(dirarchivo,FILE_WRITE); }             
          else{  archivo = SPIFFS.open(dirarchivo,FILE_WRITE); }             
            
      }
            
      double contador=0;
      sigue=0;
      int tiempo=millis();
      while(sigue==0){
            
          if(client.available()){
            int tam=client.available();
            uint8_t buf[tam];
            tiempo=millis();
            client.read(buf,tam);
            archivo.write(buf,tam);
            contador+=tam;  
            if(contador>=tamanoarchivo) { sigue=1; }
          }else{          
              if((millis()-tiempo)>500){ sigue=1;}
          }
      }

      archivo.close();
      
      Serial.print("Contador : ");      
      Serial.println(contador);

      s=F("<html><head><meta http-equiv='refresh' content='0;url=/gifs'/></head><body></body></html>");client.print(s); 
      
      
  }else if(req.indexOf("/config ")!=-1){

      webconfig();
            

  }else if (req.indexOf("/@")!=-1){

      
      
       espera=true;
      
       boolean sigue=true;
       int pos1=-1;
            
       //Serial.println("Se recogen los valores !!");
       
       pos1=req.indexOf("@",pos1+1); if(pos1==-1) { sigue=false; }

       int pos2=req.indexOf(" ",pos1+1);

       cadena=req.substring(pos1,pos2);

       guardaconfiguracion();

       //Serial.print("|");Serial.print(cadena);Serial.println("|");
       
       analizacadena();

       for(int i=0;i<numefectos;i++){

          if(i!=0) {
              Serial.print("Transicion : "); Serial.println(transicion[i-1]); Serial.println("");                      
          }
          
          Serial.print("Tipo Efecto : "); Serial.println(tipo[i]);
          Serial.print("Gif : "); Serial.println(gifs[i]);
          Serial.print("Texto : "); Serial.println(texto[i]);
          Serial.print("Velocidad : "); Serial.println(velocidad[i]);
          Serial.print("Loops : "); Serial.println(loopss[i]);
          Serial.print("Tolerancia : "); Serial.println(tolerancia[i]);
          Serial.print("Brillo : "); Serial.println(brillo[i]);
          Serial.print("Saturacion : "); Serial.println(saturacion[i]);
          Serial.print("Rojo : "); Serial.println(rojo[i]);
          Serial.print("Verde : "); Serial.println(verde[i]);
          Serial.print("Azul : "); Serial.println(azul[i]);
          Serial.println("");        
             
       }
                     
      client.print("OK\r\n\r\n");

      espera=false;    

      siguiente=true;
    
  }else{

      client.print("OK\r\n\r\n");
    
  }

  client.print("\r\n\r\n");        
  cerrarcliente(); 
  
}



static void * GIFOpenFile(const char *fname, int32_t *pSize)
{
  //log_d("GIFOpenFile( %s )\n", fname );
  if(sdcard){ FSGifFile = SD.open(fname);}
  else{ FSGifFile = SPIFFS.open(fname);}
  
  if (FSGifFile) {
    *pSize = FSGifFile.size();
    return (void *)&FSGifFile;
  }
  return NULL;
}


static void GIFCloseFile(void *pHandle)
{
  File *f = static_cast<File *>(pHandle);
  if (f != NULL)
     f->close();
}


static int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
  int32_t iBytesRead;
  iBytesRead = iLen;
  File *f = static_cast<File *>(pFile->fHandle);
  // Note: If you read a file all the way to the last byte, seek() stops working
  if ((pFile->iSize - pFile->iPos) < iLen)
      iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
  if (iBytesRead <= 0)
      return 0;
  iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
  pFile->iPos = f->position();
  return iBytesRead;
}


static int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
{
  int i = micros();
  File *f = static_cast<File *>(pFile->fHandle);
  f->seek(iPosition);
  pFile->iPos = (int32_t)f->position();
  i = micros() - i;
  //log_d("Seek time = %d us\n", i);
  return pFile->iPos;
}


static void TFTDraw(int x, int y, int w, int h, uint16_t* lBuf ){

  double H,S,V;

  if(tolerancia[numpantalla]!=0){    
  
    for(int i=0;i<w;i++){
      
        uint16_t color=lBuf[i];  
        
        color=(color >> 8)|(color << 8);
        GetRGBFrom16Bit(color); 
  
        int maximo=R; if(G>maximo){ maximo=G; } if(B>maximo){ maximo=B; }
        int minimo=R; if(G<minimo){ minimo=G; } if(B<minimo){ minimo=B; }
  
        if((maximo-minimo)<tolerancia[numpantalla]) { R=0;G=0;B=0; }
             
        //ColorConverter::RgbToHsv(R,G,B,H,S,V);
        //ColorConverter::HsvToRgb(H,S,V,R,G,B);
  
        int R2=R-tolerancia[numpantalla];if(R2<0){ R=0;}else { R=R2;}
        int G2=G-tolerancia[numpantalla];if(G2<0){ G=0;}else { G=G2;}
        int B2=B-tolerancia[numpantalla];if(B2<0){ B=0;}else { B=B2;}
        
        color=rgb565(R,G,B);
        color=(color >> 8)|(color << 8);
        lBuf[i]=color;      
        
    }

  }

  
  if(pinta){ tft.pushRect( x+xOffset, y+yOffset, w, h, lBuf ); }
  
  if(h!=1){
      Serial.print("W : ");Serial.println(w);
      Serial.print("H : ");Serial.println(h);
  }
  
  pantallax=x+xOffset;
  pantallay=y+yOffset;

  boolean primeravez=true;
  
  for(int i=0;i<w;i++){
    
      uint16_t color=lBuf[i];  
      color=(color >> 8)|(color << 8);
      
      GetRGBFrom16Bit(color); 
            
      int maximo=R;if(G>maximo){ maximo=G;}if(B>maximo){ maximo=B;}
      int minimo=R;if(G<minimo){ minimo=G;}if(B<minimo){ minimo=B;}

      int dif=maximo-minimo;
          
      if(dif<32){
          R=maximo;
          G=maximo;
          B=maximo;          
          //Serial.print(R);Serial.print(",");Serial.print(G);Serial.print(",");Serial.println(B);      
      }

      int Rojo1=R;
      int Verde1=G;
      int Azul1=B;

      int px=pantallax/escalax;
      int py=pantallay/escalay;

      float panx=(((float)pantallax)/escalax)-px;

      
      if(panx<0.5){
              
          GetRGBFrom16Bit(pantalla[px][py]); 
          int R02=R;int G02=G;int B02=B;
    
          int R03=(Rojo1+R02)/2;
          int G03=(Verde1+G02)/2;
          int B03=(Azul1+B02)/2;
    
          pantalla[px][py]=rgb565(R03,G03,B03);

      }
      //Serial.print(pantallax);Serial.print(",");Serial.print(pantallay);Serial.print(" - ");Serial.print(px);Serial.print(",");Serial.println(py);
              
      pantallax++;
      if(pantallax>=screensizex){ pantallax=0;pantallay++;}     
       
  }

  //Serial.println("");
  
}


// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
  uint8_t *s;
  uint16_t *d, *usPalette, usTemp[320];
  int x, y, iWidth;

  iWidth = pDraw->iWidth;
  if (iWidth > DISPLAY_WIDTH)
      iWidth = DISPLAY_WIDTH;
  usPalette = pDraw->pPalette;
  y = pDraw->iY + pDraw->y; // current line

  s = pDraw->pPixels;
  if (pDraw->ucDisposalMethod == 2) {// restore to background color
    for (x=0; x<iWidth; x++) {
      if (s[x] == pDraw->ucTransparent)
          s[x] = pDraw->ucBackground;
    }
    pDraw->ucHasTransparency = 0;
  }
  // Apply the new pixels to the main image
  if (pDraw->ucHasTransparency) { // if transparency used
    uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
    int x, iCount;
    pEnd = s + iWidth;
    x = 0;
    iCount = 0; // count non-transparent pixels
    while(x < iWidth) {
      c = ucTransparent-1;
      d = usTemp;
      while (c != ucTransparent && s < pEnd) {
        c = *s++;
        if (c == ucTransparent) { // done, stop
          s--; // back up to treat it like transparent
        } else { // opaque
            *d++ = usPalette[c];
            iCount++;
        }
      } // while looking for opaque pixels
      if (iCount) { // any opaque pixels?
        TFTDraw( pDraw->iX+x, y, iCount, 1, (uint16_t*)usTemp );
        x += iCount;
        iCount = 0;
      }
      // no, look for a run of transparent pixels
      c = ucTransparent;
      while (c == ucTransparent && s < pEnd) {
        c = *s++;
        if (c == ucTransparent)
            iCount++;
        else
            s--;
      }
      if (iCount) {
        x += iCount; // skip these
        iCount = 0;
      }
    }
  } else {
    s = pDraw->pPixels;
    // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
    for (x=0; x<iWidth; x++)
      usTemp[x] = usPalette[*s++];
    TFTDraw( pDraw->iX, y, iWidth, 1, (uint16_t*)usTemp );
  }
} /* GIFDraw() */



int gifPlay( char* gifPath )
{ // 0=infinite

  gif.begin(BIG_ENDIAN_PIXELS);

  if( ! gif.open( gifPath, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw ) ) {
    log_n("Could not open gif %s", gifPath );
    return maxLoopsDuration;
  }

  int frameDelay = 0; // store delay for the last frame
  int then = 0; // store overall delay
  bool showcomment = false;

  // center the GIF !!
  int w = gif.getCanvasWidth();
  int h = gif.getCanvasHeight();


  escalax= (float)w / (float)pantallasizex;
  escalay= (float)h / (float)pantallasizey;

  //Serial.print("Escala X : ");Serial.println(escalax);
  //Serial.print("Escala Y : ");Serial.println(escalay);

  xOffset=0;
  yOffset=0;
  
  if( lastFile != numpantalla ) {
      log_n("Playing %s [%d,%d] with offset [%d,%d]", gifPath, w, h, xOffset, yOffset );
      lastFile = numpantalla;
      showcomment = true;
  }

  for(;;){
    int ini=millis();
    int e=gif.playFrame(false,&frameDelay);    
    int dif=millis()-ini;
    //Serial.print("Tiempo PlayFrame: ");Serial.println(dif);
    //Serial.print("Tiempo FrameDelay: ");Serial.println(frameDelay);

    frameDelay=(frameDelay*100)/velocidad[numpantalla];
    
    if(dif<frameDelay) { 
        int t=frameDelay-dif;
        if(t>1000){ t=1000; }
        Serial.print("Tiempo Espera: ");Serial.println(t);
        ini=millis();
        while((millis()-ini)<t){ if(siguiente) { break; }delay(1);wdt();}        
    }
    
    if(e==0){ break; }
    if(siguiente) { break; }

    ///Se puede añadir botones aquí
    
    if((contadorframe==0)&&(numpantalla!=0)){            
        if(transicion[numpantalla-1]==1){ //FADE TO BLACK  
            fadetoblack2();    
        }else if(transicion[numpantalla-1]==2){ //MOVE FORDWARD
            moveforward2();    
        }else if(transicion[numpantalla-1]==3){ //MOVE BACK
            moveback2();    
        }else if(transicion[numpantalla-1]==4){ //LINES
            lines2();    
        }                     
    }
       
    actualizarleds();  
    //Serial.print("Frame : ");Serial.println(contadorframe);
    
    contadorframe++;
        
         
    if( showcomment )
      if (gif.getComment(GifComment))
        log_n("GIF Comment: %s", GifComment);


    int duracion=(maxGifDuration*100)/velocidad[numpantalla];
    
    then += frameDelay;
    if( then > duracion ) { // avoid being trapped in infinite GIF's
      //log_w("Broke the GIF loop, max duration exceeded");
      break;
    }
  }

  gif.close();

  return then;
}

void ws2812bclear(){  
  for(int i=0;i<numpixeles;i++){       
      strip.setPixelColor(i, strip.Color(0,0,0));
  }  
}

void actualizarleds(){

  //return;

  ws2812bclear();

  int x=0;
  int y=0;
  int ax=1;
  int ay=-1;
  int pixel=0;

  if(esquina==0){
      x=0;  y=0;  ax=1;  ay=1;
  }else if(esquina==1){
      x=0;  y=pantallasizey-1;  ax=1;  ay=-1;
  }else if(esquina==2){
      x=pantallasizex-1;  y=0;  ax=-1;  ay=1;
  }else if(esquina==3){
      x=pantallasizex-1;  y=pantallasizey-1;  ax=-1;  ay=-1;
  }
  
  while(pixel<numpixeles){

    GetRGBFrom16Bit(pantalla[x][y]); 
           
    float R03=((float)R*(float)brillo[numpantalla])/100.0; 
    float G03=((float)G*(float)brillo[numpantalla])/100.0; 
    float B03=((float)B*(float)brillo[numpantalla])/100.0; 

    strip.setPixelColor(pixel,strip.Color((int)R03,(int)G03,(int)B03));
    
    pixel++;

    if(rotar==0){
      y+=ay;    
      if((y>=pantallasizey)||(y<0)){
    
          if(serpentina==1){
              ay=-ay;
          }else{
              if(y>=pantallasizey) { y=-1; }
              else{ y=pantallasizey; }        
          }
          
          y+=ay;
          x+=ax;    
                  
      }
    }else{
       x+=ax;    
      if((x>=pantallasizex)||(x<0)){
    
          if(serpentina==1){
              ax=-ax;
          }else{
              if(x>=pantallasizex) { x=-1; }
              else{ x=pantallasizex; }        
          }
          
          y+=ay;
          x+=ax;    
                  
      }
      
    }
      
    
  }

  //getPixelColorHsv(i, 0, 255-i*(255/CNT), 100)
  
      
  strip.show(); 

  if(pinta){
    
      x=140;
      y=0;
      
      
      for(int j=0;j<pantallasizey;j++){
          for(int i=0;i<pantallasizex;i++){
    
              uint16_t color=pantalla[i][j];

              tft.fillRect(x,y,5,5,color);     
              tft.fillRect(x,y+120,2,4,color);
    
              x+=5;                  
          
          }    
          x=140;
          y+=5;         
      }
      
  }
  
  
}



void setup(){

  WiFi.softAP(ssid, password);        //Start Acces point mode
  
  pinMode(pinadc,INPUT);
  pinMode(pindata,OUTPUT);

  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  if(sdcard){ cargargifsSD(); }
  else { cargargifsSPIFFS(); }


  cargaconfiguracion();
  
  ////Interrupcion cada 1 segundo
  
  timer = timerBegin(0, 80, true);             // timer 0, prescalar: 80, UP counting
  timerAttachInterrupt(timer, &onTimer, true);   // Attach interrupt
  timerAlarmWrite(timer, 1000000, true);     // Match value= 1000000 for 1 sec. delay.
  timerAlarmEnable(timer);                 // Enable Timer with interrupt (Alarm Enable)
 
  
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  


  inicializatira();

  strip.setBrightness(BRIGHTNESS);   
 
  strip.clear();
  strip.show();

  if(pinta){
      tft.init();
      tft.setRotation(1);
      tft.invertDisplay(1);  
      tft.fillScreen(TFT_BLACK);
  }
  

  bufferscreen.createSprite(spritesizex,spritesizey);
  
  
  
     
}


void Task1code( void * pvParameters ){    // en este Core recogemos las peticiones web
  
  boolean servidor=false;
  
  for(;;){    

    ///////////////////Interfaz WEB

    if(!servidor) {
        server.setNoDelay(true);
        server.begin();    
        servidor=true;    
    }else{        
        client = server.available();
        if (client) {  clienteweb();  }               
    }
  
    wdt();    
  } 
  
}



void loop(){ 

  while(borrar){ yield(); }
        
  while(espera){ wdt(); }

  if(numefectos==0) { wdt(); return; }
  
  siguiente=false;  

  for(int i=0;i<pantallasizex;i++){
      for(int j=0;j<pantallasizey;j++){
          pantalla[i][j]=0;
      }
  }
  
  tftclear();


  if(tipo[numpantalla]==1) { //GIFs
  
      String archivo="";
      if(gifs[ngifs[numpantalla]].indexOf("/")==-1){ archivo+="/"; }

      archivo+=gifs[ngifs[numpantalla]];
      
      //Serial.println(archivo);
    
      const char * fileName = archivo.c_str();
      
      int loops = loopss[numpantalla]; // max loops
      int durationControl = maxLoopsDuration; // force break loop after xxx ms

      contadorframe=0;
      
      while(loops-->0 && durationControl > 0 ) {        
        durationControl -= gifPlay( (char*)fileName );  
        actualizarleds();    
        gif.reset();                
        if(siguiente){ break;}    
      }

  }else if(tipo[numpantalla]==2) { //TEXTO

      brillo[numpantalla]=100;
      
      uint16_t color=rgb565(rojo[numpantalla],verde[numpantalla],azul[numpantalla]);  

      muestratexto(texto[numpantalla],loopss[numpantalla],color);
    
  }else if(tipo[numpantalla]==3) { //FECHA

      brillo[numpantalla]=100;
      
      String t="";
      t+=dia;t+="-";    
      t+=mes;t+="-";
      t+=ano;
           
      uint16_t color=rgb565(rojo[numpantalla],verde[numpantalla],azul[numpantalla]);  
      muestratexto(t,loopss[numpantalla],color);      
    
  }else if(tipo[numpantalla]==4) { //HORA

      brillo[numpantalla]=100;
      
      String t="";
      if(horas<10){ t+="0"; }
      t+=horas;t+=":";    

      if(minutos<10){ t+="0"; }
      t+=minutos;
           
      uint16_t color=rgb565(rojo[numpantalla],verde[numpantalla],azul[numpantalla]);  
      muestratexto(t,loopss[numpantalla],color);      
    
  }else if(tipo[numpantalla]==5) { //MATRIX

      brillo[numpantalla]=100;
      
      matrix();
    
  }else if(tipo[numpantalla]==6) { //ESPECTROMETRO (M)
      
      espectrometro();
    
  }else if(tipo[numpantalla]==7) { //ESPECTROMETRO 2 (M)

      espectrometro2();
    
  }else if(tipo[numpantalla]==8) { //RUN (M)

      runescena();
    
  }else if(tipo[numpantalla]==9) { //RUNM (M)

      runmescena();
    
  }else if(tipo[numpantalla]==10) { //BATTLE (M)

      battle();
    
  }else if(tipo[numpantalla]==11) { //PONG (M)

      pong();
    
  }else if(tipo[numpantalla]==12) { //TEST

      test();
    
  }else if(tipo[numpantalla]==13) { //COLOR

      colorescena();  
    
  }


  //////////////
  //TRANSICION//
  //////////////


  if(numpantalla!=numefectos){
    
      if(transicion[numpantalla]==1){ //FADE TO BLACK  
          fadetoblack();    
      }else if(transicion[numpantalla]==2){ //MOVE FORDWARD
          moveforward();    
      }else if(transicion[numpantalla]==3){ //MOVE BACK
          moveback();    
      }else if(transicion[numpantalla]==4){ //LINES
          lines();    
      }

  }
  
  //Serial.println(numpantalla);

  if(!siguiente){
      numpantalla++;
      if(numpantalla>=numefectos){ numpantalla=0; }  
  }
  
  wdt();

      
}
