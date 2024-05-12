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

arduinoFFT FFT = arduinoFFT();

#define PINMICRO 36

///////////////////////////
//////////////////////////

SPIClass spiSD = SPIClass(VSPI);

#include "AnimatedGIF.h"

#include "ColorConverterLib.h"
#include <FastLED_NeoPixel.h>

#define DATA_PIN 16
#define NUM_PIXELS 256

FastLED_NeoPixel<NUM_PIXELS, DATA_PIN, NEO_GRB> strip;      // <- FastLED NeoPixel version

AnimatedGIF gif;

String carpeta="/gif";
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

boolean espera=false;

int numpantalla=0;

int numefectos=0;

int screensizex=320;
int screensizey=240;

uint16_t pantalla[64][64];
uint16_t pantalla2[64][64];

int pantallasizex=16;
int pantallasizey=16;

int esquina=0;
int serpentina=0;

int spritesizex=pantallasizex;
int spritesizey=pantallasizey;

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


void tftclear(){

   if(pinta){ 
        tft.fillRect(0,0,DISPLAY_WIDTH,DISPLAY_HEIGHT,0);
   }
   
}

boolean init_sd(){
  
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   
      
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
      while((millis()-ini)<tiempo) { delay(1);wdt(); }         
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
      
      while((millis()-ini)<tiempo) { delay(1);wdt(); }   

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
                vReal[i] = analogRead(PINMICRO);
                vImag[i] = 0;
                delayMicroseconds(100);
                if(siguiente){ return;} 
            }

            FFT.Windowing(vReal,samples,FFT_WIN_TYP_RECTANGLE,FFT_FORWARD);
            FFT.Compute(vReal,vImag,samples,FFT_FORWARD);
            FFT.ComplexToMagnitude(vReal,vImag,samples);

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
  
            while((millis()-ini)<tiempo) { delay(1);wdt(); }               

        }
  
    }             
  
}



void espectrometro(){

  //Serial.println("Espectrometro");
  
  int tiempo=(10000)/velocidad[numpantalla];
  if(tiempo>1000){ tiempo=1000; }
   
  uint16_t samples = 64;

  double vReal[samples];
  double vImag[samples];

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
                vReal[i] = analogRead(PINMICRO);
                vImag[i] = 0;
                delayMicroseconds(100);
                if(siguiente){ return;} 
            }

            FFT.Windowing(vReal,samples,FFT_WIN_TYP_RECTANGLE,FFT_FORWARD);
            FFT.Compute(vReal,vImag,samples,FFT_FORWARD);
            FFT.ComplexToMagnitude(vReal,vImag,samples);

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
  
            while((millis()-ini)<tiempo) { delay(1);wdt(); }     

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
  
            while((millis()-ini)<tiempo) { delay(1);wdt(); }     
               
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
      bufferscreen.drawString(t,px,yy,2);  

      if(pinta){ bufferscreen.pushSprite(0,0); }
    
      px--;    
     
      if(siguiente){ return;}
     
      tfttopantalla();
      actualizarleds();

      while((millis()-ini)<tiempo) { delay(1);wdt(); }

    
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
      gifs[amount]=file.name();      
      amount++;      
      file.close();
    }
    file = GifRootFolder.openNextFile();
  }
  
  GifRootFolder.close();
  Serial.print("Encontrados ");Serial.print(amount);Serial.println(" GIFs");
  
  return amount;
  
}




int deleteGif(int pos) {

    String archivo="/gif/";
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

      String s=F("<html>\r\n<head>\r\n<script>\r\n\r\naddEventListener(\"resize\",actualizatamano);\r\naddEventListener(\"keypress\", function(event) {  \r\n  if (event.key === \"Enter\") {    \r\nevent.preventDefault();\r\n    play(0);\r\n  }\r\n});\r\n\r\nvar numefectos=0;\r\n\r\nvar tamano=60;\r\n\r\nfunction get(peticion){\r\n\r\nip=location.host;\r\n\r\nif(window.XMLHttpRequest){xmlhttp=new XMLHttpRequest();}\r\nelse{xmlhttp=new ActiveXObject(\"Microsoft.XMLHTTP\");}\r\n\r\nxmlhttp.onreadystatechange=function(){\r\nif(xmlhttp.responseText!=\"\" && xmlhttp.readyState==4 && xmlhttp.status==200){\r\ndatorecogido=xmlhttp.responseText;\r\n}else if(xmlhttp.status==404){alert(\"FALLO COMUNICACION !!\");}\r\n}\r\n\r\n//alert(peticion);\r\n\r\nxmlhttp.open(\"GET\",\"http://\"+ip+\"/\"+peticion,true);\r\nxmlhttp.send();\r\n\r\n\r\n}\r\n\r\n\r\n\r\nfunction play(num){\r\n\r\n//alert(\"Se van a enviar los datos !!!\");\r\n\r\nvar datos=\"@\"+numefectos+\";\"+num+\";\";\r\n\r\nfor(var i=1;i<=numefectos;i++){\r\n\r\nif(i!=1){\r\ndatos+=document.getElementById(\"transicion\"+(i-1)).value+\";\";\r\n}\r\n\r\ndatos+=document.getElementById(\"seleccion\"+i).value+\";\";\r\n\r\nif(document.getElementById(\"gifs\"+i)!=null){\r\ndatos+=document.getElementById(\"gifs\"+i).value+\";\";\r\n}else{\r\ndatos+=\"0;\";\r\n}\r\n\r\nif(document.getElementById(\"texto\"+i)!=null){\r\ndatos+=document.getElementById(\"texto\"+i).value+\";\";\r\n}else{\r\ndatos+=\"0;\";\r\n}\r\n\r\ndatos+=document.getElementById(\"tvelocidad\"+i).value+\";\";\r\ndatos+=document.getElementById(\"tloops\"+i).value+\";\";\r\ndatos+=document.getElementById(\"ttolerancia\"+i).value+\";\";\r\ndatos+=document.getElementById(\"tbrillo\"+i).value+\";\";\r\ndatos+=document.getElementById(\"tsaturacion\"+i).value+\";\";\r\ndatos+=document.getElementById(\"trojo\"+i).value+\";\";\r\ndatos+=document.getElementById(\"tverde\"+i).value+\";\";\r\ndatos+=document.getElementById(\"tazul\"+i).value+\";\";\r\n\r\n}\r\n\r\nget(datos);\r\n\r\n}\r\n\r\nfunction elementFromHtml(html){\r\n\r\nconst template=document.createElement(\"template\");\r\ntemplate.innerHTML=html.trim();\r\nreturn template.content;\r\n\r\n}\r\n");client.print(s);
      s=F("\r\n\r\nfunction crearselecgifs(num){\r\n\r\nconst codigo=elementFromHtml(\"<select id='gifs\"+num+\"' style='padding:5px;background:#edf2ff;width:200px'>");client.print(s);
      
      //Se leen los Gifs de la SD y se incluyen en el select
      
      numerogifs = getGifInventory(carpeta); 
      s="";
      for(int i=0;i<numerogifs;i++){
          s+="<option value='"; s+=i; s+="'>";
          s+=gifs[i];        
          s+="</option>";
      }
      client.print(s); 
      
      s=F("</select>\");\r\n\r\ndocument.getElementById(\"imagentexto\"+num).appendChild(codigo);\r\n}\r\n\r\n\r\nfunction eliminarefecto(num){\r\n\r\n\r\n\r\nfor(i=num;i<numefectos;i++){\r\n\r\nif(i==numefectos){ break; }\r\n\r\n\r\nif(i!=1){document.getElementById(\"transicion\"+(i-1)).value=document.getElementById(\"transicion\"+i).value;  }\r\n\r\ndocument.getElementById(\"seleccion\"+i).value=document.getElementById(\"seleccion\"+(i+1)).value;\r\n\r\n\r\nactivardesactivar(i);\r\n\r\n\r\nvar imagen=document.getElementById(\"gifs\"+(i+1));\r\nvar texto=document.getElementById(\"texto\"+(i+1));\r\n\r\nif(imagen!=null) { document.getElementById(\"gifs\"+i).value=imagen.value; }\r\n\r\nif(texto!=null) {document.getElementById(\"texto\"+i).value=texto.value; }\r\n\r\ndocument.getElementById(\"velocidad\"+i).value=document.getElementById(\"velocidad\"+(i+1)).value;\r\ndocument.getElementById(\"tvelocidad\"+i).value=document.getElementById(\"tvelocidad\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"loops\"+i).value=document.getElementById(\"loops\"+(i+1)).value;\r\ndocument.getElementById(\"tloops\"+i).value=document.getElementById(\"tloops\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"tolerancia\"+i).value=document.getElementById(\"tolerancia\"+(i+1)).value;\r\ndocument.getElementById(\"ttolerancia\"+i).value=document.getElementById(\"ttolerancia\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"brillo\"+i).value=document.getElementById(\"brillo\"+(i+1)).value;\r\ndocument.getElementById(\"tbrillo\"+i).value=document.getElementById(\"tbrillo\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"saturacion\"+i).value=document.getElementById(\"saturacion\"+(i+1)).value;\r\ndocument.getElemen");client.print(s);
      s=F("tById(\"tsaturacion\"+i).value=document.getElementById(\"tsaturacion\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"rojo\"+i).value=document.getElementById(\"rojo\"+(i+1)).value;\r\ndocument.getElementById(\"trojo\"+i).value=document.getElementById(\"trojo\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"verde\"+i).value=document.getElementById(\"verde\"+(i+1)).value;\r\ndocument.getElementById(\"tverde\"+i).value=document.getElementById(\"tverde\"+(i+1)).value;\r\n\r\ndocument.getElementById(\"azul\"+i).value=document.getElementById(\"azul\"+(i+1)).value;\r\ndocument.getElementById(\"tazul\"+i).value=document.getElementById(\"tazul\"+(i+1)).value;\r\n\r\n\r\nactualizavalor(\"azul\",i,0);\r\n\r\n\r\n}\r\n\r\n\r\n\r\n\r\n\r\n\r\nvar efecto=document.getElementById(\"dive\"+numefectos);\r\nefecto.remove();\r\n\r\nnumefectos--;\r\n\r\nif(numefectos!=0){\r\nvar tran=document.getElementById(\"divt\"+numefectos);\r\ntran.remove();\r\n}\r\n\r\n\r\n\r\n\r\n}\r\n\r\n\r\n\r\nfunction creartransicion() {\r\n\r\n\r\nconst codigo=elementFromHtml(\"<div id='divt\"+numefectos+\"'><br><table width=40% style='border-radius:20px;background-color:#FAFAFF;cursor:pointer;'><tr align=center><td width=20%></td><td width=30% style='font-size:170%;font-weight:bold'>TRANSITION \"+numefectos+\"</td><td width=50%><select id='transicion\"+numefectos+\"' style='padding:5px;background:#edf2ff'><option value='0'>NONE</option><option value='1'>FADE TO BLACK</option><option value='2'>MOVE FORWARD</option><option value='3'>MOVE BACK</option><option value='4'>LINES</option></select><td><tr></table><div>\");\r\n\r\n\r\ndocument.getElementById(\"efectos\").appendChild(codigo);\r\n\r\n\r\n        \r\n}\r\n\r\nfunction actualizatamano(){\r\n\r\n//alert(window.innerHeight+\" - \"+window.innerWidth);\r\n\r\nif(window.innerWidth<=1600) { tamano=100; }\r\nelse{ tamano=(1600*100)/window.innerWidth; }\r\n\r\n\r\nfor(i=1;i<=numefectos;i++){\r\nif(document.getElementById(\"tablaefecto\"+i)!=null){\r\ndocument.getElementById(\"tablaefecto\"+i).width=tamano+\"%\";\r\n}\r\n}\r\n\r\n       ");client.print(s);
      s=F(" document.getElementById(\"tablaplay\").width=tamano+\"%\";\r\n\r\n}\r\n\r\nfunction crearefecto(num) {\r\n\r\nif(num==-1) { \r\nnum=numefectos; \r\n}\r\n\r\nif(num==numefectos){\r\nif(numefectos!=0){ creartransicion(); }\r\nnumefectos++;\r\nnum=numefectos; \r\n}\r\n\r\nconst codigo=elementFromHtml(\"<div id='dive\"+num+\"'><br><table id='tablaefecto\"+num+\"' width='60%' style='border-radius:20px;background-color:#FAFAFF;padding:20px'><tr align='left'><td witdh='10%' style='font-size:170%;font-weight:bold;cursor:pointer;' onclick='play(\"+(num-1)+\")'>SCREEN \"+num+\"</td><td witdh='20%'><select id='seleccion\"+num+\"' onchange='activardesactivar(\"+num+\")' style='padding:5px;background:#edf2ff'><option value='0'></option><option value='1'>GIF PLAYER</option><option value='2'>TEXT</option><option value='3'>DATE</option><option value='4'>TIME</option><option value='5'>MATRIX</option><option value='6'>SPECTROMETER</option><option value='7'>SPECTROMETER 2</option></select></td><td witdh='30%' align=center><div id='imagentexto\"+num+\"' width=100%><input type='text' id='texto\"+num+\"' size='30' disabled></div><br><br><table style='background-color:#fff;border:1px solid black;border-collapse:collapse' width=25% id='color\"+num+\"'><tr><td><br><br><br></td></tr></table></td><td witdh='35%'><table width=100%><tr><td width=45%>SPEED (%):</td><td width=10%><input type='text' id='tvelocidad\"+num+\"' size=2 value='100' onchange='actualizavalor(\\\"velocidad\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='velocidad\"+num+\"' value='100' min='0' max='200' onchange='actualizavalor(\\\"velocidad\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>LOOPS (N):</td><td width=10%><input type='text' id='tloops\"+num+\"' size=2 value='1' onchange='actualizavalor(\\\"loops\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='loops\"+num+\"' value='1' min='1' max='10' onchange='actualizavalor(\\\"loops\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>TOLERANCE (M):</td><t");client.print(s);
      s=F("d width=10%><input type='text' id='ttolerancia\"+num+\"' size=2 value='0' onchange='actualizavalor(\\\"tolerancia\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='tolerancia\"+num+\"' value='0' min='0' max='100' onchange='actualizavalor(\\\"tolerancia\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>BRIGHTNESS (%):</td><td width=10%><input type='text' id='tbrillo\"+num+\"' size=2 value='50' onchange='actualizavalor(\\\"brillo\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='brillo\"+num+\"' value='50' min='0' max='100' onchange='actualizavalor(\\\"brillo\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>SATURATION (%):</td><td width=10%><input type='text' id='tsaturacion\"+num+\"' size=2 value='50' onchange='actualizavalor(\\\"saturacion\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='saturacion\"+num+\"' value='50' min='0' max='100' onchange='actualizavalor(\\\"saturacion\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>RED (0-255):</td><td width=10%><input type='text' id='trojo\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"rojo\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='rojo\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"rojo\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>GREEN (0-255):</td><td width=10%><input type='text' id='tverde\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"verde\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='verde\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"verde\\\",\"+num+\",1)' disabled></td></tr><tr><td width=45%>BLUE (0-255):</td><td width=10%><input type='text' id='tazul\"+num+\"' size=2 value='128' onchange='actualizavalor(\\\"azul\\\",\"+num+\",0)' disabled></td><td width=45% align=center><input type='range' id='azul\"+num+\"' value='128' min='0' max='255' onchange='actualizavalor(\\\"azul\\\",\"+num+\",1)' disabled></td></tr></table></td><td witdh='5%' onclick='");client.print(s);
      s=F("eliminarefecto(\"+num+\")' valign='top' style='cursor:pointer;'><table style='background-color: lightgrey;'><tr><td style='background-color:white;font-family: Arial Rounded MT Bold'><b>&nbsp;X&nbsp;</b></td></tr></table></td></tr></table><div>\");\r\n\r\n\r\ndocument.getElementById(\"efectos\").appendChild(codigo);\r\n\r\nactualizatamano();\r\nactualizavalor(\"verde\",num,1);\r\n}\r\n\r\n\r\nfunction establecevalor(dato,num,valor){\r\n\r\ndocument.getElementById(dato+num).value=valor;\r\ndocument.getElementById(\"t\"+dato+num).value=valor;\r\n\r\n}\r\n\r\nfunction deshabilita(dato,num,deshabilitar){\r\n\r\ndocument.getElementById(dato+num).disabled = deshabilitar;\r\ndocument.getElementById(\"t\"+dato+num).disabled = deshabilitar;\r\n\r\n}\r\n\r\nfunction activardesactivar(num){\r\n\r\nvar opcion=document.getElementById(\"seleccion\"+num).value;\r\n\r\nvar imagentexto = document.getElementById(\"imagentexto\"+num);\r\n\r\nvar imagen = document.getElementById(\"gifs\"+num);\r\nvar texto = document.getElementById(\"texto\"+num);\r\n\r\nif(imagen!=null){ imagentexto.removeChild(imagen); }\r\nif(texto!=null){ imagentexto.removeChild(texto); }\r\n\r\nif(opcion==0){\r\n\r\nvar input = document.createElement(\"input\");input.setAttribute(\"type\",\"text\");input.setAttribute(\"id\",\"texto\"+num);input.setAttribute(\"size\",\"30\");input.setAttribute(\"disabled\",\"\");\r\nimagentexto.appendChild(input);\r\n\r\ndeshabilita(\"velocidad\",num,true);\r\ndeshabilita(\"loops\",num,true);\r\ndeshabilita(\"tolerancia\",num,true);\r\ndeshabilita(\"brillo\",num,true);\r\ndeshabilita(\"saturacion\",num,true);\r\ndeshabilita(\"rojo\",num,true);\r\ndeshabilita(\"verde\",num,true);\r\ndeshabilita(\"azul\",num,true);\r\n\r\nestablecevalor(\"rojo\",num,255);\r\nestablecevalor(\"verde\",num,255);\r\nestablecevalor(\"azul\",num,255);\r\nactualizavalor(\"verde\",num,1);\r\n\r\n}else if(opcion==1){ //GIF\r\n\r\ncrearselecgifs(num);\r\n\r\ndeshabilita(\"velocidad\",num,false);\r\ndeshabilita(\"loops\",num,false);\r\ndeshabilita(\"tolerancia\",num,false);\r\ndeshabilita(\"brillo\",num,false);\r\ndeshabilita(\"satura");client.print(s);
      s=F("cion\",num,false);\r\ndeshabilita(\"rojo\",num,true);\r\ndeshabilita(\"verde\",num,true);\r\ndeshabilita(\"azul\",num,true);\r\n\r\n}else if((opcion==2)||(opcion==3)||(opcion==4)||(opcion==5)){ //TEXTO , FECHA , HORA \r\n\r\nvar input = document.createElement('input');input.setAttribute('type',\"text\");input.setAttribute('id',\"texto\"+num);input.setAttribute('size',\"30\");\r\nimagentexto.appendChild(input);\r\n\r\ndeshabilita(\"velocidad\",num,false);\r\ndeshabilita(\"loops\",num,false);\r\ndeshabilita(\"brillo\",num,true);\r\ndeshabilita(\"saturacion\",num,true);\r\ndeshabilita(\"rojo\",num,false);\r\ndeshabilita(\"verde\",num,false);\r\ndeshabilita(\"azul\",num,false);\r\n\r\nif((opcion==3)||(opcion==4)){\r\n\r\ndeshabilita(\"tolerancia\",num,true);\r\n\r\nvar currentdate = new Date();\r\nvar datetime = \"(\" + currentdate.getDate() + \"/\" +(currentdate.getMonth()+1)+ \"/\" + currentdate.getFullYear() + \")(\" + currentdate.getHours() + \":\" + currentdate.getMinutes() + \":\" + currentdate.getSeconds()+\")\";\r\n\r\ndocument.getElementById(\"texto\"+num).value=datetime;\r\n\r\n\r\n}else if(opcion==5){\r\n\r\ndeshabilita(\"tolerancia\",num,false);\r\n\r\nestablecevalor(\"tolerancia\",num,50);\r\n\r\nestablecevalor(\"rojo\",num,0);\r\nestablecevalor(\"verde\",num,128);\r\nestablecevalor(\"azul\",num,0);\r\nactualizavalor(\"verde\",num,1);\r\n\r\n}\r\n\r\n}else if((opcion==6)||(opcion==7)){\r\n\r\nvar input = document.createElement('input');input.setAttribute('type',\"text\");input.setAttribute('id',\"texto\"+num);input.setAttribute('size',\"30\");\r\nimagentexto.appendChild(input);\r\n\r\ndeshabilita(\"velocidad\",num,false);\r\ndeshabilita(\"loops\",num,false);\r\ndeshabilita(\"tolerancia\",num,true);\r\ndeshabilita(\"brillo\",num,false);\r\ndeshabilita(\"saturacion\",num,false);\r\ndeshabilita(\"rojo\",num,true);\r\ndeshabilita(\"verde\",num,true);\r\ndeshabilita(\"azul\",num,true);\r\n}\r\n\r\n}\r\n\r\nfunction actualizavalor(dato,num,tipo){\r\nif(tipo==1){ document.getElementById(\"t\"+dato+num).value=document.getElementById(dato+num).va");client.print(s);
      s=F("lue; }\r\nelse{ document.getElementById(dato+num).value=document.getElementById(\"t\"+dato+num).value; }\r\n\r\nif((dato==\"rojo\")||(dato==\"verde\")||(dato==\"azul\")){\r\ndocument.getElementById(\"color\"+num).style=\"background-color:rgb(\"+document.getElementById(\"rojo\"+num).value+\",\"+document.getElementById(\"verde\"+num).value+\",\"+document.getElementById(\"azul\"+num).value+\");border:1px solid black;border-collapse:collapse;\";\r\n}\r\n\r\n}\r\n\r\n\r\n</script>\r\n<style>\r\nbody{\r\nbackground-image: linear-gradient(to right, #0799bf, #d5e7FF);\r\nfont-family: \"Courier New\", monospace;\r\n}\r\nh1{\r\n  border-radius:20px;\r\n  background-image: linear-gradient(to right,#6666FF,#5555FF);\r\n  color: #0000FF;\r\n  font-size:300%;\r\n}\r\n\r\n</style>\r\n\r\n</head>\r\n\r\n<BODY onload=\"crearefecto(-1)\">\r\n<center>\r\n\r\n<table><tr><td><h1>&nbsp;&nbsp;&nbsp;TLEDS&nbsp;&nbsp;<a href='/config' style='text-decoration:none;cursor:pointer;color:#0000FF;'>&#10049;</a><a href='/gifs' style='text-decoration:none;cursor:pointer;color:#0000FF;'>&#10064;</a>&nbsp;&nbsp;</h1></td></tr></table>\r\n\r\n<div id=\"efectos\"></div>\r\n\r\n<br><br>\r\n\r\n<table id='tablaplay' width=60% align=center><tr>\r\n<td width=15%><br></td>\r\n<td width=10%><table width=100% style=\"border-radius:20px;background-color:#FAFAFF;cursor:pointer;\" onclick=\"crearefecto(-1)\"><tr align=center><td width=100% style=\"font-size:600%\">+</td><td><tr></table></td>\r\n<td width=5%><br></td>\r\n<td width=10%><table width=10% style=\"border-radius:20px;background-color:#FAFAFF;cursor:pointer;\" onclick=\"play(0)\"><tr align=center><td width=100% style=\"font-size:600%\">PLAY</td><td><tr></table></td>\r\n\r\n</tr></table>\r\n\r\n</center>\r\n</BODY>\r\n\r\n\r\n</html>");client.print(s);
      s=F("");
  
}


void webconfig(){

      
      String s=F("<html><head><style>body{background-image: linear-gradient(to right, #0799bf, #d5e7FF);font-family:'Courier New',monospace;}</style><script>function salvar(){ var datos=\"?\";datos+=document.getElementById(\"ledsx\").value+\";\";datos+=document.getElementById(\"ledsy\").value+\";\";datos+=document.getElementById(\"firstpixel\").value+\";\";if(document.getElementById(\"serpentine\").checked){datos+=\"1;\";}else{datos+=\"0;\";}window.location.href=\"./\"+datos;}</script></head><body><center><table style='border-radius:20px;background-color:#FAFAFF;padding:20px'>");client.print(s);

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
     int pos2=req.indexOf(";",pos1+1);
     d=req.substring(pos1+1,pos2);
     pantallasizex=d.toInt(); 
     if(pantallasizex>64) { pantallasizex=64;}

     pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
     pantallasizey=d.toInt(); 
     if(pantallasizey>64) { pantallasizey=64;}

     pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
     esquina=d.toInt(); 
     
     pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
     serpentina=d.toInt(); 
     
      
     webprincipal();


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

      String dirarchivo="/gif/";
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
       String d="";
       
       //Serial.println("Se recogen los valores !!");
       
       pos1=req.indexOf("@",pos1+1); if(pos1==-1) { sigue=false; }
       int pos2=req.indexOf(";",pos1+1);
       d= req.substring(pos1+1,pos2);
       numefectos=d.toInt(); 

       pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
       numpantalla=d.toInt(); 
          
       for(int i=0;i<numefectos;i++){

          if(i!=0) { 
             pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
             transicion[i-1]=d.toInt(); 
          }
                    
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          tipo[i]=d.toInt(); 
          
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          ngifs[i]=d.toInt(); 
          
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
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
          
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          velocidad[i]=d.toInt(); if(velocidad[i]==0) { velocidad[i]=1; }
          
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          loopss[i]=d.toInt(); 

          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          tolerancia[i]=d.toInt(); 
   
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          brillo[i]=d.toInt(); 
   
          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          saturacion[i]=d.toInt(); 

          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          rojo[i]=d.toInt(); 

          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          verde[i]=d.toInt(); 

          pos1=pos2;pos2=req.indexOf(";",pos1+1);d = req.substring(pos1+1,pos2);
          azul[i]=d.toInt();           

       }


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
        while((millis()-ini)<t){ 
          delay(1);wdt();
          if(siguiente) { break; }
        }        
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
  for(int i=0;i<NUM_PIXELS;i++){       
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
  int numpixel=0;

  if(esquina==0){
      x=0;  y=0;  ax=1;  ay=1;
  }else if(esquina==1){
      x=0;  y=pantallasizey-1;  ax=1;  ay=-1;
  }else if(esquina==2){
      x=pantallasizex-1;  y=0;  ax=-1;  ay=1;
  }else if(esquina==3){
      x=pantallasizex-1;  y=pantallasizey-1;  ax=-1;  ay=-1;
  }
  
  while(numpixel<NUM_PIXELS){

    GetRGBFrom16Bit(pantalla[x][y]); 
           
    float R03=((float)R*(float)brillo[numpantalla])/100.0; 
    float G03=((float)G*(float)brillo[numpantalla])/100.0; 
    float B03=((float)B*(float)brillo[numpantalla])/100.0; 

    strip.setPixelColor(numpixel,strip.Color((int)R03,(int)G03,(int)B03));
    
    numpixel++;
    
    y+=ay;    
    if((y>=pantallasizey)||(y<0)){

      if(serpentina==1){
          ay=-ay;
      }else{
          if(y>=pantallasizey) { y=0; }
          else{ y=pantallasizey-1; }        
      }
      
      y+=ay;
      x+=ax;    
              
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
  
  pinMode(PINMICRO,INPUT);

  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  if(sdcard){ cargargifsSD(); }
  else { cargargifsSPIFFS(); }
      
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


  strip.begin();   
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
  
      String archivos=carpeta;archivos+="/";archivos+=gifs[ngifs[numpantalla]];
      //Serial.println(archivos);
    
      const char * fileName = archivos.c_str();
      
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
    
  }else if(tipo[numpantalla]==6) { //ESPECTROMETRO
      
      espectrometro();
    
  }else if(tipo[numpantalla]==7) { //ESPECTROMETRO 2

      espectrometro2();
    
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
