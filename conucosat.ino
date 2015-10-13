//*  versión 3  Arduino Mini**/

#define REINICIAR false
#define dserial softSerial

#define ARDUINO 103

#include "EEPROM.h"
#include "EEPROMAnything.h"
#include "OneWire.h"
#include "DallasTemperature.h"
//#include <DHT.h>
#include <SoftwareSerial.h>
#include "esp8266.h"
#include "commontexts.h"

#define maxTemp 3   // número máximo de sondas ds18B20

#define rxPin 8     // pin para RX SoftSerial
#define txPin 9     // pin para TX SoftSerial
#define dhtPin 10   // pin para sondas DHT
#define owPin 11    // pin para 1-wire

#define maxEDr 2     // número entradas digitales
#define maxSDr 6     // número salidas digitales + relés
#define maxEAr 4     // número entradas analógicassave
byte edPin[maxEDr]={12,13};      // entradas digitales
byte sdPin[maxSDr]={2,3,4,5,6,7}; // salidas digitales + relés
byte anPin[maxEAr]={0,1,2,3};    // entradas analógicas
#define espResetPin 18  // A4

#define myIDddefault 123    // ID por defecto
#define myPortdefault 88    // my Port por defecto
#define gwPortdefault 88    // GW Port por defecto

SoftwareSerial softSerial(rxPin, txPin); // RX, TX SoftSerial

#define lTAB 8      // en 8 bytes se guardan variables de 54 bits, de 7 en 7 por sencillez, máximo 64

#define dirEEID  100         // 1 byte, IDentificador de módulo
#define dirEEIP  101         // 15+1 bytes dirección IP (la última asignada)
#define dirEEgwIP 117        // dirección IP del gateway
#define dirEEgwPort 133      // port del gateway
#define dirEERTssid 140      // 20+1 bytes, ssid del router
#define dirEERTpass 170      // 20+1 bytes, pass del router
#define dirEEGWssid 200      // 20+1 bytes, ssid del gateway
#define dirEEGWpass 230      // 20+1 bytes, pass del gateway
#define dirEEmodo 251        // modo
#define dirEEAPssid 260      // 20+1 bytes, ssid del modo AP
#define dirEEAPpass 290      // 20+1 bytes, pass del modo AP
#define dirEEAnaInt 600      // valores integrados de variables analógicas
#define dirEEperact 660      // 4
#define dirEEestado 665      // 8   
#define dirEEvalAnaInt 702   // 32
#define dirEEAPch 750       // 2+1 bytes, channel del modo AP
#define dirEEAPenc 753      // 1+1 bytes, enc del modo AP

/********* IMPORTANTE  *****************/
//const byte valor[2] = {0,1};    // directo 0=LOW,  1=HIGH    depende de laplaca de relés: placa estrecha
const byte valor[2] = {1,0};    // 2  inverso 1=LOW,  0=HIGH    depende de laplaca de relés: placa ancha

byte bestado[lTAB];         // 8    bytes, estado de salidas digitales, para el arranque
long peract = 60;           // 60 período de actualización automática a nodo raíz 
byte nTemp=0;
byte addr[maxTemp][8];      // identificador de cada sonda DS18B20 (64)
int valoresTemp[maxTemp];   // guarda los valores de las 8 sondas dividido por 100 (16)
unsigned long mact=0;                 // control de tiempo de envío
unsigned long mact15=0;               // control de tiempo de integración
unsigned long mact60=0;               // control de tareas cada 60 seg
unsigned long mact300=0;              // control de tareas cada 300 seg
unsigned long mact3600=0;             // control de tiempo cada 3600 seg
unsigned long mactVar=0;              // control de tiempo variable
byte oldED0, oldED1;                  // valores de entradas digitales para ver si cambian
unsigned long valAnaInt[maxEAr];      // valores integrados de entradas analógicas. (16)
boolean iniciando=true;
byte tipoED[maxEDr];      // tipo de la entrada digital: 0=ON/OFF
float vanaTED;            // valores anal. de entradas digitales para sensores DHT11-Temp
float vanaHED;            // valores anal. de entradas digitales para sensor DHT11-Hum
byte modo=0;              // 0:factory,  1:client AP,  2:client Gateway
byte newmodo=0;           // se toma en cuenta al hacer reset

OneWire ds(owPin);    // on pin 7 (a 4.7K resistor is necessary)
DallasTemperature sensors(&ds);
//DHT dht(dhtPin);

boolean mododebug=false;
 
WIFI wifi(0);          
int mysocket=4;

// comandos
char json[]="/json";      
char jsonr[]="/jsonr";    
char cmdmyid[]="INF myid=";
char ton[]="/on";        
char toff[]="/off";     
char mainr[]="/mainr";   

void pinVAL(byte pin, byte value)    // actua sobre el pin real 
  { 
    midigitalWrite(pin, valor[value]);
    setbit8(bestado, pin, value);
    EEPROM.write(dirEEestado, bestado[0]);    
    EEPROM.write(dirEEestado+1, bestado[1]);    
  }

void leerConf()    {
    EEPROM_readAnything (dirEEID, wifi._myID); 
    EEPROM_readAnything (dirEEmodo, modo); 
    EEPROM_readAnything (dirEEAPssid, wifi._APssid); 
    EEPROM_readAnything (dirEEAPpass, wifi._APpass); 
    EEPROM_readAnything (dirEEAPch, wifi._APch); 
    EEPROM_readAnything (dirEEAPenc, wifi._APenc); 
    EEPROM_readAnything (dirEERTssid, wifi._RTssid); 
    EEPROM_readAnything (dirEERTpass, wifi._RTpass); 
    EEPROM_readAnything (dirEEGWssid, wifi._GWssid); 
    EEPROM_readAnything (dirEEGWpass, wifi._GWpass); 
    EEPROM_readAnything (dirEEgwIP, wifi._gwIP); 
    EEPROM_readAnything (dirEEgwPort, wifi._gwPort); 
    EEPROM_readAnything (dirEEperact, peract);    
//    EEPROM_readAnything (dirEEvalAnaInt, valAnaInt); 
  }
    
void guardarConf()   {
    EEPROM_writeAnything (dirEEID, wifi._myID); 
    EEPROM_writeAnything (dirEEmodo, newmodo); 
    EEPROM_writeAnything (dirEEAPssid, wifi._APssid); 
    EEPROM_writeAnything (dirEEAPpass, wifi._APpass); 
    EEPROM_writeAnything (dirEEAPch, wifi._APch); 
    EEPROM_writeAnything (dirEEAPenc, wifi._APenc); 
    EEPROM_writeAnything (dirEERTssid, wifi._RTssid); 
    EEPROM_writeAnything (dirEERTpass, wifi._RTpass); 
    EEPROM_writeAnything (dirEEGWssid, wifi._GWssid); 
    EEPROM_writeAnything (dirEEGWpass, wifi._GWpass); 
    EEPROM_writeAnything (dirEEgwIP, wifi._gwIP); 
    EEPROM_writeAnything (dirEEgwPort, wifi._gwPort); 
    EEPROM_writeAnything (dirEEperact, peract);    
  }
  
void leerEstado() {
  EEPROM_readAnything (dirEEestado, bestado); }
  
void guardarEstado() {
  EEPROM_writeAnything (dirEEestado, bestado); }

void leevaloresOW()
  { 
  sensors.requestTemperatures();
  for (int i=0; i<nTemp; i++)
    valoresTemp[i] = sensors.getTempC(addr[i])*100;
  }

//        
//////  tratamiento de bits /////////////////////
const byte tab[8] = {1,2,4,8,16,32,64,128};    // 8

byte getbit8(byte tabla[], byte pin)
 {return ((tabla[pin/8] & tab[(pin % 8)]) > 0)?1:0;}
    
void setbit8(byte tabla[], byte pin, byte value)  { 
  byte pindiv8 = pin/8;
  if (value == 0)
    tabla[pindiv8] = tabla[pindiv8] & (255 ^ tab[(pin % 8)]);  
  else
    tabla[pindiv8] = tabla[pindiv8] | tab[(pin % 8)]; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void buscaAddrSensores()
  { 
   for (int i=0; i<nTemp; i++) sensors.getAddress(addr[i], i);
  }

void strcatP(char *dest, const prog_uchar *orig)
  {
  char c;
  while((c = pgm_read_byte(orig++))) dest[strlen(dest)]=c;
  }
  
void strcatP(char *dest, const prog_uchar *orig1,const prog_uchar *orig2)
  { strcatP(dest,orig1); strcatP(dest,orig2);}
  
void strcatP(char *dest, const prog_uchar *orig1,const prog_uchar *orig2,const prog_uchar *orig3)
  { strcatP(dest,orig1,orig2);strcatP(dest,orig3);}
  
void strcatP(char *dest, const prog_uchar *orig1,const prog_uchar *orig2,const prog_uchar *orig3,const prog_uchar *orig4)
  { strcatP(dest,orig1,orig2);strcatP(dest,orig3,orig4);}

void strcatPPPt(char *dest, const prog_uchar *orig1,const prog_uchar *orig2,const prog_uchar *orig3,char *orig4)
  { strcatP(dest,orig1,orig2,orig3);strcat(dest,orig4);}
  
void strcatPt(char *dest, const prog_uchar *orig1,char *orig2)
  { strcatP(dest,orig1); strcat(dest,orig2);}

void strcatPtP(char *dest, const prog_uchar *orig1,char *orig2,const prog_uchar *orig3)
  { strcatPt(dest,orig1,orig2);strcatP(dest,orig3);}

void strcatPtPtP(char *dest, const prog_uchar *orig1,char *orig2,const prog_uchar *orig3,char *orig4,const prog_uchar *orig5)
  { strcatPt(dest,orig1,orig2); 
    strcatPtP(dest,orig3,orig4,orig5);}

void printS(const prog_uchar *str)
  {
  char c;
  while((c = pgm_read_byte(str++))) dserial.write(c);
  }

void printS(const prog_uchar *str1,const prog_uchar *str2)
  {  printS(str1); printS(str2);  }
void printS(const prog_uchar *str1,const prog_uchar *str2,const prog_uchar *str3)
  {  printS(str1,str2); printS(str3);  }
  
void printS(const prog_uchar *str1,const prog_uchar *str2,const prog_uchar *str3,const prog_uchar *str4)
  {  printS(str1,str2); printS(str3,str4);  }
  
void printS(const prog_uchar *str1,const prog_uchar *str2,const prog_uchar *str3,const prog_uchar *str4,const prog_uchar *str5)
  {  printS(str1,str2,str3);printS(str4,str5);  }

void printlnS(const prog_uchar *str)
  {  printS(str,crlf);  }
  
void printSt(const prog_uchar *str1, char *str2)
  { printS(str1);dserial.print(str2);}
void printStS(const prog_uchar *str1, char *str2,const prog_uchar *str3)
  {  printSt(str1,str2); printS(str3);  }

void iniciavalores()
  {
  printS(iniciandovalores,crlf);
  for (int i=0;i<1023;i++) EEPROM.write(i,0);
  newmodo=0;                      // modo fábrica
  wifi._myID=myIDddefault;     // ID=123
  peract = 60;
  memset(wifi._myIP,0,sizeof(wifi._myIP));
  wifi._gwPort=gwPortdefault;
  strcpyP(wifi._RTssid,routerssid);    // cambiar en la definitiva
  strcpyP(wifi._RTpass,routerpass);    // cambiar en la definitiva
  strcpyP(wifi._GWssid,gwssid);        // conucogw
  strcpyP(wifi._GWpass,gwpass);        // conucogw
  strcpyP(wifi._APssid,apssid);        // ESP_xxxxxx
  strcpyP(wifi._APpass,appass);        // conucogw
  strcpy(wifi._APch,"7");        // canal 11
  strcpy(wifi._APenc,"3");        // 
  strcpyP(wifi._gwIP,gwipdef);
  wifi._gwPort=myPortdefault;
  guardarConf();    // guarda configuración con IP actual
  
  memset(bestado,0,sizeof(bestado));
  memset(valAnaInt,0,sizeof(valAnaInt));
  guardarEstado();    // guarda Estado
  }
  
int enviaID()    // no es por petición, es siempre a inciativa de satserver
  {
   char buff[6]; memset(buff,0,sizeof(buff));
   wifi.clearInput();
   strcat(wifi.inputstr,cmdmyid);  strcat(wifi.inputstr,wifi._myIP); 
   strcatPt(wifi.inputstr,guion,itoa(wifi._myID,buff,10));
   dserial.println(wifi.inputstr);
   int auxerr=wifi.SendMsg(mysocket,wifi._gwIP,itoa(wifi._gwPort,buff,10),wifi.inputstr,3000);
   wifi.clearInput();
   return auxerr;
  }
  
int midigitalRead(byte p)
  {
    if (p<8)
      bitRead(PORTD,p);
    else
      bitRead(PORTB,p);
  }

int midigitalWrite(byte p, byte value)
  {
    if (p<8)
      bitWrite(PORTD,p,value);
    else
      bitWrite(PORTB,p,value);
  }

void formaJson()    // compone el texto json y lo guarda en wifi._inputstr
  {
   char buff[20]; memset(buff,0,sizeof(buff));
   int val=0; 
   wifi.clearInput();
   strcatP(wifi.inputstr,llave_i,ID);
   strcatPtP(wifi.inputstr, dospuntos, itoa(wifi._myID,buff,10),coma);
   for (int i=0; i<maxEAr; i++)
     {
     val=analogRead(anPin[i]);
     strcatPtPtP(wifi.inputstr,letraa,itoa(i+1,buff,10),dospuntos,itoa(val,buff,10),coma);
     }
   for (int i=0; i<maxEDr; i++)
     {
     val=midigitalRead(edPin[i]);
     strcatPtPtP(wifi.inputstr,tdi,itoa(i+1,buff,10),dospuntos,itoa(val,buff,10),coma);
     }
   for (int i=0; i<maxSDr; i++)
     {
     val=getbit8(bestado,sdPin[i]);
     strcatPtPtP(wifi.inputstr,tds,itoa(i+1,buff,10),dospuntos,itoa(val,buff,10),coma);
     }
   for (int i=0; i<maxTemp; i++)
     {
     strcatPtP(wifi.inputstr,letras,itoa(i+1,buff,10),dospuntos);
     strcat(wifi.inputstr,itoa(valoresTemp[i],buff,10));
     if (i < maxTemp-1) strcatP(wifi.inputstr,coma);
     }
   strcatP(wifi.inputstr,llave_f);
  }
  
void formaJsonConf()    // compone el texto json y lo guarda en wifi._inputstr
  {
   char buff[20]; memset(buff,0,sizeof(buff));
   int val=0; 
   wifi.clearInput();
   strcatP(wifi.inputstr,llave_i,ID);   
   strcatPtP(wifi.inputstr, dospuntos, itoa(wifi._myID,buff,10), coma);   
   strcatP(wifi.inputstr, modot,dospuntos);   
   strcat(wifi.inputstr,itoa(modo,buff,10));
   for (int j=0;j<3;j++)
     {
     strcatP(wifi.inputstr,coma);
     strcatPtP(wifi.inputstr,letras,itoa(j,buff,10),dospuntos);
     for (int i=0; i<8; i++)
       {
       strcat(wifi.inputstr,addr[j][i]<16?"0":itoa(addr[j][i]/16,buff,16));
       strcat(wifi.inputstr,itoa(addr[j][i]%16,buff,16));
       }
     }
   strcatPPPt(wifi.inputstr,coma,apssidt2,dospuntos,wifi._APssid);
   strcatPPPt(wifi.inputstr,coma,appasst2,dospuntos,wifi._APpass);
   strcatPPPt(wifi.inputstr,coma,ch,dospuntos,wifi._APch);
   strcatPPPt(wifi.inputstr,coma,enc,dospuntos,wifi._APenc);
   strcatPPPt(wifi.inputstr,coma,rtssidt2,dospuntos,wifi._RTssid);
   strcatPPPt(wifi.inputstr,coma,rtpasst2,dospuntos,wifi._RTpass);
   strcatPPPt(wifi.inputstr,coma,gwssidt2,dospuntos,wifi._GWssid);
   strcatPPPt(wifi.inputstr,coma,gwpasst2,dospuntos,wifi._GWpass);
   strcatPPPt(wifi.inputstr,coma,gwip2,dospuntos,wifi._gwIP);
   strcatPPPt(wifi.inputstr,coma,gwport2,dospuntos,itoa(wifi._gwPort,buff,10));
   strcatPPPt(wifi.inputstr,coma,tperact,dospuntos,itoa(peract,buff,10));
   strcatP(wifi.inputstr,llave_f);
  }
  
int enviaJson(boolean newsocket)    // puede ser por tetición o a iniciativa de satserver
  {
   char buff[6];memset(buff,0,sizeof(buff));
   formaJson();
   dserial.println(wifi.inputstr);
   if (newsocket)
     int auxerr=wifi.SendMsg(mysocket,wifi._gwIP,itoa(wifi._gwPort,buff,10),wifi.inputstr,3000);
   else
     int auxerr=wifi.Send(wifi.lastsocket,wifi.inputstr,3000);
  }
 
int enviaJsonr()    // puede ser por petición o a iniciativa de satserver
  {
   char buff[6];memset(buff,0,sizeof(buff));
   formaJson();
   dserial.println(wifi.inputstr);
   int auxerr=wifi.Send(wifi.lastsocket,wifi.inputstr,3000);
  }
 
int enviamainr(boolean newsocket)    // puede ser por petición o a iniciativa de satserver
  {
   char buff[6];memset(buff,0,sizeof(buff));
   formaJsonConf();
   dserial.println(wifi.inputstr);
   if (newsocket)
     int auxerr=wifi.SendMsg(mysocket,wifi._gwIP,itoa(wifi._gwPort,buff,10),wifi.inputstr,3000);
   else
     int auxerr=wifi.Send(wifi.lastsocket,wifi.inputstr,3000);
  }
 
void procesaJson()    // procesa texto JSON...
  {
  printS(procesajson);printStS(paren_i,wifi.inputstr,paren_f);
  return;
  }

void procesaInf()    // procesa comandos INF...
  {
//  dserial.print(wifi._parametros); printS(paren_f);
  }

void procesaSet()    // procesa comandos SET...
  {
  wifi.lastsocket=wifi.inputstr[5]-111;
  if ((strcmp(wifi._comando,ton)==0) || (strcmp(wifi._comando,toff)==0))   // on off
    {
     int auxpin=atoi(wifi._parametros);     // pin
     if (auxpin <= maxSDr) 
       pinVAL(sdPin[auxpin-1],((strcmp(wifi._comando,ton)==0)?1:0));
     enviaJson(false);  // es respuesta
     return;
    }
  }
  
int auxlength=0;  

int envpart (char *cad, boolean sumar)
  {
  char buff[4];
  if (sumar) auxlength=auxlength+strlen(cad);  
  dserial.println(strlen(cad));
//  dserial.print("-"); dserial.println(auxlength);
//  dserial.print("-"); dserial.println(cad);
  delay(30);
  return wifi.Send(wifi.lastsocket,cad,3000);;
  }

int envpartP (const prog_uchar *cadP, boolean sumar)
  {
  char c;
  char cad[200]; memset(cad,0,sizeof(cad));
  while((c = pgm_read_byte(cadP++))) cad[strlen(cad)]=c;
  return envpart(cad,sumar);
  }  

int envrelleno(int prelen, int auxlen, int resto)
  {
  char auxsend[128]; 
  while (prelen-auxlen-22>=100)
    {
    memset(auxsend,0x00,sizeof(auxsend));
    memset(auxsend,' ',100);   
    if (envpart(auxsend,false)<0) return -1;
    auxlen=auxlen+100;
    }
  memset(auxsend,0x00,sizeof(auxsend));
  memset(auxsend,' ',prelen-auxlen-resto);   
  return envpart(auxsend,false);
  }
  
void envheader(int prelen, char *otherurl, boolean other)
  {
  char buff[6]; memset(buff,0,sizeof(buff));
  char auxsend[70]; memset(auxsend,0,sizeof(auxsend));
  if (other)
    strcatPtP(auxsend,headerother_i,otherurl,header_f);
  else
    strcatPtP(auxsend,header_i,itoa(prelen,buff,10),header_f);
  envpart(auxsend,false);
  }
  
void envdato(int tipo, int first, int last, const prog_uchar *cadP)
  {
  char auxsend[128]; 
  char buff[6]; 
  memset(auxsend,0x00,sizeof(auxsend));
  for (int i=first; i<last; i++)
    {
    if (tipo==1)  // entradas digitales
      {
      int auxst=midigitalRead(edPin[i]);
      strcatP(auxsend,tr,auxst==0?td:tdcolor,cadP,b);
      strcat(auxsend,itoa(i+1,buff,10));
      strcatP(auxsend,td_f,tr_f); 
      }
    else
      if (tipo==2)  // salidas digitales
        {
        boolean isON=(getbit8(bestado,sdPin[i])==1);
        strcatP(auxsend,tr,isON?tdcolor:td);
        strcatPtP(auxsend,isON?hrefoff:hrefon,itoa(i+1,buff,10),mayor);
        strcatPt(auxsend,cadP,itoa(i+1,buff,10));
        strcatP(auxsend,td_f,tr_f);
        }
      else    
        {
        strcatP(auxsend,tr,td,cadP,b);
        strcat(auxsend,itoa(i+1,buff,10));
        strcatP(auxsend,td_f,td);
        if (tipo==3)  // entradas analógicas
          {
          strcat(auxsend,itoa(analogRead(anPin[i]),buff,10));
          }
        else          // sondas temperatura
          {
          strcat(auxsend,itoa(valoresTemp[i]/100,buff,10));
          strcatPtP(auxsend,coma,itoa(valoresTemp[i]%100,buff,10),celsius); 
          }
        strcatP(auxsend,td_f,tr_f); 
        }
    }
  envpart(auxsend,true);
  }
 
void indexhtml()
  {
    int prelength=1200;
    envheader(prelength,"",false);
    envpartP(htmlbodytable, true);
    envdato(1,0,maxEDr,entdig);
    for (int i=0; i<maxSDr; i++) 
      envdato(2,i,i+1,saldig);
    envdato(3,0,2,entana);
    envdato(3,2,maxEAr,entana);
    envdato(4,0,2,sonda);
    envdato(4,2,maxTemp,sonda);
    envpartP(menuindex,true);
    envpartP(htmlbodytable_f,true);   
    envrelleno(prelength,auxlength,0);
  }
  
int envvalor(const prog_uchar *tit, int numpar, char *valor, int len, int tipo)
  {
  char buff[6]; memset(buff,0,sizeof(buff));
  char auxsend[128]; memset(auxsend,0,sizeof(auxsend));
  strcatP(auxsend,tr,td);
  if (tipo!=2) strcatP(auxsend,tit);
  strcatP(auxsend,td_f,td,tipo==1?inputtypetext:inputtypesubmit);
  if (tipo==1)
    {
    strcatPtP(auxsend,name,itoa(numpar,buff,10),name_f);
    }
  strcatPtP(auxsend,value,valor,value_f);
  strcatPtP(auxsend,size,itoa(len,buff,10),size_f);
  strcatP(auxsend,value_f);
  strcatPtP(auxsend,maxlength,itoa(len,buff,10),maxlength_f);
  strcatP(auxsend,inputtypetext_f,td_f, tr_f);
  return envpart(auxsend,true);
  }

int envsonda(int pos, const prog_uchar *tit)
  {
  char buff[6]; memset(buff,0,sizeof(buff));
  char auxsend[128]; memset(auxsend, 0, sizeof(auxsend));
  strcatP(auxsend,tr,td,tit,b); 
  strcat(auxsend, itoa(pos,buff,10));
  strcatP(auxsend,td_f,td);
  for (int i=0; i<8; i++)
    {
    strcat(auxsend,addr[pos][i]<16?"0":itoa(addr[pos][i]/16,buff,16));
    strcat(auxsend,itoa(addr[pos][i]%16,buff,16));
    }
  strcatP(auxsend,td_f,tr_f);
  return envpart(auxsend,true);
  }
  
int envstring(const prog_uchar *tit, char *cad)
  {
  char buff[6]; memset(buff,0,sizeof(buff));
  char auxsend[128]; memset(auxsend, 0, sizeof(auxsend));
  strcatP(auxsend,tr,td,tit,td_f); 
  strcatPt(auxsend,td,cad);
  strcatP(auxsend,td_f,tr_f);
  return envpart(auxsend,true);
  }
  
void envfavicon()
  {
    
  }
  
void setuphtml(boolean clearurl)
  {
    char buff[6]; memset(buff,0,sizeof(buff));
    int prelength=1900;
    // headers
    if (clearurl)
      {
      envheader(prelength,"/setup",true);
      }
    else
      {
      envheader(prelength,"",false);
      // body  
      envpartP(htmlbody, true);
      envpartP(menusetup,true);
      envstring(version,wifi._ver);
      for (int i=0; i<3; i++)
        if (envsonda(i,sonda)<0) return; 
      if (envpartP(form, true)<0) return;
      if (envvalor(ID,1,itoa(wifi._myID,buff,10),3,1)<0) return;
      if (envvalor(Modo,2,itoa(modo,buff,10),1,1)<0) return; 
      if (envvalor(apssidt,3,wifi._APssid,19,1)<0) return;
      if (envvalor(appasst,4,wifi._APpass,19,1)<0) return;
      if (envvalor(Canal,5,wifi._APch,2,1)<0) return; 
      if (envvalor(Encmode,6,wifi._APenc,1,1)<0) return; 
      if (envvalor(rtssidt,7,wifi._RTssid,19,1)<0) return;
      if (envvalor(rtpasst,8,wifi._RTpass,19,1)<0) return;
      if (envvalor(gwssidt,9,wifi._GWssid,19,1)<0) return;
      if (envvalor(gwpasst,10,wifi._GWpass,19,1)<0) return;
      if (envvalor(gwipt,11,wifi._gwIP,15,1)<0) return;
      if (envvalor(gwport,12,itoa(wifi._gwPort,buff,10),4,1)<0) return;
      if (envvalor(Peract,13,itoa(peract,buff,10),5,1)<0) return;
      if (envvalor(guardar,14,"Guardar",0,2)<0) return;
      if (envpartP(htmlbodyform_f,true)<0) return;   
      // relleno
      envrelleno(prelength,auxlength,0);
      }
  }

void jsonhtml()
  {
    formaJson();
    int prelength=120;
    envheader(prelength,"",false);
    envpart(wifi.inputstr,true);
    envrelleno(prelength,auxlength,0);
  }  

void saveparm()
  {
   int maxpar=13;
   dserial.println(strtok(wifi.inputstr,"?"));
   char buff[6]; memset(buff,0,sizeof(buff));
   while (maxpar>0)
     {
     int numpar = atoi(strtok(NULL,"= "));
     char *value = strtok(NULL,"& ");
     if (numpar==1) wifi._myID=atoi(value);
     if (numpar==2) newmodo=atoi(value);
     if (numpar==3) strcpy(wifi._APssid,value);
     if (numpar==4) strcpy(wifi._APpass,value);
     if (numpar==5) strcpy(wifi._APch,value);
     if (numpar==6) strcpy(wifi._APenc,value);
     if (numpar==7) {
         dserial.println(value);
         strcpy(wifi._RTssid,value);
         }
     if (numpar==8) strcpy(wifi._RTpass,value);
     if (numpar==9) strcpy(wifi._GWssid,value);
     if (numpar==10) strcpy(wifi._GWpass,value);
     if (numpar==11) strcpy(wifi._gwIP,value);
     if (numpar==12) wifi._gwPort=atoi(value);
     if (numpar==13) peract=atoi(value);
     
     maxpar--;
     }
   guardarConf();
   dserial.println("---");
  }
  
void procesaGet()    // procesa peticiones GET ...
  {
  auxlength=0;  
  if (modo==2)    // gateway
    {
     if (strcmp(wifi._comando,json)==0)    // /json
       {
       enviaJson(false); // es respuesta
       return;
       }
     if (strcmp(wifi._comando,mainr)==0)    // /mainr
       {
       enviamainr(false); // es respuesta
       return;
       }
     if (strcmp(wifi._comando,jsonr)==0)    // /jsonr
       {
       enviaJsonr(); // es respuesta
       return;
       }
     }
   else    // modo client AP
     {
     if (strcmp(wifi._comando,"on")==0)
       {
       pinVAL(sdPin[wifi.inputstr[19]-48]-1,1);
       guardarEstado();
       strcpy(wifi._comando,"index");
       }
     if (strcmp(wifi._comando,"off")==0)
       {
       pinVAL(sdPin[wifi.inputstr[20]-48]-1,0);
       guardarEstado();
       strcpy(wifi._comando,"index");
       }
     if (strcmp(wifi._comando,"index")==0)    // index
       {
       indexhtml();
       printlnS(OK);
       }
     if (strcmp(wifi._comando,"setup")==0)    // setup
       {
       indexhtml();
//       setuphtml(false);
       printlnS(OK);
       }
     if (strcmp(wifi._comando,"json")==0)    // json
       {
        jsonhtml();
        printlnS(OK);
       }
     if (strcmp(wifi._comando,mainr)==0)    // /mainr
       {
       enviamainr(false); // es respuesta
       }
     if (strcmp(wifi._comando,"favicon")==0)    // index
       {
       envfavicon();
       }
     if (strcmp(wifi._comando,"404")==0)    // json
       {
        envpartP(resp_404, false);
       }
     if (strcmp(wifi._comando,"save")==0)    // save data
       {
       saveparm();
       setuphtml(true);
       printlnS(OK);
       }
      }
  }

void initESP8266()
{
  wifi.init(57600);  // inicia puerto serie ESP8266
  printS(treset,b,wifi.Reset()>=0?OK:ERROR,crlf);  // Reset ESP826
  delay(5000);
  wifi.clearResults();
  printS(Modo,dospuntos); dserial.print(modo); printS(b,modo==0?Cwsap:sta,crlf);
  
  printS(version,b,wifi.getVer()>=0?b:ERROR,b);  
  dserial.println(wifi._ver);
  printS(muxmode,b,((wifi.setMux(1)>=0)?b:ERROR),b); 
  dserial.println(wifi._CIPMux);
  printS(RT,dospuntos); 
    printStS(b,wifi._RTssid,coma);   dserial.println(wifi._RTpass);
  printS(gw,dospuntos); 
    printStS(b,wifi._GWssid,coma);
    dserial.println(wifi._GWpass);
  printS(Cwsap,dospuntos,b); 
    dserial.print(wifi._APssid);printS(coma);
    dserial.print(wifi._APpass);printS(coma);
    dserial.print(wifi._APch);printS(coma);   dserial.println(wifi._APenc);
  if (modo==0)  // fabrica, funciona como AP para configurar o principal
    {
    printS(blancoscwmode,b,wifi.setCWMode(AP)>=0?b:ERROR,b); 
    dserial.println(wifi._CWMode);
    printS(Cwsap,dospuntos,b,wifi.setCWSAP(wifi._APssid,wifi._APpass,wifi._APch,wifi._APenc)>=0?b:ERROR,b); 
    printS(openserver,b,wifi.openServer(wifi._myPort)>=0?OK:ERROR,crlf);
    printS(blancoIP);printStS(b,wifi._gwIP,dospuntos);
    }
  if (modo==1)  // cliente de Router
    {
    printS(blancoscwmode,b,wifi.setCWMode(STA)>=0?b:ERROR,b); dserial.println(wifi._CWMode);
    printS(conectando); printStS(b,wifi._RTssid,b);
    printS(wifi.joinAP(wifi._RTssid,wifi._RTpass,10000)>=0?OK:ERROR,b,crlf);
    printS(openserver,b,wifi.openServer(wifi._myPort)>=0?OK:ERROR,crlf);
    printS(blancoIP,b); printStS(wifi.getIP()>=0?b:ERROR,wifi._myIP,dospuntos);
    }
  if (modo==2)  // cliente de gateway
    {
    printS(blancoscwmode,b,wifi.setCWMode(STA)>=0?b:ERROR,b,crlf); 
//    dserial.println(wifi._CWMode);
    printS(conectando); printStS(b,wifi._GWssid,b);
    printS(wifi.joinAP(wifi._GWssid,wifi._GWpass,10000)>=0?OK:ERROR,b,crlf); 
    printS(openserver,b,wifi.openServer(wifi._myPort)>=0?OK:ERROR,crlf);
    printS(blancoIP,b); printStS(wifi.getIP()>=0?b:ERROR,wifi._myIP,dospuntos);
    }
  dserial.println(wifi._myPort);
  printS(cipstatus,b,wifi.getCIPStatus()>=0?b:ERROR,b); 
  dserial.println(wifi._CIPStatus);
}

void confOutput(int pin)
{
  pinMode(pin, INPUT);
  midigitalWrite(pin,HIGH);
  pinMode(pin,OUTPUT);    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // configurar hardware
  dserial.begin(115200);
  delay(100);
  for (int i=0; i<maxEDr; i++) {   // entradas digitales
    pinMode(edPin[i],INPUT);       // define como entradas pines entradas digitales
    midigitalWrite(edPin[i],HIGH);   // activa pull-up
    }
  for (int i=0; i<=maxSDr; i++) 
    confOutput(sdPin[i]);
  confOutput(espResetPin);
  midigitalWrite(espResetPin,HIGH); 
  
//  if (midigitalRead(6)==0)
//    iniciavalores();

  if (REINICIAR)
    iniciavalores();
  else
    leerConf();  
  initESP8266();
  sensors.begin();                 // sondas 1-wire
  nTemp=sensors.getDeviceCount();  // recuenta sondas
  if (nTemp>maxTemp) nTemp=maxTemp;
  buscaAddrSensores();             // obtiene direcciones de sondas
  if (nTemp>0) leevaloresOW();     // lee valores de sondas
  
  mact =millis();
  oldED0=getbit8(bestado,edPin[0]);  // lee último estado entrada digital 1
  oldED1=getbit8(bestado,edPin[1]);  // lee último estado entrada digital 2
  
  leerEstado();                     // lee estado guardado de salidas
  for (int i=0; i<=maxSDr; i++)     // pone salidas al estado guardado
    pinVAL(sdPin[i], getbit8(bestado,sdPin[i]));
//  analogReference(DEFAULT);

//  
   printS(sondas,dospuntos,b); dserial.println(nTemp);
   for (int i=0; i<nTemp; i++)   
     if (sensors.getAddress(addr[i], i))     
       {
//       sensors.setResolution(10);
       printS(b);
       for (uint8_t j = 0; j < 8; j++)     
         {
         if (addr[i][j] < 16) printS(cero);
         dserial.print(addr[i][j], HEX);   
         }
       printS(crlf);
       }
  if (modo==2)    // cliente gateway
    {
    printS(enviandoid);      enviaID();
    delay(1000);
    printS(enviandojson);    enviaJson(true);
    }
  printlnS(ready);
}

void buscatecla()
  {
  while (dserial.available()) 
    {
    int inByte = dserial.read();
    if (inByte==68) {printlnS(mododebugon); mododebug=true;}  // D
    if (inByte==72) {espReset(); setup();}                    // H
    if (inByte==48) {newmodo=0; guardarConf();setup();}       // 0
    if (inByte==49) {newmodo=1; guardarConf();setup();}       // 1
    if (inByte==50) {newmodo=2; guardarConf();setup();}       // 2
    if (inByte==57) {iniciavalores(); guardarConf();setup();} // 9
    }
  }

void espReset()
  {
  printlnS(resetesp8266); 
  wifi.HardReset(espResetPin,1000);  
  delay(5000);
  initESP8266();
  }
  
void loop()
{
  if (mododebug)
    {
    if (dserial.available()) 
      {
      char a=dserial.read();
      if (byte(a)==4) {mododebug=false;printlnS(mododebugoff);}   // CTRL+D
      else wserial.write(a);
      }
    if (wserial.available()) 
      dserial.write(wserial.read());
    return;
    }
    
  if (wifi.msgReceived)    // se ha localizado la cadena final de búsqueda
    {
//    dserial.print(wifi._comtype);
    if (wifi._comtype==0) {printS(noesperado); dserial.println(wifi.inputstr);}
    else
    if (wifi._comtype==1) procesaGet();
    else
    if (wifi._comtype==2) procesaInf();
    else
    if (wifi._comtype==3) procesaSet();
    else
    if (wifi._comtype==4) procesaJson();
    else
    if (wifi._comtype==6) procesaGet();

    wifi.indexinput=0;
    wifi.clearInput();
    wifi.msgReceived=false;
    }
    
  buscatecla();
    
  // si cambia el estado de alguna entrada digital, se envía el estado sin esperar el tiempo
  if (!iniciando)
   {
    setbit8(bestado, edPin[0], midigitalRead(edPin[0]));
    setbit8(bestado, edPin[1], midigitalRead(edPin[1]));
    if ((oldED0!=getbit8(bestado,edPin[0])) || (oldED1!=getbit8(bestado,edPin[1])))
      {
      oldED0=getbit8(bestado,edPin[0]);
      oldED1=getbit8(bestado,edPin[1]);
      if (modo==2)
        enviaJson(true);
      }
   }
  if ((millis() > (mact15 + 15000)))    // período de integración 15 segundos
    {
    iniciando = false;
    mact15=millis();
//    if (modo==2)
//      enviarestadoINT(nodoraiz);
    }
  if ((millis() > (mact60 + 60000)))    // período 60 segundos
    {
    iniciando = false;
    mact60=millis();
    }
  if ((millis() > (mact300 + 300000)))    // período 300 segundos
    {
    iniciando = false;
    mact300=millis();
    if (modo==2)
      enviaID();   
    }
  if ((millis() > (mact3600 + 3600000)))    // cada hora, 3600 segundos
    {
    mact3600=millis();
    EEPROM_writeAnything (dirEEvalAnaInt, valAnaInt); 
    }
  if ((millis() > (mactVar + peract*1000)))    // variable
    {
    iniciando = false;
    mactVar=millis();
    leevaloresOW();
//    dht.begin(dhtPin,DHT11);
//    vanaTED=dht.readTemperature(false); 
//    vanaHED=dht.readHumidity(); 
    if (modo==2)
      {  
//      dserial.print("T"); dserial.print (int(vanaTED)); dserial.println(int(100*(vanaTED-int(vanaTED))));
//      dserial.print("H"); dserial.println(int(vanaHED));dserial.println(int(100*(vanaHED-int(vanaHED))));
//      delay(200);
      enviaJson(true);
      }
    }
}

void serialEvent() 
  {
  if (!mododebug)
    wifi.procSerialEvent1();
  }
