/*
  MXS485.ino - v1.00 - 09/10/2015
  - Version inicial

  MXS485.ino - v1.00 - 22/04/2020
  - Adaptación firmware para hardware 150113
  

  Sketch para el módulo radar ultrasonico MXS485 modbus RS485
  Copyright (c) 2015 Raimundo Alfonso
  Ray Ingeniería Electrónica, S.L.
  
  Este sketch está basado en software libre. Tu puedes redistribuir
  y/o modificarlo bajo los terminos de licencia GNU.

  Esta biblioteca se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los terminos de licencia GNU para más detalles.
  
  * CARACTERISTICAS GENERALES
  - 6 interruptores dipswitch para direccionamiento modbus
  - 2 interruptores dipswitch para velocidad y paridad
  - Bus de comunicaciones RS485 con detección automática de dirección
  - Amplio rango de alimentación de 6.5 a 30VDC
  - Regulador conmutado de alta eficiencia
  - Maxima distancia de detección: 10 metros
  - Resolución: 1cm
  - Muestreo: 0.5s
  - IP66
  
  * MAPA MODBUS
    MODO R: FUNCION 3 - READ BLOCK HOLDING REGISTERS
    MODO W: FUNCION 6 - WRITE SINGLE HOLDING REGISTER
    
  DIRECCION   TIPO    MODO  FORMATO    MAXIMO      MINIMO    UNIDADES    DESCRIPCION
  ---------------------------------------------------------------------------------------------------------
  0x0000      uint    R     00000      10680       00200     mm          MEDIDA INSTANTANEA
  0x0001      uint    R     00000      10680       00200     mm          MEDIDA EN RAMPA 
  0x0002      uint    R     00000      00001       00000     ---         SENSOR LECTURA OK
  0x0003      uint    R/W   00000      10000       00000     ms/cm       RAMPA
  0x0004        
  0x0005      int     R     00000      00063       00000     ---         ESTADO DEL DIPSWITCH
*/

//  - DIP1: DIRECCION MODBUS BIT 0
//  - DIP2: DIRECCION MODBUS BIT 1
//  - DIP3: DIRECCION MODBUS BIT 2
//  - DIP4: DIRECCION MODBUS BIT 3
//  - DIP5: DIRECCION MODBUS BIT 4
//  - DIP6: DIRECCION MODBUS BIT 5
//  - DIP7: OFF(0): VELOCIDAD = 9600
//          ON(1) : VELOCIDAD = 19200
//  - DIP8: OFF(0): PARIDAD = NONE (NINGUNA)
//          ON(1) : PARIDAD = EVEN (PAR)

#include <ModbusSlave.h>
#include <avr/wdt.h> 
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <EEPROM.h>


#define DIR_EEPROM_RAMP  10
#define TIME_OUT         3000
#define TIME_WARM_UP     5000

#define DIPSW1	5   // Dirección modbus 0
#define DIPSW2	6    // Dirección modbus 1
#define DIPSW3	7    // Dirección modbus 2
#define DIPSW4	8    // Dirección modbus 3
#define DIPSW5	9    // Dirección modbus 4
#define DIPSW6	10   // Dirección modbus 5
#define DIPSW7	A6   // Paridad none/par
#define DIPSW8	A7   // Velocidad 9600/19200

#define MAX_BUFFER_RX  15

// Crea variables globales... 
byte cnt = 0;
char buffer_rx[MAX_BUFFER_RX];
unsigned int cm = 0;

SoftwareSerial cdm(A2, 3, true); // RX, TX

// Mapa de registros modbus
enum {        
        MB_MM,           // Distancia en mm
        MB_MM_RAMP,      // Distancia en mm
        MB_OK,           // Sensor OK
        MB_RAMP,         // Rampa ms/cm
        MB_4,            // Reservado
        MB_DIP,          // Estado dipswitch
        MB_REGS	 	 // Numero total de registros
};
int regs[MB_REGS];	

// Crea la clase para el modbus...
ModbusSlave modbus;

int cm_ramp = 0;
unsigned long t = 0;
unsigned long timeOut = 0;
unsigned long startDelay = 5000;
unsigned int ramp = 100;

void setup()  { 
  wdt_disable();
  int velocidad;
  char paridad;

  // Configura puertos de Arduino  
  pinMode(DIPSW1,INPUT);
  pinMode(DIPSW2,INPUT);	
  pinMode(DIPSW3,INPUT);	
  pinMode(DIPSW4,INPUT);	
  pinMode(DIPSW5,INPUT);	
  pinMode(DIPSW6,INPUT);	
  
  // Lectura de parametros en EEPROM...
  if(read_eeprom_uint(DIR_EEPROM_RAMP) == 0xffff) write_eeprom_uint (DIR_EEPROM_RAMP,  ramp);
  ramp = read_eeprom_uint(DIR_EEPROM_RAMP);  

  // configura modbus...
  if(analogRead(DIPSW7) > 512) velocidad = 9600; else velocidad = 19200;  
  if(analogRead(DIPSW8) > 512) paridad = 'n'; else paridad = 'e';  
  modbus.config(velocidad,paridad);  
  modbus.direccion = leeDIPSW() & 0x3F;
  
  // Configura puerto serie software para sensor...
  cdm.begin(9600);

  // Activa WDT cada 4 segundos...   
  wdt_enable(WDTO_4S); 

  // Asigna valores a la tabla modbus...
  regs[MB_MM] = 0;
  regs[MB_MM_RAMP] = 0;  
  regs[MB_RAMP] = ramp;
  regs[MB_OK] = 0;
  regs[MB_DIP] = leeDIPSW();
} 



void loop()  { 
  int val;
  float rs;
  char a;

  modbus.actualiza(regs,MB_REGS);

  // Asigna valores a la tabla modbus...
  if(regs[MB_RAMP] > 10000) regs[MB_RAMP] = 10000;
  if(regs[MB_RAMP] != ramp){
    ramp = regs[MB_RAMP];
    write_eeprom_uint (DIR_EEPROM_RAMP,  ramp);
  }
  regs[MB_MM] = int(cm * 10);
  regs[MB_MM_RAMP] = int(cm_ramp * 10);  
  regs[MB_DIP] = leeDIPSW();

  if(cdm.available()){
    a = cdm.read();
    buffer_rx[cnt++] = a;
    if(a == 13){
      buffer_rx[--cnt] = 0;
      cm = atoi(&buffer_rx[1]);
      cnt = 0;
      timeOut = millis();
    }
    if(cnt >= 5){
      cnt = 0;   
    }
  }
  
  
  // Comprueba que el sensor está OK...
  if(millis() < TIME_WARM_UP){
    cm_ramp = cm;
  }
  if(((millis() - timeOut) > TIME_OUT) || (millis() < TIME_WARM_UP)){
    regs[MB_OK] = 0;
  }else{
    regs[MB_OK] = 1;
  }
  
  if((millis() - t) > ramp){ // rampa en ms/cm
    t = millis();    
    if(cm_ramp > cm) cm_ramp--;
    if(cm_ramp < cm) cm_ramp++;
  }
 
  delay(1);
  wdt_reset();
}

// Rutina para leer el dipswitch
byte leeDIPSW(void){
  byte a0,a1,a2,a3,a4,a5,a6,a7;
  
  // Lee dipswitch...
  a0 = !digitalRead(DIPSW1);  
  a1 = !digitalRead(DIPSW2);
  a2 = !digitalRead(DIPSW3);
  a3 = !digitalRead(DIPSW4);
  a4 = !digitalRead(DIPSW5);  
  a5 = !digitalRead(DIPSW6);  
  if(analogRead(DIPSW7) > 512) a6 = 0; else a6 = 1;
  if(analogRead(DIPSW8) > 512) a7 = 0; else a7 = 1;

  // Calcula dirección...
  return(a0 + a1*2 + a2*4 + a3*8 + a4*16 + a5*32 + a6*64 + a7*128);
}


// Lectura de valores unsigned int16 en EEPROM....
unsigned int read_eeprom_uint(unsigned int dir){
	char n;
	char *pChar;
	unsigned int temp;

	pChar = (char*)&temp;
	for(n=0; n<2; n++)
  	  *(pChar++) = EEPROM.read(dir + n);

	return(temp);
}

// Escritura de valores unsigned int16 en EEPROM....
void write_eeprom_uint(unsigned int dir, unsigned int valor){
	char temp;
	char *pChar;

        pChar = (char*)&valor;
	for(temp=0; temp<2; temp++)
    	EEPROM.write(dir + temp, *(pChar++));
}
