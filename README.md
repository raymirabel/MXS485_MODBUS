# MXS485_MODBUS
Protocolo modbus para el sensor ultrasonico MXS485

## CARACTERISTICAS GENERALES
  - 6 interruptores dipswitch para direccionamiento modbus
  - Bus de comunicaciones RS485 con detección automática de dirección
  - Amplio rango de alimentación de 6.5 a 30VDC
  - Regulador conmutado de alta eficiencia
  - Maxima distancia de detección: 10 metros
  - Resolución: 1cm
  - Muestreo: 0.5s
  - IP66
  
# MAPA MODBUS
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
