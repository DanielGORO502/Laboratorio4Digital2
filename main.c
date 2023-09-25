/*
 * File:   main.c
 * Author: Pablo
 * Ejemplo de uso de I2C Esclavo
 * Created on 17 de febrero de 2020, 10:32 AM
 */
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = EXTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "ADC_LIB.h"
#include <xc.h>
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
uint8_t z;
uint8_t dato;
uint8_t V;
uint8_t d_reciv;
//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);
//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************
void __interrupt() isr(void){
   
//----------------------------- INTERRUPCION I2C -------------------------------     
    
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }
                
        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            d_reciv = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;                // Lectura del SSBUF para limpiar el buffer y la bandera BF
            BF = 0;
            SSPBUF = V;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }

//----------------------------- INTERRUPCION ADC -------------------------------     
    
    if(ADIF == 1){               // REVISAMOS LA BANDERA DEL ADC.   
        if(ADCON0bits.CHS == 0){ // REVISAMOS SI ESTA EN EL CANAL 0. 
             V = adc_read();}    // ASIGNAMOS EL VALOR CONVERTEIDO A UNA VARIABLE.              
    }        
}
//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setup();
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
       
//------------------------------- REVISION ADC ---------------------------------      
        if(V > 200){        // REVISAMOS LA CANTIDAD DE LUZ ES BAJA.  
            PORTB = 0x01;        // ENCENDEMOS LA LUZ.     
        }
        else{
            PORTB = 0x00;        // APGAMOS LA LUZ.
        }
        if(ADCON0bits.GO == 0){        // REVISAMOS SI YA TERMINO LA CONVERSION.  
            __delay_us(50);
            ADCON0bits .GO = 1;        // ENCENDEMOS LA CONVERSION NUEVAMENTE.     
        }
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup(void){
    ANSEL = 0x3;            // HABILITAMOS LAS ENTRADAS ANALOGICAS AN1 & AN2.
    ANSELH = 0x3;
    
    TRISA = 0x3;
    TRISB = 0;
    TRISD = 0;
    
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    
//-------------------------- CONFIGURACION DE ADC ------------------------------
    
    adc_init(2,0,0);         // (FOSC/32), VDD, VSS. 
    adc_start(0);            // CHANEL 0, ENCENDEMOS ADC, INICIAMOS CONVERSION. 
    
//-------------------------- CONFIGURACION DEL I2C -----------------------------

    I2C_Slave_Init(0x50);   
    
//---------------------- CONFIGURACION DE INTERRUPCIONES -----------------------
      
    INTCONbits.GIE  = 1;     // HABILITAMOS LA INTERRUPCION GLOBALES.
    //INTCONbits.PEIE = 1;     // HABILITAMOS LA INTERRUPCION PEIE
    //PIR1bits.SSPIF  = 0;     // BORRAMOS LA INTERRUPCION MSSP
    PIE1bits.RCIE   = 1;
    PIR1bits.ADIF   = 0;     // LIMPIAMOS LA BANDERA DEL ADC.  
}
//______________________________________________________________________________
//______________________________________________________________________________