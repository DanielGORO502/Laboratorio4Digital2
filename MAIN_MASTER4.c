
/* UNIVERSIDAD DEL VALLE DE GUATEMALA
 * DEPARTAMENTO DE INGENIERIA ELCTRONICA & MECATRONICA
 * CURSO: ELECTRONICA DIGITAL 2
 * LABORATORIO No.4
 * 
 * File:   MAIN_MASTER4.c
 * Author: DANIEL GONZALEZ
 *
 * Created on August 12, 2023, 1:58 AM
 */

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG1
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config FOSC  = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF      // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP    = OFF      // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF      // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO  = OFF      // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF      // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// CONFIG2
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT   = OFF      // Flash Program Memory Self Write Enable bits (Write protection off)

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// LIBRERIAS
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#include <xc.h>
#include <stdint.h>
#include "oscilador_config.h"   // LIBRERIA PARA CONFIGURACION DEL OSCILADOR.
#include "LCD.h"                // AGREGAMOS LA LIBRERIRA DE LA LCD.
#include "I2C.h"                // AGREGAMOS LA LIBRERIRA DEL SPI.

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// DIRECTIVAS DEL COPILADOR
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _XTAL_FREQ 8000000      /* DEFINIMOS LA FRECUNCIA DE RELOJ, 
                                   PARA PODER USAR LOS DELAY.*/

//______________________________________________________________________________
// DECLARACION DE VARIABLES
//______________________________________________________________________________
uint8_t A =0x00;

uint8_t MIN;
uint8_t SEG;
uint8_t HORA;

uint8_t DIA;
uint8_t MES;
uint8_t YEAR;

uint8_t POT;

uint8_t SEG_U;
uint8_t SEG_D;

uint8_t MIN_U;
uint8_t MIN_D;

uint8_t HORA_U;
uint8_t HORA_D;

uint8_t DIA_U;
uint8_t DIA_D;

uint8_t MES_U;
uint8_t MES_D;

uint8_t YEAR_U;
uint8_t YEAR_D;


uint8_t  NUM_1;
uint8_t  NUM_2;
uint8_t  NUM_3;
uint8_t  NUM_4;
uint8_t  NUM_5;
uint8_t  NUM_6;

uint8_t  UNID;
uint8_t  DECE;
uint8_t  CENT;

uint8_t  CONT =0;
uint8_t  SET1;
uint8_t  SET2;
uint8_t  SET3;
uint8_t  SET;
uint8_t  LOL;
uint8_t SET_U;
uint8_t SET_D;

//______________________________________________________________________________
// PROTOTIPOS DE FUNCIONES
//______________________________________________________________________________

void SETUP(void);
void DECIMAL(uint8_t V, uint8_t SELEC );

//______________________________________________________________________________
// FUNCION DE INTERRUPCIONES
//______________________________________________________________________________
void __interrupt() isr(void){
    
//----------------------------- INTERRUPCION RBIF ------------------------------     
    
    if(RBIF == 1){
        if(RB0 == 0){
            CONT++;
            if (CONT >6){
            CONT =0;
            }
        }
        if(RB1 == 0){
            __delay_us(200);
            A++;    
        }
        if(RB2 == 0){
            __delay_us(200);
            A--;    
        }
        
        RBIF = 0;
               
    }
}    
//______________________________________________________________________________
// FUNCION PRINCIPAL (MAIN & LOOP)
//______________________________________________________________________________

void main(void) {
    
    SETUP();

        I2C_Master_Start();
        I2C_Master_Write(0b11010000);
        I2C_Master_Write(0x00);
        I2C_Master_Write(0x00);
        I2C_Master_Write(0x34);
        I2C_Master_Write(0x10);
        I2C_Master_Write(0x5);
        
        I2C_Master_Write(0x12);
        I2C_Master_Write(0x08);
        I2C_Master_Write(0x23);
        
        
        I2C_Master_Stop();
        __delay_ms(200);
 
    
//_____________________________ LOOP INFINITO __________________________________    
    
    while(1){
        
//---------------------- ENVIO Y RECEPCION DE DATOS I2C ------------------------    
        if(CONT == 0){
            
        //------------------------ SEGUNDOS
            I2C_Master_Start();
            I2C_Master_Write(0b11010000);
            I2C_Master_Write(0x00);
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            SEG = I2C_Master_Read(0);

        //------------------------ MINUTOS    
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            MIN = I2C_Master_Read(0);
        //------------------------ HORAS
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            HORA = I2C_Master_Read(0);
        //------------------------ LOL    
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            LOL = I2C_Master_Read(0);

        //------------------------ DIA    
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            DIA = I2C_Master_Read(0);
        //------------------------ MES      
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            MES = I2C_Master_Read(0);
        //------------------------ YEAR      
            I2C_Master_RepeatedStart();
            I2C_Master_Write(0b11010001);
            YEAR = I2C_Master_Read(0);

            I2C_Master_Stop();
            __delay_ms(200);    
        }
        
        
        if(CONT == 1){
            
            DECIMAL(A,1);
        
            I2C_Master_Start();
            I2C_Master_Write(0b11010000);
            I2C_Master_Write(0x00);
            I2C_Master_Write(SET);
            I2C_Master_Stop();
            __delay_ms(200);
            
            SEG = SET;
            SET_D = SET & 0xF0;
            SET_D = SET_D >> 4;

            SET_U = SET & 0x0F; 
            /*
            Lcd_Set_Cursor(2,15);
            Lcd_Write_Char(SET_D);       // DECENAS DE SEGUNDO. 
            Lcd_Write_Char(SET_U);       // UNIDADES DE SEGUNDO.
            */
            
        }
     //------------------------ POTENCIOMETRO
        
        I2C_Master_Start();
        I2C_Master_Write(0x50);
        I2C_Master_Write(0);
        I2C_Master_Stop();
        __delay_ms(200);
       
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        POT = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        
        DECIMAL(POT,0);
        
     //------------------------   
        SEG_D = SEG & 0xF0;
        SEG_D = SEG_D >> 4;
        
        SEG_U = SEG & 0x0F;               
     //------------------------        
        MIN_D = MIN & 0xF0;
        MIN_D = MIN_D >> 4;
        
        MIN_U = MIN & 0x0F;               
     //------------------------        
        HORA_D = HORA & 0x30;
        HORA_D = HORA_D >> 4;
        
        HORA_U = HORA & 0x0F;  
     //------------------------
        DIA_D = DIA & 0x30;
        DIA_D = DIA_D >> 4;
        DIA_D = DIA_D + 0x30;
        
        DIA_U = DIA & 0x0F;
        DIA_U = DIA_U + 0x30;
     //------------------------
        MES_D = MES & 0x30;
        MES_D = MES_D >> 4;
        MES_D = MES_D + 0x30;
        
        MES_U = MES & 0x0F;
        MES_U = MES_U + 0x30;
     //------------------------
        YEAR_D = YEAR >> 4;
        YEAR_D = YEAR_D + 0x30;
        
        YEAR_U = YEAR & 0x0F;
        YEAR_U = YEAR_U + 0x30;
     //------------------------
        
        
        NUM_1 = HORA_D + 0x30;
        NUM_2 = HORA_U + 0x30;
        NUM_3 = MIN_D + 0x30;
        NUM_4 = MIN_U + 0x30;   
        NUM_5 = SEG_D + 0x30;
        NUM_6 = SEG_U + 0x30;
        
        //SEG_U = SEG_U + 0x30;
               
//________________________ ESCRITURA EN PANTALLA LCD ___________________________         

        
//----------------------------- DATOS SENOSR 1 ---------------------------------         
        
        Lcd_Set_Cursor(1,1);    
        Lcd_Write_String("S1");      // MOSTRAMOS EL NOMBRE DEL SENSOR "S1".   
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(CENT);        // NUMERO ENTERO POT1. 
        Lcd_Write_Char(46);          // "."
        Lcd_Write_Char(DECE);        // PRIMER DECIMAL POT1. 
        Lcd_Write_Char(UNID);        // SEGUNDO DECIMAL POT1.
        
//------------------------------ HORA Y FECHA ----------------------------------         
        
        Lcd_Set_Cursor(1,5);
        Lcd_Write_Char(CONT+0x30);
        
          
        Lcd_Set_Cursor(2,9);
        Lcd_Write_Char(NUM_1);       // DECENAS DE HORA.
        Lcd_Write_Char(NUM_2);       // UNIDADES DE HORA. 
        Lcd_Write_Char(58);          // ":"
        Lcd_Write_Char(NUM_3);       // DECENAS DE MINUTO.
        Lcd_Write_Char(NUM_4);       // UNIDADES DE MINUTO. 
        Lcd_Write_Char(58);          // ":"
        Lcd_Write_Char(NUM_5);       // DECENAS DE SEGUNDO. 
        Lcd_Write_Char(NUM_6);       // UNIDADES DE SEGUNDO. 
 
//--------------------------------- FECHA --------------------------------------         
              
        Lcd_Set_Cursor(1,9);
        Lcd_Write_Char(DIA_D);       // DECENAS DE HORA.
        Lcd_Write_Char(DIA_U);       // UNIDADES DE HORA. 
        Lcd_Write_Char(47);          // ":"
        Lcd_Write_Char(MES_D);       // DECENAS DE MINUTO.
        Lcd_Write_Char(MES_U);       // UNIDADES DE MINUTO. 
        Lcd_Write_Char(47);          // ":"
        Lcd_Write_Char(YEAR_D);       // DECENAS DE SEGUNDO. 
        Lcd_Write_Char(YEAR_U);       // UNIDADES DE SEGUNDO.
        
        
        
        
    }
    return;
}

//______________________________________________________________________________
// FUNCION DE SEPARACION DE DIGITOS Y CONVERSION ASCII.
//______________________________________________________________________________

void DECIMAL(uint8_t V, uint8_t SELEC ){

//-------------------- DECLARACION DE VARIABLES LOCALES ------------------------
    
    uint16_t VOLT;

//-------------------------- SEPARCION DE DIGITOS ------------------------------
   
    VOLT = (uint16_t)(V*1.961);              // MAPEAMOS EL VALOR ENTRE 0V a 5V.
    if(SELEC ==2){
        VOLT = V;
    } 
    CENT = VOLT/100;                         // SEPARAMOS EL PRIMER DIGITO.
    DECE = (VOLT - CENT*100)/10;             // SEPARAMOS EL SEGUNDO DIGITO.
    UNID = (VOLT - CENT*100 - DECE*10)/1;    // SEPARAMOS EL TERCER DIGITO.

//--------------- CONVERSION ASCII & SELECCION DE POTENCIOMETRO ----------------
    if(SELEC == 0){
        CENT = CENT + 0x30;
        DECE = DECE + 0x30;
        UNID = UNID + 0x30;
    } 
    
    if(SELEC == 1){
        SET3 = CENT + 0x30;
        SET1 = DECE;
        SET2 = UNID;  
        
        SET = (SET1<<4) + SET2;
    }
      
}        

//______________________________________________________________________________
// FUNCION DE CONFIGURACION
//______________________________________________________________________________

void SETUP(void){
    
//------------------- CONFIGURACION DE ENTRADAS Y SALIDAS ----------------------
    
    ANSEL  = 0x00;           // SIN ENTRADAS ANALOGICAS, SOLO DIGITALES.   
    ANSELH = 0x00;        

    TRISA = 0x00;
    TRISB = 0x7;             // DECLARAMOS EL PIN 0 Y 1 DEL PORTB COMO ENTRADAS.
    TRISD = 0x00;            // DECLARAMOS EL PORTD COMO SALIDAS.
    TRISE = 0x00;
    
    PORTA = 0x00;
    PORTB = 0x00;            // LIMPIAMOS LOS PUERTOS.
    PORTD = 0x00;  
    PORTE = 0x00;    
//---------------------- CONFIGURACION DE RELOJ A 8MHZ -------------------------
    
    int_osc_MHz(3);             // 8 MHz     

//-------------------------- CONFIGURACION DE LCD ------------------------------

    unsigned int a;
    Lcd_Init();    

//------------------------- CONFIGURACION DE BOTONES ---------------------------
    
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b00000111;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;    
    IOCBbits.IOCB2 = 1;
    
//-------------------------- CONFIGURACION DEL I2C -----------------------------
    
    I2C_Master_Init(100000);        // INICICALIZAMOS COMUNICACION I2C.
    
//--------------------- CONFIGURACION DE INTERRUPCIONES ------------------------    
    
    INTCONbits.GIE  = 1;     // HABILITAMOS LA INTERRUPCION GLOBALES.
    INTCONbits.PEIE = 1;     // HABILITAMOS LA INTERRUPCION PEIE
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1; 
}
//______________________________________________________________________________
//______________________________________________________________________________