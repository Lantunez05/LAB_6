/* 
 * File:   Main_en_C.c
 * Author: Luis Antunez
 *
 * Created on 23 de marzo de 2023, 06:01 PM
 */


// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
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


#include <xc.h>
#include <pic16f887.h>
#include <stdint.h>

/************Definir constantes*******************/
#define _tmr0_value 100
#define _XTAL_FREQ 2000000
/************Variables globales*****************/
uint8_t var;
/************Prototipos***********************/
void setup(void);
/************Interrupciones***********************/
void __interrupt() isr(void)
{
    if(T0IF)
    {
        // tmr0
        RA0=!RA0;
        T0IF =0;
        __delay_ms(20);
        TMR0 = _tmr0_value ;
        
    }
    return;
}
/************Codigo principal****************/

void main (void){
    
    setup();
    while (1)
    {
        //loop principal
        ADCON0bits.GO =1;  // Habilita las conversiones de analogico a digital
        while(ADIF==0); // 
        int adc = ADRESH;
        PORTD = (char) adc;
        __delay_ms(10);
        
    }
    return;

}

/********Funciones****************/

void setup(void)
{
    //Configuracion de entradas y salidas
    ANSEL= 0b00010000; // Configuracion analogica pin 4 del registro ANSEL (AN4)
    TRISA= 0b00100000; // Configuracion de entrada del  pin 5 (PORTA) 
    TRISC = 0;
    TRISD =0;
    // Limpiamos los puertos
    PORTA=0;
    PORTB=0;
    PORTC=0;
    PORTD=0;
    
    OSCCONbits.IRCF =0b0110; 
    OSCCONbits.SCS = 1;
    
    //config tmr0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b111;
    TMR0 = _tmr0_value;
    
    //configuración de la interrupcion del TMR0
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    INTCONbits.GIE = 1;
    
    // Configuracion del ADC
  
    ADCON0bits.ADCS = 0b01;
    ADCON0bits.CHS = 0b0100;
    __delay_ms(1);
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.ADON = 1;
    ADIF =0;
    
    return;
    
}

