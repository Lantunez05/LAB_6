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
#define _tmr0_value 100 // Valor del timer0
#define _XTAL_FREQ 2000000 // Valor del ciclo de reloj
/************Variables globales*****************/
int val; // Variable para los displays
/************Prototipos***********************/
void setup(void);
/************Interrupciones***********************/
void __interrupt() isr(void)
{
    if(T0IF)
    {
        // Interrupcion del timer0
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
        ADCON0bits.CHS = 0b00000001; // Seleccion del canal AN1
        ADCON0bits.GO =1;  // Habilita las conversiones de analogico a digital
        __delay_ms(10);
        while (ADCON0bits.GO_DONE); // Verificacion del canal AN1
        int adc = ADRESH;           // Mueve el valor almacenado en ADRESH a adc
        PORTC = (char) adc;
        __delay_ms(10);
        
        
        ADCON0bits.CHS = 0b00000010;  // Cambio al canal AN2
        ADCON0bits.GO_DONE = 1; // Iniciar la conversión en el canal AN2
        __delay_ms(10);
        while (ADCON0bits.GO_DONE); // Verificacion del canal AN2
        int adc2 = ADRESH;
        int voltage = adc2/2; // Convertir el resultado de la conversión ADC en voltaje
        
        int digit1 = voltage / 100;
        int digit2 = (voltage % 100) / 10;
        int digit3 = voltage % 10;
        int val;
        
         //Tabla display 1
        switch (digit1) {
            case 0:
                val = 0b00111111;
                break;
            case 1:
                val = 0b10000110;
                break;
            case 2:
                val = 0b11011011;
                break;
            case 3:
                val = 0b11001111;
                break;
            case 4:
                val = 0b11100110;
                break;
            case 5:
                val = 0b11101101;
                break;
            case 6:
                val = 0b11111101;
                break;
            case 7:
                val = 0b10000111;
                break;
            case 8:
                val = 0b11111111;
                break;
            case 9:
                val = 0b11100111;
                break;
            default:
                break;
        }
        
        //Mostrar valor en display 1
        PORTEbits.RE0 = 1; // Encender el pin 0 del puerto E
        PORTD=val;
        __delay_ms(5); 
        PORTEbits.RE0 = 0; // Apagar el pin 0 del Puerto E
        
        //Tabla display 2
        switch (digit2) {
            case 0:
                val = 0b00111111;
                break;
            case 1:
                val = 0b00000110;
                break;
            case 2:
                val = 0b01011011;
                break;
            case 3:
                val = 0b01001111;
                break;
            case 4:
                val = 0b01100110;
                break;
            case 5:
                val = 0b01101101;
                break;
            case 6:
                val = 0b01111101;
                break;
            case 7:
                val = 0b00000111;
                break;
            case 8:
                val = 0b01111111;
                break;
            case 9:
                val = 0b01100111;
                break;
            default:
                break;
        }
        //Mostrar valor en display 2
        PORTEbits.RE1 = 1; // Encender el pin 1 del puerto E
        PORTD=val;
        __delay_ms(5); 
        PORTEbits.RE1 = 0; 
        
        //tabla display 3
        switch (digit3) {
            case 0:
                val = 0b00111111;
                break;
            case 1:
                val = 0b00000110;
                break;
            case 2:
                val = 0b01011011;
                break;
            case 3:
                val = 0b01001111;
                break;
            case 4:
                val = 0b01100110;
                break;
            case 5:
                val = 0b01101101;
                break;
            case 6:
                val = 0b01111101;
                break;
            case 7:
                val = 0b00000111;
                break;
            case 8:
                val = 0b01111111;
                break;
            case 9:
                val = 0b01100111;
                break;
            default:
                break;
        }
        //Mostrar valor en display 3
        PORTEbits.RE2 = 1; // Encender el pin 0 del puerto E
        PORTD=val;
        __delay_ms(5); 
        PORTEbits.RE2 = 0; 
    }
    return;

}

/********Funciones****************/

void setup(void)
{
    //Configuracion de entradas y salidas
    TRISAbits.TRISA1 = 1;   // Configura el pin RA1 como entrada
    ANSELbits.ANS1 = 1;     // Configura el pin AN1 como entrada analógica
    TRISAbits.TRISA2 = 1;   // Configura el pin RA1 como entrada
    ANSELbits.ANS2 = 1;     // Configura el pin AN1 como entrada analógica
    TRISC = 0;
    TRISD =0;
    TRISE =0;
    // Limpiamos los puertos
    PORTA=0;
    PORTC=0;
    PORTD=0;
    PORTE=0;
    
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
    __delay_ms(1);
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.ADON = 1;
    ADIF =0;
    
    return;
    
}
