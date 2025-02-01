/**
 * @file     
 * @author 	Alexis ROLLAND
 * @date	
 * @brief 	
 *  
 *
 */

#include "lib_BuzzerCAN.h"

/*----------------------------------------------------------------------------*/
// PIC 18F26K80
/*----------------------------------------------------------------------------*/
#pragma config INTOSCSEL = LOW // LF-INTOSC Low-power Enable bit (HIGH/LOW)
#pragma config SOSCSEL = DIG    //	SOSC Power Selection and mode Configuration bits (HIGH / DIG / LOW)
#pragma config XINST = OFF // 	Extended Instruction Set (ON/OFF)
#pragma config RETEN =	OFF // VREG Sleep Enable bit (ON/OFF)
#pragma config IESO = OFF   // Internal/External Oscillator Switchover bit (ON / OFF)
#pragma config FOSC = INTIO1     // Oscillator Selection bits (XT, HS, ...)
#pragma config FCMEN = OFF  // Fail-Safe Clock Monitor Enable bit
#pragma config PLLCFG =	OFF   // PLL x4 Enable bit (ON/OFF)
#pragma config BOREN = OFF  // Brown-out Reset Enable bits (ON, OFF, NOSLP, SBORDIS)
#pragma config BORPWR =	HIGH // BORMV Power level (HIGH/ZPBORMV/MEDIUM/LOW)
#pragma config BORV =  1    // 2.7V
#pragma config PWRTEN = ON    // Power-up Timer Enable bit (ON / OFF)
#pragma config WDTPS = 256  // Watchdog Timer Postscale Select bits
#pragma config WDTEN = OFF    // Watchdog Timer Enable bit
#pragma config CANMX = PORTB // ECAN Mux bit (PORTB/PORTC)
#pragma config MSSPMSK = MSK7 // MSSP address masking (MSK7/MSK5)
#pragma config MCLRE = ON   // MCLR Pin Enable bit
#pragma config STVREN = OFF // Stack Full/Underflow Reset Enable bit
#pragma config BBSIZ =	BB2K // Boot Block Size (BB2K/BB1K)
#pragma config XINST = OFF   // Extended Instruction Set Enable bit

// Protection lecture
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPD = OFF, CPB = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTRB = OFF
/* Déclarations des variables globales 	*/

/* Directives de compilation		*/
					



/* Programme Principal			*/
void main(void)
{
Initialiser();

while(1)
    {
    mainTask();
    
    }
}					

