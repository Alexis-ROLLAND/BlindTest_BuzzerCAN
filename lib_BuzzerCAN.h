/**
 * @file     
 * @author 	Alexis ROLLAND
 * @date	
 * @brief 	
 *  
 *
 */
#define _XTAL_FREQ  8000000UL
#include <stdint.h>
#include <xc.h>

#include "lib_bus_CAN_K80_v3.h"
//------------------------------------------------------------------------------

#define PANIC_LED   LATBbits.LATB1

#define BLUE_LED        LATCbits.LATC0
#define TRIS_BLUE_LED   TRISCbits.TRISC0

#define RED_LED         LATCbits.LATC1
#define TRIS_RED_LED    TRISCbits.TRISC1

#define GREEN_LED       LATCbits.LATC2
#define TRIS_GREEN_LED  TRISCbits.TRISC2

#define TRIS_BP         TRISBbits.TRISB0
#define BP              PORTBbits.RB0

#define BP_IF           INTCONbits.INT0IF

#define TIMER_IF        INTCONbits.TMR0IF
#define TIMER_IE        INTCONbits.TMR0IE


#define VAL_1SEC        3036
#define TIME_OUT_VALUE  3           /**< in secs    */


typedef enum {
    OFF,
    BLUE,
    RED,
    PURPLE,
    GREEN,
    CYAN,
    LIME,
    WHITE
} led_color_t;

typedef enum{
    APP_IDLE,           /**< Game is in progress, no focus  (All leds off) */
    HAVING_FOCUS,       /**< The local buzzer has gained the focus  (GREEN LED on) */
    WAITING_FOCUS       /**< The local buzzer has lost focus (RED LED on) */
} app_status_t;

typedef enum{
    APP_OK = 0,
    APP_ERROR = -1,
    APP_CAN_ERROR = -10
} app_err_t;

//------------------------------------------------------------------------------
/**
 * Adresses en EEPROM Data
 */
#define BUZZER_ID_ADDR  0x000          /**< Buzzer ID address   */


/**
 * 
 */
#define     BUZZER_OK           0
#define     BUZZER_ID_ERROR     -1

/**
 * 
 */
#define     BUZZER_DEFAULT_ID   0xAA


/**
 * 
 */
#define     BUZZ_CANID          0x101
#define     GO_TO_IDLE_CANID    0x102

#define     BUZZ_ID_CANID       0x111
#define     SET_ID_CANID        0x112
#define     PING_ID_CANID       0x11F



//------------------------------------------------------------------------------
/**
 * 
 */
void Initialiser(void);


/**
 * 
 */
void mainTask(void);


/**
 * 
 * @return 
 */
uint8_t getBuzzerId(void);

/**
 * 
 * @param Id
 * @return 
 */
int8_t  setBuzzerId(uint8_t Id);


/**
 */
void PanicHandler(void);

/**
 * 
 * @param Color
 */
void    setLedColor(led_color_t Color);

/**
 * 
 */
void    startTimeOut(void);
/**
 * 
 * @return 
 */
app_err_t   sendBuzzerId(void);

/**
 * 
 * @return 
 */
app_err_t   sendBuzz(void);

/**
 * 
 * @return 
 */
app_err_t   sendGoToIdle(void);

/**
 * 
 * @param Addr
 * @return 
 */
extern uint8_t     EEPROM_Read(uint24_t Addr);

/**
 * 
 * @param Addr
 * @param Byte
 */
extern void        EEPROM_Write(uint24_t Addr, uint8_t Byte);



