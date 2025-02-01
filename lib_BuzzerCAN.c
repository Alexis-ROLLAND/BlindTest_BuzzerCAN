/**
 * @file     
 * @author 	Alexis ROLLAND
 * @date	
 * @brief 	
 *  
 *
 */

#include <xc.h>

#include "lib_BuzzerCAN.h"

/* Déclarations des variables globales 	*/
uint8_t         BpIsr = 0;          /** Software flag for the BP isr   */
uint8_t         TimeOutFlag = 0;    /** Software flag for time Out */

uint8_t         BuzzerID = 0;       /**     Local Buzzer ID    */ 

app_status_t    appStatus = APP_IDLE;   /** Application status */

uint8_t         txData[8];
can_frame_t     txBuzzer;

uint8_t         rxData[8];
can_frame_t     rxBuzzer;


uint8_t         NbSec = 0;

filter_structure_t  AppFramesFilter;        /** RX BUFFER1 - Filter IDs 2,3 or 4   */
filter_structure_t  ServiceFramesFilter;    /** RX BUFFER0 - Filter IDs 0 or 1   */

/* Directives de compilation		*/
				

/*	Implémentation du code */
/* Interruptions haute priorité		*/
void __interrupt(high_priority) ItHigh(void)
{
    if (BP_IF){
        BpIsr = 1;
        BP_IF = 0;  /**< RAZ Flag   */
    }
}

/* Interruptions Basse priorité	*/
void __interrupt(low_priority) ItLow(void)
{
    if (TIMER_IF){
        WRITETIMER0(VAL_1SEC);
        
        if (NbSec > 0){
            NbSec--;
        }
        else{
            TIMER_IE = 0;   /**< Disbable T0 ISR    */
            T0CONbits.TMR0ON = 0;   /**< Stop timer */
            TimeOutFlag = 1;            /**< Set time out flag  */
        }
        
        TIMER_IF = 0;
    }
}
    

void Initialiser(void){
    can_status_t Res;
    
    TRISBbits.TRISB1 = 0;   /**  RB1 as output (Panic Led)  */
    PANIC_LED = 0;
    
    TRIS_BLUE_LED = 0;  /**     Blue led GPIO as output    */
    BLUE_LED = 0;
    
    TRIS_RED_LED = 0;  /**   Red led GPIO as output    */
    RED_LED = 0;
        
    TRIS_GREEN_LED = 0;  /**     Green led GPIO as output    */
    GREEN_LED = 0;
    
    ANCON1bits.ANSEL10 = 0; /** RB0 as Digital input - Do not forget ! */
    TRIS_BP = 1;            /**  RB0 as input   */
    
    RCONbits.IPEN = 1;      /**  Activate Priorities  fo interrupts  */
    INTCONbits.GIEH = 1;    /**  High priority interrupts enabled   */ 
    INTCONbits.GIEL = 1;    /**  Low priority interrupts enabled   */
    
    INTCONbits.INT0IF = 0;      /** RAZ INT0IF flag - Security */
    INTCON2bits.INTEDG0 = 1;    /** Interrupt INT0 on rising edge */
    INTCONbits.INT0IE = 1;      /** INT0 enabled   */
    
    T0CON = 0b00000100;         /** Timer 0 - Timer Mode - Prediv by 32    */
    INTCON2bits.TMR0IP = 0;     /** Timer 0 - Low priority ISR */
      
    if (CAN_OK != can_init(MHZ8, KBPS20)) PanicHandler();
    txBuzzer.AdrBuffer = txData;
    
    if (CAN_OK != can_setGlobalFilter(RX_BUFFER0, STD_FRAMES_ONLY)) PanicHandler();
    rxBuzzer.AdrBuffer = rxData;
    
    // Set acceptance filters
    ServiceFramesFilter.rxBuffer = RX_BUFFER0;
    ServiceFramesFilter.filterId = FILTER_ID0;
    ServiceFramesFilter.TypeTrame = STD_FRAME;
    ServiceFramesFilter.MaskValue = 0b11111110000;
    ServiceFramesFilter.FilterValue = 0b00100010000;
    
    if (CAN_OK != can_setAcceptanceFilter(&ServiceFramesFilter)) PanicHandler();
    
    // Set acceptance filters
    AppFramesFilter.rxBuffer = RX_BUFFER1;
    AppFramesFilter.filterId = FILTER_ID2;
    AppFramesFilter.TypeTrame = STD_FRAME;
    AppFramesFilter.MaskValue = 0b11111110000;
    AppFramesFilter.FilterValue = 0b00100000000;
    
    if (CAN_OK != can_setAcceptanceFilter(&AppFramesFilter)) PanicHandler();
    
    // Init Buzzer Id
    BuzzerID = getBuzzerId(); 
    if ((BuzzerID == 0) || (BuzzerID == 0xFF)){
        BuzzerID = BUZZER_DEFAULT_ID;
        setBuzzerId(BuzzerID);
    }
}
//------------------------------------------------------------------------------
void mainTask(void){
    can_status_t Res;
    app_err_t appRes;
    int8_t  idRes;
    
    if (BpIsr){
        if (appStatus == APP_IDLE){
            appStatus = HAVING_FOCUS;
            setLedColor(GREEN);
            sendBuzz();
            startTimeOut();
        }
        
        BpIsr = 0;
    }
    
    if (TimeOutFlag){
        appStatus = APP_IDLE;
        sendGoToIdle();
        setLedColor(OFF);
        TimeOutFlag = 0;
    }
    
    Res = can_Receive(RX_BUFFER0, &rxBuzzer); /** Try receiving service frame   */
    if (Res == CAN_ERROR) PanicHandler();
    if (Res == MESSAGE_IN){
        PANIC_LED = ~PANIC_LED;
        switch(rxBuzzer.FrameID){
            case SET_ID_CANID :
                if (rxBuzzer.RtrBit == RTR_OFF){    // Data Frame -> Update Buzzer Id
                    if ((rxBuzzer.AdrBuffer[0] != 0x00) && (rxBuzzer.AdrBuffer[0] != 0xFF)){
                        idRes = setBuzzerId(rxBuzzer.AdrBuffer[0]);
                        if (idRes == BUZZER_OK) {
                            BuzzerID = rxBuzzer.AdrBuffer[0];
                            idRes = sendBuzzerId();
                            if (idRes != APP_OK) PanicHandler(); 
                        }
                    }
                }
                break;
            case PING_ID_CANID :
                if (rxBuzzer.RtrBit == RTR_OFF){
                    if ((rxBuzzer.AdrBuffer[0] == BuzzerID) || (rxBuzzer.AdrBuffer[0] == 0xFF)){
                        idRes = sendBuzzerId();
                            if (idRes != APP_OK) PanicHandler(); 
                    }
                }
                
                break;
            default : break;
        }
    }
        
    Res = can_Receive(RX_BUFFER1, &rxBuzzer); /** Try receiving app frame   */
    if (Res == CAN_ERROR) PanicHandler();
    if (Res == MESSAGE_IN){
        switch(rxBuzzer.FrameID){
            case BUZZ_CANID :
                setLedColor(RED);
                appStatus = WAITING_FOCUS;
                break;
            case GO_TO_IDLE_CANID :
                setLedColor(OFF);
                appStatus = APP_IDLE;
                break;
            
            default : break;
        }
    }
    
    
}
//------------------------------------------------------------------------------
uint8_t getBuzzerId(void){
    return EEPROM_Read(BUZZER_ID_ADDR);
}
//------------------------------------------------------------------------------
int8_t  setBuzzerId(uint8_t Id){
    if ((Id < 1) || (Id > 254)) return BUZZER_ID_ERROR;
    EEPROM_Write(BUZZER_ID_ADDR, Id);        
    return BUZZER_OK;
}
//------------------------------------------------------------------------------
void    setLedColor(led_color_t Color){
    LATC = Color;
}
//------------------------------------------------------------------------------
void    startTimeOut(void){
    WRITETIMER0(VAL_1SEC);
    TIMER_IF = 0;
    TIMER_IE = 1;
    NbSec = TIME_OUT_VALUE;
    TimeOutFlag = 0;
    T0CONbits.TMR0ON = 1;
}
//------------------------------------------------------------------------------
app_err_t   sendBuzzerId(void){
    can_status_t Res;
    
    txBuzzer.FrameID = BUZZ_ID_CANID;
    txBuzzer.RtrBit = RTR_OFF;
    txBuzzer.TypeTrame = STD_FRAME;
    txBuzzer.Taille = 1;
    txData[0] = BuzzerID;
    
    Res = can_Emit(TX_BUFFER0, &txBuzzer);
    
    if (Res != CAN_OK) return APP_ERROR;
    return APP_OK;
}
//------------------------------------------------------------------------------
app_err_t   sendBuzz(void){
    can_status_t Res;
        
    txBuzzer.FrameID = BUZZ_CANID;
    txBuzzer.RtrBit = RTR_OFF;
    txBuzzer.TypeTrame = STD_FRAME;
    txBuzzer.Taille = 1;
    txData[0] = BuzzerID;
    
    Res = can_Emit(TX_BUFFER0, &txBuzzer);
    
    if (Res != CAN_OK) return APP_ERROR;
    return APP_OK;
}
//------------------------------------------------------------------------------
app_err_t   sendGoToIdle(void){
    can_status_t Res;
    
    txBuzzer.FrameID = GO_TO_IDLE_CANID;
    txBuzzer.RtrBit = RTR_OFF;
    txBuzzer.TypeTrame = STD_FRAME;
    txBuzzer.Taille = 0;
        
    Res = can_Emit(TX_BUFFER0, &txBuzzer);
    
    if (Res != CAN_OK) return APP_ERROR;
    return APP_OK;
}
//------------------------------------------------------------------------------
extern uint8_t     EEPROM_Read(uint24_t Addr){
    EEADR = Addr & 0x00FF;
    EEADRH = (Addr >> 8) & 0x0F;
    
    EECON1bits.EEPGD = 0;   /**< Access EEPROM  Data */
    EECON1bits.CFGS = 0;    /**< Access EEPROM or FLASH (not config registers/ */
    EECON1bits.RD = 1;      /**< Initiates EEPROM read  */
    asm("nop");
    return EEDATA;
}
//------------------------------------------------------------------------------
extern void        EEPROM_Write(uint24_t Addr, uint8_t Byte){
    EEADR = Addr & 0x00FF;
    EEADRH = (Addr >> 8) & 0x0F;
    EEDATA = Byte;
    
    EECON1bits.EEPGD = 0;   /**< Access EEPROM  Data */
    EECON1bits.CFGS = 0;    /**< Access EEPROM or FLASH (not config registers/ */
    EECON1bits.WREN = 1;    /**< Enable writes */
    
    INTCONbits.GIEH = 0;    /**< High priority interrupts disabled   */ 
    INTCONbits.GIEL = 0;    /**< Low priority interrupts disabled   */
    
    EECON2 = 0x55;  /**< Start unlock sequence  */
    EECON2 = 0xAA;  /**< Finish unlock sequence */
    
    EECON1bits.WR = 1;      /**< Initiates EEPROM write  */
    while(EECON1bits.WR);   /**< Wait for write to complete */
    
    INTCONbits.GIEH = 1;    /**< High priority interrupts enabled   */ 
    INTCONbits.GIEL = 1;    /**< Low priority interrupts enabled   */
    
    EECON1bits.WREN = 1;    /**< Disable writes */
    
}
//------------------------------------------------------------------------------
void PanicHandler(void){
    while(1){
        __delay_ms(50);
        PANIC_LED = ~PANIC_LED;
    }
}
//------------------------------------------------------------------------------
					
					
					
