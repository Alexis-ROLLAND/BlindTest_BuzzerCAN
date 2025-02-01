/**
 * @file lib_bus_CAN_K80_v3.c
 * @author Alexis ROLLAND
 * @date October 2024
 * @brief Library for managing CAN bus communication.
 * 
 * This file contains functions for initializing and managing communication
 * over the CAN bus. It includes functions to set filters, transmit, and receive
 * CAN frames, as well as handle errors and buffer configurations.
 * 
 * @version 3.0
 */
/* inclusion des fichiers entête .h 	*/
#include "lib_bus_CAN_K80_v3.h"

/* Déclarations des variables globales 	*/
uint8_t*	dataptr;

//----------------------------------------------------------------------------
can_status_t   can_init(osc_freq_t Frequence,bus_speed_t Debit){
    /** GPIO TRIS config MUST be done before CAN configuration */
    TRISBbits.TRISB2 = 0;   /** CANTX as GPIO output    */ 
    TRISBbits.TRISB3 = 1;   /** CANTX as GPIO input    */

    CANCON = 0x80;          /** Request Configuration mode  */
    while  ((CANSTAT & 0x80) != 0x80);  /**< Wait until ECAN module is in Configuration mode */

    RXB0CON = 0x00;     /** Reset RECEIVE BUFFER 0 CONTROL REGISTER */
    RXB1CON = 0x00;     /** Reset RECEIVE BUFFER 1 CONTROL REGISTER */

    /**
     * Reset Masks registers : all frames are accepted for now
     */ 
    RXM0SIDL = 0x00;
    RXM0SIDH = 0x00;
    RXM0EIDL = 0x00;
    RXM0EIDH = 0x00;

    RXM1SIDL = 0x00;
    RXM1SIDH = 0x00;
    RXM1EIDL = 0x00;
    RXM1EIDH = 0x00;

    /** Configure baud rate generator */
    switch (Frequence)        {
        case MHZ4 :
            switch (Debit){
                case KBPS10	: BRGCON1 = 0xC7; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;
                case KBPS20 : BRGCON1 = 0xC3; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;					
                case KBPS50 : BRGCON1 = 0xC1; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                case KBPS100 : BRGCON1 = 0xC0; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                default : return CAN_ERROR;
            };break;

        case MHZ8 :
            switch (Debit){
                case KBPS10	: BRGCON1 = 0xCF; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;
                case KBPS20 : BRGCON1 = 0xC7; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;					
                case KBPS50 : BRGCON1 = 0xC3; BRGCON2 = 0xBA; BRGCON3 = 0x07; break;					
                case KBPS100 : BRGCON1 = 0xC1; BRGCON2 = 0xBA; BRGCON3 = 0x07; break;					
                case KBPS125 : BRGCON1 = 0xC1; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;
                case KBPS250 : BRGCON1 = 0xC0; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                case KBPS500 : BRGCON1 = 0x80; BRGCON2 = 0xD0; BRGCON3 = 0x02; break;					
                default : return CAN_ERROR;
            };break;

        case MHZ16 :
                switch (Debit)
                    {
                    case KBPS10	: BRGCON1 = 0xDF; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;
                    case KBPS20 : BRGCON1 = 0xCF; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;					
                    case KBPS50 : BRGCON1 = 0xC7; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                    case KBPS100 : BRGCON1 = 0xC3; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                    case KBPS125 : BRGCON1 = 0xC3; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;
                    case KBPS250 : BRGCON1 = 0xC1; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                    case KBPS500 : BRGCON1 = 0xC0; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                    case KBPS800 : BRGCON1 = 0x80; BRGCON2 = 0xE0; BRGCON3 = 0x02; break;					
                    case KBPS1000 : BRGCON1 = 0x40; BRGCON2 = 0xD0; BRGCON3 = 0x02; break;					
                    default : return CAN_ERROR;
                    };break;
        case MHZ32 :
                switch (Debit)
                    {
                    case KBPS10	: BRGCON1 = 0xFF; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;
                    case KBPS20 : BRGCON1 = 0xDF; BRGCON2 = 0xFF; BRGCON3 = 0x07; break;					
                    case KBPS50 : BRGCON1 = 0xCF; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                    case KBPS100 : BRGCON1 = 0xC7; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                    case KBPS125 : BRGCON1 = 0xC7; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;
                    case KBPS250 : BRGCON1 = 0xC3; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                    case KBPS500 : BRGCON1 = 0xC1; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                    case KBPS800 : BRGCON1 = 0xC0; BRGCON2 = 0xFA; BRGCON3 = 0x07; break;					
                    case KBPS1000 : BRGCON1 = 0xC0; BRGCON2 = 0xF8; BRGCON3 = 0x05; break;					
                    default : return CAN_ERROR;
                    };break;

        default : return CAN_ERROR;
        }

    CIOCON = 0x20;	/**     CANTX HiZ when recessive    */

    CANCON = 0x00;  /** Request normal mode */
    while ((CANSTAT & 0xE0) != 0x00);   /** Wait until ECAN module is in normal mode    */
    
    return CAN_OK;
}
//----------------------------------------------------------------------------
can_status_t    can_Emit(tx_buffer_t txBuffer,  const can_frame_t *pFrame){
    if (pFrame->Taille > DLC_MAX) return TOO_MUCH_DATA;
    if ( (pFrame->TypeTrame == STD_FRAME) && (pFrame->FrameID > STD_MAX_ID)) return BAD_FRAME_ID;
    if ( (pFrame->TypeTrame == EXT_FRAME) && (pFrame->FrameID > EXT_MAX_ID)) return BAD_FRAME_ID;
    
    switch (txBuffer){
        case TX_BUFFER0:
            switch (pFrame->TypeTrame){
                case STD_FRAME : return can_ll_sendStd_b0(pFrame);break;
                case EXT_FRAME : return can_ll_sendExt_b0(pFrame);break;
                default : return BAD_FRAME_FORMAT;break;
                
            } 
            break;
            
        case TX_BUFFER1:
            switch (pFrame->TypeTrame){
                case STD_FRAME : return can_ll_sendStd_b1(pFrame);break;
                case EXT_FRAME : return can_ll_sendExt_b1(pFrame);break;
                default : return BAD_FRAME_FORMAT;break;
                
            }
            break;
            
        case TX_BUFFER2:
            switch (pFrame->TypeTrame){
                case STD_FRAME : return can_ll_sendStd_b2(pFrame);break;
                case EXT_FRAME : return can_ll_sendExt_b2(pFrame);break;
                default : return BAD_FRAME_FORMAT;break;
                
            }
            break;
        default : 
            return BAD_BUFFER_ID;
            break;
    }
    return CAN_ERROR;
}
//----------------------------------------------------------------------------
can_status_t    can_Receive(rx_buffer_t rxBuffer, can_frame_t *pFrame){
    switch(rxBuffer){
        case RX_BUFFER0: return can_ll_Receive_b0(pFrame);break;
        case RX_BUFFER1: return can_ll_Receive_b1(pFrame);break;
        default: return BAD_BUFFER_ID;break;
    }
}
//----------------------------------------------------------------------------
can_status_t    can_setGlobalFilter(rx_buffer_t rxBuffer, filter_type_t filterType){
    switch(rxBuffer){
        case RX_BUFFER0:
            switch (filterType){
                case STD_FRAMES_ONLY :
                        RXB0CON  = 0x20;
                        RXF0SIDL = 0x00;
                        RXF1SIDL = 0x00;
                        break;

                case EXT_FRAMES_ONLY :
                        RXB0CON  = 0x40;
                        RXF0SIDL = 0x08;
                        RXF1SIDL = 0x08;
                        break;
                case BOTH_FRAMES :
                        RXB0CON |= 0x60;
                        break;

                default : return BAD_FILTER_TYPE;break;
            }
            break;
        case RX_BUFFER1:
            switch (filterType){
                case STD_FRAMES_ONLY :
                        RXB1CON  = 0x20;
                        RXF2SIDL = 0x00;
                        RXF3SIDL = 0x00;
                        RXF4SIDL = 0x00;
                        RXF5SIDL = 0x00;

                        break;

                case EXT_FRAMES_ONLY :
                        RXB1CON  = 0x40;
                        RXF2SIDL = 0x08;
                        RXF3SIDL = 0x08;
                        RXF4SIDL = 0x08;
                        RXF5SIDL = 0x08;
                        break;
                case BOTH_FRAMES :
                        RXB1CON |= 0x60;
                        break;

                default : return BAD_FILTER_TYPE;break;
                }
            break;
        default : return BAD_BUFFER_ID;break;
    }
    return CAN_OK; 
}
//----------------------------------------------------------------------------
can_status_t    can_setAcceptanceFilter(const filter_structure_t *pFilter){
    
    switch(pFilter->TypeTrame){
        case STD_FRAME:
            if (pFilter->MaskValue > STD_MAX_ID) return BAD_MASK_VALUE;
            if (pFilter->FilterValue > STD_MAX_ID) return BAD_FILTER_VALUE;
            break;
        case EXT_FRAME:
            if (pFilter->MaskValue > EXT_MAX_ID) return BAD_MASK_VALUE;
            if (pFilter->FilterValue > EXT_MAX_ID) return BAD_FILTER_VALUE;
            break;
        default : return BAD_FRAME_FORMAT;break;
    }
    
    switch(pFilter->rxBuffer){
        case RX_BUFFER0:
            if (pFilter->filterId > 1) return BAD_FILTER_ID;
            switch(pFilter->TypeTrame){
                case STD_FRAME: 
                    can_ll_set_mask_std(0, pFilter->MaskValue); /** No check needed, values are already validated */
                    can_ll_set_filter_std(pFilter->filterId,pFilter->FilterValue);
                    break;
                case EXT_FRAME: break;
                    can_ll_set_mask_ext(0, pFilter->MaskValue); /** No check needed, values are already validated */
                    can_ll_set_filter_ext(pFilter->filterId,pFilter->FilterValue);
                    break;
                default : return BAD_FRAME_FORMAT;break;
            }
            break;
        case RX_BUFFER1:
            if ( (pFilter->filterId < 2) || (pFilter->filterId > 4) ) return BAD_FILTER_ID;
            switch(pFilter->TypeTrame){
                case STD_FRAME: 
                    can_ll_set_mask_std(1, pFilter->MaskValue); /**< No check needed, values are already validated */
                    can_ll_set_filter_std(pFilter->filterId,pFilter->FilterValue);
                    break;
                case EXT_FRAME: 
                    can_ll_set_mask_ext(1, pFilter->MaskValue); /**< No check needed, values are already validated */
                    can_ll_set_filter_ext(pFilter->filterId,pFilter->FilterValue);
                    break;
                default : return BAD_FRAME_FORMAT;break;
            }
            break;
        default : return CAN_ERROR;break;
    }
    
      
    
    
    return CAN_OK;
}
//----------------------------------------------------------------------------
can_status_t    can_getFilterMatch(rx_buffer_t rxBuffer){
    uint8_t	Filtre;

    if ((rxBuffer != RX_BUFFER0) && (rxBuffer != RX_BUFFER1)) return BAD_BUFFER_ID;

    if (rxBuffer == RX_BUFFER0)
        {
        Filtre =  (RXB0CON & 0x01);
        }
    else
        {
        Filtre = (RXB1CON & 0x07);
        if ((Filtre < 2) || (Filtre > 4)) return CAN_ERROR;
        }
    return (Filtre + 10);
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//  LOW LEVEL FUNCTIONS
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendStd_b0(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB0SIDH = ( pFrame->FrameID >> 3) & 0x00FF;
    TXB0SIDL = ( pFrame->FrameID << 5) & 0x00E0;
    
    //	Taille des données
    TXB0DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB0DLC |= MASK_RTR_ON;
    else TXB0DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB0D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB0CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB0CON |= 0x08 ;

    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendStd_b1(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB1SIDH = ( pFrame->FrameID >> 3) & 0x00FF;
    TXB1SIDL = ( pFrame->FrameID << 5) & 0x00E0;
    
    //	Taille des données
    TXB1DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB1DLC |= MASK_RTR_ON;
    else TXB1DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB1D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB1CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB1CON |= 0x08 ;

    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendStd_b2(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB2SIDH = ( pFrame->FrameID >> 3) & 0x00FF;
    TXB2SIDL = ( pFrame->FrameID << 5) & 0x00E0;
    
    //	Taille des données
    TXB2DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB2DLC |= MASK_RTR_ON;
    else TXB2DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB2D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB2CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB2CON |= 0x08 ;

    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendExt_b0(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB0EIDL = ( pFrame->FrameID & 0x000000FF );
    TXB0EIDH = ( pFrame->FrameID >> 8 ) & 0x000000FF ;
    TXB0SIDH = ( pFrame->FrameID >> 21 ) & 0x000000FF ;
    TXB0SIDL = ( pFrame->FrameID >> 13 ) & 0x000000E0 ;
    TXB0SIDL |= (pFrame->FrameID >> 16) & 0x00000003;
    TXB0SIDL |= 0x08;		// Positionnement à 1 du bit EXIDE

    //	Taille des données
    TXB0DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB0DLC |= MASK_RTR_ON;
    else TXB0DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB0D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB0CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB0CON |= 0x08 ;
    
    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendExt_b1(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB1EIDL = ( pFrame->FrameID & 0x000000FF );
    TXB1EIDH = ( pFrame->FrameID >> 8 ) & 0x000000FF ;
    TXB1SIDH = ( pFrame->FrameID >> 21 ) & 0x000000FF ;
    TXB1SIDL = ( pFrame->FrameID >> 13 ) & 0x000000E0 ;
    TXB1SIDL |= (pFrame->FrameID >> 16) & 0x00000003;
    TXB1SIDL |= 0x08;		// Positionnement à 1 du bit EXIDE

    //	Taille des données
    TXB1DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB1DLC |= MASK_RTR_ON;
    else TXB1DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB1D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB1CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB1CON |= 0x08 ;
    
    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_sendExt_b2(const can_frame_t *pFrame){
    uint8_t i;
    
    // Chargement registres ID
    TXB2EIDL = ( pFrame->FrameID & 0x000000FF );
    TXB2EIDH = ( pFrame->FrameID >> 8 ) & 0x000000FF ;
    TXB2SIDH = ( pFrame->FrameID >> 21 ) & 0x000000FF ;
    TXB2SIDL = ( pFrame->FrameID >> 13 ) & 0x000000E0 ;
    TXB2SIDL |= (pFrame->FrameID >> 16) & 0x00000003;
    TXB2SIDL |= 0x08;		// Positionnement à 1 du bit EXIDE

    //	Taille des données
    TXB2DLC = pFrame->Taille;
    
    // Bit RTR
    if (pFrame->RtrBit == RTR_ON) TXB2DLC |= MASK_RTR_ON;
    else TXB2DLC &= MASK_RTR_OFF;

    // Data
    dataptr = (uint8_t*)&TXB2D0;
    
    // Chargement données (si nécessaire)
    if ( (pFrame->AdrBuffer != NULL) && (pFrame->Taille > 0) ){
        for (i=0 ; i < pFrame->Taille ; i++){
            *dataptr = pFrame->AdrBuffer[i];
            dataptr++;
       }
    }
 
    // Attente TXBnREQ = 0
    while ( (TXB2CON & 0x08) != 0x00);

    // Demande d'émission (bit TBXnREQ = 1)
    TXB2CON |= 0x08 ;
    
    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_Receive_b0(can_frame_t *pFrame){
        
    // Un message a-t-il été reçu ?
    if ((RXB0CON & 0x80) != 0x80) return NO_MESSAGE; 
    
    // La trame est-elle standard ou étandue ?
    if ( (RXB0SIDL & 0x08) == 0x08) pFrame->TypeTrame = EXT_FRAME;
    else pFrame->TypeTrame = STD_FRAME;
    
    // Récupération de l'ID
    switch (pFrame->TypeTrame){
        case STD_FRAME:
            pFrame->FrameID = (RXB0SIDH) ;
            pFrame->FrameID = (pFrame->FrameID << 3) & 0x07F8;
            pFrame->FrameID |= ((RXB0SIDL>>5) & 0x07);
            break;
        case EXT_FRAME:
            pFrame->FrameID = RXB0SIDH;
            pFrame->FrameID = (pFrame->FrameID << 3) ;
            pFrame->FrameID |= ((RXB0SIDL >> 5) & 0x07);
            pFrame->FrameID = (pFrame->FrameID << 2) ;
            pFrame->FrameID |= (RXB0SIDL & 0x03);
            pFrame->FrameID = (pFrame->FrameID << 8) ;
            pFrame->FrameID |= RXB0EIDH;
            pFrame->FrameID = (pFrame->FrameID << 8) ;
            pFrame->FrameID |= RXB0EIDL;
            break;
        default : return BAD_FRAME_FORMAT;break;
    }
    
    // Bit RTR
    if ( (RXB0CON & MASK_RX_RTR) == MASK_RX_RTR ) pFrame->RtrBit = RTR_ON;
    else pFrame->RtrBit = RTR_OFF;
	
    // Taille données
    pFrame->Taille = RXB0DLC & 0x0F; 

    // Récupération des données
    if ((pFrame->Taille > 0) && (pFrame->AdrBuffer != NULL)){
        pFrame->AdrBuffer[0] = RXB0D0;
        pFrame->AdrBuffer[1] = RXB0D1;
        pFrame->AdrBuffer[2] = RXB0D2;
        pFrame->AdrBuffer[3] = RXB0D3;
        pFrame->AdrBuffer[4] = RXB0D4;
        pFrame->AdrBuffer[5] = RXB0D5;
        pFrame->AdrBuffer[6] = RXB0D6;
        pFrame->AdrBuffer[7] = RXB0D7;
    }
        
    // RAZ bit RXFULL
    RXB0CON &= 0x7F;
    
    return MESSAGE_IN;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_Receive_b1(can_frame_t *pFrame){
        
    // Un message a-t-il été reçu ?
    if ((RXB1CON & 0x80) != 0x80) return NO_MESSAGE; 
    
    // La trame est-elle standard ou étandue ?
    if ( (RXB1SIDL & 0x08) == 0x08) pFrame->TypeTrame = EXT_FRAME;
    else pFrame->TypeTrame = STD_FRAME;
    
    // Récupération de l'ID
    switch (pFrame->TypeTrame){
        case STD_FRAME:
            pFrame->FrameID = (RXB1SIDH) ;
            pFrame->FrameID = (pFrame->FrameID << 3) & 0x07F8;
            pFrame->FrameID |= ((RXB1SIDL>>5) & 0x07);
            break;
        case EXT_FRAME:
            pFrame->FrameID = RXB1SIDH;
            pFrame->FrameID = (pFrame->FrameID << 3) ;
            pFrame->FrameID |= ((RXB1SIDL >> 5) & 0x07);
            pFrame->FrameID = (pFrame->FrameID << 2) ;
            pFrame->FrameID |= (RXB1SIDL & 0x03);
            pFrame->FrameID = (pFrame->FrameID << 8) ;
            pFrame->FrameID |= RXB1EIDH;
            pFrame->FrameID = (pFrame->FrameID << 8) ;
            pFrame->FrameID |= RXB1EIDL;
            break;
        default : return BAD_FRAME_FORMAT;break;
    }
    
    // Bit RTR
    if ( (RXB1CON & MASK_RX_RTR) == MASK_RX_RTR ) pFrame->RtrBit = RTR_ON;
    else pFrame->RtrBit = RTR_OFF;
	
    // Taille données
    pFrame->Taille = RXB1DLC & 0x0F; 

    // Récupération des données
    if ((pFrame->Taille > 0) && (pFrame->AdrBuffer != NULL)){
        pFrame->AdrBuffer[0] = RXB1D0;
        pFrame->AdrBuffer[1] = RXB1D1;
        pFrame->AdrBuffer[2] = RXB1D2;
        pFrame->AdrBuffer[3] = RXB1D3;
        pFrame->AdrBuffer[4] = RXB1D4;
        pFrame->AdrBuffer[5] = RXB1D5;
        pFrame->AdrBuffer[6] = RXB1D6;
        pFrame->AdrBuffer[7] = RXB1D7;
    }
        
    // RAZ bit RXFULL
    RXB1CON &= 0x7F;
    
    return MESSAGE_IN;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_set_mask_std(uint8_t IdMask, uint32_t Mask){
    uint8_t MasqueH,MasqueL;
    if ( (IdMask != 0) && (IdMask !=1) )  return BAD_MASK_ID;
    if ( Mask > 0x07FF ) return BAD_MASK_VALUE;

    MasqueH = (Mask >> 3) & 0x00FF;
    MasqueL = (Mask << 5) & 0x00E0;

    switch (IdMask)
        {
        case 0 :
            RXM0SIDH = MasqueH;
            RXM0SIDL = MasqueL;
            break;
        case 1 :
            RXM1SIDH = MasqueH;
            RXM1SIDL = MasqueL;
            break;
        default : return BAD_MASK_ID;break;
        }
    return CAN_OK;
}
//----------------------------------------------------------------------------
extern  can_status_t    can_ll_set_mask_ext(uint8_t IdMask, uint32_t Mask){
    if ( (IdMask != 0) && (IdMask !=1) )  return BAD_MASK_ID;
    if ( Mask > 0x1FFFFFFF ) return BAD_MASK_VALUE;

    if (IdMask == 0)
        {
        RXM0SIDH = ( Mask >> 21 ) & 0x000000FF ;
        RXM0SIDL = ( Mask >> 13 ) & 0x000000E0 ;
        RXM0SIDL |= ( (Mask >> 16) & 0x00000003);
        RXM0EIDL = ( Mask & 0x000000FF );
        RXM0EIDH = ( Mask >> 8 ) & 0x000000FF ;
        }
    else
        {
        RXM1SIDH = ( Mask >> 21 ) & 0x000000FF ;
        RXM1SIDL = ( Mask >> 13 ) & 0x000000E0 ;
        RXM1SIDL |=( (Mask >> 16) & 0x00000003);
        RXM1EIDL = ( Mask & 0x000000FF );
        RXM1EIDH = ( Mask >> 8 ) & 0x000000FF ;
        }
    return CAN_OK;
}

extern  can_status_t    can_ll_set_filter_std(filter_id_t filterID, uint32_t Filter){
    uint8_t FiltreH,FiltreL;

    if ( filterID > 4)  return BAD_FILTER_ID;
    if ( Filter > 0x07FF ) return BAD_FILTER_VALUE;

    FiltreH = (Filter >> 3) & 0x00FF;
    FiltreL = (Filter << 5) & 0x00E0;

    switch (filterID){
        case FILTER_ID0 :
            RXF0SIDH = FiltreH;
            RXF0SIDL = FiltreL;
            break;
        case FILTER_ID1 :
            RXF1SIDH = FiltreH;
            RXF1SIDL = FiltreL;
            break;

        case FILTER_ID2 :
            RXF2SIDH = FiltreH;
            RXF2SIDL = FiltreL;
            break;

        case FILTER_ID3 :
            RXF3SIDH = FiltreH;
            RXF3SIDL = FiltreL;
            break;

        case FILTER_ID4 :
            RXF4SIDH = FiltreH;
            RXF4SIDL = FiltreL;
            break;
        
        default : 
            return BAD_FILTER_ID;
    }

    return CAN_OK;
}
extern  can_status_t    can_ll_set_filter_ext(filter_id_t filterID, uint32_t Filter){
    if ( filterID > 4)  return BAD_FILTER_ID;
    if ( Filter > 0x1FFFFFFF ) return BAD_FILTER_VALUE;

    switch (filterID){
        case FILTER_ID0 : 
            RXF0SIDH = ( Filter >> 21 ) & 0x000000FF ;
            RXF0SIDL = ( Filter >> 13 ) & 0x000000E0 ;
            RXF0SIDL |= ( (Filter >> 16) & 0x00000003);
            RXF0SIDL |= 0x08;
            RXF0EIDL = ( Filter & 0x000000FF );
            RXF0EIDH = ( Filter >> 8 ) & 0x000000FF ;
            break;

        case FILTER_ID1 :
            RXF1SIDH = ( Filter >> 21 ) & 0x000000FF ;
            RXF1SIDL = ( Filter >> 13 ) & 0x000000E0 ;
            RXF1SIDL |= ( (Filter >> 16) & 0x00000003);
            RXF1SIDL |= 0x08;
            RXF1EIDL = ( Filter & 0x000000FF );
            RXF1EIDH = ( Filter >> 8 ) & 0x000000FF ;
            break;

        case FILTER_ID2 :
            RXF2SIDH = ( Filter >> 21 ) & 0x000000FF ;
            RXF2SIDL = ( Filter >> 13 ) & 0x000000E0 ;
            RXF2SIDL |= ( (Filter >> 16) & 0x00000003);
            RXF2SIDL |= 0x08;
            RXF2EIDL = ( Filter & 0x000000FF );
            RXF2EIDH = ( Filter >> 8 ) & 0x000000FF ;
            break;

        case FILTER_ID3 :
            RXF3SIDH = ( Filter >> 21 ) & 0x000000FF ;
            RXF3SIDL = ( Filter >> 13 ) & 0x000000E0 ;
            RXF3SIDL |= ( (Filter >> 16) & 0x00000003);
            RXF3SIDL |= 0x08;
            RXF3EIDL = ( Filter & 0x000000FF );
            RXF3EIDH = ( Filter >> 8 ) & 0x000000FF ;
            break;

        case FILTER_ID4 :
            RXF4SIDH = ( Filter >> 21 ) & 0x000000FF ;
            RXF4SIDL = ( Filter >> 13 ) & 0x000000E0 ;
            RXF4SIDL |= ( (Filter >> 16) & 0x00000003);
            RXF4SIDL |= 0x08;
            RXF4EIDL = ( Filter & 0x000000FF );
            RXF4EIDH = ( Filter >> 8 ) & 0x000000FF ;
            break;

        default : return CAN_ERROR;
    }
    return CAN_OK;
}