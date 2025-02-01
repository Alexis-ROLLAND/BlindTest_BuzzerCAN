/**
 * @file    lib_bus_CAN_K80_v3.h 
 * @author 	Alexis ROLLAND
 * @date	2024/10
 ** @brief  Header file for the CAN bus communication library.
 * 
 * This header file defines the functions, data structures, and types used for managing
 * communication over the CAN bus. It includes declarations for initializing the CAN,
 * transmitting and receiving frames, configuring filters, and handling error states.
 *   
 * @version 3.0
 *        
 *
 * Modifié le  	:
 *	21/10/2006 : 	Ajout de la gestion du bit RTR dans les trames CAN
 * 	25/10/2006 : 	Version 2.0
 *					Possibilité de configurer les vitesse au niveau du bus 
 *					en fonction du quartz.
 *					Les débits possibles en fonction du Quartz sont présentés
 *					dans le tableau :
 *						X : Débit supporté
 *						- : Débit non supporté
 *						! : Débit supporté, mais risques d'erreurs de comm.
 *						[Vide] : Débit supporté dans de prochaines versions
 *					
 *					| 10K | 20K | 50K | 100K | 125K | 250K | 500K | 800K |1M|  
 *				4MHz|  X  |	 X	|  X  |  X   |	-	|	-  |   -  |	 -	 |- |
 *				8MHz|  X  |	 X	|  X  |  X   |	X	|	X  |   !  |	 -	 |- |
 *			   10MHz|	  
 *			   16MHz|  X  |	 X	|  X  |  X   |	X	|	X  |   X  |	 !	 |! |
 *			   20MHz|
 *			   32MHz|  X  |	 X	|  X  |  X   |	X	|	X  |   X  |	 X	 |X |
 *			   40MHz| 	
 *	25/02/2007 :	Modification de la constante OK, qui devient CAN_OK
 *					Modification de la constante ERREUR, qui devient CAN_ERREUR							
 *					Correction des fonctions de programmation des filtres		
 *
 * 	09/03/2007 :	Ajout de la fonction "ProgrammerFiltreReceptionGeneral"
 *					Suppression de la fonction InitECAN_Base
 *					Mise au point des fonctions de filtrage pour trames 
 *					étendues : ProgrammerMasqueEtendu et ProgrammerFiltreEtendu
 *
 *	07/06/2007 :	Correction du masque ligne 290 (0x000000FF remplacer par 0x0000000E0
 *				posant des problèmes d'erreur d'identifiants dans la trame émise
 *				Même action pour les fonctions ProgrammerFiltreEtendu et ProgrammerMasqueEtendu
 * 03/03/2017 : Portage vers 18FxxK80 
 * 
 * 09/2024 : Réactivation projet - MaJ avec nouvelles normes de développement + compatibilité XC8 (v2.50)
*/


#ifndef	__BUS_CAN_H__
#define __BUS_CAN_H__

#include <xc.h>
#include <stdint.h>

/* Déclarations des équivalences #define 	*/
#define	CAN_TX	                LATBbits.LATB2  /** CAN_TX port line */
#define	CAN_RX	                PORTBbits.RB3   /** CAN_RX port line */

#ifndef	NULL
#define		NULL				(void*)0
#endif

/**
 * @brief Enumeration representing the status codes for CAN operations.
 * 
 * This enumeration defines the various status codes that can be returned by
 * functions handling CAN bus operations, indicating success, specific errors,
 * and the state of the operation.
 */
typedef enum{
    CAN_OK = 0,                 /**< Operation successfull  */              
    CAN_ERROR = -1,             /**< Non specific CAN error */
    BAD_BUFFER_ID = -10,        /**< The buffer ID identifier is invalid */
	TOO_MUCH_DATA = -11,        /**< The frame exceeds the allowed data size */
    LOST_ARBITRATION = -12,     /**< Tx has lost arbitration    */
    BAD_FRAME_FORMAT = -13,     /**< The frame has an invalid format */
    BAD_FRAME_ID = -14,         /**< The frame contains an invalid identifier */
    BAD_MASK_VALUE = -20,       /**< The mask value specified in the filter is invalid  */
    BAD_FILTER_VALUE = -21,     /**< The filter value specified is invalid */
    BAD_FILTER_ID = -22,        /**< The filter ID specified is invalid */
    BAD_MASK_ID = -23,          /**< The mask ID specified is invalid   */
    BAD_FILTER_TYPE = -24,      /**< The specified filter type is invalid   */
    
    NO_MESSAGE = -30,           /**< No message was received    */
    MESSAGE_IN = 1,             /**< A message was successfully received    */
            
    RX_FILTER0 = 10,            /**< The last received frame matched filter 0 */
    RX_FILTER1 = 11,            /**< The last received frame matched filter 1 */
    RX_FILTER2 = 12,            /**< The last received frame matched filter 2 */
    RX_FILTER3 = 13,            /**< The last received frame matched filter 3 */
    RX_FILTER4 = 14             /**< The last received frame matched filter 4 */
    
} can_status_t;

/**
 * @brief Enumeration representing the accepted CAN frames for a RX Buffer
 */
typedef enum{
    STD_FRAMES_ONLY = 0,        /**< The global filter is set to accept only standard CAN frames    */
    EXT_FRAMES_ONLY	= 1,        /**< The global filter is set to accept only extended CAN frames    */
    BOTH_FRAMES = 2             /**< The global filter is set to accept both CAN frames    */
} filter_type_t;

/**
 * @brief Enumeration representing the RTR bit value
 */
typedef enum{
    RTR_ON = 0x01,  /**< RTR is recessive -> Remote Frame   */
    RTR_OFF = 0x00  /**< RTR is dominant -> Data Frame  */
} rtr_value_t;

#define		MASK_RTR_ON			0x40    /** Mask to set RTR recessive value  (TX frame) */
#define		MASK_RTR_OFF		0xBF    /** Mask to set RTR dominant value  (TX frame) */
#define		MASK_RX_RTR			0x08    /** Mask to get RTR value (RX frame) */

#define     STD_MAX_ID          0x7FF
#define     EXT_MAX_ID          0x1FFFFFFF
#define     DLC_MAX             8
/**
 * @brief Enumeration representing various oscillator frequencies 
 */
typedef enum {
    MHZ4,   /**< Oscillator Frequency is 4 MHz  */
    MHZ8,   /**< Oscillator Frequency is 8 MHz  */
    MHZ10,  /**< Oscillator Frequency is 10 MHz  */
    MHZ16,  /**< Oscillator Frequency is 16 MHz  */
    MHZ20,  /**< Oscillator Frequency is 20 MHz  */
    MHZ32,  /**< Oscillator Frequency is 32 MHz  */
    MHZ40   /**< Oscillator Frequency is 40 MHz  */
} osc_freq_t ;

/**
 * @brief Enumeration representing various bitrates for the CAN bus
 */
typedef enum {
    KBPS10,     /**< Bitrate is 10kbps  */
    KBPS20,     /**< Bitrate is 20kbps  */
    KBPS50,     /**< Bitrate is 50kbps  */
    KBPS100,    /**< Bitrate is 100kbps  */
    KBPS125,    /**< Bitrate is 125kbps  */
    KBPS250,    /**< Bitrate is 250kbps  */
    KBPS500,    /**< Bitrate is 500kbps  */
    KBPS800,    /**< Bitrate is 800kbps  */
    KBPS1000    /**< Bitrate is 1000kbps  */
} bus_speed_t;

/**
 * @brief Enumeration representing the frame type
 */
typedef enum {
	STD_FRAME,      /**< Standard frame */
	EXT_FRAME       /**< Extended frame */
} type_trame_t; 

/**
 * @brief Enumeration representing the TX buffer to use for a transmit    
 */
typedef enum {
    TX_BUFFER0,     /**< Use TX Buffer 0    */
    TX_BUFFER1,     /**< Use TX Buffer 1    */
    TX_BUFFER2      /**< Use TX Buffer 2    */
} tx_buffer_t;

/**
 * @brief Enumeration representing the RX buffer to check/configure for a reception
 */
typedef enum{
    RX_BUFFER0 = 0, /**< Use RX Buffer 0    */
    RX_BUFFER1 = 1  /**< Use RX Buffer 1    */
} rx_buffer_t;

/**
 * @brief Enumeration representing the ID filter to configure
 */
typedef enum{
    FILTER_ID0 = 0, /**< Filter is FILTER0  */   
    FILTER_ID1 = 1, /**< Filter is FILTER1  */
    FILTER_ID2 = 2, /**< Filter is FILTER2  */
    FILTER_ID3 = 3, /**< Filter is FILTER3  */
    FILTER_ID4 = 4  /**< Filter is FILTER4  */
} filter_id_t;

/**
 * @brief Structure representing a CAN acceptance filter configuration.
 * 
 * This structure defines the configuration parameters for a CAN acceptance filter,
 * including the reception buffer, filter ID, frame type, and mask and filter values.
 */
typedef struct{
    rx_buffer_t     rxBuffer;       /**<  RXBUFFER0 or RX_BUFFER1 linked to MASKID0 or MASKID1  */
    filter_id_t     filterId;       /**< Filter ID from 0 to 4 (0,1 : MASKID0; 2,3,4 : MASKID1 */
    type_trame_t    TypeTrame;     /**< Frame type : STD / EXT */
    uint32_t        MaskValue;      /**< Mask Value : sets the bits to examine  */
    uint32_t        FilterValue;    /**< Filter Value : Values of the bits examined */ 
} filter_structure_t;
    
/**
 * @brief Structure representing a CAN frame.
 * 
 * This structure defines the parameters of a CAN frame, including the frame type,
 * identifier, size, remote transmission request (RTR) bit, and a pointer to the data buffer.
 */
typedef struct{
    type_trame_t    TypeTrame;
	uint32_t        FrameID;
	uint8_t         Taille;
	rtr_value_t     RtrBit;
	uint8_t*        AdrBuffer;
} can_frame_t;

//------------------------------------------------------------------------------

/**
 * @brief Initializes the CAN network with the specified frequency and baud rate parameters.
 * 
 * This function configures the CAN controller based on the specified oscillator frequency
 * and CAN bus speed. It prepares the CAN network to be ready for use.
 * 
 * @param Frequence  Oscillator frequency (of type osc_freq_t) used to configure the CAN.
 * @param Debit      CAN bus speed (of type bus_speed_t) used to establish communication.
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the success or failure of the initialization.
 * 
 * @retval CAN_OK    The initialization was successful.
 * @retval CAN_ERROR An error occurred during the initialization.
 */
can_status_t    can_init(osc_freq_t Frequence,bus_speed_t Debit);

/**
 * @brief Transmits a CAN frame using the specified transmission buffer.
 * 
 * This function sends a CAN frame through the transmission buffer identified by txBuffer.
 * The frame to be sent is provided through the pointer pFrame.
 * 
 * @param txBuffer   Identifier of the transmission buffer (of type tx_buffer_t).
 * @param pFrame     Pointer to the CAN frame (of type can_frame_t) to be transmitted.
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the result of the transmission.
 * 
 * @retval CAN_OK            The frame was successfully transmitted.
 * @retval CAN_ERROR         An error occurred during transmission.
 * @retval TOO_MUCH_DATA     The frame exceeds the allowed data size.
 * @retval BAD_FRAME_ID      The frame contains an invalid identifier.
 * @retval BAD_FRAME_FORMAT  The frame has an invalid format.
 * @retval BAD_BUFFER_ID     The transmission buffer identifier is invalid.
 */
can_status_t    can_Emit(tx_buffer_t txBuffer, const can_frame_t *pFrame);

/**
 * @brief Receives a CAN frame from the specified reception buffer.
 * 
 * This function retrieves a CAN frame from the reception buffer identified by rxBuffer.
 * The received frame is stored in the structure pointed to by pFrame.
 * 
 * @param rxBuffer   Identifier of the reception buffer (of type rx_buffer_t).
 * @param pFrame     Pointer to the CAN frame structure (of type can_frame_t) where the received frame will be stored.
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the result of the reception.
 * 
 * @retval MESSAGE_IN        A message was successfully received.
 * @retval NO_MESSAGE        No message was received.
 * @retval BAD_FRAME_FORMAT  The received frame has an invalid format.
 * @retval BAD_BUFFER_ID     The reception buffer identifier is invalid.
 */
can_status_t    can_Receive(rx_buffer_t rxBuffer, can_frame_t *pFrame);

/**
 * @brief Configures the global filter for the specified reception buffer.
 * 
 * This function sets a global filter on the CAN reception buffer identified by rxBuffer,
 * based on the specified filter type (filterType).
 * 
 * @param rxBuffer   Identifier of the reception buffer (of type rx_buffer_t).
 * @param filterType Type of filter to apply (of type filter_type_t).
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the result of the filter configuration.
 * 
 * @retval CAN_OK            The filter was successfully applied.
 * @retval BAD_FILTER_TYPE   The specified filter type is invalid.
 * @retval BAD_BUFFER_ID     The reception buffer identifier is invalid.
 */
can_status_t    can_setGlobalFilter(rx_buffer_t rxBuffer, filter_type_t filterType);

/**
 * @brief Configures the acceptance filter for incoming CAN frames.
 * 
 * This function sets an acceptance filter using the filter structure pointed to by pFilter.
 * The filter is applied to incoming CAN frames to determine which frames are accepted.
 * 
 * @param pFilter  Pointer to the filter structure (of type filter_structure_t) containing the filter settings.
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the result of the filter configuration.
 * 
 * @retval CAN_OK            The acceptance filter was successfully configured.
 * @retval CAN_ERROR         An error occurred while configuring the filter.
 * @retval BAD_FRAME_FORMAT  The frame format specified in the filter is invalid.
 * @retval BAD_MASK_VALUE    The mask value specified in the filter is invalid.
 * @retval BAD_FILTER_VALUE  The filter value specified is invalid.
 */
can_status_t    can_setAcceptanceFilter(const filter_structure_t *pFilter);

/**
 * @brief Retrieves the filter that matched the last received CAN frame in the specified reception buffer.
 * 
 * This function returns the identifier of the filter that matched the most recent CAN frame 
 * received in the reception buffer identified by rxBuffer. The function can return the filter 
 * number (from RX_FILTER0 to RX_FILTER4) or an error status.
 * 
 * @param rxBuffer  Identifier of the reception buffer (of type rx_buffer_t).
 * 
 * @return can_status_t  Returns a status of type can_status_t indicating the result of the operation.
 * 
 * @retval RX_FILTER0     The last received frame matched filter 0.
 * @retval RX_FILTER1     The last received frame matched filter 1.
 * @retval RX_FILTER2     The last received frame matched filter 2.
 * @retval RX_FILTER3     The last received frame matched filter 3.
 * @retval RX_FILTER4     The last received frame matched filter 4.
 * @retval CAN_ERROR      An error occurred during the operation.
 * @retval BAD_BUFFER_ID  The reception buffer identifier is invalid.
 */
can_status_t    can_getFilterMatch(rx_buffer_t rxBuffer);

//------------------------------------------------------------------------------
/**
 *  LOW LEVEL FUNCTIONS   
 */
extern  can_status_t    can_ll_sendStd_b0(const can_frame_t *pFrame);
extern  can_status_t    can_ll_sendExt_b0(const can_frame_t *pFrame);

extern  can_status_t    can_ll_sendStd_b1(const can_frame_t *pFrame);
extern  can_status_t    can_ll_sendExt_b1(const can_frame_t *pFrame);

extern  can_status_t    can_ll_sendStd_b2(const can_frame_t *pFrame);
extern  can_status_t    can_ll_sendExt_b2(const can_frame_t *pFrame);

extern  can_status_t    can_ll_Receive_b0(can_frame_t *pFrame);
extern  can_status_t    can_ll_Receive_b1(can_frame_t *pFrame);

extern  can_status_t    can_ll_set_mask_std(uint8_t IdMask, uint32_t Mask);
extern  can_status_t    can_ll_set_mask_ext(uint8_t IdMask, uint32_t Mask);

extern  can_status_t    can_ll_set_filter_std(filter_id_t filterID, uint32_t Filter);
extern  can_status_t    can_ll_set_filter_ext(filter_id_t filterID, uint32_t Filter);
#endif
