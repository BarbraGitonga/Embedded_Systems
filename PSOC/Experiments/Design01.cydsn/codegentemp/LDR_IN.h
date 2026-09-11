/*******************************************************************************
* File Name: LDR_IN.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_LDR_IN_H) /* Pins LDR_IN_H */
#define CY_PINS_LDR_IN_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "LDR_IN_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 LDR_IN__PORT == 15 && ((LDR_IN__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    LDR_IN_Write(uint8 value);
void    LDR_IN_SetDriveMode(uint8 mode);
uint8   LDR_IN_ReadDataReg(void);
uint8   LDR_IN_Read(void);
void    LDR_IN_SetInterruptMode(uint16 position, uint16 mode);
uint8   LDR_IN_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the LDR_IN_SetDriveMode() function.
     *  @{
     */
        #define LDR_IN_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define LDR_IN_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define LDR_IN_DM_RES_UP          PIN_DM_RES_UP
        #define LDR_IN_DM_RES_DWN         PIN_DM_RES_DWN
        #define LDR_IN_DM_OD_LO           PIN_DM_OD_LO
        #define LDR_IN_DM_OD_HI           PIN_DM_OD_HI
        #define LDR_IN_DM_STRONG          PIN_DM_STRONG
        #define LDR_IN_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define LDR_IN_MASK               LDR_IN__MASK
#define LDR_IN_SHIFT              LDR_IN__SHIFT
#define LDR_IN_WIDTH              1u

/* Interrupt constants */
#if defined(LDR_IN__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in LDR_IN_SetInterruptMode() function.
     *  @{
     */
        #define LDR_IN_INTR_NONE      (uint16)(0x0000u)
        #define LDR_IN_INTR_RISING    (uint16)(0x0001u)
        #define LDR_IN_INTR_FALLING   (uint16)(0x0002u)
        #define LDR_IN_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define LDR_IN_INTR_MASK      (0x01u) 
#endif /* (LDR_IN__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define LDR_IN_PS                     (* (reg8 *) LDR_IN__PS)
/* Data Register */
#define LDR_IN_DR                     (* (reg8 *) LDR_IN__DR)
/* Port Number */
#define LDR_IN_PRT_NUM                (* (reg8 *) LDR_IN__PRT) 
/* Connect to Analog Globals */                                                  
#define LDR_IN_AG                     (* (reg8 *) LDR_IN__AG)                       
/* Analog MUX bux enable */
#define LDR_IN_AMUX                   (* (reg8 *) LDR_IN__AMUX) 
/* Bidirectional Enable */                                                        
#define LDR_IN_BIE                    (* (reg8 *) LDR_IN__BIE)
/* Bit-mask for Aliased Register Access */
#define LDR_IN_BIT_MASK               (* (reg8 *) LDR_IN__BIT_MASK)
/* Bypass Enable */
#define LDR_IN_BYP                    (* (reg8 *) LDR_IN__BYP)
/* Port wide control signals */                                                   
#define LDR_IN_CTL                    (* (reg8 *) LDR_IN__CTL)
/* Drive Modes */
#define LDR_IN_DM0                    (* (reg8 *) LDR_IN__DM0) 
#define LDR_IN_DM1                    (* (reg8 *) LDR_IN__DM1)
#define LDR_IN_DM2                    (* (reg8 *) LDR_IN__DM2) 
/* Input Buffer Disable Override */
#define LDR_IN_INP_DIS                (* (reg8 *) LDR_IN__INP_DIS)
/* LCD Common or Segment Drive */
#define LDR_IN_LCD_COM_SEG            (* (reg8 *) LDR_IN__LCD_COM_SEG)
/* Enable Segment LCD */
#define LDR_IN_LCD_EN                 (* (reg8 *) LDR_IN__LCD_EN)
/* Slew Rate Control */
#define LDR_IN_SLW                    (* (reg8 *) LDR_IN__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define LDR_IN_PRTDSI__CAPS_SEL       (* (reg8 *) LDR_IN__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define LDR_IN_PRTDSI__DBL_SYNC_IN    (* (reg8 *) LDR_IN__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define LDR_IN_PRTDSI__OE_SEL0        (* (reg8 *) LDR_IN__PRTDSI__OE_SEL0) 
#define LDR_IN_PRTDSI__OE_SEL1        (* (reg8 *) LDR_IN__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define LDR_IN_PRTDSI__OUT_SEL0       (* (reg8 *) LDR_IN__PRTDSI__OUT_SEL0) 
#define LDR_IN_PRTDSI__OUT_SEL1       (* (reg8 *) LDR_IN__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define LDR_IN_PRTDSI__SYNC_OUT       (* (reg8 *) LDR_IN__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(LDR_IN__SIO_CFG)
    #define LDR_IN_SIO_HYST_EN        (* (reg8 *) LDR_IN__SIO_HYST_EN)
    #define LDR_IN_SIO_REG_HIFREQ     (* (reg8 *) LDR_IN__SIO_REG_HIFREQ)
    #define LDR_IN_SIO_CFG            (* (reg8 *) LDR_IN__SIO_CFG)
    #define LDR_IN_SIO_DIFF           (* (reg8 *) LDR_IN__SIO_DIFF)
#endif /* (LDR_IN__SIO_CFG) */

/* Interrupt Registers */
#if defined(LDR_IN__INTSTAT)
    #define LDR_IN_INTSTAT            (* (reg8 *) LDR_IN__INTSTAT)
    #define LDR_IN_SNAP               (* (reg8 *) LDR_IN__SNAP)
    
	#define LDR_IN_0_INTTYPE_REG 		(* (reg8 *) LDR_IN__0__INTTYPE)
#endif /* (LDR_IN__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_LDR_IN_H */


/* [] END OF FILE */
