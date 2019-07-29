; *****************************************************************************
; * © 2007 Microchip Technology Inc.
; *
; * FileName:        Traps.s
; * Dependencies:    Header (.inc) files if applicable, see below
; * Processor:       dsPIC30F2020
; * Compiler:        MPLAB® C30 v3.00 or higher
; * IDE:             MPLAB® IDE v7.51 or later
; * Dev. Board Used: Sync Buck Converter Using SMPS dsPIC
; *
; * SOFTWARE LICENSE AGREEMENT:
; * Microchip Technology Inc. (“Microchip”) licenses this software to you
; * solely for use with Microchip dsPIC® digital signal controller
; * products. The software is owned by Microchip and is protected under
; * applicable copyright laws.  All rights reserved.
; *
; * SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
; * WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
; * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
; * PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
; * BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
; * DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
; * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
; * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
; * ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
; *
; * REVISION HISTORY:
; *****************************************************************************
; * Author            Date      Comments on this revision
; *****************************************************************************
; * Antonio Bersani		10/22/07	First release
; *****************************************************************************
;	Notes:
;
;                                                                             
;******************************************************************************
		.include	"p33EP32GS504.inc"
							
		.global  __OscillatorFail
		.global  __AddressError
		.global  __StackError
		.global  __MathError
							
		.global  __AltOscillatorFail
		.global  __AltAddressError
		.global  __AltStackError
		.global  __AltMathError


         	.text

; Default Exception Vector handlers if ALTIVT(INTCON2<15>) = 0

; Oscillator Fail Trap
__OscillatorFail:
		bclr	INTCON1, #OSCFAIL
		nop
OFLabel:
		bra	OFLabel
		retfie

; Address Error Trap
__AddressError:
		bclr	INTCON1, #ADDRERR
		nop
		nop
		nop
		reset
ADLabel:
		bra	ADLabel
		retfie

; Stack Error Trap
__StackError:
		bclr	INTCON1, #STKERR
		nop
SELabe1:
		bra	SELabe1
		retfie

; Math Error Trap
__MathError:
		bclr	INTCON1, #MATHERR
		nop
MELabel:
		bra	MELabel
		retfie


; Alternate Exception Vector handlers if ALTIVT(INTCON2<15>) = 1

; Alternate Oscillator Fail Trap
__AltOscillatorFail:
		bclr	INTCON1, #OSCFAIL
		nop
AOFLabel:
		bra	AOFLabel
		retfie

; Alternate Address Error Trap
__AltAddressError:
		bclr	INTCON1, #ADDRERR
		nop
AAELabel:
		bra	AAELabel
		retfie

; Alternate Stack Error Trap
__AltStackError:
		bclr	INTCON1, #STKERR
		nop
ASELabel:
		bra	ASELabel
		retfie

; Alternate Math Error Trap
__AltMathError:
		bclr	INTCON1, #MATHERR
		nop
AMELabel:
		bra	AMELabel
		retfie



;--------End of All Code Sections ---------------------------------------------

        .end                               ;End of program code in this file



