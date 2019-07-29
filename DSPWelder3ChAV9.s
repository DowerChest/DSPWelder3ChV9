    .ifndef ffunction
    .section .init,code
    .else
    .section .init.resetALT, code
    .endif
    .global __resetALT
    .ifdef __C30ELF
    .type   __resetALT,@function
    .endif
__resetALT:
    .weak  __reset
    .ifdef __C30ELF
    .type   __reset,@function
    .endif
__reset:
    .global	START
    goto	START
    .pword 0xDA4000			; halt the simulator
    reset				; reset the processor
    .include "null_signature.s"
    .include "DSPWelder3ChAV9.inc"
    .section .usercode, code
;------------------------------------------------------------------------------------------------
;������������� ��������� ������������ ��
    .global	START
START:		
    mov      #__SP_init, W15		; initialize w15
    mov      #__SPLIM_init, W14		;
    mov      W14, _SPLIM		; initialize SPLIM
    nop					; wait 1 cycle
    mov      #0x0010, W0
    mov      W0, CORCON			; enable super saturation and
    mov      #__const_length, W0	;
    cp0      W0				; test length of constants
    bra      z, CONST_INIT		; br if zero
    mov      #__const_psvpage, W0	;
    mov      W0, _DSRPAG		; DSRPAG = psvpage(constants)
CONST_INIT:
    bset     INTCON1,#NSTDIS            ;
    mov      #0x00e0,W0                 ;
    ior      SR                         ;
;RA0 - ���������� ���� ����������:1
;RA1 - ���������� ���� ����:1
;RA2 - ���������� ���� ������������� ������� ���������� :1
;RA3 - PWM1H (����):1
;RA4 - PWM1L (����):1
;TRISA=1111 1111 1111 1111
    mov	    #0xffff,W0
    mov     W0,TRISA
    mov	    #0x0007,W0
    mov	    W0,ANSELA
    clr     ODCA
    clr	    CNENA
    clr	    LATA
;RB0 - RP32 -> SPI1 SCK (����):1
;RB1 - ���� �������� ���������� (����):1
;RB2 - TMP1 - ���� ����������� 1-�� ������ (����):1
;RB3 - TMP2 - ���� ����������� 2-�� ������ (����):1
;RB4 - TMP3 - ���� ����������� 3-�� ������ (����):1
;RB5 - RP37 -> SPI2 SDI (MISO) ���� ������ SPI2 :1 
;RB6 - PGED1 (�����):0
;RB7 - PGEC1 (�����):0
;RB8 - RP40 -> SPI2 SCK (����):1
;RB9 - RP41 -> SPI1 SDO (����):1
;RB10 - RP42 -> SPI1 SDI (����):1
;RB11 - RP43 -> IC1 ���� ��� ���������� ����� ���������� � ������ MIG (����):1
;RB12 - RP44 -> OC1 ����� ��� ���������� ������� � ������ MIG (����): 1
;RB13 - PWM2H (����):1
;RB14 - PWM2L (����):1
;RB15 - RP47 -> SPI2 SDO (����):1
;TRISB=1111 1111 1111 1111
    mov     #0xffff,W0
    mov     W0,TRISB
    mov	    #0x001c, W0
    mov	    W0, CNPUB
    clr	    ANSELB
    clr     ODCB
    clr	    CNENB
    clr     LATB
;RC0 - SPB - ������ ������ ������ "���/���/���" (����):1
;RC1 - �� ������������ (�����):0
;RC2 - TMP4 - ���� ����������� 4-�� ������ (����):1
;RC3 - VLV - ����� �� ������ ���� (�����):0
;RC4 - EN# - ����� ���������� ����������� (�����):0
;RC5 - MIGBut - ���� ������ �� ������� (����):1
;RC6 - �� ������������ (�����):0
;RC7 - HPG - ���� "������� ���������� � �����" (����):1
;RC8 - LATCH - ����� ������� ���������� (�����):0
;RC9 - CS - ����� CS EEPROM (�����):0
;RC10 - MemB - ���� ������ "������" (����):1
;RC11 - GasB - ���� ������ "������������ ���� ����" (����):1
;RC12 - SelB - ���� ������ "����� ����������� ���������� ���������" (����):1
;RC13 - T24B - ���� ������ "T2/T4" (����):1
;TRISC=1111 1100 1010 0101
    mov     #0xfca5,W0
    mov     W0,TRISC
    mov	    #0x3ca5, W0
    mov	    W0, CNPUC
    clr	    ANSELC
    clr     ODCC
    clr	    CNENB
    clr     LATC
    SetDM13En
;������������� ������������ ��������������� �������	
    mov     #OSCCON, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   W2, [W1]
    mov.b   W3, [W1]
    bclr    OSCCON, #IOLOCK
;���������������� �������������� ������
    mov     #0x0000,W0
    mov     W0,RPINR0
    mov     #0x0000,W0
    mov     W0,RPINR1
    mov     #0x0000,W0
    mov     W0,RPINR2
    mov     #0x0000,W0
    mov     W0,RPINR3
    mov     #0x002b,W0 ;IC1 -> RP43
    mov     W0,RPINR7
    mov     #0x0000,W0
    mov     W0,RPINR11
    mov     #0x0000,W0
    mov     W0,RPINR18
    mov     #0x0000,W0
    mov     W0,RPINR19
    mov     #0x2029,W0 ; SPI1 SDI -> RP41, SPI1 SCK -> RP32
    mov     W0,RPINR20
    mov     #0x0000,W0
    mov     W0,RPINR21
    mov	    #0x2825,W0
    mov	    W0,RPINR22 ; SPI2 SDI -> RP37, SPI2 SCK -> RP40
    mov	    #0x0000,W0
    mov	    W0,RPINR23
    mov	    #0x0000,W0
    mov	    W0,RPINR37
    mov	    #0x0000,W0
    mov	    W0,RPINR38
    mov	    #0x0000,W0
    mov	    W0,RPINR42
    mov	    #0x0000,W0
    mov	    W0,RPINR43
;���������������� �������������� �������
    mov     #0x0006,W0 ; SPI1 SCK -> RP32
    mov     W0,RPOR0
    mov     #0x0000,W0
    mov     W0,RPOR1
    mov     #0x0000,W0 
    mov     W0,RPOR2
    mov	    #0x0000,W0
    mov     W0,RPOR3
    mov     #0x0009,W0 ; SPI2 SCK -> RP40
    mov     W0,RPOR4
    mov     #0x0005,W0 ; SPI1 SDO  -> RP42
    mov     W0,RPOR5
    mov     #0x0010,W0 ; OC1 -> RP44
    mov     W0,RPOR6
    mov     #0x0800,W0 ; SPI2 SDO -> RP47
    mov     W0,RPOR7
    mov     #0x0000,W0
    mov     W0,RPOR8
    mov     #0x0000,W0
    mov     W0,RPOR9
    mov     #0x0000,W0
    mov     W0,RPOR10
    mov     #0x0000,W0
    mov     W0,RPOR11
    mov     #0x0000,W0
    mov     W0,RPOR12
    mov     #0x0000,W0
    mov     W0,RPOR13
    mov     #0x0000,W0
    mov     W0,RPOR14
    mov     #0x0000,W0
    mov     W0,RPOR16
    mov     #0x0000,W0
    mov     W0,RPOR17
;���������� ������������ ��������������� �������	
    mov     #OSCCON, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   W2, [W1]
    mov.b   W3, [W1]
    bset    OSCCON, #IOLOCK
;Fcy=Fosc/2=1/2((Fin*M)/(N1*N2))=1/2((8 000 000*60)/(2*2))=60 000 000 ��
;����� �������� �=60 (PLLFBD=58)
    mov     #0x003A, W0
    mov     W0, PLLFBD
;����� �������� N2=2 (PLLPOST=0), ����� �������� N1=2 (PLLPRE=0)
    mov     #0x0000, W0
    mov     W0, CLKDIV
;���������� ������ ���� ���������� (NOSC=0b011 ��������� ��������� � ����) � W0
    mov     #0x0003,W0
;������������� OSCCONH (������� ����) 
    mov     #OSCCONH, W1
    mov     #0x78, W2
    mov     #0x9A, W3
    mov.b   W2, [W1]		; ������ 0x0078
    mov.b   W3, [W1]		; ������ 0x009A
;��������� ������ ���� ����������
    mov.b   WREG, OSCCONH
;������������� OSCCONL (������� ����)
    mov     #OSCCONL, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   w2,[W1]		;������ 0x0046
    mov.b   w3,[W1]		;������ 0x0057
;���������� ����������� ������������
    bset    OSCCON, #OSWEN      ;������ �� ������������ ������������ ���������� ���� OSWEN bit
;�������� ���������� ����������� ������������
WaitOSCSwitch:	
    btsc    OSCCON, #OSWEN      ;
    bra	    WaitOSCSwitch       ;
;�������� ���������� �������� PLL
WaitMainPLLLock:
    btss    OSCCON,#LOCK	;
    bra	    WaitMainPLLLock     ;
;��������� ���������� ���������� ��� ��� � ��� � �������������� ����������� ���������� � �������� REFCLK
;ACLK=(Fin*M)/N=(7 370 000 * 16)/1 = 117 920 000
    bclr    ACLKCON,#ASRCSEL	;�������� ��������� - ��������� ���������
    bset    ACLKCON,#FRCSEL	;�������� ��������� ��������� ���������� ���� ARCSEL
    bset    ACLKCON,#SELACLK	;���������� - �������� ��� ��������
    bset    ACLKCON,#APSTSCLR2	;
    bset    ACLKCON,#APSTSCLR1	;�������� ���������� 1:1
    bset    ACLKCON,#APSTSCLR0	;
    bset    ACLKCON,#ENAPLL	;�������������� ��������� ���������� �������
;�������� ���������� ��������� �������������� ����
WaitAuxPLLLock:
    btss    ACLKCON,#APLLCK	;
    bra     WaitAuxPLLLock	;
;������������� ���
    mov     #PTCON_INIT,W0      ;
    mov     W0,PTCON            ;
    mov     #PTCON2_INIT,W0     ;
    mov     W0,PTCON2           ;
    mov     #PTPER_INIT, W0     ;
    mov     W0,PTPER            ;
    mov     #SEVTCMP_INIT,W0    ;
    mov     W0,SEVTCMP          ;
    mov	    #STCON2_INIT,W0	;
    mov	    W0,STCON2		;
    mov	    #STPER_INIT,W0	;
    mov	    W0,STPER		;
    mov	    #SSEVTCMP_INIT,W0	;
    mov	    W0,SSEVTCMP		;
    mov	    #CHOP_INIT,W0	;
    mov	    W0,CHOP		;
    mov     #MDC_INIT,W0        ;
    mov     W0,MDC              ;
;������������� ��� ����� 1
    mov     #PWMCON1_INIT,W0    ;
    mov     W0,PWMCON1          ;
    mov     #PDC1_INIT,W0       ;
    mov     W0,PDC1             ;
    mov     #SDC1_INIT,W0       ;
    mov     W0,SDC1             ;
    mov     #PHASE1_INIT,W0     ;
    mov     W0,PHASE1           ;
    mov     #SPHASE1_INIT,W0    ;
    mov     W0,SPHASE1          ;
    mov     #DTR1_INIT,W0       ;
    mov	    W0,DTR1             ;
    mov     #ALTDTR1_INIT,W0    ;
    mov     W0,ALTDTR1          ;
    mov     #TRGCON1_INIT,W0	;
    mov     W0,TRGCON1          ;
    mov     #IOCON1_INIT,W0     ;
    mov     W0,IOCON1           ;
    mov     #FCLCON1_INIT,W0	;
    mov     W0,FCLCON1          ;
    mov     #TRIG1_INIT,W0      ;
    mov     W0,TRIG1            ;
    mov     #STRIG1_INIT,W0     ;
    mov     W0,STRIG1           ;
    mov	    #FCLCON1_INIT,W0	;
    mov	    W0,FCLCON1		;
    mov     #LEBCON1_INIT,W0	;
    mov     W0,LEBCON1          ;
    mov     #LEBDLY1_INIT,W0	;
    mov     W0,LEBDLY1          ;
    mov     #AUXCON1_INIT,W0	;
    mov     W0,AUXCON1          ;    
;������������� ��� ����� 2
    mov     #PWMCON2_INIT,W0    ;
    mov     W0,PWMCON2          ;
    mov     #PDC2_INIT,W0       ;
    mov     W0,PDC2             ;
    mov     #SDC2_INIT,W0       ;
    mov     W0,SDC2             ;
    mov     #PHASE2_INIT,W0     ;
    mov     W0,PHASE2           ;
    mov     #SPHASE2_INIT,W0    ;
    mov     W0,SPHASE2          ;
    mov     #DTR2_INIT,W0       ;
    mov	    W0,DTR2             ;
    mov     #ALTDTR2_INIT,W0    ;
    mov     W0,ALTDTR2          ;
    mov     #TRGCON2_INIT,W0	;
    mov     W0,TRGCON2          ;
    mov     #IOCON2_INIT,W0     ;
    mov     W0,IOCON2           ;
    mov     #FCLCON2_INIT,W0	;
    mov     W0,FCLCON2          ;
    mov     #TRIG2_INIT,W0      ;
    mov     W0,TRIG2            ;
    mov     #STRIG2_INIT,W0     ;
    mov     W0,STRIG2           ;
    mov	    #FCLCON2_INIT,W0	;
    mov	    W0,FCLCON2		;
    mov     #LEBCON2_INIT,W0	;
    mov     W0,LEBCON2          ;
    mov     #LEBDLY2_INIT,W0	;
    mov     W0,LEBDLY2          ;
    mov     #AUXCON2_INIT,W0	;
    mov     W0,AUXCON2          ;
;������������� ���
    mov     #ADCON1L_INIT,W0    ;
    mov     W0,ADCON1L          ;
    mov     #ADCON1H_INIT,W0    ;
    mov     W0,ADCON1H          ;
    mov     #ADCON2L_INIT,W0    ;
    mov     W0,ADCON2L          ;
    mov     #ADCON2H_INIT,W0    ;
    mov     W0,ADCON2H          ;
    mov     #ADCON3L_INIT,W0    ;
    mov     W0,ADCON3L          ;
    mov     #ADCON3H_INIT,W0    ;
    mov     W0,ADCON3H          ;
    mov     #ADCON4L_INIT,W0    ;
    mov     W0,ADCON4L          ;
    mov     #ADCON4H_INIT,W0    ;
    mov     W0,ADCON4H          ;
    mov     #ADCON5L_INIT,W0    ;
    mov     W0,ADCON5L          ;
    mov     #ADCON5H_INIT,W0    ;
    mov     W0,ADCON5H          ;
    mov     #ADCORE0L_INIT,W0   ;
    mov     W0,ADCORE0L         ;
    mov     #ADCORE0H_INIT,W0   ;
    mov     W0,ADCORE0H         ;
    mov     #ADCORE1L_INIT,W0   ;
    mov     W0,ADCORE1L         ;
    mov     #ADCORE1H_INIT,W0   ;
    mov     W0,ADCORE1H         ;
    mov     #ADCORE2L_INIT,W0   ;
    mov     W0,ADCORE2L         ;
    mov     #ADCORE2H_INIT,W0   ;
    mov     W0,ADCORE2H         ;
    mov     #ADLVLTRGL_INIT,W0  ;
    mov     W0,ADLVLTRGL        ;
    mov     #ADLVLTRGH_INIT,W0  ;
    mov     W0,ADLVLTRGH        ;
    mov     #ADEIEL_INIT,W0	;
    mov     W0,ADEIEL           ;
    mov     #ADEIEH_INIT,W0     ;
    mov     W0,ADEIEH           ;
    mov     #ADMOD0L_INIT,W0	;
    mov     W0,ADMOD0L          ;
    mov     #ADMOD0H_INIT,W0    ;
    mov     W0,ADMOD0H          ;
    mov     #ADIEL_INIT,W0	;
    mov     W0,ADIEL		;
    mov     #ADIEH_INIT,W0	;
    mov     W0,ADIEH		;
    mov     #ADTRIG0H_INIT,W0	;
    mov     W0,ADTRIG0H		;   
    mov     #ADTRIG0L_INIT,W0	;
    mov     W0,ADTRIG0L		;
    mov     #ADCMP0CON_INIT,W0	;
    mov     W0,ADCMP0CON	;
    mov     #ADCMP1CON_INIT,W0	;
    mov     W0,ADCMP1CON	;
    mov     #ADFL0CON_INIT,W0	;
    mov     W0,ADFL0CON		;
    mov     #ADFL1CON_INIT,W0	;
    mov     W0,ADFL1CON		;
;������������� SPI1
    mov	    #SPI1STAT_INIT,W0	;
    mov	    W0,SPI1STAT		;
    mov     #SPI1CON1_INIT,W0	;
    mov	    W0,SPI1CON1     	;
    mov     #SPI1CON2_INIT,W0	;
    mov	    W0,SPI1CON2     	;
;������������� SPI2
    mov	    #SPI2STAT_INIT,W0	;
    mov	    W0,SPI2STAT		;
    mov     #SPI2CON1_INIT,W0	;
    mov	    W0,SPI2CON1     	;
    mov     #SPI2CON2_INIT,W0	;
    mov	    W0,SPI2CON2     	;
;������������� TMR2
    mov     #T2CON_INIT, W0     ;
    mov     W0, T2CON           ;
    mov     #PR2_INIT, W0       ;
    mov     W0, PR2             ;
    clr     TMR2                ;
;������������� OC1
    mov     #OC1CON1_INIT, W0   ;
    mov     W0, OC1CON1         ;
    mov     #OC1CON2_INIT, W0   ;
    mov     W0, OC1CON2         ;
    mov     #OC1RS_INIT, W0     ;
    mov     W0, OC1RS           ;
    mov     #OC1R_INIT, W0      ;
    mov     W0, OC1R            ;
;������������� IC1
    mov	    #IC1CON1_INIT, W0	;
    mov	    W0, IC1CON1		;
    mov	    #IC1CON2_INIT, W0	;
    mov	    W0, IC1CON2		;
    bset    IC1CON2, #ICTRIG	;
;����������
    clr     IPC0                ;
    clr     IPC1                ;
    clr     IPC2                ;
    clr     IPC3                ;
    clr     IPC4                ;
    clr     IPC5                ;
    clr     IPC7                ;
    clr	    IPC8		;
    clr	    IPC9		;
    clr	    IPC12		;
    clr	    IPC13		;
    clr     IPC14               ;
    clr     IPC16               ;
    clr	    IPC18		;
    clr     IPC23               ;
    clr	    IPC24		;
    clr     IPC25               ;
    clr     IPC26               ;
    clr	    IPC27		;
    clr     IPC28               ;
    clr     IPC29               ;
    clr	    IPC35		;
    clr     IPC37               ;
    clr     IPC38               ;
    clr     IPC39               ;
    clr     IPC40               ;
    clr     IPC41               ;
    clr     IPC43               ;
    clr     IPC44               ;
    clr     IPC45               ;
;��������� ���������� �� ���
    mov     #ADC_IPR_MASK,W0	;
    and     IPC27		;
    mov     #ADC_IPR, W0	;
    ior     IPC27		;
;��������� ���������� �� ������� 0 ���
    mov     #ADC_FLT0_IPR_MASK,W0
    and     IPC44		;
    mov     #ADC_FLT0_IPR, W0	;
    ior     IPC44		;   
;��������� ���������� �� SPI1
    mov     #SPI1_IPR_MASK, W0	;
    and     IPC2		;
    mov     #SPI1_IPR, W0	;
    ior     IPC2		;
;��������� ���������� �� SPI2
    mov     #SPI2_IPR_MASK, W0	;
    and     IPC8		;
    mov     #SPI2_IPR, W0	;
    ior     IPC8		;
;��������� ���������� �� ������� 2
    mov     #T2_IPR_MASK, W0
    and     IPC1		;
    mov     #T2_IPR, W0		;
    ior     IPC1		;
;��������� ���������� �� IC1
    mov	    #IC1_IPR_MASK, W0	;
    and	    IPC0		;
    mov	    #IC1_IPR, W0	;
    ior	    IPC0		;
;������������� ����������� ����������
;��� �� ����������
    clr     UError_n
    clr     UError_n_1
    clr     UError_n_2
    mov     #U_PID_PROP_GAIN, W1
    mov     #U_PID_INT_GAIN, W2
    mov     #U_PID_DERIV_GAIN, W3
;������ UK_A
    clr     W0
    add     W0, W1, W0
    add     W0, W2, W0
    add     W0, W3, W0
    mov     W0, UK_A
;������ UK_B
    clr     W0
    mov     W3, W0
    sl      W0, #1, W0
    add     W1, W0, W0
    neg     W0, W0
    mov     W0, UK_B
;������ UK_C
    mov     W3, UK_C
;��� �� ����
    clr     IError_n
    clr     IError_n_1
    clr     IError_n_2
    mov     #I_PID_PROP_GAIN, W1
    mov     #I_PID_INT_GAIN, W2
    mov     #I_PID_DERIV_GAIN, W3
;������ IK_A
    clr     W0
    add     W0, W1, W0
    add     W0, W2, W0
    add     W0, W3, W0
    mov     W0, IK_A
;������ IK_B
    clr     W0
    mov     W3, W0
    sl      W0, #1, W0
    add     W1, W0, W0
    neg     W0, W0
    mov     W0, IK_B
;������ IK_C
    mov     W3, IK_C
;������������� DSP ����
    bclr    CORCON, #US1
    bclr    CORCON, #US0
    bclr    CORCON, #EDT
    bset    CORCON, #SATA
    bset    CORCON, #SATB
    bset    CORCON, #SATDW
    bclr    CORCON, #ACCSAT
    bset    CORCON, #IF
;������������� SPI
    SetCSEEPROM
    clr.b   CntEEBuf		;
;������������� �������� ������
    clr     _FlagReg		;
;DS80439L �.32 (�����������; ������ - ��. ����� ������������� IOCON1 � ����� MCPIDReg.inc)
;���������� ���������� ������� PWM1H � PWM1L, PWM2H � PWM2L
    bclr    IOCON1, #OVRENH	;
    bclr    IOCON1, #OVRENL	;
    bclr    IOCON2, #OVRENH	;
    bclr    IOCON2, #OVRENL	;
;�������� �����, ��� �� ���� ������ ���
    repeat  #OVERRIDE_DELAY	;
    nop				;
;������������ ��� ������� �� CPU � ������ ���
    bset    IOCON1, #PENH	;
    bset    IOCON1, #PENL	;
    bset    IOCON2, #PENH	;
    bset    IOCON2, #PENL	;
;������ ��� PWM3H � PWM3L �� ������������ � �� ���������� �������� ���� B
    bclr    IOCON3,#PENL	;
    bclr    IOCON3,#PENH	;
;������ ������������� ��������� ������������
;���������� � ������ ������� AN0, AN1 � AN2 ���
    mov	    #0x0f00,W0		;��������� ������� ������������� ��� � ��������
    ior	    ADCON5H		;
    bset    ADCON1L,#ADON	;����� ��������� ���
    bset    ADCON5L,#C0PWR	;��������� ����������� ������� ������ 0 ���
WAIT_CH0_ADC:
    btss    ADCON5L,#C0RDY	;�������� ��������� ������ 0 ���
    bra	    WAIT_CH0_ADC	;
    bset    ADCON3H,#C0EN	;��������� ��������� ������� ��������� ��� ������ 0 ���
    bset    ADCON5L,#C1PWR	;��������� ����������� ������� ������ 1 ���
WAIT_CH1_ADC:
    btss    ADCON5L,#C1RDY	;�������� ��������� ������ 1 ���
    bra	    WAIT_CH1_ADC	;
    bset    ADCON3H,#C1EN	;��������� ��������� ������� ��������� ��� ������ 1 ���
    bset    ADCON5L,#C2PWR	;��������� ����������� ������� ������ 2 ���
WAIT_CH2_ADC:
    btss    ADCON5L,#C2RDY	;�������� ��������� ������ 2 ���
    bra	    WAIT_CH2_ADC	;
    bset    ADCON3H,#C2EN	;��������� ��������� ������� ��������� ��� ������ 2 ���    
    
    bset    ADCAL0L,#CAL0EN	;���������� ���������� ������ 0 ���
    bclr    ADCAL0L,#CAL0DIFF	;���������� ������������ ����� ������ 0 ���
    bset    ADCAL0L,#CAL0RUN	;����� ���������� ������ 0 ���
WAIT_CH0_CAL_UNI:
    btss    ADCAL0L,#CAL0RDY	;�������� ���������� ���������� ������������ ����� ������ 0 ���
    bra	    WAIT_CH0_CAL_UNI	;
    bset    ADCAL0L,#CAL0DIFF	;���������� ����������������� ����� ������ 0 ���
    bset    ADCAL0L,#CAL0RUN	;����� ���������� ������ 0 ���
WAIT_CH0_CAL_DIFF:
    btss    ADCAL0L,#CAL0RDY	;�������� ���������� ���������� ����������������� ����� ������ 0 ���
    bra	    WAIT_CH0_CAL_DIFF	;
    bclr    ADCAL0L,#CAL0EN	;��������� ���������� ������ 0 ���   
    bset    ADCAL0L,#CAL1EN	;���������� ���������� ������ 1 ���
    bclr    ADCAL0L,#CAL1DIFF	;���������� ������������ ����� ������ 1 ���
    bset    ADCAL0L,#CAL1RUN	;����� ���������� ������ 1 ���
WAIT_CH1_CAL_UNI:
    btss    ADCAL0L,#CAL1RDY	;�������� ���������� ���������� ������������ ����� ������ 1 ���
    bra	    WAIT_CH1_CAL_UNI	;
    bset    ADCAL0L,#CAL1DIFF	;���������� ����������������� ����� ������ 1 ���
    bset    ADCAL0L,#CAL1RUN	;����� ���������� ������ 1 ���
WAIT_CH1_CAL_DIFF:
    btss    ADCAL0L,#CAL1RDY	;�������� ���������� ���������� ����������������� ����� ������ 1 ���
    bra	    WAIT_CH1_CAL_DIFF	;
    bclr    ADCAL0L,#CAL1EN	;��������� ���������� ������ 1 ���  
    bset    ADCAL0L,#CAL2EN	;���������� ���������� ������ 2 ���
    bclr    ADCAL0L,#CAL2DIFF	;���������� ������������ ����� ������ 2 ���
    bset    ADCAL0L,#CAL2RUN	;����� ���������� ������ 2 ���
WAIT_CH2_CAL_UNI:
    btss    ADCAL0L,#CAL2RDY	;�������� ���������� ���������� ������������ ����� ������ 2 ���
    bra	    WAIT_CH2_CAL_UNI	;
    bset    ADCAL0L,#CAL2DIFF	;���������� ����������������� ����� ������ 2 ���
    bset    ADCAL0L,#CAL2RUN	;����� ���������� ������ 2 ���
WAIT_CH2_CAL_DIFF:
    btss    ADCAL0L,#CAL2RDY	;�������� ���������� ���������� ����������������� ����� ������ 2 ���
    bra	    WAIT_CH2_CAL_DIFF	;
    bclr    ADCAL0L,#CAL2EN	;��������� ���������� ������ 2 ���    
;������������� ����� � �������� ���
    clr	    UADSum		;
    clr	    RADSum		;
;������������� �������� ��������� DM13
    mov.b   #DM13Idle, W0	;
    mov.b   WREG, DM13State	; 
;���������� ��������� ����������
    bclr    INTCON1,#NSTDIS	;
    bset    INTCON2,#GIE	;
    bclr    CORCON,#IPL3	;
    mov     #0xff1f,W0		;��������� ���������� CPU=0
    and     SR			;
    clr	    RCON		;
;������ ������� 2
    mov.b   #10, W0		;	    
    mov.b   WREG, TMR10ms	;
    mov     #50, W0             ;
    mov     W0, BlinkCnt        ;
    bset    T2CON, #TON         ;
;���������� ������������ ���������� ����������
;���������� ���������� �� TMR2
    bclr    IFS0, #T2IF		;
    bset    IEC0, #T2IE		;
;������������� �������
    call    _DispInit
;���������� ���������� �� ADC AN0, AN1 � ������� 0
    bset    ADIEL,#IE0		;
    bset    ADIEL,#IE1		; 
    bset    ADFL0CON, #IE_ADFL0CON;
    bclr    IFS6, #ADCAN0IF	;
    bset    IEC6, #ADCAN0IE	;
    bclr    IFS6, #ADCAN1IF	;
    bset    IEC6, #ADCAN1IE	;
    bclr    IFS11, #ADFLTR0IF	;
    bset    IEC11, #ADFLTR0IE	;
    bset    ADFL0CON, #FLEN	;
;���������� ���������� �� IC1
    bclr    IFS0, #IC1IF	;
    bset    IEC0, #IC1IE	;
;������������� ���������
    
    call    _InitProg
;������ ���
    bset    PTCON, #PTEN        ;
;������ SPI1 � ������ Errata DS80000656C �. 28.SPI. Work around �1
SPI1_START:
    SetCSEEPROM			;������ CS EEPROM ��������������� � 1
    bclr    SPI1STAT,#SPIROV    ;��������� ������ SPI1
    bset    SPI1STAT,#SPIEN     ;
    clr	    W0			;������ ���� SPI
    bclr    IFS0, #SPI1IF	;
    mov	    W0, SPI1BUF		;
FIRST_SPI1_CYCLE:
    btss    IFS0, #SPI1IF	;
    bra	    FIRST_SPI1_CYCLE	;
    mov	    SPI1BUF, W0		;
    clr	    W0			;������ ���� SPI
    bclr    IFS0, #SPI1IF	;
    mov	    W0, SPI1BUF		;
SECOND_SPI1_CYCLE:
    btss    IFS0, #SPI1IF	;
    bra	    SECOND_SPI1_CYCLE	;
    mov	    SPI1BUF, W0		;
    bclr    SPI1STAT,#SPIEN     ;���������� ������ SPI1
    repeat  #200		;�������� 200 ������
    nop				;
    bclr    SPI1STAT,#SPIROV    ;��������� ��������� ������ SPI1
    bset    SPI1STAT,#SPIEN     ;
;���������� ���������� �� SPI1
    bclr    IFS0, #SPI1IF	;
    bset    IEC0, #SPI1IE	;
;������� ������ � EEPROM
    mov	    #0x55aa, W0		;
    mov	    W0, EEPROMData	;
    mov	    #EEPROMData, W0	;
    mov	    #0x07fe, W1		;
    mov	    #0x0002, W2		;
    rcall   _WR_EEPROM		;
WAIT_WR_EEPROM:
    btsc    _FlagReg, #EEBusyFl	;�������� ���������� �������� ������
    bra	    WAIT_WR_EEPROM	;
;������� ������ �� EEPROM
    mov	    #EEPROMData, W0	;    
    mov	    #0x07fe, W1		;
    mov	    #0x0002, W2		;
    rcall   _RD_EEPROM		;
WAIT_RD_EEPROM:
    btsc    _FlagReg, #EEBusyFl	;�������� ���������� �������� ������
    bra	    WAIT_RD_EEPROM	;    
    mov	    EEPROMData, W0	;
    mov	    #0x55aa, W1		;
    cp	    W0, W1		;
    bra	    z, SPI2_START	;���� ���������� ������ �� ������� � ������������, ��:
    bclr    IEC0, #SPI1IE	; ������ ���������� �� SPI
    bclr    SPI1STAT,#SPIEN     ; ���������� ������ SPI
    repeat  #200		; �������� 200 ������
    nop				;    
    bra	    SPI1_START		;������� � ����������� ������ SPI
;������ SPI2 � ������ Errata DS80000656C �. 28.SPI. Work around �1
SPI2_START:
    bclr    SPI2STAT,#SPIROV    ;��������� ������ SPI2
    bset    SPI2STAT,#SPIEN     ;
    clr	    W0			;������ ���� SPI
    bclr    IFS2, #SPI2IF	;
    mov	    W0, SPI2BUF		;
FIRST_SPI2_CYCLE:
    btss    IFS2, #SPI2IF	;
    bra	    FIRST_SPI2_CYCLE	;
    mov	    SPI2BUF, W0		;
    clr	    W0			;������ ���� SPI
    bclr    IFS2, #SPI2IF	;
    mov	    W0, SPI2BUF		;
SECOND_SPI2_CYCLE:
    btss    IFS2, #SPI2IF	;
    bra	    SECOND_SPI2_CYCLE	;
    mov	    SPI2BUF, W0		;
    bclr    SPI2STAT,#SPIEN     ;���������� ������ SPI2
    repeat  #200		;�������� 200 ������
    nop				;
    bclr    SPI2STAT,#SPIROV    ;��������� ��������� ������ SPI2
    bset    SPI2STAT,#SPIEN     ;
    mov	    #0x55aa, W0		;��������� ������������ ����� 6 ��� � ����� SPI2
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    bclr    IFS2, #SPI2IF	;������� ����� ����������
SPI2_WAIT1:
    btss    IFS2, #SPI2IF	;�������� �������� ���� ���� ��
    bra	    SPI2_WAIT1		;����������� ������ SPI2
SPI2_PROB_RD:
    mov	    SPI2BUF, W0		;������ ���� ���� �� ����������� 
    btss    SPI2STAT, #SRXMPT	;��������� ������ SPI2
    bra	    SPI2_PROB_RD	;
    mov	    #0x55aa, W1		;
    cp	    W0, W1		;
    bra	    z, SPI2_INT_EN	;
    bclr    IEC2, #SPI2IE	; ������ ���������� �� SPI2
    bclr    SPI2STAT,#SPIEN     ; ���������� ������ SPI2
    repeat  #200		; �������� 200 ������
    nop				;  
    bra	    SPI2_START		;
SPI2_INT_EN:  
;���������� ���������� �� SPI2
    bclr    IFS2, #SPI2IF	;
    bset    IEC2, #SPI2IE	;

;������������� �������
DISPLAY_INIT:
    call    _InitDisplay
;------------------------------------------------------------------------------------------------
;�������� ����
Main:
;���������� �����
    btss    _FlagReg, #ADCP0IntFl
    bra	    ADCP1_TST
;��������� ����������
;���� ���� ������� ��� �� ����������, �� ������ �� �����������, ���� ����� ����������� ������������
    btsc    _FlagReg, #PIDCalc  ;
    bra     U_PID_CALC          ;
    btss    _FlagReg, #PIDModeFl;
    bra	    U_PID_LOAD_I	;
    mov     #DUTY_MIN, W0       ;
    bra     U_PWM_LOAD_U	;
;����� �������� ����� �� ����������
;������������� ���������� �� ������ ��� ���������� � ������ ������
U_PID_CALC:
    mov     #UError_n, W10
    mov     #UK_A, W8
    mov     #Ufb, W3            ; �������� �� ������� �������� ����������
;���������� ������ �� ����������
    mov     [W3], W0
;���������� ���������������� ������
;ep = _USetM - �������� �������� ����������
;ep [W1] = _USetM - Ufb
    mov     _USetM, W1
    sub     W1, W0, W0		; W0 = ����� ���������������� ������ ���������
    mov     W0, [W10]
;���������� ���������� ���
    mov     #UCtrl, W2
    movsac  A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A
;���������� ����������� ��������
    sac.r   A, -#8, [W2]
;�������� ��� ����� �����
    mov     [W2],W3
;�������� ��������� �� ���������� �� ��������
;�������� ������������� ���������� �� ����������
;���� ����� PID "��������" (���� ������ PID =1), �� ����� ���������� ��������� ������������ ������������� � MDC,
;����� ����� ��������� ������������ ���������� ����� *8, �. �. ������������ ��� * 8
    btss    _FlagReg, #PIDModeFl
    bra	    I_MAX_LOAD
    mov	    #DUTY_MAX, W0    
    bra	    CP_U_MAX
I_MAX_LOAD:
    mov     _IMaxU, W0
    sl	    W0,#3,W0
CP_U_MAX:
    cp      W0, W3
    bra     gt, PID_Umin
U_SET_MAX:
    mov     W0, W3
    clr     A
    mov     W3, ACCAH
    clr     ACCAL
    sftac   A, #8
    mov     W3, UCtrl
    bra     PID_U_End
PID_Umin:
;������� ���������� �� ���������� �� �������
;���� ����� "��������" (���� ������ PID = 1), �� ������� - ����������� ������������ �����,
;����� ������� - ����������� ���
    btss    _FlagReg, #PIDModeFl
    bra	    I_MIN_LOAD
    mov     #DUTY_MIN, W0
    bra	    CP_U_MIN
I_MIN_LOAD:
    mov	    _IMinU, W0
    sl	    W0, #3, W0
CP_U_MIN:
    cp      W0, W3
    bra     lt, PID_U_End
    mov     W0, W3
    clr     A
    mov     W3, ACCAH
    clr	    ACCAL
    sftac   A, #8
    mov     W3, UCtrl
PID_U_End:
;��������� ������ ������ �� ����������
    mov     #UError_n, W10
    mov     [W10+2], W2
    mov     W2, [W10+4]
    mov     [W10], W2
    mov     W2, [W10+2]   
;���� ������������ ���������� �� ���������� ������, ��� ���������� �� ����, �� ���������� MDC
;��� �������, ��� ����� - "��������" (���� ������ PID = 1)
    mov     UCtrl, W0
    btss    _FlagReg, #PIDModeFl
    bra	    U_PID_LOAD_I
    mov     ICtrl, W3
    cpsgt   W0, W3
U_PWM_LOAD_U:
    mov     W0, MDC
    bra	    U_PID_EXIT
U_PID_LOAD_I:
    mov	    UCtrl, W0
    mov	    W0, _ISetU
U_PID_EXIT:
    bclr    _FlagReg, #ADCP0IntFl
;��������� ����
ADCP1_TST:
    btss    _FlagReg, #ADCP1IntFl
    bra     IC1_TST		;
;���� ���� ������� ��� �� ����������, �� ������ �� �����������
    btsc    _FlagReg, #PIDCalc  ;
    bra     I_PID_CALC          ;
    mov     #DUTY_MIN, W0       ;
    bra     I_PWM_LOAD		;
;������� ����� �������� �����
;������������� ���������� �� ������ ��� ����� � ������ ������
I_PID_CALC:
    mov     #IError_n, W10
    mov     #IK_A, W8
    mov     #Ifb, W3		; �������� �� ������� �������� ���
    mov     _ISetM, W1		; ����� - ������������� ��������
    btss    _FlagReg, #PIDModeFl;���� ����� "�������" (���� ������ PID=0), �� ������� ������� �� 8
    asr	    W1, #3, W1
;���������� ������ �� ����
    mov     [W3], W0
; ���������� ���������������� ������
; ep = _ISetM - �������� �������� ����������
; ep [W1] = _ISetM - Ifb
    sub     W1, W0, W0	; W0 = ����� ���������������� ������ ����
    mov     W0, [W10]
;���������� ���������� ���
    mov     #ICtrl, W2
    movsac  B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B
;���������� ����������� ��������
    sac.r   B, -#8, [W2]
;�������� ��� ����� �����
    mov     [W2],W3
;�������� ��������� �� ���� �� ��������
;�������� ������������� ���������� �� ����
    mov     #DUTY_MAX, W0
    cp      W0, W3
    bra     gt, PID_Imin
I_SET_MAX:
    mov     W0, W3
    clr     B
    mov     W3, ACCBH
    clr     ACCBL
    sftac   B, #8
    mov     W3, ICtrl
    bra     PID_I_End
PID_Imin:
;�������� ���������� �� ���� �� �������
    mov     #DUTY_MIN, W0
    cp      W0, W3
    bra     lt, PID_I_End
    mov     W0, W3
    clr     B
    mov     W3, ACCBH
    clr     ACCBL
    sftac   B, #8
    mov     W3, ICtrl
PID_I_End:
;��������� ������ ������ �� ����
    mov     #IError_n, W10
    mov     [W10+2], W2
    mov     W2, [W10+4]
    mov     [W10], W2
    mov     W2, [W10+2]
;���� ������������ ���������� �� ���� ������, ��� ���������� �� ����������, �� ���������� MDC
;��� �������, ��� ����� - "��������" (���� ������ PID = 1)
    mov     ICtrl, W0
    btss    _FlagReg, #PIDModeFl
    bra	    I_PWM_LOAD
    mov     UCtrl, W3
    cpsgt   W0, W3
I_PWM_LOAD:
    mov     W0, MDC
    call    _ArcControl
    bclr    _FlagReg, #ADCP1IntFl
;��������� ������� ��������� ����� IC1
IC1_TST:
    btss    _FlagReg, #IC1IntFl	;
    bra	    T2_TIC_TST		;
    mov	    Param, W0		;
    lsr	    W0, #0x01, W0	;
    rcall   _ParamCalc		;
    inc	    IC1IntCnt		;
    mov	    IC1IntCnt, W0	;
    cp	    W0, #MAX_IC1_INT_CNT;
    bra	    lt, SET_MODE_FL	;
    mov	    #MAX_IC1_INT_CNT, W0;
    mov	    W0, IC1IntCnt	;
SET_MODE_FL:
    bset    _FlagReg, #ModeFl	;
    bclr    _FlagReg, #IC1IntFl	;
;����� ������
T2_TIC_TST:
    btss    _FlagReg, #T2Tic
    bra     CYCLE_END
    mov.b   _ButReg, WREG
    mov.b   WREG, _OldButReg
;���������� �������� ������ � ��������
;���������� �������� ���� "������� � �����"
    btss    PORTC, #RC7
    bra     HPG_FALSE
    inc.b   CntHPG
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntHPG
    bra     nc, TMP1_DEBOUNCE
    bset    _AlmReg,#HPGM 
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntHPG
    bra     TMP1_DEBOUNCE
HPG_FALSE:
    cp0.b   CntHPG
    bra     z, HPG_RES
    dec.b   CntHPG
    bra     TMP1_DEBOUNCE
HPG_RES:
    bclr    _AlmReg,#HPGM  
TMP1_DEBOUNCE:
;���������� �������� ���� "�����1"
    btss    PORTB, #RB2
    bra     TMP1_FALSE
    inc.b   CntTMP1
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntTMP1
    bra     nc, TMP2_DEBOUNCE
    bset    _AlmReg,#Thermo1M 
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntTMP1
    bra     TMP2_DEBOUNCE
TMP1_FALSE:
    cp0.b   CntTMP1
    bra     z, TMP1_RES
    dec.b   CntTMP1
    bra     TMP2_DEBOUNCE
TMP1_RES:
    bclr    _AlmReg,#Thermo1M  
TMP2_DEBOUNCE:
;���������� �������� ���� "�����2"
    btss    PORTB, #RB3
    bra     TMP2_FALSE
    inc.b   CntTMP2
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntTMP2
    bra     nc, TMP3_DEBOUNCE
    bset    _AlmReg,#Thermo2M 
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntTMP2
    bra     TMP3_DEBOUNCE
TMP2_FALSE:
    cp0.b   CntTMP2
    bra     z, TMP2_RES
    dec.b   CntTMP2
    bra     TMP3_DEBOUNCE
TMP2_RES:
    bclr    _AlmReg,#Thermo2M  
TMP3_DEBOUNCE:
;���������� �������� ���� "�����3"
    btss    PORTB, #RB4 
    bra     TMP3_FALSE
    inc.b   CntTMP3
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntTMP3
    bra     nc, TMP4_DEBOUNCE
    bset    _AlmReg,#Thermo3M 
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntTMP3
    bra     TMP4_DEBOUNCE
TMP3_FALSE:
    cp0.b   CntTMP3
    bra     z, TMP3_RES
    dec.b   CntTMP3
    bra     TMP4_DEBOUNCE
TMP3_RES:
    bclr    _AlmReg,#Thermo3M  
TMP4_DEBOUNCE:
;���������� �������� ���� "�����4"
    btss    PORTC, #RC2
    bra     TMP4_FALSE
    inc.b   CntTMP4
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntTMP4
    bra     nc, SPB_DEBOUNCE
    bset    _AlmReg,#Thermo4M 
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntTMP4
    bra     SPB_DEBOUNCE
TMP4_FALSE:
    cp0.b   CntTMP4
    bra     z, TMP4_RES
    dec.b   CntTMP4
    bra     SPB_DEBOUNCE
TMP4_RES:
    bclr    _AlmReg,#Thermo4M    
SPB_DEBOUNCE:
;���������� �������� ������ ������ ������ "���/���/���"
    btss    PORTC, #RC0
    bra     SPB_FALSE
    inc.b   CntSPB
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntSPB
    bra     nc, MEMB_DEBOUNCE
    bset    _ButReg,#SPBtn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntSPB
    bra     MEMB_DEBOUNCE
SPB_FALSE:
    cp0.b   CntSPB
    bra     z, SPB_RES
    dec.b   CntSPB
    bra     MEMB_DEBOUNCE
SPB_RES:
    bclr    _ButReg,#SPBtn
;���������� �������� ������ "������"
MEMB_DEBOUNCE:
    btss    PORTC, #RC10
    bra     MEMB_FALSE
    inc.b   CntMEMB
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntMEMB
    bra     nc, GASB_DEBOUNCE
    bset    _ButReg,#MemBtn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntMEMB
    bra     GASB_DEBOUNCE
MEMB_FALSE:
    mov.b   CntMEMB, WREG
    cp0.b   CntMEMB
    bra     z, MEMB_RES
    dec.b   CntMEMB
    bra     GASB_DEBOUNCE
MEMB_RES:
    bclr    _ButReg,#MemBtn
;���������� �������� ������ ������ ���� ����
GASB_DEBOUNCE:
    btss    PORTC, #RC11
    bra     GASB_FALSE
    inc.b   CntGASB
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntGASB
    bra     nc, SELB_DEBOUNCE
    bset    _ButReg,#GasBtn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntGASB
    bra     SELB_DEBOUNCE
GASB_FALSE:
    cp0.b   CntGASB
    bra     z, GASB_RES
    dec.b   CntGASB
    bra     SELB_DEBOUNCE
GASB_RES:
    bclr    _ButReg,#GasBtn
;���������� �������� ������ ������ ��������� ��� ��������� ����������
SELB_DEBOUNCE:
    btss    PORTC, #RC12
    bra     SELB_FALSE
    inc.b   CntSELB
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntSELB
    bra     nc, T24B_DEBOUNCE
    bset    _ButReg,#SelBtn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntSELB
    bra	    T24B_DEBOUNCE
SELB_FALSE:
    cp0.b   CntSELB
    bra     z, SELB_RES
    dec.b   CntSELB
    bra     T24B_DEBOUNCE
SELB_RES:
    bclr    _ButReg,#SelBtn
;���������� �������� ������ 2T/4T
T24B_DEBOUNCE:
    btss    PORTC,#RC13
    bra     T24B_FALSE
    inc.b   CntT24B
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntT24B
    bra     nc, MIGB_DEBOUNCE
    bset    _ButReg,#T24Btn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntT24B
    bra     MIGB_DEBOUNCE
T24B_FALSE:
    cp0.b   CntT24B
    bra     z, T24B_RES
    dec.b   CntT24B
    bra     MIGB_DEBOUNCE
T24B_RES:
    bclr    _ButReg,#T24Btn
;���������� �������� ������ "MIG"
MIGB_DEBOUNCE:
    btss    PORTC,#RC5
    bra     MIGB_FALSE
    inc.b   CntMIGB
    mov.b   #MAX_INP_CNT, W0
    cp.b    CntMIGB
    bra     nc, DISPLAY
    bset    _ButReg,#MIGBtn
    mov.b   #MAX_INP_CNT, W0
    mov.b   WREG, CntMIGB
    bra     DISPLAY
MIGB_FALSE:
    cp0.b   CntMIGB
    bra     z, MIGB_RES
    dec.b   CntMIGB
    bra     DISPLAY
MIGB_RES:
    bclr    _ButReg,#MIGBtn
;�������
DISPLAY:
    call    _Display
;����� ����� ������ (������� �� ������ MIG � ����� MMA)
    dec	    IC1IntCnt
    mov	    IC1IntCnt, W0
    cp0	    W0
    bra	    gt, RES_T2_TIC
    inc	    IC1IntCnt
    bclr    _FlagReg, #ModeFl
RES_T2_TIC:
    bclr    _FlagReg, #T2Tic
CYCLE_END:   
;����� ��������� �����
    goto    Main		
;------------------------------------------------------------------------------------------------
;���������� �� ������ AN0 ���
    .global __ADCAN0Interrupt
__ADCAN0Interrupt:
;������ ���������� �� ���
    push    W0
    push    W1
    push    W2
    btsc    _FlagReg, #ModeFl	;
    bra	    AD0_NO_AVG		;
    mov	    UADSum, W0		;
    mov	    W0, W2		;
    mov     ADCBUF0, W1		;
    lsr	    W2, #4, W2		;
    sub	    W0, W2, W0		;
    add	    W0, W1, W0		;
    mov	    W0, UADSum		;UADSum=UADSum-UADSum/16+ADCBUF0
    lsr	    W0, #4, W2		;
    mov     W2, Ufb             ;Ufb=(UADSum-UADSum/16+ADCBUF0)/16
    mov     W2, _UfbCtrl        ;_UfbCtrl=(UADSum-UADSum/16+ADCBUF0)/16
    bra	    AD0_INT_FL_SET	;    
AD0_NO_AVG:     
    mov     ADCBUF0, W0		;
    mov     W0, Ufb             ;
    mov     W0, _UfbCtrl        ; 
AD0_INT_FL_SET:
    bset    _FlagReg, #ADCP0IntFl
    pop	    W2
    pop	    W1
    pop	    W0
    bclr    IFS6, #ADCAN0IF	;
    retfie   
;------------------------------------------------------------------------------------------------
;���������� �� ������ AN1 ���
    .global __ADCAN1Interrupt
__ADCAN1Interrupt:
    push    W0			;
    push    W1			;
    mov     ADCBUF1, W0         ;
    mov     W0, Ifb             ;
    mov     W0, _IfbCtrl        ;
;������ ��� ���������� �����
    btsc    _FlagReg,#ArcContrFl;���� ���� ������ ���� ����� ����, ��
    bra     SYNCHRO_SPI1	;
    dec     _ArcTimer           ;��������� ������ ����
    bra     nz, SYNCHRO_SPI1	;���� ������ ���� ����� ����, ��
    bset    _FlagReg,#ArcContrFl;���� ������� ���� ����� 1
;������������� EEPROM � ������� ��� � ���
SYNCHRO_SPI1:
    btss    _FlagReg,#EEBusyFl  ;����(!�������������EEPROM)
    bra     ADC1_INT_END	;{
    mov     EE_State,W0         ;   �������������(���������EEPROM)
    mov     #EE_STAT_RD,W1      ;   {
    cp      W0,W1               ;
    bra     z, RD_IDLE_ADC1     ;
    mov     EE_State,W0         ;
    mov     #EE_STAT_WR,W1      ;
    cp      W0,W1               ;
    bra     z, WR_IDLE_ADC1	;
    bra     ADC1_INT_END        ;	���������: �����;
RD_IDLE_ADC1:			;	������ ���������������:
    ClrCSEEPROM			;	    CSEEPROM=0;
    mov     #EE_STAT_RD,W0      ;           ���������EEPROM=���������������;
    mov     W0,EE_State         ;
    mov	    #0x05,W0            ;           ������ ������� ������ �������� ��������� EEPROM;
    mov	    W0,SPI1BUF		;
    bra     ADC1_INT_END        ;           �����;
WR_IDLE_ADC1:                   ;       ������ ���������������:
    ClrCSEEPROM                 ;           CSEEPROM=0;                   
    mov     #EE_STAT_WR,W0      ;	    ���������EEPROM=���������������;
    mov     W0,EE_State         ;           ������ ������� ������ �������� ��������� EEPROM;
    mov	    #0x05,W0            ;              }
    mov	    W0,SPI1BUF		;       }
;����� ���������� �� ���
ADC1_INT_END:
    bset    _FlagReg, #ADCP1IntFl
    bclr   IFS6, #ADCAN1IF	;
    pop	    W1
    pop	    W0
    retfie
;------------------------------------------------------------------------------------------------
    .global __ADFLTR0Interrupt
__ADFLTR0Interrupt:
    push    W0			;
    push    W1			;
    push    W2			;
    mov	    RADSum, W0		;
    mov	    W0, W2		;
    mov     ADFL0DAT, W1	;
    lsr	    W2, #4, W2		;
    sub	    W0, W2, W0		;
    add	    W0, W1, W0		;
    mov	    W0, RADSum		;RADSum=W0=RADSum-RADSum/16+ADFL0DAT
    mov	    _ParamADC, W1	;
    mov	    W1, _OldParamADC	;_OldParamADC =_ParamADC
    lsr	    W0, #4, W2		;
    mov     W2, _ParamADC       ;_ParamADC=(RADSum-RADSum/16+ADFL0DAT)/16
    bclr    IFS11, #ADFLTR0IF	;
    pop	    W2			;
    pop	    W1			;
    pop	    W0			;
    retfie
;------------------------------------------------------------------------------------------------
;���������� �� SPI1
    .global	__SPI1Interrupt
__SPI1Interrupt:
    push    W0
    push    W1
    mov     EE_State, W0
    bra     W0
    bra     EE_RD_STAT          ;EE_STAT_RD=0
    bra     EE_RD_IDLE          ;IDLE_EE_RD=1
    bra     EE_ADM_RD           ;EE_RD_ADM=2
    bra     EE_ADL_RD           ;EE_RD_ADL=3
    bra     EE_RD_DT            ;EE_RD_DATA=4
    bra     EE_RD_BYTE          ;EE_MOVE_BYTE=5
    bra     EE_WR_STAT          ;EE_STAT_WR=6
    bra     EE_WR_IDLE          ;IDLE_EE_WR=7
    bra     EE_WR_ENA           ;EE_WR_EN=8
    bra     EE_ADM_WR           ;EE_WR_ADM=9
    bra     EE_ADL_WR           ;EE_WR_ADL=a
    bra     EE_WR_BYTE          ;EE_WR_DATA=b
    bra     EE_WR_POST          ;EE_POST_WR=c
    bra     EE_DIS_WR           ;EE_WR_DIS=d
;��������� ��������� ������� ����� ��� ������ �������� ��������� EEPROM ��� ������
EE_RD_STAT:
    mov     SPI1BUF,W0          ;
    mov     #IDLE_EE_RD,W0      ;
    mov     W0,EE_State         ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ������ �������� ��������� EEPROM ��� ������
EE_RD_IDLE:
    SetCSEEPROM
    mov     SPI1BUF,W0          ;
    btss    W0,#WIP             ;���� EEPROM ������ �������, ��
    bra     WelRdTest           ;
    mov     #EE_STAT_RD,W0      ;������� � ���������
    mov     W0,EE_State         ;������ �������� ��������� EEPROM
    bra     SPI1IntEnd          ;��� ������ 
WelRdTest:
    btss    W0,#WEL             ;
    bra     EERdComSend         ;
    ClrCSEEPROM                 ;
    mov     #EE_WR_DIS,W0       ;
    mov     W0,EE_State         ;
    mov     #0x0004,W0          ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
EERdComSend:
    ClrCSEEPROM                 ;����� -
    mov     #EE_RD_ADM,W0       ;
    mov     W0,EE_State         ;
    mov     #0x0003,W0          ;��������� ������� ������
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� �������� ����� ������ ��� ������
EE_ADM_RD:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    lsr     W1,#8,W1            ;
    mov     #EE_RD_ADL,W0       ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� �������� ����� ������ ��� ������
EE_ADL_RD:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    mov     #0x00ff,W0          ;
    and     W1,W0,W1            ;
    mov     #EE_RD_DATA, W0     ;
    mov     W0, EE_State        ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ������� ������� ����� ��� ������ ������
EE_RD_DT:
    mov     SPI1BUF, W0         ;
    mov     #EE_MOVE_BYTE, W0   ;
    mov     W0, EE_State        ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� ������������ ����� � �����
EE_RD_BYTE:
    SetCSEEPROM                 ;
    mov     EEBufPtr, W0        ;���������� ������
    add     CntEEBuf, WREG      ;������������ �����
    mov     SPI1BUF, W1         ;
    mov.b   W1,[W0]             ;
    inc     EEPROM_Ad           ;���������� ������ � EEPROM
    inc     CntEEBuf            ;���������� �������� ����������� ����
    mov     EELength,W0         ;
    cp      CntEEBuf            ;���� ������� ����������� ���� �� ����� �������� �����, ��
    bra     ge,EEReadEnd        ;
    mov     #EE_STAT_RD,W0      ;���� ������ ������������
    mov     W0,EE_State         ;
    bra     SPI1IntEnd          ;
EEReadEnd:
    bclr    _FlagReg,#EEBusyFl  ;����� - ����� ����� ������
    bra     SPI1IntEnd          ;
;��������� ��������� ������� ����� ��� ������ �������� ��������� EEPROM ��� ������
EE_WR_STAT:
    mov     SPI1BUF,WREG        ;
    mov     #IDLE_EE_WR,W0      ;
    mov     W0,EE_State         ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ������ �������� ��������� EEPROM ��� ������
EE_WR_IDLE:
    SetCSEEPROM
    mov     SPI1BUF, WREG       ;
    btss    W0,#0x00            ;���� EEPROM ������ �������, ��
    bra     EEWrEnSet           ;������� � ���������
    mov     #EE_STAT_WR, W0     ;������ �������� ��������� EEPROM
    mov     W0,EE_State         ;��� ������
    bra     SPI1IntEnd          ;
EEWrEnSet:
    ClrCSEEPROM                 ;
    mov     #EE_WR_EN, W0       ;����� -
    mov     W0, EE_State        ;��������� ������� ����������
    mov.b   #0x06, W1           ;������ � EEPROM
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ���������� ������
EE_WR_ENA:
    SetCSEEPROM                 ;
    mov     SPI1BUF,W0          ;
    mov     #0x0002,W1          ;��������� ������� ������
    mov     #EE_WR_ADM, W0      ;
    mov     W0, EE_State        ;
    ClrCSEEPROM                 ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� �������� ����� ������
EE_ADM_WR:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    lsr     W1,#8,W1            ;
    mov     #EE_WR_ADL, W0      ;
    mov     W0, EE_State        ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� �������� ����� ������
EE_ADL_WR:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad, W1       ;
    mov     #0x00ff,W0          ;
    and     W1,W0,W1            ;
    mov     #EE_WR_DATA, W0     ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ������ �����
EE_WR_BYTE:
    mov     SPI1BUF,W0          ;
    mov     EEBufPtr,W0         ;���������� ������
    add     CntEEBuf,WREG       ;������������� �����
    mov.b   [W0],W1             ;
    mov     #EE_POST_WR,W0      ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;��������� ��������� ����� ������ �����
EE_WR_POST:
    SetCSEEPROM                 ;
    mov     SPI1BUF,WREG        ;
    inc     EEPROM_Ad           ;���������� ������ � EEPROM
    inc     CntEEBuf            ;���������� �������� ���������� ����
    mov     EELength, W0        ;
    cp      CntEEBuf            ;���� ������� ���������� ���� �� ����� �������� �����, ��
    bra     ge,EEWriteEnd       ;
    mov     #EE_STAT_WR,W0      ;���� ������ ������������
    mov     W0,EE_State         ;
    bra     SPI1IntEnd          ;
EEWriteEnd:
    bclr    _FlagReg,#EEBusyFl  ;����� - ����� ������
    bra     SPI1IntEnd          ;
EE_DIS_WR:
    SetCSEEPROM                 ;
    mov     SPI1BUF,W0          ;
    mov     #EE_STAT_RD,W0      ;������� � ���������
    mov     W0,EE_State         ;������ �������� ��������� EEPROM           
;����� ���������� �� SPI
SPI1IntEnd:
    bclr    IFS0,#SPI1IF	;
    pop     W1
    pop     W0                  ;
    retfie                      ;
;------------------------------------------------------------------------------------------------
;���������� �� SPI2 (DM13)
    .global	__SPI2Interrupt
__SPI2Interrupt:
    push    W0			    ;
SPI2_READ:
    mov	    SPI2BUF, W0		    ;
    btss    SPI2STAT, #SRXMPT	    ;
    bra	    SPI2_READ		    ;
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13Shift	    ;
    bra	    nz, SET_SPI2_DM13_IDLE  ;
    mov.b   #DM13SetLatch, W0	    ;
    bra	    SPI2_INT_END	    ;
SET_SPI2_DM13_IDLE:
    mov.b   #DM13Idle, W0	    ;   
SPI2_INT_END:
    mov.b   WREG, DM13State	    ;
    bclr    IFS2,#SPI2IF	    ;
    pop	    W0			    ;
    retfie
;------------------------------------------------------------------------------------------------
;���������� �� TMR2
    .global	__T2Interrupt
__T2Interrupt:
    push    W0			    ;
    push    W1			    ;
    btss    _FlagReg, #DigitModifyFl;
    bra	    TST_0N1MS_TIMER	    ;
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13Idle	    ;
    bra	    nz, DM13_SET_EN_TST	    ;
    SetDM13En			    ;
    mov.b   #DM13SetEn, W0	    ;
    mov.b   WREG, DM13State	    ;
    bra	    TST_0N1MS_TIMER	    ;
DM13_SET_EN_TST:
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13SetEn	    ;
    bra	    nz, DM13_SET_LATCH_TST  ;
    mov	    #_DM13Buf, W0	    ;
    mov	    #SPI2BUF, W1	    ;
    mov	    [W0++], [W1]	    ;
    mov	    [W0++], [W1]	    ;
    mov	    [W0++], [W1]	    ;
    mov	    [W0++], [W1]	    ;
    mov	    [W0], [W1]		    ;
    mov.b   #DM13Shift, W0	    ;
    mov.b   WREG, DM13State	    ;
    bra	    TST_0N1MS_TIMER	    ;    
DM13_SET_LATCH_TST:
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13SetLatch	    ;
    bra	    nz, DM13_RES_LATCH_TST  ;
    SetDM13Latch		    ;
    mov.b   #DM13ResLatch, W0	    ;
    mov.b   WREG, DM13State	    ;
    bra	    TST_0N1MS_TIMER	    ;     
DM13_RES_LATCH_TST:
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13ResLatch	    ;
    bra	    nz, DM13_RES_EN_TST	    ;
    ResDM13Latch		    ;
    mov.b   #DM13ResEn, W0	    ;
    mov.b   WREG, DM13State	    ;
    bra	    TST_0N1MS_TIMER	    ;
DM13_RES_EN_TST:
    mov.b   DM13State, WREG	    ;
    cp.b    W0, #DM13ResEn	    ;
    bra	    nz, SET_DM13_IDLE	    ;
    ResDM13En			    ;
SET_DM13_IDLE:
    bclr    _FlagReg, #DigitModifyFl;
    mov.b   #DM13Idle, W0	    ;
    mov.b   WREG, DM13State	    ;    
;������ � ��������� 0,1 ��
TST_0N1MS_TIMER:
    btsc    _FlagReg, #T0n1msFl	    ;���� ���� �������0.1ms ����� ����, ��
    bra	    DEC_10MS_TMR	    ;
    cp0	    _Timer0n1ms		    ;���� ������0.1ms �� ����� ����
    bra	    z, SET_0N1MS_TMR	    ;
    dec	    _Timer0n1ms		    ;��������� ������0.1ms
    bra	    nz, DEC_10MS_TMR	    ;���� ������0.1ms ����� ����, ��
SET_0N1MS_TMR:
    bset    _FlagReg, #T0n1msFl	    ;���� �������0.1ms ����� 1
DEC_10MS_TMR:
    dec.b   TMR10ms		    ;
    bra	    nz, T2_INT_END	    ;
    mov.b   #100, W0		    ;	    
    mov.b   WREG, TMR10ms	    ;
    bset    ADCON3L, #SWCTRG	    ;
    btsc    _FlagReg, #T10msFl	    ;���� ���� �������10ms ����� ����, ��
    bra     BLINK_PROG		    ;
    cp0	    _Timer10ms		    ;
    bra	    z, SET_10MS_TMR	    ;
    dec     _Timer10ms		    ;��������� ������10ms
    bra     nz, BLINK_PROG	    ;���� ������10ms ����� ����, ��
SET_10MS_TMR:
    bset    _FlagReg,#T10msFl	    ;���� �������10ms ����� 1  
BLINK_PROG:
    dec     BlinkCnt		    ;
    bra     nz, SET_T2_INT_FL	    ;
    mov     #50, W0		    ;
    mov     W0, BlinkCnt	    ;
    btg     _FlagReg,#BlinkFl	    ;
SET_T2_INT_FL:
    bset    _FlagReg, #T2Tic	    ;
T2_INT_END:
    pop	    W1			    ;
    pop     W0			    ;
    bclr    IFS0, #T2IF		    ;
    retfie
;-------------------------------------------------------------------------------
;���������� �� IC1
    .global __IC1Interrupt  
__IC1Interrupt:
    push    W0			;
    mov	    IC1TMR, W0		;
    mov	    W0, Param		;
    bclr    IC1CON2, #TRIGSTAT	;
    bset    IC1CON2, #TRIGSTAT	;
    bset    _FlagReg, #IC1IntFl;
    bclr    IFS0, #IC1IF        ;
    pop	    W0			;
    retfie
;-------------------------------------------------------------------------------
;������������ ������/������ EEPROM
    .global _RD_EEPROM, _WR_EEPROM
_RD_EEPROM:
    push    W4
    mov     #EE_STAT_RD, W4     ;��������� ��������� ������
    mov     W4, EE_State        ;
    bra     EEPROM_NEXT
_WR_EEPROM:
    push    W4
    mov     #EE_STAT_WR, W4     ;��������� ��������� ������
    mov     W4, EE_State        ;
EEPROM_NEXT:
    bset    _FlagReg, #EEBusyFl ;��������� ����� ��������� ������ ������
    clr     CntEEBuf            ;������� �������� ���� ��� ������
    mov     W0, EEBufPtr        ;��������� ������
    mov     W1, EEPROM_Ad       ;
    mov     W2, EELength        ;
SEND:
    pop     W4                  ;
    return                      ;
;------------------------------------------------------------------------------------------------
    .global _SetMIGMotorPWM
_SetMIGMotorPWM:
    mov     W0, OC1R		;
    return                      ;
;------------------------------------------------------------------------------------------------
    .global _SetValve
_SetValve:
    bset    LATC, #RC3		;
    return			;
;------------------------------------------------------------------------------------------------
    .global _ResValve
_ResValve:
    bclr    LATC, #RC3		;
    return			;
;------------------------------------------------------------------------------------------------
;������������� �������
    .global _DispInit
_DispInit:

    return
;------------------------------------------------------------------------------------------------
    .global _CalcUPIDRatio
_CalcUPIDRatio:
;������ UK_A
    push    W3
    clr     W3
    add     W3, W0, W3
    add     W3, W1, W3
    add     W3, W2, W3
    mov     W3, UK_A
;������ UK_B
    clr     W3
    mov     W2, W3
    sl      W3, #1, W3
    add     W0, W3, W3
    neg     W3, W3
    mov     W3, UK_B
;������ UK_C
    mov     W2, UK_C
    pop	    W3
    return
    