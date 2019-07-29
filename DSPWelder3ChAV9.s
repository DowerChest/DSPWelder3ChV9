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
;инициализация бортового оборудования МК
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
;RA0 - аналоговый вход напряжения:1
;RA1 - аналоговый вход тока:1
;RA2 - аналоговый вход сопротивления задания параметров :1
;RA3 - PWM1H (вход):1
;RA4 - PWM1L (вход):1
;TRISA=1111 1111 1111 1111
    mov	    #0xffff,W0
    mov     W0,TRISA
    mov	    #0x0007,W0
    mov	    W0,ANSELA
    clr     ODCA
    clr	    CNENA
    clr	    LATA
;RB0 - RP32 -> SPI1 SCK (вход):1
;RB1 - вход внешнего генератора (вход):1
;RB2 - TMP1 - вход температуры 1-го канала (вход):1
;RB3 - TMP2 - вход температуры 2-го канала (вход):1
;RB4 - TMP3 - вход температуры 3-го канала (вход):1
;RB5 - RP37 -> SPI2 SDI (MISO) вход данных SPI2 :1 
;RB6 - PGED1 (выход):0
;RB7 - PGEC1 (выход):0
;RB8 - RP40 -> SPI2 SCK (вход):1
;RB9 - RP41 -> SPI1 SDO (вход):1
;RB10 - RP42 -> SPI1 SDI (вход):1
;RB11 - RP43 -> IC1 вход для удаленного ввода параметров в режиме MIG (вход):1
;RB12 - RP44 -> OC1 выход для управления мотором в режиме MIG (вход): 1
;RB13 - PWM2H (вход):1
;RB14 - PWM2L (вход):1
;RB15 - RP47 -> SPI2 SDO (вход):1
;TRISB=1111 1111 1111 1111
    mov     #0xffff,W0
    mov     W0,TRISB
    mov	    #0x001c, W0
    mov	    W0, CNPUB
    clr	    ANSELB
    clr     ODCB
    clr	    CNENB
    clr     LATB
;RC0 - SPB - кнопка выбора режима "Пар/Син/Прг" (вход):1
;RC1 - не используется (выход):0
;RC2 - TMP4 - вход температуры 4-го канала (вход):1
;RC3 - VLV - выход на клапан газа (выход):0
;RC4 - EN# - выход разрешения индикаторов (выход):0
;RC5 - MIGBut - вход кнопки на горелке (вход):1
;RC6 - не используется (выход):0
;RC7 - HPG - вход "высокое напряжение в норме" (вход):1
;RC8 - LATCH - выход защелки индикатора (выход):0
;RC9 - CS - выход CS EEPROM (выход):0
;RC10 - MemB - вход кнопки "Память" (вход):1
;RC11 - GasB - вход кнопки "Переключение типа гаэа" (вход):1
;RC12 - SelB - вход кнопки "Выбор изменяемого резистором параметра" (вход):1
;RC13 - T24B - вход кнопки "T2/T4" (вход):1
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
;разблокировка конфигурации переназначаемых выводов	
    mov     #OSCCON, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   W2, [W1]
    mov.b   W3, [W1]
    bclr    OSCCON, #IOLOCK
;конфигурирование переазначаемых входов
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
;конфигурирование переазначаемых выходов
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
;блокировка конфигурации переназначаемых выводов	
    mov     #OSCCON, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   W2, [W1]
    mov.b   W3, [W1]
    bset    OSCCON, #IOLOCK
;Fcy=Fosc/2=1/2((Fin*M)/(N1*N2))=1/2((8 000 000*60)/(2*2))=60 000 000 Гц
;новая величина М=60 (PLLFBD=58)
    mov     #0x003A, W0
    mov     W0, PLLFBD
;новая величина N2=2 (PLLPOST=0), новая величина N1=2 (PLLPRE=0)
    mov     #0x0000, W0
    mov     W0, CLKDIV
;размещение нового типа генератора (NOSC=0b011 первичный генератор с ФАПЧ) в W0
    mov     #0x0003,W0
;разблокировка OSCCONH (старший байт) 
    mov     #OSCCONH, W1
    mov     #0x78, W2
    mov     #0x9A, W3
    mov.b   W2, [W1]		; запись 0x0078
    mov.b   W3, [W1]		; запись 0x009A
;установка нового типа генератора
    mov.b   WREG, OSCCONH
;разблокировка OSCCONL (младший байт)
    mov     #OSCCONL, W1
    mov     #0x46, W2
    mov     #0x57, W3
    mov.b   w2,[W1]		;запись 0x0046
    mov.b   w3,[W1]		;запись 0x0057
;разрешение переключния тактирования
    bset    OSCCON, #OSWEN      ;запрос на переключение тактирования установкой бита OSWEN bit
;ожидание завершения преключения тактирования
WaitOSCSwitch:	
    btsc    OSCCON, #OSWEN      ;
    bra	    WaitOSCSwitch       ;
;ожидание блокировки основной PLL
WaitMainPLLLock:
    btss    OSCCON,#LOCK	;
    bra	    WaitMainPLLLock     ;
;включение отдельного генератора для ШИМ и АЦП с использованием внутреннего генератора в качестве REFCLK
;ACLK=(Fin*M)/N=(7 370 000 * 16)/1 = 117 920 000
    bclr    ACLKCON,#ASRCSEL	;источник импульсов - первичный генератор
    bset    ACLKCON,#FRCSEL	;источник импульсов определен установкой бита ARCSEL
    bset    ACLKCON,#SELACLK	;умножитель - источник для делителя
    bset    ACLKCON,#APSTSCLR2	;
    bset    ACLKCON,#APSTSCLR1	;делитель установлен 1:1
    bset    ACLKCON,#APSTSCLR0	;
    bset    ACLKCON,#ENAPLL	;дополнительный частотный умножитель включен
;ожидание завершения включения дополнительной ФАПЧ
WaitAuxPLLLock:
    btss    ACLKCON,#APLLCK	;
    bra     WaitAuxPLLLock	;
;инициализация ШИМ
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
;инициализация ШИМ канал 1
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
;инициализация ШИМ канал 2
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
;Инициализация АЦП
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
;Инициализация SPI1
    mov	    #SPI1STAT_INIT,W0	;
    mov	    W0,SPI1STAT		;
    mov     #SPI1CON1_INIT,W0	;
    mov	    W0,SPI1CON1     	;
    mov     #SPI1CON2_INIT,W0	;
    mov	    W0,SPI1CON2     	;
;Инициализация SPI2
    mov	    #SPI2STAT_INIT,W0	;
    mov	    W0,SPI2STAT		;
    mov     #SPI2CON1_INIT,W0	;
    mov	    W0,SPI2CON1     	;
    mov     #SPI2CON2_INIT,W0	;
    mov	    W0,SPI2CON2     	;
;Инициализация TMR2
    mov     #T2CON_INIT, W0     ;
    mov     W0, T2CON           ;
    mov     #PR2_INIT, W0       ;
    mov     W0, PR2             ;
    clr     TMR2                ;
;Инициализация OC1
    mov     #OC1CON1_INIT, W0   ;
    mov     W0, OC1CON1         ;
    mov     #OC1CON2_INIT, W0   ;
    mov     W0, OC1CON2         ;
    mov     #OC1RS_INIT, W0     ;
    mov     W0, OC1RS           ;
    mov     #OC1R_INIT, W0      ;
    mov     W0, OC1R            ;
;Инициализация IC1
    mov	    #IC1CON1_INIT, W0	;
    mov	    W0, IC1CON1		;
    mov	    #IC1CON2_INIT, W0	;
    mov	    W0, IC1CON2		;
    bset    IC1CON2, #ICTRIG	;
;Приоритеты
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
;приоритет прерываний от АЦП
    mov     #ADC_IPR_MASK,W0	;
    and     IPC27		;
    mov     #ADC_IPR, W0	;
    ior     IPC27		;
;приоритет прерываний от фильтра 0 АЦП
    mov     #ADC_FLT0_IPR_MASK,W0
    and     IPC44		;
    mov     #ADC_FLT0_IPR, W0	;
    ior     IPC44		;   
;приоритет прерываний от SPI1
    mov     #SPI1_IPR_MASK, W0	;
    and     IPC2		;
    mov     #SPI1_IPR, W0	;
    ior     IPC2		;
;приоритет прерываний от SPI2
    mov     #SPI2_IPR_MASK, W0	;
    and     IPC8		;
    mov     #SPI2_IPR, W0	;
    ior     IPC8		;
;приоритет прерываний от таймера 2
    mov     #T2_IPR_MASK, W0
    and     IPC1		;
    mov     #T2_IPR, W0		;
    ior     IPC1		;
;приоритет прерываний от IC1
    mov	    #IC1_IPR_MASK, W0	;
    and	    IPC0		;
    mov	    #IC1_IPR, W0	;
    ior	    IPC0		;
;инициализация программных переменных
;ПИД по напряжению
    clr     UError_n
    clr     UError_n_1
    clr     UError_n_2
    mov     #U_PID_PROP_GAIN, W1
    mov     #U_PID_INT_GAIN, W2
    mov     #U_PID_DERIV_GAIN, W3
;расчет UK_A
    clr     W0
    add     W0, W1, W0
    add     W0, W2, W0
    add     W0, W3, W0
    mov     W0, UK_A
;расчет UK_B
    clr     W0
    mov     W3, W0
    sl      W0, #1, W0
    add     W1, W0, W0
    neg     W0, W0
    mov     W0, UK_B
;расчет UK_C
    mov     W3, UK_C
;ПИД по току
    clr     IError_n
    clr     IError_n_1
    clr     IError_n_2
    mov     #I_PID_PROP_GAIN, W1
    mov     #I_PID_INT_GAIN, W2
    mov     #I_PID_DERIV_GAIN, W3
;расчет IK_A
    clr     W0
    add     W0, W1, W0
    add     W0, W2, W0
    add     W0, W3, W0
    mov     W0, IK_A
;расчет IK_B
    clr     W0
    mov     W3, W0
    sl      W0, #1, W0
    add     W1, W0, W0
    neg     W0, W0
    mov     W0, IK_B
;расчет IK_C
    mov     W3, IK_C
;инициализация DSP ядра
    bclr    CORCON, #US1
    bclr    CORCON, #US0
    bclr    CORCON, #EDT
    bset    CORCON, #SATA
    bset    CORCON, #SATB
    bset    CORCON, #SATDW
    bclr    CORCON, #ACCSAT
    bset    CORCON, #IF
;инициализация SPI
    SetCSEEPROM
    clr.b   CntEEBuf		;
;инициализация регистра флагов
    clr     _FlagReg		;
;DS80439L п.32 (продолжение; начало - см. слово инициализации IOCON1 в файле MCPIDReg.inc)
;запрещение перезаписи выводов PWM1H и PWM1L, PWM2H и PWM2L
    bclr    IOCON1, #OVRENH	;
    bclr    IOCON1, #OVRENL	;
    bclr    IOCON2, #OVRENH	;
    bclr    IOCON2, #OVRENL	;
;задержка более, чем на один период ШИМ
    repeat  #OVERRIDE_DELAY	;
    nop				;
;переключение ШИМ выводов от CPU к модулю ШИМ
    bset    IOCON1, #PENH	;
    bset    IOCON1, #PENL	;
    bset    IOCON2, #PENH	;
    bset    IOCON2, #PENL	;
;выводы ШИМ PWM3H и PWM3L не используются и их владельцем является порт B
    bclr    IOCON3,#PENL	;
    bclr    IOCON3,#PENH	;
;запуск используемого бортового оборудования
;калибровка и запуск каналов AN0, AN1 и AN2 АЦП
    mov	    #0x0f00,W0		;установка времени инициализации АЦП в максимум
    ior	    ADCON5H		;
    bset    ADCON1L,#ADON	;общее включение АЦП
    bset    ADCON5L,#C0PWR	;включение аналогового питания канала 0 АЦП
WAIT_CH0_ADC:
    btss    ADCON5L,#C0RDY	;ожидание включения канала 0 АЦП
    bra	    WAIT_CH0_ADC	;
    bset    ADCON3H,#C0EN	;включение цифрового питания триггеров для канала 0 АЦП
    bset    ADCON5L,#C1PWR	;включение аналогового питания канала 1 АЦП
WAIT_CH1_ADC:
    btss    ADCON5L,#C1RDY	;ожидание включения канала 1 АЦП
    bra	    WAIT_CH1_ADC	;
    bset    ADCON3H,#C1EN	;включение цифрового питания триггеров для канала 1 АЦП
    bset    ADCON5L,#C2PWR	;включение аналогового питания канала 2 АЦП
WAIT_CH2_ADC:
    btss    ADCON5L,#C2RDY	;ожидание включения канала 2 АЦП
    bra	    WAIT_CH2_ADC	;
    bset    ADCON3H,#C2EN	;включение цифрового питания триггеров для канала 2 АЦП    
    
    bset    ADCAL0L,#CAL0EN	;разрешение калибровки канала 0 АЦП
    bclr    ADCAL0L,#CAL0DIFF	;калибровка униполярного входа канала 0 АЦП
    bset    ADCAL0L,#CAL0RUN	;старт калибровки канала 0 АЦП
WAIT_CH0_CAL_UNI:
    btss    ADCAL0L,#CAL0RDY	;ожидание завершения калибровки униполярного входа канала 0 АЦП
    bra	    WAIT_CH0_CAL_UNI	;
    bset    ADCAL0L,#CAL0DIFF	;калибровка дифференциального входа канала 0 АЦП
    bset    ADCAL0L,#CAL0RUN	;старт калибровки канала 0 АЦП
WAIT_CH0_CAL_DIFF:
    btss    ADCAL0L,#CAL0RDY	;ожидание завершения калибровки дифференциального входа канала 0 АЦП
    bra	    WAIT_CH0_CAL_DIFF	;
    bclr    ADCAL0L,#CAL0EN	;окончание калибровки канала 0 АЦП   
    bset    ADCAL0L,#CAL1EN	;разрешение калибровки канала 1 АЦП
    bclr    ADCAL0L,#CAL1DIFF	;калибровка униполярного входа канала 1 АЦП
    bset    ADCAL0L,#CAL1RUN	;старт калибровки канала 1 АЦП
WAIT_CH1_CAL_UNI:
    btss    ADCAL0L,#CAL1RDY	;ожидание завершения калибровки униполярного входа канала 1 АЦП
    bra	    WAIT_CH1_CAL_UNI	;
    bset    ADCAL0L,#CAL1DIFF	;калибровка дифференциального входа канала 1 АЦП
    bset    ADCAL0L,#CAL1RUN	;старт калибровки канала 1 АЦП
WAIT_CH1_CAL_DIFF:
    btss    ADCAL0L,#CAL1RDY	;ожидание завершения калибровки дифференциального входа канала 1 АЦП
    bra	    WAIT_CH1_CAL_DIFF	;
    bclr    ADCAL0L,#CAL1EN	;окончание калибровки канала 1 АЦП  
    bset    ADCAL0L,#CAL2EN	;разрешение калибровки канала 2 АЦП
    bclr    ADCAL0L,#CAL2DIFF	;калибровка униполярного входа канала 2 АЦП
    bset    ADCAL0L,#CAL2RUN	;старт калибровки канала 2 АЦП
WAIT_CH2_CAL_UNI:
    btss    ADCAL0L,#CAL2RDY	;ожидание завершения калибровки униполярного входа канала 2 АЦП
    bra	    WAIT_CH2_CAL_UNI	;
    bset    ADCAL0L,#CAL2DIFF	;калибровка дифференциального входа канала 2 АЦП
    bset    ADCAL0L,#CAL2RUN	;старт калибровки канала 2 АЦП
WAIT_CH2_CAL_DIFF:
    btss    ADCAL0L,#CAL2RDY	;ожидание завершения калибровки дифференциального входа канала 2 АЦП
    bra	    WAIT_CH2_CAL_DIFF	;
    bclr    ADCAL0L,#CAL2EN	;окончание калибровки канала 2 АЦП    
;инициализация суммы и счетчика АЦП
    clr	    UADSum		;
    clr	    RADSum		;
;инициализация автомата состояний DM13
    mov.b   #DM13Idle, W0	;
    mov.b   WREG, DM13State	; 
;разрешение вложенных прерываний
    bclr    INTCON1,#NSTDIS	;
    bset    INTCON2,#GIE	;
    bclr    CORCON,#IPL3	;
    mov     #0xff1f,W0		;Приоритет прерываний CPU=0
    and     SR			;
    clr	    RCON		;
;запуск таймера 2
    mov.b   #10, W0		;	    
    mov.b   WREG, TMR10ms	;
    mov     #50, W0             ;
    mov     W0, BlinkCnt        ;
    bset    T2CON, #TON         ;
;разрешение используемых источников прерываний
;разрешение прерываний от TMR2
    bclr    IFS0, #T2IF		;
    bset    IEC0, #T2IE		;
;инициализация дисплея
    call    _DispInit
;разрешение прерываний от ADC AN0, AN1 и фильтра 0
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
;разрешение прерываний от IC1
    bclr    IFS0, #IC1IF	;
    bset    IEC0, #IC1IE	;
;инициализация программы
    
    call    _InitProg
;запуск ШИМ
    bset    PTCON, #PTEN        ;
;запуск SPI1 с учетом Errata DS80000656C п. 28.SPI. Work around №1
SPI1_START:
    SetCSEEPROM			;сигнал CS EEPROM устанавливается в 1
    bclr    SPI1STAT,#SPIROV    ;включение модуля SPI1
    bset    SPI1STAT,#SPIEN     ;
    clr	    W0			;первый цикл SPI
    bclr    IFS0, #SPI1IF	;
    mov	    W0, SPI1BUF		;
FIRST_SPI1_CYCLE:
    btss    IFS0, #SPI1IF	;
    bra	    FIRST_SPI1_CYCLE	;
    mov	    SPI1BUF, W0		;
    clr	    W0			;второй цикл SPI
    bclr    IFS0, #SPI1IF	;
    mov	    W0, SPI1BUF		;
SECOND_SPI1_CYCLE:
    btss    IFS0, #SPI1IF	;
    bra	    SECOND_SPI1_CYCLE	;
    mov	    SPI1BUF, W0		;
    bclr    SPI1STAT,#SPIEN     ;выключение модуля SPI1
    repeat  #200		;ожидание 200 циклов
    nop				;
    bclr    SPI1STAT,#SPIROV    ;повторное включение модуля SPI1
    bset    SPI1STAT,#SPIEN     ;
;разрешение прерываний от SPI1
    bclr    IFS0, #SPI1IF	;
    bset    IEC0, #SPI1IE	;
;пробная запись в EEPROM
    mov	    #0x55aa, W0		;
    mov	    W0, EEPROMData	;
    mov	    #EEPROMData, W0	;
    mov	    #0x07fe, W1		;
    mov	    #0x0002, W2		;
    rcall   _WR_EEPROM		;
WAIT_WR_EEPROM:
    btsc    _FlagReg, #EEBusyFl	;ожидание завершения операции записи
    bra	    WAIT_WR_EEPROM	;
;пробное чтение из EEPROM
    mov	    #EEPROMData, W0	;    
    mov	    #0x07fe, W1		;
    mov	    #0x0002, W2		;
    rcall   _RD_EEPROM		;
WAIT_RD_EEPROM:
    btsc    _FlagReg, #EEBusyFl	;ожидание завершения операции чтения
    bra	    WAIT_RD_EEPROM	;    
    mov	    EEPROMData, W0	;
    mov	    #0x55aa, W1		;
    cp	    W0, W1		;
    bra	    z, SPI2_START	;если записанные данные не совпали с прочитанными, то:
    bclr    IEC0, #SPI1IE	; запрет прерываний от SPI
    bclr    SPI1STAT,#SPIEN     ; выключение модуля SPI
    repeat  #200		; ожидание 200 циклов
    nop				;    
    bra	    SPI1_START		;переход к перезапуску модуля SPI
;запуск SPI2 с учетом Errata DS80000656C п. 28.SPI. Work around №1
SPI2_START:
    bclr    SPI2STAT,#SPIROV    ;включение модуля SPI2
    bset    SPI2STAT,#SPIEN     ;
    clr	    W0			;первый цикл SPI
    bclr    IFS2, #SPI2IF	;
    mov	    W0, SPI2BUF		;
FIRST_SPI2_CYCLE:
    btss    IFS2, #SPI2IF	;
    bra	    FIRST_SPI2_CYCLE	;
    mov	    SPI2BUF, W0		;
    clr	    W0			;второй цикл SPI
    bclr    IFS2, #SPI2IF	;
    mov	    W0, SPI2BUF		;
SECOND_SPI2_CYCLE:
    btss    IFS2, #SPI2IF	;
    bra	    SECOND_SPI2_CYCLE	;
    mov	    SPI2BUF, W0		;
    bclr    SPI2STAT,#SPIEN     ;выключение модуля SPI2
    repeat  #200		;ожидание 200 циклов
    nop				;
    bclr    SPI2STAT,#SPIROV    ;повторное включение модуля SPI2
    bset    SPI2STAT,#SPIEN     ;
    mov	    #0x55aa, W0		;пересылка проверочного слова 6 раз в буфер SPI2
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    mov	    W0, SPI2BUF		;
    bclr    IFS2, #SPI2IF	;очистка флага прерываний
SPI2_WAIT1:
    btss    IFS2, #SPI2IF	;ожидание передачи всех слов из
    bra	    SPI2_WAIT1		;аппаратного буфера SPI2
SPI2_PROB_RD:
    mov	    SPI2BUF, W0		;чтение всех байт из аппаратного 
    btss    SPI2STAT, #SRXMPT	;приемного буфера SPI2
    bra	    SPI2_PROB_RD	;
    mov	    #0x55aa, W1		;
    cp	    W0, W1		;
    bra	    z, SPI2_INT_EN	;
    bclr    IEC2, #SPI2IE	; запрет прерываний от SPI2
    bclr    SPI2STAT,#SPIEN     ; выключение модуля SPI2
    repeat  #200		; ожидание 200 циклов
    nop				;  
    bra	    SPI2_START		;
SPI2_INT_EN:  
;разрешение прерываний от SPI2
    bclr    IFS2, #SPI2IF	;
    bset    IEC2, #SPI2IE	;

;инициализация дисплея
DISPLAY_INIT:
    call    _InitDisplay
;------------------------------------------------------------------------------------------------
;основной цикл
Main:
;управление дугой
    btss    _FlagReg, #ADCP0IntFl
    bra	    ADCP1_TST
;регулятор напряжения
;если флаг расчета ПИД не установлен, то расчет не выполняется, цикл имеет минимальную длительность
    btsc    _FlagReg, #PIDCalc  ;
    bra     U_PID_CALC          ;
    btss    _FlagReg, #PIDModeFl;
    bra	    U_PID_LOAD_I	;
    mov     #DUTY_MIN, W0       ;
    bra     U_PWM_LOAD_U	;
;петля обратной связи по напряжению
;инициализация указателей на массив для напряжений в памяти данных
U_PID_CALC:
    mov     #UError_n, W10
    mov     #UK_A, W8
    mov     #Ufb, W3            ; указатль на текущее выходное напряжение
;вычисление ошибки по напряжению
    mov     [W3], W0
;вычисление пропорциональной ошибки
;ep = _USetM - заданное выходное напряжение
;ep [W1] = _USetM - Ufb
    mov     _USetM, W1
    sub     W1, W0, W0		; W0 = новая пропорциональная ошибка напряжния
    mov     W0, [W10]
;выполнение вычислений ПИД
    mov     #UCtrl, W2
    movsac  A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, A
;сохранение округленной величины
    sac.r   A, -#8, [W2]
;создание еще одной копии
    mov     [W2],W3
;проверка заполнния по напряжению на максимум
;фиксация максимального заполнения по напряжению
;если режим PID "холодный" (флаг режима PID =1), то выход регулятора ограничен максимальной длительностью в MDC,
;иначе выход ограничен максимальным измеренным током *8, т. е. разрядностью АЦП * 8
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
;проврка заполнения по напряжению на минимум
;если режим "холодный" (флаг режима PID = 1), то минимум - минимальная длительность цикла,
;иначе минимум - минимальный ток
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
;обновлние буфера ошибок по напряжению
    mov     #UError_n, W10
    mov     [W10+2], W2
    mov     W2, [W10+4]
    mov     [W10], W2
    mov     W2, [W10+2]   
;если рассчитанное заполнение по напряжению меньше, чем заполнение по току, то обновление MDC
;при условии, что режим - "холодный" (Флаг режима PID = 1)
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
;регулятор тока
ADCP1_TST:
    btss    _FlagReg, #ADCP1IntFl
    bra     IC1_TST		;
;если флаг расчета ПИД не установлен, то расчет не выполняется
    btsc    _FlagReg, #PIDCalc  ;
    bra     I_PID_CALC          ;
    mov     #DUTY_MIN, W0       ;
    bra     I_PWM_LOAD		;
;токовая петля обратной связи
;инициализация указателей на массив для токов в памяти данных
I_PID_CALC:
    mov     #IError_n, W10
    mov     #IK_A, W8
    mov     #Ifb, W3		; указатль на текущий выходной ток
    mov     _ISetM, W1		; опора - установленное значение
    btss    _FlagReg, #PIDModeFl;если режим "горячий" (флаг режима PID=0), то уставка делится на 8
    asr	    W1, #3, W1
;вычисление ошибки по току
    mov     [W3], W0
; вычисление пропорциональной ошибки
; ep = _ISetM - заданное выходное напряжение
; ep [W1] = _ISetM - Ifb
    sub     W1, W0, W0	; W0 = новая пропорциональная ошибка тока
    mov     W0, [W10]
;выполнение вычислений ПИД
    mov     #ICtrl, W2
    movsac  B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B, [W8]+=2, W6, [W10]+=2, W7
    mac     W6*W7, B
;сохранение округленной величины
    sac.r   B, -#8, [W2]
;создание еще одной копии
    mov     [W2],W3
;проверка заполнния по току на максимум
;фиксация максимального заполнения по току
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
;проверка заполнения по току на минимум
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
;обновлние буфера ошибок по току
    mov     #IError_n, W10
    mov     [W10+2], W2
    mov     W2, [W10+4]
    mov     [W10], W2
    mov     W2, [W10+2]
;если рассчитанное заполнение по току меньше, чем заполнение по напряжению, то обновление MDC
;при условии, что режим - "холодный" (Флаг режима PID = 1)
    mov     ICtrl, W0
    btss    _FlagReg, #PIDModeFl
    bra	    I_PWM_LOAD
    mov     UCtrl, W3
    cpsgt   W0, W3
I_PWM_LOAD:
    mov     W0, MDC
    call    _ArcControl
    bclr    _FlagReg, #ADCP1IntFl
;получение первого параметра через IC1
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
;опрос входов
T2_TIC_TST:
    btss    _FlagReg, #T2Tic
    bra     CYCLE_END
    mov.b   _ButReg, WREG
    mov.b   WREG, _OldButReg
;подавление дребезга кнопок и датчиков
;подавление дребезга реле "Высокое в норме"
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
;подавление дребезга реле "термо1"
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
;подавление дребезга реле "термо2"
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
;подавление дребезга реле "термо3"
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
;подавление дребезга реле "термо4"
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
;подавление дребезга кнопки выбора режима "Пар/Син/Прг"
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
;подавление дребезга кнопки "память"
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
;подавление дребезга кнопки выбора типа газа
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
;подавление дребезга кнопки выбора параметра для изменения резистором
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
;подавление дребезга кнопки 2T/4T
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
;подавление дребезга кнопки "MIG"
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
;дисплей
DISPLAY:
    call    _Display
;сброс флага режима (переход из режима MIG в режим MMA)
    dec	    IC1IntCnt
    mov	    IC1IntCnt, W0
    cp0	    W0
    bra	    gt, RES_T2_TIC
    inc	    IC1IntCnt
    bclr    _FlagReg, #ModeFl
RES_T2_TIC:
    bclr    _FlagReg, #T2Tic
CYCLE_END:   
;конец основного цикла
    goto    Main		
;------------------------------------------------------------------------------------------------
;прерывания от канала AN0 АЦП
    .global __ADCAN0Interrupt
__ADCAN0Interrupt:
;чтение напряжения из АЦП
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
;прерывания от канала AN1 АЦП
    .global __ADCAN1Interrupt
__ADCAN1Interrupt:
    push    W0			;
    push    W1			;
    mov     ADCBUF1, W0         ;
    mov     W0, Ifb             ;
    mov     W0, _IfbCtrl        ;
;таймер для управления дугой
    btsc    _FlagReg,#ArcContrFl;если флаг таймра дуги равен нулю, то
    bra     SYNCHRO_SPI1	;
    dec     _ArcTimer           ;умньшение таймра дуги
    bra     nz, SYNCHRO_SPI1	;если таймер дуги равен нулю, то
    bset    _FlagReg,#ArcContrFl;флаг таймера дуги равен 1
;синхронизация EEPROM с тактами ШИМ и АЦП
SYNCHRO_SPI1:
    btss    _FlagReg,#EEBusyFl  ;если(!ФлагЗанятостиEEPROM)
    bra     ADC1_INT_END	;{
    mov     EE_State,W0         ;   переключатель(СостояниеEEPROM)
    mov     #EE_STAT_RD,W1      ;   {
    cp      W0,W1               ;
    bra     z, RD_IDLE_ADC1     ;
    mov     EE_State,W0         ;
    mov     #EE_STAT_WR,W1      ;
    cp      W0,W1               ;
    bra     z, WR_IDLE_ADC1	;
    bra     ADC1_INT_END        ;	умолчание: выход;
RD_IDLE_ADC1:			;	случай СостояниеЧтения:
    ClrCSEEPROM			;	    CSEEPROM=0;
    mov     #EE_STAT_RD,W0      ;           СостояниеEEPROM=СостояниеЧтения;
    mov     W0,EE_State         ;
    mov	    #0x05,W0            ;           Выдача команды чтения регистра состояния EEPROM;
    mov	    W0,SPI1BUF		;
    bra     ADC1_INT_END        ;           выход;
WR_IDLE_ADC1:                   ;       случай СостояниеЗаписи:
    ClrCSEEPROM                 ;           CSEEPROM=0;                   
    mov     #EE_STAT_WR,W0      ;	    СостояниеEEPROM=СостояниеЗаписи;
    mov     W0,EE_State         ;           Выдача команды чтения регистра состояния EEPROM;
    mov	    #0x05,W0            ;              }
    mov	    W0,SPI1BUF		;       }
;конец прерывания от АЦП
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
;прерывания от SPI1
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
;состояние пересылки пустого байта для чтения регистра состояния EEPROM при чтении
EE_RD_STAT:
    mov     SPI1BUF,W0          ;
    mov     #IDLE_EE_RD,W0      ;
    mov     W0,EE_State         ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние чтения регистра состояния EEPROM при чтении
EE_RD_IDLE:
    SetCSEEPROM
    mov     SPI1BUF,W0          ;
    btss    W0,#WIP             ;если EEPROM занято записью, то
    bra     WelRdTest           ;
    mov     #EE_STAT_RD,W0      ;переход к состоянию
    mov     W0,EE_State         ;чтения регистра состояния EEPROM
    bra     SPI1IntEnd          ;при чтении 
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
    ClrCSEEPROM                 ;иначе -
    mov     #EE_RD_ADM,W0       ;
    mov     W0,EE_State         ;
    mov     #0x0003,W0          ;пересылка команды чтения
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние пересылки старшего байта адреса при чтении
EE_ADM_RD:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    lsr     W1,#8,W1            ;
    mov     #EE_RD_ADL,W0       ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние пересылки младшего байта адреса при чтении
EE_ADL_RD:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    mov     #0x00ff,W0          ;
    and     W1,W0,W1            ;
    mov     #EE_RD_DATA, W0     ;
    mov     W0, EE_State        ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние посылки пустого байта для чтения данных
EE_RD_DT:
    mov     SPI1BUF, W0         ;
    mov     #EE_MOVE_BYTE, W0   ;
    mov     W0, EE_State        ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние пересылки прочитанного байта в буфер
EE_RD_BYTE:
    SetCSEEPROM                 ;
    mov     EEBufPtr, W0        ;вычисление адреса
    add     CntEEBuf, WREG      ;прочитанного байта
    mov     SPI1BUF, W1         ;
    mov.b   W1,[W0]             ;
    inc     EEPROM_Ad           ;увеличение адреса в EEPROM
    inc     CntEEBuf            ;увеличение счетчика прочитанных байт
    mov     EELength,W0         ;
    cp      CntEEBuf            ;если счетчик прочитанных байт не равен заданной длине, то
    bra     ge,EEReadEnd        ;
    mov     #EE_STAT_RD,W0      ;цикл чтения продолжается
    mov     W0,EE_State         ;
    bra     SPI1IntEnd          ;
EEReadEnd:
    bclr    _FlagReg,#EEBusyFl  ;иначе - конец цикла чтения
    bra     SPI1IntEnd          ;
;состояние пересылки пустого байта для чтения регистра состояния EEPROM при записи
EE_WR_STAT:
    mov     SPI1BUF,WREG        ;
    mov     #IDLE_EE_WR,W0      ;
    mov     W0,EE_State         ;
    clr     W0                  ;
    mov     W0,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние чтения регистра состояния EEPROM при записи
EE_WR_IDLE:
    SetCSEEPROM
    mov     SPI1BUF, WREG       ;
    btss    W0,#0x00            ;если EEPROM занято записью, то
    bra     EEWrEnSet           ;переход к состоянию
    mov     #EE_STAT_WR, W0     ;чтения регистра состояния EEPROM
    mov     W0,EE_State         ;при записи
    bra     SPI1IntEnd          ;
EEWrEnSet:
    ClrCSEEPROM                 ;
    mov     #EE_WR_EN, W0       ;иначе -
    mov     W0, EE_State        ;пересылка команды разрешения
    mov.b   #0x06, W1           ;записи в EEPROM
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние разрешение записи
EE_WR_ENA:
    SetCSEEPROM                 ;
    mov     SPI1BUF,W0          ;
    mov     #0x0002,W1          ;пересылка команды записи
    mov     #EE_WR_ADM, W0      ;
    mov     W0, EE_State        ;
    ClrCSEEPROM                 ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние пересылки старшего байта адреса
EE_ADM_WR:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad,W1        ;
    lsr     W1,#8,W1            ;
    mov     #EE_WR_ADL, W0      ;
    mov     W0, EE_State        ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние пересылки младшего байта адреса
EE_ADL_WR:
    mov     SPI1BUF,W0          ;
    mov     EEPROM_Ad, W1       ;
    mov     #0x00ff,W0          ;
    and     W1,W0,W1            ;
    mov     #EE_WR_DATA, W0     ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние записи байта
EE_WR_BYTE:
    mov     SPI1BUF,W0          ;
    mov     EEBufPtr,W0         ;вычисление адреса
    add     CntEEBuf,WREG       ;записываемого байта
    mov.b   [W0],W1             ;
    mov     #EE_POST_WR,W0      ;
    mov     W0,EE_State         ;
    mov     W1,SPI1BUF          ;
    bra     SPI1IntEnd          ;
;состояние окончания цикла записи байта
EE_WR_POST:
    SetCSEEPROM                 ;
    mov     SPI1BUF,WREG        ;
    inc     EEPROM_Ad           ;увеличение адреса в EEPROM
    inc     CntEEBuf            ;увеличение счетчика записанных байт
    mov     EELength, W0        ;
    cp      CntEEBuf            ;если счетчик записанных байт не равен заданной длине, то
    bra     ge,EEWriteEnd       ;
    mov     #EE_STAT_WR,W0      ;цикл записи продолжается
    mov     W0,EE_State         ;
    bra     SPI1IntEnd          ;
EEWriteEnd:
    bclr    _FlagReg,#EEBusyFl  ;иначе - конец записи
    bra     SPI1IntEnd          ;
EE_DIS_WR:
    SetCSEEPROM                 ;
    mov     SPI1BUF,W0          ;
    mov     #EE_STAT_RD,W0      ;переход к состоянию
    mov     W0,EE_State         ;чтения регистра состояния EEPROM           
;конец прерывания от SPI
SPI1IntEnd:
    bclr    IFS0,#SPI1IF	;
    pop     W1
    pop     W0                  ;
    retfie                      ;
;------------------------------------------------------------------------------------------------
;прерывания от SPI2 (DM13)
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
;прерывания от TMR2
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
;таймер с дискретой 0,1 мс
TST_0N1MS_TIMER:
    btsc    _FlagReg, #T0n1msFl	    ;если флаг таймера0.1ms равен нулю, то
    bra	    DEC_10MS_TMR	    ;
    cp0	    _Timer0n1ms		    ;если таймер0.1ms не равен нулю
    bra	    z, SET_0N1MS_TMR	    ;
    dec	    _Timer0n1ms		    ;умньшение таймра0.1ms
    bra	    nz, DEC_10MS_TMR	    ;если таймер0.1ms равен нулю, то
SET_0N1MS_TMR:
    bset    _FlagReg, #T0n1msFl	    ;флаг таймера0.1ms равен 1
DEC_10MS_TMR:
    dec.b   TMR10ms		    ;
    bra	    nz, T2_INT_END	    ;
    mov.b   #100, W0		    ;	    
    mov.b   WREG, TMR10ms	    ;
    bset    ADCON3L, #SWCTRG	    ;
    btsc    _FlagReg, #T10msFl	    ;если флаг таймера10ms равен нулю, то
    bra     BLINK_PROG		    ;
    cp0	    _Timer10ms		    ;
    bra	    z, SET_10MS_TMR	    ;
    dec     _Timer10ms		    ;умньшение таймра10ms
    bra     nz, BLINK_PROG	    ;если таймер10ms равен нулю, то
SET_10MS_TMR:
    bset    _FlagReg,#T10msFl	    ;флаг таймера10ms равен 1  
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
;прерывания от IC1
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
;подпрограмма чтения/записи EEPROM
    .global _RD_EEPROM, _WR_EEPROM
_RD_EEPROM:
    push    W4
    mov     #EE_STAT_RD, W4     ;установка состояния чтения
    mov     W4, EE_State        ;
    bra     EEPROM_NEXT
_WR_EEPROM:
    push    W4
    mov     #EE_STAT_WR, W4     ;установка состояния записи
    mov     W4, EE_State        ;
EEPROM_NEXT:
    bset    _FlagReg, #EEBusyFl ;установка флага занятости канала обмена
    clr     CntEEBuf            ;очистка счетчика байт для обмена
    mov     W0, EEBufPtr        ;пересылка адреса
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
;инициализация дисплея
    .global _DispInit
_DispInit:

    return
;------------------------------------------------------------------------------------------------
    .global _CalcUPIDRatio
_CalcUPIDRatio:
;расчет UK_A
    push    W3
    clr     W3
    add     W3, W0, W3
    add     W3, W1, W3
    add     W3, W2, W3
    mov     W3, UK_A
;расчет UK_B
    clr     W3
    mov     W2, W3
    sl      W3, #1, W3
    add     W0, W3, W3
    neg     W3, W3
    mov     W3, UK_B
;расчет UK_C
    mov     W2, UK_C
    pop	    W3
    return
    