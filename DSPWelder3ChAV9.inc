    .equ	__33EP32GS504, 1
    .include	"p33EP32GS504.inc"
;������������
;����������� ������� ����������� � � ���� ����� ����������, �������� ������� �� ������� � � ���� ����� ������, 
;�������������� ���������� ���������
    .section	__FSEC.sec, code
    .global	__FSEC
__FSEC:		.pword	BWRP_OFF & BSS_DISABLED & BSEN_OFF & GWRP_OFF & GSS_DISABLED & CWRP_OFF & CSS_DISABLED & AIVTDIS_OFF
;����� ������������ ��������
    .section	__FBSLIM.sec, code
    .global	__FBSLIM
__FBSLIM:	.pword	BSLIM_BSLIM
;����� ��� ��������������� ����������� �� ���������� ���������� ��� PLL
    .section	__FOSCSEL.sec, code
    .global	__FOSCSEL
__FOSCSEL:	.pword	FNOSC_FRC & IESO_OFF
;������������ ��������, ������� ��������,
;��������� ������������ ���������, ����� OSC ������ ����������
;��������� ��������� ��������
    .section	__FOSC.sec, code
    .global	__FOSC
__FOSC:		
    .pword	POSCMD_EC & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSECMD
;WDT ��������
    .section	__FWDT.sec, code
    .global	__FWDT
__FWDT:		
    .pword	WDTEN_OFF & WINDIS_OFF
;JTAG ��������, ������������ PGEC1 � PGED1 ��� ICD
    .section	__FICD.sec, code
    .global	__FICD
__FICD:		
    .pword	JTAGEN_OFF & ICS_PGD1
;����������� ������������������ ��� ������ ��� �� ������������, ��� ���������� �� ����� ������
    .section	__FDEVOPT.sec, code
    .global	__FDEVOPT
__FDEVOPT:		
    .pword	PWMLOCK_OFF & ALTI2C1_OFF & ALTI2C2_OFF & DBCC_OFF
;�������� ��������� ������� � �������
    .equiv	FAUXPLL, 117920000  ;
    .equiv	FPWM,66555          ;��������� ������� 33 ��� � ������ ��������� � ���� ������
    .equiv	PWM_PERIOD, (FAUXPLL*8 / FPWM);14143
    .equiv	DUTY_MIN,500        ;(1600 - 200 ��)
    .equiv	DUTY_MAX,PWM_PERIOD-8
    .equiv	OVERRIDE_DELAY,1500 ;�������� �����, ��� �� 1 ������ ��� (DS80439L �.32)
    .equiv	DEAD_TIME,(PWM_PERIOD*9/100);(PWM_PERIOD*9/100)������� ����� 9% �� �������
    .equiv	MEAS_TIME,(PWM_PERIOD*55/100);��������� � ����� 55% �� �������
;��������� ��� ����������� ���
    .equiv	PTCON_INIT, 0x0001  ;��� ������� � IDLE,
				    ;������� ������������� ���
				    ;���������� ����������
				    ;������� ������� ����������� �� ������� ������� ����������
    .equiv	PTCON2_INIT,0x0000  ;�������� ��� -> 1:1	
    .equiv	PTPER_INIT,PWM_PERIOD	;	
    .equiv	SEVTCMP_INIT,MEAS_TIME	;����� ������������ ��������� ��������
    .equiv	STCON_INIT, 0x0000  ;������ ����� ���������� ������� ��� �� ������������
    .equiv	STCON2_INIT, 0x0000 ;
    .equiv	STPER_INIT, 0x0000  ;
    .equiv	SSEVTCMP_INIT, 0x0000	;
    .equiv	CHOP_INIT, 0x0000   ;���������� ���������� �� ������������
    .equiv	MDC_INIT,DUTY_MIN   ;� �������� MDC ����������� ������������
;���������������� ������ 1-�� ������ ���
    .equiv	PWMCON1_INIT,0x0101 ;������ ������������ �������� PTPER
				    ;���������� �� �������� �������� ���������
				    ;������������ �������� � �������� MDC
				    ;���������� ������� �����
				    ;PWMH1 �����������, PWML1 ����������
				    ;������������ ������� �������� �������
				    ;����������� ������������ �� ������������
				    ;������� ����� ������������� �� ������������
				    ;���������� ������������ ��������� � ��������			
    .equiv	PDC1_INIT,0x0000    ;������������ �������� � MDC, PDC1 �� ������������
    .equiv	SDC1_INIT,0x0000    ;������ ������ ����������� �� �����������
    .equiv	PHASE1_INIT,0x0000  ;������� ����� ��� PWMH1 � PWML1 ����� 0
    .equiv	SPHASE1_INIT,0x0000 ;������� SPHASE1 �� ������������
    .equiv	DTR1_INIT,DEAD_TIME ;
    .equiv	ALTDTR1_INIT,DEAD_TIME	;
    .equiv	TRGCON1_INIT,0x0000 ;�������1 �� ������������
    .equiv	IOCON1_INIT, 0x0b00 ;����� PWM1H ����������� CPU (DS80000656B �.7)
				    ;����� PWM1L ����������� CPU (DS80000656B �.7) 
				    ;����������� ��. �������� ������ ��� � ����� MCPIDReg.asm)
				    ;�������� ������� �� PWM1H - ������
				    ;�������� ������� �� PWM1L - ������
				    ;��� � ������ Push-Pull
				    ;���������� PWM1H � PWM1L ��������, ����������� ���� (DS80000656B �.7)
				    ;������ �� ��������������
    .equiv	TRIG1_INIT,0x0000   ;�������1 �� ������������
    .equiv	STRIG1_INIT,0x0000  ;��������� �������1 �� ������������   
    .equiv	FCLCON1_INIT,0x0003 ;����� ������ ���������
    .equiv	LEBCON1_INIT,0x0000 ;���������� ������ �� ������������
    .equiv	LEBDLY1_INIT, 0x0000;������ �� ������������
    .equiv	AUXCON1_INIT, 0x0000;������� ���������� ���������� ���������
				    ;������� ���������� ������� ���������
				    ;������������� ���
				    ;��������� ��������� �������� ������� ���
				    ;���������� ���������� ������ PWM1H ���
				    ;���������� ���������� ������ PWM1L ���
;���������������� ������ 2-�� ������ ���
    .equiv	PWMCON2_INIT,0x0100 ;������ ������������ �������� PTPER
				    ;���������� �� �������� �������� ���������
				    ;������������ �������� � �������� MDC
				    ;���������� ������� �����
				    ;PWMH2 �����������, PWML2 ����������
				    ;������������ ������� �������� �������
				    ;����������� ������������ �� ������������
				    ;������� ����� ������������� �� ������������
				    ;���������� ������������ ��������� � ��������			
    .equiv	PDC2_INIT,0x0000    ;������������ �������� � MDC, PDC2 �� ������������
    .equiv	SDC2_INIT,0x0000    ;������ ������ ����������� �� �����������
    .equiv	PHASE2_INIT,PWM_PERIOD/2  ;������� ����� ��� PWMH2 � PWML2 ����� 90 ����.
    .equiv	SPHASE2_INIT,0x0000 ;������� SPHASE2 �� ������������
    .equiv	DTR2_INIT,DEAD_TIME ;
    .equiv	ALTDTR2_INIT,DEAD_TIME	;
    .equiv	TRGCON2_INIT,0x0000 ;�������2 �� ������������
    .equiv	IOCON2_INIT, 0x0b00 ;����� PWM1H ����������� CPU (DS80000656B �.7)
				    ;����� PWM1L ����������� CPU (DS80000656B �.7) 
				    ;����������� ��. �������� ������ ��� � ����� MCPIDReg.asm)
				    ;�������� ������� �� PWM1H - ������
				    ;�������� ������� �� PWM1L - ������
				    ;��� � ������ Push-Pull
				    ;���������� PWM1H � PWM1L ��������, ����������� ���� (DS80000656B �.7)
				    ;������ �� ��������������
    .equiv	TRIG2_INIT,0x0000   ;�������1 �� ������������
    .equiv	STRIG2_INIT,0x0000  ;��������� �������1 �� ������������   
    .equiv	FCLCON2_INIT,0x0003 ;����� ������ ���������
    .equiv	LEBCON2_INIT,0x0000 ;���������� ������ �� ������������
    .equiv	LEBDLY2_INIT, 0x0000;������ �� ������������
    .equiv	AUXCON2_INIT, 0x0000;������� ���������� ���������� ���������
				    ;������� ���������� ������� ���������
				    ;������������� ���
				    ;��������� ��������� �������� ������� ���
				    ;���������� ���������� ������ PWM1H ���
				    ;���������� ���������� ������ PWM1L ���
;���������� ��� ���
    .equiv	ADC_IPR_MASK,0x88ff ;
    .equiv	ADC_IPR, 0x6600	    ;��������� ���������� �� ���� AN0 � AN1 ����� 6
    .equiv	ADCON1L_INIT,0x0000 ;��� ������� � ������ IDLE
    .equiv	ADCON1H_INIT,0x0060 ;������ ������ - �����
				    ;����� ������ ������ ������ ��� - 12 ���
    .equiv	ADCON2L_INIT,0x0000 ;���������� �� ������� ����������� �������� ��������� ���������
				    ;���������� �� ������ ����������� �������� ��������� ���������
				    ;������ ���������� ���������
				    ;����� ������ ��� �� ������������
    .equiv	ADCON2H_INIT,0x0000 ;
    .equiv	ADCON3L_INIT,0x0000 ;������� �������� AVDD - AVSS
				    ;��� ������ ��� ����� ���� �������� ����������
				    ;����������� �������������� ��������������� �� �����������
    .equiv	ADCON3H_INIT,0x4000 ;������������ � �������� ����������� Fosc=120���
				    ;����� �������� ��� ���� ������� 1:1
				    ;��� ������ ��� ���������
    .equiv	ADCON4L_INIT,0x0004 ;����� �������� 2-� ���� ��� ���������� ���������
				    ;� ������� ����� ��������, ��������� �  ADCORE2L.SAMC<9:0>
    .equiv	ADCON4H_INIT,0x0000 ;������ 3 ��� ��������� � AN3 
				    ;������ 2 ��� ��������� � AN2
				    ;������ 1 ��� ��������� � AN1 
				    ;������ 0 ��� ��������� � AN0
    .equiv	ADCON5L_INIT,0x0000 ;��� ������ ��� ���������
    .equiv	ADCON5H_INIT,0x0000 ;�������� ��������� 16 ������ 
    .equiv	ADCORE0L_INIT,0x0000;
    .equiv	ADCORE0H_INIT,0x0300;������ ���������� �� �����������
				    ;���������� 12 ���
				    ;������� ������� ������������ ������� �� 2
    .equiv	ADCORE1L_INIT,0x0000;
    .equiv	ADCORE1H_INIT,0x0300;������ ���������� �� �����������
				    ;���������� 12 ���
				    ;������� ������� ������������ ������� �� 2
    .equiv	ADCORE2L_INIT,0x000a;���� 2 ���������� ��������� � ������� 10 �������� Tad
				    ;(��. ���������� ��� ADCON4L_INIT)
    .equiv	ADCORE2H_INIT,0x032f;������ ���������� �� �����������
				    ;���������� 12 ���
				    ;������� ������� ������������ ������� �� 94
    .equiv	ADLVLTRGL_INIT,0x0000;��� �������� ������������� � ��������
    .equiv	ADLVLTRGH_INIT,0x0000;
    .equiv	ADEIEL_INIT,0x0000  ;������ ���������� ��������� ��� ���� �������
    .equiv	ADEIEH_INIT,0x0000  ;
    .equiv	ADMOD0L_INIT,0x0000 ;��� ������ ��� ����� � �� ����������������
    .equiv	ADMOD0H_INIT,0x0000 ;
    .equiv	ADIEL_INIT,0x0000   ;���������� ��� ���� ������� ���������.
    .equiv	ADIEH_INIT,0x0000   ;
    .equiv	ADTRIG0L_INIT,0x0404;������� ��� ������� AN0 � AN1 - ������� ������� PWM
    .equiv	ADTRIG0H_INIT,0x0001;������� ��� ������ AN2 - ����� ����������� �������   
				    ;��������� ������ ��� �� ������������
    .equiv	ADCMP0CON_INIT,0x0000;���������� 0 ��� �������� � �� ������������
    .equiv	ADCMP1CON_INIT,0x0000;���������� 1 ��� �������� � �� ������������
    .equiv	ADC_FLT0_IPR_MASK,0x8fff;
    .equiv	ADC_FLT0_IPR, 0x2000;��������� ���������� ��������� ������� 0 ����� 2
    .equiv	ADFL0CON_INIT,0x7c02;�������� ������ 0  ��������
				    ;����� ����������
				    ;256 ��������� �� ���� ����������
				    ;����� ���������� ��� �� ������� ���������
				    ;� ������� ��������� ����� 2
    .equiv	ADFL1CON_INIT,0x0000;�������� ������ 1  �������� � �� ������������     
;���������� ��� SPI1
    .equiv	SPI1_IPR_MASK,0xf8ff;
    .equiv	SPI1_IPR,0x0400	    ;��������� ���������� �� SPI1 ����� 4
    .equiv	SPI1STAT_INIT,0x0000;SPI �������� � IDLE
    .equiv	SPI1CON1_INIT,0x0276;���������� ������������
				    ;SDO ������������
				    ;8 ������ �����
				    ;������� ������ ������������ � ����� �����
				    ;�������� ������ ���������� ��� �������� �� ��������� ��������� � ����������
				    ;SS �� ������������
				    ;Idle - ������ �������, �������� ������� - �������
				    ;����� �������
				    ;������ ��������� 1:3 (������� ������ 5 ���)
				    ;������ ��������� 1:4
    .equiv	SPI1CON2_INIT,0x0000;������ �� ��������������
				    ;����������� ����� ��������
;���������� ��� SPI2
    .equiv	SPI2_IPR_MASK,0xff8f;
    .equiv	SPI2_IPR,0x0040	    ;��������� ���������� �� SPI2 ����� 4
    .equiv	SPI2STAT_INIT,0x0014;SPI �������� � IDLE
				    ;���������� ��������� �� ��������� ��������
    .equiv	SPI2CON1_INIT,0x0675;���������� ������������
				    ;SDO ������������
				    ;16 ������ �����
				    ;������� ������ ������������ � ����� �����
				    ;�������� ������ ���������� ��� �������� �� ��������� ��������� � ����������
				    ;SS �� ������������
				    ;Idle - ������ �������, �������� ������� - �������
				    ;����� �������
				    ;������ ��������� 1:3 (������� ������ 1 ���)
				    ;������ ��������� 1:16
    .equiv	SPI2CON2_INIT,0x0001;������ �� ��������������
				    ;����������� ����� ��������
;���������� ��� OC1
;������� OC1CON1
;15...14 - �� ������������: 00
;13 -> OCSIDL - OC3 �������� � IDLE: 0
;12...10 -> OCTSEL - ����������� �� ��������� �������: 111
;9 -> ENFLT2 - ������ �� ����������� 2 ���������: 0
;8 -> ENFLT1 - ������ �� ����������� 1 ���������: 0
;7 -> ENFLT0 - ������ �� ����������� 0 ���������: 0
;6 -> OCFLT2 - ������ ������ �� ����������� 2: 0
;5 -> OCFLT1 - ������ ������ �� ����������� 1: 0
;4 -> OCFLT0 - ������ ������ �� ����������� 0: 0
;3 -> TRIGMODE - TRIGSTAT ��������� ������ ����������: 0
;2...0 -> OCM - ��������� ��� �� ������ OC3: 110
;OC1CON1=0b 0001 1100 0000 0110=0x1c06
    .equiv  OC1CON1_INIT, 0x1c06
;������� OC1CON2
;15 -> FLTMD - ��������� ������ ����������� �� ������ ���������� �����:0
;14 -> FLTOUT - ����� ������������ � 0 ��� ������: 0
;13 -> FLTTRIEN - ��������� ������ �� ������ �� ������: 0
;12 -> OCINV - ����� �� �������������: 0
;11 - �� ������������: 0
;10...9 -> DCB - ������� � ������ � ������ ����� �������: 00
;8 -> OC32 - �������������� �� ������������: 0
;7 -> OCTRIG - ������� ���������� ������ SYNCSEL: 0
;6 -> TRIGSTAT - ������ ��������� ��������: 0
;5 -> OCTRIS - ����� OC3 ��������� � ���������������� ������: 0
;4...0 -> SYNCSEL - ������������� � ������� �������� ���: 11111
;OC1CON2=0b 0000 0000 0001 1111=0x001f
    .equiv  OC1CON2_INIT, 0x001f
;������� OC1R - ��������� ������ �������� ��� = 0
    .equiv  OC1R_INIT, 0x0000
;������� OC1RS - ������� ��� ������ 60 000 000 / 2000 = 30 000 ���
    .equiv  OC1RS_INIT, 0x07d0
;���������� ��� IC1
    .equiv  IC1_IPR_MASK, 0xff8f
    .equiv  IC1_IPR, 0x0020
;������� IC1CON1
;15...14 - �� ������������: 00
;13 -> ICSIDL - IC1 �������� � IDLE: 0
;12...10 -> ICTSEL<2:0> - IC1 ����������� �� Fp=TCy: 111
;9...7 - �� ������������: 000
;6...5 -> ICI<1:0> - ���������� �� ������ �������: 00
;4 -> ICOV - ��� ������������ ��������� ������ (������ ��� ������): 0
;3 -> ICBNE - ��� ����������� ��������� ������ (������ ��� ������): 0
;2...0 -> ICM<2:0> - ����� ���������� �� ������� ������: 011
;IC1CON1 = 0b 0001 1100 0000 0011 = 0x1c03
    .equiv  IC1CON1_INIT, 0x1c03
;������� IC1CON2
;15...9 - �� ������������: 0000000
;8 -> IC32 - �������������� �� ������������: 0
;7 -> ICTRIG - ����������� ������������� � ������� TRIGSTAT: 1
;6 -> TRIGSTAT - ������ IC1 ������� � ������������ � ����: 0
;5 - �� ������������: 0
;4...0 -> SYNCSEL<4:0> �������� ������������� ����������� (���������� �������� �����������): 00000
;IC1CON2 = 0b 0000 0000 1000 0000 = 0x0800
    .equiv  IC1CON2_INIT, 0x0800
;���������� ��� ������� 2
    .equiv  T2_IPR_MASK, 0x8fff;
    .equiv  T2_IPR, 0x2000
    .equiv  T2CON_INIT,0x0010		    ;������ � ������ IDLE
					    ;��������� 1:8
					    ;������ 16 ���
					    ;�������� ������������ - ����������
    .equiv  PR2_INIT,0x02ee		    ;���� ��� ������� 2 - 0,1 ��
;��������� ��������� EEPROM �� SPI1
    .equiv  EE_STAT_RD,0x0000               ;
    .equiv  IDLE_EE_RD,0x0001               ;
    .equiv  EE_RD_ADM,0x0002                ;
    .equiv  EE_RD_ADL, 0x0003               ;
    .equiv  EE_RD_DATA, 0x0004              ;
    .equiv  EE_MOVE_BYTE, 0x0005            ;
    .equiv  EE_STAT_WR,0x0006               ;
    .equiv  IDLE_EE_WR,0x0007               ;
    .equiv  EE_WR_EN, 0x0008                ;
    .equiv  EE_WR_ADM,0x0009                ;
    .equiv  EE_WR_ADL,0x000a                ;
    .equiv  EE_WR_DATA, 0x000b              ;
    .equiv  EE_POST_WR,0x000c               ;
    .equiv  EE_WR_DIS,0x000d                ;
;���� �������� ��������� EEPROM
    .equiv  WIP, 0x00                       ;
    .equiv  WEL, 0x01                       
;��������� ��������� ������ � DM13
    .equiv  DM13Idle, 0x00		    ;
    .equiv  DM13SetEn, 0x01		    ;
    .equiv  DM13Shift, 0x02		    ;
    .equiv  DM13SetLatch, 0x03		    ;
    .equiv  DM13ResLatch, 0x04		    ;
    .equiv  DM13ResEn, 0x05		    ;
;�����
    .equiv  TMP1, 0x0			    ;
    .equiv  TMP2, 0x1			    ;
    .equiv  TMP3, 0x2			    ;
    .equiv  TMP4, 0x3			    ;
    .equiv  HPG, 0x4			    ;
    .equiv  SPB, 0x5			    ;
    .equiv  MemB, 0x6			    ;
    .equiv  GasB, 0x7			    ;
    .equiv  SelB, 0x8			    ;
    .equiv  T24B, 0x9			    ;
    .equiv  MIGBut, 0xa			    ;
;������� ������   
    .equiv  Thermo1M, 0x00		    ;
    .equiv  Thermo2M, 0x01		    ;
    .equiv  Thermo3M, 0x02		    ;
    .equiv  Thermo4M, 0x03		    ;
    .equiv  HPGM, 0x04			    ;
    .equiv  StartBit, 0x01		    ;
;������
    .equiv  SPBtn, 0x0			    ;
    .equiv  MemBtn, 0x1			    ;
    .equiv  GasBtn, 0x2			    ;
    .equiv  SelBtn, 0x3			    ;
    .equiv  T24Btn, 0x4			    ;
    .equiv  MIGBtn, 0x5			    ;
;���������� ��� EEPROM
    .macro  SetCSEEPROM
    bset    LATC, #RC9
    .endm
    .macro  ClrCSEEPROM
    bclr    LATC, #RC9
    .endm
;���������� ��� �������
    .macro  SetDM13En
    bset    LATC, #RC4
    .endm
    .macro  ResDM13En
    bclr    LATC, #RC4
    .endm
    .macro  SetDM13Latch
    bset    LATC, #RC8
    .endm
    .macro  ResDM13Latch
    bclr    LATC, #RC8
    .endm
;��������� ��� ���������
    .equiv  InputsNum, 13	    ;
    .equiv  MAX_INP_CNT, 5          ;
    .equiv  MAX_IC1_INT_CNT, 20	    ;
;�����
    .equiv  PIDModeFl,0x0	    ;
    .equiv  ADCP0IntFl,0x1          ;
    .equiv  ModeFl,0x2		    ;
    .equiv  DigitModifyFl,0x3	    ;
    .equiv  ScrModifyFl, 0x4	    ;
    .equiv  EEBusyFl,0x5            ;
    .equiv  ADCP1IntFl,0x6          ;
    .equiv  PIDCalc,0x7             ;
    .equiv  GasSupplyFl, 0x8	    ;
    .equiv  ArcContrFl,0x9          ;
    .equiv  T10msFl,0xa             ;
    .equiv  BlinkFl,0xb		    ;
    .equiv  T2Tic,0xc               ;
    .equiv  T0n1msFl,0xd	    ;
    .equiv  IC1IntFl, 0xe	    ;
    .equiv  ShtCktFl, 0xf	    ;
;��������� ��� �����������
;��������� ����������
    .equiv  U_PID_PROP_GAIN, 0x0030 ; 0x0800 0x0700
    .equiv  U_PID_INT_GAIN, 0x0005  ; 0x0100 0x00c0
    .equiv  U_PID_DERIV_GAIN, 0x0080;
    ;��������� ����
    .equiv  I_PID_PROP_GAIN, 1500    ;0x0800 0x0600 900 1500
    .equiv  I_PID_INT_GAIN, 70	    ;0x0100 0x00a0 60 70
    .equiv  I_PID_DERIV_GAIN, 160   ;0x0080 160
;�������������� ������
    .section	DATA_SEC,bss,near
USet:
    .space  2
ISet:
    .space  2
Ufb:
    .space  2
Ifb:
    .space  2
BlinkCnt:
    .space  2
CntHPG:
    .space  1    
CntTMP1:
    .space  1
CntTMP2:
    .space  1
CntTMP3:
    .space  1
CntTMP4:
    .space  1
CntSPB:
    .space  1
CntMEMB:
    .space  1
CntGASB:
    .space  1
CntSELB:
    .space  1
CntT24B:
    .space  1
CntMIGB:
    .space  1
ModeReg:
    .space  1
TMR10ms:
    .space  1
DM13State:
    .space  1
IC1IntCnt:
    .space  2
UCtrl:		
    .space  2
ICtrl:		
    .space  2
CntEEBuf:   
    .space  2   ;������� ���������� (���������) ����
EE_State:   
    .space  2   ;������� ��������� EEPROM
EEPROM_Ad:  
    .space  2   ;����� � EEPROM
EEBufPtr:   
    .space  2   ;��������� �� ����� ������ � ����������������
EELength:   
    .space  2   ;����� ��� ������ � ������
Param:
    .space  2
EEPROMData:
    .space  2
UADSum:
    .space  2
RADSum:
    .space  2
    .section PidVars_Xmem, bss, near, xmemory
;������������ ��� ��� �����������
UK_A:	
    .space  2
UK_B:
    .space  2
UK_C:
    .space  2
IK_A:
    .space  2
IK_B:
    .space  2
IK_C:
    .space  2

    .section PidVars_Ymem, bss, near, ymemory
;������ ��� ��� �����������
UError_n:
    .space  2
UError_n_1:
    .space  2
UError_n_2:
    .space  2
IError_n:
    .space  2
IError_n_1:
    .space  2
IError_n_2:
    .space  2
