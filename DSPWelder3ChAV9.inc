    .equ	__33EP32GS504, 1
    .include	"p33EP32GS504.inc"
;конфигурация
;загрузочный сегмент отсутствует и в него можно записывать, основной сегмент не защищен и в него можно писать, 
;альтернативные прерывания запрещены
    .section	__FSEC.sec, code
    .global	__FSEC
__FSEC:		.pword	BWRP_OFF & BSS_DISABLED & BSEN_OFF & GWRP_OFF & GSS_DISABLED & CWRP_OFF & CSS_DISABLED & AIVTDIS_OFF
;адрес загрузочного сегмента
    .section	__FBSLIM.sec, code
    .global	__FBSLIM
__FBSLIM:	.pword	BSLIM_BSLIM
;старт без автоматического переключния на внутреннем генераторе без PLL
    .section	__FOSCSEL.sec, code
    .global	__FOSCSEL
__FOSCSEL:	.pword	FNOSC_FRC & IESO_OFF
;переключение разршено, монитор запрещен,
;разрешена многократная презапись, выход OSC общего назначения
;первичный генератор запрещен
    .section	__FOSC.sec, code
    .global	__FOSC
__FOSC:		
    .pword	POSCMD_EC & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSECMD
;WDT запрещен
    .section	__FWDT.sec, code
    .global	__FWDT
__FWDT:		
    .pword	WDTEN_OFF & WINDIS_OFF
;JTAG запрещен, используются PGEC1 и PGED1 для ICD
    .section	__FICD.sec, code
    .global	__FICD
__FICD:		
    .pword	JTAGEN_OFF & ICS_PGD1
;блокирующая последовательность для модуля ШИМ не используется, все устройства на своих местах
    .section	__FDEVOPT.sec, code
    .global	__FDEVOPT
__FDEVOPT:		
    .pword	PWMLOCK_OFF & ALTI2C1_OFF & ALTI2C2_OFF & DBCC_OFF
;основные расчетные частоты и периоды
    .equiv	FAUXPLL, 117920000  ;
    .equiv	FPWM,66555          ;поскольку частота 33 кГц и каждое колебание в свой период
    .equiv	PWM_PERIOD, (FAUXPLL*8 / FPWM);14143
    .equiv	DUTY_MIN,500        ;(1600 - 200 нс)
    .equiv	DUTY_MAX,PWM_PERIOD-8
    .equiv	OVERRIDE_DELAY,1500 ;задержка более, чем на 1 период ШИМ (DS80439L п.32)
    .equiv	DEAD_TIME,(PWM_PERIOD*9/100);(PWM_PERIOD*9/100)мертвое время 9% от периода
    .equiv	MEAS_TIME,(PWM_PERIOD*55/100);измерения в точке 55% от периода
;назначния для контроллера ШИМ
    .equiv	PTCON_INIT, 0x0001  ;ШИМ работат в IDLE,
				    ;внешней синхронизации нет
				    ;обновление синхронное
				    ;триггер событий срабатывает по каждому второму совпадению
    .equiv	PTCON2_INIT,0x0000  ;делитель ШИМ -> 1:1	
    .equiv	PTPER_INIT,PWM_PERIOD	;	
    .equiv	SEVTCMP_INIT,MEAS_TIME	;время срабатывания основного триггера
    .equiv	STCON_INIT, 0x0000  ;второй канал управления модулем ШИМ не используется
    .equiv	STCON2_INIT, 0x0000 ;
    .equiv	STPER_INIT, 0x0000  ;
    .equiv	SSEVTCMP_INIT, 0x0000	;
    .equiv	CHOP_INIT, 0x0000   ;заполнение импульсами не используется
    .equiv	MDC_INIT,DUTY_MIN   ;в регистре MDC минимальная длительность
;параметрирование канала 1-го канала ШИМ
    .equiv	PWMCON1_INIT,0x0101 ;период определяется ргистром PTPER
				    ;прерывание от главного триггера запрещено
				    ;длительность импульса в регистре MDC
				    ;позитивное мертвое время
				    ;PWMH1 укороченный, PWML1 удлиненный
				    ;используется главный задатчик периода
				    ;центральное выравнивание не используется
				    ;внешний вывод синхронизации не используется
				    ;обновление длительности синхронно с периодом			
    .equiv	PDC1_INIT,0x0000    ;длительность импульса в MDC, PDC1 не используется
    .equiv	SDC1_INIT,0x0000    ;второй ргистр длитльности не использутся
    .equiv	PHASE1_INIT,0x0000  ;фазовый сдвиг для PWMH1 и PWML1 равен 0
    .equiv	SPHASE1_INIT,0x0000 ;регистр SPHASE1 не используется
    .equiv	DTR1_INIT,DEAD_TIME ;
    .equiv	ALTDTR1_INIT,DEAD_TIME	;
    .equiv	TRGCON1_INIT,0x0000 ;триггер1 не используется
    .equiv	IOCON1_INIT, 0x0b00 ;вывод PWM1H управляется CPU (DS80000656B п.7)
				    ;вывод PWM1L управляется CPU (DS80000656B п.7) 
				    ;продолжение см. включние модуля ШИМ в файле MCPIDReg.asm)
				    ;активный уровень на PWM1H - выокий
				    ;активный уровень на PWM1L - выокий
				    ;ШИМ в режиме Push-Pull
				    ;перезапись PWM1H и PWM1L разрешна, записывются нули (DS80000656B п.7)
				    ;выводы не переставляются
    .equiv	TRIG1_INIT,0x0000   ;триггер1 не используется
    .equiv	STRIG1_INIT,0x0000  ;вторичный триггер1 не используется   
    .equiv	FCLCON1_INIT,0x0003 ;входы ошибок запрещены
    .equiv	LEBCON1_INIT,0x0000 ;подавление фронта не используется
    .equiv	LEBDLY1_INIT, 0x0000;ошибки не бланкируются
    .equiv	AUXCON1_INIT, 0x0000;высокое разрешение заполнения разрешено
				    ;высокое разрешение периода разрешено
				    ;бланкирования нет
				    ;генератор заполняет активный импульс ШИМ
				    ;заполнения импульсами канала PWM1H нет
				    ;заполнения импульсами канала PWM1L нет
;параметрирование канала 2-го канала ШИМ
    .equiv	PWMCON2_INIT,0x0100 ;период определяется ргистром PTPER
				    ;прерывание от главного триггера запрещено
				    ;длительность импульса в регистре MDC
				    ;позитивное мертвое время
				    ;PWMH2 укороченный, PWML2 удлиненный
				    ;используется главный задатчик периода
				    ;центральное выравнивание не используется
				    ;внешний вывод синхронизации не используется
				    ;обновление длительности синхронно с периодом			
    .equiv	PDC2_INIT,0x0000    ;длительность импульса в MDC, PDC2 не используется
    .equiv	SDC2_INIT,0x0000    ;второй ргистр длитльности не использутся
    .equiv	PHASE2_INIT,PWM_PERIOD/2  ;фазовый сдвиг для PWMH2 и PWML2 равен 90 град.
    .equiv	SPHASE2_INIT,0x0000 ;регистр SPHASE2 не используется
    .equiv	DTR2_INIT,DEAD_TIME ;
    .equiv	ALTDTR2_INIT,DEAD_TIME	;
    .equiv	TRGCON2_INIT,0x0000 ;триггер2 не используется
    .equiv	IOCON2_INIT, 0x0b00 ;вывод PWM1H управляется CPU (DS80000656B п.7)
				    ;вывод PWM1L управляется CPU (DS80000656B п.7) 
				    ;продолжение см. включние модуля ШИМ в файле MCPIDReg.asm)
				    ;активный уровень на PWM1H - выокий
				    ;активный уровень на PWM1L - выокий
				    ;ШИМ в режиме Push-Pull
				    ;перезапись PWM1H и PWM1L разрешна, записывются нули (DS80000656B п.7)
				    ;выводы не переставляются
    .equiv	TRIG2_INIT,0x0000   ;триггер1 не используется
    .equiv	STRIG2_INIT,0x0000  ;вторичный триггер1 не используется   
    .equiv	FCLCON2_INIT,0x0003 ;входы ошибок запрещены
    .equiv	LEBCON2_INIT,0x0000 ;подавление фронта не используется
    .equiv	LEBDLY2_INIT, 0x0000;ошибки не бланкируются
    .equiv	AUXCON2_INIT, 0x0000;высокое разрешение заполнения разрешено
				    ;высокое разрешение периода разрешено
				    ;бланкирования нет
				    ;генератор заполняет активный импульс ШИМ
				    ;заполнения импульсами канала PWM1H нет
				    ;заполнения импульсами канала PWM1L нет
;назначения для АЦП
    .equiv	ADC_IPR_MASK,0x88ff ;
    .equiv	ADC_IPR, 0x6600	    ;приоритет прерываний от пары AN0 и AN1 равен 6
    .equiv	ADCON1L_INIT,0x0000 ;АЦП работат в режиме IDLE
    .equiv	ADCON1H_INIT,0x0060 ;формат данных - целый
				    ;длина данных общего модуля АЦП - 12 бит
    .equiv	ADCON2L_INIT,0x0000 ;прерывания от событий внутреннего опорного источника запрещены
				    ;прерывания от ошибок внутреннего опорного источника запрещены
				    ;ранние прерывания запрещены
				    ;общий модуль АЦП не используется
    .equiv	ADCON2H_INIT,0x0000 ;
    .equiv	ADCON3L_INIT,0x0000 ;опорный источник AVDD - AVSS
				    ;все модули АЦП могут быть запущены триггерами
				    ;программное индивидуальное конвертирование не применяется
    .equiv	ADCON3H_INIT,0x4000 ;тактирование с частотой осциллятора Fosc=120МГц
				    ;общий делитель для всех модулей 1:1
				    ;все модули АЦП выключены
    .equiv	ADCON4L_INIT,0x0004 ;после триггера 2-е ядро АЦП продолжент измерение
				    ;в течение числа отсчетов, указанных в  ADCORE2L.SAMC<9:0>
    .equiv	ADCON4H_INIT,0x0000 ;модуль 3 АЦП подключен к AN3 
				    ;модуль 2 АЦП подключен к AN2
				    ;модуль 1 АЦП подключен к AN1 
				    ;модуль 0 АЦП подключен к AN0
    .equiv	ADCON5L_INIT,0x0000 ;все модули АЦП выключены
    .equiv	ADCON5H_INIT,0x0000 ;задержка включения 16 тактов 
    .equiv	ADCORE0L_INIT,0x0000;
    .equiv	ADCORE0H_INIT,0x0300;ранние прерывания не испльзуются
				    ;разрешение 12 бит
				    ;входная частота тактирования делится на 2
    .equiv	ADCORE1L_INIT,0x0000;
    .equiv	ADCORE1H_INIT,0x0300;ранние прерывания не испльзуются
				    ;разрешение 12 бит
				    ;входная частота тактирования делится на 2
    .equiv	ADCORE2L_INIT,0x000a;ядро 2 продолжает измерения в течение 10 отсчетов Tad
				    ;(см. назначение для ADCON4L_INIT)
    .equiv	ADCORE2H_INIT,0x032f;ранние прерывания не испльзуются
				    ;разрешение 12 бит
				    ;входная частота тактирования делится на 94
    .equiv	ADLVLTRGL_INIT,0x0000;все триггеры чувствительны к переходу
    .equiv	ADLVLTRGH_INIT,0x0000;
    .equiv	ADEIEL_INIT,0x0000  ;ранние прерывания запрещены для всех каналов
    .equiv	ADEIEH_INIT,0x0000  ;
    .equiv	ADMOD0L_INIT,0x0000 ;все каналы без знака и не дифференциальные
    .equiv	ADMOD0H_INIT,0x0000 ;
    .equiv	ADIEL_INIT,0x0000   ;прерывания для всех каналов запрещены.
    .equiv	ADIEH_INIT,0x0000   ;
    .equiv	ADTRIG0L_INIT,0x0404;триггер для каналов AN0 и AN1 - триггер событий PWM
    .equiv	ADTRIG0H_INIT,0x0001;триггер для канала AN2 - общий программный триггер   
				    ;остальные каналы АЦП не используются
    .equiv	ADCMP0CON_INIT,0x0000;компаратор 0 АЦП выключен и не используется
    .equiv	ADCMP1CON_INIT,0x0000;компаратор 1 АЦП выключен и не используется
    .equiv	ADC_FLT0_IPR_MASK,0x8fff;
    .equiv	ADC_FLT0_IPR, 0x2000;приоритет прерываний цифрового фильтра 0 равен 2
    .equiv	ADFL0CON_INIT,0x7c02;цифровой фильтр 0  разрешен
				    ;режим усреднения
				    ;256 измерений на одно усреднение
				    ;общее прерывание АЦП от фильтра разрешено
				    ;к фильтру подключен канал 2
    .equiv	ADFL1CON_INIT,0x0000;цифровой фильтр 1  выключен и не используется     
;назначения для SPI1
    .equiv	SPI1_IPR_MASK,0xf8ff;
    .equiv	SPI1_IPR,0x0400	    ;приоритет прерываний от SPI1 равен 4
    .equiv	SPI1STAT_INIT,0x0000;SPI работает в IDLE
    .equiv	SPI1CON1_INIT,0x0276;внутреннее тактирование
				    ;SDO используется
				    ;8 битный режим
				    ;входные данные опрашиваются в конце такта
				    ;выходные данные изменяются при переходе из активного состояния к пассивному
				    ;SS не используется
				    ;Idle - низкий уровень, активный уровень - высокий
				    ;режим мастера
				    ;второй прескалер 1:3 (частота работы 5 МГц)
				    ;первый прескалер 1:4
    .equiv	SPI1CON2_INIT,0x0000;фреймы не поддерживаются
				    ;расширенный буфер запрещен
;назначения для SPI2
    .equiv	SPI2_IPR_MASK,0xff8f;
    .equiv	SPI2_IPR,0x0040	    ;приоритет прерываний от SPI2 равен 4
    .equiv	SPI2STAT_INIT,0x0014;SPI работает в IDLE
				    ;прерывание наступает по окончанию передачи
    .equiv	SPI2CON1_INIT,0x0675;внутреннее тактирование
				    ;SDO используется
				    ;16 битный режим
				    ;входные данные опрашиваются в конце такта
				    ;выходные данные изменяются при переходе из активного состояния к пассивному
				    ;SS не используется
				    ;Idle - низкий уровень, активный уровень - высокий
				    ;режим мастера
				    ;второй прескалер 1:3 (частота работы 1 МГц)
				    ;первый прескалер 1:16
    .equiv	SPI2CON2_INIT,0x0001;фреймы не поддерживаются
				    ;расширенный буфер разрешен
;назначения для OC1
;регистр OC1CON1
;15...14 - не используются: 00
;13 -> OCSIDL - OC3 работает в IDLE: 0
;12...10 -> OCTSEL - такирование от системной частоты: 111
;9 -> ENFLT2 - ошибка от компаратора 2 запрещена: 0
;8 -> ENFLT1 - ошибка от компаратора 1 запрещена: 0
;7 -> ENFLT0 - ошибка от компаратора 0 запрещена: 0
;6 -> OCFLT2 - сигнал ошибки от компаратора 2: 0
;5 -> OCFLT1 - сигнал ошибки от компаратора 1: 0
;4 -> OCFLT0 - сигнал ошибки от компаратора 0: 0
;3 -> TRIGMODE - TRIGSTAT очищается только программно: 0
;2...0 -> OCM - граничная ШИМ на выводе OC3: 110
;OC1CON1=0b 0001 1100 0000 0110=0x1c06
    .equiv  OC1CON1_INIT, 0x1c06
;регистр OC1CON2
;15 -> FLTMD - состояние ошибки сохраняется до начала следующего цикла:0
;14 -> FLTOUT - выход сбрасывается в 0 при ошибке: 0
;13 -> FLTTRIEN - состояние вывода не влияет на ошибку: 0
;12 -> OCINV - выход не инвертируется: 0
;11 - не используется: 0
;10...9 -> DCB - переход к ошибке в начале цикла команды: 00
;8 -> OC32 - каскадирование не используется: 0
;7 -> OCTRIG - триггер выбирается битами SYNCSEL: 0
;6 -> TRIGSTAT - сигнал состояния триггера: 0
;5 -> OCTRIS - выход OC3 подключен к соответствующему выводу: 0
;4...0 -> SYNCSEL - синхронизации с другими модулями нет: 11111
;OC1CON2=0b 0000 0000 0001 1111=0x001f
    .equiv  OC1CON2_INIT, 0x001f
;регистр OC1R - начальная ширина импульса ШИМ = 0
    .equiv  OC1R_INIT, 0x0000
;регистр OC1RS - частота ШИМ мотора 60 000 000 / 2000 = 30 000 кГц
    .equiv  OC1RS_INIT, 0x07d0
;назначения для IC1
    .equiv  IC1_IPR_MASK, 0xff8f
    .equiv  IC1_IPR, 0x0020
;регистр IC1CON1
;15...14 - не используются: 00
;13 -> ICSIDL - IC1 работает в IDLE: 0
;12...10 -> ICTSEL<2:0> - IC1 тактируется от Fp=TCy: 111
;9...7 - не используются: 000
;6...5 -> ICI<1:0> - прерывание на каждое событие: 00
;4 -> ICOV - бит переполнения выходного буфера (только для чтения): 0
;3 -> ICBNE - бит опустошения выходного буфера (только для чтения): 0
;2...0 -> ICM<2:0> - режим накопления по каждому фронту: 011
;IC1CON1 = 0b 0001 1100 0000 0011 = 0x1c03
    .equiv  IC1CON1_INIT, 0x1c03
;регистр IC1CON2
;15...9 - не используются: 0000000
;8 -> IC32 - каскадирование не используется: 0
;7 -> ICTRIG - программная синхронизация с помощью TRIGSTAT: 1
;6 -> TRIGSTAT - таймер IC1 сброшен и удерживается в нуле: 0
;5 - не используется: 0
;4...0 -> SYNCSEL<4:0> источник синхронизации отсутствует (управление таймером программное): 00000
;IC1CON2 = 0b 0000 0000 1000 0000 = 0x0800
    .equiv  IC1CON2_INIT, 0x0800
;назначения для таймера 2
    .equiv  T2_IPR_MASK, 0x8fff;
    .equiv  T2_IPR, 0x2000
    .equiv  T2CON_INIT,0x0010		    ;работа в режиме IDLE
					    ;прескалер 1:8
					    ;таймер 16 бит
					    ;источник тактирования - внутренний
    .equiv  PR2_INIT,0x02ee		    ;один тик таймера 2 - 0,1 мс
;состояния алгоритма EEPROM на SPI1
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
;биты регистра состояния EEPROM
    .equiv  WIP, 0x00                       ;
    .equiv  WEL, 0x01                       
;состояния алгоритма записи в DM13
    .equiv  DM13Idle, 0x00		    ;
    .equiv  DM13SetEn, 0x01		    ;
    .equiv  DM13Shift, 0x02		    ;
    .equiv  DM13SetLatch, 0x03		    ;
    .equiv  DM13ResLatch, 0x04		    ;
    .equiv  DM13ResEn, 0x05		    ;
;входы
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
;регистр аварий   
    .equiv  Thermo1M, 0x00		    ;
    .equiv  Thermo2M, 0x01		    ;
    .equiv  Thermo3M, 0x02		    ;
    .equiv  Thermo4M, 0x03		    ;
    .equiv  HPGM, 0x04			    ;
    .equiv  StartBit, 0x01		    ;
;кнопки
    .equiv  SPBtn, 0x0			    ;
    .equiv  MemBtn, 0x1			    ;
    .equiv  GasBtn, 0x2			    ;
    .equiv  SelBtn, 0x3			    ;
    .equiv  T24Btn, 0x4			    ;
    .equiv  MIGBtn, 0x5			    ;
;назначения для EEPROM
    .macro  SetCSEEPROM
    bset    LATC, #RC9
    .endm
    .macro  ClrCSEEPROM
    bclr    LATC, #RC9
    .endm
;назначения для дисплея
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
;назначния для программы
    .equiv  InputsNum, 13	    ;
    .equiv  MAX_INP_CNT, 5          ;
    .equiv  MAX_IC1_INT_CNT, 20	    ;
;флаги
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
;константы ПИД регуляторов
;регулятор напряжения
    .equiv  U_PID_PROP_GAIN, 0x0030 ; 0x0800 0x0700
    .equiv  U_PID_INT_GAIN, 0x0005  ; 0x0100 0x00c0
    .equiv  U_PID_DERIV_GAIN, 0x0080;
    ;регулятор тока
    .equiv  I_PID_PROP_GAIN, 1500    ;0x0800 0x0600 900 1500
    .equiv  I_PID_INT_GAIN, 70	    ;0x0100 0x00a0 60 70
    .equiv  I_PID_DERIV_GAIN, 160   ;0x0080 160
;резервирование данных
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
    .space  2   ;счетчик записанных (считанных) байт
EE_State:   
    .space  2   ;регистр состояний EEPROM
EEPROM_Ad:  
    .space  2   ;адрес в EEPROM
EEBufPtr:   
    .space  2   ;указатель на буфер обмена в микроконтроллере
EELength:   
    .space  2   ;длина для обмена в байтах
Param:
    .space  2
EEPROMData:
    .space  2
UADSum:
    .space  2
RADSum:
    .space  2
    .section PidVars_Xmem, bss, near, xmemory
;Коэффициенты для ПИД регуляторов
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
;Ошибки для ПИД регуляторов
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
