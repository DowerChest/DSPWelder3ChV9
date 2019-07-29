typedef unsigned int UINT16;
typedef int INT16;
typedef long INT32;
typedef unsigned long UINT32;
typedef unsigned char UINT8;
typedef char INT8;
typedef union
{
    UINT16 Word;
    struct
    {
        unsigned Flag0 :1;
        unsigned Flag1 :1;
        unsigned Flag2 :1;
        unsigned Flag3 :1;
        unsigned Flag4 :1;
        unsigned Flag5 :1;
        unsigned Flag6 :1;
        unsigned Flag7 :1;
        unsigned Flag8 :1;
        unsigned Flag9 :1;
        unsigned Flag10 :1;
        unsigned Flag11 :1;
        unsigned Flag12 :1;
        unsigned Flag13 :1;
        unsigned Flag14 :1;
        unsigned Flag15 :1;
    }Bits;
}WORDBITS;
typedef enum
{
    MMA_PROC,
    MMA_WAIT_OFF,
    MMA_WAIT_ON,
    MMA_ON
} MMA_STATE;
typedef enum
{
    IDLE,
    WAIT_POWER_ON,
    WAIT_U,
    WAIT_FIRE,
    FIRE,      
    PROCESS_1,
    PROCESS_2,
    PROCESS_3,
    WAIT_INV,
    WAIT_0_4T
} MIG_STATE;
typedef struct
{
    INT16 Min;
    INT16 Max;
} LIMITS;
typedef union
{
    UINT8 AlmReg;
    struct
    {
        unsigned Thermo1 :1;
        unsigned Thermo2 :1;
        unsigned Thermo3 :1;
        unsigned Thermo4 :1;
        unsigned HVOK :1;
        unsigned StartBit:1;
        unsigned reserve :2;
    }AlmBits;
}ALM_REG;
typedef union
{
    UINT8 ButReg;
    struct
    {
        UINT8 SPBtn :1;
        UINT8 MemBtn :1;
        UINT8 GasBtn :1;
        UINT8 SelBtn :1;
        UINT8 T24Btn :1;
        UINT8 MIGBtn :1;
        UINT8 Reserve:2;  
    }ButBits;
}BUT_REG;
typedef struct
{
   UINT8 IntGainSyn;
   UINT8 IntGainPrm;
   UINT8 IBaseSyn;
   UINT16 UArcSyn;
}SYNERGY_PRM;
typedef union
{
    UINT16 MIGModReg;
    struct
    {
        UINT16 TorchButMd :1;    //0 - 2T, 1 - 4T
        UINT16 GasMd: 1;         //0 - CO2, 1 - Ar
        UINT16 WorkMd: 2;        //00 - PRM, 01 - SYN, 10 - PGM
        UINT16 DiamMd: 2;        //00 - 0.8, 01 - 1.0, 10 - 1.2, 11 - 1.6
        UINT16 ParSel: 3;        //000 - L, 001 - D, 010 - GB, 011 - GA, 100 - NP
        UINT16 reserve: 7;
        
    }MIGModBits;
}MIG_MOD;
#define LED4T 0x0001
#define LEDCO2 0x0002
#define LEDPGM 0x0004
#define LEDSYN 0x0008
#define LEDPAR 0x0010
#define LEDAR 0x0020
#define LED2T 0x0040
#define LEDL 0x0080
#define LEDD 0x0100
#define LEDGB 0x0200
#define LEDGA 0x0400
#define LEDNP 0x0800

#define A2_MASK 0xff00

#define A2_0 0x00ed
#define A2_1 0x0041
#define A2_2 0x00dc
#define A2_3 0x00d5
#define A2_4 0x0071
#define A2_5 0x00b5
#define A2_6 0x00bd
#define A2_7 0x00c1
#define A2_8 0x00fd
#define A2_9 0x00f5
#define A2_DP 0x0002

#define A3_MASK 0x00ff

#define A3_0 0xed00
#define A3_1 0x2800
#define A3_2 0xce00
#define A3_3 0xae00
#define A3_4 0x2b00
#define A3_5 0xa700
#define A3_6 0xe700
#define A3_7 0x2c00
#define A3_8 0xef00
#define A3_9 0xaf00

#define D0n8 0xefef
#define D1n0 0xed43
#define D1n2 0xce43
#define D1n6 0xe743

#define A1U_MASK 0xff00

#define A1U_0 0x00af
#define A1U_1 0x0028
#define A1U_2 0x009d
#define A1U_3 0x00bc
#define A1U_4 0x003a
#define A1U_5 0x00b6
#define A1U_6 0x00b7
#define A1U_7 0x002c
#define A1U_8 0x00bf
#define A1U_9 0x00be

#define A2U_MASK 0x00ff

#define A2U_0 0xef00
#define A2U_1 0x6800
#define A2U_2 0xdd00
#define A2U_3 0xfc00
#define A2U_4 0x7a00
#define A2U_5 0xf600
#define A2U_6 0xf700
#define A2U_7 0x6c00
#define A2U_8 0xff00
#define A2U_9 0xfe00

#define A3U_MASK 0xff00

#define A3U_0 0x00af
#define A3U_1 0x00a0
#define A3U_2 0x00cd
#define A3U_3 0x00ec
#define A3U_4 0x00e2
#define A3U_5 0x006e
#define A3U_6 0x006f
#define A3U_7 0x00a4
#define A3U_8 0x00ef
#define A3U_9 0x00ee

#define A1I_MASK 0x00ff

#define A1I_0 0xaf00
#define A1I_1 0x2800
#define A1I_2 0x9d00
#define A1I_3 0xb900
#define A1I_4 0x3a00
#define A1I_5 0xb300
#define A1I_6 0xb700
#define A1I_7 0x2900
#define A1I_8 0xbf00
#define A1I_9 0xbb00

#define A2I_MASK 0xff00

#define A2I_0 0x00af
#define A2I_1 0x0028
#define A2I_2 0x009d
#define A2I_3 0x00bc
#define A2I_4 0x003a
#define A2I_5 0x00b6
#define A2I_6 0x00b7
#define A2I_7 0x002c
#define A2I_8 0x00bf
#define A2I_9 0x00be
#define A2I_DP 0x0040

#define A3I_MASK 0x00ff

#define A3I_0 0xaf00
#define A3I_1 0x2800
#define A3I_2 0x9d00
#define A3I_3 0xbc00
#define A3I_4 0x3a00
#define A3I_5 0xb600
#define A3I_6 0xb700
#define A3I_7 0x2c00
#define A3I_8 0xbf00
#define A3I_9 0xbe00