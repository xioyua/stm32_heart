#ifndef  __ADS1298_H_
#define  __ADS1298_H_

/********************************************************
注释为一长串大写A的声明为第二块ads1298的函数
**********************************************************/
//头文件位置


//#include "include.h"

//#include "intrins.h"

//ADS1298相关操作函数声明
unsigned char init_ADS1x9x1 (void);
unsigned char init_ADS1x9x2 (void);
unsigned char init_ADS1x9x3 (void);


unsigned char ADS1x9x_SPI_Burst (unsigned char);   //发送一个字节数据，并读回一个字节数据


void init_ADS1x9x_IO (void);     //IO初始化，spi初始化


void POR_Reset_ADS1x9x (void);
void Hard_Start_ReStart_ADS1x9x (void);
void Soft_Start_ReStart_ADS1x9x (void);
void Power_Up_ADS1x9x (void);


void Stop_Read_Data_Continuous (void);
void Start_Read_Data_Continuous (void);


void enable_ADS1x9x_Conversion (void);  //
void disable_ADS1x9x_Conversion (void);

void set_ADS1x9x_Start_Pin (unsigned char state);


unsigned char Initialize_ADS1x9x_Registers (void);
unsigned char ADS1x9x_Read_Version (void);
unsigned char ADS1x9x_Read_Version_2 (void );        //AAAAAAAAAAAAAAAAAAAAAAAAA

void ADS1x9x_SPI_Address_Byte_Count (unsigned char, unsigned char);
void ADS1x9x_SPI_Address_Byte_Count_2 (unsigned char, unsigned char);      //AAAAAAAAAAAAAAAAAAAAA

unsigned char ADS1x9x_SPI_Data (unsigned char);
unsigned char ADS1x9x_SPI_Data_2 (unsigned char);
unsigned char ADS1x9x_SPI_Data_All ( unsigned char Data); // Complements the SPI_Address command


void init_ADS1x9x_Via_Constant_Table (unsigned char*);
void init_ADS1x9x_Via_Constant_Table_2 (unsigned char*);//AAAAAAAAAAAAAAAAAAAAAAAAAA
unsigned char verify_ADS1x9x_Registers (unsigned char*);
unsigned char verify_ADS1x9x_Registers_2 (unsigned char*);  //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
//unsigned char Enable_ADS1x9x_Test_Mode (unsigned char, unsigned char, unsigned char);


void Set_ADS1x9x_Chip_Enable (void);			//==CLR_ADS128_CS;
void Set_ADS1x9x_Chip_Enable_2 (void);	          //AAAAAAAAAAAAAAAAAAAAA
void Clear_ADS1x9x_Chip_Enable (void);          //==SET_ADS128_CS;
void Clear_ADS1x9x_Chip_Enable_2 (void);         //AAAAAAAAAAAAAAAAAAAAA

unsigned char Initialize_ADS1x9x_Data_Rate (unsigned char);
unsigned char Initialize_ADS1x9x_Data_Rate_2 (unsigned char); //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

unsigned char Initialize_ADS1x9x_Mode (unsigned char);
unsigned char Initialize_ADS1x9x_Mode_2 (unsigned char);    //AAAAAAAAAAAAAAAAAAAAAA

unsigned char Initialize_ADS1x9x_Channel (unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
unsigned char Initialize_ADS1x9x_Channel_2 (unsigned char, unsigned char, unsigned char, unsigned char, unsigned char); //AAAAAAAAAAAAAAAAAAAAAA

/**************************************************************************************************************************************************
*               Union Structures and Definitions                                                                                                  *
**************************************************************************************************************************************************/










#ifndef SPI_TEST_DATA
#define SPI_TEST_DATA   0x00                                        // Use 1010 1010 to SPI port for debug purposes
#endif
#define VERIFY                1
#define ECG_Num_Channels      8

#define ADS1x9x_DEFAULT_GPIO_STATE                      0x07


enum Context_Save_Channel_Info_Type
{
    CONTEXT_SAVE_CHANNEL                                = 1,
    IGNORE_PREVIOUS_STATE                               = 0
};

enum ADS_1x9x_Status_Type
{
    ADS_1x9x_VERIFY_ERROR                               = 3,
    ADS_1x9x_INIT_SUCCESS                               = 2,
    ADS_1x9x_NOT_FOUND                                  = 1
};

#define NEW_ADS1x9x_DATA                                  5

#define ADS1x9x_DATA_SIZE                                 NUMBER_ADC_CHANNELS * ADS1x9x_DATA_WIDTH

typedef struct
{
    unsigned char ADC_Data_Ready:                         1;
    unsigned char ADC_New_Data:                           1;
    unsigned char ADS_1298_Module_Status:                 2;
    unsigned char Reserved:                               4;
} ADS1x9x_Status_Flags_type;

#define ADS1x9x_PREAMBLE_GOOD                                   0x0C

typedef struct
{
    unsigned char ID_Reserved:                            5;           // Reserved
    unsigned char Revision_ID:                            3;           // Revision ID (REV_ID[3:1])
} ADS1x9x_ID_Register_type;

// REVISION_ID bit field definition
#define ADS1x9x_REV                                       0

typedef struct
{
    unsigned char Output_Data_Rate:                       3;           // Output Data Rate (DR[2:0])
    unsigned char Config_1_Reserved:                      2;           // Reserved
    unsigned char Oscillator_Clock_Output:                1;           // CLKOUT Connection (CLK_EN)
    unsigned char Readback_Mode:                          1;           // Daisy-chain/multiple readback mode (DAISY_EN')
    unsigned char Power_Resolution_Optimization:          1;           // Low-Power/High-Resolution mode (HR)
} ADS1x9x_Config_1_Register_type;


// OUTPUT_DATA_RATE bit field definition
enum Output_Data_Rate_Type
{
    DEFAULT_MODULATION_FREQUENCY_DIVIDED_BY_16          = 0,
    MODULATION_FREQUENCY_DIVIDED_BY_32                  = 1,
    MODULATION_FREQUENCY_DIVIDED_BY_64                  = 2,
    MODULATION_FREQUENCY_DIVIDED_BY_128                 = 3,
    MODULATION_FREQUENCY_DIVIDED_BY_256                 = 4,
    MODULATION_FREQUENCY_DIVIDED_BY_512                 = 5,
    MODULATION_FREQUENCY_DIVIDED_BY_1024                = 6
};

enum Readback_Mode_Type
{
    DEFAULT_DAISY_CHAIN_MODE                            = 0,
    MULTIPLE_READBACK_MODE                              = 1
};

enum Power_Resolution_Optimization_Type
{
    DEFAULT_LOW_POWER_MODE                              = 0,
    HIGH_RESOLUTION_MODE                                = 1
};

typedef struct
{
    unsigned char Test_Signal_Frequency:                  2;           // Test Signal Frequency (TEST_FREQ[1:0])
    unsigned char Test_Signal_Amplitude:                  2;           // Test Signal Amplitude (TEST_AMP[1:0])
    unsigned char Test_Source:                            1;           // Test Source
    unsigned char Config_2_Reserved:                      3;           // Reserved - Needs to be written with 1's
} ADS1x9x_Config_2_Register_type;

// CONFIG_2_RESERVED bit field definition
#define CONFIG_2_RESERVED_VALUE                           7

enum Test_Source_Type
{
    DEFAULT_TEST_SIGNALS_ARE_DRIVEN_EXTERNALLY          = 0,
    TEST_SIGNALS_ARE_DRIVEN_INTERNALLY                  = 1
};

enum Test_Signal_Amplitude_Type
{
    DEFAULT_PLUS_MINUS_1_MV_TIMES_VREF_DIVIDED_BY_2_4   = 0,
    PLUS_MINUS_2_MV_TIMES_VREF_DIVIDED_BY_2_4           = 1,
    PLUS_MINUS_10_MV_TIMES_VREF_DIVIDED_BY_2_4          = 2,
    PLUS_MINUS_1V_TIMES_VREF_DIVIDED_BY_2_4             = 3
};

enum Test_Signal_Frequency_Type
{
    DEFAULT_PULSED_AT_CLOCK_FREQUENCY_DIVIDED_BY_2_TO_THE_21ST   = 0,
    PULSED_AT_CLOCK_FREQUENCY_DIVIDED_BY_2_TO_THE_20TH           = 1,
    AT_DC                                                        = 2
};

typedef struct
{
    unsigned char Config_3_Reserved:                      2;           // Reserved - Needs to be written with 0's
    unsigned char Disable_Right_Leg_Detect_Buffer:        1;           // RLD Buffer Power (PD_RLD)
    unsigned char Right_Leg_Detect_Reference_Source:      1;           // RLD Reference Select (RLDREF)
    unsigned char Right_Leg_Detect_Signal_Route:          1;           // RLD Output Routed to ADC Mux (RLD_MEAS)
    unsigned char Reference_Voltage:                      1;           // Reference Voltage (VREF_4V)
    unsigned char Config_3_Reserved_1:                    1;           // Reserved - Needs to be written with 1
    unsigned char Internal_Reference_Buffer:              1;           // Reference Voltage (PD_REFBUR')
} ADS1x9x_Config_3_Register_type;

// CONFIG_3_RESERVED_1 bit field definition
#define CONFIG_3_RESERVED_1_VALUE                         1

enum Right_Leg_Detect_Reference_Source_Type
{
    DEFAULT_IS_FED_EXTERNALLY                           = 0,
    IS_FED_INTERNALLY                                   = 1
};

enum Right_Leg_Detect_Signal_Route_Type
{
    DEFAULT_IS_OPEN                                     = 0,
    ROUTED_TO_MUX_SETTING_VREF                          = 1
};

enum Reference_Voltage_Type
{
    DEFAULT_IS_SET_TO_2_4_VOLTS                         = 0,
    IS_SET_TO_4_VOLTS                                   = 1
};

typedef struct
{
    unsigned char Lead_Off_Frequency:                     2;           // Frequency Lead (FLEAD_OFF[1:0])
    unsigned char Lead_Off_Current:                       2;           // Lead off Current Magnitude (ILEAD_OFF[1:0])
    unsigned char Lead_Off_Detection_Mode:                1;           // Lead off Detection Mode (VLEAD_OFF_EN)
    unsigned char Lead_Off_Comparator_Threshold:          3;           // Lead off Comparator Threshold (COMP_TH[2:0])
} ADS1x9x_Lead_Off_Control_Register_type;

enum Lead_Off_Frequency_Type
{
    DEFAULT_LEAD_OFF_DETECTION_DISABLED                 = 0,
    ONE_HALF_THE_OUTPUT_DATA_RATE                       = 1,
    ONE_FOURTH_THE_OUTPUT_DATA_RATE                     = 2,
    DC_LEAD_OFF_DETECT                                  = 3
};

enum Lead_Off_Current_Type
{
    DEFAULT_12_5_NA                                     = 0,
    _25_NA                                              = 1,
    _35_5NA                                             = 2,
    _50NA                                               = 3
};

enum Lead_Off_Detection_Mode_Type
{
    DEFAULT_CURRENT_MODE                                = 0,
    VOLTAGE_MODE                                        = 1
};

enum Lead_Off_Comparator_Threshold_Type
{
    DEFAULT_55_PERCENT                                  = 0,
    _60_PERCENT                                         = 1,
    _65_PERCENT                                         = 2,
    _70_PERCENT                                         = 3,
    _75_PERCENT                                         = 4,
    _80_PERCENT                                         = 5,
    _85_PERCENT                                         = 6,
    _90_PERCENT                                         = 7
};

typedef struct
{
    unsigned char Channel_Input_Is:                       3;           // Channel Input (MUX[2:0])
    unsigned char Channel_Setting_Reserved:               1;           // Reserved (Must Write 0)
    unsigned char Programmable_Gain_Setting:              3;           // PGA Gain (GAIN[2:0])
    unsigned char Power_Down_Channel:                     1;           // Power-Down (PD)
} ADS1x9x_Channel_Settings_Register_type;

enum Programmable_Gain_Setting_Type
{
    DEFAULT_GAIN_OF_6                                   = 0,
    GAIN_OF_1                                           = 1,
    GAIN_OF_2                                           = 2,
    GAIN_OF_3                                           = 3,
    GAIN_OF_4                                           = 4,
    GAIN_OF_8                                           = 5,
    GAIN_OF_12                                          = 6
};

enum Channel_Input_Is_Type
{
    DEFAULT_ADS1x9x_ELECTRODE_INPUT                     = 0,
    ADS1x9x_INPUT_SHORTED                               = 1,
    ADS1x9x_RIGHT_LEG_DETECT                            = 2,
    ADS1x9x_ONE_HALF_DIGITAL_SUPPLY                     = 3,
    ADS1x9x_TEMPERATURE_SENSOR                          = 4,
    ADS1x9x_TEST_SIGNAL                                 = 5,
    ADS1x9x_RIGHT_LEG_DETECT_POSITIVE                   = 6,
    ADS1x9x_RIGHT_LEG_DETECT_NEGATIVE                   = 7
};

// Used in Test Routine
#define ADS1x9x_CHANNEL_IS_DISABLED                     0xFF

enum Power_Down_Channel_Type
{
    DEFAULT_DISABLE_POWER_DOWN                          = 0,
    ENABLE_POWER_DOWN                                   = 1
};

typedef struct
{
    unsigned char GPIO_Direction:                         4;           // Corresponding GPIOD (GPIOC[3:0])
    unsigned char GPIO_Data:                              4;           // GPIO Data (GPIOD[4:7])
} ADS1x9x_GPIO_Settings_Register_type;

typedef struct
{
    unsigned char Pace_Detect_Buffer:                     1;           // PACE detect buffer (PD_PACE')
    unsigned char Odd_Pace_Channel_Select:                2;           // PACE_OUT1 odd (PACEO[1:0]
    unsigned char Even_Pace_Channel_Select:               2;           // PACE_OUT2 even (PACEE[1:0])
    unsigned char Bypass_Respiration_DC_Block_Capacitor:  1;           // Respiration DC Block (RESP_NO_DCBLK)
    unsigned char Channel_1_Internal_Demodulation:        1;           // Channel 1 internal demodulation (RESP_DEMOD_EN1)
    unsigned char Channel_2_Internal_Demodulation:        1;           // Channel 2 internal demodulation (RESP_DEMOD_EN2)
} ADS1x9x_PACE_Detect_Settings_Register_type;


// ODD_PACE_CHANNEL_SELECT & EVEN_PACE_CHANNEL_SELECT bit field definition
enum Pace_Channel_Select_Type
{
    DEFAULT_PACE_CHANNEL_IS_1                           = 0,
    DEFAULT_PACE_CHANNEL_IS_2                           = 0,
    PACE_CHANNEL_IS_3                                   = 1,
    PACE_CHANNEL_IS_4                                   = 1,
    PACE_CHANNEL_IS_5                                   = 2,
    PACE_CHANNEL_IS_6                                   = 2,
    PACE_CHANNEL_IS_7                                   = 3,
    PACE_CHANNEL_IS_8                                   = 3
};

typedef struct
{
    unsigned char Respiration_Clock_Source:               2;           // Respiration Control Mode (RESP_CTRL[1:0])
    unsigned char Respiration_Signal_Phase:               2;           // Respiration Phase (RESP_PH[2:0])
    unsigned char Respiration_Frequency:                  1;           // Respiration Frequency (RESP_FREQ)
    unsigned char Channel_1_Internal_Modulation:          1;           // Channel 1 internal modulation (RESP_MOD_EN1)
    unsigned char Channel_2_Internal_Modulation:          1;           // Channel 2 internal modulation (RESP_MOD_EN2)
} ADS1x9x_Respiration_Control_Register_type;

enum Respiration_Signal_Phase_Type
{
    DEFAULT_PHASE_IS_22_5_DEGREES                       = 0,
    PHASE_IS_45_DEGREES                                 = 1,
    PHASE_IS_67_5_DEGREES                               = 2,
    PHASE_IS_90_DEGREES                                 = 3,
    PHASE_IS_112_5_DEGREES                              = 4,
    PHASE_IS_135_DEGREES                                = 5,
    PHASE_IS_157_5_DEGREES                              = 6,
    PHASE_IS_180_DEGREES                                = 7
};

enum Respiration_Clock_Source_Type
{
    DEFAULT_RESPIRATION_MODE_DISABLED                        = 0,
    GPIO_3_IS_RESP_CLK_OUT_AND_GPIO_4_IS_RESP_PHASE_CLK_OUT  = 1,
    INTERNAL_RESPIRATION_USING_INTERNAL_CLOCK                = 2,
    GPIO_3_IS_RESP_CLK_IN_AND_GPIO_4_IS_RESP_PHASE_CLK_IN    = 3
};

typedef struct
{
    unsigned char Config_4_Reserved_2:                    1;           // Reserved (Must be One)
    unsigned char Disable_Lead_Off_Comparators:           1;           // Respiration Frequency (RESP_FREQ)
    unsigned char Config_4_Reserved_1:                    1;           // Reserved (Must be Zero)
    unsigned char Pulse_Mode:                             1;           // Pulse Mode (PULSE_MODE)
    unsigned char Config_4_Reserved_0:                    4;           // Reserved (Must be Zero)
} ADS1x9x_Config_4_Register_type;

// CONFIG_2_RESERVED bit field definition
#define CONFIG_4_RESERVED_2_VALUE                         1

//-----------------------------------------------------------------------------------------------------------------
enum ADS1x9x_Command_Values
{
// System Commands                                                     //  ADS1x9x Command Definitions
    WAKE_CONVERTER_FROM_SLEEP                           = 0x02,        //  WAKEUP
    PLACE_CONVERTER_IN_SLEEP_MODE                       = 0x04,        //  SLEEP
    RESET_CONVERTER                                     = 0x06,        //  RESET
    START_RESTART_CONVERSION                            = 0x08,        //  START
    STOP_CONVERSION                                     = 0x0A,        //  STOP

// Cal Commands
    CALIBRATE_OFFSET_FOR_ALL_CHANNELS                   = 0x1A,        //  OFFCAL

// Data Read Commands
    SET_READ_DATA_CONTINUOUSLY                          = 0x10,        //  RDATAC
    STOP_READ_DATA_CONTINUOUSLY                         = 0x11,        //  SDATAC
    READ_DATA_MANUALLY                                  = 0x12,        //  RDATA

//  Register Read Commands
    DEFAULT_READ_NUMBER_OF_REGISTERS                    = 0x21,
    DEFAULT_WRITE_NUMBER_OF_REGISTERS                   = 0x41
};

//  Specific Register Read Commands
enum Specific_Register_Read_Command_Type
{
//  Device Settings
    READ_DEVICE_ID                                      = 0x20,
// Glocal Settings Across Channels
    READ_CONFIG_1_REGISTER                              = 0x21,
    WRITE_CONFIG_1_REGISTER                             = 0x41,
    READ_CONFIG_2_REGISTER                              = 0x22,
    WRITE_CONFIG_2_REGISTER                             = 0x42,
    READ_CONFIG_3_REGISTER                              = 0x23,
    WRITE_CONFIG_3_REGISTER                             = 0x43,
    READ_CONFIG_4_REGISTER                              = 0x37,
    WRITE_CONFIG_4_REGISTER                             = 0x57,
    READ_LEAD_OFF_CONTROL_REGISTER                      = 0x24,
    WRITE_LEAD_OFF_CONTROL_REGISTER                     = 0x44,
// Channel Specific Settings
    READ_CHANNEL_1_SET_REGISTER                         = 0x25,
    WRITE_CHANNEL_1_SET_REGISTER                        = 0x45,
    READ_CHANNEL_2_SET_REGISTER                         = 0x26,
    WRITE_CHANNEL_2_SET_REGISTER                        = 0x46,
    READ_CHANNEL_3_SET_REGISTER                         = 0x27,
    WRITE_CHANNEL_3_SET_REGISTER                        = 0x47,
    READ_CHANNEL_4_SET_REGISTER                         = 0x28,
    WRITE_CHANNEL_4_SET_REGISTER                        = 0x48,
    READ_CHANNEL_5_SET_REGISTER                         = 0x29,
    WRITE_CHANNEL_5_SET_REGISTER                        = 0x49,
    READ_CHANNEL_6_SET_REGISTER                         = 0x2A,
    WRITE_CHANNEL_6_SET_REGISTER                        = 0x4A,
    READ_CHANNEL_7_SET_REGISTER                         = 0x2B,
    WRITE_CHANNEL_7_SET_REGISTER                        = 0x4B,
    READ_CHANNEL_8_SET_REGISTER                         = 0x2C,
    WRITE_CHANNEL_8_SET_REGISTER                        = 0x4C,

    READ_RIGHT_LEG_DRIVE_SENSE_POSITIVE_REGISTER        = 0x2D,
    WRITE_RIGHT_LEG_DRIVE_SENSE_POSITIVE_REGISTER       = 0x4D,
    READ_RIGHT_LEG_DRIVE_SENSE_NEGATIVE_REGISTER        = 0x2E,
    WRITE_RIGHT_LEG_DRIVE_SENSE_NEGATIVE_REGISTER       = 0x4E,

    READ_LEAD_OFF_SENSE_POSITIVE_REGISTER               = 0x2F,
    WRITE_LEAD_OFF_SENSE_POSITIVE_REGISTER              = 0x4F,
    READ_LEAD_OFF_SENSE_NEGATIVE_REGISTER               = 0x30,
    WRITE_LEAD_OFF_SENSE_NEGATIVE_REGISTER              = 0x50,

    READ_LEAD_OFF_FLIP_REGISTER                         = 0x31,
    WRITE_LEAD_OFF_FLIP_REGISTER                        = 0x51,
// Lead Off Status Registers
    READ_LEAD_OFF_STATUS_POSITIVE_REGISTER              = 0x32,
    READ_LEAD_OFF_STATUS_NEGATIVE_REGISTER              = 0x33,
// GPIO and OTHER Registers
    READ_GENERAL_PORT_IO                                = 0x34,
    WRITE_GENERAL_PORT_IO                               = 0x54,
    READ_PACE_DETECT_REGISTER                           = 0x35,
    WRITE_PACE_DETECT_REGISTER                          = 0x55,

    READ_RESPIRATION_CONTROL_REGISTER                   = 0x36,
    WRITE_RESPIRATION_CONTROL_REGISTER                  = 0x56,
    READ_CONFIGURATION_CONTROL_REGISTER                 = 0x37,
    WRITE_CONFIGURATION_CONTROL_REGISTER                = 0x57
};

#define SINGLE_BYTE_READ_WRITE                            0x00

#define ADS1x9x_SPI_WRITE_DELAY                           0x04 //用于SPI通信时的延时

#define ADS1x9x_TOP_REGISTER_SIZE                         0x11 //0x01~0x11h,总共有11个寄存器
#define ADS1x9x_REGISTER_OFFSET                           0x14 //0x12h~0x13h，跳过两个只读寄存器
#define ADS1x9x_BOTTOM_REGISTER_SIZE                      0x06 //0x14~0x19h,总共有6个寄存器


/**************************************************************************************************************************************************
*               Union Structures and Definitions                                                                                                  *
**************************************************************************************************************************************************/
// BITWISE Definitions
#define DISABLE                 0
#define ENABLE                  1

#define DEFAULT_DISABLE         0
#define DEFAULT_ENABLE          1

#define DISABLED                0
#define ENABLED                 1

#define DEFAULT_DISABLED        0
#define DEFAULT_ENABLED         1

#define LOW                     0
#define HIGH                    1

#define DEFAULT_LOW             0
#define DEFAULT_HIGH            1

#define INACTIVE                0
#define ACTIVE                  1

#define CLEAR                   0
#define SET                     1

#define OFF                     0
#define ON                      1

#define NULL                    0


#endif

