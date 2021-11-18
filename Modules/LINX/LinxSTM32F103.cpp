/****************************************************************************************
**  LINX STM32F103 code
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**  
**  Written By Ksen
**
** BSD2 License.
****************************************************************************************/	

/****************************************************************************************
**  Includes
****************************************************************************************/		
//#include <SPI.h>

#include "LinxDevice.h"
#include "LinxWiringDevice.h"
#include "LinxArduino.h"
#include "LinxSTM32F103.h"

/****************************************************************************************
**  Member Variables
****************************************************************************************/
//System
const unsigned char LinxSTM32F103::m_DeviceName[DEVICE_NAME_LEN] = "STM32F103";

//AI
const unsigned char LinxSTM32F103::m_AiChans[NUM_AI_CHANS] = {0, 1, 2, 3, 4, 5};	//������ ��� ��� (ADC_IN[0, 1, 4, 8, 10, 11])
const unsigned long LinxSTM32F103::m_AiRefIntVals[NUM_AI_INT_REFS] = {};	//??? ����� ����� �� �������� �������� ��� ���� �������, ��� ������ �����
const int LinxSTM32F103::m_AiRefCodes[NUM_AI_INT_REFS] = {};				//???

//AO
//None

//CAN
const unsigned char LinxSTM32F103::m_CanChans[NUM_CAN_CHANS] = {0};

//DIGITAL
const unsigned char LinxSTM32F103::m_DigitalChans[NUM_DIGITAL_CHANS] = {1, 2, 3, 4, 5, 6};
												//1-PB[1], 2-PB[3], 3- PB[8], 4-PB[9], 5-PA[9], 6-PA[10]

//PWM
const unsigned char LinxSTM32F103::m_PwmChans[NUM_PWM_CHANS] = {1, 2, 3, 4}; //4 ������ ���

//QE
//None

//SPI
const unsigned char LinxSTM32F103::m_SpiChans[NUM_SPI_CHANS] = {1, 2}; // {0, 1, 2};
unsigned long LinxSTM32F103::m_SpiSupportedSpeeds[NUM_SPI_SPEEDS] = {36000000}; //36���
int LinxSTM32F103::m_SpiSpeedCodes[NUM_SPI_SPEEDS] = {36000000};			//36��� = 72/2

//I2C
unsigned char LinxSTM32F103::m_I2cChans[NUM_I2C_CHANS] = {}; // {1}
unsigned char LinxSTM32F103::m_I2cRefCount[NUM_I2C_CHANS];

//UART
unsigned char LinxSTM32F103::m_UartChans[NUM_UART_CHANS] = {1,2}; //{1, 2, 3};
unsigned long LinxSTM32F103::m_UartSupportedSpeeds[NUM_UART_SPEEDS] = {300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200};

//SERVO
//Servo* LinxSTM32F103::m_Servos[NUM_SERVO_CHANS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};			//Initialize To Null Pointers

/****************************************************************************************
**  Constructors /  Destructor
****************************************************************************************/
LinxSTM32F103::LinxSTM32F103()
{
	//Arduino Family Code Set At Family Level
	DeviceId = 0x00;	//(0x5 == Nano) 
	DeviceNameLen = DEVICE_NAME_LEN;	 
	DeviceName =  m_DeviceName;
	ListenerBufferSize = 64;

	//LINX API Version
	LinxApiMajor = 3;
	LinxApiMinor = 0;
	LinxApiSubminor = 0;
	
	//DIGITAL
	NumDigitalChans = NUM_DIGITAL_CHANS;			
	DigitalChans = m_DigitalChans;
		
	//AI
	NumAiChans = NUM_AI_CHANS;
	AiChans = m_AiChans;
	AiResolution = AI_RES_BITS;
	AiRefSet = AI_REFV;
		
	AiRefDefault = AI_REFV;
	AiRefSet = AI_REFV;
	AiRefCodes = m_AiRefCodes;
	
	NumAiRefIntVals = NUM_AI_INT_REFS;
	AiRefIntVals = m_AiRefIntVals;
	
	AiRefExtMin = 0;
	AiRefExtMax = 5000000;
	
	//AO
	NumAoChans = 0;
	AoChans = 0;
	
	//PWM
	NumPwmChans = NUM_PWM_CHANS;
	PwmChans = m_PwmChans;
	
	//QE
	NumQeChans = 0;
	QeChans = 0;
	
	
	//UART
	NumUartChans = NUM_UART_CHANS;
	UartChans = m_UartChans;
	UartMaxBaud = m_UartSupportedSpeeds[NUM_UART_SPEEDS - 1];
	NumUartSpeeds = NUM_UART_SPEEDS;
	UartSupportedSpeeds = m_UartSupportedSpeeds;

	//I2C
	NumI2cChans = NUM_I2C_CHANS;	
	I2cChans = m_I2cChans;
	I2cRefCount = m_I2cRefCount;
		
	//SPI
	NumSpiChans = NUM_SPI_CHANS;	
	SpiChans = m_SpiChans;		
	NumSpiSpeeds = NUM_SPI_SPEEDS;
	SpiSupportedSpeeds = m_SpiSupportedSpeeds;
	SpiSpeedCodes = m_SpiSpeedCodes;
		
	//CAN
	NumCanChans = 0;
	CanChans = 0;
	
	//SERVO
	NumServoChans = NUM_SERVO_CHANS;	
	ServoChans = m_DigitalChans;
	//Servos = m_Servos;
	
	//If Debuging Is Enabled Call EnableDebug()
	#if DEBUG_ENABLED > 0
		EnableDebug(DEBUG_ENABLED);
	#endif
}

//Destructor
LinxSTM32F103::~LinxSTM32F103()
{
	//Handle Any Device Clean Up Here.
	//UartClose();
}

/****************************************************************************************
**  Functions
****************************************************************************************/

