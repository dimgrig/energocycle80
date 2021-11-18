/****************************************************************************************
**  LINX header for STM32
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**  
**  Written By Ksen
**
** BSD2 License.
****************************************************************************************/	

#ifndef LINX_STM32F103_H
#define LINX_STM32F103_H

/****************************************************************************************
**  Defines
****************************************************************************************/	
#define DEVICE_NAME_LEN 10 // STM32F103+*NUL_symbol*

#define NUM_AI_CHANS 6 	//Кол-во каналов АЦП
#define AI_RES_BITS 12		//Разрядность АЦП  (AiResolution)
#define AI_REFV 3300000		//Макс. напряжение АЦП (3.3 В)
#define NUM_AI_INT_REFS 1	//���-�� ��������������� ������� ����������

#define NUM_CAN_CHANS

#define NUM_DIGITAL_CHANS 6
			//PB[1] - выход: (вкл/выкл схемы питания), PB[3] - выход: (вентилятор), PB[8, 9] - входы: (резерв), PA[9, 10] - выходы: (резерв)

#define NUM_PWM_CHANS 4		 //���-�� ��� ������� (4 ������)

#define NUM_SPI_CHANS 2      //??? 3 2 ����������� SPI
#define NUM_SPI_SPEEDS 1	 //�������� (��, ����� ����� ���� ��������)

#define NUM_I2C_CHANS 0 //1		 //1 �����

#define NUM_UART_CHANS 2 //3 //1 �����
#define NUM_UART_SPEEDS 13	//���-�� ��������� UART

#define NUM_SERVO_CHANS NUM_DIGITAL_CHANS

/****************************************************************************************
**  Includes
****************************************************************************************/	
#include "LinxDevice.h"
#include "LinxWiringDevice.h"
#include "LinxArduino.h"
	
class LinxSTM32F103 : public LinxArduino
{
	public:	
		/****************************************************************************************
		**  Variables
		****************************************************************************************/		
		//System
		static const unsigned char m_DeviceName[DEVICE_NAME_LEN];
		
		//AI
		static const unsigned char m_AiChans[NUM_AI_CHANS];
		static const unsigned long m_AiRefIntVals[NUM_AI_INT_REFS];
		static const int m_AiRefCodes[NUM_AI_INT_REFS];
		
		//AI
		//None
		
		//CAN
		static const unsigned char m_CanChans[NUM_CAN_CHANS];

		//DIGITAL
		static const unsigned char m_DigitalChans[NUM_DIGITAL_CHANS];	
		
		//PWM
		static const unsigned char m_PwmChans[NUM_PWM_CHANS];			
		
		//SPI
		static const unsigned char m_SpiChans[NUM_SPI_CHANS];
		static unsigned long m_SpiSupportedSpeeds[NUM_SPI_SPEEDS];
		static int m_SpiSpeedCodes[NUM_SPI_SPEEDS];
		
		//I2C
		static unsigned char m_I2cChans[NUM_I2C_CHANS];
		static unsigned char m_I2cRefCount[NUM_I2C_CHANS];						
		
		//UART
		static unsigned char m_UartChans[NUM_UART_CHANS];
		static unsigned long m_UartSupportedSpeeds[NUM_UART_SPEEDS];
		
		//Servo		
		static const unsigned char m_ServoChans[NUM_SERVO_CHANS];
		//static Servo* m_Servos[NUM_SERVO_CHANS];
		
		/****************************************************************************************
		**  Constructors /  Destructor
		****************************************************************************************/
		LinxSTM32F103();
		
		~LinxSTM32F103();
			
		/****************************************************************************************
		**  Functions
		****************************************************************************************/
		
		
	private:
		/****************************************************************************************
		**  Variables
		****************************************************************************************/		
				
		
		/****************************************************************************************
		**  Functions
		****************************************************************************************/
		
		
};


#endif //LINX_ARDUINO_NANO_H
