#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f429i_discovery.h"

#include "mpu6050.h"

#define MUP6050_ADDRESS 0x68

unsigned int strlen(const char *str)
{
	unsigned int i = 0;
	for(i=0; str[i]!='\0'; i++);
	return i;
}

void reverse(char *str)
{
	unsigned int i = 0;
	unsigned int length = strlen(str) - 1;
	char c;

	for (i = 0; i < length; i++, length--) {
		c = str[i];
		str[i] = str[length];
		str[length] = c;
	}
}

void itoa(uint16_t n, char *str)
{
	int i = 0, sign;
	if ((sign = n) < 0)
		n = -n;
	do {
		str[i++] = n % 10 + '0';
	} while((n /= 10) > 0);
	if (sign < 0)
		str[i++] = '-';
	str[i] = '\0';
	reverse(str);
}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

void I2C_Config(void)
{


    /* 
    *         SCL = PB6
    *         SDA = PB7
    */


  GPIO_InitTypeDef GPIO_InitStructure;

  
  /* Enable IOE_I2C and IOE_I2C_GPIO_PORT & Alternate Function clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOB , ENABLE);

  
  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); 
    
  /* SCL and SDA pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the I2C event priority */
  NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
 

        /* Enable the I2C peripheral */
  I2C_DeInit( I2C1);
        
  I2C_InitTypeDef I2C_InitStructure;

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;
//  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  
  /* Initialize the I2C peripheral */
  I2C_Init( I2C1 , &I2C_InitStructure);

 
       
  I2C_ITConfig(I2C1,I2C_IT_ERR , ENABLE);
  I2C_Cmd( I2C1 , ENABLE);
  
}


void I2C1_ER_IRQHandler(void)
{
  /* ACK failure */
  
  if (I2C_GetITStatus(I2C1, I2C_IT_AF)) 
  {
    USART1_puts("I2C_IT_AF");
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_BERR))
  {
    USART1_puts("I2C_IT_BERR");
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_ARLO))
  {
    USART1_puts("I2C_IT_ARLO");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_OVR))
  {
    USART1_puts("I2C_IT_OVR");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_PECERR))
  {
    USART1_puts("I2C_IT_PECERR");
  }
    else if(I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
  {
    USART1_puts("I2C_IT_TIMEOUT");
  }
  else if(I2C_GetITStatus(I2C1, I2C_IT_SMBALERT))
  {
    USART1_puts("I2C_IT_SMBALERT");
  }
    
}


uint8_t I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
 {
	// wait until I2C1 is not busy anymore
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

  return  0;
}

/* This function transmits one byte to the slave device
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1
* data --> the data byte to be transmitted
*/
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2C_SendData(I2Cx, data);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This funtion issues a stop condition and therefore
* releases the bus
*/
void I2C_stop(I2C_TypeDef* I2Cx){
    // Send I2C1 STOP Condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

/* This function reads one byte from the slave device
* and acknowledges the byte (requests another byte)
*/
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
    uint8_t data;
    // enable acknowledge of recieved data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This function reads one byte from the slave device
* and doesn't acknowledge the recieved data
*/
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
    uint8_t data;
    // disabe acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}


void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}
 

 void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 9600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}



int main(){

	TM_MPU6050_t MPU6050_Data0;
	uint8_t sensor1 = 0;
	char str[120];

	RCC_Configuration();
  	GPIO_Configuration();
  	USART1_Configuration();	

	/* Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low */
	if (MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s)
			== TM_MPU6050_Result_Ok) {
		/* Display message to user */
		USART1_puts("MPU6050 sensor 0 is ready to use!\n");

		/* Sensor 1 OK */
		sensor1 = 1;
	}
	
	uint8_t received_data[2];
	char uart_out[32];

	while(1){

		/* If sensor 1 is connected */
		if (sensor1) {
			/* Read all data from sensor 1 */
			MPU6050_ReadAll(&MPU6050_Data0);

			USART1_puts("\nNext: ");
			USART1_puts(" AX: ");
			itoa(MPU6050_Data0.Accelerometer_X, uart_out);
			USART1_puts(uart_out);
			USART1_puts(" AY: ");
			itoa(MPU6050_Data0.Accelerometer_Y, uart_out);
			USART1_puts(uart_out);
			USART1_puts(" AZ: ");
			itoa(MPU6050_Data0.Accelerometer_Z, uart_out);
			USART1_puts(uart_out);
		}
		for (int i = 0; i < 100000; i++);
	}
}
