#include "mpu6050.h"

static uint32_t MPU6050_I2C_Timeout;

TM_MPU6050_Result_t MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {

	uint8_t temp;
	
	MPU6050_I2C_Init();

	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR;
	
	/* Check if device is connected */
	if (!MPU6050_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->Address)) {
		/* Return error */
		return TM_MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who I am */
	if (I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return TM_MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */
	I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_PWR_MGMT_1, 0x00);
	
	/* Config accelerometer */
	temp = I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG, temp);
	
	/* Config gyroscope */
	temp = I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case TM_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case TM_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case TM_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case TM_MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case TM_MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case TM_MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

void MPU6050_I2C_Init() {
	/*
	 *         SCL = PB6
	 *         SDA = PB7
	 */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable IOE_I2C and IOE_I2C_GPIO_PORT & Alternate Function clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect PXx to I2C_SCL*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	/* Connect PXx to I2C_SDA*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	/* SCL and SDA pins configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the I2C event priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the I2C peripheral */
	I2C_DeInit(MPU6050_I2C);

	I2C_InitTypeDef I2C_InitStructure;

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_CLOCK;

	/* Initialize the I2C peripheral */
	I2C_Init(MPU6050_I2C, &I2C_InitStructure);

	I2C_ITConfig(MPU6050_I2C, I2C_IT_ERR, ENABLE);
	I2C_Cmd(MPU6050_I2C, ENABLE);
}

TM_MPU6050_Result_t MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read accelerometer data */
	I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read gyroscope data */
	I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct) {
	uint8_t data[2];
	int16_t temp;
	
	/* Read temperature */
	I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_TEMP_OUT_H, data, 2);
	
	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t MPU6050_ReadAll(TM_MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;
	
	/* Read full raw data, 14bytes */
	I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);
	
	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

uint8_t MPU6050_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address) {
	uint8_t connected = 0;
	/* Try to start, function will return 0 in case device will send ACK */
	if (!I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Enable)) {
		connected = 1;
	}

	/* STOP I2C */
	I2C_Stop(I2Cx);

	/* Return status */
	return connected;
}

int16_t I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint16_t ack) {
	/* Generate I2C start pulse */
	I2Cx->CR1 |= I2C_CR1_START;

	/* Wait till I2C is busy */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_SB)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Enable ack if we select it */
	if (ack) {
		I2Cx->CR1 |= I2C_CR1_ACK;
	}

	/* Send write/read bit */
	if (direction == I2C_Direction_Transmitter) {
		/* Send address with zero last bit */
		I2Cx->DR = address & ~I2C_OAR1_ADD0;

		/* Wait till finished */
		MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
		while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
			if (--MPU6050_I2C_Timeout == 0x00) {
				return 1;
			}
		}
	}
	if (direction == I2C_Direction_Receiver) {
		/* Send address with 1 last bit */
		I2Cx->DR = address | I2C_OAR1_ADD0;

		/* Wait till finished */
		MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if (--MPU6050_I2C_Timeout == 0x00) {
				return 1;
			}
		}
	}

	/* Read status register to clear ADDR flag */
	I2Cx->SR2;

	/* Return 0, everything ok */
	return 0;
}

uint8_t I2C_Stop(I2C_TypeDef* I2Cx) {
	/* Wait till transmitter not empty */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF)))) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;

	/* Return 0, everything ok */
	return 0;
}

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data;
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, reg);
	I2C_Stop(I2Cx);
	I2C_Start(I2Cx, address, I2C_Direction_Receiver, I2C_Ack_Disable);
	received_data = I2C_ReadNack(I2Cx);
	return received_data;
}

uint8_t I2C_ReadAck(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Enable ACK */
	I2Cx->CR1 |= I2C_CR1_ACK;

	/* Wait till not received */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

uint8_t I2C_ReadNack(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Disable ACK */
	I2Cx->CR1 &= ~I2C_CR1_ACK;

	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;

	/* Wait till received */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--MPU6050_I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

void I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Enable);
	I2C_WriteData(I2Cx, reg);
	I2C_Stop(I2Cx);
	I2C_Start(I2Cx, address, I2C_Direction_Receiver, I2C_Ack_Enable);
	for (i = 0; i < count; i++) {
		if (i == (count - 1)) {
			/* Last byte */
			data[i] = I2C_ReadNack(I2Cx);
		} else {
			data[i] = I2C_ReadAck(I2Cx);
		}
	}
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	I2C_Start(I2Cx, address, I2C_Direction_Transmitter, I2C_Ack_Disable);
	I2C_WriteData(I2Cx, reg);
	I2C_WriteData(I2Cx, data);
	I2C_Stop(I2Cx);
}

void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	/* Wait till I2C is not busy anymore */
	MPU6050_I2C_Timeout = MPU6050_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_TXE) && MPU6050_I2C_Timeout) {
		MPU6050_I2C_Timeout--;
	}

	/* Send I2C data */
	I2Cx->DR = data;
}
