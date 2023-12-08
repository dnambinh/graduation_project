/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Input capture callback
uint32_t channel_1_start, channel_1;
uint32_t channel_2_start, channel_2;
uint32_t channel_3_start, channel_3;
uint32_t channel_4_start, channel_4;

volatile uint32_t diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;

uint8_t firt_capture1 = 0, firt_capture2 = 0, firt_capture3 = 0, firt_capture4 =
		0;	//fist_capture? 0-no 1-yes

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (firt_capture1 == 0) {
			channel_1_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			firt_capture1 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture1 == 1) {
			channel_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (channel_1 > channel_1_start)
				diff1 = channel_1 - channel_1_start;
			else if (channel_1 < channel_1_start)
				diff1 = (0xffff - channel_1_start) + channel_1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture1 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (firt_capture2 == 0) {					//read
			channel_2_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			firt_capture2 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture2 == 1) {
			channel_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if (channel_2 > channel_2_start)
				diff2 = channel_2 - channel_2_start;
			else if (channel_2 < channel_2_start)
				diff2 = (0xffff - channel_2_start) + channel_2;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture2 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (firt_capture3 == 0) {					//read
			channel_3_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			firt_capture3 = 1;
			//Change the input capture mode to the falling edge of the pulse
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture3 == 1) {
			channel_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			if (channel_3 > channel_3_start)
				diff3 = channel_3 - channel_3_start;
			else if (channel_3 < channel_3_start)
				diff3 = (0xffff - channel_3_start) + channel_3;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture3 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (firt_capture4 == 0) {					//read
			channel_4_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			firt_capture4 = 1;
			//Change the input capture mode to the falling edge of the pulse
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture4 == 1) {
			channel_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			if (channel_4 > channel_4_start)
				diff4 = channel_4 - channel_4_start;
			else if (channel_4 < channel_4_start)
				diff4 = (0xffff - channel_4_start) + channel_4;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture4 = 0;
		}
	}
}
//MPU6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0, Gyro_X_RAW = 0,
		Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;
int16_t temp_raw = 0;
float Ax, Ay, Az, Gx, Gy, Gz, temp;

//the gyro offset
//int16_t os_Ax = 71, os_Ay = 96, os_Az = 4456, os_Gx = -299, os_Gy = -43, os_Gz = 13;	//1
//int16_t os_Ax = 79, os_Ay = 90, os_Az = 4463, os_Gx = -301, os_Gy = -43, os_Gz = 13;	//2
//int16_t os_Ax = -50, os_Ay = 115, os_Az = 4569, os_Gx = -302, os_Gy = -43, os_Gz = 13;	//3
int16_t os_Ax = 114, os_Ay = 56, os_Az = 4566, os_Gx = -300, os_Gy = -52,
		os_Gz = 11;	//4
//int16_t os_Ax = 0, os_Ay = 0, os_Az = 0, os_Gx = 0, os_Gy = 0, os_Gz = 0;

int32_t ax_cal = 0, ay_cal = 0, az_cal = 0, gx_cal = 0, gy_cal = 0, gz_cal = 0;

void MPU6050_Init(void) {
	uint8_t check, Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 104) // 0x68 will be returned by the sensor if everything goes well
			{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
				1000);	//done

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> ± 500 °/s
		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1,
				1000);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=3 -> ± 8g
		Data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,
				1000);

		Data = 0x03;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read(void) {
	uint8_t Rec_Data[14];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,
			1000);

	Accel_X_RAW = (int16_t) Rec_Data[0] << 8 | Rec_Data[1];
	Accel_Y_RAW = (int16_t) Rec_Data[2] << 8 | Rec_Data[3];
	Accel_Z_RAW = (int16_t) Rec_Data[4] << 8 | Rec_Data[5];
	temp_raw = (int16_t) Rec_Data[6] << 8 | Rec_Data[7];
	Gyro_X_RAW = (int16_t) Rec_Data[8] << 8 | Rec_Data[9];
	Gyro_Y_RAW = (int16_t) Rec_Data[10] << 8 | Rec_Data[11];
	Gyro_Z_RAW = (int16_t) Rec_Data[12] << 8 | Rec_Data[13];

	Gyro_Y_RAW *= -1;
	Gyro_Z_RAW *= -1;

	Accel_X_RAW -= os_Ax;
	Accel_Y_RAW -= os_Ay;
	Accel_Z_RAW -= os_Az;
	Gyro_X_RAW -= os_Gx;
	Gyro_Y_RAW -= os_Gy;
	Gyro_Z_RAW -= os_Gz;
}

void offset_calculate() {
	for (int i = 0; i < 500; i++) {
		MPU6050_Read();
		HAL_Delay(4);
	}
	for (int j = 0; j < 3000; j++) {
		MPU6050_Read();
		ax_cal += Accel_X_RAW;
		ay_cal += Accel_Y_RAW;
		az_cal += Accel_Z_RAW;
		gx_cal += Gyro_X_RAW;
		gy_cal += Gyro_Y_RAW;
		gz_cal += Gyro_Z_RAW;
		HAL_Delay(4);
	}
	os_Ax = ax_cal / 3000;
	os_Ay = ay_cal / 3000;
	os_Az = az_cal / 3000;
	os_Gx = gx_cal / 3000;
	os_Gy = gy_cal / 3000;
	os_Gz = gz_cal / 3000;
}

//PID gain and limit settings
#define roll     0
#define pitch    1
#define yaw      2
// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = { 0, 0, 0 }; // roll, pitch, yaw
int pid_max[3] = { 180, 180, 20 };		 // roll, pitch, yaw
// Errors
float errors[3]; // Measured errors (compared to instructions) : [roll, pitch, yaw]
float delta_err[3] = { 0, 0, 0 }; // Error deltas in that order   : roll, pitch, yaw
float error_sum[3] = { 0, 0, 0 }; // Error sums (used for integral component) : [roll, pitch, yaw]
float previous_error[3] = { 0, 0, 0 }; // Last errors (used for derivative component) : [roll, pitch, yaw]
// PID coefficients
//float Kp[3] = { 0.4, 0.4, 0.4 };     	   // P: roll, pitch, yaw
float Kp[3] = { 4, 4, 0.4 };
float Ki[3] = {0.15, 0.15, 0.02}; 	 	// I: roll, pitch, yaw
//float Ki[3] = { 0.0, 0.0, 0.0 };
float Kd[3] = { 0.01, 0.01, 0 };         // D: roll, pitch, yaw

float pid_roll = 0;
float pid_pitch = 0;
float pid_yaw = 0;
// ---------------------------------------------------------------------------

// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = { 0, 0, 0 };
// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = { 0, 0, 0 };
// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3] = { 0, 0, 0 };
// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = { 0, 0, 0 };
// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = { 0, 0, 0 };
// Total 3D acceleration vector in m/s²
long acc_total_vector;
// Calculated angular motion on each axis: roll, pitch, yaw
float angular_motions[3] = { 0, 0, 0 };
float pre_angular_motions[3] = { 0, 0, 0 };
float measures[3] = { 0, 0, 0 };

uint8_t active, first_angle = 0;
int16_t esc1, esc2, esc3, esc4, throttle;
uint32_t looptimer;
float pitch_adjust, roll_adjust, pitch_angle, roll_angle;

//KALMAN-------------------------------------------------------------------------------------------
float Kalman_angle_pitch_acc, Kalman_angle_roll_acc, Kalman_Gx, Kalman_Gy,
		Kalman_Gz;

float _err_measure;
float _err_estimate;
float _q;
float _current_estimate;
float _last_estimate = 0;
float _kalman_gain;
void KalmanFilterX(float mea_e, float est_e, float q) {
	_err_measure = mea_e;
	_err_estimate = est_e;
	_q = q;
}
float updateEstimateX(float mea) {
	_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
	_current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
	_err_estimate = (1.0 - _kalman_gain) * _err_estimate
			+ fabs(_last_estimate - _current_estimate) * _q;
	_last_estimate = _current_estimate;
	return _current_estimate;
}

float _err_measure1;
float _err_estimate1;
float _q1;
float _current_estimate1;
float _last_estimate1 = 0;
float _kalman_gain1;
void KalmanFilterY(float mea_e1, float est_e1, float q1) {
	_err_measure1 = mea_e1;
	_err_estimate1 = est_e1;
	_q1 = q1;
}
float updateEstimateY(float mea1) {
	_kalman_gain1 = _err_estimate1 / (_err_estimate1 + _err_measure1);
	_current_estimate1 = _last_estimate1
			+ _kalman_gain1 * (mea1 - _last_estimate1);
	_err_estimate1 = (1.0 - _kalman_gain1) * _err_estimate1
			+ fabs(_last_estimate1 - _current_estimate1) * _q1;
	_last_estimate1 = _current_estimate1;
	return _current_estimate1;
}

float _err_measure2;
float _err_estimate2;
float _q2;
float _current_estimate2;
float _last_estimate2 = 0;
float _kalman_gain2;
void KalmanFilterGX(float mea_e2, float est_e2, float q2) {
	_err_measure2 = mea_e2;
	_err_estimate2 = est_e2;
	_q2 = q2;
}
float updateEstimateGX(float mea2) {
	_kalman_gain2 = _err_estimate2 / (_err_estimate2 + _err_measure2);
	_current_estimate2 = _last_estimate2
			+ _kalman_gain2 * (mea2 - _last_estimate2);
	_err_estimate2 = (1.0 - _kalman_gain2) * _err_estimate2
			+ fabs(_last_estimate2 - _current_estimate2) * _q2;
	_last_estimate2 = _current_estimate2;
	return _current_estimate2;
}

float _err_measure3;
float _err_estimate3;
float _q3;
float _current_estimate3;
float _last_estimate3 = 0;
float _kalman_gain3;
void KalmanFilterGY(float mea_e3, float est_e3, float q3) {
	_err_measure3 = mea_e3;
	_err_estimate3 = est_e3;
	_q3 = q3;
}
float updateEstimateGY(float mea3) {
	_kalman_gain3 = _err_estimate3 / (_err_estimate3 + _err_measure3);
	_current_estimate3 = _last_estimate3
			+ _kalman_gain3 * (mea3 - _last_estimate3);
	_err_estimate3 = (1.0 - _kalman_gain3) * _err_estimate3
			+ fabs(_last_estimate3 - _current_estimate3) * _q3;
	_last_estimate3 = _current_estimate3;
	return _current_estimate3;
}

float _err_measure4;
float _err_estimate4;
float _q4;
float _current_estimate4;
float _last_estimate4 = 0;
float _kalman_gain4;
void KalmanFilterGZ(float mea_e4, float est_e4, float q4) {
	_err_measure4 = mea_e4;
	_err_estimate4 = est_e4;
	_q4 = q4;
}
float updateEstimateGZ(float mea4) {
	_kalman_gain4 = _err_estimate4 / (_err_estimate4 + _err_measure4);
	_current_estimate4 = _last_estimate4
			+ _kalman_gain4 * (mea4 - _last_estimate4);
	_err_estimate4 = (1.0 - _kalman_gain4) * _err_estimate4
			+ fabs(_last_estimate4 - _current_estimate4) * _q4;
	_last_estimate4 = _current_estimate4;
	return _current_estimate4;
}

float _err_measureEsc1;
float _err_estimateEsc1;
float _qEsc1;
float _current_estimateEsc1;
float _last_estimateEsc1 = 0;
float _kalman_gainEsc1;
void Kalman_esc1(float mea_eEsc1, float est_eEsc1, float qEsc1) {
	_err_measureEsc1 = mea_eEsc1;
	_err_estimateEsc1 = est_eEsc1;
	_qEsc1 = qEsc1;
}
float update_esc1(float meaEsc1) {
	_kalman_gainEsc1 = _err_estimateEsc1
			/ (_err_estimateEsc1 + _err_measureEsc1);
	_current_estimateEsc1 = _last_estimateEsc1
			+ _kalman_gainEsc1 * (meaEsc1 - _last_estimateEsc1);
	_err_estimateEsc1 = (1.0 - _kalman_gainEsc1) * _err_estimateEsc1
			+ fabs(_last_estimateEsc1 - _current_estimateEsc1) * _qEsc1;
	_last_estimateEsc1 = _current_estimateEsc1;
	return _current_estimateEsc1;
}

float _err_measureEsc2;
float _err_estimateEsc2;
float _qEsc2;
float _current_estimateEsc2;
float _last_estimateEsc2 = 0;
float _kalman_gainEsc2;
void Kalman_esc2(float mea_eEsc2, float est_eEsc2, float qEsc2) {
	_err_measureEsc2 = mea_eEsc2;
	_err_estimateEsc2 = est_eEsc2;
	_qEsc2 = qEsc2;
}
float update_esc2(float meaEsc2) {
	_kalman_gainEsc2 = _err_estimateEsc2
			/ (_err_estimateEsc2 + _err_measureEsc2);
	_current_estimateEsc2 = _last_estimateEsc2
			+ _kalman_gainEsc2 * (meaEsc2 - _last_estimateEsc2);
	_err_estimateEsc2 = (1.0 - _kalman_gainEsc2) * _err_estimateEsc2
			+ fabs(_last_estimateEsc2 - _current_estimateEsc2) * _qEsc2;
	_last_estimateEsc2 = _current_estimateEsc2;
	return _current_estimateEsc2;
}

float _err_measureEsc3;
float _err_estimateEsc3;
float _qEsc3;
float _current_estimateEsc3;
float _last_estimateEsc3 = 0;
float _kalman_gainEsc3;
void Kalman_esc3(float mea_eEsc3, float est_eEsc3, float qEsc3) {
	_err_measureEsc3 = mea_eEsc3;
	_err_estimateEsc3 = est_eEsc3;
	_qEsc3 = qEsc3;
}
float update_esc3(float meaEsc3) {
	_kalman_gainEsc3 = _err_estimateEsc3
			/ (_err_estimateEsc3 + _err_measureEsc3);
	_current_estimateEsc3 = _last_estimateEsc3
			+ _kalman_gainEsc3 * (meaEsc3 - _last_estimateEsc3);
	_err_estimateEsc3 = (1.0 - _kalman_gainEsc3) * _err_estimateEsc3
			+ fabs(_last_estimateEsc3 - _current_estimateEsc3) * _qEsc3;
	_last_estimateEsc3 = _current_estimateEsc3;
	return _current_estimateEsc3;
}

float _err_measureEsc4;
float _err_estimateEsc4;
float _qEsc4;
float _current_estimateEsc4;
float _last_estimateEsc4 = 0;
float _kalman_gainEsc4;
void Kalman_esc4(float mea_eEsc4, float est_eEsc4, float qEsc4) {
	_err_measureEsc4 = mea_eEsc4;
	_err_estimateEsc4 = est_eEsc4;
	_qEsc4 = qEsc4;
}
float update_esc4(float meaEsc4) {
	_kalman_gainEsc4 = _err_estimateEsc4
			/ (_err_estimateEsc4 + _err_measureEsc4);
	_current_estimateEsc4 = _last_estimateEsc4
			+ _kalman_gainEsc4 * (meaEsc4 - _last_estimateEsc4);
	_err_estimateEsc4 = (1.0 - _kalman_gainEsc4) * _err_estimateEsc4
			+ fabs(_last_estimateEsc4 - _current_estimateEsc4) * _qEsc4;
	_last_estimateEsc4 = _current_estimateEsc4;
	return _current_estimateEsc4;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	MPU6050_Init();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	//offset_calculate();

	for (int i = 0; i < 20; i++) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
	}

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	//wait for connection tx-rx
	while (diff1 < 990 || diff2 < 990 || diff3 < 990 || diff4 < 990) {
	}
	while (diff3 < 990 || diff3 > 1050) {
	}

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//------------------------------------------------------
	//esc calibration
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);	//fl
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);	//fr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);	//rr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2000); //rl

	HAL_Delay(50);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	//fl
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	//fr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);	//rr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000); //rl

	HAL_Delay(300);
	//------------------------------------------------------

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	for (int i = 0; i < 20; i++) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
	}

	looptimer = HAL_GetTick();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		MPU6050_Read(); //ok
		// Apply low-pass filter (10Hz cutoff frequency)
		angular_motions[roll] = 0.7 * angular_motions[roll]
				+ ((float) Gyro_X_RAW * 0.3 / 65.5); //Gyro pid input is deg/sec.
		angular_motions[pitch] = 0.7 * angular_motions[pitch]
				+ ((float) Gyro_Y_RAW * 0.3 / 65.5);
		angular_motions[yaw] = 0.7 * angular_motions[yaw]
				+ ((float) Gyro_Z_RAW * 0.3 / 65.5);

		// Angle calculation using integration
		gyro_angle[roll] += (float) (Gyro_X_RAW / (250 * 65.5));
		gyro_angle[pitch] += (float) (Gyro_Y_RAW / (250 * 65.5));

		// Transfer roll to pitch if IMU has yawed
		gyro_angle[pitch] -= gyro_angle[roll]
				* sin((float) Gyro_Z_RAW * (M_PI / (250 * 65.5 * 180)));
		gyro_angle[roll] += gyro_angle[pitch]
				* sin((float) Gyro_Z_RAW * (M_PI / (250 * 65.5 * 180)));

		//Accelerometer angle calculations
		//acc_total_vector = sqrt(pow(Accel_X_RAW, 2) + pow(Accel_Y_RAW, 2) + pow(Accel_Z_RAW, 2));
		acc_total_vector = sqrt(pow(os_Ax, 2) + pow(os_Ay, 2) + pow(os_Az, 2));
		if (Accel_X_RAW > acc_total_vector)
			Accel_X_RAW = acc_total_vector; //Limit the maximum accelerometer value.
		if (Accel_X_RAW < -acc_total_vector)
			Accel_X_RAW = -acc_total_vector;
		if (Accel_Y_RAW > acc_total_vector)
			Accel_Y_RAW = acc_total_vector; //Limit the maximum accelerometer value.
		if (Accel_Y_RAW < -acc_total_vector)
			Accel_Y_RAW = -acc_total_vector;

		acc_angle[roll] = asin((float) Accel_X_RAW / acc_total_vector)
				* (-180 / M_PI); //Calculate the pitch angle.
		acc_angle[pitch] = asin((float) Accel_Y_RAW / acc_total_vector)
				* (180 / M_PI); //Calculate the roll angle.

		gyro_angle[roll] = gyro_angle[roll] * 0.995 + acc_angle[roll] * 0.005;
		gyro_angle[pitch] = gyro_angle[pitch] * 0.995
				+ acc_angle[pitch] * 0.005;

		// To dampen the pitch and roll angles a complementary filter is used
		measures[roll] = measures[roll] * 0.8 + gyro_angle[roll] * 0.2;
		measures[pitch] = measures[pitch] * 0.8 + gyro_angle[pitch] * 0.2;
		measures[yaw] = Gyro_Z_RAW / 65.5; //Store the angular motion for this axis

		pitch_adjust = measures[pitch] * 15;
		roll_adjust = measures[roll] * 15;

		pitch_angle = measures[pitch];
		roll_angle = measures[roll];

		//active
		if (diff3 < 1050 && diff4 < 1050)
			active = 1;
		if (active == 1 && diff3 < 1050 && diff4 > 1450) {
			active = 2;
			//When this is the first time.
			gyro_angle[roll] = acc_angle[roll];
			gyro_angle[pitch] = acc_angle[pitch];

			//Reset the PID controllers for a bumpless start.
			errors[0] = 0;
			errors[1] = 0;
			errors[2] = 0;
			delta_err[0] = 0;
			delta_err[1] = 0;
			delta_err[2] = 0;
			error_sum[0] = 0;
			error_sum[1] = 0;
			error_sum[2] = 0;
			previous_error[0] = 0;
			previous_error[1] = 0;
			previous_error[2] = 0;
			pid_roll = 0;
			pid_pitch = 0;
			pid_yaw = 0;
		}
		//Stopping the motors: throttle low and yaw right.
		if (active == 2 && diff3 < 1050 && diff4 > 1950)
			active = 0;

		pid_set_points[roll] = 0;
		pid_set_points[pitch] = 0;
		pid_set_points[yaw] = 0;

		if (diff1 > 1525)
			pid_set_points[roll] = diff1 - 1525;
		else if (diff1 < 1490)
			pid_set_points[roll] = diff1 - 1490;
		pid_set_points[roll] = (pid_set_points[roll] - roll_adjust) / 3.0;

		if (diff2 > 1525)
			pid_set_points[pitch] = diff2 - 1525;
		else if (diff2 < 1490)
			pid_set_points[pitch] = diff2 - 1490;
		pid_set_points[pitch] = (pid_set_points[pitch] - pitch_adjust) / 3.0;

		if (diff3 > 1150) {  //Do not yaw when turning off the motors.
			if (diff4 > 1550)
				pid_set_points[yaw] = (diff4 - 1550) / 3.0;
			else if (diff4 < 1350)
				pid_set_points[yaw] = (diff4 - 1350) / 3.0;
		}

		// Calculate current errors
		errors[roll] = angular_motions[roll] - pid_set_points[roll];
		errors[pitch] = angular_motions[pitch] - pid_set_points[pitch];
		errors[yaw] = angular_motions[yaw] - pid_set_points[yaw];

		// Calculate sum of errors : Integral coefficients
		error_sum[roll] += errors[roll];
		error_sum[pitch] += errors[pitch];
		error_sum[yaw] += errors[yaw];

		// Keep values in acceptable range
		if (error_sum[roll] < (-180 / Ki[roll]))
			error_sum[roll] = -180 / Ki[roll];
		if (error_sum[roll] > 180 / Ki[roll])
			error_sum[roll] = 180 / Ki[roll];
		if (error_sum[pitch] < (-180 / Ki[pitch]))
			error_sum[pitch] = -180 / Ki[pitch];
		if (error_sum[pitch] > 180 / Ki[pitch])
			error_sum[pitch] = 180 / Ki[pitch];
		if (error_sum[yaw] < (-180 / Ki[yaw]))
			error_sum[yaw] = -180 / Ki[yaw];
		if (error_sum[yaw] > 180 / Ki[yaw])
			error_sum[yaw] = 180 / Ki[yaw];

		// Calculate error delta : Derivative coefficients
		delta_err[roll] = angular_motions[roll] - pre_angular_motions[roll];
		delta_err[pitch] = angular_motions[pitch] - pre_angular_motions[pitch];
		delta_err[yaw] = angular_motions[yaw] - pre_angular_motions[yaw];

		// Save current error as previous_error for next time
		pre_angular_motions[roll] = angular_motions[roll];
		pre_angular_motions[pitch] = angular_motions[pitch];
		pre_angular_motions[yaw] = angular_motions[yaw];

		throttle = diff3;

		if (throttle > 1050) {
			// PID = e.Kp + ∫e.Ki.t + Δe.Kd/t
			pid_roll = (errors[roll] * Kp[roll])
					+ (error_sum[roll] * Ki[roll]) * 0.004
					+ (delta_err[roll] * Kd[roll]) / 0.004;
			pid_pitch = (errors[pitch] * Kp[pitch])
					+ (error_sum[pitch] * Ki[pitch]) * 0.004
					+ (delta_err[pitch] * Kd[pitch]) / 0.004;
			pid_yaw = (errors[yaw] * Kp[yaw])
					+ (error_sum[yaw] * Ki[yaw]) * 0.004
					+ (delta_err[yaw] * Kd[yaw]) / 0.004;

			if (pid_roll > pid_max[roll])
				pid_roll = pid_max[roll];
			if (pid_roll < (-pid_max[roll]))
				pid_roll = -pid_max[roll];
			if (pid_pitch > pid_max[pitch])
				pid_pitch = pid_max[pitch];
			if (pid_pitch < (-pid_max[pitch]))
				pid_pitch = -pid_max[pitch];
			if (pid_yaw > pid_max[yaw])
				pid_yaw = pid_max[yaw];
			if (pid_yaw < (-pid_max[yaw]))
				pid_yaw = -pid_max[yaw];
		}

		if (active == 2) {
			if (throttle > 1800)
				throttle = 1800;

			esc1 = throttle + pid_roll + pid_pitch + pid_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
			esc2 = throttle - pid_roll + pid_pitch - pid_yaw; //Calculate the pulse for esc 2 (rear-right - CW).
			esc3 = throttle - pid_roll - pid_pitch + pid_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
			esc4 = throttle + pid_roll - pid_pitch - pid_yaw; //Calculate the pulse for esc 4 (front-left - CW).

//			esc1 = update_esc1(esc1);
//			esc2 = update_esc1(esc2);
//			esc3 = update_esc1(esc3);
//			esc4 = update_esc1(esc4);

			if (esc1 < 1100)
				esc1 = 1100;  //Keep the motors running.
			if (esc2 < 1100)
				esc2 = 1100;  //Keep the motors running.
			if (esc3 < 1100)
				esc3 = 1100;  //Keep the motors running.
			if (esc4 < 1100)
				esc4 = 1100;  //Keep the motors running.

			if (esc1 > 2000)
				esc1 = 2000;  //Limit the esc-1 pulse to 2000us.
			if (esc2 > 2000)
				esc2 = 2000;  //Limit the esc-2 pulse to 2000us.
			if (esc3 > 2000)
				esc3 = 2000;  //Limit the esc-3 pulse to 2000us.
			if (esc4 > 2000)
				esc4 = 2000;  //Limit the esc-4 pulse to 2000us.
		} else {
			esc1 = 1000;
			esc2 = 1000;
			esc3 = 1000;
			esc4 = 1000;
		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, esc1);			//fr
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, esc4);			//fl
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, esc2);			//br
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, esc3);			//bl

		if (HAL_GetTick() - looptimer > 4) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);			//fr
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);			//fl
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);			//br
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);			//bl
		}
		while (HAL_GetTick() - looptimer < 4) {
		}
		looptimer = HAL_GetTick();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 5000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 72 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 5000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
