/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
#include "sensors/sensor_if.h"
#include "sensors/dps310.h"
#include "sensors/icm42688.h"
#include "sensors/gps6m.h"
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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static sensor_if_t if_dps, if_icm, if_gps;
static dps310_t    dps;
static icm42688_t  icm;
static gps6m_t 	   gps;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scan(I2C_HandleTypeDef *hi2c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
	float x;          // altitude (m)
	float v;          // vertical speed (m/s)
	float P00, P01, P10, P11;  // covariance
} AltKF;

static void altkf_init(AltKF *kf, float x0) {
	kf->x = x0; kf->v = 0.0f;
	kf->P00 = 10.0f; kf->P01 = 0.0f;
	kf->P10 = 0.0f;  kf->P11 = 10.0f;
}

static void altkf_predict(AltKF *kf, float dt, float Q_pos, float Q_vel) {
	// x = x + v*dt
	kf->x += kf->v * dt;
	// v = v
	// P = F P F' + Q, F = [[1 dt],[0 1]]
	float P00 = kf->P00 + dt*(kf->P10 + kf->P01) + dt*dt*kf->P11 + Q_pos;
	float P01 = kf->P01 + dt*kf->P11;
	float P10 = kf->P10 + dt*kf->P11;
	float P11 = kf->P11 + Q_vel;
	kf->P00=P00; kf->P01=P01; kf->P10=P10; kf->P11=P11;
}

static void altkf_update_scalar(AltKF *kf, float z, float R) {
	// H = [1 0]
	float y  = z - kf->x;
	float S  = kf->P00 + R;
	float K0 = kf->P00 / S;
	float K1 = kf->P10 / S;
	kf->x += K0 * y;
	kf->v += K1 * y;
	float P00 = (1.0f - K0) * kf->P00;
	float P01 = (1.0f - K0) * kf->P01;
	float P10 = kf->P10 - K1 * kf->P00;
	float P11 = kf->P11 - K1 * kf->P01;
	kf->P00=P00; kf->P01=P01; kf->P10=P10; kf->P11=P11;
}

/* Simple EMA smoother for GPS */
typedef struct {
	int    seeded;
	double lat_deg_f;
	double lon_deg_f;
	float  spd_mps_f;
	float  course_deg_f;
} GpsEMA;

static void gps_ema_reset(GpsEMA *f) { memset(f, 0, sizeof(*f)); }
static void gps_ema_update(GpsEMA *f, const gps6m_t *g, float alpha_latlon, float alpha_spd) {
	if (!f->seeded) {
		f->lat_deg_f = g->lat_deg;
		f->lon_deg_f = g->lon_deg;
		f->spd_mps_f = g->speed_mps;
		f->course_deg_f = g->course_deg;
		f->seeded = 1;
		return;
	}
	f->lat_deg_f = f->lat_deg_f + alpha_latlon * (g->lat_deg - f->lat_deg_f);
	f->lon_deg_f = f->lon_deg_f + alpha_latlon * (g->lon_deg - f->lon_deg_f);
	f->spd_mps_f = f->spd_mps_f + alpha_spd    * (g->speed_mps - f->spd_mps_f);
	// course is meaningless at low speed, keep last when speed < 0.5 m/s
	if (g->speed_mps > 0.5f) {
		float e = g->course_deg - f->course_deg_f;
		// wrap to [-180,180]
		while (e > 180.0f) e -= 360.0f;
		while (e < -180.0f) e += 360.0f;
		f->course_deg_f += alpha_spd * e;
		// wrap back to [0,360)
		if (f->course_deg_f < 0.0f)  f->course_deg_f += 360.0f;
		if (f->course_deg_f >= 360.0f) f->course_deg_f -= 360.0f;
	}
}

/* Baro <-> QNH helper: compute sea-level pressure P0 from local P,z */
static float baro_compute_P0(float P_pa, float alt_m) {
	// Standard atmosphere: P = P0 * (1 - L*h/T0)^(g*M/R/L)
	// Invert for P0. Constants for ISA:
	const float T0 = 288.15f;     // K
	const float L  = 0.0065f;     // K/m
	const float g0 = 9.80665f;    // m/s^2
	const float R  = 287.053f;    // J/(kg*K)
	const float expn = g0/(R*L);  // ~5.25588
	float ratio = 1.0f - (L*alt_m)/T0;
	if (ratio <= 0.0f) ratio = 0.0001f;
	float P0 = P_pa / powf(ratio, expn);
	return P0;  // Pa
}

/* Keep global filter state */
static AltKF  altkf;
static GpsEMA gpsf;
static uint8_t filters_seeded = 0;

/* One-step function you asked for: reads sensors, updates filters, and prints */
static void sensors_step(float dt_s,
		float Q_pos, float Q_vel,
		float R_baro, float R_gps,
		float ema_latlon_alpha, float ema_spd_alpha,
		float *alt_fused_out) {
	// 1) Read all devices (non-blocking reads)
	if_dps.vTable->read(&if_dps);
	if_icm.vTable->read(&if_icm);

	if (HAL_GetTick() - gps.last_update_ms > 150) {
		uint32_t t_end = HAL_GetTick() + 60;         // ~60 ms budget

		do {
			if_gps.vTable->read(&if_gps);
			if (HAL_GetTick() - gps.last_update_ms <= 50) break; // fresh enough
		} while ((int32_t)(HAL_GetTick() - t_end) < 0);

	} else {
		// still give it a light read so we don't fall behind
		if_gps.vTable->read(&if_gps);
	}

	// 2) Seed filters once we have a GPS fix
	if (!filters_seeded && gps.has_fix) {
		altkf_init(&altkf, gps.alt_m);  // start from GPS altitude
		gps_ema_reset(&gpsf);
		filters_seeded = 1;
	}

	// 3) Predict step for altitude KF
	altkf_predict(&altkf, dt_s, Q_pos, Q_vel);

	// 4) Baro absolute altitude, using your current DPS altitude.
	// If your dps.altitude already uses a good P0, this is fine.
	// Otherwise, compute dps.altitude yourself from pressure and a calibrated P0.
	float z_baro = (float)dps.altitude;

	// 5) Update with baro at high rate
	altkf_update_scalar(&altkf, z_baro, R_baro);

	// 6) If a fresh GPS fix is present, update with GPS altitude as absolute correction
	if (gps.has_fix) {
		altkf_update_scalar(&altkf, (float)gps.alt_m, R_gps);
		gps_ema_update(&gpsf, &gps, ema_latlon_alpha, ema_spd_alpha);
	}

	if (alt_fused_out) *alt_fused_out = altkf.x;

	// 7) Print once per call for visibility
	printf("ICM temp: %.2f C\r\n", icm.last_temp_c);
	printf("ICM Accel (g): [%.2f, %.2f, %.2f]\r\n", (double)icm.accel_g[0], (double)icm.accel_g[1], (double)icm.accel_g[2]);
	printf("ICM Gyro  (dps): [%.2f, %.2f, %.2f]\r\n", (double)icm.gyro_dps[0], (double)icm.gyro_dps[1], (double)icm.gyro_dps[2]);

	printf("DPS Pressure: %.2f Pa   Temp: %.2f C   Alt_raw: %.2f m\r\n",
			(double)dps.pressure, (double)dps.temperature, (double)dps.altitude);

	if (gps.has_fix) {
		printf("[gps] FIX sats=%u hdop=%.2f lat=%.6f lon=%.6f alt=%.1f m spd=%.2f m/s cog=%.1f utc=%06lu\r\n",
				gps.sats_used, gps.hdop, gpsf.lat_deg_f, gpsf.lon_deg_f, gps.alt_m,
				gpsf.spd_mps_f, gpsf.course_deg_f, (unsigned long)gps.utc_hms);
	} else {
		printf("[gps] NO FIX, sats=%u hdop=%.2f\r\n", gps.sats_used, gps.hdop);
	}

	printf("[fuse] alt_fused=%.2f m  v_z=%.2f m/s\r\n", (double)altkf.x, (double)altkf.v);
}
/* ========= END USER FILTERS ========= */


void I2C_Scan(I2C_HandleTypeDef *hi2c) {
	char msg[40];
	int found = 0;
	for (uint16_t addr = 0; addr < 128; addr++) {
		if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 3, 10) == HAL_OK) {
			int n = snprintf(msg, sizeof(msg), "Found device at 0x%02X\r\n", addr);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)n, HAL_MAX_DELAY);
			found = 1;
		}
	}
	if (!found) {
		const char *s = "No device found\r\n";
		HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_USART3_UART_Init();
	MX_I2C2_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("BOOT\r\n");

	HAL_Delay(50);   // let sensors power up

	// Optional quick sanity scan
	// I2C_Scan(&hi2c1);
	// I2C_Scan(&hi2c2);

	// Bind drivers to their buses/addresses (adjust to your wiring)
	dps310_bind(&if_dps, &dps, &hi2c2, DPS_I2C_ADDRESS);  // 0x77 by default, 0x76 if SDO low
	icm42688_bind(&if_icm, &icm, &hi2c1, ICM_ADDR_7B);  // 0x68 or 0x69 depending on AD0
	gps6m_bind(&if_gps, &gps, &huart6);

	HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

	//	if (if_dps.vTable->probe(&if_dps) != HAL_OK)  printf("DPS310 probe FAIL\r\n");
	//	else                                      printf("DPS310 probe OK\r\n");
	//
	//	HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
	//
	//	if (if_icm.vTable->probe(&if_icm) != HAL_OK)  printf("ICM42688 probe FAIL\r\n");
	//	else                                      printf("ICM42688 probe OK\r\n");
	//
	//	HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);
	//
	//	if (if_gps.vTable->probe(&if_gps) != HAL_OK)  printf("GPS probe FAIL\r\n");
	//	else                                      printf("GPS probe OK\r\n");
	//
	//	HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

	// Init devices (config, coeffs, etc. — your driver stubs can be expanded later)
	HAL_StatusTypeDef response;
	response = if_icm.vTable->init(&if_icm);
	if (response != HAL_OK) while(1);

	HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

	response =if_dps.vTable->init(&if_dps);
	if (response != HAL_OK) while(1);

	HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);

	response = if_gps.vTable->probe(&if_gps);
	if (response != HAL_OK) while(1);

	response =if_gps.vTable->init(&if_gps);
	if (response != HAL_OK) while(1);

	HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t t_prev = HAL_GetTick();
	while (1)
	{
		uint32_t t_now = HAL_GetTick();
		float dt_s = (t_now - t_prev) * 0.001f;
		if (dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.01f; // guard
		t_prev = t_now;

		// Tuning knobs:
		const float Q_pos = 0.05f;    // process noise on position, m^2 per step
		const float Q_vel = 0.5f;     // process noise on velocity
		const float R_baro = 1.0f;    // baro meas var (m^2)
		const float R_gps  = 16.0f;   // gps meas var (m^2) ~4 m std^2

		const float EMA_LATLON = 0.15f; // 0..1, higher = less smoothing
		const float EMA_SPD    = 0.25f;

		float alt_fused = 0.0f;
		sensors_step(dt_s, Q_pos, Q_vel, R_baro, R_gps, EMA_LATLON, EMA_SPD, &alt_fused);

		// Example: light heartbeat and 10 Hz loop
		HAL_GPIO_TogglePin(Green_LED_GPIO_Port, Green_LED_Pin);
		HAL_Delay(100);


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10707DBC;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10707DBC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */
	extern void retarget_init(UART_HandleTypeDef *huart);
	retarget_init(&huart3);
	printf("boot ok\n");
	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
	huart6.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|Yellow_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Green_LED_Pin Yellow_LED_Pin */
	GPIO_InitStruct.Pin = Green_LED_Pin|Yellow_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Red_LED_Pin */
	GPIO_InitStruct.Pin = Red_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Red_LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
