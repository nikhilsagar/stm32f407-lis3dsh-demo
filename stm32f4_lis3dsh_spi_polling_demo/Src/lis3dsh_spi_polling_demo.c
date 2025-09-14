/**
 * @file    msme_lis3dsh_demo_polling.c
 * @brief   STM32F4-DISCOVERY + LIS3DSH demo using our LIS3DSH driver (SPI, polling)
 *
 * @details
 *   Minimal bring-up and tilt demo on STM32F407G-DISC1 using a custom
 *   GPIO/SPI stack and a small LIS3DSH driver. Data is read in polling
 *   mode, smoothed with a moving average + 1st-order low-pass filter,
 *   converted to g-units, and mapped to LED indications for simple UX.
 *
 *   Board wiring (F4DISC defaults):
 *     - SPI1_SCK  = PA5 (AF5)
 *     - SPI1_MISO = PA6 (AF5)
 *     - SPI1_MOSI = PA7 (AF5)
 *     - MEMS_CS   = PE3  (GPIO output, idle HIGH)
 *
 *   Notes:
 *     - Many F4DISC + LIS3DSH boards prefer SPI mode 3 (CPOL=1, CPHA=1).
 *       Start with Mode 0; if reads look wrong (0xFF/0x00), switch to Mode 3.
 *     - SPI transfers use blocking/polling via spi_driver.h.
 *     - This demo uses single-precision floats; enable the FPU before use:
 *         SCB->CPACR |= (0xF << 20); __DSB(); __ISB();
 *
 * @author  You
 */

#include "STM32f407VGT.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "lis3dsh.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ================================ Config ================================== */

/** Output data rate for the sensor (must match APP_FS_HZ for filter math). */
#define APP_ODR               LIS3DSH_ODR_100HZ
/** Sensor anti-aliasing bandwidth. */
#define APP_BW                LIS3DSH_BW_50HZ
/** Sensor full-scale range. */
#define APP_FS                LIS3DSH_FS_2G

/** Sample rate used in filter design (Hz). Keep consistent with ODR. */
#define APP_FS_HZ             (100.0f)
/** 1st-order LPF cutoff frequency (Hz). */
#define APP_FC_HZ             (3.0f)
/** Moving-average window size (samples). */
#define AVG_SAMPLES           (8U)
/** Startup offset calibration samples (board must be still). */
#define CALIB_SAMPLES         (50U)
/** Data-ready polling guard to avoid lock-ups if sensor stops updating. */
#define STATUS_MAX_TRIES      (5000U)
/** Radians to degrees conversion constant. */
#define RAD2DEG               (57.2957795f)

/* LEDs (STM32F4DISC port/pin map) */
#define LED_PORT              GPIOD
#define LED_PIN_GREEN_LX      12U   /* LEFT  indicator  */
#define LED_PIN_ORANGE_UPY    13U   /* UP    indicator  */
#define LED_PIN_RED_RX        14U   /* RIGHT indicator  */
#define LED_PIN_BLUE_DOY      15U   /* DOWN  indicator  */

/* Chip select (CS) for LIS3DSH (manual CS, idle HIGH) */
#define SPI1_CS_PORT          GPIOE
#define SPI1_CS_PIN           3U

/* =============================== Local types ============================== */

/**
 * @brief Simple 1st-order low-pass filter (exponential smoothing).
 *        y[n] = y[n-1] + a * (x[n] - y[n-1])
 */
typedef struct {
	float a; /**< Filter coefficient (0..1), computed from fc & fs. */
	float y; /**< Filter state (previous output). */
} LPF1_t;

/* =========================== Private state/decls ========================== */

static SPI_Handle_t hspi1; /* SPI1 handle (polling) */
static LIS3DSH_Handle_t hlis; /* LIS3DSH driver handle */

static void Init_LEDs(void);
static void Init_SPI1_GPIO_CS(void);
static void Config_SPI1(void);
static void delay_cycles(volatile uint32_t n);

/* ------------------------------ Filters ---------------------------------- */

/**
 * @brief Compute LPF alpha from cutoff and sample rate.
 * @param fc_hz  Cutoff frequency (Hz).
 * @param fs_hz  Sampling frequency (Hz).
 * @return alpha in [0..1]. Higher a = faster response, less smoothing.
 */
static inline float lpf_alpha(float fc_hz, float fs_hz) {
	const float two_pi_fc = 6.28318530718f * fc_hz;
	return (two_pi_fc) / (two_pi_fc + fs_hz);
}

/**
 * @brief Initialize a 1st-order LPF.
 */
static inline void lpf_init(LPF1_t *f, float fc_hz, float fs_hz) {
	f->a = lpf_alpha(fc_hz, fs_hz);
	f->y = 0.0f;
}

/**
 * @brief Update a 1st-order LPF with new sample.
 * @return New filtered output.
 */
static inline float lpf_update(LPF1_t *f, float x) {
	f->y += f->a * (x - f->y);
	return f->y;
}

/**
 * @brief Running moving average of last AVG_SAMPLES raw readings (per axis).
 *        Integer-domain accumulation to keep cost low; returns float averages.
 *
 * @note  Uses static ring buffers (not re-entrant).
 */
static void add_sample(int16_t x, int16_t y, int16_t z, float *x_avg,
		float *y_avg, float *z_avg) {
	static int16_t bx[AVG_SAMPLES], by[AVG_SAMPLES], bz[AVG_SAMPLES];
	static uint8_t idx = 0;

	bx[idx] = x;
	by[idx] = y;
	bz[idx] = z;
	idx = (uint8_t) ((idx + 1U) % AVG_SAMPLES);

	int32_t sx = 0, sy = 0, sz = 0;
	for (uint32_t i = 0; i < AVG_SAMPLES; ++i) {
		sx += bx[i];
		sy += by[i];
		sz += bz[i];
	}

	*x_avg = (float) (sx / (int32_t) AVG_SAMPLES);
	*y_avg = (float) (sy / (int32_t) AVG_SAMPLES);
	*z_avg = (float) (sz / (int32_t) AVG_SAMPLES);
}

/* ================================ Demo ==================================== */

/**
 * @brief LIS3DSH SPI polling demo entry.
 *
 * Flow:
 *   1) Initialize LEDs and SPI (GPIO+AF+CS, then SPI1 config, enable SPE).
 *   2) Initialize LIS3DSH (WHO_AM_I check, enable XYZ, set ODR/BW/FS).
 *   3) Calibrate startup offsets (board held still).
 *   4) Loop: wait data-ready → burst read → MA → LPF → convert to g →
 *            clamp → axis-only tilt angles (deg) → print + LED UI.
 *
 * @return 0 on success (never returns in practice).
 */
int lis3dsh_spi_polling_demo_main(void) {
	/* -------- Bring-up -------- */
	Init_LEDs();
	Init_SPI1_GPIO_CS();
	Config_SPI1();
	SPI_PeripheralControl(&hspi1, ENABLE); /* SPE=1: enable SPI peripheral */

	/* -------- Sensor init (WHO_AM_I, XYZ enable, BDU=1, ODR/BW/FS) -------- */
	if (LIS3DSH_Init(&hlis, &hspi1, SPI1_CS_PORT, SPI1_CS_PIN,
	APP_ODR, APP_BW, APP_FS) < 0) {
		/* Visual error: blink RED forever */
		printf("ERR: LIS3DSH init failed\r\n");
		while (1) {
			GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 1);
			delay_cycles(800000);
			GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 0);
			delay_cycles(400000);
		}
	}
	printf("WHO_AM_I=0x%02X\r\n", (unsigned) LIS3DSH_WhoAmI(&hlis));

	/* -------- Filters (per-axis) -------- */
	LPF1_t fx, fy, fz;
	lpf_init(&fx, APP_FC_HZ, APP_FS_HZ);
	lpf_init(&fy, APP_FC_HZ, APP_FS_HZ);
	lpf_init(&fz, APP_FC_HZ, APP_FS_HZ);

	/* -------- Startup offset calibration (board still) -------- */
	float x_off = 0, y_off = 0, z_off = 0;
	for (uint32_t i = 0; i < CALIB_SAMPLES; ++i) {
		int16_t rx, ry, rz;
		LIS3DSH_ReadXYZ_Raw(&hlis, &rx, &ry, &rz);
		x_off += rx;
		y_off += ry;
		z_off += rz;
		delay_cycles(50000); /* gentle pacing */
	}
	x_off /= (float) CALIB_SAMPLES;
	y_off /= (float) CALIB_SAMPLES;
	z_off /= (float) CALIB_SAMPLES;

	/* -------- Main loop -------- */
	while (1) {
		/* Data-ready poll (with timeout as a safety net) */
		uint32_t tries = 0;
		while (!LIS3DSH_DataReady(&hlis) && (++tries < STATUS_MAX_TRIES)) {
			/* spin */
		}
		if (tries >= STATUS_MAX_TRIES) {
			/* If DR never sets, print status for debugging and continue. */
			printf("WARN: no data ready; WHO=0x%02X\r\n",
					(unsigned) LIS3DSH_WhoAmI(&hlis));
			continue;
		}

		/* Burst read raw XYZ (little-endian 16-bit each) */
		int16_t rx, ry, rz;
		LIS3DSH_ReadXYZ_Raw(&hlis, &rx, &ry, &rz);

		/* Moving average (reduces quantization + random noise) */
		float x_ma, y_ma, z_ma;
		add_sample(rx, ry, rz, &x_ma, &y_ma, &z_ma);

		/* Remove initial biases estimated at startup */
		float x = x_ma - x_off, y = y_ma - y_off, z = z_ma - z_off;

		/* 1st-order LPF (suppress residual noise/jerk) */
		float xf = lpf_update(&fx, x);
		float yf = lpf_update(&fy, y);
		float zf = lpf_update(&fz, z);

		/* Convert LSB → g (FS=±2g sensitivity: 0.06 mg/LSB) */
		const float LSB_TO_G = 0.00006f; /* 60 µg/LSB */
		float ax = xf * LSB_TO_G, ay = yf * LSB_TO_G, az = zf * LSB_TO_G;

		/* Clamp to [-1, 1] to keep asin() in domain */
		if (ax > 1)
			ax = 1;
		if (ax < -1)
			ax = -1;
		if (ay > 1)
			ay = 1;
		if (ay < -1)
			ay = -1;
		if (az > 1)
			az = 1;
		if (az < -1)
			az = -1;

		/* Axis-only tilt angles (deg). For simple UI (not full 3D orientation). */
		float x_deg = asinf(ax) * RAD2DEG;
		float y_deg = asinf(ay) * RAD2DEG;
		float z_deg = asinf(az) * RAD2DEG;

		printf("Axis-only ang  X:%8.2f  Y:%8.2f  Z:%8.2f deg\r\n", x_deg, y_deg,
				z_deg);

		/* ----------------------------- LED UI ------------------------------
		 * PD12 = GREEN  = LEFT  (X < -15)
		 * PD14 = RED    = RIGHT (X > +15)
		 * PD13 = ORANGE = UP    (Y > +15)
		 * PD15 = BLUE   = DOWN  (Y < -15)
		 * ------------------------------------------------------------------ */

		/* X-axis LEFT/RIGHT with deadband around 0 deg */
		if (x_deg > 15.0f) {
			GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 1);
			GPIO_WritePin(LED_PORT, LED_PIN_GREEN_LX, 0);
		} else if (x_deg < -15.0f) {
			GPIO_WritePin(LED_PORT, LED_PIN_GREEN_LX, 1);
			GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 0);
		} else {
			GPIO_WritePin(LED_PORT, LED_PIN_GREEN_LX, 0);
			GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 0);
		}

		/* Y-axis UP/DOWN with deadband */
		if (y_deg > 15.0f) {
			GPIO_WritePin(LED_PORT, LED_PIN_ORANGE_UPY, 1);
			GPIO_WritePin(LED_PORT, LED_PIN_BLUE_DOY, 0);
		} else if (y_deg < -15.0f) {
			GPIO_WritePin(LED_PORT, LED_PIN_BLUE_DOY, 1);
			GPIO_WritePin(LED_PORT, LED_PIN_ORANGE_UPY, 0);
		} else {
			GPIO_WritePin(LED_PORT, LED_PIN_ORANGE_UPY, 0);
			GPIO_WritePin(LED_PORT, LED_PIN_BLUE_DOY, 0);
		}
	}
}

/* =============================== Peripherals ============================== */

/**
 * @brief Configure LED pins on GPIOD as push-pull outputs (default LOW).
 */
static void Init_LEDs(void) {
	GPIO_EnableClock(GPIO_PORT_D, ENABLE);

	GPIO_Config_t led = { 0 };
	led.Mode = GPIO_MODE_OUTPUT;
	led.OutputType = GPIO_OTYPE_PP;
	led.PuPd = GPIO_PULLDOWN;

	GPIO_Init(LED_PORT, LED_PIN_GREEN_LX, &led);
	GPIO_Init(LED_PORT, LED_PIN_ORANGE_UPY, &led);
	GPIO_Init(LED_PORT, LED_PIN_RED_RX, &led);
	GPIO_Init(LED_PORT, LED_PIN_BLUE_DOY, &led);

	GPIO_OSpeed(LED_PORT, LED_PIN_GREEN_LX, GPIO_SPEED_LOW);
	GPIO_OSpeed(LED_PORT, LED_PIN_ORANGE_UPY, GPIO_SPEED_LOW);
	GPIO_OSpeed(LED_PORT, LED_PIN_RED_RX, GPIO_SPEED_LOW);
	GPIO_OSpeed(LED_PORT, LED_PIN_BLUE_DOY, GPIO_SPEED_LOW);

	GPIO_WritePin(LED_PORT, LED_PIN_GREEN_LX, 0);
	GPIO_WritePin(LED_PORT, LED_PIN_ORANGE_UPY, 0);
	GPIO_WritePin(LED_PORT, LED_PIN_RED_RX, 0);
	GPIO_WritePin(LED_PORT, LED_PIN_BLUE_DOY, 0);
}

/**
 * @brief Configure SPI1 pins (PA5/6/7 as AF5) and manual CS on PE3 (idle HIGH).
 */
static void Init_SPI1_GPIO_CS(void) {
	/* SPI pins */
	GPIO_EnableClock(GPIO_PORT_A, ENABLE);

	GPIO_Config_t af = { 0 };
	af.Mode = GPIO_MODE_ALT;
	af.OutputType = GPIO_OTYPE_PP;
	af.PuPd = GPIO_NOPULL;
	af.AltFunction = 5; /* AF5 = SPI1 */

	/* High speed to keep clean edges at higher SPI clocks. */
	GPIO_OSpeed(GPIOA, 5U, GPIO_SPEED_HIGH);
	GPIO_OSpeed(GPIOA, 6U, GPIO_SPEED_HIGH);
	GPIO_OSpeed(GPIOA, 7U, GPIO_SPEED_HIGH);

	GPIO_Init(GPIOA, 7U, &af); /* MOSI */
	GPIO_Init(GPIOA, 6U, &af); /* MISO */
	af.PuPd = GPIO_PULLDOWN; /* Optional pad idle on SCK */
	GPIO_Init(GPIOA, 5U, &af); /* SCK  */

	/* CS pin (manual control, active LOW) */
	GPIO_EnableClock(GPIO_PORT_E, ENABLE);
	GPIO_Config_t cs = { 0 };
	cs.Mode = GPIO_MODE_OUTPUT;
	cs.OutputType = GPIO_OTYPE_PP;
	cs.PuPd = GPIO_PULLUP;

	GPIO_OSpeed(GPIOE, SPI1_CS_PIN, GPIO_SPEED_HIGH);
	GPIO_Init(GPIOE, SPI1_CS_PIN, &cs);
	GPIO_WritePin(GPIOE, SPI1_CS_PIN, 1); /* idle HIGH */
}

/**
 * @brief Configure SPI1 peripheral (master, full-duplex, 8-bit, Mode 0 initially).
 *
 * @note If reads look wrong (all 0xFF or 0x00), switch to Mode 3:
 *       cpol = SPI_CPOL_HIGH; cpha = SPI_CPHA_SECOND;
 */
static void Config_SPI1(void) {
	hspi1.SPIx = SPI1;
	hspi1.config.deviceMode = SPI_MODE_MASTER;
	hspi1.config.busConfig = SPI_BUS_FULL_DUPLEX;
	hspi1.config.dff = SPI_DFF_8BITS;

	/* Start with Mode 0; many LIS3DSH boards prefer Mode 3. */
	hspi1.config.cpol = SPI_CPOL_LOW;
	hspi1.config.cpha = SPI_CPHA_FIRST;

	/* Software NSS (SSM=1) with SSI=1 to avoid MODF. */
	hspi1.config.ssm = SPI_SSM_ENABLE;

	/* Start slow (BR=/256). Increase after basic comms verified. */
	hspi1.config.clkSpeed = 7;

	/* Enable peripheral clock and program CR1. */
	RCC->APB2ENR |= (1U << 12); /* SPI1EN */
	SPI_Init(&hspi1);

	/* With SSM=1, set SSI=1 so NSS is seen high internally. */
	hspi1.SPIx->CR1 |= (1U << 8);
}

/**
 * @brief Small cycle-burning delay (portable NOP loop).
 * @param n  Number of iterations (approximate).
 */
static void delay_cycles(volatile uint32_t n) {
	while (n--) {
		__asm__("nop");
	}
}
