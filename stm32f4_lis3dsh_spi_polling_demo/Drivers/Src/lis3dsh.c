/**
 * @file    lis3dsh.c
 * @brief   LIS3DSH accelerometer driver (SPI, blocking)
 */

#include "lis3dsh.h"
#include <stddef.h>

/* ============================= Local helpers ============================== */

#define READ_CMD(addr)   (uint8_t)(0x80U | ((addr) & 0x7FU))
#define WRITE_CMD(addr)  (uint8_t)((addr) & 0x7FU)

static inline void cs_low(LIS3DSH_Handle_t *h) {
	GPIO_WritePin(h->cs_port, h->cs_pin, 0);
}
static inline void cs_high(LIS3DSH_Handle_t *h) {
	GPIO_WritePin(h->cs_port, h->cs_pin, 1);
}

/* mg/LSB sensitivities for each FS, from datasheet (typical) */
static float fs_mg_per_lsb(LIS3DSH_FS_t fs) {
	switch (fs) {
	case LIS3DSH_FS_2G:
		return 0.06f; /* 0.06 mg/LSB */
	case LIS3DSH_FS_4G:
		return 0.12f;
	case LIS3DSH_FS_6G:
		return 0.18f;
	case LIS3DSH_FS_8G:
		return 0.24f;
	case LIS3DSH_FS_16G:
		return 0.73f; /* note larger step at 16g */
	default:
		return 0.06f;
	}
}

/* ============================ Low-level I/O =============================== */

uint8_t LIS3DSH_ReadReg(LIS3DSH_Handle_t *h, uint8_t reg) {
	uint8_t tx = READ_CMD(reg);
	uint8_t rx = 0, dummy = 0x00;

	cs_low(h);
	(void) SPI_TransmitReceive(h->spi, &tx, &rx, 1);
	(void) SPI_TransmitReceive(h->spi, &dummy, &rx, 1);
	cs_high(h);
	return rx;
}

void LIS3DSH_WriteReg(LIS3DSH_Handle_t *h, uint8_t reg, uint8_t val) {
	uint8_t tx = WRITE_CMD(reg);
	uint8_t rx = 0;

	cs_low(h);
	(void) SPI_TransmitReceive(h->spi, &tx, &rx, 1);
	(void) SPI_TransmitReceive(h->spi, &val, &rx, 1);
	cs_high(h);
}

void LIS3DSH_ReadMulti(LIS3DSH_Handle_t *h, uint8_t start_reg, uint8_t *buf,
		uint8_t len) {
	uint8_t tx = READ_CMD(start_reg);
	uint8_t rx = 0, dummy = 0x00;

	cs_low(h);
	(void) SPI_TransmitReceive(h->spi, &tx, &rx, 1);
	for (uint8_t i = 0; i < len; ++i) {
		(void) SPI_TransmitReceive(h->spi, &dummy, &buf[i], 1);
	}
	cs_high(h);
}

/* ============================== Public API ================================ */

uint8_t LIS3DSH_WhoAmI(LIS3DSH_Handle_t *h) {
	return LIS3DSH_ReadReg(h, LIS3DSH_REG_WHO_AM_I);
}

int LIS3DSH_Init(LIS3DSH_Handle_t *h, SPI_Handle_t *spi, GPIO_RegDef_t *cs_port,
		uint8_t cs_pin, LIS3DSH_ODR_t odr, LIS3DSH_BW_t bw, LIS3DSH_FS_t fs) {
	if ((h == NULL) || (spi == NULL) || (cs_port == NULL)) {
		return -1;
	}

	h->spi = spi;
	h->cs_port = cs_port;
	h->cs_pin = cs_pin;
	h->fs = fs;

	/* WHO_AM_I */
	uint8_t who = LIS3DSH_WhoAmI(h);
	if (who != LIS3DSH_WHOAMI_VAL) {
		return -2;
	}

	/* CTRL4: ODR | BDU=1 | XYZ enable */
	h->ctrl4 = ((uint8_t) odr) | LIS3DSH_CTRL4_BDU | LIS3DSH_CTRL4_XEN
			| LIS3DSH_CTRL4_YEN | LIS3DSH_CTRL4_ZEN;
	LIS3DSH_WriteReg(h, LIS3DSH_REG_CTRL4, h->ctrl4);

	/* CTRL5: BW, FS, 4-wire SPI (SIM=0) */
	uint8_t bw_bits = ((uint8_t) bw & 0x03U) << LIS3DSH_CTRL5_BW_Pos;
	uint8_t fs_bits = ((uint8_t) fs & 0x07U) << LIS3DSH_CTRL5_FS_Pos;
	h->ctrl5 = (uint8_t) (bw_bits | fs_bits); /* ST bits=0, SIM=0 */
	LIS3DSH_WriteReg(h, LIS3DSH_REG_CTRL5, h->ctrl5);

	return 0;
}

uint8_t LIS3DSH_DataReady(LIS3DSH_Handle_t *h) {
	uint8_t s = LIS3DSH_ReadReg(h, LIS3DSH_REG_STATUS);
	return (uint8_t) ((s & LIS3DSH_STATUS_ZYXDA_Msk) ? 1U : 0U);
}

static inline int16_t pack_le16(uint8_t lo, uint8_t hi) {
	return (int16_t) ((((uint16_t) hi) << 8) | lo);
}

void LIS3DSH_ReadXYZ_Raw(LIS3DSH_Handle_t *h, int16_t *x, int16_t *y,
		int16_t *z) {
	uint8_t buf[6];
	LIS3DSH_ReadMulti(h, LIS3DSH_REG_OUT_X_L, buf, 6);
	if (x)
		*x = pack_le16(buf[0], buf[1]);
	if (y)
		*y = pack_le16(buf[2], buf[3]);
	if (z)
		*z = pack_le16(buf[4], buf[5]);
}

void LIS3DSH_ReadXYZ_mg(LIS3DSH_Handle_t *h, float *x_mg, float *y_mg,
		float *z_mg) {
	int16_t xr, yr, zr;
	LIS3DSH_ReadXYZ_Raw(h, &xr, &yr, &zr);

	const float k = fs_mg_per_lsb(h->fs);
	if (x_mg)
		*x_mg = (float) xr * k;
	if (y_mg)
		*y_mg = (float) yr * k;
	if (z_mg)
		*z_mg = (float) zr * k;
}

int LIS3DSH_SetODR(LIS3DSH_Handle_t *h, LIS3DSH_ODR_t odr) {
	h->ctrl4 &= (uint8_t) ~0xF0U; /* clear ODR bits */
	h->ctrl4 |= (uint8_t) odr; /* set new ODR */
	LIS3DSH_WriteReg(h, LIS3DSH_REG_CTRL4, h->ctrl4);
	return 0;
}

int LIS3DSH_SetAAFilterBW(LIS3DSH_Handle_t *h, LIS3DSH_BW_t bw) {
	h->ctrl5 &= (uint8_t) ~(0x03U << LIS3DSH_CTRL5_BW_Pos);
	h->ctrl5 |= (uint8_t) (((uint8_t) bw & 0x03U) << LIS3DSH_CTRL5_BW_Pos);
	LIS3DSH_WriteReg(h, LIS3DSH_REG_CTRL5, h->ctrl5);
	return 0;
}

int LIS3DSH_SetFullScale(LIS3DSH_Handle_t *h, LIS3DSH_FS_t fs) {
	h->ctrl5 &= (uint8_t) ~(0x07U << LIS3DSH_CTRL5_FS_Pos);
	h->ctrl5 |= (uint8_t) (((uint8_t) fs & 0x07U) << LIS3DSH_CTRL5_FS_Pos);
	LIS3DSH_WriteReg(h, LIS3DSH_REG_CTRL5, h->ctrl5);
	h->fs = fs;
	return 0;
}
