#ifndef QMI8658_H
#define QMI8658_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "qmi8658_register.h"

#if 1
#define qmi8658_log printf
#else
#define qmi8658_log(...)
#endif

#define QMI8658_CALI_DATA_NUM 200

typedef struct qmi8658_cali
{
	float acc_last[3];
	float acc[3];
	float acc_fix[3];
	float acc_bias[3];
	float acc_sum[3];

	float gyr_last[3];
	float gyr[3];
	float gyr_fix[3];
	float gyr_bias[3];
	float gyr_sum[3];

	unsigned char imu_static_flag;
	unsigned char acc_fix_flag;
	unsigned char gyr_fix_flag;
	char acc_fix_index;
	unsigned char gyr_fix_index;

	unsigned char acc_cali_flag;
	unsigned char gyr_cali_flag;
	unsigned short acc_cali_num;
	unsigned short gyr_cali_num;
	//    unsigned char	acc_avg_num;
	//    unsigned char	gyr_avg_num;
} qmi8658_cali;

typedef struct
{
	unsigned char enSensors;
	enum qmi8658_AccRange accRange;
	enum qmi8658_AccOdr accOdr;
	enum qmi8658_GyrRange gyrRange;
	enum qmi8658_GyrOdr gyrOdr;
	unsigned char ctrl8_value;
	unsigned char syncSample;
#if defined(QMI8658_USE_FIFO)
	unsigned char fifo_ctrl;
#endif
} qmi8658_config;

typedef struct
{
	unsigned char slave;
	qmi8658_config cfg;
	unsigned short ssvt_a;
	unsigned short ssvt_g;
	unsigned int timestamp;
	unsigned int step;
	float imu[6];
	unsigned char cod_data[6];
	float st_out[6];
} qmi8658_state;

int qmi8658_hw_init(void);

int qmi8658_i2c_write(uint16_t address, uint8_t *p_data, uint16_t size);

int qmi8658_i2c_read(uint16_t address, uint8_t *p_data, uint16_t size);

void qmi8658_delay_ms(unsigned int ms);

void qmi8658_delay_us(unsigned int us);

int qmi8658_init(void);

void qmi8658_deinit(void);

float qmi8658_read_temp(void);

void qmi8658_read_timestamp(unsigned int *tim_count);

void qmi8658_read_xyz(float acc[3], float gyro[3]);

extern int qmi8658_send_ctl9cmd(enum qmi8658_Ctrl9Command cmd);

#endif
