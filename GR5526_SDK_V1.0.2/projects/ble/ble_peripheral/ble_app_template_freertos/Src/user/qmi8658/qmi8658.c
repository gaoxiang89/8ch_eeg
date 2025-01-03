

#include "qmi8658.h"

// #define QMI8658_UINT_MG_DPS
#define M_PI (3.14159265358979323846f)
#define ONE_G (9.807f)
#define QFABS(x) (((x) < 0.0f) ? (-1.0f * (x)) : (x))

static int qmi8658_write_reg(unsigned char reg, unsigned char value);
static int qmi8658_read_reg(unsigned char reg, unsigned char *buf, unsigned short len);
static void qmi8658_soft_reset(void);
static void qmi8658_get_chip_info(void);
static void qmi8658_config_reg(unsigned char low_power);
static void qmi8658_enableSensors(unsigned char enableFlags);
static void qmi8658_read_sensor_data(float acc[3], float gyro[3]);
static void qmi8658_axis_convert(short data_a[3], short data_g[3], int layout);

	static qmi8658_state g_imu;

static int qmi8658_write_reg(unsigned char reg, unsigned char value)
{
	int ret = 0;
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = value;

	ret = qmi8658_i2c_write(g_imu.slave, buf, 2);

	return ret;
}

static int qmi8658_read_reg(unsigned char reg, unsigned char *buf, unsigned short len)
{
	int ret = 0;

	ret = qmi8658_i2c_write(g_imu.slave, &reg, 1);
	if (ret < 0)
	{
		qmi8658_log("read reg failed, write return %d \r\n", ret);
	}

	ret = qmi8658_i2c_read(g_imu.slave , buf, len);
	if (ret < 0)
	{
		qmi8658_log("read reg failed, read return %d \r\n", ret);
	}

	return ret;
}


static void qmi8658_axis_convert(short data_a[3], short data_g[3], int layout)
{
	short raw[3], raw_g[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];
	raw_g[0] = data_g[0];
	raw_g[1] = data_g[1];

	if (layout >= 4 && layout <= 7)
	{
		data_a[2] = -data_a[2];
		data_g[2] = -data_g[2];
	}

	if (layout % 2)
	{
		data_a[0] = raw[1];
		data_a[1] = raw[0];

		data_g[0] = raw_g[1];
		data_g[1] = raw_g[0];
	}
	else
	{
		data_a[0] = raw[0];
		data_a[1] = raw[1];

		data_g[0] = raw_g[0];
		data_g[1] = raw_g[1];
	}

	if ((layout == 1) || (layout == 2) || (layout == 4) || (layout == 7))
	{
		data_a[0] = -data_a[0];
		data_g[0] = -data_g[0];
	}
	if ((layout == 2) || (layout == 3) || (layout == 6) || (layout == 7))
	{
		data_a[1] = -data_a[1];
		data_g[1] = -data_g[1];
	}
}

void qmi8658_config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	unsigned char ctl_dada;

	switch (range)
	{
	case Qmi8658AccRange_2g:
		g_imu.ssvt_a = (1 << 14);
		break;
	case Qmi8658AccRange_4g:
		g_imu.ssvt_a = (1 << 13);
		break;
	case Qmi8658AccRange_8g:
		g_imu.ssvt_a = (1 << 12);
		break;
	case Qmi8658AccRange_16g:
		g_imu.ssvt_a = (1 << 11);
		break;
	default:
		range = Qmi8658AccRange_8g;
		g_imu.ssvt_a = (1 << 12);
	}
	if (stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;

	qmi8658_write_reg(Qmi8658Register_Ctrl2, ctl_dada);
	// set LPF & HPF
	qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada, 1);
	ctl_dada &= 0xf0;
	if (lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= A_LSP_MODE_3;
		ctl_dada |= 0x01;
	}
	else
	{
		ctl_dada &= ~0x01;
	}
	// ctl_dada = 0x00;
	qmi8658_write_reg(Qmi8658Register_Ctrl5, ctl_dada);
	// set LPF & HPF
}

void qmi8658_config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada;

	// Store the scale factor for use when processing raw data
	switch (range)
	{
	case Qmi8658GyrRange_16dps:
		g_imu.ssvt_g = 2048;
		break;
	case Qmi8658GyrRange_32dps:
		g_imu.ssvt_g = 1024;
		break;
	case Qmi8658GyrRange_64dps:
		g_imu.ssvt_g = 512;
		break;
	case Qmi8658GyrRange_128dps:
		g_imu.ssvt_g = 256;
		break;
	case Qmi8658GyrRange_256dps:
		g_imu.ssvt_g = 128;
		break;
	case Qmi8658GyrRange_512dps:
		g_imu.ssvt_g = 64;
		break;
	case Qmi8658GyrRange_1024dps:
		g_imu.ssvt_g = 32;
		break;
	case Qmi8658GyrRange_2048dps:
		g_imu.ssvt_g = 16;
		break;

	default:
		range = Qmi8658GyrRange_512dps;
		g_imu.ssvt_g = 64;
		break;
	}

	if (stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;
	qmi8658_write_reg(Qmi8658Register_Ctrl3, ctl_dada);

	// Conversion from degrees/s to rad/s if necessary
	// set LPF & HPF
	qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada, 1);
	ctl_dada &= 0x0f;
	if (lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= G_LSP_MODE_3;
		ctl_dada |= 0x10;
	}
	else
	{
		ctl_dada &= ~0x10;
	}
	// ctl_dada = 0x00;
	qmi8658_write_reg(Qmi8658Register_Ctrl5, ctl_dada);
	// set LPF & HPF
}

int qmi8658_send_ctl9cmd(enum qmi8658_Ctrl9Command cmd)
{
	unsigned char status1 = 0x00;
	unsigned short count = 0;
	unsigned char status_reg = Qmi8658Register_StatusInt;
	unsigned char cmd_done = 0x80;
	unsigned char retry = 0;
	int ret1 = 0;
	int ret2 = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
	if (g_imu.cfg.syncSample == 1)
	{
		status_reg = Qmi8658Register_Status1;
		cmd_done = 0x01;
	}
#endif
	while (retry++ < 3)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)cmd); // write commond to ctrl9

		qmi8658_read_reg(status_reg, &status1, 1);
		while (((status1 & cmd_done) != cmd_done) && (count++ < 100)) // read statusINT until bit7 is 1
		{
			qmi8658_delay_ms(1);
			qmi8658_read_reg(status_reg, &status1, 1);
		}
		// qmi8658_log("ctrl9 cmd (%d) done1 count=%d\n", cmd, count);
		if (count < 100)
		{
			ret1 = 1;
		}
		else
		{
			ret1 = 0;
		}

		qmi8658_write_reg(Qmi8658Register_Ctrl9, qmi8658_Ctrl9_Cmd_Ack); // write commond  0x00 to ctrl9
		count = 0;
		qmi8658_read_reg(status_reg, &status1, 1);
		while (((status1 & cmd_done) == cmd_done) && (count++ < 100)) // read statusINT until bit7 is 0
		{
			qmi8658_delay_ms(1); // 1 ms
			qmi8658_read_reg(status_reg, &status1, 1);
		}
		// qmi8658_log("ctrl9 cmd (%d) done2 count=%d\n", qmi8658_Ctrl9_Cmd_Ack, count);
		if (count < 100)
		{
			ret2 = 1;
		}
		else
		{
			ret2 = 0;
		}

		if ((ret1 == 0) || (ret2 == 0))
		{
			continue;
		}
		else
		{
			break;
		}
	}

	if (ret1 && ret2)
	{
		return 1;
	}
	else
	{
		qmi8658_log("qmi8658_send_ctl9cmd fail cmd=%d\n", cmd);
		return 0;
	}
}

float qmi8658_readTemp(void)
{
	unsigned char buf[2];
	short temp = 0;
	float temp_f = 0;

	qmi8658_read_reg(Qmi8658Register_Tempearture_L, buf, 2);
	temp = ((short)buf[1] << 8) | buf[0];
	temp_f = (float)temp / 256.0f;

	return temp_f;
}

void qmi8658_read_timestamp(unsigned int *tim_count)
{
	unsigned char buf[3];
	unsigned int timestamp;

	if (tim_count)
	{
		qmi8658_read_reg(Qmi8658Register_Timestamp_L, buf, 3);
		timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
		if (timestamp > g_imu.timestamp)
			g_imu.timestamp = timestamp;
		else
			g_imu.timestamp = (timestamp + 0x1000000 - g_imu.timestamp);

		*tim_count = g_imu.timestamp;
	}
}

static void qmi8658_read_sensor_data(float acc[3], float gyro[3])
{
	unsigned char buf_reg[12];
	short raw_acc_xyz[3];
	short raw_gyro_xyz[3];

	qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 12);
	raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
	raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
	raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

	raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
	raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
	raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

	qmi8658_axis_convert(raw_acc_xyz, raw_gyro_xyz, 0);

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[0] = (float)(raw_acc_xyz[0] * 1000.0f) / g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1] * 1000.0f) / g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2] * 1000.0f) / g_imu.ssvt_a;
#else
	// m/s2
	acc[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;
#endif

#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0] * 1.0f) / g_imu.ssvt_g;
	gyro[1] = (float)(raw_gyro_xyz[1] * 1.0f) / g_imu.ssvt_g;
	gyro[2] = (float)(raw_gyro_xyz[2] * 1.0f) / g_imu.ssvt_g;
#else
	// rad/s
	gyro[0] = (float)(raw_gyro_xyz[0] * M_PI) / (g_imu.ssvt_g * 180); // *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1] * M_PI) / (g_imu.ssvt_g * 180);
	gyro[2] = (float)(raw_gyro_xyz[2] * M_PI) / (g_imu.ssvt_g * 180);
#endif
}

#if defined(QMI8658_EN_CGAIN)
static void readIndirectData(unsigned int startAddr, unsigned char *buf, unsigned char bytesNum)
{
	unsigned char addrList[4];
	unsigned char ctrlData = 0;

	qmi8658_delay_ms(1);
	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, 0x7F);
	ctrlData = (1 << 7) + (bytesNum << 3) + (0 << 2) + (0 << 1) + 1;
	qmi8658_write_reg(BANK0_INDIRECT_CTRL_ADDR, ctrlData); // apply new settings
	qmi8658_delay_ms(1);
	// Start reading the data
	qmi8658_read_reg(BANK0_INDIRECT_SYS_DATA_ADDR, buf, bytesNum);
}

unsigned char qmi8658_read_cgain(void)
{
	unsigned char data, start_en, cgain;

	readIndirectData(QMI8658_EN_CGAIN, &data, 1);
	start_en = (data & 0x80) >> 7;
	cgain = data & 0x3F;
	//	printf("qmi8658_read_cgain: %d %d\r\n", cgain, start_en);
	if (cgain >= 63)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_delay_ms(10);
		qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
		qmi8658_delay_ms(10);
	}

	return cgain;
}
#endif

void qmi8658_read_xyz(float acc[3], float gyro[3])
{
	unsigned char status = 0;
	unsigned char data_ready = 0;
	int retry = 0;

	while (retry++ < 3)
	{
#if defined(QMI8658_EN_CGAIN)
		qmi8658_read_cgain();
#endif
#if defined(QMI8658_SYNC_SAMPLE_MODE)
		qmi8658_read_reg(Qmi8658Register_StatusInt, &status, 1);
		if ((status & 0x01) && (status & 0x02))
		{
			data_ready = 1;
			qmi8658_delay_us(12); // delay 12us <=500Hz�� 12us 1000Hz, 4us 2000Hz 2us > 2000Hz
			break;
		}
#else
		qmi8658_read_reg(Qmi8658Register_Status0, &status, 1);
		// qmi8658_log("status0 0x%x\n", status);
		if (status & 0x03)
		{
			data_ready = 1;
			break;
		}
#endif
	}
	if (data_ready)
	{
		qmi8658_read_sensor_data(acc, gyro);
		qmi8658_log("data ready ok!\r\n");
		g_imu.imu[0] = acc[0];
		g_imu.imu[1] = acc[1];
		g_imu.imu[2] = acc[2];
		g_imu.imu[3] = gyro[0];
		g_imu.imu[4] = gyro[1];
		g_imu.imu[5] = gyro[2];
	}
	else
	{
		acc[0] = g_imu.imu[0];
		acc[1] = g_imu.imu[1];
		acc[2] = g_imu.imu[2];
		gyro[0] = g_imu.imu[3];
		gyro[1] = g_imu.imu[4];
		gyro[2] = g_imu.imu[5];
		qmi8658_log("data ready fail!\r\n");
	}
}

#if defined(QMI8658_SYNC_SAMPLE_MODE)
void qmi8658_enable_AHB_clock(int enable)
{
	if (enable)
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x00);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_AHB_Clock_Gating);
	}
	else
	{
		qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
		qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x01);
		qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_AHB_Clock_Gating);
	}
}
#endif

void qmi8658_enableSensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#else
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags); // QMI8658_DRDY_DISABLE
#endif
	g_imu.cfg.enSensors = enableFlags & 0x03;

	qmi8658_delay_ms(2);
}

void qmi8658_dump_reg(void)
{
#if 1
	unsigned char read_data[8];

	qmi8658_read_reg(Qmi8658Register_Ctrl1, read_data, 8);
	qmi8658_log("Ctrl1[0x%x]\nCtrl2[0x%x]\nCtrl3[0x%x]\nCtrl4[0x%x]\nCtrl5[0x%x]\nCtrl6[0x%x]\nCtrl7[0x%x]\nCtrl8[0x%x]\n",
				read_data[0], read_data[1], read_data[2], read_data[3], read_data[4], read_data[5], read_data[6], read_data[7]);
	// qmi8658_read_reg(Qmi8658Register_FifoWmkTh, read_data, 4);
	// qmi8658_log("FIFO reg[0x%x 0x%x 0x%x 0x%x]\n", read_data[0],read_data[1],read_data[2],read_data[3]);
	qmi8658_log("\n");
#else

#endif
}

void qmi8658_get_chip_info(void)
{
	unsigned char revision_id = 0x00;
	unsigned char firmware_id[3];
	unsigned char uuid[6];
	unsigned int uuid_low, uuid_high;

	qmi8658_read_reg(Qmi8658Register_Revision, &revision_id, 1);
	qmi8658_read_reg(Qmi8658Register_firmware_id, firmware_id, 3);
	qmi8658_read_reg(Qmi8658Register_uuid, uuid, 6);
	uuid_low = (unsigned int)((unsigned int)(uuid[2] << 16) | (unsigned int)(uuid[1] << 8) | (uuid[0]));
	uuid_high = (unsigned int)((unsigned int)(uuid[5] << 16) | (unsigned int)(uuid[4] << 8) | (uuid[3]));
	// qmi8658_log("VS ID[0x%x]\n", revision_id);
	qmi8658_log("**FW ID[%d %d %d] Revision;0x%x\n", firmware_id[2], firmware_id[1], firmware_id[0], revision_id);
	qmi8658_log("**UUID[0x%x %x]\n", uuid_high, uuid_low);
}

void qmi8658_soft_reset(void)
{
	unsigned char reset_done = 0x00;
	int retry = 0;

	qmi8658_log("qmi8658_soft_reset \n");
	qmi8658_write_reg(Qmi8658Register_Reset, 0xb0);
	qmi8658_delay_ms(10); // delay
	while (reset_done != 0x80)
	{
		qmi8658_delay_ms(1);
		qmi8658_read_reg(Qmi8658Register_Reset_done, &reset_done, 1);
		if (retry++ > 500)
		{
			break;
		}
	}
	qmi8658_log("qmi8658_soft_reset done retry=%d\n", retry);
}

void qmi8658_get_gyro_gain(unsigned char cod_data[6])
{
	qmi8658_read_reg(Qmi8658Register_Dvx_L, &cod_data[0], 6);
	qmi8658_log("cod data[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n", cod_data[0], cod_data[1], cod_data[2],
				cod_data[3], cod_data[4], cod_data[5]);
}

void qmi8658_apply_gyr_gain(unsigned char cod_data[6])
{
	qmi8658_enableSensors(QMI8658_DISABLE_ALL);
	qmi8658_write_reg(Qmi8658Register_Cal1_L, cod_data[0]);
	qmi8658_write_reg(Qmi8658Register_Cal1_H, cod_data[1]);
	qmi8658_write_reg(Qmi8658Register_Cal2_L, cod_data[2]);
	qmi8658_write_reg(Qmi8658Register_Cal2_H, cod_data[3]);
	qmi8658_write_reg(Qmi8658Register_Cal3_L, cod_data[4]);
	qmi8658_write_reg(Qmi8658Register_Cal3_H, cod_data[5]);

	qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Apply_Gyro_Gain);
}

void qmi8658_on_demand_cali(void)
{
	unsigned char cod_status = 0x00;
	qmi8658_log("qmi8658_on_demand_cali start\n");
	qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	qmi8658_delay_ms(2200); // delay 2000ms above
	qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_Ack);
	qmi8658_delay_ms(10); // delay
	qmi8658_read_reg(Qmi8658Register_Cod_Status, &cod_status, 1);
	if (cod_status)
	{
		qmi8658_log("qmi8658_on_demand_cali fail! status=0x%x\n", cod_status);
	}
	else
	{
		qmi8658_get_gyro_gain(g_imu.cod_data);
		qmi8658_log("qmi8658_on_demand_cali done! cod[%d %d %d]\n",
					(unsigned short)(g_imu.cod_data[1] << 8 | g_imu.cod_data[0]),
					(unsigned short)(g_imu.cod_data[3] << 8 | g_imu.cod_data[2]),
					(unsigned short)(g_imu.cod_data[5] << 8 | g_imu.cod_data[4]));
	}
}

void qmi8658_config_reg(unsigned char low_power)
{
	qmi8658_enableSensors(QMI8658_DISABLE_ALL);
	if (low_power)
	{
		g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_LowPower_21Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_125Hz;
	}
	else
	{
		g_imu.cfg.enSensors = QMI8658_ACCGYR_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_250Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_125Hz;
	}

	if (g_imu.cfg.enSensors & QMI8658_ACC_ENABLE)
	{
		qmi8658_config_acc(g_imu.cfg.accRange, g_imu.cfg.accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
	if (g_imu.cfg.enSensors & QMI8658_GYR_ENABLE)
	{
		qmi8658_config_gyro(g_imu.cfg.gyrRange, g_imu.cfg.gyrOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
}

unsigned char qmi8658_get_id(void)
{
	unsigned char qmi8658_chip_id = 0x00;
	unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
	int retry = 0;
	unsigned char iCount = 0;

	while (iCount < 2)
	{
		g_imu.slave = qmi8658_slave[iCount];
		retry = 0;
		while ((qmi8658_chip_id != 0x05) && (retry++ < 5))
		{
			qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
		}
		qmi8658_log("qmi8658 slave = 0x%x WhoAmI = 0x%x\n", g_imu.slave, qmi8658_chip_id);
		if (qmi8658_chip_id == 0x05)
		{
			g_imu.cfg.syncSample = 0;
			g_imu.cfg.ctrl8_value = 0xc0;
			qmi8658_soft_reset();
			qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60 | QMI8658_INT2_ENABLE | QMI8658_INT1_ENABLE);
			qmi8658_get_chip_info();
#if defined(QMI8658_USE_GYRO_STARTUP_TEST) // check 0x45
			unsigned char opt_status = 0x00;
			qmi8658_write_reg(Qmi8658Register_Ctrl2, 0x25);
			qmi8658_write_reg(Qmi8658Register_Ctrl3, 0x66);
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x03);
			qmi8658_delay_ms(300);
			qmi8658_read_reg(0x45, &opt_status, 1);
			// qmi8658_log("opt_status = 0x%x\n", opt_status);
			if (opt_status != 0x80)
			{
				qmi8658_log("**ERROR[0x45=0x%x]\n", opt_status);
				return 0;
			}
			else
			{
				qmi8658_log("**SUCCESS[0x45=0x%x]\n", opt_status);
			}
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
#endif
#if defined(QMI8658_USE_HW_SELFTEST)
			qmi8658_do_hw_selftest(QMI8658_ACCGYR_ENABLE);
#endif
			qmi8658_on_demand_cali();
			qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
			qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
#if defined(QMI8658_EN_CGAIN)
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_7_0_ADDR, 0x2a);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_15_8_ADDR, 0x00);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_23_16_ADDR, 0x06);
			qmi8658_write_reg(BANK0_INDIRECT_SYS_ADDR_31_24_ADDR, 0x00);
#endif
			break;
		}
		iCount++;
	}
	if (qmi8658_chip_id != 0x05)
	{
		qmi8658_log("**ERROR1[id=0x%x]\n", qmi8658_chip_id);
	}

	return qmi8658_chip_id;
}

int qmi8658_init(void)
{
	/* hardware init */
	qmi8658_hw_init();

	if (qmi8658_get_id() == 0x05)
	{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
		qmi8658_enable_AHB_clock(0);
		g_imu.cfg.syncSample = 1;
#endif

		qmi8658_config_reg(0);
		qmi8658_enableSensors(g_imu.cfg.enSensors);
		qmi8658_dump_reg();

		qmi8658_delay_ms(300);

		return 1;
	}
	else
	{
		qmi8658_log("qmi8658_init fail\n");
		return 0;
	}
}
