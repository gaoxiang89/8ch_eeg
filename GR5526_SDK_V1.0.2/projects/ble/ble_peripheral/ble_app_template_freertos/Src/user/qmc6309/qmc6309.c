
#include "qmc6309.h"

static qmc6309_data_t p_mag;
//static const unsigned char mag_slave[] = {QMC6309_IIC_ADDR, QMC6309H_IIC_ADDR};
//static const unsigned char mag_slave[] = {QMC6309H_IIC_ADDR};
static const unsigned char mag_slave[] = {QMC6309_IIC_ADDR};

int qmc6309_read_reg(unsigned char addr, unsigned char *buf, unsigned short len)

{
	int ret = 0;

	ret = qmc6309_i2c_write(p_mag.slave_addr, &addr, 1);
	if (ret < 0)
	{
		QMC6309_LOG("read reg failed, write return %d \r\n", ret);
	}

	ret = qmc6309_i2c_read(p_mag.slave_addr , buf, len);
	if (ret < 0)
	{
		QMC6309_LOG("read reg failed, read return %d \r\n", ret);
	}

	return ret;
}

int qmc6309_write_reg(unsigned char addr, unsigned char data)
{
	int ret = 0;
	uint8_t buf[2];

	buf[0] = addr;
	buf[1] = data;

	ret = qmc6309_i2c_write(p_mag.slave_addr, buf, 2);

	return ret;
}


void qmc6309_axis_convert(float data[3], int layout)
{
	float raw[3];

	raw[0] = data[0];
	raw[1] = data[1];
	//raw[2] = data[2];
	if(layout >=4 && layout <= 7)
	{
		data[2] = -data[2];
	}
	//else
	//{
	//	data[2] = raw[2];
	//}

	if(layout%2)
	{
		data[0] = raw[1];
		data[1] = raw[0];
	}
	else
	{
		data[0] = raw[0];
		data[1] = raw[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data[0] = -data[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data[1] = -data[1];
	}
}

void qmc6309_dump_reg(void)
{
	unsigned char ctrl_value[4];
	unsigned char version_id;
	unsigned char wafer_id;
	unsigned char die_id[2];

	QMC6309_LOG("qmc6309_dump_reg\r\n");
	qmc6309_read_reg(0x12, &version_id, 1);
	QMC6309_LOG("version id:0x%x \r\n", version_id);
	qmc6309_read_reg(0x37, &wafer_id, 1);
	QMC6309_LOG("wafer id:0x%x \r\n", wafer_id&0x1f);
	qmc6309_read_reg(0x38, die_id, 2);
	QMC6309_LOG("die id:0x%x \r\n", (die_id[1]<<8)|die_id[0]);

	qmc6309_read_reg(QMC6309_CTL_REG_ONE, ctrl_value, 2);
	QMC6309_LOG("ctrlreg [0x0a=0x%x 0x0b=0x%x] \r\n", ctrl_value[0], ctrl_value[1]);
	qmc6309_read_reg(QMC6309_FIFO_REG_CTRL, ctrl_value, 1);
	QMC6309_LOG("fifo-ctrl 0x%x=0x%x \r\n", QMC6309_FIFO_REG_CTRL, ctrl_value[0]);

	qmc6309_read_reg(0x40, ctrl_value, 1);
	QMC6309_LOG("0x40 = 0x%x \r\n", ctrl_value[0]);
}


int qmc6309_get_chipid(void)
{
	int ret = QMC6309_FAIL;
	int retry=0;
	unsigned char chip_id = 0x00;

	retry = 0;
	while((chip_id != QMC6309_CHIP_ID) && (retry++<5))
	{
		ret = qmc6309_read_reg(QMC6309_CHIP_ID_REG, &chip_id, 1);
		if(ret == QMC6309_OK)
		{
			break;
		}
	}
	if(chip_id == QMC6309_CHIP_ID)
	{
		QMC6309_LOG("qmc6309_get_chipid-ok slave:0x%x chipid = 0x%x\r\n", p_mag.slave_addr, chip_id);
		return 1;
	}
	else
	{
		QMC6309_LOG("qmc6309_get_chipid-fail slave:0x%x chip_id = 0x%x\r\n", p_mag.slave_addr, chip_id);
		return 0;
	}
}

void qmc6309_set_range(unsigned char range)
{
	QMC6309_LOG("qmc6309_set_range 0x%x\r\n", range);
	p_mag.ctrl2.bit.range = range;

	switch(p_mag.ctrl2.bit.range)
	{
		case QMC6309_RNG_32G:
			p_mag.ssvt = 1000;
			break;
		case QMC6309_RNG_16G:
			p_mag.ssvt = 2000;
			break;
		case QMC6309_RNG_8G:
			p_mag.ssvt = 4000;
			break;
		default:
			p_mag.ssvt = 1000;
			break;
	}
}

int qmc6309_enable(void)
{
	int ret = 0;

	QMC6309_LOG("qmc6309_enable!\r\n");
	if(p_mag.chip_type == TYPE_QMC6309H)
	{
		unsigned char reg = 0;

		ret = qmc6309_read_reg(0x40, &reg, 1);
		QMC6309_LOG("read 0x40=0x%x\r\n", reg);
#if defined(QMC6309H_VDD_MODE1)
		reg = ((reg&0x3f)|0x80);
		ret = qmc6309_write_reg(0x40, reg);
#elif defined(QMC6309H_VDD_MODE2)
		reg = ((reg&0x3f)|0x40);
		ret = qmc6309_write_reg(0x40, reg);
#endif
		qmc6309_delay_ms(1);
	}
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, p_mag.ctrl2.value);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay_ms(1);
	ret = qmc6309_write_reg(QMC6309_CTL_REG_ONE, p_mag.ctrl1.value);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay_ms(1);

	return ret;
}

int qmc6309_disable(void)
{
	int ret = 0;

	QMC6309_LOG("qmc6309_disable!\r\n");
	ret = qmc6309_write_reg(QMC6309_CTL_REG_ONE, 0x00);
	QMC6309_CHECK_ERR(ret);

	return ret;
}

void qmc6309_enable_ibi(qmc6309_fifo_ibi flag)
{
	int ret = QMC6309_FAIL;
	unsigned char ibi_value = 0x00;

	QMC6309_LOG("qmc6309_enable_ibi 0x%x\r\n", flag);
	ibi_value = (unsigned char)flag;
	ret = qmc6309_write_reg(QMC6309_CTL_IBI, ibi_value);

	QMC6309_CHECK_ERR(ret);
}

void qmc6309_init_para(unsigned char mode, unsigned char odr)
{
	p_mag.ctrl1.bit.mode = mode;	// QMC6309_MODE_HPFM; QMC6309_MODE_NORMAL
	p_mag.ctrl1.bit.osr1 = QMC6309_OSR1_8;
	p_mag.ctrl1.bit.osr2 = QMC6309_OSR2_4;
	if(p_mag.chip_type == TYPE_QMC6309)	
		p_mag.ctrl1.bit.zdbl_enb = QMC6309_ZDBL_ENB_OFF;
	else
		p_mag.ctrl1.bit.zdbl_enb = QMC6309H_ZDBL_ENB_OFF;		

	p_mag.ctrl2.bit.set_rst = QMC6309_SET_RESET_ON;		// QMC6309_SET_ON, QMC6309_SET_RESET_ON
	p_mag.ctrl2.bit.range = QMC6309_RNG_32G;
	p_mag.ctrl2.bit.odr = odr;
	p_mag.ctrl2.bit.soft_rst = 0;

	qmc6309_set_range(p_mag.ctrl2.bit.range);
#if defined(QMC6309_MODE_SWITCH)
	p_mag.set_ctl.mode = 0;
	p_mag.set_ctl.count = 0;
#endif
}

void qmc6309_soft_reset(void)
{
	int ret = QMC6309_FAIL;
	int retry = 0;
	unsigned char status = 0x00;

	QMC6309_LOG("qmc6309_soft_reset!\r\n");
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, 0x80);
	QMC6309_CHECK_ERR(ret);
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, 0x00);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay_ms(5);

	while(retry++<5)
	{
		ret = qmc6309_read_reg(QMC6309_STATUS_REG, &status, 1);
		QMC6309_CHECK_ERR(ret);
		QMC6309_LOG("qmc6309 status 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			QMC6309_LOG("qmc6309 NVM load done!\r\n");
			break;
		}
		qmc6309_delay_ms(1);
	}
}


void qmc6309_reload_otp(unsigned char slave)
{
	int ret = 0;
	unsigned char status = 0;
	int retry = 0;
	int count = 0;

	while(retry++ < 5)
	{
		p_mag.slave_addr = slave;
		ret = qmc6309_write_reg(0x28, 0x02);
		if(ret != QMC6309_OK)
		{
			QMC6309_LOG("write 0x28 = 0x02 fail!\r\n");
		}
		count = 0;

		while(count++<100)
		{
			qmc6309_delay_ms(1);
			status = 0;
			// try 0x7c
			p_mag.slave_addr = QMC6309_IIC_ADDR;
			ret = qmc6309_read_reg(QMC6309_STATUS_REG, &status, 1);
			if((ret==QMC6309_OK)&&(status & 0x10))
			{
				QMC6309_LOG("qmc6309_reload_otp done slave=0x%x status=0x%x\r\n", p_mag.slave_addr, status);
				return;
			}
			// try 0x0c
			p_mag.slave_addr = QMC6309H_IIC_ADDR;
			ret = qmc6309_read_reg(QMC6309_STATUS_REG, &status, 1);
			if((ret==QMC6309_OK)&&(status & 0x10))
			{
				QMC6309_LOG("qmc6309_reload_otp done slave=0x%x status=0x%x\r\n", p_mag.slave_addr, status);
				return;
			}
		}
	}
}


int qmc6309_read_mag_raw(short raw[3])
{
	static int qmc6309_fail_num = 0;
	int res = QMC6309_FAIL;
	unsigned char mag_data[6];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	while(!(rdy & QMC6309_STATUS_DRDY) && (t1++ < 5))
	{
		res = qmc6309_read_reg(QMC6309_STATUS_REG, &rdy, 1);
		qmc6309_delay_ms(1);
	}
	if((res == QMC6309_FAIL)||(!(rdy & QMC6309_STATUS_DRDY)))
  	{
		raw[0] = p_mag.last_data[0];
		raw[1] = p_mag.last_data[1];
		raw[2] = p_mag.last_data[2];
		QMC6309_LOG("qmc6309_read_mag_xyz read drdy fail! res=%d rdy=0x%x\r\n",res,rdy);
		if(qmc6309_fail_num++ > 10)
		{
#if defined(QMC6309_RECOVER)
			qmc6309_recover();
#endif
			qmc6309_enable();
			qmc6309_fail_num = 0;
		}
		res = QMC6309_OK;	// QMC6309_FAIL;
	}
	else
	{
		qmc6309_fail_num = 0;
		mag_data[0] = QMC6309_DATA_OUT_X_LSB_REG;
		res = qmc6309_read_reg(QMC6309_DATA_OUT_X_LSB_REG, mag_data, 6);
		if(res == QMC6309_FAIL)
	  	{
			QMC6309_LOG("qmc6309_read_mag_xyz read data fail! res=%d\r\n",res);
			raw[0] = p_mag.last_data[0];
			raw[1] = p_mag.last_data[1];
			raw[2] = p_mag.last_data[2];
			res = QMC6309_OK;	// QMC6309_FAIL;
		}
		else
		{
			raw[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
			raw[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
			raw[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
		}
	}
	
	p_mag.last_data[0] = raw[0];
	p_mag.last_data[1] = raw[1];
	p_mag.last_data[2] = raw[2];	
#if defined(QMC6309_MODE_SWITCH)
	qmc6309_setrst_auto_mode(raw);
#endif

	return res;
}

int qmc6309_read_mag_xyz(float data[3])
{
	int res = QMC6309_FAIL;
	short hw_d[3] = {0};

	res = qmc6309_read_mag_raw(hw_d);
	if(res == QMC6309_OK)
	{
		data[0] = (float)((float)hw_d[0] / ((float)p_mag.ssvt/100.f));		// ut
		data[1] = (float)((float)hw_d[1] / ((float)p_mag.ssvt/100.f));		// ut
		data[2] = (float)((float)hw_d[2] / ((float)p_mag.ssvt/100.f));		// ut
	}
	else
	{
		data[0] = data[1]= data[2] = 0.0f;
	}
	qmc6309_axis_convert(data, 0);
	if(p_mag.ctrl1.bit.mode == QMC6309_MODE_SINGLE)
	{
		res = qmc6309_write_reg(QMC6309_CTL_REG_ONE, p_mag.ctrl1.value);
		QMC6309_CHECK_ERR(res);
	}

	return res;
}



#if defined(QMC6309_RECOVER)
int qmc6309_recover(void)
{
	int ret = QMC6309_FAIL;
	unsigned char slave_loop[]={0x7c,0x0c};

	for(int i=0; i<sizeof(slave_loop)/sizeof(slave_loop[0]); i++)
	{
		p_mag.chip_type = TYPE_UNKNOW;
		p_mag.slave_addr = slave_loop[i];
		ret = qmc6309_get_chipid();	// read id again
		if(ret) 	// read id OK
		{
#if 1
			qmc6309_soft_reset();	// softreset reload OTP
#else
			qmc6309_reload_otp(slave_loop[i]);	// reload otp
#endif
			p_mag.chip_type = TYPE_UNKNOW;
			for(i=0; i<sizeof(mag_slave)/sizeof(mag_slave[0]); i++)
			{
				p_mag.slave_addr = mag_slave[i];
				ret = qmc6309_get_chipid();
				if(ret)
				{
					if(p_mag.slave_addr == QMC6309_IIC_ADDR)
					{
						p_mag.chip_type = TYPE_QMC6309;
					}
					else if(p_mag.slave_addr == QMC6309H_IIC_ADDR)
					{
						p_mag.chip_type = TYPE_QMC6309H;
					}
					else
					{
						p_mag.chip_type = TYPE_UNKNOW;
					}
					break;
				}
			}

			break;
		}
	}
 
	QMC6309_LOG("qmc6309_recover %s mag_type=%d\r\n", ret?"OK":"FAIL", p_mag.chip_type);
	return ret;
}
#endif

int qmc6309_init(void)
{
	int ret = 0;
	int i = 0;

	qmc6309_hw_init();

	p_mag.chip_type = TYPE_UNKNOW;
	for(i=0; i<sizeof(mag_slave)/sizeof(mag_slave[0]); i++)
	{
		p_mag.slave_addr = mag_slave[i];
		ret = qmc6309_get_chipid();
		if(ret)
		{
			if(p_mag.slave_addr == QMC6309_IIC_ADDR)
			{
				p_mag.chip_type = TYPE_QMC6309;
			}
			else if(p_mag.slave_addr == QMC6309H_IIC_ADDR)
			{
				p_mag.chip_type = TYPE_QMC6309H;
			}
			else
			{
				p_mag.chip_type = TYPE_UNKNOW;
			}
			break;
		}
	}

#if defined(QMC6309_RECOVER)
	if(p_mag.chip_type == TYPE_UNKNOW)
	{
		qmc6309_recover();
	}
#endif

	if(p_mag.chip_type != TYPE_UNKNOW)
	{
		qmc6309_soft_reset();
		qmc6309_init_para(QMC6309_MODE_NORMAL, QMC6309_ODR_50HZ);
		ret = qmc6309_disable();
		QMC6309_CHECK_ERR(ret);
		ret = qmc6309_enable();
		QMC6309_CHECK_ERR(ret);
		qmc6309_dump_reg();

		return 1;
	}
	else
	{
		return 0;
	}
}


