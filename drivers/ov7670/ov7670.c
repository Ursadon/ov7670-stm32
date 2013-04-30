
uint8_t ov7670_get(uint8_t reg) {
    uint8_t data = 0;
	I2C_start(I2C2, 0x42, I2C_Direction_Transmitter);
	I2C_write(I2C2, reg);
	I2C_stop(I2C2);
	delay(1000);
	I2C_start(I2C2, 0x43, I2C_Direction_Receiver);
	data = I2C_read_nack(I2C2);
	I2C_stop(I2C2);
	delay(1000);
	return data;
}

uint8_t ov7670_set(uint8_t reg, uint8_t data) {
	I2C_start(I2C2, 0x42, I2C_Direction_Transmitter);
	I2C_write(I2C2, reg);
	I2C_write(I2C2, data);
	I2C_stop(I2C2);
	delay(1000);
	return 0;
}

int ov7670_init(){
    int hstart = 456, hstop = 24, vstart = 14, vstop = 494;
	unsigned char v;
    
    if (ov7670_get(REG_PID) != 0x76) {
		return 1;
	}
	ov7670_set(REG_COM7, COM7_RESET); /* reset to default values */
	ov7670_set(REG_CLKRC, 0x01);
	ov7670_set(REG_COM7, COM7_FMT_VGA | COM7_YUV); /* output format: YUCV */

	ov7670_set(REG_HSTART, (hstart >> 3) & 0xff);
	ov7670_set(REG_HSTOP, (hstop >> 3) & 0xff);
	v = ov7670_get(REG_HREF);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	ov7670_set(REG_HREF, v);

	ov7670_set(REG_VSTART, (vstart >> 2) & 0xff);
	ov7670_set(REG_VSTOP, (vstop >> 2) & 0xff);
	v = ov7670_get(REG_VREF);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	ov7670_set(REG_VREF, v);
    
	ov7670_set(REG_COM3, COM3_SCALEEN | COM3_DCWEN);
	ov7670_set(REG_COM14, COM14_DCWEN | 0x01);
	ov7670_set(0x73, 0xf1);
	ov7670_set(0xa2, 0x52);
	ov7670_set(0x7b, 0x1c);
	ov7670_set(0x7c, 0x28);
	ov7670_set(0x7d, 0x3c);
	ov7670_set(0x7f, 0x69);
	ov7670_set(REG_COM9, 0x38);
	ov7670_set(0xa1, 0x0b);
	ov7670_set(0x74, 0x19);
	ov7670_set(0x9a, 0x80);
	ov7670_set(0x43, 0x14);
	ov7670_set(REG_COM13, 0xc0);
	ov7670_set(0x70, 0x3A);
	ov7670_set(0x71, 0x35);
	ov7670_set(0x72, 0x11);

	/* Gamma curve values */
	ov7670_set(0x7a, 0x20);
	ov7670_set(0x7b, 0x10);
	ov7670_set(0x7c, 0x1e);
	ov7670_set(0x7d, 0x35);
	ov7670_set(0x7e, 0x5a);
	ov7670_set(0x7f, 0x69);
	ov7670_set(0x80, 0x76);
	ov7670_set(0x81, 0x80);
	ov7670_set(0x82, 0x88);
	ov7670_set(0x83, 0x8f);
	ov7670_set(0x84, 0x96);
	ov7670_set(0x85, 0xa3);
	ov7670_set(0x86, 0xaf);
	ov7670_set(0x87, 0xc4);
	ov7670_set(0x88, 0xd7);
	ov7670_set(0x89, 0xe8);

	/* AGC and AEC parameters.  Note we start by disabling those features,
	 then turn them only after tweaking the values. */
	ov7670_set(REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT);
	ov7670_set(REG_GAIN, 0);
	ov7670_set(REG_AECH, 0);
	ov7670_set(REG_COM4, 0x40); /* magic reserved bit */
	ov7670_set(REG_COM9, 0x18); /* 4x gain + magic rsvd bit */
	ov7670_set(REG_BD50MAX, 0x05);
	ov7670_set(REG_BD60MAX, 0x07);
	ov7670_set(REG_AEW, 0x95);
	ov7670_set(REG_AEB, 0x33);
	ov7670_set(REG_VPT, 0xe3);
	ov7670_set(REG_HAECC1, 0x78);
	ov7670_set(REG_HAECC2, 0x68);
	ov7670_set(0xa1, 0x03); /* magic */
	ov7670_set(REG_HAECC3, 0xd8);
	ov7670_set(REG_HAECC4, 0xd8);
	ov7670_set(REG_HAECC5, 0xf0);
	ov7670_set(REG_HAECC6, 0x90);
	ov7670_set(REG_HAECC7, 0x94);
	ov7670_set(REG_COM8,
			COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC);

	/* Almost all of these are magic "reserved" values.  */
	ov7670_set(REG_COM5, 0x61);
	ov7670_set(REG_COM6, 0x4b);
	ov7670_set(0x16, 0x02);
	ov7670_set(REG_MVFP, 0x07);
	ov7670_set(0x21, 0x02);
	ov7670_set(0x22, 0x91);
	ov7670_set(0x29, 0x07);
	ov7670_set(0x33, 0x0b);
	ov7670_set(0x35, 0x0b);
	ov7670_set(0x37, 0x1d);
	ov7670_set(0x38, 0x71);
	ov7670_set(0x39, 0x2a);
	ov7670_set(REG_COM12, 0x78);
	ov7670_set(0x4d, 0x40);
	ov7670_set(0x4e, 0x20);
	ov7670_set(REG_GFIX, 0);
	ov7670_set(0x6b, 0x4a);
	ov7670_set(0x74, 0x10);
	ov7670_set(0x8d, 0x4f);
	ov7670_set(0x8e, 0);
	ov7670_set(0x8f, 0);
	ov7670_set(0x90, 0);
	ov7670_set(0x91, 0);
	ov7670_set(0x96, 0);
	ov7670_set(0x9a, 0);
	ov7670_set(0xb0, 0x84);
	ov7670_set(0xb1, 0x0c);
	ov7670_set(0xb2, 0x0e);
	ov7670_set(0xb3, 0x82);
	ov7670_set(0xb8, 0x0a);

	/* Matrix coefficients */
	ov7670_set(0x4f, 0x80);
	ov7670_set(0x50, 0x80);
	ov7670_set(0x51, 0);
	ov7670_set(0x52, 0x22);
	ov7670_set(0x53, 0x5e);
	ov7670_set(0x54, 0x80);
	ov7670_set(0x58, 0x9e);

	/* More reserved magic, some of which tweaks white balance */
	ov7670_set(0x43, 0x0a);
	ov7670_set(0x44, 0xf0);
	ov7670_set(0x45, 0x34);
	ov7670_set(0x46, 0x58);
	ov7670_set(0x47, 0x28);
	ov7670_set(0x48, 0x3a);
	ov7670_set(0x59, 0x88);
	ov7670_set(0x5a, 0x88);
	ov7670_set(0x5b, 0x44);
	ov7670_set(0x5c, 0x67);
	ov7670_set(0x5d, 0x49);
	ov7670_set(0x5e, 0x0e);
	ov7670_set(0x6c, 0x0a);
	ov7670_set(0x6d, 0x55);
	ov7670_set(0x6e, 0x11);
	ov7670_set(0x6f, 0x9f); /* "9e for advance AWB" */
	ov7670_set(0x6a, 0x40);
	ov7670_set(REG_BLUE, 0x40);
	ov7670_set(REG_RED, 0x60);
	ov7670_set(REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC | COM8_AWB);
    return 0;
}