/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2020 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "board.h"

#include "ac101.h"

static const char *AC101_TAG = "AC101";

static i2c_config_t ac_i2c_cfg = {
	.mode = I2C_MODE_MASTER,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 100000};


#define AC101_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(AC101_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static bool codec_init_flag;

audio_hal_func_t AUDIO_CODEC_AC101_CODEC_HANDLE = {
    .audio_codec_initialize = ac101_init,
    .audio_codec_deinitialize = ac101_deinit,
    .audio_codec_ctrl = ac101_ctrl_state,
    .audio_codec_config_iface = ac101_config_i2s,
    .audio_codec_set_mute = ac101_set_output_mute,
    .audio_codec_set_volume = ac101_set_output_volume,
    .audio_codec_get_volume = ac101_get_output_volume,
};

static esp_err_t ac101_write_register(uint8_t reg_addr, uint16_t val)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	esp_err_t ret = 0;
	uint8_t send_buff[4];
	send_buff[0] = (AC101_ADDR << 1);
	send_buff[1] = reg_addr;
	send_buff[2] = (val >> 8) & 0xff;
	send_buff[3] = val & 0xff;
	ret |= i2c_master_start(cmd);
	ret |= i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN);
	ret |= i2c_master_stop(cmd);
	ret |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}
//static esp_err_t i2c_example_master_read_slave(uint8_t DevAddr, uint8_t reg,uint8_t* data_rd, size_t size)
//{
//}

static esp_err_t i2c_example_master_read_slave(uint8_t DevAddr, uint8_t reg, uint8_t *data_rd, size_t size)
{
	if (size == 0)
	{
		return ESP_OK;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DevAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DevAddr << 1) | READ_BIT, ACK_CHECK_EN); //check or not
	i2c_master_read(cmd, data_rd, size, ACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static uint16_t ac101_read_register(uint8_t reg_addr)
{
	uint16_t val = 0;
	uint8_t data_rd[2];
	i2c_example_master_read_slave(AC101_ADDR, reg_addr, data_rd, 2);
	val = (data_rd[0] << 8) + data_rd[1];
	return val;
}

static int ac101_i2c_init()
{
	int res = 0;
	get_i2c_pins(I2C_NUM_0, &ac_i2c_cfg);
	res |= i2c_param_config(I2C_NUM_0, &ac_i2c_cfg);
	res |= i2c_driver_install(I2C_NUM_0, ac_i2c_cfg.mode, 0, 0, 0);
	AC101_ASSERT(res, "i2c_init error", -1);
	return res;
}

void ac101_pa_power(bool enable)
{
    gpio_config_t  io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT(GPIO_PA_EN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    if (enable) {
        gpio_set_level(GPIO_PA_EN, 1);
    } else {
        gpio_set_level(GPIO_PA_EN, 0);
    }
}

int ac101_get_speaker_volume(void)
{
    int res;
    res = ac101_read_register(SPKOUT_CTRL);
    res &= 0x1f;
    return res*2;
}

esp_err_t ac101_set_speaker_volume(uint8_t volume)
{
	uint16_t res;
	esp_err_t ret;
	volume = volume/2;
	res = ac101_read_register(SPKOUT_CTRL);
	res &= (~0x1f);
	volume &= 0x1f;
	res |= volume;
	ret = ac101_write_register(SPKOUT_CTRL,res);
	return ret;
}

int ac101_get_earphone_volume(void)
{
    int res;
    res = ac101_read_register(HPOUT_CTRL);
    return (res>>4)&0x3f;
}

esp_err_t ac101_set_earphone_volume(uint8_t volume)
{
	uint16_t res,tmp;
	esp_err_t ret;
	res = ac101_read_register(HPOUT_CTRL);
	tmp = ~(0x3f<<4);
	res &= tmp;
	volume &= 0x3f;
	res |= (volume << 4);
	ret = ac101_write_register(HPOUT_CTRL,res);
	return ret;
}


esp_err_t ac101_start(ac_module_t mode)
{
	esp_err_t res = 0;

    ESP_LOGI(AC101_TAG, "Start AC101");

    if (mode == AC_MODULE_LINE) {
        ESP_LOGI(AC101_TAG, "Enable Line");
		res |= ac101_write_register(0x51, 0x0408);
		res |= ac101_write_register(0x40, 0x8000);
		res |= ac101_write_register(0x50, 0x3bc0);
    }
    if (mode == AC_MODULE_ADC || mode == AC_MODULE_ADC_DAC || mode == AC_MODULE_LINE) {
        ESP_LOGI(AC101_TAG, "Enable Mic");
		//I2S1_SDOUT_CTRL
		//res |= ac101_write_register(PLL_CTRL2, 0x8120);
    	res |= ac101_write_register(0x04, 0x800c);
    	res |= ac101_write_register(0x05, 0x800c);
		//res |= ac101_write_register(0x06, 0x3000);
    }
    if (mode == AC_MODULE_DAC || mode == AC_MODULE_ADC_DAC || mode == AC_MODULE_LINE) {
        ESP_LOGI(AC101_TAG, "Enable Headphones");
    	//* Enable Headphoe output  
		res |= ac101_write_register(OMIXER_DACA_CTRL, 0xff80);
    	res |= ac101_write_register(HPOUT_CTRL, 0xc3c1);
    	res |= ac101_write_register(HPOUT_CTRL, 0xcb00);
    	vTaskDelay(200 / portTICK_PERIOD_MS);
		res |= ac101_write_register(HPOUT_CTRL, 0xfbc0);
		
        ESP_LOGI(AC101_TAG, "Enable Speaker");
    	//* Enable Speaker output
		res |= ac101_write_register(SPKOUT_CTRL, 0xeabd);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		ac101_set_output_volume(120);
		
		vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return res;
}

esp_err_t ac101_stop(ac_module_t mode)
{
	esp_err_t res = 0;

    ESP_LOGI(AC101_TAG, "Stop AC101");

	res |= ac101_write_register(HPOUT_CTRL, 0x01);			//disable earphone
	res |= ac101_write_register(SPKOUT_CTRL, 0xe880);		//disable speaker
	return res;
}

/* This are the audio_hal interface fucntions every codec have to implement.
   For reference see audio_hal.h for audio_hal_func_t:

     audio_codec_initialize => ac101_init,
     audio_codec_deinitialize => ac101_deinit,
     audio_codec_ctrl => ac101_ctrl_state,
     audio_codec_config_iface => ac101_config_i2s,
     audio_codec_set_mute => ac101_set_output_mute,
     audio_codec_set_volume => ac101_set_output_volume,
     audio_codec_get_volume => ac101_get_output_volume,
*/

esp_err_t ac101_init(audio_hal_codec_config_t *cfg)
{
	esp_err_t res;

    ESP_LOGW(AC101_TAG, "Using custom codec (AC101)");
    if(ac101_i2c_init()) 
    {
		ESP_LOGE(AC101_TAG, "Init. of I2C failed!");        
        return -1;
    } 

	res = ac101_write_register(CHIP_AUDIO_RS, 0x123);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	if (ESP_OK != res) {
		ESP_LOGE(AC101_TAG, "reset failed!");
		return res;
	} else {
		ESP_LOGI(AC101_TAG, "reset succeed");
	}
	res |= ac101_write_register(SPKOUT_CTRL, 0xe880);

	//Enable the PLL from 256*44.1KHz MCLK source
	res |= ac101_write_register(PLL_CTRL1, 0x014f);
	//res |= ac101_write_register(PLL_CTRL2, 0x83c0);
	res |= ac101_write_register(PLL_CTRL2, 0x8600);

	//Clocking system
	res |= ac101_write_register(SYSCLK_CTRL, 0x8b08);
	res |= ac101_write_register(MOD_CLK_ENA, 0x800c);
	res |= ac101_write_register(MOD_RST_CTRL, 0x800c);
	res |= ac101_write_register(I2S_SR_CTRL, 0x7000);			//sample rate
	//AIF config
	res |= ac101_write_register(I2S1LCK_CTRL, 0x8850);			//BCLK/LRCK
	res |= ac101_write_register(I2S1_SDOUT_CTRL, 0xc000);		//
	res |= ac101_write_register(I2S1_SDIN_CTRL, 0xc000);
	res |= ac101_write_register(I2S1_MXR_SRC, 0x2200);			//

	res |= ac101_write_register(ADC_SRCBST_CTRL, 0xccc4);
	res |= ac101_write_register(ADC_SRC, 0x2020);
	res |= ac101_write_register(ADC_DIG_CTRL, 0x8000);
	res |= ac101_write_register(ADC_APC_CTRL, 0xbbc3);

	//Path Configuration
	res |= ac101_write_register(DAC_MXR_SRC, 0xcc00);
	res |= ac101_write_register(DAC_DIG_CTRL, 0x8000);
	res |= ac101_write_register(OMIXER_SR, 0x0081);
	res |= ac101_write_register(OMIXER_DACA_CTRL, 0xf080);//}

		//* Enable Speaker output
	res |= ac101_write_register(0x58, 0xeabd);

    ESP_LOGI(AC101_TAG, "init done");
    ac101_pa_power(true);
	return res;    
}

esp_err_t ac101_deinit(void)
{
	return	ac101_write_register(CHIP_AUDIO_RS, 0x123);		//soft reset
}

esp_err_t ac101_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
	int res = 0;
	int es_mode_t = 0;

	switch (mode) {
		case AUDIO_HAL_CODEC_MODE_ENCODE:
			es_mode_t  = AC_MODULE_ADC;
			break;
		case AUDIO_HAL_CODEC_MODE_LINE_IN:
			es_mode_t  = AC_MODULE_LINE;
			break;
		case AUDIO_HAL_CODEC_MODE_DECODE:
			es_mode_t  = AC_MODULE_DAC;
			break;
		case AUDIO_HAL_CODEC_MODE_BOTH:
			es_mode_t  = AC_MODULE_ADC_DAC;
			break;
		default:
			es_mode_t = AC_MODULE_DAC;
			ESP_LOGW(AC101_TAG, "Codec mode not support, default is decode mode");
			break;
	}
	if (AUDIO_HAL_CTRL_STOP == ctrl_state) {
		res = ac101_stop(es_mode_t);
	} else {
		res = ac101_start(es_mode_t);
	}
	return res;
}

esp_err_t ac101_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
	esp_err_t res = 0;
	int bits = 0;
	int fmat = 0;
	int sample = 0;
	uint16_t regval;
	switch(iface->bits)						//0x10
	{
	// case AUDIO_HAL_BIT_LENGTH_8BITS:
	// 	bits = BIT_LENGTH_8_BITS;
	// 	break;
	case AUDIO_HAL_BIT_LENGTH_16BITS:
		bits = BIT_LENGTH_16_BITS;
		break;
	case AUDIO_HAL_BIT_LENGTH_24BITS:
		bits = BIT_LENGTH_24_BITS;
		break;
	default:
		bits = BIT_LENGTH_16_BITS;
	}

	switch(iface->fmt)						//0x10
	{
	case AUDIO_HAL_I2S_NORMAL:
		fmat = 0x0;
		break;
	case AUDIO_HAL_I2S_LEFT:
		fmat = 0x01;
		break;
	case AUDIO_HAL_I2S_RIGHT:
		fmat = 0x02;
		break;
	case AUDIO_HAL_I2S_DSP:
		fmat = 0x03;
		break;
	default:
		fmat = 0x00;
		break;
	}

	switch(iface->samples)
	{
	case AUDIO_HAL_08K_SAMPLES:
		sample = SIMPLE_RATE_8000;
		break;
	case AUDIO_HAL_11K_SAMPLES:
		sample = SIMPLE_RATE_11052;
		break;
	case AUDIO_HAL_16K_SAMPLES:
		sample = SIMPLE_RATE_16000;
		break;
	case AUDIO_HAL_22K_SAMPLES:
		sample = SIMPLE_RATE_22050;
		break;
	case AUDIO_HAL_24K_SAMPLES:
		sample = SIMPLE_RATE_24000;
		break;
	case AUDIO_HAL_32K_SAMPLES:
		sample = SIMPLE_RATE_32000;
		break;
	case AUDIO_HAL_44K_SAMPLES:
		sample = SIMPLE_RATE_44100;
		break;
	case AUDIO_HAL_48K_SAMPLES:
		sample = SIMPLE_RATE_48000;
		break;
	default:
		sample = SIMPLE_RATE_44100;
	}
	regval = ac101_read_register(I2S1LCK_CTRL);
	regval &= 0xffc3;
	regval |= (iface->mode << 15);
	regval |= (bits << 4);
	regval |= (fmat << 2);
	res |= ac101_write_register(I2S1LCK_CTRL, regval);
	res |= ac101_write_register(I2S_SR_CTRL, sample);
	return res;
}

esp_err_t ac101_set_output_mute(bool mute)
{
    ESP_LOGW(AC101_TAG, "-----------------------> ac101_set_output_mute");

    return ESP_OK;
}

esp_err_t ac101_set_output_volume(int volume)
{
    ESP_LOGW(AC101_TAG, "++++++++++++++++++++++> ac101_set_output_volume %d", volume);
	esp_err_t res;
	res = ac101_set_earphone_volume(volume);
	res |= ac101_set_speaker_volume(volume);
	return res;
}

esp_err_t ac101_get_output_volume(int *volume)
{
    ESP_LOGW(AC101_TAG, "||||||||||||||||||||||> ac101_get_output_volume");
	*volume = ac101_get_earphone_volume();
    return ESP_OK;
}

