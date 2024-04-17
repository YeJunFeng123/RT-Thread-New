#include "rn8306.h"
#include "drv_spi.h"
#include <board.h>
#include <rtthread.h>
#include <drv_log.h>
#define THREAD_NAME "RN8306"
#define THREAD_STACKSIZE 1024
#define THREAD_PRIORITY 16
#define THREAD_TICKS 10
#define DEVICE_NAME "spi3"
#define SPI3_CS_PIN GET_PIN(A,15)
#define LOG_TAG             "drv.spi_rn8306"
static rt_thread_t _thread;
static struct rt_spi_device* _spi3;
static rt_err_t _rn8306_write_cmd(const rt_uint8_t cmd)
{
    rt_size_t len;

    rt_pin_write(SPI3_CS_PIN, PIN_LOW);

    len = rt_spi_send(_spi3, &cmd, 1);

    if (len != 1)
    {
        LOG_I("lcd_write_cmd error. %d", len);
        return -RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}

static rt_err_t  _rn8306_write_data(const rt_uint8_t data)
{
    rt_size_t len;

    rt_pin_write(SPI3_CS_PIN, PIN_HIGH);

    len = rt_spi_send(_spi3, &data, 1);

    if (len != 1)
    {
        LOG_I("lcd_write_data error. %d", len);
        return -RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}
static void _rn8306_thread_entry(void* param)
{
    #define DeviceID_ADDRESS  0x018f//RN8302B Device ID
    rt_uint8_t sendBuf[10],recvBuf[10];      
    uint32_t temp = 0;
    sendBuf[0]=DeviceID_ADDRESS;
    sendBuf[1]=DeviceID_ADDRESS>>4;
    while (1)
    {
        rt_spi_send_then_recv(_spi3,sendBuf,2,recvBuf,4);
//        //rt_kprintf("Device_ID=%x(hex)\r\n",*(int*)recvBuf);
//        rt_memset(recvBuf,0,10);
        rt_thread_mdelay(1000);
    }
}
rt_err_t RN8036_Init(void)
{
    struct rt_spi_configuration cfg;
    _thread = rt_thread_create(THREAD_NAME,_rn8306_thread_entry,RT_NULL,THREAD_STACKSIZE,THREAD_PRIORITY,THREAD_TICKS);
	if(_thread==RT_NULL)
	{	
		return -RT_ERROR;
	}

    rt_hw_spi_device_attach("spi3", "spi30",SPI3_CS_PIN);
    _spi3 = (struct rt_spi_device*)rt_device_find("spi30");
    _spi3->bus->owner=_spi3;
    _spi3->config=cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 2.5 * 1000 * 1000;
    rt_spi_configure( _spi3, &cfg);
    if(rt_thread_startup(_thread)!=RT_EOK)
	{
		return -RT_ERROR;
	}
	return RT_EOK;
}
INIT_APP_EXPORT(RN8036_Init);
