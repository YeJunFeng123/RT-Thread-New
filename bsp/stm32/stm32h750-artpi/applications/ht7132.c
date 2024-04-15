#include "ht7132.h"
#include "drv_spi.h"
#include <board.h>
#include <rtthread.h>
#include <drv_log.h>
#define THREAD_NAME "HT7132"
#define THREAD_STACKSIZE 1024
#define THREAD_PRIORITY 16
#define THREAD_TICKS 10
#define DEVICE_NAME "spi4"
#define SPI4_CS_PIN GET_PIN(E,4)
#define LOG_TAG             "drv.spi_ht7132"
static rt_thread_t _thread;
static struct rt_spi_device* _spi4;

static void _ht7132_thread_entry(void* param)
{
    #define DeviceID_ADDRESS  0x018f//RN8302B Device ID
    rt_uint8_t sendBuf[10],recvBuf[18];      
    uint32_t temp = 0;
    rt_uint8_t i;
    sendBuf[0]=DeviceID_ADDRESS;
    sendBuf[1]=DeviceID_ADDRESS>>4;
    rt_kprintf("ht7132 task enter success\r\n",*(int*)recvBuf);
    /*
    rt_spi_recv(_spi4, recvBuf, 18);
    for(i=0; i<18; i++)
    {
        rt_kprintf("recvBuf = %x \r\n",recvBuf[i]);
    }
    */
    while (1)
    {
    
        rt_spi_send_then_recv(_spi4,sendBuf,2,recvBuf,4);
        rt_kprintf("Device_ID=%x(hex)\r\n",*(int*)recvBuf);
        rt_memset(recvBuf,0,10);
        
        rt_thread_mdelay(1000);
    }
}


rt_err_t HT7132_Init(void)
{
    struct rt_spi_configuration cfg;
    
    _thread = rt_thread_create(THREAD_NAME,_ht7132_thread_entry,RT_NULL,THREAD_STACKSIZE,THREAD_PRIORITY,THREAD_TICKS);
	if(_thread==RT_NULL)
	{	
		return -RT_ERROR;
	}
    
    rt_hw_spi_device_attach("spi4", "spi40",SPI4_CS_PIN);
    _spi4 = (struct rt_spi_device*)rt_device_find("spi40");
    _spi4->bus->owner=_spi4;
    _spi4->config=cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.max_hz = 2.5 * 1000 * 1000;
    rt_spi_configure( _spi4, &cfg);
    if(rt_thread_startup(_thread)!=RT_EOK)
	{
		return -RT_ERROR;
	}
	return RT_EOK;
}
INIT_APP_EXPORT(HT7132_Init);



