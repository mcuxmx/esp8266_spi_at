#include "driver/spi_interface.h"
#include "eagle_soc.h"
#include "osapi.h"
#include "spi_packet.h"
#include "ringbuf.h"
#include "driver/hspi_slv.h"
#include "at_custom.h"
#include "mem.h"


//#define AT_SPI_BLOCK_USE_LIST 1
#define AT_SPI_RX_BLOCK_USE_LIST 	0
#define AT_SPI_TX_BLOCK_USE_LIST	1
/*----------------------------------------------------------------------------*/
#define AT_SPI_EVENT_RECEIVED 		1
#define AT_SPI_EVENT_SEND_START		2
#define AT_SPI_EVENT_SEND_COMPLETED 3
/*----------------------------------------------------------------------------*/

/*SPI number define*/
#define SPI 			0
#define HSPI			1

/*----------------------------------------------------------------------------*/
#define RD_PIN		5
#define WR_PIN 		4

#if 0
#define LOG_D	os_printf
#else
#define LOG_D(fmt,...)	
#endif
/*----------------------------------------------------------------------------*/

#define SPI_QUEUE_LEN 8
static os_event_t * spiQueue;
#define AT_SPI_TASK_PRIO	USER_TASK_PRIO_2
/*----------------------------------------------------------------------------*/

#define SPI_AT_BUF_MAX      ( 2048 )
static uint8_t at_buff[SPI_AT_BUF_MAX];
static uint32_t at_buff_size = 0;

void spi_slave_isr_handler( void *para );
bool ICACHE_FLASH_ATTR hspi_slave_try_send( void );
void ICACHE_FLASH_ATTR at_custom_uart_rx_buffer_fetch_cb(void);
void ICACHE_FLASH_ATTR at_spi_task_init(void);

#if !AT_SPI_RX_BLOCK_USE_LIST
static uint8_t spiRxBuf[SPI_AT_BUF_MAX];
static uint32_t spiRxIndex = 0;
#endif
static uint32_t spiRxBufReady = 0;


#if !AT_SPI_TX_BLOCK_USE_LIST
static uint8_t atTxBuf[SPI_AT_BUF_MAX];
static uint32_t atTxBufIndex = 0;
static uint32_t atTxBufSize = 0;
static uint32_t atTxBufReady = 0;
#endif

#define AT_RX_BUFF_NUM		5
static uint8_t atRxBuff[AT_RX_BUFF_NUM][SPI_AT_BUF_MAX];
static uint8_t atRxBuffId = 0;
static uint8_t atRxBuffSize = 0;
static uint8_t *pAtBuff = NULL;


#define AT_RX_RING_BUF_SIZE 	( 16 )
static uint8_t atRxRingData[AT_RX_RING_BUF_SIZE];
static struct ringbuf atRxRingBuf;

/************************************************************************************/

/*typedef union
{
	struct
	{
		uint8_t		cmd;
		uint8_t		block_index;
		uint8_t		block_num;
		uint8_t		size;
		uint8_t		packet[HSPI_BLOCK_SIZE-4];
	}
	uint8_t data[HSPI_BLOCK_SIZE];
}hspi_block_t;
*/
static hspi_recv_data_callback_t hspi_recv_data_callback_ptr = NULL;


void at_spi_gpio_init()
{

	//PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	//PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
	GPIO_OUTPUT_SET(WR_PIN, 1);
	GPIO_OUTPUT_SET(RD_PIN, 0);
	//GPIO_OUTPUT_SET(4, 1);
}

/*----------------------------------------------------------------------------*/
void ICACHE_FLASH_ATTR hspi_slave_init(void)
{
	SpiAttr hSpiAttr;
	hSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
	hSpiAttr.speed = 0;
	hSpiAttr.mode = SpiMode_Slave;
	hSpiAttr.subMode = SpiSubMode_0;

	ringbuf_init( &atRxRingBuf, atRxRingData, AT_RX_RING_BUF_SIZE );
	
	at_spi_task_init();
	at_spi_gpio_init();
	
	// Init HSPI GPIO
	WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);//configure io to spi mode
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);//configure io to spi mode
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);//configure io to spi mode
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);//configure io to spi mode

	os_printf("\r\n ============= spi init slave =============\r\n");
	SPIInit(SpiNum_HSPI, &hSpiAttr);
	
	// Set spi interrupt information.
	SpiIntInfo spiInt;
	spiInt.src = (SpiIntSrc_TransDone 
		| SpiIntSrc_WrStaDone 
		|SpiIntSrc_RdStaDone 
		|SpiIntSrc_WrBufDone 
		|SpiIntSrc_RdBufDone);
	spiInt.isrFunc = spi_slave_isr_handler;
	SPIIntCfg(SpiNum_HSPI, &spiInt);
   // SHOWSPIREG(SpiNum_HSPI);
	
	SPISlaveRecvData(SpiNum_HSPI);

	
#if 0	
	uint32_t sndData[8] = { 0 };
	sndData[0] = 0x35343332;
	sndData[1] = 0x39383736;
	sndData[2] = 0x3d3c3b3a;
	sndData[3] = 0x11103f3e;
	sndData[4] = 0x15141312;
	sndData[5] = 0x19181716;
	sndData[6] = 0x1d1c1b1a;
	sndData[7] = 0x21201f1e;

	SPISlaveSendData(SpiNum_HSPI, sndData, 8);
	WRITE_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI), 0x8A);
	WRITE_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI), 0x83);

	GPIO_OUTPUT_SET(RD_PIN, 1);
#endif

	

}

/*----------------------------------------------------------------------------*/
//从slave 往master上写数据
uint8_t ICACHE_FLASH_ATTR hspi_write(uint8_t * data)
{
	uint32_t sndData[8];
	//uint8_t data[32] = {0};
	uint8_t idx = 0;

	//上次发的数据已经被Master读出了吗？如果没读，还不能发送。
    //if (1 == GPIO_INPUT_GET(2))
    //{
	//	 os_printf("gpio2 == 1\r\n");
	//	 return 1; //Err NO. 1 表示缓存中数据还没被读出。
    //}

	memset(sndData, 0, sizeof(sndData));
	//将字节数据转为4字节的数据，因为寄存器是4byte的。
	while( idx < 8 )
    {
		sndData[idx] <<= 0; sndData[idx]|= (data[(idx<<2)+3]&0x000000ff);
		sndData[idx] <<= 8; sndData[idx]|= (data[(idx<<2)+2]&0x000000ff);
		sndData[idx] <<= 8; sndData[idx]|= (data[(idx<<2)+1]&0x000000ff);
		sndData[idx] <<= 8; sndData[idx]|= (data[(idx<<2)+0]&0x000000ff);

		LOG_D("%x ",sndData[idx]);
		idx++;
	}

	LOG_D("\r\n");
	SPISlaveSendData(SpiNum_HSPI, sndData, 8);
	WRITE_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI), 0x8A); //这两句是更新状态寄存器
	WRITE_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI), 0x83);

	GPIO_OUTPUT_SET(RD_PIN, 1);

	return 0; //成功标志。
}

bool at_spi_received = false;
static uint8 spi_data[32] = {0};
/*----------------------------------------------------------------------------*/
void spi_slave_isr_handler( void *para )
{
	uint32		regvalue, calvalue;
	static uint8	state = 0;
	uint32		recv_data, send_data;
	spi_block_t *block;
	
	LOG_D("\r\nspi_slave_isr_handler\r\n");
	//BIT4 表示SPI中断
	if ( READ_PERI_REG( 0x3ff00020 ) & BIT4 )
	{
		LOG_D("BIT4\r\n");
		//following 3 lines is to clear isr signal
        CLEAR_PERI_REG_MASK(SPI_SLAVE(SpiNum_SPI), 0x3ff);
	}
	else if ( READ_PERI_REG( 0x3ff00020 ) & BIT7 ) /* bit7 is for hspi isr, */
	{
		LOG_D("BIT7\r\n");
		int i; //debug
		//记录中断类型
		regvalue = READ_PERI_REG( SPI_SLAVE( SpiNum_HSPI ) );

		//os_printf("spi_slave_isr_handler SPI_SLAVE[0x%08x]\n\r", regvalue);

		//关闭spi中断使能
		CLEAR_PERI_REG_MASK( SPI_SLAVE( SpiNum_HSPI ),
				     SPI_TRANS_DONE_EN |
				     SPI_SLV_WR_STA_DONE_EN |
				     SPI_SLV_RD_STA_DONE_EN |
				     SPI_SLV_WR_BUF_DONE_EN |
				     SPI_SLV_RD_BUF_DONE_EN );
		//将SPI从机恢复到可通信状态，准备下一次通信
		SET_PERI_REG_MASK( SPI_SLAVE( SpiNum_HSPI ), SPI_SYNC_RESET );
		//清楚中断标志
		CLEAR_PERI_REG_MASK( SPI_SLAVE( SpiNum_HSPI ),
				     SPI_TRANS_DONE |
				     SPI_SLV_WR_STA_DONE |
				     SPI_SLV_RD_STA_DONE |
				     SPI_SLV_WR_BUF_DONE |
				     SPI_SLV_RD_BUF_DONE );
		//打开spi中断
		SET_PERI_REG_MASK( SPI_SLAVE( SpiNum_HSPI ),
				   SPI_TRANS_DONE_EN |
				   SPI_SLV_WR_STA_DONE_EN |
				   SPI_SLV_RD_STA_DONE_EN |
				   SPI_SLV_WR_BUF_DONE_EN |
				   SPI_SLV_RD_BUF_DONE_EN );

		//主机写入，从机接收处理程序
		if ( regvalue & SPI_SLV_WR_BUF_DONE )
		{
			LOG_D("SPI_SLV_WR_BUF_DONE\r\n");
			GPIO_OUTPUT_SET( WR_PIN, 0 ); //Slave 收到数据将中断线拉低
			int idx = 0;

			memset(spi_data,0,32);
			while ( idx < 8 )
			{
				recv_data = READ_PERI_REG( SPI_W0( SpiNum_HSPI ) + (idx << 2) );
				spi_data[idx << 2]		    = recv_data & 0xff;
				spi_data[(idx << 2) + 1]	= (recv_data >> 8) & 0xff;
				spi_data[(idx << 2) + 2]	= (recv_data >> 16) & 0xff;
				spi_data[(idx << 2) + 3]	= (recv_data >> 24) & 0xff;
				idx++;
			}

		#if 0
			LOG_D("rx:");
			for( i = 0; i < 32; i++ )
            {
				LOG_D("0x%02x ",spi_data[i]);
			}
			LOG_D("\r\n");
		#endif	

		#if AT_SPI_RX_BLOCK_USE_LIST
			spiRxBufReady = true;
		#else
			block = (spi_block_t *)&spi_data;
			
			if( block->block_index == block->block_num )
			{
				spiRxBufReady = true;
			}
			else
			{
				spiRxBufReady = false;
				if( block->block_index == 1 )
				{
					spiRxIndex = 0;
				}
			}
			memcpy( spiRxBuf + spiRxIndex, block->packet, block->size );
			spiRxIndex += block->size;
			spiRxBuf[spiRxIndex] = '\0';
		#endif
		
			/* add system_os_post here */
			//system_os_post(USER_TASK_PRIO_0,	MOSI,	‘a’);
			if( spiRxBufReady )
			{
				system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_RECEIVED, 0 );
			}
		
			GPIO_OUTPUT_SET( WR_PIN, 1 ); //Slave处理完收到的数据 将中断再次拉高，通知Master 可以再次写了。
			//at_custom_uart_rx_buffer_fetch_cb();
		}

		//主机读取，从机发送处理程序
		if ( regvalue & SPI_SLV_RD_BUF_DONE )
		{
			LOG_D("SPI_SLV_RD_BUF_DONE\r\n");
			/* it is necessary to call GPIO_OUTPUT_SET(2, 1), when new data is preped in SPI_W8-15 and needs to be sended. */
			
			GPIO_OUTPUT_SET( RD_PIN, 0 );//Master读完将中断线恢复为低电平。
			system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_SEND_COMPLETED, 0 );
			
			/*
			 * add system_os_post here
			 * system_os_post(USER_TASK_PRIO_1,WR_RD,regvalue);
			 */
		}
	}
	else if ( READ_PERI_REG( 0x3ff00020 ) & BIT9 ) /* bit7 is for i2s isr, */
	{
		LOG_D("BIT9\r\n");
	}
}



bool ICACHE_FLASH_ATTR hspi_slave_try_send( void )
{
    spi_block_t *block;

    if (1 == GPIO_INPUT_GET(RD_PIN))
    {
		 LOG_D("gpio2 == 1\r\n");
		 return false; //Err NO. 1 表示缓存中数据还没被读出。
    }

    block = spi_tx_block_pop();
    if( block != NULL )
    {
        hspi_write( (uint8_t *)block );
    }
	return true;
}

int32 ICACHE_FLASH_ATTR hspi_load_data(const uint8* data, uint32 len )
{
#if AT_SPI_TX_BLOCK_USE_LIST

	uint8_t *pAtBuff = atRxBuff[atRxBuffId]; 
	

	os_memcpy( pAtBuff + atRxBuffSize, data, len );
	atRxBuffSize += len;
	if( data[len - 1] == '\n' )
	{
		pAtBuff[atRxBuffSize] = '\0';
		ringbuf_put( &atRxRingBuf, atRxBuffId );
		
		atRxBuffId ++;
		if( atRxBuffId >= AT_RX_BUFF_NUM ){
			atRxBuffId = 0;
		}
		atRxBuffSize = 0;
		system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_SEND_START, 0 );
	}

#else
	if( atTxBufReady == 0 )
	{
		atTxBufIndex = 0;
		if( atTxBufSize >= SPI_AT_BUF_MAX  ){
			atTxBufSize = 0;
		}
		
		memcpy( atTxBuf + atTxBufSize, data, len );
		atTxBufSize += len;
		if( atTxBuf[atTxBufSize-1] == '\n' )
		{
			atTxBufReady = 1;
			atTxBufIndex = 0;
			system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_SEND_START, atTxBufSize );
		}
	}
	else{
		LOG_D("tx buf busy\n");
	}
	
#endif	
	
}


bool hspi_register_recv_cb(hspi_recv_data_callback_t cb)
{
	hspi_recv_data_callback_ptr = cb;

	return TRUE;
}

void ICACHE_FLASH_ATTR at_custom_uart_rx_buffer_fetch_cb(void)
{
	uint8_t *buf;
	uint32_t buf_size;
	
	LOG_D("at_custom_uart_rx_buffer_fetch_cb\r\n");
	if(hspi_recv_data_callback_ptr && spiRxBufReady )
	{
	#if AT_SPI_RX_BLOCK_USE_LIST
		buf = at_buff;
        buf_size = spi_rx_buffer_check(buf);
        
	#else
		buf = spiRxBuf;
		buf_size = spiRxIndex;
		spiRxIndex = 0;
	#endif	

		if( buf_size > 0 )
        {
        	LOG_D("at rx:%d\r\n", buf_size);
            if (hspi_recv_data_callback_ptr((uint8*)buf, buf_size) == buf_size)
            {
                LOG_D("at handle success\r\n");
            }
        }
	}
}

#ifdef AT_SPI_DEBUG
void ICACHE_FLASH_ATTR at_spi_check_cb(void *arg)
{
	//TRIG_TOHOST_INT();
	if( spiRxBufReady )
	{
		LOG_D("at_spi_received:%d\r\n",spiRxBufReady);
		//sdio_load_data("esp8266\r\n",os_strlen("esp8266\r\n"));
		at_custom_uart_rx_buffer_fetch_cb();
	}
}
#endif

#if !AT_SPI_TX_BLOCK_USE_LIST
static void at_spi_send_block( void )
{
	if( atTxBufReady == 0 )
	{
		return;
	}
	
	if( atTxBufSize == 0 )
	{
		atTxBufIndex = 0;
		atTxBufReady = 0;
		return;
	}

	if( atTxBufIndex >= atTxBufSize )
	{
		atTxBufIndex = 0;
		atTxBufSize = 0;
		atTxBufReady = 0;
		return;
	}

	#define BLOCK_DATA_SIZE	28
	spi_block_t block;
	memset( &block, 0, sizeof(spi_block_t) );


	block.cmd = SPI_CMD_AT;
	block.block_num = atTxBufSize / BLOCK_DATA_SIZE + 1;
	block.block_index = atTxBufIndex / BLOCK_DATA_SIZE + 1;
	if( atTxBufIndex + BLOCK_DATA_SIZE <= atTxBufSize )
	{
		block.size = BLOCK_DATA_SIZE;
	}
	else
	{
		block.size = atTxBufSize - atTxBufIndex;
	}
	memcpy(block.packet, atTxBuf+atTxBufIndex, block.size);
	atTxBufIndex += block.size;
	
    hspi_write( (uint8_t *)&block );

	if( atTxBufIndex >= atTxBufSize )
	{
		atTxBufIndex = 0;
		atTxBufSize = 0;
		atTxBufReady = 0;
	}
	
    
}
#endif

void ICACHE_FLASH_ATTR at_spi_task(os_event_t *e)
{
	int id;
    uint8 data;
	bool bRes;
	
    switch(e->sig)
	{
	case AT_SPI_EVENT_RECEIVED:
		LOG_D("AT_SPI_EVENT_RECEIVED\n");
	  #if AT_SPI_RX_BLOCK_USE_LIST
		spi_add_rx_block( (uint8_t *)&spi_data );
	  #endif
		at_custom_uart_rx_buffer_fetch_cb();
      	break;
	case AT_SPI_EVENT_SEND_START :
		LOG_D("AT_SPI_EVENT_SEND_START\n");
	  #if AT_SPI_TX_BLOCK_USE_LIST
		while( (id = ringbuf_get( &atRxRingBuf )) != -1 )
		{
			bRes = spi_tx_buff_to_list( atRxBuff[id], strlen(atRxBuff[id]) );
			if(bRes == false)
			{
				hspi_load_data("overflow\n", strlen("overflow\n"));
				break;
			}
		}
    	bRes = hspi_slave_try_send();
		if( bRes == false ){
			system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_SEND_COMPLETED, 0 );
		}
	  #else
		at_spi_send_block();
	  #endif
		break;
	case AT_SPI_EVENT_SEND_COMPLETED:
		LOG_D("AT_SPI_EVENT_SEND_START\n");
	  #if AT_SPI_TX_BLOCK_USE_LIST
		bRes = hspi_slave_try_send();
		if( bRes == false ){
			system_os_post( AT_SPI_TASK_PRIO, AT_SPI_EVENT_SEND_COMPLETED, 0 );
		}
	  #else
		at_spi_send_block();
	  #endif
		break;		
   	default:
    	break;
    }
}

void ICACHE_FLASH_ATTR at_spi_task_init(void)
{
    spiQueue = (os_event_t*)os_malloc(sizeof(os_event_t)*SPI_QUEUE_LEN);
    system_os_task(at_spi_task,AT_SPI_TASK_PRIO,spiQueue,SPI_QUEUE_LEN);
}



