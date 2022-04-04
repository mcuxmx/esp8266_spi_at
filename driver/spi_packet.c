
#include "spi_packet.h"
#include <stdlib.h>
#include <string.h>

#ifdef ESP8266_WIFI
#include "c_types.h"
#include "mem.h"
#else
#include <stdint.h>
#include <stdbool.h>
#endif


static uint8_t TX_DataBuffer[TX_BUFFER_SIZE_MAX];
static uint8_t RX_DataBuffer[RX_BUFFER_SIZE_MAX];


typedef struct
{
	uint8_t 	cmd;
	uint16_t	size;
	uint8_t		*data;
}spi_buffer_t;
static spi_buffer_t	txBuf;
static spi_buffer_t rxBuf;




#ifdef ESP8266_WIFI
#include "mem.h"
#undef block_malloc
#define block_malloc	os_malloc
#undef block_free
#define block_free		os_free
#else
#ifndef block_malloc
#define block_malloc	malloc
#endif
#ifndef block_free
#define block_free		free
#endif
#endif /* #ifdef ESP8266_WIFI */

LIST(tx_list);
LIST(rx_list);


void spi_block_init( void )
{
	list_init(tx_list);
	memset( &txBuf, 0, sizeof(spi_buffer_t));
	txBuf.data = TX_DataBuffer;

	list_init(rx_list);
	memset( &rxBuf, 0, sizeof(spi_buffer_t));
	rxBuf.data = RX_DataBuffer;
}

bool spi_block_valid(  spi_block_t *p )
{
	if( p == NULL ){
		return false;
	}


	if( p->size > SPI_BLOCK_SIZE-4 ){
		return false;
	}

	return true;
}



bool spi_block_add(   list_t list, uint8_t *data )
{

	spi_block_t *block;
	spi_block_list_t *buf;


	block = (spi_block_t *)data;
	if( spi_block_valid(block) )
	{
		buf = (spi_block_list_t *)block_malloc(sizeof(spi_block_list_t));
        if( buf == NULL )
        {
            return false;
        }
		memcpy( &buf->block, block, sizeof(spi_block_t) );
		list_add(list, buf);

		return true;
	}

	return false;
}

bool spi_add_rx_block(  uint8_t * buf)
{
 	return spi_block_add( rx_list, buf );
}

bool spi_add_tx_block(  uint8_t * buf)
{
 	return spi_block_add( tx_list, buf );
}


bool spi_buff_to_list( list_t list, const uint8_t *data, uint16_t size )
{
    uint8_t block_num = 0;
    uint8_t block_index = 0;
    uint16_t index = 0;
    spi_block_t block;
    
    block_num = ( size + SPI_BLOCK_DATA_SIZE - 1 ) / SPI_BLOCK_DATA_SIZE;
    while( index < size )
    {
        block_index ++;

        memset( &block, 0, sizeof(spi_block_t) );
        block.cmd = SPI_CMD_AT;
        block.block_index = block_index;
        block.block_num = block_num;

        if( index + SPI_BLOCK_DATA_SIZE >= size )
        {
            block.size = size - index;
        }
        else
        {
            block.size = SPI_BLOCK_DATA_SIZE;
        }
        memcpy( block.packet, data + index, block.size );

        if( spi_block_add( list, (uint8_t *)&block ) )
        {
            index += block.size;
        }
        else
        {
            return false;
        }
    }
    
    return true;
}

bool spi_rx_buff_to_list( const uint8_t *data, uint16_t size )
{
    return spi_buff_to_list( rx_list, data, size );
}

bool spi_tx_buff_to_list( const uint8_t *data, uint16_t size )
{
    return spi_buff_to_list( tx_list, data, size );
}

int spi_buffer_check( list_t list, spi_buffer_t *buf,  uint8_t *outBuf )
{
	int size = 0;

	spi_block_list_t *p;
	p = (spi_block_list_t *)list_pop( list );

	while( p != NULL )
	{
		if( buf->size == 0 ){
			buf->cmd = p->block.cmd;
		}

		if( buf->cmd != p->block.cmd ){ // 上一次的缓存不完整
			buf->size = 0;
		}

		memcpy( buf->data + buf->size, p->block.packet, p->block.size );
		buf->size += p->block.size;

		if( p->block.block_index == p->block.block_num ){
			size = buf->size;
            buf->size = 0;
			memcpy( outBuf, buf->data, size );
			block_free(p);
            
			break;
		}
		else{
			block_free(p);
		}
		p = (spi_block_list_t *)list_pop( list );
	}

	return size;

}

int spi_tx_buffer_check( uint8_t *outBuf )
{
	return spi_buffer_check( tx_list, &txBuf, outBuf );

}


int spi_rx_buffer_check( uint8_t *outBuf )
{
	return spi_buffer_check( rx_list, &rxBuf, outBuf );
}


spi_block_t * spi_block_pop( list_t list )
{
    static spi_block_t block;

    spi_block_list_t *p;
	p = (spi_block_list_t *)list_pop( list );
	if( p != NULL )
    {
        memcpy( &block, &p->block, sizeof(spi_block_t) );
        block_free(p);

        return &block;
	}
	else
    {
        return NULL;
	}
}

spi_block_t * spi_rx_block_pop( void )
{
    return spi_block_pop( rx_list );
}


spi_block_t * spi_tx_block_pop( void )
{
    return spi_block_pop( tx_list );
}
