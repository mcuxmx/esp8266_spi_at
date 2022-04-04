
#ifndef __SPI_PACKET_H_
#define __SPI_PACKET_H_
#include "list.h"


/************************************************************************************/
#define RX_BUFFER_SIZE_MAX		( 2048 )
#define TX_BUFFER_SIZE_MAX		( 2048 )

#define SPI_BLOCK_SIZE			( 32 )
#define SPI_BLOCK_DATA_SIZE		( SPI_BLOCK_SIZE - 4 )

#pragma pack(1)
typedef struct
{
	uint8_t		cmd;
	uint8_t		block_num;					// 总包数，1~N
	uint8_t		block_index;				// 当前包序号，从1~blocknum
	uint8_t		size;						// 当前数据包的有效数据长度，小于等于SPI_BLOCK_SIZE-4
	uint8_t		packet[SPI_BLOCK_DATA_SIZE];	// 有效数据内容
}spi_block_t;
#pragma pack()


struct spi_block_list
{
	struct spi_block_list 	*next;
	spi_block_t				block;
};

typedef struct spi_block_list spi_block_list_t;


#define CMD_NULL		0
#define SPI_CMD_AT      1


void spi_block_init( void );
bool spi_add_rx_block(  uint8_t * buf);
bool spi_add_tx_block(  uint8_t * buf);
bool spi_rx_buff_to_list( const uint8_t *data, uint16_t size );
bool spi_tx_buff_to_list( const uint8_t *data, uint16_t size );
int spi_tx_buffer_check( uint8_t *outBuf );
int spi_rx_buffer_check( uint8_t *outBuf );
spi_block_t * spi_rx_block_pop( void );
spi_block_t * spi_tx_block_pop( void );

#endif

