/*
 * memsic_type.h
 *
 *  Created on: Sep 27, 2016
 *      Author: mason
 */

#ifndef MEMSIC_TYPE_H_
#define MEMSIC_TYPE_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <errno.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <linux/usbdevice_fs.h>

#define MAXQUEUE 500

/*
 * circular queue
 */
typedef struct queue_tag
{
	int count;
	int front;
	int rear;
	char entry[MAXQUEUE];
} QUEUE_TYPE;

/*
 * MEMSIC packet
 */

typedef struct xbow_packet
{
	unsigned short packet_type;
	char
	length;
	unsigned short crc;
	char
	data[256];
} XBOW_PACKET;


/*******************************************************************************
 * FUNCTION:
process_xbow_packet looks for packets in a queue
 * ARGUMENTS: queue_ptr: is pointer to queue to process
 *
result: will contain the parsed info when return value is 1
 * RETURNS:
0 when failed.
 *
1 when successful
 *******************************************************************************/

int process_xbow_packet(QUEUE_TYPE *queue_ptr, XBOW_PACKET *result);
/*******************************************************************************
 * FUNCTION: calcCRC calculates a 2-byte CRC on serial data using
 * CRC-CCITT 16-bit standard maintained by the ITU
 *
(International Telecommunications Union).
 * ARGUMENTS: queue_ptr is pointer to queue holding area to be CRCed
startIndex is offset into buffer where to begin CRC calculation
num is offset into buffer where to stop CRC calculation
 * RETURNS:
2-byte CRC
 *******************************************************************************/
unsigned short calcCRC(QUEUE_TYPE *queue_ptr, unsigned int startIndex, unsigned int num);

/*******************************************************************************
 * FUNCTION:
Initialize - initialize the queue
 * ARGUMENTS: queue_ptr is pointer to the queue
 *******************************************************************************/
void Initialize(QUEUE_TYPE *queue_ptr);

/*******************************************************************************
 * FUNCTION:
AddQueue - add item in front of queue
 * ARGUMENTS: item holds item to be added to queue
 *
queue_ptr is pointer to the queue
 * RETURNS:
returns 0 if queue is full. 1 if successful
 *******************************************************************************/
int AddQueue(char item, QUEUE_TYPE *queue_ptr);

/*******************************************************************************
 * FUNCTION:
DeleteQeue - return an item from the queue
 * ARGUMENTS: item will hold item popped from queue
 *
queue_ptr is pointer to the queue
 * RETURNS:
returns 0 if queue is empty. 1 if successful
 *******************************************************************************/
int DeleteQueue(char *item, QUEUE_TYPE *queue_ptr);

/*******************************************************************************
 * FUNCTION:
peekByte returns 1 byte from buffer without popping
 * ARGUMENTS: queue_ptr is pointer to the queue to return byte from
 *
index is offset into buffer to which byte to return
 * RETURNS: 1 byte
 * REMARKS: does not do boundary checking. please do this first
 *******************************************************************************/
char peekByte(QUEUE_TYPE *queue_ptr, unsigned int index);

/*******************************************************************************
 * FUNCTION:
peekWord returns 2-byte word from buffer without popping
 * ARGUMENTS: queue_ptr is pointer to the queue to return word from
 *
index is offset into buffer to which word to return
 * RETURNS: 2-byte word
 * REMARKS: does not do boundary checking. please do this first
 *******************************************************************************/
unsigned short peekWord(QUEUE_TYPE *queue_ptr, unsigned int index);

/*******************************************************************************
 * FUNCTION:
Pop - discard item(s) from queue
 * ARGUMENTS: queue_ptr is pointer to the queue
 *
numToPop is number of items to discard
 * RETURNS:
return the number of items discarded
 *******************************************************************************/
int Pop(QUEUE_TYPE *queue_ptr, int numToPop);

/*******************************************************************************
 * FUNCTION:
Size
 * ARGUMENTS: queue_ptr is pointer to the queue
 * RETURNS:
return the number of items in the queue
 *******************************************************************************/
int Size(QUEUE_TYPE *queue_ptr);

/*******************************************************************************
 * FUNCTION:
Empty
 * ARGUMENTS: queue_ptr is pointer to the queue
 * RETURNS:
return 1 if empty, 0 if not
 *******************************************************************************/
int Empty(QUEUE_TYPE *queue_ptr);

/*******************************************************************************
 * FUNCTION:
Full
 * ARGUMENTS: queue_ptr is pointer to the queue
 * RETURNS:
return 1 if full, 0 if not full
 *******************************************************************************/
int Full(QUEUE_TYPE *queue_ptr);
#endif /* MEMSIC_TYPE_H_ */
