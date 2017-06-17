/***************************************************************************
 *
 *  	Filename: cirbuff.h
 *      Description:  This header file contains the functions for circular buffer
 *
 *      Author:Vignesh Jayaram
 *      Date: March 17,2017
 *
 *      1. struct cirbuff_t
 *      > This structure contains members of the circular buffer
 *
 *      2. enum states
 *      > Enum contains the various states the buffer can exist in
 *
 *      3. Buffer_Full
 *      > function to check if the buffer is full to its capacity
 *
 *      4. Buffer_Empty
 *      > function to check if buffer has no elements in it
 *
 *      5. add_item
 *      > This function first checks if buffer is full.
 *      > If buffer is full ,then function returns 1.
 *		> If there is space in buffer, item is added to the buffer.
 *		> The function also checks for wrapping functionality.
 *		> Function returns SUCCESS i.e 3 if item is successfully added.
 *
 *		6. remove_item
 *		> This function first checks if buffer is empty.
 *      > If buffer is empty ,then function returns 2.
 *		> If there are elements in buffer, item is removed from the buffer.
 *		> The function also checks for wrapping functionality.
 *		> Function returns SUCCESS if item is successfully removed.
 *
 *		7. Buffer_Init
 *		> This function dynamically allocates space for the buffer on heap.
 *
 *		8. Buffer_Flush
 *		> This function uses remove_item function to completely empty the buffer.
 *
 *		9. Buffer_fill
 *		> This function uses add_item function to fill the buffer completely.
 *
 *		10. Buffer_free
 *		> This function is used to destroy the memory allocated for buffer.
 *
 *
 *
 ****************************************************************************/

#ifndef SRC_CIRCBUFF_H_
#define SRC_CIRCBUFF_H_

/*This structure contains the variables for the buffer*/
typedef struct {
	uint8_t* buffer;		//pointer to the circular buffer
	uint8_t* head;			//pointer to the head of the buffer
	uint8_t* tail;			//pointer to the tail of the buffer
	uint32_t length;		//Total length of the buffer
	uint32_t no_of_items;		//number of items present in the buffer
}cirbuff_t;


/*Enumeration for the state of the buffer*/
typedef enum {
	FULL,
	EMPTY,
	SUCCESS,
	FAIL
}states;



/*Prototypes of the functions*/
uint8_t Buffer_Full(cirbuff_t *buff);
uint8_t Buffer_Empty(cirbuff_t *buff);
states add_item(cirbuff_t *buff, uint8_t data);
states remove_item(cirbuff_t *buff, uint8_t *removed_item);
void Buffer_Init(cirbuff_t *buff,uint8_t *tx, uint32_t length);
void Buffer_Flush(cirbuff_t *buff);
void Buffer_fill(cirbuff_t *buff);
void Buffer_Free(cirbuff_t *buff);


#endif
