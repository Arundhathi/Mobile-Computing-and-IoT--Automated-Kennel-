/***************************************************************************
 *
 *  	Filename: cirbuff.c
 *      Description:  This source file contains the functions for circular buffer
 *
 *      Author: Vignesh Jayaram
 *      Date: March 17,2016
 *
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "circbuff.h"



/*This functions checks of the buffer is full or not if full it returns 1*/
uint8_t Buffer_Full(cirbuff_t *buff){
	return (buff->no_of_items == buff->length);
}

/*This functionc check if the buffer is empty if empty it returns 1*/
uint8_t Buffer_Empty(cirbuff_t *buff){
	return (buff->no_of_items == 0);
}

/*This function is used to add an element in the buffer*/
states add_item(cirbuff_t *buff, uint8_t data){
	if(Buffer_Full(buff))
		return FULL;

	*(buff->head) = data;
	buff->head++;
	buff->no_of_items++;						//Increaments the number of items after addition to buffer

	if((buff->head) == buff->buffer + buff->length) //This statement prevents buffer overflow by bringing head back
		buff -> head = buff->buffer;			  //back to the first elemnt

	return SUCCESS;
}

/*This function remove an item from the buffer*/
states remove_item(cirbuff_t *buff, uint8_t * removed_item){
	if (Buffer_Empty(buff)){
		return EMPTY ;
	}

	*removed_item = *(buff->tail);
	buff->tail++;
	buff->no_of_items--;							//Decrements the number after removal

	if((buff->tail) == buff->buffer + buff->length) //Brings the tail back to the first element
		buff -> tail = buff->buffer;

	return SUCCESS;
}

/*This function initialises the circular buffer of a particular length*/
void Buffer_Init(cirbuff_t *buff,uint8_t *tx,  uint32_t length){
	//buff->buffer = malloc(length);
	buff->buffer = tx;
	buff->head = buff->buffer;
	buff->tail = buff->buffer;
	buff->length = length;
	buff->no_of_items = 0;
}

void Buffer_Free(cirbuff_t *buff){
	free(buff->buffer);
}

void Buffer_Flush(cirbuff_t *buff){
	uint8_t i,len,value;

	len = buff->no_of_items;

	for(i=0; i<len; i++){
		remove_item(buff, &value);
	}
}

void Buffer_fill(cirbuff_t *buff){
	uint8_t i, len;
	len = buff->length;
	Buffer_Flush(buff);

	for(i=0;i<len;i++){
		add_item(buff,i);
	}
}
