#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX 5

uint8_t queue[MAX];
int front = 0;
int rear = -1;
uint8_t itemCount = 0;

uint8_t peek() {
   return queue[front];
}

bool isEmpty() {
   return itemCount == 0;
}

bool isFull() {
   return itemCount == MAX;
}

uint8_t size() {
   return itemCount;
}  

void enqueue(uint8_t data) {
   if(!isFull()) {
      if(rear == MAX-1) {
         rear = -1;            
      }       

      queue[++rear] = data;
      itemCount++;
   }
}

uint8_t dequeue() {
   uint8_t data = queue[front++];
   if(front == MAX) {
      front = 0;
   }
   itemCount--;
   return data;  
}

#endif 

