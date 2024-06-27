#ifndef LINKED_LIST 
#define LINKED_LIST
// the "define" is used by the compiler/linking stage as token

#include <stdlib.h>
#include <errno.h>
#include "defs.h"

// MAKE A LINKED LIST OF VECTORS TO REPRESENT ARM SEGMENTS

struct node {
  struct Vector* vector; 
  struct node* previous;
  struct node* next; // pointer to the next node in the linked list
}; 

struct node* getNewNode(struct Vector* vector);
void insertAtHead(struct Vector* vector); // create and insert a new node at the head of the list... 
void insertAtTail(struct Vector* vector); // create and insert a new node at the tail of the list... 
int printList(struct node* head); // prints all nodes in list from head to tail, param is the head of the list, returns status code 


#endif // LINKED_LIST

