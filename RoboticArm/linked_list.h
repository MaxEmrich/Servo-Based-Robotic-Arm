#ifndef LINKED_LIST 
#define LINKED_LIST
// the "define" is used by the compiler/linking stage as token

#include <stdlib.h>
#include <errno.h>
#include "defs.h"

// MAKE A LINKED LIST OF VECTORS TO REPRESENT ARM SEGMENTS

struct head {
  int isHeadNode = 1; // sets isHeadNode to true 
  struct node* first_node;
};

struct node {
  struct Vector* vector; 
  struct node* next; // pointer to the next node in the linked list
}; 

struct node* getNewNode(struct Vector* vector, struct node* nextNode = NULL);

#endif // LINKED_LIST

