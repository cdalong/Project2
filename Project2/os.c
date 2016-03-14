
#include <string.h>
//#include "os.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <assert.h>

/**
 * \file active.c
 * \brief A Skeleton Implementation of an RTOS
 * 
 * \mainpage A Skeleton Implementation of a "Full-Served" RTOS Model
 * This is an example of how to implement context-switching based on a 
 * full-served model. That is, the RTOS is implemented by an independent
 * "kernel" task, which has its own stack and calls the appropriate kernel 
 * function on behalf of the user task.
 *
 * \author Dr. Mantis Cheng
 * \date 29 September 2006
 *
 * ChangeLog: Modified by Alexander M. Hoole, October 2006.
 *			  -Rectified errors and enabled context switching.
 *			  -LED Testing code added for development (remove later).
 *
 * \section Implementation Note
 * This example uses the ATMEL AT90USB1287 instruction set as an example
 * for implementing the context switching mechanism. 
 * This code is ready to be loaded onto an AT90USBKey.  Once loaded the 
 * RTOS scheduling code will alternate lighting of the GREEN LED light on
 * LED D2 and D5 whenever the corresponding PING and PONG tasks are running.
 * (See the file "cswitch.S" for details.)
 */

typedef void (*voidfuncptr) (void);      /* pointer to void f(void) */ 

#define WORKSPACE     256
#define MAXPROCESS   8
typedef unsigned int PID;        // always non-zero if it is valid
typedef unsigned int TICK;
typedef unsigned int MUTEX;      // always non-zero if it is valid
typedef unsigned int EVENT;      // always non-zero if it is valid
typedef unsigned char PRIORITY;
const unsigned int PT;
const unsigned int PPP[]; 
static uint8_t slot_task_finished = 0; 
int interrupt_handles = 0;// indicates if peridoic task has run



//typedef unsigned char PRIORITY;


/*===========
  * RTOS Internal
  *===========
  */

/**
  * This internal kernel function is the context switching mechanism.
  * It is done in a "funny" way in that it consists two halves: the top half
  * is called "Exit_Kernel()", and the bottom half is called "Enter_Kernel()".
  * When kernel calls this function, it starts the top half (i.e., exit). Right in
  * the middle, "Cp" is activated; as a result, Cp is running and the kernel is
  * suspended in the middle of this function. When Cp makes a system call,
  * it enters the kernel via the Enter_Kernel() software interrupt into
  * the middle of this function, where the kernel was suspended.
  * After executing the bottom half, the context of Cp is saved and the context
  * of the kernel is restore. Hence, when this function returns, kernel is active
  * again, but Cp is not running any more. 
  * (See file "switch.S" for details.)
  */
void Task_Terminate(void);
extern void CSwitch();
extern void Exit_Kernel();    /* this is the same as CSwitch() */

/* Prototype */
//void Task_Terminate(void);

/** 
  * This external function could be implemented in two ways:
  *  1) as an external function call, which is called by Kernel API call stubs;
  *  2) as an inline macro which maps the call into a "software interrupt";
  *       as for the AVR processor, we could use the external interrupt feature,
  *       i.e., INT0 pin.
  *  Note: Interrupts are assumed to be disabled upon calling Enter_Kernel().
  *     This is the case if it is implemented by software interrupt. However,
  *     as an external function call, it must be done explicitly. When Enter_Kernel()
  *     returns, then interrupts will be re-enabled by Enter_Kernel().
  */ 
extern void Enter_Kernel();

#define Disable_Interrupt()		asm volatile ("cli"::)
#define Enable_Interrupt()		asm volatile ("sei"::)
  

/**
  *  This is the set of states that a task can be in at any given time.
  */
typedef enum process_states 
{ 
   DEAD = 0, 
   READY, 
   RUNNING,
   BLOCKED,
   SUSP_BLOCKED,
   SUSP_READY 
} PROCESS_STATES;

/**
  * This is the set of kernel requests, i.e., a request code for each system call.
  */
typedef enum kernel_request_type 
{
   NONE = 0,
   CREATE,
   NEXT,
   SLEEP,
   SUSPEND,
   BLOCK,
   UNBLOCK,
   GET_ARG,
   RESUME,
   YIELD,
   TERMINATE
} KERNEL_REQUEST_TYPE;

typedef enum { 
  false, 
  true 
} bool;

/**
  * Each task is represented by a process descriptor, which contains all
  * relevant information about this task. For convenience, we also store
  * the task's stack, i.e., its workspace, in here.
  */
typedef struct processDescriptor {
   unsigned char *sp;                   // Stack pointer into the "workSpace"
   unsigned char workSpace[WORKSPACE];  // Memory available to the process
   TICK ticks;       
   TICK period;   
   TICK remaining; // Ticks remaining to sleep
   PROCESS_STATES state;                // Current state of process
   PRIORITY original;                   // Priority of created function
   PRIORITY inherited;                  // Inherited priority (to counter priority inversion)
   voidfuncptr  code;                   // Function to be executed as a task
   KERNEL_REQUEST_TYPE request;         // State change request to be conducted by kernel when appropriate
   PID p;                               // Process identification number
   struct processDescriptor* next;      // Next process holding this task
   int PID;
   int susp;                            //Boolean for indicating suspension
   int arg;
} processDescriptor;

typedef struct sleep_node {

	processDescriptor* task;
	struct sleep_node* next;

} sleep_node;

typedef struct mutex_node{
	
	processDescriptor* task;
	struct mutex_node* next;

} mutex_node;

typedef struct queue_t {

	 processDescriptor* head;
	 processDescriptor* tail;

} queue_t;

typedef struct sleep_queue {

	sleep_node* head;
	sleep_node* tail;
	
} sleep_queue;

typedef struct mutex_queue{
	
	mutex_node* head;
	mutex_node* tail;
	
} mutex_queue;

typedef struct mutex {
	
	bool locked;                     // Indicates whether mutex is locked or not
  	int count;                       // Indicates the times it has been locked by owner (for recursive locks)
	processDescriptor* owner;        // Task that locked the mutex
	mutex_queue* queue;              // Queue of waiting tasks
	//mutex_node * next;
	
} mutex;

volatile static  processDescriptor* Cp;           // The process descriptor of the currently RUNNING task.
static  processDescriptor Process[MAXPROCESS];    // Contains ALL process descriptors, regardless of state
static processDescriptor  task_desc[MAXPROCESS + 1];
static processDescriptor* idle_task = &task_desc[MAXPROCESS]; // Idle task at end of array

//Data from currently running task
static queue_t Dead_tasks;                // terminated tasks
static queue_t Round_Robin;               // round robin task queue
static queue_t system_tasks;              // system task queue
static sleep_queue sleeping_tasks;        // sleeping task queue
static queue_t event_queue;               // tasks waiting on events
static volatile uint8_t ticks_remaining;  // time remaining in current slot?
static queue_t periodic_tasks;  // time remaining in current slot?

/** 
  * Since this is a "full-served" model, the kernel is executing using its own
  * stack. We can allocate a new workspace for this kernel stack, or we can
  * use the stack of the "main()" function, i.e., the initial C runtime stack.
  * (Note: This and the following stack pointers are used primarily by the
  *   context switching code, i.e., CSwitch(), which is written in assembly
  *   language.)
  */         
volatile unsigned char *KernelSp;

/**
  * This is a "shadow" copy of the stack pointer of "Cp", the currently
  * running task. During context switching, we need to save and restore
  * it into the appropriate process descriptor.
  */
volatile unsigned char *CurrentSp;

/** index to next task to run */
volatile static unsigned int NextP;  

/** 1 if kernel has been started; 0 otherwise. */
volatile static unsigned int KernelActive;  

/** number of tasks created so far */
volatile static unsigned int Tasks;  

/**
 * When creating a new task, it is important to initialize its stack just like
 * it has called "Enter_Kernel()"; so that when we switch to it later, we
 * can just restore its execution context on its stack.
 * (See file "cswitch.S" for details.)
 */

/*Chane to include Priority*/
// Adds task to the queue, UNORDERED
static void enqueue(queue_t* input_queue, processDescriptor* input_process){
  
  input_process->next = NULL;

  if(input_queue->head == NULL){              // If queue is empty

    input_queue->head = input_process;        // Node becomes head and tail
    input_queue->tail = input_process;

  } else {
    
    input_queue->tail->next = input_process;  // Else new node becomes tail
    input_queue->tail = input_process;
    
  }
}

void Kernel_Create_Task_At(  processDescriptor *p, voidfuncptr f ) 
{   
   unsigned char *sp;

   //Changed -2 to -1 to fix off by one error.
   sp = (unsigned char *) &(p->workSpace[WORKSPACE-1]);

   /*----BEGIN of NEW CODE----*/
   //Initialize the workspace (i.e., stack) and PD here!

   //Clear the contents of the workspace
   memset(&(p->workSpace),0,WORKSPACE);

   //Notice that we are placing the address (16-bit) of the functions
   //onto the stack in reverse byte order (least significant first, followed
   //by most significant).  This is because the "return" assembly instructions 
   //(rtn and rti) pop addresses off in BIG ENDIAN (most sig. first, least sig. 
   //second), even though the AT90 is LITTLE ENDIAN machine.

   //Store terminate at the bottom of stack to protect against stack underrun.
   *(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;

   //Place return address of function at bottom of stack
   *(unsigned char *)sp-- = ((unsigned int)f) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)f) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;

   //Place stack pointer at top of stack
   sp = sp - 34;

   p->sp = sp;		/* stack pointer into the "workSpace" */
   p->code = f;		/* function to be executed as a task */
   p->request = NONE;

   /*----END of NEW CODE----*/

   
   
    if (p->original == 0)
    {
	    enqueue(&system_tasks, p);
    }
    else if (p->original == 1)
    {
	    enqueue(&periodic_tasks, p);
    }
    else if (p->original == 2)
    {
	    enqueue(&Round_Robin, p);
    }
    p->state = READY;
}

/**
  *  Create a new task
  */
//This is probably good for Task_Create()
static void Kernel_Create_Task( voidfuncptr f ) 
{
   int x;

   if (Tasks == MAXPROCESS) return;  /* Too many tasks! */

   /* find a DEAD PD that we can use  */
   for (x = 0; x < MAXPROCESS; x++) {
       if (Process[x].state == DEAD) break;
   }

   ++Tasks;
   Kernel_Create_Task_At( &(Process[x]), f );

}


// Removes head of the queue
static processDescriptor* dequeue(queue_t* input_queue){
  processDescriptor* next_task = input_queue->head;
  
  if(input_queue->head != NULL) {

    input_queue->head = input_queue->head->next;
    next_task->next = NULL;
    
  }
  return next_task;
}

static void enqueue_mutex(mutex_queue* input_queue, mutex_node* input_node){
	
	//sort by priority so I don't have to do it later.
	mutex_node* tmp;
	mutex_node* tmp2;
	
	if (input_queue->head == NULL)
	{
		input_queue->head = input_node;
		input_queue->tail = input_node;
	}
	else{
		
		for(tmp = input_queue->head; tmp->next != NULL; tmp=tmp->next){
			
			if(tmp->next->task->original < input_node->task->original){
				//find place in queue to put the damn thing
				
				tmp2 = tmp->next;
				
				tmp ->next = input_node;
				input_node->next = tmp2;
			}
			
		}
		
		
		
		
		
		
	}
	
}


static void enqueue_sleep(sleep_queue* input_queue, sleep_node* input_sleepnode){
	
	sleep_node* tmp = NULL;
	TICK before;
	TICK after;
	
  // If queue is empty
	if(input_queue->head == NULL){
		
		input_queue->head = input_sleepnode;             // New task becomes head and tail
		input_queue->tail = input_sleepnode;
		PORTL |= (1 << DDL1);
    return;
	}                                                

  // If adding a task to a pre-existing queue
	before = input_queue->head->task->ticks;
	
	if (input_sleepnode->task->ticks < before) {     // If new node has less ticks than head, becomes new head

    input_sleepnode->next = input_queue->head;
    input_queue->head = input_sleepnode;

	} else {                                         // Need to search queue for correct placement

    tmp = input_queue->head;

    while (tmp->next != NULL) {
      after = tmp->next->task->ticks;

      if (input_sleepnode->task->ticks >= before && input_sleepnode->task->ticks <= after) { // Found placement, insert node

          input_sleepnode->next = tmp->next;
          tmp->next = input_sleepnode;
          return;
      }

      before = after;                             // Before = tmp->next->ticks
      tmp = tmp->next;                            // Before = tmp->ticks

    }

    // Add to end of queue
    input_queue->tail->next = input_sleepnode;
    input_queue->tail = input_sleepnode; 
	}		
}

sleep_node* dequeue_sleep (sleep_queue* input_queue) {
  sleep_node* ready_task = input_queue->head;

  if (ready_task != NULL){
    input_queue->head = input_queue->head->next;
    ready_task->next = NULL;
  }

  return ready_task;
}
void Update_Sleep_Queue(){
	
	int tasksRemain = 1;										// Condition variable
	sleep_node sleeping_task;
	sleeping_task.task = sleeping_tasks.head->task; //handle the sleeping tasks during dispatch
	
	if (sleeping_task.task != NULL){
		while (tasksRemain){
			sleeping_task.task->ticks -= interrupt_handles;
			
			if (sleeping_task.task->ticks <= 0){
				Disable_Interrupt();
				sleeping_task.task->request = RESUME;
				Enter_Kernel();
				break;
			}
			
			if (sleeping_task.next) {
				sleeping_task = *sleeping_task.next;
				} else {
				tasksRemain = 0;									// We're done
			}
		}
	}
	
	
}
void Update_Period_Queue(){
	
	processDescriptor* head;
	
	for (head = periodic_tasks.head; head->next != NULL; head = head->next)
	{
		head ->remaining -= interrupt_handles;
		
		if (head ->remaining <= 0)
		{
			head ->state = READY;
			head->remaining = head->period;
			
		}
		
	}
	
}


/* ****************************************** TODO ****************************************************
 * We need to add a check for the suspension flag, which would prevent it from being selected to run
 */
static void new_dispatch()
{
	if (interrupt_handles > 0)
	{
		
		Update_Sleep_Queue();
		Update_Period_Queue();
		interrupt_handles = 0;
	}
		
	// What about tasks that are not ready?
	if (Cp->state != RUNNING || Cp == idle_task ){
		
		if (system_tasks.head != NULL) {

			Cp = dequeue(&system_tasks);

		} else if (periodic_tasks.head != NULL) { // There is some more to add here, but I don't know what they're doing
			
      // Current task equals something from the PPP array
			Cp = dequeue(&periodic_tasks);
			
		} else if(Round_Robin.head != NULL) {
			
			Cp = dequeue(&Round_Robin);

		} else{
		
			Cp = idle_task;
		}
	}
	Cp->state = RUNNING;
}

/**
  * This internal kernel function is a part of the "scheduler". It chooses the 
  * next task to run, i.e., Cp.
  */
  /*
static void Dispatch()
{
     /* find the next READY task
       * Note: if there is no READY task, then this will loop forever!.
       
   while(Process[NextP].state != READY) {
      NextP = (NextP + 1) % MAXPROCESS;
   }

   Cp = &(Process[NextP]);
   CurrentSp = Cp->sp;
   Cp->state = RUNNING;

   NextP = (NextP + 1) % MAXPROCESS;
}
*/
/**
  * This internal kernel function is the "main" driving loop of this full-served
  * model architecture. Basically, on OS_Start(), the kernel repeatedly
  * requests the next user task's next system call and then invokes the
  * corresponding kernel function on its behalf.
  *
  * This is the main loop of our kernel, called by OS_Start().
  */
static void Next_Kernel_Request() 
{
   new_dispatch();  /* select a new task to run */

   while(1) {
       Cp->request = NONE;              // Clear its request
       CurrentSp = Cp->sp;              // Active this newly selected task
       Exit_Kernel();                   // Or C-Switch

       /* if this task makes a system call, it will return to here! */

       Cp->sp = CurrentSp;              // Save the Cp's stack pointer

       switch(Cp->request){
       case CREATE:
           Kernel_Create_Task( Cp->code );
           break;
       case NEXT:
	     case NONE:                      // Could be caused by timer interrupt
          Cp->state = READY;
          new_dispatch();
          break;
		   case SUSPEND:
    			Cp->susp = 1;
    			if (Cp->state == RUNNING){
    				Cp->state = SUSP_READY;
    			} else if (Cp->state == BLOCKED) {
            Cp->state = SUSP_BLOCKED;
          }
          break;
        case RESUME:
          Cp->susp = 0;
          if (Cp->state == SUSP_READY) {
            Cp->state = READY;
          } else if (Cp->state == SUSP_BLOCKED) {
            Cp->state = BLOCKED;
          }
          break;
        case BLOCK:
          if (Cp->state == SUSP_READY){
            Cp->state = SUSP_BLOCKED;
          } else {
            Cp->state = BLOCKED;
          }
          break;
        case UNBLOCK:
          if (Cp->state == SUSP_BLOCKED){
            Cp->state = SUSP_READY;
          } else {
            Cp->state = READY;
          }
          break;
        case TERMINATE:
          Cp->state = DEAD;           // Deallocate all resources used by this task
          if (Cp->ticks > 0) {
            /* Need to dequeue from sleep queue */
          }

          /* Need to dequeue from any task queues */
          new_dispatch();
       default:
         
          break;                     // PROBLEM
       }
    } 
}

/**
  * This function initializes the RTOS and must be called before any other
  * system calls.
  */
void OS_Init() 
{
  int x;
	DDRA = (1<<PA0);
	DDRB = (1<<DDB7);
	DDRL |= (1<<DDL1);// pin 48 test
	DDRA = (1<<PA1);
	PORTA &= ~(1<<PA1);

  Tasks = 0;
  KernelActive = 0;
  NextP = 0;

	//Reminder: Clear the memory for the task on creation.
  for (x = 0; x < MAXPROCESS; x++) {
    memset(&(Process[x]),0,sizeof(processDescriptor));
    Process[x].state = DEAD;
  }
}


/**
  * This function starts the RTOS after creating a few tasks.
  */
void OS_Start() 
{   
   if ( (! KernelActive) && (Tasks > 0)) {
       Disable_Interrupt();
      /* we may have to initialize the interrupt vector for Enter_Kernel() here. */

      /* here we go...  */
      KernelActive = 1;
      Next_Kernel_Request();
      /* NEVER RETURNS!!! */
   }
}


/**
  * For this example, we only support cooperatively multitasking, i.e.,
  * each task gives up its share of the processor voluntarily by calling
  * Task_Next().
  */
void Task_Create( voidfuncptr f, PRIORITY py, int arg)
{
   if (KernelActive ) {
     Disable_Interrupt();
     Cp ->request = CREATE;
     Cp->code = f;
     Enter_Kernel();
   } else { 
      /* call the RTOS function directly */
      Kernel_Create_Task( f );
   }
}

/**
  * The calling task gives up its share of the processor voluntarily.
  */
void Task_Next() 
{
   if (KernelActive) {
     Disable_Interrupt();
     Cp ->request = NEXT;
     Enter_Kernel();
  }
}

/**
  * The calling task terminates itself.
  */
void Task_Terminate() 
{
   if (KernelActive) {
      Disable_Interrupt();
      Cp -> request = TERMINATE;
      Enter_Kernel();
     /* never returns here! */
   }
}

void Task_Suspend(PID p)
{
  //Suspend Task
  if(Cp->p == p && KernelActive){
    Disable_Interrupt();
    Cp->request = SUSPEND;
    Enter_Kernel();
  }
}

processDescriptor* get_Task(PID p){
	int x;
	processDescriptor* pd = NULL;
	for (x=0;x<MAXPROCESS;x++){
		if (Process[x].p == p){
			pd = &Process[x];
			break;
		}
	}
	return pd;
}

void Task_Resume(PID p){
	processDescriptor* pd;
	pd = get_Task(p);
	if (pd){
    Disable_Interrupt();
		pd->request = RESUME;
    Enter_Kernel();
	}
}

void Task_Yield(){
  //Yield to Priority Tasks
}

int Task_Get_Arg(PID p){
	processDescriptor* pd;
	pd = get_Task(p);

	if (pd) {
		return pd->arg;
	} else {
		return 1;
	}
}

void Mutex_Lock(mutex* m, processDescriptor* caller){
	
	mutex_node newNode;
	newNode.task = caller;
	
	if (m->locked != true) {                  // If the mutex is unlocked

    assert(!m->owner);                      // Assert mutex has no owner
    Disable_Interrupt();
		m->owner = m->queue->head->task;
		m->locked = true;
		m->queue->head = m->queue->head->next;  /* TODO Dequeue from mutex queue */
    Enable_Interrupt();

	} else if (m->owner == caller) {         // Owner locking a second time

    Disable_Interrupt();
    m->count += 1;
    Enable_Interrupt();

  } else if (m->owner->state == DEAD) {    // Owner was terminated before unlocking mutex

    Disable_Interrupt();
    m->owner = m->queue->head->task;
    m->count = 0;
    m->locked = true;
    m->queue->head = m->queue->head->next; /* TODO Dequeue from mutex queue */
    Enable_Interrupt();
    
  } else {                                 // New task trying to acquire claimed lock
		  
    Disable_Interrupt();
    caller->request = BLOCK;
    enqueue_mutex(&m->queue, &newNode);    /* This should maybe be a signal? */
    Enter_Kernel();

  }
}

// Need to include recursive locks/unlocks
// Should be conditional on WHO is locking/unlocking
void Mutex_Unlock(mutex* m, processDescriptor* caller){

  // Recursive unlock
  if (m->owner == caller && m->count > 1){  

    Disable_Interrupt();
    m->count -= 1;
    Enable_Interrupt();

  // The queue has tasks waiting
  } else if (m->owner == caller && m->queue->head != NULL) {

    Disable_Interrupt();
		m->owner = dequeue(&m->queue);
		m->locked = true;                           // Should have already been locked, but safe measure
		m->owner->request = UNBLOCK;
		Enter_Kernel();
		
  // Queue is empty
	} else if (m->owner == caller) {

    Disable_Interrupt();
		m->owner = NULL;
		m->locked = false;
    Enable_Interrupt();

	}
}

void Inherit_Priority(mutex* m){
	
	//find the maximum priority in the queue

	
	
}
//Put the task to sleep for a certain amount of time
//Eventually to take a clock tick parameter
//current process can only call sleep on itself
//Iterate through sleep queue and place in timer fashion  
void Task_Sleep(TICK t){
	sleep_node newnode;

	Cp->ticks += t;
	newnode.task = Cp;
	enqueue_sleep(&sleeping_tasks, &newnode);
	Task_Suspend(Cp->p);
}

// System timer that interrupts every 10 ms
void TIMER3_COMPA_vect(void){
  Disable_Interrupt();
  interrupt_handles += 1;
  Enable_Interrupt();
}


/*============
  * A Simple Test 
  *============
  */

/**
  * A cooperative "Ping" task.
  * Added testing code for LEDs.
  */
void Ping()
{
	//Enable_Interrupt();
	int  x ;
	for(;;){
		//LED on
		//PORTA |= (1<<PA0);
		PORTA &= ~(1<<PA0);
		PORTB &= ~(1<<DDB7);
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		//Task_Sleep(50);
		Task_Next();
	}
}


/**
  * A cooperative "Pong" task.
  * Added testing code for LEDs.
  */
void Pong()
{
	//Enable_Interrupt();
	int  x;
	for(;;) {
		//LED off
		//PORTA &= ~(1<<PA0);
		PORTA |= (1<<PA0);
		PORTB |= (1<<DDB7);
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		Task_Next();
	}
}


/**
  * This function creates two cooperative tasks, "Ping" and "Pong". Both
  * will run forever.
  */
void main() 
{
   OS_Init();
   Task_Create( Ping, 8, 8 );
   Task_Create( Pong, 8, 8 );
   OS_Start();
}

  

