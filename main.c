#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdio.h> 
#include <stdlib.h> 

int counter = 0; //global counter made to know how many tasks were added.
int dispatched = 0; //another global counter to know how many tasks were dispatched.
struct Task //this is done to be able to store the task name with its priority.
{
    void (*task)(void); //this takes the function name.
    int priority; //this takes the priority.
};

static void sendUART(uint8_t * data, uint32_t length)
{
	 for (uint32_t i=0; i<length; ++i){
      // add new data without messing up DR register
      uint32_t value = (USART2->DR & 0x00) | data[i];
		  // send data
			USART2->DR = value;
      // busy wait for transmit complete
      while(!(USART2->SR & (1 << 6)));
		  // delay
      for(uint32_t j=0; j<1000; ++j);
      }
}

//void QueTask(void(*task)(void), int priority);
struct Task Queue[10]; //creating a queue to be able store all the tasks inside it.
struct Task DelayedQueue[10]; //the delayed queue to be used in rerunme.
void TaskA() //creating the TaskA function which is void.
{
	  static uint8_t taskamsg[] = "Task A\n"; //The message to be displayed in renode. 
		sendUART(taskamsg,sizeof(taskamsg));
	  //ReRunMe(0);
}
void TaskB()//creating the TaskB function which is void.
{
	
	    static uint8_t taskbmsg[] = "Task B\n"; //The message to be displayed in renode.
			sendUART(taskbmsg,sizeof(taskbmsg));
			//ReRunMe(0);
}
void TaskC() //creating the TaskC function which is void.
{
	  static uint8_t taskcmsg[] = "Task C\n"; //The message to be displayed in renode.
		sendUART(taskcmsg,sizeof(taskcmsg));
    //ReRunMe(0);
}


void QueTask(void(*task2)(void), int priority2) //this is the quetask function which takes the function along with the priority.
{
    int temp = 0; //created a temp value to be able to swap values.
    void(*tasktemp)(void);

    if (counter == 0) //checking if this is the first entry.
    {
        Queue[0].priority = priority2; //if its the first entry it automatically enters in the first position.
        Queue[0].task = task2;
        counter++; //increase the counter.
    }
    else //however, if its not the first one 
    {
        Queue[counter].priority = priority2; // we add it to the next empty value which is at the counter position. 
        Queue[counter].task = task2;
        counter++;
    }

    for (int i = 0; i < 10; i++) { //the sorting is done here
        for (int j = i + 1; j < 10; j++) {
            if ((Queue[i].priority > Queue[j].priority) &&(Queue[j].priority !=0)) { //checking if its smaller than the priority or not and if it has a stored value or not.
                temp = Queue[i].priority;
                tasktemp = Queue[i].task;
                Queue[i].priority = Queue[j].priority;
                Queue[i].task = Queue[j].task;
                Queue[j].priority = temp;
                Queue[j].task = tasktemp;
            }
        }

    }
}
void Dispatch() //the dispatch function.
{

    for (int i = 0; i < counter; i++) //for loop to dispatch all tasks given. 
    {
        if (Queue[dispatched].task == &TaskA) //to check which task is at first position 
        {
            TaskA(); //run the task 
            dispatched=dispatched+1; //increment the dispatched counter.
        }
        else if (Queue[dispatched].task == &TaskB)
        {
            TaskB();
            dispatched=dispatched+1;
        }
        else if (Queue[dispatched].task == &TaskC)
        {
            TaskC();
            dispatched=dispatched+1;

        }
    }
		dispatched=0; //this is done so the code keeps running in the while loop.
}
//void ReRunMe(int delay) //this is the rerunme function , however, its not fully working and thats why its commented.
//{
//    if (delay == 0) //checking if the delay is equal to 0 in order to dispatch.
//    {
//        if (Queue[rerun].task = TaskA)
//        {
//            QueTask(TaskA, 0);
//            rerun++;
//            TaskA();
//        }
//        if (Queue[rerun].task = TaskB)
//        {
//            QueTask(TaskB, 0);
//            rerun++;
//            TaskB();
//        }
//        if (Queue[rerun].task = TaskC)
//        {
//            QueTask(TaskC, 0);
//            rerun++;
//            TaskC();
//        }
//    }
//    else { //storing them in delayedqueue until the delay is 0.
//        DelayedQueue[rerun].task = Queue[rerun].task;
//        DelayedQueue[rerun].priority = Queue[rerun].priority;
//         }
//}
void init()
{
    QueTask(TaskA, 4); //setting quetask 
    QueTask(TaskB, 2);
    QueTask(TaskC, 3);
}
//static uint8_t msg[] = "Renode Alive !!\n";
//static uint8_t pressedMsg[] = "Button is pressed !!\n";
//static uint8_t releasedMsg[] = "Button is released !!\n";
static char buttonPressed = 1;
static char timerFlag = 0;
static volatile uint8_t stopFlag = 0;


void SysTick_Handler(void);
void USART2_IRQHandler(void);
void EXTI0_IRQHandler(void);
static void sendUART(uint8_t * data, uint32_t length);
static uint8_t receiveUART(void);
void TaskA(void);
void SysTick_Handler(void)  {
	timerFlag = 1;
}

void USART2_IRQHandler(void) {
	/* pause/resume UART messages */
	stopFlag = !stopFlag;
	
	/* dummy read */
	(void)receiveUART();
}

void EXTI0_IRQHandler(void) {
		/* Clear interrupt request */
		EXTI->PR |= 0x01;

}

static uint8_t receiveUART()
{
	  // extract data
	  uint8_t data = USART2->DR & 0xFF;
	
	  return data;
}

static void gpioInit()
{	
    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); // Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3
}

static void uartInit()
{
	
    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);
	
	  // USART2 TX enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // USART2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);
	
	  // USART2 rx interrupt, RXNEIE bit 5
    USART2->CR1 |= (1 << 5);

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 16 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 16Mhz / 115200 / 16 = 8.6805
    // Fraction : 16*0.6805 = 11 (multiply fraction with 16)
    // Mantissa : 8
    // 12-bit mantissa and 4-bit fraction
    USART2->BRR |= (8 << 4);
    USART2->BRR |= 11;

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);
}

int main()
{	
	  /* startup code initialization */
	  SystemInit();
	  SystemCoreClockUpdate();
	  /* intialize UART */
	  gpioInit();
		/* intialize UART */
	  uartInit();
	  /* enable SysTick timer to interrupt system every second */
	  SysTick_Config(SystemCoreClock/10);
	  /* enable interrupt controller for USART2 external interrupt */
		NVIC_EnableIRQ(USART2_IRQn);
		/* Unmask External interrupt 0 */
		EXTI->IMR |= 0x0001;
	  /* Enable rising and falling edge triggering for External interrupt 0 */
		EXTI->RTSR |= 0x0001;
		EXTI->FTSR |= 0x0001;
	  /* enable interrupt controller for External interrupt 0 */
		NVIC_EnableIRQ(EXTI0_IRQn);
    init(); //calling function.
//	  Dispatch(); //we can call this function to run one instance of this code.

	  while(1)
		{

			 Dispatch(); //this function is called to rerun the code forever. 

		}

}
