/*
 * 	main.c
 *
 *
 * 	Author : Kevin.delvallez
 */

#include "stm32f0xx.h"
#include "main.h"
#include "bsp.h"
#include "factory_io.h"
#include "delay.h"
#include <math.h>

// DEFINE
#define Number_subscription 4

// SYNCHROS
EventGroupHandle_t Synchro;
#define	synchro_ascensseur_place	( (EventBits_t)( 0x01 <<0) )
#define	synchro_cartons_places	( (EventBits_t)( 0x01 <<1) )
#define	synchro_paletiseur_vide	( (EventBits_t)( 0x01 <<2) )
#define	synchro_cartons_entres	( (EventBits_t)( 0x01 <<3) )
#define	synchro_fermeture_porte	( (EventBits_t)( 0x01 <<3) )
traceString synchronisation;
uint32_t free_heap_size;                  // <- Variable globale

//COMPTEURS
uint8_t compt_carton = 0;
uint8_t compt_entree_paletiseur = 0;

// Task
void task_pub(void *pvParameters);
void task_write(void *pvParameters);
void task_palette(void *pvParameters);
void task_chargement(void *pvParameters);
void task_distribution(void *pvParameters);

static uint8_t 	SystemClock_Config	(void);

// Global variable


typedef struct
	{
		xSemaphoreHandle *sem; // Semaphore ID to use for publication
		uint32_t sensor_msk; // Awaited sensor ID
		uint32_t sensor_state; // Awaited sensor State
	} subscribe_message_t;



typedef struct
	{
		uint32_t actuator_msk; // Awaited sensor ID
		uint32_t actuator_state; // Awaited sensor State
	} actuator_message_t;

xQueueHandle 	 xSubscribeQueue;
xQueueHandle 	 xActuatorsQueue;
xSemaphoreHandle xSem_DMA_TC;
xSemaphoreHandle xSem1;
xSemaphoreHandle xSem2;
xSemaphoreHandle xSem3;
uint8_t buffer[7];


// Main program

int main()
{
	//SystemClock_Config();

	// Initialize LED pin
	BSP_LED_Init();

	// Initialize Debug Console
	BSP_Console_Init();

	// Init buton
	BSP_PB_Init();

	// Read all states from the scene
	FACTORY_IO_update();
	BSP_LED_Toggle();

	// Wait here for user button
	while(BSP_PB_GetState() == 0);

	// Start conveyor A[0] = 1
	my_printf("Starting Conveyor\r\n");
	FACTORY_IO_Actuators_Set(0x003D1003,0xFFFFFFFF);

	// Start Trace Recording
	vTraceEnable(TRC_START);

	//Tracing Eevent Group
	synchronisation = xTraceRegisterString("synchronisation");

	// Creation task
	xTaskCreate(task_pub, "Task_pub", 128, NULL, 9, NULL);
	xTaskCreate(task_write, "Task_write", 128, NULL, 10, NULL);
	xTaskCreate(task_chargement, "Task_chargement", 128, NULL, 4, NULL);
	xTaskCreate(task_palette, "Task_Palette", 128, NULL, 5, NULL);
	xTaskCreate(task_distribution, "Task_Ascensseur", 128, NULL, 3, NULL);

	//Create queue
	xSubscribeQueue = xQueueCreate(10, sizeof(subscribe_message_t *));
	vTraceSetQueueName(xSubscribeQueue, "xSubscribeQueue");
	xActuatorsQueue = xQueueCreate(10, sizeof(uint32_t *));
	vTraceSetQueueName(xActuatorsQueue, "xActuatorsQueue");

	// EVENT GROUP
	Synchro = xEventGroupCreate();

	//Create semaphore
	xSem_DMA_TC = xSemaphoreCreateBinary();
	vTraceSetSemaphoreName(xSem_DMA_TC, "xSem_DMA_TC");
	xSem1 = xSemaphoreCreateBinary();
	vTraceSetSemaphoreName(xSem1, "xSem1");
	xSem2 = xSemaphoreCreateBinary();
	vTraceSetSemaphoreName(xSem2, "xSem2");
	xSem3 = xSemaphoreCreateBinary();
	vTraceSetSemaphoreName(xSem3, "xSem3");

	// Start the Scheduler
	vTaskStartScheduler();


	while(1)
	{
		// The program should never be here...
	}
}


void task_pub(void *pvParameters){
	portTickType    xLastWakeTime;
	// Initialize timing
	xLastWakeTime = xTaskGetTickCount();
	subscribe_message_t sub_table[Number_subscription];
	subscribe_message_t *message;
	uint8_t inclu = 0;
	uint8_t index = 0;
	uint8_t i = 0;
	uint32_t sensors = 0;

	for(int i = 0; i<Number_subscription;i++){
		sub_table[i].sem = NULL;
		sub_table[i].sensor_msk = 0x00000000;
		sub_table[i].sensor_state = 0x00000000;
	}
	portBASE_TYPE p;

	while(1){
		p = xQueueReceive(xSubscribeQueue, &message, 0);

		inclu = 0;

		sensors = FACTORY_IO_Sensors_Get();

		if(p == pdPASS){
			index = 0;
			while(index < Number_subscription && inclu == 0 && sub_table[index].sem != NULL){
				if((message->sem == sub_table[index].sem)){
					inclu = 1;
				}
				index ++;
			}
			if(inclu == 0){
				sub_table[index].sensor_state = message->sensor_state;
				sub_table[index].sensor_msk = message->sensor_msk;
				sub_table[index].sem = message->sem;
			}
		}
		for(i = 0;i<Number_subscription;i++){
			if((sensors & sub_table[i].sensor_msk) == sub_table[i].sensor_state && sub_table[i].sem != NULL){
				xSemaphoreGive(*(sub_table[i].sem));
				sub_table[i].sem = NULL;
				sub_table[i].sensor_msk = 0x00000000;
				sub_table[i].sensor_state = 0x00000000;
			}
		}
		p = pdFAIL;
		free_heap_size = xPortGetFreeHeapSize();  // <- A glisser dans n'importe quelle tâche periodique
		vTaskDelayUntil(&xLastWakeTime,200/portTICK_RATE_MS);
	}
}

void task_write(void *pvParameters){
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

	actuator_message_t *actuator;

	while(1){
		xQueueReceive(xActuatorsQueue, &actuator, portMAX_DELAY);

		FACTORY_IO_Actuators_Set(actuator->actuator_state,actuator->actuator_msk);

		DMA1_Channel4->CNDTR = 7;

		DMA1_Channel4->CCR |= DMA_CCR_EN;
		USART2->CR3 |= USART_CR3_DMAT;

		xSemaphoreTake(xSem_DMA_TC, portMAX_DELAY);

		DMA1_Channel4->CCR &=~ DMA_CCR_EN;
		USART2->CR3 &=~ USART_CR3_DMAT;

	}
}

void task_palette(void *pvParameters){
	subscribe_message_t message;
	subscribe_message_t *pm;
	actuator_message_t send;
	actuator_message_t *psend;
	xSemaphoreTake(xSem1,0);
	while(1){
		// Act distrib palette
		send.actuator_msk = DISTRIBUTION_PALETTE_msk | CHARGER_PALETTE_msk | ASCENSSEUR_TO_LIMIT_msk | DESCENDRE_ASCENSSEUR_msk;
		send.actuator_state = DISTRIBUTION_PALETTE_ON | CHARGER_PALETTE_ON | ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);

		// Attente capteur distrib palette
		message.sem = &xSem1;
		message.sensor_msk = PALETTE_DISTRIBUEE_msk;
		message.sensor_state = PALETTE_DISTRIBUEE_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);

		//Desactivation distrib palette
		send.actuator_msk = DISTRIBUTION_PALETTE_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		vTaskDelay(500/portTICK_RATE_MS);

		//Attente entrée palette
		message.sem = &(xSem1);
		message.sensor_msk = ENTRE_PALETTE_msk;
		message.sensor_state = ENTRE_PALETTE_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);

		//Attente fin entree palette
		message.sem = &(xSem1);
		message.sensor_msk = SORTIE_PALETTE_msk;
		message.sensor_state = SORTIE_PALETTE_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);

		//Arret chargement palette et monter ascensseur
		send.actuator_msk = CHARGER_PALETTE_msk | MONTER_ASCENSSEUR_msk | ASCENSSEUR_TO_LIMIT_msk;
		send.actuator_state = ACTUATORS_OFF | MONTER_ASCENSSEUR_ON | ASCENSSEUR_TO_LIMIT_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);

		//Attente fin montée
		message.sem = &(xSem1);
		message.sensor_msk = ASCENSSEUR_ETAGE_1_msk;
		message.sensor_state = ASCENSSEUR_ETAGE_1_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);
		xEventGroupSetBits(Synchro, synchro_ascensseur_place);
		vTracePrintF(synchronisation, "%d", synchro_ascensseur_place);

		//Stop montée
		send.actuator_msk = MONTER_ASCENSSEUR_msk | ASCENSSEUR_TO_LIMIT_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);

		//Attente cartons pour descente
		xEventGroupWaitBits(Synchro, synchro_cartons_places, pdTRUE, pdTRUE, portMAX_DELAY);
		send.actuator_msk = DESCENDRE_ASCENSSEUR_msk;
		send.actuator_state = DESCENDRE_ASCENSSEUR_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);

		//Attente fin descente puis attente cartons
		message.sem = &(xSem1);
		message.sensor_msk = ASCENSSEUR_ETAGE_1_msk;
		message.sensor_state = ASCENSSEUR_ETAGE_1_OFF;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);
		send.actuator_msk = DESCENDRE_ASCENSSEUR_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		vTaskDelay(100/portTICK_RATE_MS);
		xEventGroupSetBits(Synchro, synchro_fermeture_porte);
		xEventGroupSetBits(Synchro, synchro_ascensseur_place);
		vTracePrintF(synchronisation, "%d", synchro_fermeture_porte | synchro_ascensseur_place);
		xEventGroupWaitBits(Synchro, synchro_cartons_places, pdTRUE, pdTRUE, portMAX_DELAY);

		//Descente rdc
		send.actuator_msk = ASCENSSEUR_TO_LIMIT_msk | DESCENDRE_ASCENSSEUR_msk;
		send.actuator_state = ASCENSSEUR_TO_LIMIT_ON | DESCENDRE_ASCENSSEUR_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		message.sem = &(xSem1);
		message.sensor_msk = ASCENSSEUR_ETAGE_RDC_msk;
		message.sensor_state = ASCENSSEUR_ETAGE_RDC_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem1,portMAX_DELAY);
		vTaskDelay(100/portTICK_RATE_MS);
		xEventGroupSetBits(Synchro, synchro_fermeture_porte);
		vTracePrintF(synchronisation, "%d", synchro_fermeture_porte);
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

void task_chargement(void *pvParameters){
	subscribe_message_t message;
	subscribe_message_t *pm;
	actuator_message_t send;
	actuator_message_t *psend;
	xSemaphoreTake(xSem2,0);
	while(1){

		compt_entree_paletiseur = 0;
		while(compt_entree_paletiseur < 3){
			//Attente entree des cartons
			xEventGroupWaitBits(Synchro, synchro_cartons_entres, pdTRUE, pdTRUE, portMAX_DELAY);
			vTaskDelay(600/portTICK_RATE_MS);

			//Poussoir
			send.actuator_msk = POUSSOIR_msk;
			send.actuator_state = POUSSOIR_ON;
			psend = &send;
			xQueueSendToBack(xActuatorsQueue,&psend,0);
			vTaskDelay(1000/portTICK_RATE_MS);

			//Stop poussoir
			send.actuator_msk = POUSSOIR_msk;
			send.actuator_state = ACTUATORS_OFF;
			psend = &send;
			xQueueSendToBack(xActuatorsQueue,&psend,0);
			xEventGroupSetBits(Synchro, synchro_paletiseur_vide);
			vTracePrintF(synchronisation, "%d", synchro_paletiseur_vide);

			compt_entree_paletiseur ++;
		}

		//Attente placement de la palette
		xEventGroupWaitBits(Synchro, synchro_ascensseur_place, pdTRUE, pdTRUE, portMAX_DELAY);

		//Accrochage puis ouverture
		send.actuator_msk = CLAMP_msk;
		send.actuator_state = CLAMP_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		vTaskDelay(500/portTICK_RATE_MS);
		send.actuator_msk = PORTE_msk;
		send.actuator_state = PORTE_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);

		//Attente fin ouverture de porte pour lacher les cartons
		message.sem = &(xSem2);
		message.sensor_msk = PORTE_OUVERTE_msk;
		message.sensor_state = PORTE_OUVERTE_ON;
		pm = &message;
		xQueueSendToBack(xSubscribeQueue,&pm,0);
		xSemaphoreTake(xSem2,portMAX_DELAY);
		send.actuator_msk = CLAMP_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		xEventGroupSetBits(Synchro, synchro_cartons_places);
		vTracePrintF(synchronisation, "%d", synchro_cartons_places);
		vTaskDelay(200/portTICK_RATE_MS);

		//Fermeture de la porte
		xEventGroupWaitBits(Synchro, synchro_fermeture_porte, pdTRUE, pdTRUE, portMAX_DELAY);
		send.actuator_msk = PORTE_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
	}
}

void task_distribution(void *pvParameters){
	subscribe_message_t message;
	subscribe_message_t *pm;
	actuator_message_t send;
	actuator_message_t *psend;
	xSemaphoreTake(xSem3,0);
	while(1){
		//Passage de deux cartons
		compt_carton = 0;
		while(compt_carton < 2){
			message.sem = &(xSem3);
			message.sensor_msk = ENTREE_PALETISEUR_msk;
			message.sensor_state = ENTREE_PALETISEUR_ON;
			pm = &message;
			xQueueSendToBack(xSubscribeQueue,&pm,0);
			xSemaphoreTake(xSem3,portMAX_DELAY);
			message.sem = &(xSem3);
			message.sensor_msk = ENTREE_PALETISEUR_msk;
			message.sensor_state = ENTREE_PALETISEUR_OFF;
			pm = &message;
			xQueueSendToBack(xSubscribeQueue,&pm,0);
			xSemaphoreTake(xSem3,portMAX_DELAY);
			compt_carton ++;
		}
		vTaskDelay(600/portTICK_RATE_MS);

		//Activation barrière et attente de desactivation
		send.actuator_msk = BLOCAGE_ENTREE_PALETISEUR_msk;
		send.actuator_state = BLOCAGE_ENTREE_PALETISEUR_ON;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		xEventGroupSetBits(Synchro, synchro_cartons_entres);
		vTracePrintF(synchronisation, "%d", synchro_cartons_entres);
		xEventGroupWaitBits(Synchro, synchro_paletiseur_vide, pdTRUE, pdTRUE, portMAX_DELAY);

		//Desactivation blocage
		send.actuator_msk = BLOCAGE_ENTREE_PALETISEUR_msk;
		send.actuator_state = ACTUATORS_OFF;
		psend = &send;
		xQueueSendToBack(xActuatorsQueue,&psend,0);
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

void vApplicationIdleHook()
{
  __WFI();
}


/*
 * 	Clock configuration for the Nucleo STM32F072RB board
 * 	HSE input Bypass Mode 			-> 8MHz
 * 	SYSCLK, AHB, APB1 				-> 48MHz
 *  PA8 as MCO with /16 prescaler 	-> 3MHz
 */

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSE error


	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency (required for high speed)
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	timeout = 1000;

	do
	{
		status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error


	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;

	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
	return (0);
}
