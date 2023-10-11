/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <stdlib.h>
#include <string.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* IOS                                                                  */
/************************************************************************/

#define BTN_PIO PIOA
#define BTN_PIO_ID ID_PIOA
#define BTN_PIO_PIN 11
#define BTN_PIO_PIN_MASK (1u << BTN_PIO_PIN)

#define BUZZER_PIO PIOD
#define BUZZER_PIO_ID  ID_PIOD
#define BUZZER_PIO_PIN 30
#define BUZZER_PIO_PIN_MASK (1u << BUZZER_PIO_PIN)

#define NOTE_B5  988
#define NOTE_E6  1319
/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/

void btn_init(void);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* rtos vars                                                            */
/************************************************************************/


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*4/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_PLAY_STACK_SIZE                (1024*4/sizeof(portSTACK_TYPE))
#define TASK_PLAY_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_COINS_STACK_SIZE                (1024*4/sizeof(portSTACK_TYPE))
#define TASK_COINS_STACK_PRIORITY            (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphoreInit;
QueueHandle_t xQueueCoins;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	xSemaphoreGiveFromISR(xSemaphoreInit, 0);
}

void set_buzzer(void);

void set_buzzer(void){
	pio_clear(BUZZER_PIO, BUZZER_PIO_PIN_MASK);
}

void clear_buzzer(void);

void clear_buzzer(void){
	pio_set(BUZZER_PIO, BUZZER_PIO_PIN_MASK);
}
void buzzer_test(int freq);

void buzzer_test(int freq){
	set_buzzer();
	delay_us(freq);
	clear_buzzer();
	delay_us(freq);
}
void tone(int freq, int time);

void tone(int freq, int	time){
	
	int t = (1000000/freq);
	int ciclo = time*1000/(2*t);

		
	for (int i = 0; i<ciclo; i++){
		buzzer_test(t);
	}
	
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/


static void task_debug(void *pvParameters) {
	gfx_mono_ssd1306_init();

	for (;;) {
		gfx_mono_draw_filled_circle(10,10,4,1,GFX_WHOLE);
		vTaskDelay(150);
		gfx_mono_draw_filled_circle(10,10,4,0,GFX_WHOLE);
		vTaskDelay(150);

	}
}
static void task_coins(void *pvParameters){
	btn_init();
	int i = 0;
	int t = 0;
	int coins = 0;
	RTT_init(32000,0,0);
	for(;;){
		if(xSemaphoreTake(xSemaphoreInit, 0)){
			if(i==0){
				t = rtt_read_timer_value(RTT);
				printf("Seed: %d \n",t);
				srand(t);
				i++;
			}
			coins = rand() % 3 + 1;
			printf("Coins: %d \n",coins);
			xQueueSend(xQueueCoins, &coins,0);
		}

	}
}

static void task_play(void *pvParameters){
	int coins = 0;
	for(;;){
		if(xQueueReceive(xQueueCoins,&coins,1000)){
			for(int num_coins = 0; num_coins<coins;num_coins++){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
			}
		}

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void init_periph(void){
	pmc_enable_periph_clk(BUZZER_PIO_ID);
	pio_set_output(BUZZER_PIO,BUZZER_PIO_PIN_MASK,0,0,0);
}
void btn_init(void) {
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BTN_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BTN_PIO, PIO_INPUT, BTN_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN_PIO, BTN_PIO_PIN_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BTN_PIO,
	BTN_PIO_ID,
	BTN_PIO_PIN_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BTN_PIO, BTN_PIO_PIN_MASK);
	pio_get_interrupt_status(BTN_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BTN_PIO_ID);
	NVIC_SetPriority(BTN_PIO_ID, 4); // Prioridade 4
}


void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	/* Initialize the console uart */
	configure_console();
	xSemaphoreInit = xSemaphoreCreateBinary();
	if (xSemaphoreInit == NULL)
	printf("falha em criar o semaforo \n");
	
	xQueueCoins = xQueueCreate(32, sizeof(int));
	if (xQueueCoins == NULL)
	printf("falha em criar a queue Coins \n");
	
	
	init_periph();
	
	tone(NOTE_B5,  80);
	tone(NOTE_E6, 640);
	if (xTaskCreate(task_play, "play", TASK_PLAY_STACK_SIZE, NULL,
	TASK_PLAY_STACK_PRIORITY+1, NULL) != pdPASS) {
		printf("Failed to create play task\r\n");
	}
	if (xTaskCreate(task_debug, "debug", TASK_OLED_STACK_SIZE, NULL,
	TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create debug task\r\n");
	}
	if (xTaskCreate(task_coins, "coins", TASK_COINS_STACK_SIZE, NULL,
	TASK_COINS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create coins task\r\n");
	}
	

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
