
#include "main.h"
#include "stm32f1xx.h"

#define GPIO_PIN_13                                 (1U << 13)

#define FLASH_APP_START_ADDRESS                     0x08002000U

#define SYSCLOCK                                    16000000U

//#define BUILD_BOOTLOADER

void SystemClock_Config(void);
static void GPIO_Init(void);
void delay_ms(uint32_t ms);

static void uart1_init(void);
void uart1_send_byte(uint8_t byte);
void uart1_send_str(char *str);

/* Function pointer for jumping to user application. */
typedef void (*pfunction)(void);
void flash_jump_to_app(void);


static volatile uint32_t tick = 0;
static volatile uint32_t delay_cnt = 0;

void func1(void)
{
    char str[] = "Im func1\r\n";
    uart1_send_str(str);
}

void SysTick_Init(void)
{
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, SYSCLOCK / 1000 - 1);
    CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

int main(void) {
    SystemClock_Config();
    SysTick_Init();
    GPIO_Init();
    uart1_init();

    while (1) {
#if defined(BUILD_BOOTLOADER)
        // bootloader
        while (tick < 5000) {
            // for (volatile uint32_t i = 0; i < 30000; i++) {} // задержка
            delay_ms(30);
            GPIOC->ODR ^= GPIO_PIN_13;
        }
        flash_jump_to_app();
#else
        // app
        // for (volatile uint32_t i = 0; i < 300000; i++) {} // задержка
        delay_ms(500);
        GPIOC->ODR ^= GPIO_PIN_13;
#endif
    }
}

#if defined(BUILD_BOOTLOADER)
void flash_jump_to_app(void)
{
    RCC->APB1RSTR = 0xFFFFFFFF;
    RCC->APB1RSTR = 0x00;
    RCC->APB2RSTR = 0xFFFFFFFF;
    RCC->APB2RSTR = 0x00;

    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    SysTick->LOAD = 0;

    __disable_irq();

    __set_BASEPRI(0);
    __set_CONTROL(0);
    for (uint8_t i = 0; i < 3; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    __enable_irq();

    /* Function pointer to the address of the user application. */
    pfunction jump_to_app;
    jump_to_app = (pfunction)(*(volatile uint32_t *)(FLASH_APP_START_ADDRESS + 4));

    /* Change the main and local  stack pointer. */
    __set_MSP(*(volatile uint32_t *)FLASH_APP_START_ADDRESS);
    SCB->VTOR = *(volatile uint32_t *)FLASH_APP_START_ADDRESS;

    jump_to_app();
}
#endif

void SystemClock_Config(void)
{
    volatile uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* Конфигурация  SYSCLK, HCLK, PCLK2 и PCLK1 */
    /* Включаем HSE */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);

    /* Ждем пока HSE не выставит бит готовности либо не выйдет таймаут*/
    do {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != 10000));

    if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
        HSEStatus = (uint32_t)0x01;
    } else {
        HSEStatus = (uint32_t)0x00;
    }

    /* Если HSE запустился нормально */
    if (HSEStatus == (uint32_t)0x01) {
        /* Включаем буфер предвыборки FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTBE;

        /* Конфигурируем Flash на 2 цикла ожидания */
        /* Это нужно потому, что Flash не может работать на высокой частоте */

        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;

        /* HCLK = SYSCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

        /* PCLK2 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

        /* PCLK1 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

        /* Конфигурируем множитель PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
        /* При условии, что кварц на 8МГц! */
        /* RCC_CFGR_PLLMULL9 - множитель на 9. Если нужна другая частота, не 72МГц */
        /* то выбираем другой множитель. */
        RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL2);  // 16 MHz

        /* Включаем PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Ожидаем, пока PLL выставит бит готовности */
        while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
            // Ждем
        }

        /* Выбираем PLL как источник системной частоты */
        RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

        /* Ожидаем, пока PLL выберется как источник системной частоты */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) {
            // Ждем
        }

    } else {
        /* Все плохо... HSE не завелся... Чего-то с кварцем или еще что...
      	Надо бы както обработать эту ошибку... Если мы здесь, то мы работаем
      	от HSI! */
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // тактирование порта C

    // PC13 - выход, pp
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);  //для начала все сбрасываем в ноль

    //MODE: выход с максимальной частотой 2 МГц
    //CNF: режим push-pull
    GPIOC->CRH |= (0x02 << GPIO_CRH_MODE13_Pos) | (0x00 << GPIO_CRH_CNF13_Pos);
}

static void uart1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    // тактирование порта A
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // включаем тактирование UART1

    // настройка вывода PA9 (TX1) на режим альтернативной функции с активным выходом
    GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

    // настройка вывода PA10 (RX1) на режим входа с подтягивающим резистором
    GPIOA->CRH &= (~GPIO_CRH_CNF10_0);
    GPIOA->CRH |= GPIO_CRH_CNF10_1;
    GPIOA->CRH &= (~(GPIO_CRH_MODE10));
    GPIOA->BSRR |= GPIO_ODR_ODR10;

    USART1->CR1 = USART_CR1_UE;  // разрешаем USART1, сбрасываем остальные биты

    //USART_BRR = (Fck / (16 * BAUD)) * 16 = 16000000 / 9600 = 1666,666

    USART1->BRR = 1667;  // скорость 9600 бод , пока что в тупняка влупил без вычисления , сейчас не суть))

    //Разрешаем работу приемника и передатчика
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // разрешаем приемник и передатчик
    USART1->CR2 = 0;
    USART1->CR3 = 0;
}

void uart1_send_byte(uint8_t byte)
{
    while ((USART1->SR & USART_SR_TXE) == 0) {
    }
    USART1->DR = byte;
}

void uart1_send_str(char *str)
{
    while (*str != '\0') {
        while ((USART1->SR & USART_SR_TXE) == 0) {
        }
        USART1->DR = *str;
        str++;
    }
}

void delay_ms(uint32_t ms)
{
    static uint32_t old_tick;
    old_tick = tick;

    while (tick - old_tick < ms) {
    }
}

void Error_Handler(void) 
{
    __disable_irq();
    while (1) {
    }
}

void SysTick_Handler(void)
{
    tick++;
}
