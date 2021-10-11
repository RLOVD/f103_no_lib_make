
#include "main.h"
#include "stm32f1xx.h"

#define GPIO_PIN_13         (1 << 13)

void SystemClock_Config(void);
static void GPIO_Init(void);

int main(void)
{
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        for (volatile uint32_t i = 0; i < 300000; i++) {}
        GPIOC->ODR ^= GPIO_PIN_13;
    }
}

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
    // тактирование порта A и C
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // PC13 - выход, pp
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);  //для начала все сбрасываем в ноль

    //MODE: выход с максимальной частотой 2 МГц
    //CNF: режим push-pull
    GPIOC->CRH |= (0x02 << GPIO_CRH_MODE13_Pos) | (0x00 << GPIO_CRH_CNF13_Pos);
}

void Error_Handler(void) 
{
    __disable_irq();
    while (1) {
    }
}
