/* Minimal STM32F446RE register map for this bench firmware only (RM0390).
 * Kept local so the build needs nothing but arm-none-eabi-gcc. */
#ifndef F446_H
#define F446_H
#include <stdint.h>
#define REG(a) (*(volatile uint32_t *)(a))
#define RCC_BASE 0x40023800u
#define RCC_CR REG(RCC_BASE + 0x00)
#define RCC_PLLCFGR REG(RCC_BASE + 0x04)
#define RCC_CFGR REG(RCC_BASE + 0x08)
#define RCC_AHB1ENR REG(RCC_BASE + 0x30)
#define RCC_APB1ENR REG(RCC_BASE + 0x40)
#define RCC_APB2ENR REG(RCC_BASE + 0x44)
#define RCC_CSR REG(RCC_BASE + 0x74)
#define FLASH_ACR REG(0x40023C00u)
#define PWR_CR REG(0x40007000u)
#define RTC_BKP0R REG(0x40002850u)
#define GPIOA_BASE 0x40020000u
#define GPIOA_MODER REG(GPIOA_BASE + 0x00)
#define GPIOA_OSPEEDR REG(GPIOA_BASE + 0x08)
#define GPIOA_PUPDR REG(GPIOA_BASE + 0x0C)
#define GPIOA_ODR REG(GPIOA_BASE + 0x14)
#define GPIOA_BSRR REG(GPIOA_BASE + 0x18)
#define GPIOA_AFRL REG(GPIOA_BASE + 0x20)
#define GPIOA_AFRH REG(GPIOA_BASE + 0x24)
#define GPIOB_BASE 0x40020400u
#define GPIOB_MODER REG(GPIOB_BASE + 0x00)
#define GPIOB_OTYPER REG(GPIOB_BASE + 0x04)
#define GPIOB_OSPEEDR REG(GPIOB_BASE + 0x08)
#define GPIOB_ODR REG(GPIOB_BASE + 0x14)
#define GPIOB_BSRR REG(GPIOB_BASE + 0x18)
#define GPIOC_BASE 0x40020800u
#define GPIOC_MODER REG(GPIOC_BASE + 0x00)
#define GPIOC_OTYPER REG(GPIOC_BASE + 0x04)
#define GPIOC_OSPEEDR REG(GPIOC_BASE + 0x08)
#define GPIOC_ODR REG(GPIOC_BASE + 0x14)
#define GPIOC_BSRR REG(GPIOC_BASE + 0x18)
#define TIM2_BASE 0x40000000u
#define TIM2_CR1 REG(TIM2_BASE + 0x00)
#define TIM2_EGR REG(TIM2_BASE + 0x14)
#define TIM2_CNT REG(TIM2_BASE + 0x24)
#define TIM2_PSC REG(TIM2_BASE + 0x28)
#define TIM2_ARR REG(TIM2_BASE + 0x2C)
#define USART1_BASE 0x40011000u
#define USART2_BASE 0x40004400u
#define USART_SR(b) REG((b) + 0x00)
#define USART_DR(b) REG((b) + 0x04)
#define USART_BRR(b) REG((b) + 0x08)
#define USART_CR1(b) REG((b) + 0x0C)
#define USART_SR_PE (1u << 0)
#define USART_SR_FE (1u << 1)
#define USART_SR_NF (1u << 2)
#define USART_SR_ORE (1u << 3)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TXE (1u << 7)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TXEIE (1u << 7)
#define USART_CR1_UE (1u << 13)
#define NVIC_ISER1 REG(0xE000E104u)
#define UID_BASE 0x1FFF7A10u
static inline void irq_off(void) { __asm volatile ("cpsid i" ::: "memory"); }
static inline void irq_on(void) { __asm volatile ("cpsie i" ::: "memory"); }
#endif
