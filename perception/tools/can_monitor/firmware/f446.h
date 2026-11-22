#ifndef MICROROS_F446_REGS_H
#define MICROROS_F446_REGS_H
#include <stdint.h>
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define RCC_BASE 0x40023800u
#define RCC_CR REG(RCC_BASE + 0x00u)
#define RCC_PLLCFGR REG(RCC_BASE + 0x04u)
#define RCC_CFGR REG(RCC_BASE + 0x08u)
#define RCC_AHB1ENR REG(RCC_BASE + 0x30u)
#define RCC_APB1ENR REG(RCC_BASE + 0x40u)
#define RCC_CSR REG(RCC_BASE + 0x74u)
#define FLASH_ACR REG(0x40023C00u)
#define GPIOA_BASE 0x40020000u
#define GPIOA_MODER REG(GPIOA_BASE + 0x00u)
#define GPIOA_OSPEEDR REG(GPIOA_BASE + 0x08u)
#define GPIOA_PUPDR REG(GPIOA_BASE + 0x0Cu)
#define GPIOA_BSRR REG(GPIOA_BASE + 0x18u)
#define GPIOA_AFRL REG(GPIOA_BASE + 0x20u)
#define GPIOB_BASE 0x40020400u
#define GPIOB_MODER REG(GPIOB_BASE + 0x00u)
#define GPIOB_OTYPER REG(GPIOB_BASE + 0x04u)
#define GPIOB_OSPEEDR REG(GPIOB_BASE + 0x08u)
#define GPIOB_PUPDR REG(GPIOB_BASE + 0x0Cu)
#define GPIOB_BSRR REG(GPIOB_BASE + 0x18u)
#define GPIOB_AFRH REG(GPIOB_BASE + 0x24u)
#define GPIOC_BASE 0x40020800u
#define GPIOC_MODER REG(GPIOC_BASE + 0x00u)
#define GPIOC_OTYPER REG(GPIOC_BASE + 0x04u)
#define GPIOC_OSPEEDR REG(GPIOC_BASE + 0x08u)
#define GPIOC_BSRR REG(GPIOC_BASE + 0x18u)
#define TIM2_BASE 0x40000000u
#define TIM2_CR1 REG(TIM2_BASE + 0x00u)
#define TIM2_EGR REG(TIM2_BASE + 0x14u)
#define TIM2_CNT REG(TIM2_BASE + 0x24u)
#define TIM2_PSC REG(TIM2_BASE + 0x28u)
#define TIM2_ARR REG(TIM2_BASE + 0x2Cu)
#ifdef UROS_USART1
/* micro-ROS over USART1 PA9/PA10 (Arduino D8/D2) = the Pi GPIO UART wires, no USB needed */
#define UROS_USART_BASE 0x40011000u
#else
/* micro-ROS over USART2 PA2/PA3 = ST-Link virtual COM port */
#define UROS_USART_BASE 0x40004400u
#endif
#define RCC_APB2ENR REG(RCC_BASE + 0x44u)
#define GPIOA_AFRH REG(GPIOA_BASE + 0x24u)
#define USART_SR REG(UROS_USART_BASE + 0x00u)
#define USART_DR REG(UROS_USART_BASE + 0x04u)
#define USART_BRR REG(UROS_USART_BASE + 0x08u)
#define USART_CR1 REG(UROS_USART_BASE + 0x0Cu)
#define USART_SR_PE (1u << 0)
#define USART_SR_FE (1u << 1)
#define USART_SR_NF (1u << 2)
#define USART_SR_ORE (1u << 3)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TXE (1u << 7)
#define USART_SR_TC (1u << 6)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE (1u << 13)
#define NVIC_ISER1 REG(0xE000E104u)
static inline void irq_off(void) { __asm volatile("cpsid i" ::: "memory"); }
static inline void irq_on(void) { __asm volatile("cpsie i" ::: "memory"); }
#endif
