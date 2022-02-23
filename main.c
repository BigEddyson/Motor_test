#include "stm32f103xb.h"

static uint8_t count = 0;
static uint8_t count2 = 0;

void bus_init(void) {
  //RCC setting
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; 
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
}
/***************************************************************************/
void GPIO_init(void) {
  //PA5 - floating input
  GPIOA->CRL &= ~GPIO_CRL_MODE5; 
  GPIOA->CRL |= GPIO_CRL_CNF5_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
  // PA3 as output, alternative, push-pull
  GPIOA->CRL &= ~GPIO_CRL_CNF3;
  GPIOA->CRL |= GPIO_CRL_CNF3_1;
  GPIOA->CRL &= ~GPIO_CRL_MODE3;
  GPIOA->CRL |= GPIO_CRL_MODE3_1;
  // PC13 as output, LED
  GPIOC->CRH &= ~GPIO_CRH_CNF13; 
  GPIOC->CRH |= GPIO_CRH_MODE13;
}
/***************************************************************************/
void TIM_init(){
  // TiM2 init
  TIM2->CR1 &= ~TIM_CR1_DIR;                                                    // upcounting
  TIM2->PSC = 400-1;                                                            //timer prescaler
  TIM2->ARR = 100-1;                                                            // 100 Hz 
  TIM2->CCR4 = 50-1;                                                            // Duty cycle 50%
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;                           //compare settings
  TIM2->CCER |= TIM_CCER_CC4E;                                                  
  //enable_interrupt;
  TIM2->EGR |= TIM_EGR_CC4G;
  TIM2->DIER |= TIM_DIER_CC4IE;
   // TiM3 init
  TIM3->CR1 &= ~TIM_CR1_DIR;                                                     // upcounting
  TIM3->PSC = 800-1;                                                             //timer prescaler
  TIM3->ARR = 10000-1;                                                          // 1 Hz
  //enable_interrup
  TIM3->DIER |= TIM_DIER_UIE;
  TIM3->EGR |= TIM_EGR_UG;
  //TIM3->CR1 |= TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM3_IRQn);     
} 
/***************************************************************************/
void motorDrive(uint8_t i) {                                                    //The argument enables(1) or disables(0) interrupt
  TIM2->CR1 |= TIM_CR1_CEN;                                                     // counting
  i > 0 ? NVIC_EnableIRQ(TIM2_IRQn) : NVIC_DisableIRQ(TIM2_IRQn);
}
/***************************************************************************/
void motorInit(void){
  motorDrive(0);
  while(!(GPIOA->IDR & GPIO_IDR_IDR5)){
  }
  TIM2->CR1 &= ~TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM2_IRQn); 
}
/***************************************************************************/
/***************************************************************************/
void main()
{
  bus_init();                                                                   //Peripheral initialization 
  GPIO_init();
  TIM_init();
  motorInit();                                                                  //adjusting the motor position and checking the position during operation 
  __iar_builtin_enable_interrupt();  
  motorDrive(1);                                                                //motor running
  while(1)
  {
    
  }
}
/***************************************************************************/
/***************************************************************************/
void TIM2_IRQHandler (void){
  TIM2->SR &= ~TIM_SR_CC4IF;
  count++;
  if(count == 200){
    count = 0;
    GPIOC->ODR ^= GPIO_ODR_ODR13;
    if(count2%2 == 0 & !(GPIOA->IDR & GPIO_IDR_IDR5)){                          //motor position check 
      motorInit();
      }
      TIM2->CR1 &= ~TIM_CR1_CEN;
      TIM3->CR1 |= TIM_CR1_CEN;
  }
}
/***************************************************************************/
void TIM3_IRQHandler (void){
  TIM3->SR &= ~TIM_SR_UIF;
  GPIOC->ODR ^= GPIO_ODR_ODR13;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN;
}
