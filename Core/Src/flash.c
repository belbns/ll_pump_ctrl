#include "flash.h"

#include "stm32f0xx.h"
#include "stm32f030x6.h"


#define FLASH_KEYR_KEY1		((uint32_t)0x45670123)
#define FLASH_KEYR_KEY2		((uint32_t)0xcdef89ab)

static void FLASH_Lock(void);
static uint8_t FLASH_Unlock(void);
static void FLASH_wait_for_last_operation(void);
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static void FLASH_PageErase(uint32_t PageAddress);


static void FLASH_Lock(void)
{
  SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

static uint8_t FLASH_Unlock(void)
{
  uint8_t status = 0;

  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);
  
    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0)
    {
      status = 1;
    }                     
  }
  
  return status;
}

static void FLASH_wait_for_last_operation(void)
{
  // timeout?
  while (((FLASH->SR) & FLASH_SR_BSY) == FLASH_SR_BSY);
}

/*
uint8_t check_EOP(void)
{
	if(FLASH->SR & FLASH_SR_EOP)
	{	
		SET_BIT(FLASH->SR, FLASH_SR_EOP);
		return 1;
	}	
	return 0;
}	
*/

static void FLASH_PageErase(uint32_t PageAddress)
{
  /* Proceed to erase the page */
  SET_BIT(FLASH->CR, FLASH_CR_PER);
  WRITE_REG(FLASH->AR, PageAddress);
  SET_BIT(FLASH->CR, FLASH_CR_STRT);
}

static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_wait_for_last_operation();
  // разрешаем программирование полуслова (16 бит)
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  // пишем
  *(__IO uint16_t*)Address = Data;
  // ожидаем окончания операции
  FLASH_wait_for_last_operation();
  // запрещаем программирование
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
}

void save_parameters_to_flash(uint16_t *data_for_save, uint16_t data_length)
{
  FLASH_Unlock();
  FLASH_PageErase(PARAMETERS_PAGE);
  FLASH_wait_for_last_operation();
  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);    
  FLASH_Lock();

  uint32_t dst_addr = (uint32_t)PARAMETERS_PAGE;

  FLASH_Unlock();
  for (uint16_t i = 0; i < data_length; i++)
  {
    FLASH_Program_HalfWord((uint32_t)dst_addr, *data_for_save);
    dst_addr += 2;
    data_for_save++;
  }
  FLASH_Lock();
}


