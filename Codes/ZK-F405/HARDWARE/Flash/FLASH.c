#include "flash.h"
#include "stm32f4xx_flash.h"

typedef enum {FAILED = 0, PASSED = !FAILED} Status;//定义要使用的状态枚举变量




uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t First_Page = 0x00;                       //起始页


__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO Status MemoryProgramStatus = PASSED;
//函数声明


//=============================================================================
//文件名称：main
//功能概要：主函数
//参数说明：无
//函数返回：int
//=============================================================================
void FLASH_write(int16_t *data,uint8_t len)
{
  FLASH_Unlock();//先解锁
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR); //清楚相应的标志位
//  First_Page = (FLASH_End_Addr - FLASH_Start_Addr+1) / FLASH_Page_Size;//计算出起始页
  /* 使用前先擦除 */
//  for(EraseCounter = 0; (EraseCounter < First_Page) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
//    if (FLASH_ErasePage(FLASH_Start_Addr)!= FLASH_COMPLETE)
//    {
//      while (1)
//      {			
//      }
//    }
  }
	
  /* 写入FLASH */
  Address = FLASH_Start_Addr;
  while (len--)
  {
    if (FLASH_ProgramHalfWord(Address, *data) == FLASH_COMPLETE)
    {
		data++;
		Address = Address + 2;
    }
    else
    { 
      while (1)
      {
      }
    }
  }
  FLASH_Lock();
	/* 上锁 */
}	




void FLASH_read(int16_t *data,uint8_t len)
{
  Address = FLASH_Start_Addr;
  
  while (len--)
  {
    *data = *(__IO int16_t *)Address;
		data++;
    Address = Address + 2;
  }
}

/*****END OF FILE****/
