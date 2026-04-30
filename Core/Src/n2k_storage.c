/**
 * n2k_storage.c
 *
 * See n2k_storage.h for layout. Uses HAL_FLASH for word programming
 * and a single-page erase. STM32F103 medium-density: page = 1 KB.
 */
#include "n2k_storage.h"
#include "stm32f1xx_hal.h"

#define N2K_STORAGE_ADDR    (0x0801FC00UL)   /* last 1 KB page */
#define N2K_STORAGE_PAGE    (0x0801FC00UL)
#define N2K_STORAGE_MAGIC   (0x4E324B53UL)   /* 'N','2','K','S' */

static inline uint32_t pack_addr(uint8_t a) {
  return ((uint32_t)a) | ((uint32_t)((uint8_t)~a) << 8);
}

static bool unpack_addr(uint32_t w, uint8_t *out) {
  uint8_t a  = (uint8_t)(w & 0xFF);
  uint8_t na = (uint8_t)((w >> 8) & 0xFF);
  if ((uint8_t)~a != na) return false;
  *out = a;
  return true;
}

bool N2kStorage_LoadSource(uint8_t *out_addr)
{
  if (out_addr == NULL) return false;

  const uint32_t magic = *(const volatile uint32_t *)(N2K_STORAGE_ADDR + 0);
  const uint32_t value = *(const volatile uint32_t *)(N2K_STORAGE_ADDR + 4);

  if (magic != N2K_STORAGE_MAGIC) return false;
  return unpack_addr(value, out_addr);
}

bool N2kStorage_SaveSource(uint8_t addr)
{
  /* Skip write when the stored value already matches */
  uint8_t current = 0;
  if (N2kStorage_LoadSource(&current) && current == addr) return true;

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef er = {0};
  er.TypeErase   = FLASH_TYPEERASE_PAGES;
  er.Banks       = FLASH_BANK_1;
  er.PageAddress = N2K_STORAGE_PAGE;
  er.NbPages     = 1;

  uint32_t page_err = 0;
  if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK) {
    HAL_FLASH_Lock();
    return false;
  }

  HAL_StatusTypeDef s1 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                           N2K_STORAGE_ADDR + 0,
                                           N2K_STORAGE_MAGIC);
  HAL_StatusTypeDef s2 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                           N2K_STORAGE_ADDR + 4,
                                           pack_addr(addr));

  HAL_FLASH_Lock();
  return (s1 == HAL_OK) && (s2 == HAL_OK);
}
