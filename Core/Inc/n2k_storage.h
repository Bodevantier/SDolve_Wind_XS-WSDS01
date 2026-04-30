/**
 * n2k_storage.h
 *
 * Persistence of NMEA 2000 negotiated source address in the last
 * flash page of the STM32F103 (0x0801FC00..0x0801FFFF, 1 KB).
 *
 * The linker script reserves this page by limiting FLASH to 127 KB.
 *
 * Storage format (32-bit words, little-endian):
 *   word[0] = magic (0x4E324B53 = "N2KS")
 *   word[1] = ((uint32_t)source_address) | (~source_address << 8)
 *             (low byte = address, next byte = bitwise NOT for integrity)
 *
 * If the page is empty (0xFFFF) or magic mismatches, Load returns false
 * and the caller falls back to its default preferred address.
 */
#ifndef N2K_STORAGE_H
#define N2K_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Read the persisted N2K source address.
 *
 * @param  out_addr  receives the stored address on success
 * @return true if a valid stored address was found
 */
bool N2kStorage_LoadSource(uint8_t *out_addr);

/**
 * Persist the given source address, erasing/rewriting the page only
 * when the new value differs from the currently stored one.
 *
 * @return true on success (or no-op when value already matches)
 */
bool N2kStorage_SaveSource(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* N2K_STORAGE_H */
