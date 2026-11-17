/**
  ******************************************************************************
  * @file    crc.h
  * @brief   Checksums used by the telemetry station. Pure C, no RTOS calls,
  *          safe from any task or ISR (no shared state).
  ******************************************************************************
  */
#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

/* CRC-8/SMBUS over the application packet (docs 15.1, D6).
 * poly 0x07, init 0x00, no reflection, xorout 0x00.
 * Check value: Crc8_Compute( "123456789", 9 ) == 0xF4.
 * A NULL pointer or a zero length returns 0x00 (the CRC of nothing). */
uint8_t Crc8_Compute( const uint8_t * pucData, size_t uxLen );

/* Ethernet FCS (IEEE 802.3 CRC-32, same as zlib crc32) over a frame without
 * its FCS (docs 13.4, D7). Reflected poly 0xEDB88320, init and final xor
 * 0xFFFFFFFF. Check value: Crc32_Eth( "123456789", 9 ) == 0xCBF43926.
 * On the wire the FCS follows the frame least significant byte first:
 *   fcs[0] = crc & 0xFF, fcs[1] = (crc >> 8) & 0xFF, ...
 * A NULL pointer or a zero length returns 0x00000000 (the CRC of nothing). */
uint32_t Crc32_Eth( const uint8_t * pucData, size_t uxLen );

#endif /* CRC_H */
