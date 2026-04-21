/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_sccb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform a write transaction for 8-bit reg_addr and 8-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[in] reg_val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_reg_a8v8(esp_sccb_io_handle_t io_handle, uint8_t reg_addr, uint8_t reg_val);

/**
 * @brief Perform a write transaction for 16-bit reg_addr and 8-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[in] reg_val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint8_t reg_val);

/**
 * @brief Perform a write transaction for 8-bit reg_addr and 16-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[in] reg_val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_reg_a8v16(esp_sccb_io_handle_t io_handle, uint8_t reg_addr, uint16_t reg_val);

/**
 * @brief Perform a write transaction for 16-bit reg_addr and 16-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[in] reg_val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_reg_a16v16(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint16_t reg_val);

/**
 * @brief Perform a write transaction for 16-bit reg_addr and 32-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[in] reg_val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_reg_a16v32(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint32_t reg_val);

/**
 * @brief Perform a write-read transaction for 8-bit reg_addr and 8-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[out] reg_val Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb transmit-receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_receive_reg_a8v8(esp_sccb_io_handle_t io_handle, uint8_t reg_addr, uint8_t *reg_val);

/**
 * @brief Perform a write-read transaction for 16-bit reg_addr and 8-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[out] reg_val Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb transmit-receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint8_t *reg_val);

/**
 * @brief Perform a write-read transaction for 8-bit reg_addr and 16-bit reg_val.
 *
 * @param[in]  handle   SCCB IO handle
 * @param[in]  reg_addr address to send on the sccb bus.
 * @param[out] reg_val  Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb transmit-receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_receive_reg_a8v16(esp_sccb_io_handle_t io_handle, uint8_t reg_addr, uint16_t *reg_val);

/**
 * @brief Perform a write-read transaction for 16-bit reg_addr and 16-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[out] reg_val Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb transmit-receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_receive_reg_a16v16(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint16_t *reg_val);

/**
 * @brief Perform a write-read transaction for 16-bit reg_addr and 32-bit reg_val.
 *
 * @param[in] handle SCCB IO handle
 * @param[in] reg_addr address to send on the sccb bus.
 * @param[out] reg_val Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb transmit-receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_receive_reg_a16v32(esp_sccb_io_handle_t io_handle, uint16_t reg_addr, uint32_t *reg_val);

/**
 * @brief Block write: 16-bit register address (MSB first), data sent in chunks.
 *
 * I2C framing matches 16-bit-address sequential write: each transaction is
 * [RegAddr[15:8]][RegAddr[7:0]][payload...]. Payload is split into chunks
 * (default size 8 bytes); the start address is advanced in software per chunk.
 *
 * @param[in] io_handle SCCB IO handle
 * @param[in] start_addr First register address (16-bit, MSB first on bus)
 * @param[in] data       Data bytes to write (may be NULL if @p data_len is 0)
 * @param[in] data_len   Number of bytes to write
 * @return
 *      - ESP_OK: success
 *      - ESP_ERR_INVALID_ARG: invalid parameters
 *      - ESP_ERR_NOT_SUPPORTED: backend does not support 16-bit-address write
 */
esp_err_t esp_sccb_transmit_block_a16(esp_sccb_io_handle_t io_handle, uint16_t start_addr, const uint8_t *data, size_t data_len);

/**
 * @brief Block read: 16-bit register address then read @p data_len bytes.
 *
 * I2C/SCCB: write phase [RegAddr[15:8]][RegAddr[7:0]], then read @p data_len bytes
 * (repeated start + read), same as @ref esp_sccb_transmit_receive_reg_a16v8 with
 * multi-byte read.
 *
 * @param[in]  io_handle  SCCB IO handle
 * @param[in]  start_addr Register address (16-bit, MSB first on bus)
 * @param[out] data       Buffer for read data (may be NULL if @p data_len is 0)
 * @param[in]  data_len   Number of bytes to read
 * @return
 *      - ESP_OK: success
 *      - ESP_ERR_INVALID_ARG: invalid parameters
 *      - ESP_ERR_NOT_SUPPORTED: backend does not support 16-bit-address read
 */
esp_err_t esp_sccb_transmit_receive_block_a16(esp_sccb_io_handle_t io_handle, uint16_t start_addr, uint8_t *data, size_t data_len);

/**
 * @brief Perform a write transaction for 16-bit val.
 *
 * @param[in]  handle   SCCB IO handle
 * @param[in] val  Data to send on the sccb bus.
 * @return
 *      - ESP_OK: sccb transmit success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_transmit_v16(esp_sccb_io_handle_t io_handle, uint16_t val);

/**
 * @brief Perform a read transaction for 16-bit val.
 *
 * @param[in] handle SCCB IO handle
 * @param[out] val Data bytes received from sccb bus.
 * @return
 *      - ESP_OK: sccb receive success
 *      - ESP_ERR_INVALID_ARG: sccb transmit parameter invalid.
 */
esp_err_t esp_sccb_receive_v16(esp_sccb_io_handle_t io_handle, uint16_t *val);

/**
 * @brief Delete sccb I2C IO handle
 *
 * @param[in] handle SCCB IO handle
 * @return
 *        - ESP_OK: If controller is successfully deleted.
 */
esp_err_t esp_sccb_del_i2c_io(esp_sccb_io_handle_t io_handle);

#ifdef __cplusplus
}
#endif
