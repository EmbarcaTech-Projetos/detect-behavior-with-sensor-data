#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// Opaque struct to hide implementation details from main.c
typedef struct HTTP_REQUEST_STATE_T HTTP_REQUEST_STATE_T;

/**
 * @brief Initializes the Wi-Fi connection with automatic retries.
 * * This function will block until a Wi-Fi connection is successfully established.
 * It will endlessly retry if the connection fails.
 * * @return true on success, false on hardware initialization failure.
 */
bool network_init(void);

/**
 * @brief Checks the Wi-Fi connection and attempts to reconnect if it has dropped.
 *
 * This function should be called periodically in the main application loop to 
 * maintain a stable connection for long-running devices.
 */
void network_ensure_connected(void);

/**
 * @brief Starts a new HTTP POST request by resolving DNS and connecting.
 *
 * This is a non-blocking function. The status of the connection must be
 * polled using network_is_ready_for_data().
 *
 * @param content_length The total length of the POST body to be sent.
 * @return A pointer to a state object, or NULL on allocation failure.
 */
HTTP_REQUEST_STATE_T* network_start_post_request(size_t content_length);

/**
 * @brief Checks if the network connection is established and ready to accept data chunks.
 * @param state The request state object.
 * @return true if ready for data, false otherwise.
 */
bool network_is_ready_for_data(HTTP_REQUEST_STATE_T *state);

/**
 * @brief Writes a chunk of data to the TCP send buffer.
 *
 * This function will fail if the TCP send buffer is full. The caller must
 * retry if this function returns false.
 *
 * @param state The request state object.
 * @param data Pointer to the data chunk to send.
 * @param len Length of the data chunk.
 * @return true if the data was queued successfully, false if the buffer is full.
 */
bool network_write_chunk(HTTP_REQUEST_STATE_T *state, const char *data, size_t len);

/**
 * @brief Checks if the entire HTTP request/response cycle is complete.
 * @param state The request state object.
 * @return true if the request is finished (successfully or not), false if in progress.
 */
bool network_is_request_complete(HTTP_REQUEST_STATE_T *state);

/**
 * @brief Frees all resources associated with a request state.
 *
 * Must be called after network_is_request_complete() returns true.
 * @param state The request state object to free.
 */
void network_free_request(HTTP_REQUEST_STATE_T *state);

#endif // NETWORK_H