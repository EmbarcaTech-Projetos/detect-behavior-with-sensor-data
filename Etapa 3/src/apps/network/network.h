#ifndef _NETWORK_H_
#define _NETWORK_H_

#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// Define a structure to hold the state of our network request
typedef struct HTTP_REQUEST_STATE_T {
    struct tcp_pcb *tcp_pcb; // TCP Protocol Control Block
    ip_addr_t remote_addr;   // Remote server IP address
    char *request_body;      // The data to send (e.g., JSON payload)
    int sent_len;            // Length of the data sent so far
    bool complete;           // Flag to indicate if the request is finished
} HTTP_REQUEST_STATE_T;

/**
 * @brief Initializes the Wi-Fi module and connects to the specified network.
 * * This function must be called once at the beginning of the program.
 * It handles the initialization of the CYW43 Wi-Fi chip and attempts to
 * connect to the Wi-Fi network specified by WIFI_SSID and WIFI_PASSWORD.
 * * @return true if initialization and connection are successful, false otherwise.
 */
bool network_init(void);

/**
 * @brief Sends sensor data asynchronously via an HTTP POST request.
 * * This function is non-blocking. It formats and initiates an HTTP POST request
 * and returns immediately. The actual network transaction happens in the
 * background. The main loop should continue to call cyw43_arch_poll()
 * to allow the network stack to process the request.
 *
 * @param json_payload A null-terminated string containing the data (e.g., in JSON format) to be sent.
 * @return A pointer to the HTTP request state object if the request was successfully initiated,
 * or NULL if another request is already in progress or an error occurred.
 */
HTTP_REQUEST_STATE_T* network_send_post_request(const char *json_payload);


#endif // _NETWORK_H_