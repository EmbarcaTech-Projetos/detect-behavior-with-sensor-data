#include <string.h>
#include <stdlib.h>
#include "pico/cyw43_arch.h"
#include "pico/time.h" // Added for sleep_ms
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "network.h"

// --- Configuration ---
#define WIFI_SSID "XXXXX"
#define WIFI_PASSWORD "XXXX"
#define API_HOST "XXX.XXX.XXX.XXX"
#define API_URL "/api/"
#define API_PORT 80
#define WIFI_RETRY_DELAY_MS 5000 // Time to wait between connection retries

// --- Internal State ---
typedef enum {
    REQ_STATE_DNS_LOOKUP,
    REQ_STATE_CONNECTING,
    REQ_STATE_SENDING_HEADERS,
    REQ_STATE_SENDING_BODY,
    REQ_STATE_DONE,
    REQ_STATE_ERROR
} INTERNAL_REQUEST_STATUS;

struct HTTP_REQUEST_STATE_T {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    INTERNAL_REQUEST_STATUS status;
    size_t content_length_total;
    size_t content_length_sent;
    bool ready_for_chunk; 
};

// --- Internal Callbacks ---
static err_t http_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t http_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void http_client_err_cb(void *arg, err_t err);
static err_t http_client_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_client_close(HTTP_REQUEST_STATE_T *state);
static void http_dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg);

// --- Internal Helper Functions ---

/**
 * @brief Internal function to connect to Wi-Fi with infinite retries.
 */
static void network_connect_with_retry(void) {
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi network: %s\n", WIFI_SSID);

    // Loop until the connection is successful
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Failed to connect. Retrying in %d ms...\n", WIFI_RETRY_DELAY_MS);
        sleep_ms(WIFI_RETRY_DELAY_MS);
    }
    
    printf("Successfully connected to Wi-Fi.\n");
}


// --- Public Functions ---

bool network_init(void) {
    if (cyw43_arch_init()) {
        printf("Wi-Fi hardware init failed.\n");
        return false;
    }
    // This function will now block until connected
    network_connect_with_retry();
    return true;
}

void network_ensure_connected(void) {
    int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
    // Check for connection failure or disconnection
    if (link_status < 0 || link_status == CYW43_LINK_DOWN) {
        printf("Wi-Fi connection lost (status: %d). Reconnecting...\n", link_status);
        network_connect_with_retry();
    }
}

HTTP_REQUEST_STATE_T* network_start_post_request(size_t content_length) {
    // Before starting a new request, ensure the connection is active
    network_ensure_connected();

    HTTP_REQUEST_STATE_T *state = calloc(1, sizeof(HTTP_REQUEST_STATE_T));
    if (!state) {
        printf("Failed to allocate memory for request state.\n");
        return NULL;
    }

    state->content_length_total = content_length;
    state->status = REQ_STATE_DNS_LOOKUP;
    state->ready_for_chunk = false;

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(API_HOST, &state->remote_addr, http_dns_found_cb, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        // DNS result was cached, call callback immediately
        http_dns_found_cb(API_HOST, &state->remote_addr, state);
    } else if (err != ERR_INPROGRESS) {
        // An immediate error occurred
        printf("DNS request failed immediately: %d\n", err);
        free(state);
        return NULL;
    }
    
    return state;
}

bool network_is_ready_for_data(HTTP_REQUEST_STATE_T *state) {
    if (!state) return false;
    return state->status == REQ_STATE_SENDING_BODY && state->ready_for_chunk;
}

bool network_write_chunk(HTTP_REQUEST_STATE_T *state, const char *data, size_t len) {
    if (!state || !state->tcp_pcb) {
        return false;
    }

    err_t err = ERR_OK;
    cyw43_arch_lwip_begin();

    if (tcp_sndbuf(state->tcp_pcb) < len) {
        // Not enough space in the buffer
        state->ready_for_chunk = false; 
        err = ERR_MEM;
    } else {
        err = tcp_write(state->tcp_pcb, data, len, TCP_WRITE_FLAG_COPY);
        if (err == ERR_OK) {
            state->content_length_sent += len;
            // Nudge lwIP to send the data out
            err = tcp_output(state->tcp_pcb);
            if (err != ERR_OK) {
                printf("tcp_output failed with error %d\n", err);
            }
            state->ready_for_chunk = false;
        } else {
            printf("tcp_write failed with error %d\n", err);
        }
    }
    
    cyw43_arch_lwip_end();
    return err == ERR_OK;
}

bool network_is_request_complete(HTTP_REQUEST_STATE_T *state) {
    if (!state) return true;
    return state->status == REQ_STATE_DONE || state->status == REQ_STATE_ERROR;
}

void network_free_request(HTTP_REQUEST_STATE_T *state) {
    if (state) {
        if (state->tcp_pcb) {
            http_client_close(state);
        }
        free(state);
    }
}

// --- Internal Callbacks and Helper Functions ---

static void http_dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    if (ipaddr) {
        state->remote_addr = *ipaddr;
        printf("DNS resolved for %s: %s\n", name, ip4addr_ntoa(ipaddr));
        state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(ipaddr));
        if (!state->tcp_pcb) {
            printf("Failed to create TCP PCB.\n");
            state->status = REQ_STATE_ERROR;
            return;
        }

        // Disable Nagle's Algorithm to send data immediately
        tcp_nagle_disable(state->tcp_pcb);

        tcp_arg(state->tcp_pcb, state);
        tcp_sent(state->tcp_pcb, http_client_sent_cb);
        tcp_recv(state->tcp_pcb, http_client_recv_cb);
        tcp_err(state->tcp_pcb, http_client_err_cb);

        state->status = REQ_STATE_CONNECTING;
        cyw43_arch_lwip_begin();
        err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, API_PORT, http_client_connected_cb);
        cyw43_arch_lwip_end();
        if (err != ERR_OK) {
            printf("TCP connect failed: %d\n", err);
            http_client_close(state);
        }
    } else {
        printf("DNS resolution failed for %s\n", name);
        state->status = REQ_STATE_ERROR;
    }
}

static err_t http_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    if (err != ERR_OK) {
        printf("Connection error: %d\n", err);
        return http_client_close(state);
    }
    
    printf("Connected. Sending HTTP headers...\n");
    state->status = REQ_STATE_SENDING_HEADERS;
    
    char header[256];
    snprintf(header, sizeof(header),
             "POST %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %u\r\n"
             "Connection: close\r\n\r\n",
             API_URL, API_HOST, (unsigned int)state->content_length_total);
    
    cyw43_arch_lwip_begin();
    err_t write_err = tcp_write(tpcb, header, strlen(header), TCP_WRITE_FLAG_COPY);
    if (write_err == ERR_OK) {
        tcp_output(tpcb);
    } else {
        printf("Error writing header: %d\n", write_err);
        cyw43_arch_lwip_end();
        return http_client_close(state);
    }
    cyw43_arch_lwip_end();
    
    return ERR_OK;
}

static err_t http_client_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    
    if (state->status == REQ_STATE_SENDING_HEADERS) {
        printf("Headers sent. Ready for data chunks.\n");
        state->status = REQ_STATE_SENDING_BODY;
    }
    
    state->ready_for_chunk = true;

    return ERR_OK;
}

static err_t http_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    if (!p) {
        printf("Connection closed by server.\n");
        return http_client_close(state);
    }
    
    if (p->tot_len > 0) {
        printf("--- API Response ---\n");
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            printf("%.*s", q->len, (char *)q->payload);
        }
        printf("\n--------------------\n");
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
    
    return ERR_OK;
}

static void http_client_err_cb(void *arg, err_t err) {
    printf("TCP error callback: %d\n", err);
    http_client_close(arg);
}

static err_t http_client_close(HTTP_REQUEST_STATE_T *state) {
    if (!state) return ERR_OK;
    
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL) {
        cyw43_arch_lwip_begin();
        tcp_arg(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        tcp_sent(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            printf("TCP close failed: %d. Aborting.\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
        cyw43_arch_lwip_end();
    }
    
    if (state->status != REQ_STATE_ERROR) {
        state->status = REQ_STATE_DONE;
    } else {
        printf("Request finished with error.\n");
    }
    
    return err;
}
