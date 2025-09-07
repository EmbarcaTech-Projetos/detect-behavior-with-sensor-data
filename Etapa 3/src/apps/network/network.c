#include <string.h>
#include <stdlib.h>
#include "network.h"
#include "lwip/dns.h"

// --- Configuration ---
// REPLACE WITH YOUR NETWORK DETAILS
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// REPLACE WITH YOUR API ENDPOINT DETAILS
#define API_HOST "your-api-endpoint.com"
#define API_URL "/api/data"
#define API_PORT 80

// --- Function Prototypes for Internal Callbacks ---
static err_t http_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t http_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void http_client_err_cb(void *arg, err_t err);
static err_t http_client_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_client_close(HTTP_REQUEST_STATE_T *state);

// Initializes the Wi-Fi connection
bool network_init(void) {
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed.\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi network: %s\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to Wi-Fi.\n");
        return false;
    }
    
    printf("Successfully connected to Wi-Fi.\n");
    return true;
}

// Internal callback for when DNS is resolved
static void http_dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    if (ipaddr) {
        state->remote_addr = *ipaddr;
        printf("DNS resolved for %s: %s\n", name, ip4addr_ntoa(ipaddr));

        // Create a new TCP connection
        state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(ipaddr));
        if (!state->tcp_pcb) {
            printf("Failed to create TCP PCB.\n");
            free(state->request_body);
            free(state);
            return;
        }

        tcp_arg(state->tcp_pcb, state);
        tcp_sent(state->tcp_pcb, http_client_sent_cb);
        tcp_recv(state->tcp_pcb, http_client_recv_cb);
        tcp_err(state->tcp_pcb, http_client_err_cb);

        // Attempt to connect
        cyw43_arch_lwip_begin();
        err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, API_PORT, http_client_connected_cb);
        cyw43_arch_lwip_end();
        if (err != ERR_OK) {
            printf("TCP connect failed: %d\n", err);
            http_client_close(state);
        }

    } else {
        printf("DNS resolution failed for %s\n", name);
        free(state->request_body);
        free(state);
    }
}

// Internal function to start the DNS lookup
static void start_dns_lookup(HTTP_REQUEST_STATE_T *state) {
    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(API_HOST, &state->remote_addr, (dns_found_callback) http_dns_found_cb, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        // IP address is already cached, proceed to connect
        http_dns_found_cb(API_HOST, &state->remote_addr, state);
    } else if (err != ERR_INPROGRESS) {
        printf("DNS request failed immediately: %d\n", err);
        free(state->request_body);
        free(state);
    }
}

// Main function to initiate the POST request
HTTP_REQUEST_STATE_T* network_send_post_request(const char *json_payload) {
    HTTP_REQUEST_STATE_T *state = calloc(1, sizeof(HTTP_REQUEST_STATE_T));
    if (!state) {
        printf("Failed to allocate memory for request state.\n");
        return NULL;
    }

    // Copy the payload
    state->request_body = strdup(json_payload);
    if (!state->request_body) {
        printf("Failed to allocate memory for request body.\n");
        free(state);
        return NULL;
    }

    state->complete = false;
    state->sent_len = 0;

    start_dns_lookup(state);
    
    return state; // The caller can check state->complete to see when it's done
}


// --- LWIP Callback Implementations ---

// Called when TCP connection is established
static err_t http_client_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection error: %d\n", err);
        return http_client_close(arg);
    }

    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    printf("Connected to server. Sending POST request...\n");
    
    // Form the HTTP request header
    char header[256];
    int content_len = strlen(state->request_body);
    snprintf(header, sizeof(header),
             "POST %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n",
             API_URL, API_HOST, content_len);
    
    // Write header and body
    cyw43_arch_lwip_begin();
    err_t write_err = tcp_write(tpcb, header, strlen(header), TCP_WRITE_FLAG_COPY);
    if (write_err == ERR_OK) {
        write_err = tcp_write(tpcb, state->request_body, content_len, TCP_WRITE_FLAG_COPY);
    }
    
    if (write_err == ERR_OK) {
        tcp_output(tpcb);
    } else {
        printf("Error writing to TCP stream: %d\n", write_err);
        cyw43_arch_lwip_end();
        return http_client_close(state);
    }
    cyw43_arch_lwip_end();

    return ERR_OK;
}

// Called when data is received from the server
static err_t http_client_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    if (!p) { // Null pbuf indicates the connection was closed by the remote host
        printf("Connection closed by server.\n");
        return http_client_close(state);
    }
    
    if (p->tot_len > 0) {
        printf("--- API Response ---\n");
        // Print the received payload
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            printf("%.*s", q->len, (char *)q->payload);
        }
        printf("\n--------------------\n");
        tcp_recved(tpcb, p->tot_len); // Acknowledge that we've received the data
    }
    pbuf_free(p);
    
    return ERR_OK;
}

// Called when an error occurs
static void http_client_err_cb(void *arg, err_t err) {
    printf("TCP error callback: %d\n", err);
    http_client_close(arg);
}

// Called when data has been successfully sent and acknowledged
static err_t http_client_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    HTTP_REQUEST_STATE_T *state = (HTTP_REQUEST_STATE_T *)arg;
    state->sent_len += len;
    // You could add logic here if you were sending a very large file
    return ERR_OK;
}

// Cleans up the state and closes the TCP connection
static err_t http_client_close(HTTP_REQUEST_STATE_T *state) {
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL) {
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
    }
    
    if (state->request_body) {
        free(state->request_body);
        state->request_body = NULL;
    }

    state->complete = true; // Mark the request as complete
    printf("Request finished.\n\n");
    
    return err;
}