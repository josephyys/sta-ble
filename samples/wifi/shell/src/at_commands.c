/*
 * AT Command Interface for NUC980 Communication
 * Provides network services via UART AT commands
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>
#include <string.h>
#include <stdio.h>

#define AT_CMD_BUFFER_SIZE 256
#define AT_RESPONSE_BUFFER_SIZE 1024

static char at_cmd_buffer[AT_CMD_BUFFER_SIZE];
static char at_response_buffer[AT_RESPONSE_BUFFER_SIZE];
static int at_cmd_pos = 0;
static const struct device *at_uart;

/* AT Command responses */
#define AT_OK "OK\r\n"
#define AT_ERROR "ERROR\r\n"
#define AT_READY "+READY\r\n"

/* Function declarations */
static void at_send_response(const char *response);
static void at_process_command(const char *cmd);
static void at_cmd_wifi_connect(const char *ssid, const char *password);
static void at_cmd_wifi_status(void);
static void at_cmd_http_get(const char *url);

/* Send AT response to NUC980 */
static void at_send_response(const char *response)
{
    if (!at_uart) return;
    
    for (int i = 0; response[i]; i++) {
        uart_poll_out(at_uart, response[i]);
    }
}

/* WiFi Connect Command: AT+WIFICONN=ssid,password */
static void at_cmd_wifi_connect(const char *ssid, const char *password)
{
    struct net_if *iface = net_if_get_first_wifi();
    if (!iface) {
        at_send_response(AT_ERROR);
        return;
    }
    
    struct wifi_connect_req_params params = {
        .ssid = ssid,
        .ssid_length = strlen(ssid),
        .psk = password,
        .psk_length = strlen(password),
        .security = WIFI_SECURITY_TYPE_PSK,
        .channel = WIFI_CHANNEL_ANY,
    };
    
    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
    if (ret == 0) {
        at_send_response("+WIFICONN:CONNECTING\r\n");
        at_send_response(AT_OK);
    } else {
        at_send_response(AT_ERROR);
    }
}

/* WiFi Status Command: AT+WIFISTATUS */
static void at_cmd_wifi_status(void)
{
    struct net_if *iface = net_if_get_first_wifi();
    if (!iface) {
        at_send_response(AT_ERROR);
        return;
    }
    
    struct wifi_iface_status status = {0};
    int ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status));
    
    if (ret == 0) {
        if (status.state >= WIFI_STATE_ASSOCIATED) {
            snprintf(at_response_buffer, sizeof(at_response_buffer),
                    "+WIFISTATUS:CONNECTED,%s,%d\r\n", status.ssid, status.rssi);
        } else {
            snprintf(at_response_buffer, sizeof(at_response_buffer),
                    "+WIFISTATUS:DISCONNECTED\r\n");
        }
        at_send_response(at_response_buffer);
        at_send_response(AT_OK);
    } else {
        at_send_response(AT_ERROR);
    }
}

/* HTTP GET Command: AT+HTTPGET=url */
static void at_cmd_http_get(const char *url)
{
    /* Simple HTTP GET implementation */
    int sock;
    struct sockaddr_in addr;
    char request[512];
    char response[1024];
    
    /* Parse URL to get host and path */
    char host[128] = {0};
    char path[256] = "/";
    
    /* Simple URL parsing - assumes http://host/path format */
    if (strncmp(url, "http://", 7) == 0) {
        const char *host_start = url + 7;
        const char *path_start = strchr(host_start, '/');
        
        if (path_start) {
            strncpy(host, host_start, path_start - host_start);
            strncpy(path, path_start, sizeof(path) - 1);
        } else {
            strncpy(host, host_start, sizeof(host) - 1);
        }
    } else {
        at_send_response(AT_ERROR);
        return;
    }
    
    /* Create socket */
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        at_send_response(AT_ERROR);
        return;
    }
    
    /* Resolve hostname (simplified - assumes IP address) */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(80);
    
    /* For demo, use a simple IP resolution */
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        /* If not IP, try to resolve - simplified for demo */
        close(sock);
        at_send_response("+HTTPGET:DNS_FAILED\r\n");
        at_send_response(AT_ERROR);
        return;
    }
    
    /* Connect */
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        at_send_response("+HTTPGET:CONNECT_FAILED\r\n");
        at_send_response(AT_ERROR);
        return;
    }
    
    /* Send HTTP request */
    snprintf(request, sizeof(request),
            "GET %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Connection: close\r\n"
            "\r\n", path, host);
    
    if (send(sock, request, strlen(request), 0) < 0) {
        close(sock);
        at_send_response(AT_ERROR);
        return;
    }
    
    /* Receive response */
    int bytes = recv(sock, response, sizeof(response) - 1, 0);
    close(sock);
    
    if (bytes > 0) {
        response[bytes] = '\0';
        
        /* Send response header */
        at_send_response("+HTTPGET:200,");
        
        /* Send data length */
        snprintf(at_response_buffer, sizeof(at_response_buffer), "%d\r\n", bytes);
        at_send_response(at_response_buffer);
        
        /* Send actual data (truncated for demo) */
        at_send_response(response);
        at_send_response("\r\n");
        at_send_response(AT_OK);
    } else {
        at_send_response(AT_ERROR);
    }
}

/* Process AT command */
static void at_process_command(const char *cmd)
{
    if (strncmp(cmd, "AT", 2) == 0) {
        if (strlen(cmd) == 2) {
            /* Basic AT command */
            at_send_response(AT_OK);
        }
        else if (strncmp(cmd, "AT+WIFICONN=", 12) == 0) {
            /* Parse SSID and password */
            char ssid[64], password[64];
            if (sscanf(cmd + 12, "%63[^,],%63s", ssid, password) == 2) {
                at_cmd_wifi_connect(ssid, password);
            } else {
                at_send_response(AT_ERROR);
            }
        }
        else if (strcmp(cmd, "AT+WIFISTATUS") == 0) {
            at_cmd_wifi_status();
        }
        else if (strncmp(cmd, "AT+HTTPGET=", 11) == 0) {
            at_cmd_http_get(cmd + 11);
        }
        else if (strcmp(cmd, "AT+HELP") == 0) {
            at_send_response("+HELP:Available commands\r\n");
            at_send_response("AT - Test command\r\n");
            at_send_response("AT+WIFICONN=ssid,password - Connect to WiFi\r\n");
            at_send_response("AT+WIFISTATUS - Get WiFi status\r\n");
            at_send_response("AT+HTTPGET=url - HTTP GET request\r\n");
            at_send_response(AT_OK);
        }
        else {
            at_send_response(AT_ERROR);
        }
    } else {
        at_send_response(AT_ERROR);
    }
}

/* UART interrupt handler for AT commands */
static void at_uart_callback(const struct device *dev, void *user_data)
{
    uint8_t c;
    
    if (!uart_irq_update(dev)) {
        return;
    }
    
    if (!uart_irq_rx_ready(dev)) {
        return;
    }
    
    while (uart_fifo_read(dev, &c, 1) == 1) {
        if (c == '\r' || c == '\n') {
            if (at_cmd_pos > 0) {
                at_cmd_buffer[at_cmd_pos] = '\0';
                at_process_command(at_cmd_buffer);
                at_cmd_pos = 0;
            }
        } else if (at_cmd_pos < (AT_CMD_BUFFER_SIZE - 1)) {
            at_cmd_buffer[at_cmd_pos++] = c;
        }
    }
}

/* Initialize AT command interface */
int at_commands_init(void)
{
    /* Get UART device for AT commands */
    at_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(at_uart)) {
        printk("AT UART device not ready\n");
        return -1;
    }
    
    /* Configure UART callback */
    uart_irq_callback_set(at_uart, at_uart_callback);
    uart_irq_rx_enable(at_uart);
    
    /* Send ready message */
    k_sleep(K_MSEC(100));
    at_send_response(AT_READY);
    
    printk("AT Command interface initialized on UART1\n");
    return 0;
}