/*
 * Shell commands for AT interface testing
 */

#include <zephyr/shell/shell.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/dns_resolve.h>
#include <string.h>
#include <stdio.h>

/* Simple Base64 decode table */
static const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* Simple Base64 decode function */
static int base64_decode(const char *input, char *output, size_t output_size)
{
    int len = strlen(input);
    int i, j = 0;
    int pad = 0;
    
    if (len % 4 != 0) return -1;
    
    for (i = 0; i < len && j < output_size - 1; i += 4) {
        int a = strchr(base64_chars, input[i]) - base64_chars;
        int b = strchr(base64_chars, input[i+1]) - base64_chars;
        int c = (input[i+2] == '=') ? 0 : strchr(base64_chars, input[i+2]) - base64_chars;
        int d = (input[i+3] == '=') ? 0 : strchr(base64_chars, input[i+3]) - base64_chars;
        
        if (input[i+2] == '=') pad++;
        if (input[i+3] == '=') pad++;
        
        int combined = (a << 18) | (b << 12) | (c << 6) | d;
        
        output[j++] = (combined >> 16) & 0xFF;
        if (pad < 2) output[j++] = (combined >> 8) & 0xFF;
        if (pad < 1) output[j++] = combined & 0xFF;
    }
    
    output[j] = '\0';
    return j;
}

/* AT+WIFICONN shell command */
static int cmd_at_wificonn(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_error(sh, "Usage: AT+WIFICONN <ssid> <password>");
        return -EINVAL;
    }
    
    const char *ssid = argv[1];
    const char *password = argv[2];
    
    struct net_if *iface = net_if_get_first_wifi();
    if (!iface) {
        shell_error(sh, "No WiFi interface found");
        return -ENODEV;
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
        shell_print(sh, "+WIFICONN:CONNECTING");
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERROR: WiFi connect failed (%d)", ret);
    }
    
    return ret;
}

/* AT+WIFISTATUS shell command */
static int cmd_at_wifistatus(const struct shell *sh, size_t argc, char **argv)
{
    struct net_if *iface = net_if_get_first_wifi();
    if (!iface) {
        shell_error(sh, "No WiFi interface found");
        return -ENODEV;
    }
    
    struct wifi_iface_status status = {0};
    int ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status));
    
    if (ret == 0) {
        if (status.state >= WIFI_STATE_ASSOCIATED) {
            shell_print(sh, "+WIFISTATUS:CONNECTED,%s,%d", status.ssid, status.rssi);
        } else {
            shell_print(sh, "+WIFISTATUS:DISCONNECTED");
        }
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERROR: Status request failed (%d)", ret);
    }
    
    return ret;
}

/* Simple test command */
static int cmd_test_args(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Test command - argc=%d", argc);
    for (int i = 0; i < argc; i++) {
        shell_print(sh, "argv[%d]='%s'", i, argv[i]);
    }
    return 0;
}

/* AT+HTTPGET shell command with Base64 decoding */
static int cmd_at_httpget(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: AT+HTTPGET <base64_encoded_url>");
        shell_print(sh, "Example: echo -n 'http://google.com' | base64");
        return -EINVAL;
    }
    
    /* Decode Base64 URL */
    static char decoded_url[512];
    int decoded_len = base64_decode(argv[1], decoded_url, sizeof(decoded_url));
    
    if (decoded_len < 0) {
        shell_error(sh, "ERROR: Invalid Base64 encoding");
        return -EINVAL;
    }
    
    const char *url = decoded_url;
    shell_print(sh, "HTTP GET: %s", url);
    
    /* Simple HTTP GET implementation */
    int sock;
    struct sockaddr_in addr;
    char request[512];
    char response[1024];
    
    /* Parse URL to get host, port and path */
    char host[128] = {0};
    char path[256] = "/";
    int port = 80;
    
    /* Simple URL parsing - assumes http://host:port/path format */
    if (strncmp(url, "http://", 7) == 0) {
        const char *host_start = url + 7;
        const char *path_start = strchr(host_start, '/');
        const char *port_start = strchr(host_start, ':');
        
        if (path_start) {
            strncpy(path, path_start, sizeof(path) - 1);
        }
        
        /* Extract host and port */
        if (port_start && (!path_start || port_start < path_start)) {
            /* Host with port */
            strncpy(host, host_start, port_start - host_start);
            port = atoi(port_start + 1);
        } else if (path_start) {
            /* Host without port */
            strncpy(host, host_start, path_start - host_start);
        } else {
            /* Just host */
            strncpy(host, host_start, sizeof(host) - 1);
        }
    } else {
        shell_error(sh, "ERROR: Invalid URL format");
        return -EINVAL;
    }
    
    shell_print(sh, "Host: %s, Port: %d, Path: %s", host, port, path);
    
    /* Create socket */
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        shell_error(sh, "ERROR: Socket creation failed");
        return -errno;
    }
    
    /* Setup address */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    
    /* Try to parse as IP address first */
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        /* Not an IP address, try DNS resolution */
        shell_print(sh, "Resolving hostname: %s", host);
        
        struct zsock_addrinfo hints = {0};
        struct zsock_addrinfo *result;
        
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        
        int dns_ret = zsock_getaddrinfo(host, NULL, &hints, &result);
        if (dns_ret != 0 || !result) {
            close(sock);
            shell_error(sh, "+HTTPGET:DNS_FAILED");
            shell_error(sh, "ERROR: DNS resolution failed for %s", host);
            return -EINVAL;
        }
        
        /* Copy the resolved IP address */
        struct sockaddr_in *resolved_addr = (struct sockaddr_in *)result->ai_addr;
        addr.sin_addr = resolved_addr->sin_addr;
        
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, ip_str, sizeof(ip_str));
        shell_print(sh, "Resolved to IP: %s", ip_str);
        
        zsock_freeaddrinfo(result);
    }
    
    /* Connect */
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        shell_error(sh, "+HTTPGET:CONNECT_FAILED");
        shell_error(sh, "ERROR: Connection failed");
        return -errno;
    }
    
    /* Send HTTP request */
    snprintf(request, sizeof(request),
            "GET %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Connection: close\r\n"
            "\r\n", path, host);
    
    if (send(sock, request, strlen(request), 0) < 0) {
        close(sock);
        shell_error(sh, "ERROR: Send failed");
        return -errno;
    }
    
    /* Receive response with multiple recv calls */
    int total_bytes = 0;
    int bytes;
    
    /* Receive data in chunks */
    while (total_bytes < sizeof(response) - 1) {
        bytes = recv(sock, response + total_bytes, sizeof(response) - 1 - total_bytes, 0);
        if (bytes <= 0) {
            break; /* Connection closed or error */
        }
        total_bytes += bytes;
        
        /* Small delay to allow more data */
        k_msleep(10);
    }
    
    close(sock);
    
    if (total_bytes > 0) {
        response[total_bytes] = '\0';
        
        shell_print(sh, "Total received: %d bytes", total_bytes);
        
        /* Find the end of HTTP headers (\r\n\r\n) */
        char *body_start = strstr(response, "\r\n\r\n");
        if (body_start) {
            body_start += 4; /* Skip past \r\n\r\n */
            shell_print(sh, "+HTTPGET:200,%d", strlen(body_start));
            shell_print(sh, "Response Body: '%s'", body_start);
        } else {
            shell_print(sh, "+HTTPGET:200,%d", total_bytes);
            shell_print(sh, "Full Response: %s", response);
        }
        shell_print(sh, "OK");
    } else {
        shell_error(sh, "ERROR: No data received");
        return -errno;
    }
    
    return 0;
}

/* Simple network info command */
static int cmd_net_info(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Network Information:");
    
    struct net_if *iface = net_if_get_first_wifi();
    if (iface) {
        char buf[NET_IPV4_ADDR_LEN];
        struct in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
        if (addr) {
            shell_print(sh, "IP: %s", net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
        } else {
            shell_print(sh, "No IP address assigned");
        }
        
        shell_print(sh, "Interface up: %s", net_if_is_up(iface) ? "Yes" : "No");
    }
    
    return 0;
}

/* Register individual shell commands */
SHELL_CMD_REGISTER(test_args, NULL, "Test arguments", cmd_test_args);
SHELL_CMD_REGISTER(net_info, NULL, "Show network info", cmd_net_info);
SHELL_CMD_REGISTER(at_wificonn, NULL, "AT+WIFICONN <ssid> <password>", cmd_at_wificonn);
SHELL_CMD_REGISTER(at_wifistatus, NULL, "AT+WIFISTATUS", cmd_at_wifistatus);
SHELL_CMD_REGISTER(at_httpget, NULL, "AT+HTTPGET <base64_url>", cmd_at_httpget);