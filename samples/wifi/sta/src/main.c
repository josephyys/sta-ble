/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief WiFi station sample
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sta, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/init.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/uart.h>
#include <string.h>


#ifdef CONFIG_WIFI_READY_LIB
#include <net/wifi_ready.h>
#endif /* CONFIG_WIFI_READY_LIB */

#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
#include <zephyr/drivers/wifi/nrf_wifi/bus/qspi_if.h>
#endif

#include "net_private.h"

#define WIFI_SHELL_MODULE "wifi"

#define WIFI_SHELL_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT |		\
				NET_EVENT_WIFI_DISCONNECT_RESULT)

#define MAX_SSID_LEN        32
#define STATUS_POLLING_MS   300

/* 1000 msec = 1 sec */
#define LED_SLEEP_TIME_MS   100

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static struct net_mgmt_event_callback wifi_shell_mgmt_cb;
static struct net_mgmt_event_callback net_shell_mgmt_cb;

#ifdef CONFIG_WIFI_READY_LIB
static K_SEM_DEFINE(wifi_ready_state_changed_sem, 0, 1);
static bool wifi_ready_status;
#endif /* CONFIG_WIFI_READY_LIB */


// #define CMD_UART_LABEL "UART_0"

#define MAX_CMD_LEN 128

#define UART_RX_BUF_SIZE 128
static char rx_buf[UART_RX_BUF_SIZE];
static int rx_pos = 0;
static volatile bool cmd_ready = false;
static char cmd_copy[UART_RX_BUF_SIZE];

// At the beginning of your file, with other global variables
static K_MUTEX_DEFINE(cmd_mutex);

// static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
static const struct device *cmd_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static void test_wifi_connect(const char *ssid, const char *password);

static void cmd_wifi_info(void)
{
    struct net_if *iface = net_if_get_first_wifi();
    
    printk("WiFi Status:\n");
    printk("------------\n");
    
    if (!iface) {
        printk("No WiFi interface found\n");
        return;
    }
    
    // Print WiFi interface status
    printk("Interface: %d\n", net_if_get_by_iface(iface));
    printk("Status: %s\n", net_if_is_up(iface) ? "UP" : "DOWN");
    
    // Get IP address if available - use NET_ADDR_PREFERRED enum
    struct in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
    char buf[NET_IPV4_ADDR_LEN];
    
    if (addr) {
        printk("IP Address: %s\n", 
               net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
    } else {
        printk("IP Address: Not assigned\n");
    }
    
    // Print connection state from context
    // printk("Connected: %s\n", context.connected ? "Yes" : "No");
    // if (context.connected) {
    //     printk("SSID: %s\n", context.ssid);
    // }
}

static void cmd_slip_setup(void)
{
    struct net_if *iface = net_if_get_by_index(1);
    
    if (!iface) {
        printk("SLIP interface not found\n");
        return;
    }
    
    // First bring it down to reset state
    net_if_down(iface);
    k_sleep(K_MSEC(500));
    
    // Then try to bring it up
    int ret = net_if_up(iface);
    printk("SLIP interface activation result: %d\n", ret);
    
    // Check status
    k_sleep(K_MSEC(500));
    printk("SLIP interface status: %s\n", 
           net_if_is_up(iface) ? "UP" : "DOWN");
}

static void cmd_slip_info(void)
{
    struct net_if *iface;
    int i = 0;
    
    printk("Network Interfaces:\n");
    printk("------------------\n");
    
    for (i = 0; i < 8; i++) {  // Check first 8 interfaces
        iface = net_if_get_by_index(i);
        if (!iface) {
            continue;
        }
        
        // Print basic interface information
        printk("Interface %d:\n", i);
        printk("  Status: %s\n", net_if_is_up(iface) ? "UP" : "DOWN");
        printk("  MTU: %d\n", net_if_get_mtu(iface));
        
        // Try to identify interface type
        if (net_if_get_by_iface(iface) == 1) {
            printk("  Type: Likely SLIP (interface #1)\n");
        } else if (net_if_get_by_iface(iface) == 0) {
            printk("  Type: Likely WiFi (interface #0)\n");
        }
    }
}

static void cmd_net_scan(void)
{
    printk("Scanning all network interfaces:\n");
    printk("-------------------------------\n");
    
    for (int i = 0; i < 8; i++) {
        struct net_if *iface = net_if_get_by_index(i);
        if (!iface) {
            continue;
        }
        
        // Print basic info
        printk("Interface %d:\n", i);
        printk("  Status: %s\n", net_if_is_up(iface) ? "UP" : "DOWN");
        printk("  MTU: %d\n", net_if_get_mtu(iface));
        
        // Try to get IP address info
        char buf[NET_IPV4_ADDR_LEN];
        struct in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
        if (addr) {
            printk("  IPv4: %s\n", net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
        }
        
        // Try to activate if down
        if (!net_if_is_up(iface)) {
            int ret = net_if_up(iface);
            printk("  Activation attempt: %d\n", ret);
            k_sleep(K_MSEC(100));
            printk("  Status after attempt: %s\n", 
                   net_if_is_up(iface) ? "UP" : "DOWN");
        }
        
        printk("\n");
    }
    
    // Also remove the unused function warning
    #ifdef CONFIG_SLIP
    printk("SLIP support is enabled in config\n");
    #else
    printk("SLIP support is NOT enabled in config\n");
    #endif
}

static void cmd_ip_check(void) {
    printk("Network Interface IP Check:\n");
    printk("--------------------------\n");
    
    for (int i = 0; i < 8; i++) {
        struct net_if *iface = net_if_get_by_index(i);
        if (!iface) continue;
        
        printk("Interface %d: %s\n", i, 
               net_if_is_up(iface) ? "UP" : "DOWN");
        
        // Try all possible address states
        char buf[NET_IPV4_ADDR_LEN];
        struct in_addr *addr;
        
        // Check PREFERRED addresses (primary addresses)
        addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
        if (addr) {
            printk("  IPv4 (Preferred): %s\n", 
                   net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
        }
        
        // Check TENTATIVE addresses (being verified)
        addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_TENTATIVE);
        if (addr) {
            printk("  IPv4 (Tentative): %s\n", 
                   net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
        }
        
        // Check ANY addresses (any state)
        addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_ANY);
        if (addr) {
            printk("  IPv4 (Any): %s\n", 
                   net_addr_ntop(AF_INET, addr, buf, sizeof(buf)));
        }
        
        // Skip direct config access - API changed in v3.0.2
        printk("  (Direct config access not available in v3.0.2)\n");
    }
}

static void handle_cmd(const char *cmd) {
    printk("Processing command: %s\n", cmd);
    
    if (strncmp(cmd, "wifi_connect", 12) == 0) {
        char ssid[64], pw[64];
        if (sscanf(cmd, "wifi_connect %63s %63s", ssid, pw) == 2) {
            printk("Connecting to SSID: %s, PW: %s\n", ssid, pw);
            
            struct wifi_connect_req_params cnx_params = {
                .ssid = ssid,
                .ssid_length = strlen(ssid),
                .psk = pw,
                .psk_length = strlen(pw),
                .security = WIFI_SECURITY_TYPE_PSK,
                .channel = WIFI_CHANNEL_ANY,
            };
            
            struct net_if *iface = net_if_get_first_wifi();
            if (!iface) {
                printk("No WiFi interface found\n");
                return;
            }
            
            printk("Sending connection request...\n");
            int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, 
                             &cnx_params, sizeof(cnx_params));
            if (ret) {
                printk("Connection request failed: %d\n", ret);
            } else {
                printk("Connection request sent successfully\n");
            }
        } else {
            printk("Invalid format. Use: wifi_connect ssid=YOURSSID pw=YOURPASSWORD\n");
        }
    } else if (strncmp(cmd, "slip_info", 14) == 0) {
		cmd_slip_info();
    } else if (strncmp(cmd, "slip_setup", 14) == 0) {
		cmd_slip_setup();
	} else if (strncmp(cmd, "wifi_info", 9) == 0) {
		cmd_wifi_info();
	} else if (strncmp(cmd, "net_scan", 8) == 0) {
		cmd_net_scan();
	} else if (strncmp(cmd, "slip2_setup", 11) == 0) {
		struct net_if *iface = net_if_get_by_index(2);
		if (iface) {
			printk("Activating Interface 2 as SLIP\n");
			int ret = net_if_up(iface);
			printk("Result: %d\n", ret);
		}
	} else if (strncmp(cmd, "ip_check", 8) == 0) {
		cmd_ip_check();
	}
	else {
		printk("Invalid cmd. %s\n", cmd);

	} 
}


void uart_rx_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
		if (cmd_ready) {
            // Copy the command in a thread-safe way
            k_mutex_lock(&cmd_mutex, K_FOREVER);
            strcpy(cmd_copy, rx_buf);
            cmd_ready = false;
            k_mutex_unlock(&cmd_mutex);
            
            // Now process the command in thread context
			handle_cmd(cmd_copy);
        }
        
        // Sleep to prevent CPU hogging
        k_msleep(10);
    }
}

static struct {
	const struct shell *sh;
	union {
		struct {
			uint8_t connected	: 1;
			uint8_t connect_result	: 1;
			uint8_t disconnect_requested	: 1;
			uint8_t _unused		: 5;
		};
		uint8_t all;
	};
} context;

void toggle_led(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		LOG_ERR("LED device is not ready");
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Error %d: failed to configure LED pin", ret);
		return;
	}

	while (1) {
		if (context.connected) {
			gpio_pin_toggle_dt(&led);
			k_msleep(LED_SLEEP_TIME_MS);
		} else {
			gpio_pin_set_dt(&led, 0);
			k_msleep(LED_SLEEP_TIME_MS);
		}
	}
}

K_THREAD_DEFINE(led_thread_id, 1024, toggle_led, NULL, NULL, NULL,
		7, 0, 0);

static int cmd_wifi_status(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_iface_status status = { 0 };

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				sizeof(struct wifi_iface_status))) {
		LOG_INF("Status request failed");

		return -ENOEXEC;
	}

	LOG_INF("==================");
	LOG_INF("State: %s", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED) {
		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

		LOG_INF("Interface Mode: %s",
		       wifi_mode_txt(status.iface_mode));
		LOG_INF("Link Mode: %s",
		       wifi_link_mode_txt(status.link_mode));
		LOG_INF("SSID: %.32s", status.ssid);
		LOG_INF("BSSID: %s",
		       net_sprint_ll_addr_buf(
				status.bssid, WIFI_MAC_ADDR_LEN,
				mac_string_buf, sizeof(mac_string_buf)));
		LOG_INF("Band: %s", wifi_band_txt(status.band));
		LOG_INF("Channel: %d", status.channel);
		LOG_INF("Security: %s", wifi_security_txt(status.security));
		LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp));
		LOG_INF("RSSI: %d", status.rssi);
	}
	return 0;
}

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (context.connected) {
		return;
	}

	if (status->status) {
		LOG_ERR("Connection failed (%d)", status->status);
	} else {
		LOG_INF("Connected");
		context.connected = true;
	}

	context.connect_result = true;
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status =
		(const struct wifi_status *) cb->info;

	if (!context.connected) {
		return;
	}

	if (context.disconnect_requested) {
		LOG_INF("Disconnection request %s (%d)",
			 status->status ? "failed" : "done",
					status->status);
		context.disconnect_requested = false;
	} else {
		LOG_INF("Received Disconnected");
		context.connected = false;
	}

	cmd_wifi_status();
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				     uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		handle_wifi_disconnect_result(cb);
		break;
	default:
		break;
	}
}

static void print_dhcp_ip(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	LOG_INF("DHCP IP address: %s", dhcp_info);
}
static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_IPV4_DHCP_BOUND:
		print_dhcp_ip(cb);
		break;
	default:
		break;
	}
}

static int wifi_connect(void)
{
	struct net_if *iface = net_if_get_first_wifi();

	context.connected = false;
	context.connect_result = false;

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0)) {
		LOG_ERR("Connection request failed");

		return -ENOEXEC;
	}

	LOG_INF("Connection requested");	

	return 0;
}

int bytes_from_str(const char *str, uint8_t *bytes, size_t bytes_len)
{
	size_t i;
	char byte_str[3];

	if (strlen(str) != bytes_len * 2) {
		LOG_ERR("Invalid string length: %zu (expected: %d)\n",
			strlen(str), bytes_len * 2);
		return -EINVAL;
	}

	for (i = 0; i < bytes_len; i++) {
		memcpy(byte_str, str + i * 2, 2);
		byte_str[2] = '\0';
		bytes[i] = strtol(byte_str, NULL, 16);
	}

	return 0;
}

int start_app(void)
{
#if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
	if (strlen(CONFIG_NRF70_QSPI_ENCRYPTION_KEY)) {
		int ret;
		char key[QSPI_KEY_LEN_BYTES];

		ret = bytes_from_str(CONFIG_NRF70_QSPI_ENCRYPTION_KEY, key, sizeof(key));
		if (ret) {
			LOG_ERR("Failed to parse encryption key: %d\n", ret);
			return 0;
		}

		LOG_DBG("QSPI Encryption key: ");
		for (int i = 0; i < QSPI_KEY_LEN_BYTES; i++) {
			LOG_DBG("%02x", key[i]);
		}
		LOG_DBG("\n");

		ret = qspi_enable_encryption(key);
		if (ret) {
			LOG_ERR("Failed to enable encryption: %d\n", ret);
			return 0;
		}
		LOG_INF("QSPI Encryption enabled");
	} else {
		LOG_INF("QSPI Encryption disabled");
	}
#endif /* CONFIG_BOARD_NRF700XDK_NRF5340 */

	LOG_INF("Static IP address (overridable): %s/%s -> %s",
		CONFIG_NET_CONFIG_MY_IPV4_ADDR,
		CONFIG_NET_CONFIG_MY_IPV4_NETMASK,
		CONFIG_NET_CONFIG_MY_IPV4_GW);

	while (1) {
#ifdef CONFIG_WIFI_READY_LIB
		int ret;

		LOG_INF("Waiting for Wi-Fi to be ready");
		ret = k_sem_take(&wifi_ready_state_changed_sem, K_FOREVER);
		if (ret) {
			LOG_ERR("Failed to take semaphore: %d", ret);
			return ret;
		}

check_wifi_ready:
		if (!wifi_ready_status) {
			LOG_INF("Wi-Fi is not ready");
			/* Perform any cleanup and stop using Wi-Fi and wait for
			 * Wi-Fi to be ready
			 */
			continue;
		}
#endif /* CONFIG_WIFI_READY_LIB */
		wifi_connect();

		while (!context.connect_result) {
			cmd_wifi_status();
			k_sleep(K_MSEC(STATUS_POLLING_MS));
		}

		if (context.connected) {
			cmd_wifi_status();
#ifdef CONFIG_WIFI_READY_LIB
			ret = k_sem_take(&wifi_ready_state_changed_sem, K_FOREVER);
			if (ret) {
				LOG_ERR("Failed to take semaphore: %d", ret);
				return ret;
			}
			goto check_wifi_ready;
#else
			k_sleep(K_FOREVER);
#endif /* CONFIG_WIFI_READY_LIB */
		}
	}

	return 0;
}

#ifdef CONFIG_WIFI_READY_LIB
void start_wifi_thread(void);
#define THREAD_PRIORITY K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES - 1)
K_THREAD_DEFINE(start_wifi_thread_id, CONFIG_STA_SAMPLE_START_WIFI_THREAD_STACK_SIZE,
		start_wifi_thread, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

void start_wifi_thread(void)
{
	start_app();
}

void wifi_ready_cb(bool wifi_ready)
{
	LOG_DBG("Is Wi-Fi ready?: %s", wifi_ready ? "yes" : "no");
	wifi_ready_status = wifi_ready;
	k_sem_give(&wifi_ready_state_changed_sem);
}
#endif /* CONFIG_WIFI_READY_LIB */

void net_mgmt_callback_init(void)
{
	memset(&context, 0, sizeof(context));

	net_mgmt_init_event_callback(&wifi_shell_mgmt_cb,
				     wifi_mgmt_event_handler,
				     WIFI_SHELL_MGMT_EVENTS);

	net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

	net_mgmt_init_event_callback(&net_shell_mgmt_cb,
				     net_mgmt_event_handler,
				     NET_EVENT_IPV4_DHCP_BOUND);

	net_mgmt_add_event_callback(&net_shell_mgmt_cb);

	LOG_INF("Starting %s with CPU frequency: %d MHz", CONFIG_BOARD, SystemCoreClock/MHZ(1));
	k_sleep(K_SECONDS(1));
}

#ifdef CONFIG_WIFI_READY_LIB
static int register_wifi_ready(void)
{
	int ret = 0;
	wifi_ready_callback_t cb;
	struct net_if *iface = net_if_get_first_wifi();

	if (!iface) {
		LOG_ERR("Failed to get Wi-Fi interface");
		return -1;
	}

	cb.wifi_ready_cb = wifi_ready_cb;

	LOG_DBG("Registering Wi-Fi ready callbacks");
	ret = register_wifi_ready_callback(cb, iface);
	if (ret) {
		LOG_ERR("Failed to register Wi-Fi ready callbacks %s", strerror(ret));
		return ret;
	}

	return ret;
}
#endif /* CONFIG_WIFI_READY_LIB */



void uart_rx_handler(const struct device *dev, void *user_data) {
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1)) {
        // Debug output for every character received
        // printk("UART RX: 0x%02x ('%c')\n", c, isprint(c) ? c : '.');
		//echo back the character
		uart_fifo_fill(dev, &c, 1);
		// printk("%c", c);
   		
        if (c == '\n' || c == '\r') {
            rx_buf[rx_pos] = 0;
			printk("cmd %s \n", rx_buf);
			cmd_ready = true;  // Set a flag for execution
            // if (strncmp(rx_buf, "wifi", 4) == 0) {
            //     char ssid[32], psk[64];
			// 	printk("UART RX: %s\n", rx_buf);
            //     if (sscanf(rx_buf, "wifi_connect %31s %63s", ssid, psk) == 2) {
			// 		printk("wifi_connect %s %s\n", ssid, psk);
			// 		cmd_ready = true;  // Set a flag for execution
            //     }
            // } else {
			// 	printk("unkown cmd %s\n", rx_buf);

			// }
            rx_pos = 0;
        } else if (rx_pos < UART_RX_BUF_SIZE - 1) {
            rx_buf[rx_pos++] = c;
        }
    }
}
void process_uart_command(char *cmd) {
    // Add these lines
    printk("UART CMD RECEIVED: '%s'\n", cmd);
    
    // Before parsing the command
    printk("Starting command processing...\n");
    
    // Add more printk statements throughout your command parsing logic
    if (strncmp(cmd, "wifi_connect", 12) == 0) {
        printk("WiFi connect command detected\n");
        // Your existing processing code
    }
    
    // After command execution
    printk("Command processing complete\n");
}
int main_old(void)
{
	int ret = 0;

	net_mgmt_callback_init();

#ifdef CONFIG_WIFI_READY_LIB
	ret = register_wifi_ready();
	if (ret) {
		return ret;
	}
	k_thread_start(start_wifi_thread_id);
#else
	start_app();
#endif /* CONFIG_WIFI_READY_LIB */
	return 0;
}

/**
 * Helper function to test WiFi connection using handle_cmd
 */
static void test_wifi_connect(const char *ssid, const char *password)
{
    char cmd_buffer[256];
    
    // Construct the command string in the format handle_cmd expects
    snprintf(cmd_buffer, sizeof(cmd_buffer), "wifi_connect ssid=%s pw=%s", ssid, password);
    
    // Print debug message
    printk("Testing WiFi connection with command: %s\n", cmd_buffer);
    
    // Call the command handler
    handle_cmd(cmd_buffer);
}

static struct k_thread uart_thread;
K_THREAD_STACK_DEFINE(uart_stack, 8192);

void main(void) {
	int ret = 0;
    printk("System Boot\n");

    // Initialize mutex
    k_mutex_init(&cmd_mutex);	

	// cmd_uart = device_get_binding(CMD_UART_LABEL);
    // if (!cmd_uart) {
    //     printk("Failed to bind command UART\n");
    //     return;
    // }

	uart_irq_callback_set(cmd_uart, uart_rx_handler);
    uart_irq_rx_enable(cmd_uart);

    printk("Custom UART command handler initialized on UART1\n");
    printk("UART1 initialized for commands, baud: %d\n", 115200);



    // Create thread using the globally defined stack and thread struct
    k_thread_create(&uart_thread, 
                    uart_stack,
                    K_THREAD_STACK_SIZEOF(uart_stack),
                    uart_rx_thread,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(7),
                    0,
                    K_NO_WAIT);

	net_mgmt_callback_init();

	k_sleep(K_SECONDS(1));  // Wait for network stack to stabilize
    
  
	// Try to find and activate SLIP interface
    for (int i = 0; i < 8; i++) {
        struct net_if *iface = net_if_get_by_index(i);
        if (iface) {
            // Try to identify if this is a SLIP interface
            // For SLIP, we can check MTU (typically 1500) and whether it's not the loopback
            if (net_if_get_mtu(iface) == 1500 && !net_if_is_up(iface)) {
                printk("Found likely SLIP interface (index %d)\n", i);
                int ret = net_if_up(iface);
                printk("Bringing up SLIP interface: %s (result: %d)\n", 
                       ret == 0 ? "SUCCESS" : "FAILED", ret);
            }
        }
    }
// #ifdef CONFIG_WIFI_READY_LIB
// 	ret = register_wifi_ready();
// 	if (ret) {
// 		return ret;
// 	}
// 	k_thread_start(start_wifi_thread_id);
// #else
// 	start_app();
// #endif /* CONFIG_WIFI_READY_LIB */
// 
// test_wifi_connect("d10", "0928099869");

}
