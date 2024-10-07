/*
 ETH.h - espre ETH PHY support.
 Based on WiFi.h from Arduino WiFi shield library.
 Copyright (c) 2011-2014 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "ETH.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_eth_phy.h"
#include "esp_eth_mac.h"
#include "esp_eth_com.h"
#include "soc/rtc.h"
#include "lwip/err.h"
#include "lwip/dns.h"

#include "esp_eth_ksz8863.h"

extern void tcpipInit();
extern void add_esp_interface_netif(esp_interface_t interface, esp_netif_t* esp_netif); /* from WiFiGeneric */

/**
* @brief Callback function invoked when lowlevel initialization is finished
*
* @param[in] eth_handle: handle of Ethernet driver
*
* @return
*       - ESP_OK: process extra lowlevel initialization successfully
*       - ESP_FAIL: error occurred when processing extra lowlevel initialization
*/

static eth_clock_mode_t eth_clock_mode = ETH_CLK_MODE;

#if CONFIG_ETH_RMII_CLK_INPUT
/*
static void emac_config_apll_clock(void)
{
    // apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536)/((o_div + 2) * 2)
    rtc_xtal_freq_t rtc_xtal_freq = rtc_clk_xtal_freq_get();
    switch (rtc_xtal_freq) {
    case RTC_XTAL_FREQ_40M: // Recommended
        // 50 MHz = 40MHz * (4 + 6) / (2 * (2 + 2) = 50.000
        // sdm0 = 0, sdm1 = 0, sdm2 = 6, o_div = 2
        rtc_clk_apll_enable(true, 0, 0, 6, 2);
        break;
    case RTC_XTAL_FREQ_26M:
        // 50 MHz = 26MHz * (4 + 15 + 118 / 256 + 39/65536) / ((3 + 2) * 2) = 49.999992
        // sdm0 = 39, sdm1 = 118, sdm2 = 15, o_div = 3
        rtc_clk_apll_enable(true, 39, 118, 15, 3);
        break;
    case RTC_XTAL_FREQ_24M:
        // 50 MHz = 24MHz * (4 + 12 + 255 / 256 + 255/65536) / ((2 + 2) * 2) = 49.499977
        // sdm0 = 255, sdm1 = 255, sdm2 = 12, o_div = 2
        rtc_clk_apll_enable(true, 255, 255, 12, 2);
        break;
    default: // Assume we have a 40M xtal
        rtc_clk_apll_enable(true, 0, 0, 6, 2);
        break;
    }
}
*/
#endif

/**
* @brief Callback function invoked when lowlevel deinitialization is finished
*
* @param[in] eth_handle: handle of Ethernet driver
*
* @return
*       - ESP_OK: process extra lowlevel deinitialization successfully
*       - ESP_FAIL: error occurred when processing extra lowlevel deinitialization
*/
//static esp_err_t on_lowlevel_deinit_done(esp_eth_handle_t eth_handle){
//    return ESP_OK;
//}

ETHClass::ETHClass()
    :initialized(false)
    ,staticIP(false)
     ,eth_handle(NULL)
     ,started(false)
{
}

ETHClass::~ETHClass()
{}

bool ETHClass::begin(uint8_t phy_addr, int power, int mdc, int mdio, eth_phy_type_t type, eth_clock_mode_t clock_mode, bool use_mac_from_efuse)
{
    eth_clock_mode = clock_mode;
    tcpipInit();

    if (use_mac_from_efuse)
    {
        uint8_t p[6] = { 0x00,0x00,0x00,0x00,0x00,0x00 };
        esp_efuse_mac_get_custom(p);
        esp_base_mac_addr_set(p);
    }

    tcpip_adapter_set_default_eth_handlers();

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();


    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    phy_config.reset_gpio_num = -1; // KSZ8863 is reset by separate function call since multiple instances exist
    phy_config.phy_addr = -1; // this PHY is entry point to host
    // MIIM interface is not used since does not provide access to all registers
    esp32_emac_config.smi_gpio.mdc_num = -1;
    esp32_emac_config.smi_gpio.mdio_num = -1;


    esp_eth_mac_t *host_mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *host_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t host_config = ETH_KSZ8863_DEFAULT_CONFIG(host_mac, host_phy);
    host_config.on_lowlevel_init_done = ksz8863_board_specific_init;
    esp_eth_handle_t host_eth_handle = NULL;

    if(esp_eth_driver_install(&host_config, &host_eth_handle) != ESP_OK || host_eth_handle == NULL){
        log_e("Esp_eth_driver_install host failed");
        return false;
    }

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    /* attach Ethernet driver to TCP/IP stack */
    if(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(host_eth_handle)) != ESP_OK){
        log_e("esp_netif_attach host failed");
        return false;
    }

    // Init Switch P1 Ethernet Interface
    ksz8863_eth_mac_config_t ksz8863_pmac_config = {
        .pmac_mode = KSZ8863_SWITCH_MODE,
        .port_num = KSZ8863_PORT_1,
    };

    esp_eth_mac_t *p1_mac = esp_eth_mac_new_ksz8863(&ksz8863_pmac_config, &mac_config);
    phy_config.phy_addr = KSZ8863_PORT_1;
    esp_eth_phy_t *p1_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p1_config = ETH_KSZ8863_DEFAULT_CONFIG(p1_mac, p1_phy);
    esp_eth_handle_t p1_eth_handle = NULL;
    if(esp_eth_driver_install(&p1_config, &p1_eth_handle) != ESP_OK || p1_eth_handle == NULL){
        log_e("esp_eth_driver_install switch p1 failed");
        return false;
    }

    // Init Switch P2 Ethernet Interface
    ksz8863_pmac_config.port_num = KSZ8863_PORT_2;
    esp_eth_mac_t *p2_mac = esp_eth_mac_new_ksz8863(&ksz8863_pmac_config, &mac_config);
    phy_config.phy_addr = KSZ8863_PORT_2;
    esp_eth_phy_t *p2_phy = esp_eth_phy_new_ksz8863(&phy_config);

    esp_eth_config_t p2_config = ETH_KSZ8863_DEFAULT_CONFIG(p2_mac, p2_phy);
    esp_eth_handle_t p2_eth_handle = NULL;

    if(esp_eth_driver_install(&p2_config, &p2_eth_handle) != ESP_OK || p2_eth_handle == NULL){
        log_e("esp_eth_driver_install switch p2 failed");
        return false;
    }

    /* attach to WiFiGeneric to receive events */
    add_esp_interface_netif(ESP_IF_ETH, eth_netif);

    if(esp_eth_start(eth_handle) != ESP_OK){
        log_e("esp_eth_start failed");
        return false;
    }

    // holds a few milliseconds to let DHCP start and enter into a good state
    // FIX ME -- adresses issue https://github.com/espressif/arduino-esp32/issues/5733
    delay(50);

    return true;
}

// board specific initialization routine, user to update per specific needs
esp_err_t ksz8863_board_specific_init(esp_eth_handle_t eth_handle)
{
    esp_err_t ret = ESP_OK;

    // initialize I2C interface
    i2c_config_t i2c_bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_KHZ * 1000,
    };
    ESP_GOTO_ON_ERROR(i2c_init(CONFIG_EXAMPLE_I2C_MASTER_PORT, &i2c_bus_config), err, TAG, "I2C initialization failed");
    ksz8863_ctrl_i2c_config_t i2c_dev_config = {
        .dev_addr = KSZ8863_I2C_DEV_ADDR,
        .i2c_master_port = CONFIG_EXAMPLE_I2C_MASTER_PORT,
    };
    ksz8863_ctrl_intf_config_t ctrl_intf_cfg = {
        .host_mode = KSZ8863_I2C_MODE,
        .i2c_dev_config = &i2c_dev_config,
    };

    ESP_GOTO_ON_ERROR(ksz8863_ctrl_intf_init(&ctrl_intf_cfg), err, TAG, "KSZ8863 control interface initialization failed");

    ESP_GOTO_ON_ERROR(ksz8863_hw_reset(CONFIG_EXAMPLE_KSZ8863_RST_GPIO), err, TAG, "hardware reset failed");
    // it does not make much sense to execute SW reset right after HW reset but it is present here for demonstration purposes
    ESP_GOTO_ON_ERROR(ksz8863_sw_reset(eth_handle), err, TAG, "software reset failed");
err:
    return ret;
}

bool ETHClass::config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1, IPAddress dns2)
{
    esp_err_t err = ESP_OK;
    tcpip_adapter_ip_info_t info;

    if(static_cast<uint32_t>(local_ip) != 0){
        info.ip.addr = static_cast<uint32_t>(local_ip);
        info.gw.addr = static_cast<uint32_t>(gateway);
        info.netmask.addr = static_cast<uint32_t>(subnet);
    } else {
        info.ip.addr = 0;
        info.gw.addr = 0;
        info.netmask.addr = 0;
	}

    err = tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_ETH);
    if(err != ESP_OK && err != ESP_ERR_TCPIP_ADAPTER_DHCP_ALREADY_STOPPED){
        log_e("DHCP could not be stopped! Error: %d", err);
        return false;
    }

    err = tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_ETH, &info);
    if(err != ERR_OK){
        log_e("STA IP could not be configured! Error: %d", err);
        return false;
    }

    if(info.ip.addr){
        staticIP = true;
    } else {
        err = tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_ETH);
        if(err != ESP_OK && err != ESP_ERR_TCPIP_ADAPTER_DHCP_ALREADY_STARTED){
            log_w("DHCP could not be started! Error: %d", err);
            return false;
        }
        staticIP = false;
    }

    ip_addr_t d;
    d.type = IPADDR_TYPE_V4;

    if(static_cast<uint32_t>(dns1) != 0) {
        // Set DNS1-Server
        d.u_addr.ip4.addr = static_cast<uint32_t>(dns1);
        dns_setserver(0, &d);
    }

    if(static_cast<uint32_t>(dns2) != 0) {
        // Set DNS2-Server
        d.u_addr.ip4.addr = static_cast<uint32_t>(dns2);
        dns_setserver(1, &d);
    }

    return true;
}

IPAddress ETHClass::localIP()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return IPAddress();
    }
    return IPAddress(ip.ip.addr);
}

IPAddress ETHClass::subnetMask()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return IPAddress();
    }
    return IPAddress(ip.netmask.addr);
}

IPAddress ETHClass::gatewayIP()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return IPAddress();
    }
    return IPAddress(ip.gw.addr);
}

IPAddress ETHClass::dnsIP(uint8_t dns_no)
{
    const ip_addr_t * dns_ip = dns_getserver(dns_no);
    return IPAddress(dns_ip->u_addr.ip4.addr);
}

IPAddress ETHClass::broadcastIP()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return IPAddress();
    }
    return WiFiGenericClass::calculateBroadcast(IPAddress(ip.gw.addr), IPAddress(ip.netmask.addr));
}

IPAddress ETHClass::networkID()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return IPAddress();
    }
    return WiFiGenericClass::calculateNetworkID(IPAddress(ip.gw.addr), IPAddress(ip.netmask.addr));
}

uint8_t ETHClass::subnetCIDR()
{
    tcpip_adapter_ip_info_t ip;
    if(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip)){
        return (uint8_t)0;
    }
    return WiFiGenericClass::calculateSubnetCIDR(IPAddress(ip.netmask.addr));
}

const char * ETHClass::getHostname()
{
    const char * hostname;
    if(tcpip_adapter_get_hostname(TCPIP_ADAPTER_IF_ETH, &hostname)){
        return NULL;
    }
    return hostname;
}

bool ETHClass::setHostname(const char * hostname)
{
    return tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_ETH, hostname) == 0;
}

bool ETHClass::fullDuplex()
{
    eth_duplex_t link_duplex;
    esp_eth_ioctl(eth_handle, ETH_CMD_G_DUPLEX_MODE, &link_duplex);
    return (link_duplex == ETH_DUPLEX_FULL);
}

bool ETHClass::linkUp()
{
    return WiFiGenericClass::getStatusBits() & ETH_CONNECTED_BIT;
}

uint8_t ETHClass::linkSpeed()
{
    eth_speed_t link_speed;
    esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &link_speed);
    return (link_speed == ETH_SPEED_10M)?10:100;
}

bool ETHClass::enableIpV6()
{
    return tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_ETH) == 0;
}

IPv6Address ETHClass::localIPv6()
{
    static ip6_addr_t addr;
    if(tcpip_adapter_get_ip6_linklocal(TCPIP_ADAPTER_IF_ETH, &addr)){
        return IPv6Address();
    }
    return IPv6Address(addr.addr);
}

uint8_t * ETHClass::macAddress(uint8_t* mac)
{
    if(!mac){
        return NULL;
    }
#ifdef ESP_IDF_VERSION_MAJOR
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac);
#else
    esp_eth_get_mac(mac);
#endif
    return mac;
}

String ETHClass::macAddress(void)
{
    uint8_t mac[6] = {0,0,0,0,0,0};
    char macStr[18] = { 0 };
    macAddress(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

ETHClass ETH;