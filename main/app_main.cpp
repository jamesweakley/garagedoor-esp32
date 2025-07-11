/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_timer.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include "lock/door_lock_manager.h"
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t door_lock_endpoint_id = 0;
uint16_t contact_sensor_endpoint_id = 0;

// Watchdog timer for detecting stuck initialization
static esp_timer_handle_t init_watchdog_timer = NULL;
static bool matter_started = false;

static void init_watchdog_callback(void* arg)
{
    if (!matter_started) {
        ESP_LOGE(TAG, "Matter initialization appears stuck, restarting device...");
        esp_restart();
    }
}

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using namespace chip;

constexpr auto k_timeout_seconds = 300;

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    // No need to check for contact sensor updates here anymore
    // Updates are now scheduled directly on the Matter thread
    
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        // Try to reopen commissioning window after failure
        {
            chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen())
            {
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR)
                {
                    ESP_LOGE(TAG, "Failed to reopen commissioning window after fail safe, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
                else
                {
                    ESP_LOGI(TAG, "Reopened commissioning window after fail safe timer expired");
                }
            }
        }
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;
        
    // Matter connection events
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionEstablished:
        ESP_LOGI(TAG, "BLE connection established");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionClosed:
        ESP_LOGI(TAG, "BLE connection closed");
        break;
        
    // Handle BLE advertising errors
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEAdvertisingChange:
        ESP_LOGI(TAG, "BLE advertising state changed");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished:
        ESP_LOGI(TAG, "Secure session established");
        break;
        
    // Thread/OpenThread events
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(TAG, "Thread state changed");
        break;
        
    // Add error handling for connectivity issues
    case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
        ESP_LOGI(TAG, "DNS-SD initialized");
        break;

    default:
        // Log unknown events for debugging
        ESP_LOGD(TAG, "Unhandled device event: %d", event->Type);
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
// Custom attribute callback for contact sensor
static esp_err_t contact_sensor_attribute_callback(attribute::callback_type_t type, uint16_t endpoint_id,
                                                  uint32_t cluster_id, uint32_t attribute_id,
                                                  esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    
    // Only handle the contact sensor endpoint
    if (endpoint_id == contact_sensor_endpoint_id) {
        // BooleanState cluster ID: 0x0045, StateValue attribute ID: 0x0000
        if (cluster_id == 0x0045 && attribute_id == 0x0000) {
            if (type == attribute::PRE_UPDATE) {
                // This is called before the attribute is updated
                ESP_LOGI(TAG, "Contact sensor state will be updated to: %s",
                         val->val.b ? "OPEN (active)" : "CLOSED (inactive)");
            } else if (type == attribute::POST_UPDATE) {
                // This is called after the attribute is updated
                ESP_LOGI(TAG, "Contact sensor state was updated to: %s",
                         val->val.b ? "OPEN (active)" : "CLOSED (inactive)");
            }
        }
    }
    
    return err;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }
    
    // Also call our custom contact sensor callback
    contact_sensor_attribute_callback(type, endpoint_id, cluster_id, attribute_id, val, priv_data);

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Clear BLE bonding data to resolve "Failed to restore IRKs from store" errors
    // This is necessary when the BLE bonding data becomes corrupted
    nvs_handle_t nvs_handle;
    err = nvs_open("bt_cfg", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = 0;
        err = nvs_get_blob(nvs_handle, "bt_cfg", NULL, &required_size);
        if (err == ESP_OK && required_size > 0) {
            ESP_LOGI(TAG, "Found existing BLE configuration data (%d bytes), clearing to prevent IRK errors", required_size);
            nvs_erase_key(nvs_handle, "bt_cfg");
            ESP_LOGI(TAG, "Cleared potentially corrupted BLE configuration");
        }
        nvs_close(nvs_handle);
    }
    
    // Also clear other BLE-related NVS keys that might be corrupted
    err = nvs_open("nimble_bond", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared NimBLE bonding data");
    }
    
    err = nvs_open("bt_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared BT config data");
    }

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Create a garage door controller using the door lock cluster
    door_lock::config_t door_lock_config;
    // endpoint handles can be used to add/modify clusters.
    endpoint_t *endpoint = door_lock::create(node, &door_lock_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create garage door endpoint"));
    
    // Get the door lock cluster but don't add advanced features
    cluster_t *door_lock_cluster = cluster::get(endpoint, DoorLock::Id);
    
    // Set auto-relock time to 0 (disabled) since we're using a simple relay toggle
    cluster::door_lock::attribute::create_auto_relock_time(door_lock_cluster, 0);
    
    ESP_LOGI(TAG, "Created garage door controller endpoint");

    door_lock_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Garage door controller created with endpoint_id %d", door_lock_endpoint_id);
    
    // Create a contact sensor endpoint for the garage door reed switch
    contact_sensor::config_t contact_sensor_config;
    endpoint_t *contact_sensor_ep = contact_sensor::create(node, &contact_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(contact_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create contact sensor endpoint"));
    
    // Store the contact sensor endpoint ID
    contact_sensor_endpoint_id = endpoint::get_id(contact_sensor_ep);
    ESP_LOGI(TAG, "Contact sensor created with endpoint_id %d", contact_sensor_endpoint_id);
    
    // We'll use the global attribute callback mechanism that's already set up
    // The contact_sensor_attribute_callback will be called through app_attribute_update_cb
    ESP_LOGI(TAG, "Contact sensor will use the global attribute callback mechanism");

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Initialize door lock before starting Matter */
    door_lock_init();

    // Start a watchdog timer to detect if Matter initialization gets stuck
    esp_timer_create_args_t timer_args = {
        .callback = init_watchdog_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "init_watchdog"
    };
    esp_timer_create(&timer_args, &init_watchdog_timer);
    esp_timer_start_once(init_watchdog_timer, 30000000); // 30 seconds timeout
    ESP_LOGI(TAG, "Started initialization watchdog timer (30s timeout)");

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
    
    // Mark that Matter has started successfully
    matter_started = true;
    if (init_watchdog_timer) {
        esp_timer_stop(init_watchdog_timer);
        esp_timer_delete(init_watchdog_timer);
        init_watchdog_timer = NULL;
        ESP_LOGI(TAG, "Matter started successfully, stopped watchdog timer");
    }

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}
