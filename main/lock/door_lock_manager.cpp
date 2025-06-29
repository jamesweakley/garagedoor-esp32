/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "door_lock_manager.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <cstring>
#include <esp_log.h>
#include <esp_matter.h>
#include <platform/CHIPDeviceLayer.h>

static const char *TAG = "doorlock_manager";

// External reference to the endpoint IDs defined in app_main.cpp
extern uint16_t door_lock_endpoint_id;
extern uint16_t contact_sensor_endpoint_id;

BoltLockManager BoltLockManager::sLock;

// Initialize static variables
bool BoltLockManager::sContactSensorStateChanged = false;
bool BoltLockManager::sContactSensorState = false;
SemaphoreHandle_t BoltLockManager::sContactSensorMutex = NULL;

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::DoorLock;

BoltLockManager::~BoltLockManager()
{
    // Clean up the mutex if it exists
    if (sContactSensorMutex != NULL) {
        vSemaphoreDelete(sContactSensorMutex);
        sContactSensorMutex = NULL;
        ESP_LOGI(TAG, "Cleaned up contact sensor mutex");
    }
    
    // Stop the door sensor task if it's running
    if (mDoorSensorTaskHandle != NULL) {
        vTaskDelete(mDoorSensorTaskHandle);
        mDoorSensorTaskHandle = NULL;
        ESP_LOGI(TAG, "Stopped door sensor task");
    }
}

CHIP_ERROR BoltLockManager::Init(DataModel::Nullable<DlLockState> state)
{
    ESP_LOGI(TAG, "Initializing simplified door lock manager");
    
    // Create mutex for thread-safe access to contact sensor state
    if (sContactSensorMutex == NULL) {
        sContactSensorMutex = xSemaphoreCreateMutex();
        if (sContactSensorMutex == NULL) {
            ESP_LOGE(TAG, "Failed to create contact sensor mutex");
            return CHIP_ERROR_NO_MEMORY;
        }
        ESP_LOGI(TAG, "Created contact sensor mutex for thread-safe access");
    }
    
    // Initialize GPIO pins for actuator control
    initActuatorPins();
    
    // Initialize door sensor
    initDoorSensor();
    
    // Create a task to monitor the door sensor
    xTaskCreate(doorSensorTask, "door_sensor_task", 2048, this, 5, &mDoorSensorTaskHandle);
    
    return CHIP_NO_ERROR;
}

void BoltLockManager::initActuatorPins()
{
    ESP_LOGI(TAG, "Initializing actuator pins with maximum drive capability");
    
    // Configure GPIO pins for actuator control
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ACTUATOR_PIN_OPEN) | (1ULL << ACTUATOR_PIN_CLOSE);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;  // Disable pull-up resistors
    gpio_config(&io_conf);
    
    // Initialize both pins to low
    gpio_set_level(ACTUATOR_PIN_OPEN, 0);
    gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
    
    // Set maximum GPIO drive capability for both pins
    // GPIO_DRIVE_CAP_3 is the strongest drive capability (40mA)
    ESP_ERROR_CHECK(gpio_set_drive_capability(ACTUATOR_PIN_OPEN, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_drive_capability(ACTUATOR_PIN_CLOSE, GPIO_DRIVE_CAP_3));
    
    // Verify drive capability was set correctly
    gpio_drive_cap_t drive_cap_open, drive_cap_close;
    ESP_ERROR_CHECK(gpio_get_drive_capability(ACTUATOR_PIN_OPEN, &drive_cap_open));
    ESP_ERROR_CHECK(gpio_get_drive_capability(ACTUATOR_PIN_CLOSE, &drive_cap_close));
    
    ESP_LOGI(TAG, "Actuator pins initialized: OPEN=%d (drive=%d), CLOSE=%d (drive=%d)",
             ACTUATOR_PIN_OPEN, drive_cap_open, ACTUATOR_PIN_CLOSE, drive_cap_close);
}

void BoltLockManager::initDoorSensor()
{
    ESP_LOGI(TAG, "Initializing reed switch door sensor with single pin GP%d (INVERTED LOGIC)",
             REED_SWITCH_PIN);
    
    // Reset the pin to default state
    gpio_reset_pin(REED_SWITCH_PIN);
    
    // Configure the pin as input with pull-up
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_DISABLE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pin_bit_mask = (1ULL << REED_SWITCH_PIN);
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&input_conf);
    
    // Explicitly set the internal pull-up resistor for the input pin
    gpio_set_pull_mode(REED_SWITCH_PIN, GPIO_PULLUP_ONLY);
    
    // Add a small delay to allow the GPIO to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Read the initial state a few times to ensure stability
    int state1 = gpio_get_level(REED_SWITCH_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
    int state2 = gpio_get_level(REED_SWITCH_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
    int state3 = gpio_get_level(REED_SWITCH_PIN);
    
    // Initialize door state
    mDoorIsOpen = getDoorState();
    
    ESP_LOGI(TAG, "Door sensor initialized: PIN=%d (sense), Raw GPIO readings: %d, %d, %d, Initial state: %s",
             REED_SWITCH_PIN, state1, state2, state3, mDoorIsOpen ? "OPEN" : "CLOSED");
}

bool BoltLockManager::getDoorState()
{
    // INVERTED LOGIC with pull-up resistor on a single pin:
    // When the pin is shorted to ground, the pin reads LOW (0) = DOOR OPEN
    // When the pin is not shorted, the pin reads HIGH (1) = DOOR CLOSED
    
    // Read the pin multiple times to debounce
    int reading1 = gpio_get_level(REED_SWITCH_PIN);
    vTaskDelay(pdMS_TO_TICKS(5));
    int reading2 = gpio_get_level(REED_SWITCH_PIN);
    vTaskDelay(pdMS_TO_TICKS(5));
    int reading3 = gpio_get_level(REED_SWITCH_PIN);
    
    // Use the majority reading
    int state;
    if ((reading1 + reading2 + reading3) >= 2) {
        state = 1; // Majority HIGH
    } else {
        state = 0; // Majority LOW
    }
    
    // Enhanced debugging information
    ESP_LOGI(TAG, "Reed switch readings: PIN=GP%d, Raw GPIO=%d,%d,%d (final=%d)",
             REED_SWITCH_PIN, reading1, reading2, reading3, state);
    
    // INVERTED LOGIC to match actual hardware behavior:
    // When pin is shorted to ground, GPIO reads LOW (0), meaning door is OPEN
    // When pin is not shorted, GPIO reads HIGH (1), meaning door is CLOSED
    bool doorState = (state == 1) ? DOOR_STATE_CLOSED : DOOR_STATE_OPEN;
    
    // Log the door state with clear instructions for testing
    ESP_LOGI(TAG, "Door state: %s (GPIO=%d) - %s",
            doorState ? "OPEN" : "CLOSED",
            state,
            doorState ? "Reed switch shorted to ground" : "Reed switch NOT shorted");
    
    return doorState;
}

void BoltLockManager::updateContactSensorState(bool isOpen)
{
    // Check if the state has actually changed
    static bool lastState = !isOpen; // Initialize to opposite to force first update
    bool stateChanged = (lastState != isOpen);
    
    if (stateChanged) {
        // Log the state change with clear information
        ESP_LOGI(TAG, "Contact sensor state CHANGED: %s -> %s",
                lastState ? "OPEN (active)" : "CLOSED (inactive)",
                isOpen ? "OPEN (active)" : "CLOSED (inactive)");
        
        // Update the last state
        lastState = isOpen;
    } else {
        // Log that the state hasn't changed
        ESP_LOGI(TAG, "Contact sensor state UNCHANGED: Still %s",
                isOpen ? "OPEN (active)" : "CLOSED (inactive)");
    }
    
    // Schedule the update on the Matter thread
    ScheduleContactSensorUpdate(isOpen);
}

void BoltLockManager::ScheduleContactSensorUpdate(bool isOpen)
{
    // Store the state in a static variable that the handler can access
    static bool sStateToUpdate = false;
    sStateToUpdate = isOpen;
    
    // Schedule the work on the Matter thread
    ESP_LOGI(TAG, "Scheduling contact sensor update on Matter thread");
    chip::DeviceLayer::PlatformMgr().ScheduleWork(ContactSensorUpdateHandler, reinterpret_cast<intptr_t>(&sStateToUpdate));
}

void BoltLockManager::ContactSensorUpdateHandler(intptr_t context)
{
    // This method runs in the Matter thread context
    bool* isOpenPtr = reinterpret_cast<bool*>(context);
    bool isOpen = *isOpenPtr;
    
    // Get the endpoint ID from the global variable
    extern uint16_t contact_sensor_endpoint_id;
    
    if (contact_sensor_endpoint_id > 0) {
        // Use the esp_matter API to update the contact sensor state
        // BooleanState cluster ID: 0x0045, StateValue attribute ID: 0x0000
        esp_matter_attr_val_t val = esp_matter_bool(isOpen);
        
        // This is safe because we're in the Matter context
        esp_err_t err = esp_matter::attribute::report(contact_sensor_endpoint_id, 0x0045, 0x0000, &val);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Matter thread: Updated contact sensor state to %s",
                     isOpen ? "OPEN (active)" : "CLOSED (inactive)");
        } else {
            ESP_LOGE(TAG, "Failed to update contact sensor state: %d", err);
        }
    } else {
        ESP_LOGE(TAG, "Contact sensor endpoint ID not set");
    }
}

void BoltLockManager::updateDoorState(bool isOpen)
{
    // Update the internal state
    mDoorIsOpen = isOpen;
    
    // Log the door state change
    ESP_LOGI(TAG, "Door state changed: %s", isOpen ? "OPEN" : "CLOSED");
    
    // Schedule the door lock state update on the Matter thread
    struct DoorStateContext {
        bool isOpen;
        EndpointId lockEndpointId;
    };
    
    static DoorStateContext doorContext;
    doorContext.isOpen = isOpen;
    doorContext.lockEndpointId = 1;
    
    // Schedule the door lock state update on the Matter thread
    ESP_LOGI(TAG, "Scheduling door lock state update on Matter thread");
    chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t context) {
            DoorStateContext* ctx = reinterpret_cast<DoorStateContext*>(context);
            
            // If the door is open but the lock state is locked, update to not fully locked
            if (ctx->isOpen) {
                DlLockState currentState;
                DataModel::Nullable<DlLockState> state;
                DoorLock::Attributes::LockState::Get(ctx->lockEndpointId, state);
                
                if (!state.IsNull()) {
                    currentState = state.Value();
                    if (currentState == DlLockState::kLocked) {
                        // Door is open but lock state is locked, update to not fully locked
                        DoorLockServer::Instance().SetLockState(ctx->lockEndpointId, DlLockState::kNotFullyLocked);
                        ESP_LOGI(TAG, "Updated lock state to NOT_FULLY_LOCKED because door is open");
                    }
                }
            }
        },
        reinterpret_cast<intptr_t>(&doorContext)
    );
    
    // Update the contact sensor state
    updateContactSensorState(isOpen);
}

void BoltLockManager::doorSensorTask(void *pvParameters)
{
    BoltLockManager *manager = static_cast<BoltLockManager *>(pvParameters);
    bool lastDoorState = manager->getDoorState();
    
    // Log initial state
    ESP_LOGI(TAG, "Door sensor task started. Initial state: %s",
             lastDoorState ? "OPEN" : "CLOSED");
    
    // Force an initial update to ensure the contact sensor state is set correctly
    manager->updateDoorState(lastDoorState);
    
    // Counter for periodic status logging
    int logCounter = 0;
    
    while (1) {
        // Read current door state
        bool currentDoorState = manager->getDoorState();
        
        // If door state has changed, update it immediately
        if (currentDoorState != lastDoorState) {
            ESP_LOGI(TAG, "Door state changed from %s to %s",
                     lastDoorState ? "OPEN" : "CLOSED",
                     currentDoorState ? "OPEN" : "CLOSED");
            
            // Update the door state in the system
            manager->updateDoorState(currentDoorState);
            lastDoorState = currentDoorState;
        }
        
        // Periodically log the current state (every ~5 seconds)
        if (++logCounter >= 50) {
            ESP_LOGI(TAG, "Door sensor periodic status: %s",
                     currentDoorState ? "OPEN" : "CLOSED");
            logCounter = 0;
        }
        
        // Check every 100ms for more responsive detection
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void BoltLockManager::controlActuator(bool isOpen)
{
    const int startup_delay = 100;        // 100ms startup delay
    const int direction_change_delay = 300; // 300ms when changing direction
    const int operation_time = 5000;      // 5 seconds of continuous operation
    
    ESP_LOGI(TAG, "Door actuator: Starting %s operation", isOpen ? "OPEN" : "CLOSE");
    
    // First ensure both pins are off to avoid any conflicts
    gpio_set_level(ACTUATOR_PIN_OPEN, 0);
    gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
    vTaskDelay(pdMS_TO_TICKS(direction_change_delay));
    
    if (isOpen) {
        // Open the door
        ESP_LOGI(TAG, "Setting CLOSE pin to LOW");
        gpio_set_level(ACTUATOR_PIN_CLOSE, 0);  // Ensure close pin is off
        vTaskDelay(pdMS_TO_TICKS(startup_delay));
        
        ESP_LOGI(TAG, "Setting OPEN pin to HIGH");
        gpio_set_level(ACTUATOR_PIN_OPEN, 1);   // Activate open pin
        
        // Keep the actuator running for the specified time
        vTaskDelay(pdMS_TO_TICKS(operation_time));
        
        // Turn off the pin when done
        gpio_set_level(ACTUATOR_PIN_OPEN, 0);
        ESP_LOGI(TAG, "Door actuator: OPENED");
    } else {
        // Close the door
        ESP_LOGI(TAG, "Setting OPEN pin to LOW");
        gpio_set_level(ACTUATOR_PIN_OPEN, 0);   // Ensure open pin is off
        vTaskDelay(pdMS_TO_TICKS(startup_delay));
        
        ESP_LOGI(TAG, "Setting CLOSE pin to HIGH");
        gpio_set_level(ACTUATOR_PIN_CLOSE, 1);  // Activate close pin
        
        // Keep the actuator running for the specified time
        vTaskDelay(pdMS_TO_TICKS(operation_time));
        
        // Turn off the pin when done
        gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
        ESP_LOGI(TAG, "Door actuator: CLOSED");
    }
    
    ESP_LOGI(TAG, "Door actuator: Operation completed");
}

bool BoltLockManager::Lock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Lock command received [endpointId=%d]", endpointId);
    // These methods are called from the Matter context, so they're safe to use
    return setLockState(endpointId, DlLockState::kLocked, pin, err);
}

bool BoltLockManager::Unlock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Unlock command received [endpointId=%d]", endpointId);
    // These methods are called from the Matter context, so they're safe to use
    return setLockState(endpointId, DlLockState::kUnlocked, pin, err);
}

const char * BoltLockManager::lockStateToString(DlLockState lockState) const
{
    switch (lockState)
    {
    case DlLockState::kNotFullyLocked:
        return "Not Fully Locked";
    case DlLockState::kLocked:
        return "Locked";
    case DlLockState::kUnlocked:
        return "Unlocked";
    case DlLockState::kUnlatched:
        return "Unlatched";
    case DlLockState::kUnknownEnumValue:
        break;
    }

    return "Unknown";
}

bool BoltLockManager::setLockState(EndpointId endpointId, DlLockState lockState, const Optional<ByteSpan> & pin,
                                   OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Setting door lock state to \"%s\" [endpointId=%d]", lockStateToString(lockState), endpointId);
    
    // Update the lock state in the Matter system
    // This is safe because this method is called from the Matter context
    DoorLockServer::Instance().SetLockState(endpointId, lockState);
    
    // Control the actuator based on the lock state
    if (lockState == DlLockState::kLocked) {
        controlActuator(false); // Close the door
    } else if (lockState == DlLockState::kUnlocked) {
        controlActuator(true);  // Open the door
    }
    
    return true;
}

CHIP_ERROR BoltLockManager::InitLockState()
{
    // Initial lock state
    DataModel::Nullable<DlLockState> state;
    EndpointId lockEndpointId{ 1 };
    DoorLock::Attributes::LockState::Get(lockEndpointId, state);

    // Initialize the simplified door lock manager
    CHIP_ERROR err = BoltLockMgr().Init(state);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "BoltLockMgr().Init() failed");
        return err;
    }

    // Set initial state to locked
    OperationErrorEnum opErr;
    setLockState(lockEndpointId, DlLockState::kLocked, Optional<ByteSpan>(), opErr);
    
    // Check initial door state
    bool doorIsOpen = BoltLockMgr().getDoorState();
    
    // Update contact sensor state with initial door state
    BoltLockMgr().updateContactSensorState(doorIsOpen);
    
    if (doorIsOpen) {
        // If door is open but lock state is locked, update to not fully locked
        DoorLockServer::Instance().SetLockState(lockEndpointId, DlLockState::kNotFullyLocked);
        ESP_LOGI(TAG, "Initial door state is OPEN, setting lock state to NOT_FULLY_LOCKED");
    }
    
    ESP_LOGI(TAG, "Door lock and contact sensor initialized successfully");
    return CHIP_NO_ERROR;
}
