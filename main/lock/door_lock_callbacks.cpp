/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include "door_lock_manager.h"
#include <app/clusters/door-lock-server/door-lock-server.h>
#include <platform/CHIPDeviceLayer.h>

#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/ConcreteAttributePath.h>
#include <app/data-model/Nullable.h>
#include <lib/core/DataModelTypes.h>

using namespace chip::app::Clusters;
using chip::app::DataModel::Nullable;

static const char *TAG = "doorlock_callback";

void door_lock_init()
{
    ESP_LOGI(TAG, "Simple door lock init");
}

void emberAfDoorLockClusterInitCallback(EndpointId endpoint)
{
    ESP_LOGI(TAG, "Door Lock Cluster Init Callback for endpoint %d", endpoint);
    DoorLockServer::Instance().InitServer(endpoint);
    BoltLockMgr().InitLockState();
}

bool emberAfPluginDoorLockOnDoorLockCommand(chip::EndpointId endpointId, const Nullable<chip::FabricIndex> & fabricIdx,
                                            const Nullable<chip::NodeId> & nodeId, const Optional<ByteSpan> & pinCode,
                                            OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Lock Command endpoint=%d", endpointId);
    bool status = BoltLockMgr().Lock(endpointId, pinCode, err);
    return status;
}

bool emberAfPluginDoorLockOnDoorUnlockCommand(chip::EndpointId endpointId, const Nullable<chip::FabricIndex> & fabricIdx,
                                              const Nullable<chip::NodeId> & nodeId, const Optional<ByteSpan> & pinCode,
                                              OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Unlock Command endpoint=%d", endpointId);
    bool status = BoltLockMgr().Unlock(endpointId, pinCode, err);
    return status;
}

// Simplified implementation - these functions are required by the Matter framework
// but we don't need to implement their full functionality for our simple door lock

bool emberAfPluginDoorLockGetCredential(chip::EndpointId endpointId, uint16_t credentialIndex, CredentialTypeEnum credentialType,
                                        EmberAfPluginDoorLockCredentialInfo & credential)
{
    ESP_LOGI(TAG, "GetCredential called but not implemented in simplified door lock");
    credential.status = DlCredentialStatus::kAvailable;
    return true;
}

bool emberAfPluginDoorLockSetCredential(chip::EndpointId endpointId, uint16_t credentialIndex, chip::FabricIndex creator,
                                        chip::FabricIndex modifier, DlCredentialStatus credentialStatus,
                                        CredentialTypeEnum credentialType, const chip::ByteSpan & credentialData)
{
    ESP_LOGI(TAG, "SetCredential called but not implemented in simplified door lock");
    return true;
}

bool emberAfPluginDoorLockGetUser(chip::EndpointId endpointId, uint16_t userIndex, EmberAfPluginDoorLockUserInfo & user)
{
    ESP_LOGI(TAG, "GetUser called but not implemented in simplified door lock");
    user.userStatus = UserStatusEnum::kAvailable;
    return true;
}

bool emberAfPluginDoorLockSetUser(chip::EndpointId endpointId, uint16_t userIndex, chip::FabricIndex creator,
                                  chip::FabricIndex modifier, const chip::CharSpan & userName, uint32_t uniqueId,
                                  UserStatusEnum userStatus, UserTypeEnum usertype, CredentialRuleEnum credentialRule,
                                  const CredentialStruct * credentials, size_t totalCredentials)
{
    ESP_LOGI(TAG, "SetUser called but not implemented in simplified door lock");
    return true;
}

DlStatus emberAfPluginDoorLockGetSchedule(chip::EndpointId endpointId, uint8_t weekdayIndex, uint16_t userIndex,
                                          EmberAfPluginDoorLockWeekDaySchedule & schedule)
{
    ESP_LOGI(TAG, "GetSchedule (weekday) called but not implemented in simplified door lock");
    return DlStatus::kNotFound;
}

DlStatus emberAfPluginDoorLockGetSchedule(chip::EndpointId endpointId, uint8_t yearDayIndex, uint16_t userIndex,
                                          EmberAfPluginDoorLockYearDaySchedule & schedule)
{
    ESP_LOGI(TAG, "GetSchedule (yearday) called but not implemented in simplified door lock");
    return DlStatus::kNotFound;
}

DlStatus emberAfPluginDoorLockGetSchedule(chip::EndpointId endpointId, uint8_t holidayIndex,
                                          EmberAfPluginDoorLockHolidaySchedule & holidaySchedule)
{
    ESP_LOGI(TAG, "GetSchedule (holiday) called but not implemented in simplified door lock");
    return DlStatus::kNotFound;
}

DlStatus emberAfPluginDoorLockSetSchedule(chip::EndpointId endpointId, uint8_t weekdayIndex, uint16_t userIndex,
                                          DlScheduleStatus status, DaysMaskMap daysMask, uint8_t startHour, uint8_t startMinute,
                                          uint8_t endHour, uint8_t endMinute)
{
    ESP_LOGI(TAG, "SetSchedule (weekday) called but not implemented in simplified door lock");
    return DlStatus::kSuccess;
}

DlStatus emberAfPluginDoorLockSetSchedule(chip::EndpointId endpointId, uint8_t yearDayIndex, uint16_t userIndex,
                                          DlScheduleStatus status, uint32_t localStartTime, uint32_t localEndTime)
{
    ESP_LOGI(TAG, "SetSchedule (yearday) called but not implemented in simplified door lock");
    return DlStatus::kSuccess;
}

DlStatus emberAfPluginDoorLockSetSchedule(chip::EndpointId endpointId, uint8_t holidayIndex, DlScheduleStatus status,
                                          uint32_t localStartTime, uint32_t localEndTime, OperatingModeEnum operatingMode)
{
    ESP_LOGI(TAG, "SetSchedule (holiday) called but not implemented in simplified door lock");
    return DlStatus::kSuccess;
}

void emberAfPluginDoorLockOnAutoRelock(chip::EndpointId endpointId)
{
    ESP_LOGI(TAG, "Auto relock called but not implemented in simplified door lock");
}
