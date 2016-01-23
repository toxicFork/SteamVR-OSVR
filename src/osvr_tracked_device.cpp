/** @file
@brief Header

@date 2016

@author
Sensics, Inc.
<http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <headers/osvr_tracked_device.h>

OSVRTrackedDevice::OSVRTrackedDevice(osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::ETrackedDeviceClass deviceClass, vr::IDriverLog* driver_log) : 
    m_Context(context), 
    driver_host_(driver_host), 
    logger_(driver_log), 
    pose_(), 
    deviceClass_(deviceClass)
{
}

vr::EVRInitError OSVRTrackedDevice::Activate(uint32_t object_id)
{
    const std::time_t waitTime = 5; // wait up to 5 seconds for init
                                    // Register tracker callback
    if (m_TrackerInterface.notEmpty()) {
        m_TrackerInterface.free();
    }

    // ensure context is fully started up
    logger_->Log("Waiting for the context to fully start up...\n");
    std::time_t startTime = std::time(nullptr);
    while (!m_Context.checkStatus()) {
        m_Context.update();
        if (std::time(nullptr) > startTime + waitTime) {
            logger_->Log("Context startup timed out!\n");
            return vr::VRInitError_Driver_Failed;
        }
    }

    return vr::VRInitError_None;
}

void OSVRTrackedDevice::Deactivate()
{
    /// Have to force freeing here
    if (m_TrackerInterface.notEmpty()) {
        m_TrackerInterface.free();
    }
}

void OSVRTrackedDevice::DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size)
{
    // TODO
    // make use of (from vrtypes.h) static const uint32_t k_unMaxDriverDebugResponseSize = 32768;
}

void OSVRTrackedDevice::GetWindowBounds(int32_t* x, int32_t* y, uint32_t* width, uint32_t* height)
{
    logger_->Log("OSVRTrackedDevice::GetWindowBounds");
}

bool OSVRTrackedDevice::IsDisplayOnDesktop()
{
    logger_->Log("OSVRTrackedDevice::IsDisplayOnDesktop");
    return true;
}

bool OSVRTrackedDevice::IsDisplayRealDisplay()
{
    logger_->Log("OSVRTrackedDevice::IsDisplayRealDisplay");
    return true;
}

void OSVRTrackedDevice::GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height)
{
    logger_->Log("OSVRTrackedDevice::GetRecommendedRenderTargetSize");
}

void OSVRTrackedDevice::GetEyeOutputViewport(vr::EVREye eye, uint32_t* x, uint32_t* y, uint32_t* width, uint32_t* height)
{
    logger_->Log("OSVRTrackedDevice::GetEyeOutputViewport");
}

void OSVRTrackedDevice::GetProjectionRaw(vr::EVREye eye, float* left, float* right, float* top, float* bottom)
{
    logger_->Log("OSVRTrackedDevice::GetProjectionRaw");
}

vr::DistortionCoordinates_t OSVRTrackedDevice::ComputeDistortion(vr::EVREye eye, float u, float v)
{
    logger_->Log("OSVRTrackedDevice::ComputeDistortion");
    vr::DistortionCoordinates_t coords = vr::DistortionCoordinates_t();
    return coords;
}

vr::TrackedDeviceDriverInfo_t OSVRTrackedDevice::GetTrackedDeviceDriverInfo()
{
    vr::TrackedDeviceDriverInfo_t info;
    util::strcpy_safe(info.rchTrackingSystemId, "OSVR"); // TODO name of the underlying tracking system
    util::strcpy_safe(info.rchSerialNumber, GetSerialNumber());
    util::strcpy_safe(info.rchModelNumber, GetModelNumber());
    info.rchRenderModelName[0] = '\0';        // TODO pass this to GetRenderModel to get the mesh and texture to render this device
    info.eClass = deviceClass_; // TODO adjust accordingly

    return info;
}

const char* OSVRTrackedDevice::GetModelNumber()
{
    /// @todo When available, return the actual model number of the Device
    return "UNKNOWN";
}

const char* OSVRTrackedDevice::GetSerialNumber()
{
    /// @todo When available, return the actual serial number of the Device
    return "0";
}

vr::DriverPose_t OSVRTrackedDevice::GetPose()
{
    return pose_;
}

bool OSVRTrackedDevice::GetBoolTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const bool default_value = false;

    if (isWrongDataType(prop, bool())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_WillDriftInYaw_Bool: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
        break;
    case vr::Prop_ReportsTimeSinceVSync_Bool: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
        break;
    case vr::Prop_IsOnDesktop_Bool: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
        break;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

float OSVRTrackedDevice::GetFloatTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const float default_value = 0.0f;

    if (isWrongDataType(prop, float())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

    return GetFloatProperty(prop, error);
}

int32_t OSVRTrackedDevice::GetInt32TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const int32_t default_value = 0;

    if (isWrongDataType(prop, int32_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_Axis0Type_Int32: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_Axis1Type_Int32: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_Axis2Type_Int32: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_Axis3Type_Int32: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_Axis4Type_Int32: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

uint64_t OSVRTrackedDevice::GetUint64TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const uint64_t default_value = 0;

    if (isWrongDataType(prop, uint64_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_CurrentUniverseId_Uint64: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_PreviousUniverseId_Uint64: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_SupportedButtons_Uint64: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

vr::HmdMatrix34_t OSVRTrackedDevice::GetMatrix34TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    // Default value is identity matrix
    vr::HmdMatrix34_t default_value;
    map(default_value) = Matrix34f::Identity();

    if (isWrongDataType(prop, vr::HmdMatrix34_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_StatusDisplayTransform_Matrix34: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

uint32_t OSVRTrackedDevice::GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, char* value, uint32_t buffer_size, vr::ETrackedPropertyError* error)
{
    const uint32_t default_value = 0;

    if (isWrongDataType(prop, value)) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_TrackingSystemName_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_ModelNumber_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_SerialNumber_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_RenderModelName_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_ManufacturerName_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingFirmwareVersion_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_HardwareRevision_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_DisplayFirmwareVersion_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_AttachedDeviceId_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_AllWirelessDongleDescriptions_String:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_ConnectedWirelessDongle_String:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

vr::VRControllerState_t OSVRTrackedDevice::GetControllerState()
{
    // TODO
    vr::VRControllerState_t controller_state = vr::VRControllerState_t();

#if 0
    // If packet num matches that on your prior call, then the controller state hasn't been changed since
    // your last call and there is no need to process it
    uint32_t unPacketNum;

    // bit flags for each of the buttons. Use ButtonMaskFromId to turn an ID into a mask
    uint64_t ulButtonPressed;
    uint64_t ulButtonTouched;

    // Axis data for the controller's analog inputs
    VRControllerAxis_t rAxis[k_unControllerStateAxisCount];

    /** contains information about one axis on the controller */
    struct VRControllerAxis_t
    {
        float x; // Ranges from -1.0 to 1.0 for joysticks and track pads. Ranges from 0.0 to 1.0 for triggers were 0 is fully released.
        float y; // Ranges from -1.0 to 1.0 for joysticks and track pads. Is always 0.0 for triggers.
    };
    /** the number of axes in the controller state */
    static const uint32_t k_unControllerStateAxisCount = 5;
#endif

    return controller_state;
}

bool OSVRTrackedDevice::TriggerHapticPulse(uint32_t axis_id, uint16_t pulse_duration_microseconds)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}

bool OSVRTrackedDevice::HasCamera()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::GetCameraFirmwareDescription(char *pBuffer, uint32_t nBufferLen)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::GetCameraFrameDimensions(vr::ECameraVideoStreamFormat nVideoStreamFormat, uint32_t *pWidth, uint32_t *pHeight)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::GetCameraFrameBufferingRequirements(int *pDefaultFrameQueueSize, uint32_t *pFrameBufferDataSize)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::SetCameraFrameBuffering(int nFrameBufferCount, void **ppFrameBuffers, uint32_t nFrameBufferDataSize)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat nVideoStreamFormat)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
vr::ECameraVideoStreamFormat OSVRTrackedDevice::GetCameraVideoStreamFormat()
{
    /// @todo SteamVR drivers return 0 and do nothing else, doing the same here
    return vr::CVS_FORMAT_UNKNOWN; // this maps to 0
}
bool OSVRTrackedDevice::StartVideoStream()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
void OSVRTrackedDevice::StopVideoStream()
{
    /// @todo SteamVR drivers does nothing, doing the same here
}
bool OSVRTrackedDevice::IsVideoStreamActive()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
float OSVRTrackedDevice::GetVideoStreamElapsedTime()
{
    /// @todo SteamVR drivers return 0 and do nothing else, doing the same here
    return 0;
}
const vr::CameraVideoStreamFrame_t * OSVRTrackedDevice::GetVideoStreamFrame()
{
    /// @todo SteamVR drivers return NULL and do nothing else, doing the same here
    return NULL;
}
void OSVRTrackedDevice::ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *pFrameImage)
{
    /// @todo SteamVR drivers does nothing, doing the same here
}
bool OSVRTrackedDevice::SetAutoExposure(bool bEnable)
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::PauseVideoStream()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::ResumeVideoStream()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}
bool OSVRTrackedDevice::IsVideoStreamPaused()
{
    /// @todo SteamVR drivers return false and do nothing else, doing the same here
    return false;
}

bool OSVRTrackedDevice::GetCameraDistortion(float flInputU, float flInputV, float *pflOutputU, float *pflOutputV)
{
    return false;
}
bool OSVRTrackedDevice::GetCameraProjection(float flWidthPixels, float flHeightPixels, float flZNear, float flZFar, vr::HmdMatrix44_t *pProjection)
{
    return false;
}
bool OSVRTrackedDevice::GetRecommendedCameraUndistortion(uint32_t *pUndistortionWidthPixels, uint32_t *pUndistortionHeightPixels)
{
    return false;
}
bool OSVRTrackedDevice::SetCameraUndistortion(uint32_t nUndistortionWidthPixels, uint32_t nUndistortionHeightPixels)
{
    return false;
}
bool OSVRTrackedDevice::GetCameraFirmwareVersion(uint64_t *pFirmwareVersion)
{
    return false;
}
bool OSVRTrackedDevice::SetFrameRate(int nISPFrameRate, int nSensorFrameRate)
{
    return false;
}
