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

#include <headers/osvr_camera_device.h>

OSVRCameraDevice::OSVRCameraDevice(osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::IDriverLog* driver_log) :
    OSVRTrackedDevice(context, driver_host, vr::TrackedDeviceClass_TrackingReference, driver_log)
{
}

const char* OSVRCameraDevice::GetId()
{
    /// @todo When available, return the actual unique ID of the HMD
    return "OSVR Camera";
}

vr::TrackedDeviceDriverInfo_t OSVRCameraDevice::GetTrackedDeviceDriverInfo()
{
    vr::TrackedDeviceDriverInfo_t info = OSVRTrackedDevice::GetTrackedDeviceDriverInfo();

    info.bDeviceIsConnected = true;           // false if user unplugs device
    info.bWillDriftInYaw = false;              // true if gyro-only tracking system

    return info;
}

const char* OSVRCameraDevice::GetModelNumber()
{
    /// @todo When available, return the actual model number of the HMD
    return "OSVR Camera";
}

const char* OSVRCameraDevice::GetSerialNumber()
{
    /// @todo When available, return the actual serial number of the HMD
    return "0";
}

vr::EVRInitError OSVRCameraDevice::Activate(uint32_t object_id)
{
    vr::EVRInitError baseActivate = OSVRTrackedDevice::Activate(object_id);

    if(baseActivate != vr::VRInitError_None)
    {
        return baseActivate;
    }

    const std::time_t waitTime = 5; // wait up to 5 seconds for init
                                    // Register tracker callback
    // register tracker callback
    m_TrackerInterface = m_Context.getInterface("/me/head");
//    m_TrackerInterface.registerCallback(&OSVRCameraDevice::HandTrackerCallback, this);

    return vr::VRInitError_None;
}

void OSVRCameraDevice::Deactivate()
{
    OSVRTrackedDevice::Deactivate();
}

vr::VRControllerState_t OSVRCameraDevice::GetControllerState()
{
    // TODO
    vr::VRControllerState_t controller_state = vr::VRControllerState_t();
//#if 0
//    // If packet num matches that on your prior call, then the controller state hasn't been changed since
//    // your last call and there is no need to process it
//    uint32_t unPacketNum;
//
//    // bit flags for each of the buttons. Use ButtonMaskFromId to turn an ID into a mask
//    uint64_t ulButtonPressed;
//    uint64_t ulButtonTouched;
//
//    // Axis data for the controller's analog inputs
//    VRControllerAxis_t rAxis[k_unControllerStateAxisCount];
//
//    /** contains information about one axis on the controller */
//    struct VRControllerAxis_t
//    {
//        float x; // Ranges from -1.0 to 1.0 for joysticks and track pads. Ranges from 0.0 to 1.0 for triggers were 0 is fully released.
//        float y; // Ranges from -1.0 to 1.0 for joysticks and track pads. Is always 0.0 for triggers.
//    };
//    /** the number of axes in the controller state */
//    static const uint32_t k_unControllerStateAxisCount = 5;
//#endif

    return controller_state;
}

void OSVRCameraDevice::HandTrackerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_PoseReport* report)
{
    if (!userdata)
        return;

    auto* self = static_cast<OSVRCameraDevice*>(userdata);

    vr::DriverPose_t pose;
    pose.poseTimeOffset = 0; // close enough

    Eigen::Vector3d::Map(pose.vecWorldFromDriverTranslation) = Eigen::Vector3d::Zero();
    Eigen::Vector3d::Map(pose.vecDriverFromHeadTranslation) = Eigen::Vector3d::Zero();

    map(pose.qWorldFromDriverRotation) = Eigen::Quaterniond::Identity();

    map(pose.qDriverFromHeadRotation) = Eigen::Quaterniond::Identity();

    // Position
    Eigen::Vector3d::Map(pose.vecPosition) = osvr::util::vecMap(report->pose.translation);

    // Position velocity and acceleration are not currently consistently provided
    Eigen::Vector3d::Map(pose.vecVelocity) = Eigen::Vector3d::Zero();
    Eigen::Vector3d::Map(pose.vecAcceleration) = Eigen::Vector3d::Zero();

    // Orientation
    map(pose.qRotation) = osvr::util::fromQuat(report->pose.rotation);

    // Angular velocity and acceleration are not currently consistently provided
    Eigen::Vector3d::Map(pose.vecAngularVelocity) = Eigen::Vector3d::Zero();
    Eigen::Vector3d::Map(pose.vecAngularAcceleration) = Eigen::Vector3d::Zero();

    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;
    pose.willDriftInYaw = true;
    pose.shouldApplyHeadModel = true;

    self->pose_ = pose;
    self->driver_host_->TrackedDevicePoseUpdated(2, self->pose_); /// @fixme figure out ID correctly, don't hardcode to zero
}

float OSVRCameraDevice::GetFloatProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    float default_value = 0.0f;
#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_SecondsFromVsyncToPhotons_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_DisplayFrequency_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_UserIpdMeters_Float:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewLeftDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
        return 90.0f;
    case vr::Prop_FieldOfViewRightDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
        return 90.0f;
    case vr::Prop_FieldOfViewTopDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
        return 90.0f;
    case vr::Prop_FieldOfViewBottomDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
        return 90.0f;
    case vr::Prop_TrackingRangeMinimumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return 0.3f;
    case vr::Prop_TrackingRangeMaximumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
        return 90.0f;
    case vr::Prop_DeviceBatteryPercentage_Float: // TODO
    case vr::Prop_DisplayMCOffset_Float: // TODO
    case vr::Prop_DisplayMCScale_Float: // TODO
    case vr::Prop_DisplayGCBlackClamp_Float: // TODO
    case vr::Prop_DisplayGCOffset_Float: // TODO
    case vr::Prop_DisplayGCScale_Float: // TODO
    case vr::Prop_DisplayGCPrescale_Float: // TODO
    case vr::Prop_LensCenterLeftU_Float: // TODO
    case vr::Prop_LensCenterLeftV_Float: // TODO
    case vr::Prop_LensCenterRightU_Float: // TODO
    case vr::Prop_LensCenterRightV_Float: // TODO
    case vr::Prop_UserHeadToEyeDepthMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    default: break;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}
