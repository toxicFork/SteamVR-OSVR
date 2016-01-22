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

#include <headers/osvr_hmd_device.h>

OSVRHMDDevice::OSVRHMDDevice(const std::string& display_description, osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::IDriverLog* driver_log) :
    OSVRTrackedDevice(display_description, context, driver_host, vr::TrackedDeviceClass_HMD, driver_log)
{
}

const char* OSVRHMDDevice::GetId()
{
    /// @todo When available, return the actual unique ID of the HMD
    return "OSVR HMD";
}

void OSVRHMDDevice::GetWindowBounds(int32_t* x, int32_t* y, uint32_t* width, uint32_t* height)
{
    int nDisplays = m_DisplayConfig.getNumDisplayInputs();
    if (nDisplays != 1) {
        logger_->Log("OSVRHMDDevice::OSVRHMDDevice(): Unexpected display number of displays!\n");
    }
    osvr::clientkit::DisplayDimensions displayDims = m_DisplayConfig.getDisplayDimensions(0);
    *x = m_RenderManagerConfig.getWindowXPosition(); // todo: assumes desktop display of 1920. get this from display config when it's exposed.
    *y = m_RenderManagerConfig.getWindowYPosition();
    *width = displayDims.width;
    *height = displayDims.height;
}

bool OSVRHMDDevice::IsDisplayOnDesktop()
{
    // TODO get this info from display description?
    return true;
}

bool OSVRHMDDevice::IsDisplayRealDisplay()
{
    // TODO get this info from display description?
    return true;
}

void OSVRHMDDevice::GetRecommendedRenderTargetSize(uint32_t* width, uint32_t* height)
{
    /// @todo calculate overfill factor properly
    double overfillFactor = 1.0;
    int32_t x, y;
    uint32_t w, h;
    GetWindowBounds(&x, &y, &w, &h);
    *width = static_cast<uint32_t>(w * overfillFactor);
    *height = static_cast<uint32_t>(h * overfillFactor);
}

void OSVRHMDDevice::GetEyeOutputViewport(vr::EVREye eye, uint32_t* x, uint32_t* y, uint32_t* width, uint32_t* height)
{
    osvr::clientkit::RelativeViewport viewPort = m_DisplayConfig.getViewer(0).getEye(eye).getSurface(0).getRelativeViewport();
    *x = viewPort.left;
    *y = viewPort.bottom;
    *width = viewPort.width;
    *height = viewPort.height;
}

void OSVRHMDDevice::GetProjectionRaw(vr::EVREye eye, float* left, float* right, float* top, float* bottom)
{
    // Reference: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetProjectionRaw
    // SteamVR expects top and bottom to be swapped!
    osvr::clientkit::ProjectionClippingPlanes pl = m_DisplayConfig.getViewer(0).getEye(eye).getSurface(0).getProjectionClippingPlanes();
    *left = static_cast<float>(pl.left);
    *right = static_cast<float>(pl.right);
    *bottom = static_cast<float>(pl.top); // SWAPPED
    *top = static_cast<float>(pl.bottom); // SWAPPED
}

vr::DistortionCoordinates_t OSVRHMDDevice::ComputeDistortion(vr::EVREye eye, float u, float v)
{
    /// @todo FIXME Compute distortion using display configuration data
    vr::DistortionCoordinates_t coords;
    coords.rfRed[0] = u;
    coords.rfRed[1] = v;
    coords.rfBlue[0] = u;
    coords.rfBlue[1] = v;
    coords.rfGreen[0] = u;
    coords.rfGreen[1] = v;
    return coords;
}

vr::TrackedDeviceDriverInfo_t OSVRHMDDevice::GetTrackedDeviceDriverInfo()
{
    vr::TrackedDeviceDriverInfo_t info = OSVRTrackedDevice::GetTrackedDeviceDriverInfo();

    info.bDeviceIsConnected = true;           // false if user unplugs device
    info.bWillDriftInYaw = true;              // true if gyro-only tracking system
    info.bReportsTimeSinceVSync = false;
    info.fSecondsFromVsyncToPhotons = 0.0; // seconds between vsync and photons hitting wearer's eyes
    info.fDisplayFrequency = 60.0;         // fps of display

    return info;
}

const char* OSVRHMDDevice::GetModelNumber()
{
    /// @todo When available, return the actual model number of the HMD
    return "OSVR HMD";
}

const char* OSVRHMDDevice::GetSerialNumber()
{
    /// @todo When available, return the actual serial number of the HMD
    return "0";
}

vr::EVRInitError OSVRHMDDevice::Activate(uint32_t object_id)
{
    vr::EVRInitError baseActivate = OSVRTrackedDevice::Activate(object_id);

    if(baseActivate != vr::VRInitError_None)
    {
        return baseActivate;
    }

    const std::time_t waitTime = 5; // wait up to 5 seconds for init
                                    // Register tracker callback

    m_DisplayConfig = osvr::clientkit::DisplayConfig(m_Context);

    // ensure display is fully started up
    logger_->Log("Waiting for the display to fully start up, including receiving initial pose update...\n");
    std::time_t startTime = std::time(nullptr);
    while (!m_DisplayConfig.checkStartup()) {
        m_Context.update();
        if (std::time(nullptr) > startTime + waitTime) {
            logger_->Log("Display startup timed out!\n");
            return vr::VRInitError_Driver_Failed;
        }
    }

    // verify valid display config
    if ((m_DisplayConfig.getNumViewers() != 1) && (m_DisplayConfig.getViewer(0).getNumEyes() != 2) && (m_DisplayConfig.getViewer(0).getEye(0).getNumSurfaces() == 1) && (m_DisplayConfig.getViewer(0).getEye(1).getNumSurfaces() != 1)) {
        logger_->Log("OSVRTrackedDevice::OSVRTrackedDevice(): Unexpected display parameters!\n");
        if (m_DisplayConfig.getNumViewers() < 1) {
            logger_->Log("OSVRTrackedDevice::OSVRTrackedDevice(): At least one viewer must exist.\n");
            return vr::VRInitError_Driver_HmdDisplayNotFound;
        }
        else if (m_DisplayConfig.getViewer(0).getNumEyes() < 2) {
            logger_->Log("OSVRTrackedDevice::OSVRTrackedDevice(): At least two eyes must exist.\n");
            return vr::VRInitError_Driver_HmdDisplayNotFound;
        }
        else if ((m_DisplayConfig.getViewer(0).getEye(0).getNumSurfaces() < 1) || (m_DisplayConfig.getViewer(0).getEye(1).getNumSurfaces() < 1)) {
            logger_->Log("OSVRTrackedDevice::OSVRTrackedDevice(): At least one surface must exist for each eye.\n");
            return vr::VRInitError_Driver_HmdDisplayNotFound;
        }
    }

    // register tracker callback
    m_TrackerInterface = m_Context.getInterface("/me/head");
    m_TrackerInterface.registerCallback(&OSVRHMDDevice::HmdTrackerCallback, this);

    auto const configString =
        m_Context.getStringParameter("/renderManagerConfig");

    m_RenderManagerConfig.parse(configString);

    return vr::VRInitError_None;
}

void OSVRHMDDevice::Deactivate()
{
    OSVRTrackedDevice::Deactivate();
}

float OSVRHMDDevice::GetIPD()
{
    OSVR_Pose3 leftEye, rightEye;
    if (m_DisplayConfig.getViewer(0).getEye(0).getPose(leftEye) != true) {
        logger_->Log("OSVRHMDDevice::GetHeadFromEyePose(): Unable to get left eye pose!\n");
    }
    if (m_DisplayConfig.getViewer(0).getEye(1).getPose(rightEye) != true) {
        logger_->Log("OSVRHMDDevice::GetHeadFromEyePose(): Unable to get right eye pose!\n");
    }
    return static_cast<float>((osvr::util::vecMap(leftEye.translation) - osvr::util::vecMap(rightEye.translation)).norm());
}

void OSVRHMDDevice::HmdTrackerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_PoseReport* report)
{
    if (!userdata)
        return;

    auto* self = static_cast<OSVRHMDDevice*>(userdata);

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
    self->driver_host_->TrackedDevicePoseUpdated(0, self->pose_); /// @fixme figure out ID correctly, don't hardcode to zero
}

float OSVRHMDDevice::GetFloatProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
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
            *error = vr::TrackedProp_Success;
        return GetIPD();
    case vr::Prop_FieldOfViewLeftDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewRightDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewTopDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewBottomDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingRangeMinimumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingRangeMaximumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
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