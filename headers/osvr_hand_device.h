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

#ifndef INCLUDED_osvr_hand_device_h
#define INCLUDED_osvr_hand_device_h

// Internal Includes
#include "./osvr_tracked_device.h"

class OSVRHandDevice : public OSVRTrackedDevice
{
public:
    OSVRHandDevice(osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::IDriverLog* driver_log = nullptr);

    /**
     * Returns the ID of this particular HMD. This value is opaque to the VR
     * system itself, but should be unique within the driver because it will be
     * passed back in via FindHmd().
     */
    virtual const char* GetId() OSVR_OVERRIDE;
    // -----------------------------------
    // Assorted capability methods
    // -----------------------------------

    /**
     * Returns all the static information that will be reported to the clients.
     */
    virtual vr::TrackedDeviceDriverInfo_t GetTrackedDeviceDriverInfo() OSVR_OVERRIDE;

    // -----------------------------------
    // Administrative Methods
    // -----------------------------------

    /**
     * Returns the model number of this tracked device.
     */
    virtual const char* GetModelNumber() OSVR_OVERRIDE;

    /**
     * Returns the serial number of this tracked device.
     */
    virtual const char* GetSerialNumber() OSVR_OVERRIDE;


    vr::EVRInitError Activate(uint32_t object_id) OSVR_OVERRIDE;
    void Deactivate() OSVR_OVERRIDE;

    vr::VRControllerState_t GetControllerState() override;
protected:
    float GetFloatProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;
private:
    static void HandTrackerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_PoseReport* report);
};

#endif // INCLUDED_osvr_hand_device_h
