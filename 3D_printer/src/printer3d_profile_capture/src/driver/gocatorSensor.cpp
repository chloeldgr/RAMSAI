// Copyright 2023 ICube Laboratory, University of Strasbourg
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

#include "gocatorSensor.h"

GocatorSensor::Device::Device(const std::string & _ip_address)
{
  kStatus status;
  kIpAddress ipAddress;
  kChar model_name[50];

  // init all GO API objects
  go_api_ = kNULL;
  go_system_ = kNULL;
  go_sensor_ = kNULL;
  go_setup_ = kNULL;
  go_dataset_ = kNULL;
  go_stamp_ptr_ = kNULL;

  // construct Gocator API Library
  if ((status = GoSdk_Construct(&go_api_)) != kOK) {
    std::cout << "Device(). Error: GoSdk_Construct: " << status << std::endl;
    status_ = DEVICE_NOT_FOUND;
    return;
  }

  // construct GoSystem object
  if ((status = GoSystem_Construct(&go_system_, kNULL)) != kOK) {
    std::cout << "Device(). Error: GoSystem_Construct: " << status << std::endl;
    status_ = DEVICE_NOT_FOUND;
    return;
  }

  // obtain GoSensor object by sensor IP address
  kIpAddress_Parse(&ipAddress, _ip_address.c_str());
  if ((status = GoSystem_FindSensorByIpAddress(go_system_, &ipAddress, &go_sensor_)) != kOK) {
    std::cout << "Device(). Error: GoSystem_FindSensorByIpAddress: " << status << std::endl;
    status_ = DEVICE_NOT_FOUND;
    return;
  }

  // Success case. Set status and device fixed params (ip, model name and serial number ).
  status_ = DEVICE_FOUND;
  device_params_.ip_address_ = _ip_address;

  // create connection to GoSensor object
  if ((status = GoSensor_Connect(go_sensor_)) != kOK) {
    std::cout << "Device(). Error: GoSensor_Connect: " << status << std::endl;
    status_ = DEVICE_NOT_CONNECT;
    return;
  }
  status_ = DEVICE_CONNECT;

  // enable sensor data channel
  if ((status = GoSystem_EnableData(go_system_, kTRUE)) != kOK) {
    std::cout << "Device(). Error: GoSensor_EnableData: " << status << std::endl;
    return;
  }

  // retrieve setup handle
  if ((go_setup_ = GoSensor_Setup(go_sensor_)) == kNULL) {
    std::cout << "Device(). Error: GoSensor_Setup: Invalid Handle" << std::endl;
    return;
  }

  // Obtain camera model
  if ((status = GoSensor_Model(go_sensor_, model_name, 50)) != kOK) {
    std::cout << "Device(). Error: GoSensor_Model: " << status << std::endl;
    return;
  }
  device_params_.model_name_ = model_name;

  // Obtain camera Serial number
  device_params_.sn_ = (unsigned int)GoSensor_Id(go_sensor_);

  // Obtain exposure
  capture_params_.exposure_time_ = GoSetup_Exposure(go_setup_, GO_ROLE_MAIN);

  // Obtain spacing interval
  capture_params_.spacing_interval_ = GoSetup_SpacingInterval(go_setup_, GO_ROLE_MAIN);

  // print info
  std::cout << "Found Sensor: " << std::endl;
  device_params_.print();
}

GocatorSensor::Device::~Device()
{
  kStatus status;

  // destroy handles
  GoDestroy(go_system_);
  GoDestroy(go_api_);

  // bye bye message
  std::cout << "~Device(). Gocator Sensor Stopped and Device Object Destroyed." << std::endl;
}

int GocatorSensor::Device::configure(const CaptureParams & _configs)
{
  kStatus status;

  // set exposure
  if ((status = GoSetup_SetExposure(go_setup_, GO_ROLE_MAIN, _configs.exposure_time_)) != kOK) {
    std::cout << "configure(): Error setting Exposure Time to " << _configs.exposure_time_ <<
      std::endl;
    return -1;
  }

  // set spacing interval
  if ((status =
    GoSetup_SetSpacingInterval(go_setup_, GO_ROLE_MAIN, _configs.spacing_interval_)) != kOK)
  {
    std::cout << "configure(): Error setting Spacing Interval to " << _configs.spacing_interval_ <<
      std::endl;
    return -1;
  }

  // set this->capture_params_ with true values from camera
  capture_params_.exposure_time_ = GoSetup_Exposure(go_setup_, GO_ROLE_MAIN);
  capture_params_.spacing_interval_ = GoSetup_SpacingInterval(go_setup_, GO_ROLE_MAIN);

  // print
  std::cout << "Configuration Settings: " << std::endl;
  capture_params_.print();

  // return
  return 1;
}

int GocatorSensor::Device::start()
{
  kStatus status;

  // start Gocator sensor
  if ((status = GoSystem_Start(go_system_)) != kOK) {
    std::cout << "Device(). Error: GoSystem_Start: " << status << std::endl;
    return -1;
  }

  // message to std out
  // std::cout << "Gocator running ... " << std::endl;

  // set this->status_
  this->status_ = DEVICE_RUNNING;

  // return success
  return 1;
}

int GocatorSensor::Device::stop()
{
  kStatus status;

  // stop Gocator sensor
  if ((status = GoSystem_Stop(go_system_)) != kOK) {
    std::cout << "~Device(). Error: GoSystem_Stop: " << status << std::endl;
    return -1;
  }

  // message to std out
  // std::cout << "... Gocator stopped" << std::endl << std::endl;

  // set this->status_
  this->status_ = DEVICE_CONNECT;

  // return success
  return 1;
}

int GocatorSensor::Device::getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{
}

int GocatorSensor::Device::getSingleSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{
}

int GocatorSensor::Device::getProfile(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{
  unsigned int i, j, k, arrayIndex;
  GoDataSet dataset = kNULL;
  std::vector<ProfilePoint> profileBuffer;
  GoStamp * stamp = kNULL;
  GoDataMsg dataObj;
  k32u profilePointCount;
  if (GoSystem_ReceiveData(go_system_, &dataset, RECEIVE_TIMEOUT) == kOK) {
    printf("Data message received:\n");
    printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
    // each result can have multiple data items
    // loop through all items in result message
    for (i = 0; i < GoDataSet_Count(dataset); ++i) {
      dataObj = GoDataSet_At(dataset, i);
      // Retrieve GoStamp message
      switch (GoDataMsg_Type(dataObj)) {
        case GO_DATA_MESSAGE_TYPE_STAMP:
          {
            GoStampMsg stampMsg = dataObj;

            printf("Stamp Message batch count: %u\n", (k32u)GoStampMsg_Count(stampMsg));
            for (j = 0; j < GoStampMsg_Count(stampMsg); ++j) {
              stamp = GoStampMsg_At(stampMsg, j);
              printf("  Timestamp: %llu\n", stamp->timestamp);
              printf("  Encoder: %lld\n", stamp->encoder);
              printf("  Frame index: %llu\n", stamp->frameIndex);
            }
          }
          break;
        case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:
          {
            GoResampledProfileMsg profileMsg = dataObj;

            printf(
              "Resampled Profile Message batch count: %u\n",
              (k32u)GoResampledProfileMsg_Count(profileMsg));

            _p_cloud.height = GoResampledProfileMsg_Count(profileMsg);
            _p_cloud.width = GoResampledProfileMsg_Width(profileMsg);
            _p_cloud.resize(_p_cloud.height * _p_cloud.width);

            for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k) {
              unsigned int validPointCount = 0;
              short * data = GoResampledProfileMsg_At(profileMsg, k);
              double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
              double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
              double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
              double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));

              // profileBuffer.resize(GoResampledProfileMsg_Width(profileMsg));


              // translate 16-bit range data to engineering units and copy profiles to memory array
              for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg);
                ++arrayIndex)
              {
                if (data[arrayIndex] != INVALID_RANGE_16BIT) {
                  // profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                  // profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).x = XOffset + XResolution *
                    arrayIndex;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).z = ZOffset + ZResolution *
                    data[arrayIndex];
                  validPointCount++;
                } else {
                  // profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                  // profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).x = XOffset + XResolution *
                    arrayIndex;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).z = INVALID_RANGE_DOUBLE;
                }
              }
              printf(
                "  Profile Valid Point %d out of max %d\n", validPointCount,
                profilePointCount);
            }
          }
          break;
        case GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD:     // Note this is NON resampled profile
          {
            GoProfileMsg profileMsg = dataObj;

            printf("Profile Message batch count: %u\n", (k32u)GoProfileMsg_Count(profileMsg));

            _p_cloud.height = GoProfileMsg_Count(profileMsg);
            _p_cloud.width = GoProfileMsg_Width(profileMsg);
            _p_cloud.resize(_p_cloud.height * _p_cloud.width);

            for (k = 0; k < GoProfileMsg_Count(profileMsg); ++k) {
              kPoint16s * data = GoProfileMsg_At(profileMsg, k);
              unsigned int validPointCount = 0;
              double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
              double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
              double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
              double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));

              // profileBuffer.resize(GoResampledProfileMsg_Width(profileMsg));

              // translate 16-bit range data to engineering units and copy profiles to memory array
              for (arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex) {
                if (data[arrayIndex].x != INVALID_RANGE_16BIT) {
                  // profileBuffer[arrayIndex].x = XOffset + XResolution * data[arrayIndex].x;
                  // profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).x = XOffset + XResolution *
                    data[arrayIndex].x;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).z = ZOffset + ZResolution *
                    data[arrayIndex].y;
                  validPointCount++;
                } else {
                  // profileBuffer[arrayIndex].x = INVALID_RANGE_DOUBLE;
                  // profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).x = INVALID_RANGE_DOUBLE;
                  _p_cloud.points.at(k * _p_cloud.width + arrayIndex).z = INVALID_RANGE_DOUBLE;
                }
              }
              printf(
                "  Profile Valid Point %d out of max %d\n", validPointCount,
                profilePointCount);
            }
          }
          break;
        case GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY:
          {
            unsigned int validPointCount = 0;
            GoProfileIntensityMsg intensityMsg = dataObj;
            printf(
              "Intensity Message batch count: %u\n",
              (k32u)GoProfileIntensityMsg_Count(intensityMsg));

            profileBuffer.resize(GoProfileIntensityMsg_Count(intensityMsg));

            for (k = 0; k < GoProfileIntensityMsg_Count(intensityMsg); ++k) {
              unsigned char * data = GoProfileIntensityMsg_At(intensityMsg, k);
              for (arrayIndex = 0; arrayIndex < GoProfileIntensityMsg_Width(intensityMsg);
                ++arrayIndex)
              {
                profileBuffer[arrayIndex].intensity = data[arrayIndex];
              }
            }
          }
          break;
      }
    }
    GoDestroy(dataset);
  } else {
    printf("Error: No data received during the waiting period\n");
  }
  return 1;
}

void GocatorSensor::Device::getDeviceHealth(std::string & _health_str) const
{
  // local variables
  GoDataSet health_data = kNULL;
  GoHealthMsg health_msg = kNULL;
  GoIndicator * health_indicator = kNULL;
  std::ostringstream sstr;

  // get health from device
  if ( (GoSystem_ReceiveHealth(go_system_, &health_data, RECEIVE_TIMEOUT)) == kOK) {
    for (unsigned int ii = 0; ii < GoDataSet_Count(health_data); ii++) {
      health_msg = GoDataSet_At(health_data, ii);
      for (unsigned int jj = 0; jj < GoHealthMsg_Count(health_msg); jj++) {
        health_indicator = GoHealthMsg_At(health_msg, jj);
        sstr << "Indicator[" << jj << "]:\n"
             << "\tId: " << health_indicator->id << "\n"
             << "\tInstance: " << health_indicator->instance << "\n"
             << "\tValue: " << health_indicator->value << "\n";
      }
    }
    GoDestroy(health_msg);
  }

  _health_str = sstr.str();
}

void GocatorSensor::Device::getTemperature(
  double & _internal_temp, double & _projector_temp,
  double & _laser_temp) const
{
  // local variables
  GoDataSet health_data = kNULL;
  GoHealthMsg health_msg = kNULL;
  GoIndicator * health_indicator = kNULL;
  // k32u instance;

  // get health dataset from device
  if ( (GoSystem_ReceiveHealth(go_system_, &health_data, RECEIVE_TIMEOUT)) == kOK) {
    for (unsigned int ii = 0; ii < GoDataSet_Count(health_data); ii++) {
      // get the health message
      health_msg = GoDataSet_At(health_data, ii);

      // find in the message the internal temperature indicator, and set the value
      health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_TEMPERATURE, 0);
      if (health_indicator != kNULL) {_internal_temp = health_indicator->value;} else {
        _internal_temp = -100.;
      }

      // find in the message the projector temperature indicator, and set the value
      health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_PROJECTOR_TEMPERATURE, 0);
      if (health_indicator != kNULL) {_projector_temp = health_indicator->value;} else {
        _projector_temp = -100.;
      }

      // find in the message the projector temperature indicator, and set the value
      health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_LASER_TEMPERATURE, 0);
      if (health_indicator != kNULL) {_laser_temp = health_indicator->value;} else {
        _laser_temp = -100.;
      }
    }
    GoDestroy(health_msg);
  }
}

int GocatorSensor::Device::close()
{
}

void GocatorSensor::Device::printDeviceData() const
{
}

void GocatorSensor::Device::sendTrigger() const
{
  printf("sending trigger \n");
  kStatus status = GoSensor_Trigger(go_sensor_);
  if (status != kOK) {
    printf("Error: GoSensor_Connect:%d\n", status);
    return;
  }
}
