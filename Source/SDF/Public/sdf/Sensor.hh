/*
 * Copyright 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef SDF_SENSOR_HH_
#define SDF_SENSOR_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class SensorPrivate;

  /// \enum SensorType
  /// \brief The set of sensor types.
  enum class SensorType
  {
    /// \brief An unspecified sensor type.
    NONE = 0,

    /// \brief An altimeter sensor.
    ALTIMETER = 1,

    /// \brief A monocular camera sensor.
    CAMERA = 2,

    /// \brief A contact sensor.
    CONTACT = 3,

    /// \brief A depth camera sensor.
    DEPTH = 4,

    /// \brief A force-torque sensor.
    FORCE_TORQUE = 5,

    /// \brief A GPS sensor.
    GPS = 6,

    /// \brief A GPU based ray sensor.
    GPU_RAY = 7,

    /// \brief An IMU sensor.
    IMU = 8,

    /// \brief A logical camera sensor.
    LOGICAL_CAMERA = 9,

    /// \brief A magnetometer sensor.
    MAGNETOMETER = 10,

    /// \brief A multicamera sensor.
    MULTICAMERA = 11,

    /// \brief A CPU based ray sensor.
    RAY = 12,

    /// \brief An RFID sensor.
    RFID = 13,

    /// \brief An RFID tag.
    RFIDTAG = 14,

    /// \brief A sonar tag sensor.
    SONAR = 15,

    /// \brief A wireless receiver.
    WIRELESS_RECEIVER = 16,

    /// \brief A wireless transmitter.
    WIRELESS_TRANSMITTER = 17
  };

  /// \brief Information about an SDF sensor.
  class SDFORMAT_VISIBLE Sensor
  {
    /// \brief Default constructor
    public: Sensor();

    /// \brief Move constructor
    /// \param[in] _sensor Sensor to move.
    public: Sensor(Sensor &&_sensor);

    /// \brief Destructor
    public: ~Sensor();

    /// \brief Load the sensor based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the sensor.
    /// The name of the sensor should be unique within the scope of a World.
    /// \return Name of the sensor.
    public: std::string Name() const;

    /// \brief Set the name of the sensor.
    /// The name of the sensor should be unique within the scope of a World.
    /// \param[in] _name Name of the sensor.
    public: void SetName(const std::string &_name);

    /// \brief Get the pose of the sensor. This is the pose of the sensor
    /// as specified in SDF (<sensor> <pose> ... </pose></sensor>), and is
    /// typically used to express the position and rotation of a sensor in a
    /// global coordinate frame.
    /// \return The pose of the sensor.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the sensor.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The new sensor pose.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame in which this sensor's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \return The name of the pose frame.
    public: const std::string &PoseFrame() const;

    /// \brief Set the name of the coordinate frame in which this sensor's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \param[in] _frame The name of the pose frame.
    public: void SetPoseFrame(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the sensor type.
    /// \return The sensor type.
    public: SensorType Type() const;

    /// \brief Set the sensor type.
    /// \param[in] _type The sensor type.
    public: void SetType(const SensorType _type);

    /// \brief Private data pointer.
    private: SensorPrivate *dataPtr = nullptr;
  };
  }
}
#endif
