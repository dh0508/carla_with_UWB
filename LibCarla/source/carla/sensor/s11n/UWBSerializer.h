// CARLA UWB sensor extension
// https://carla.org
//
// Author: dh0508 (GitHub: https://github.com/dh0508)
// Date: 2026
//
// Added UWB sensor to CARLA

#pragma once

#include "carla/Buffer.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"

#include <cstdint>
#include <string>
#include <vector>

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  /// One UWB ranging result received from a remote anchor.
  ///
  /// anchor_id : UE4 unique actor ID of the remote anchor's owner
  /// distance  : ToF-based distance in meters (noise already applied)
  /// payload   : raw bytes the remote anchor sent via sensor.send()
  struct UWBDetection {
    uint64_t    anchor_id = 0u;
    float       distance  = 0.0f;
    std::string payload;

    MSGPACK_DEFINE_ARRAY(anchor_id, distance, payload)
  };

  class UWBSerializer
  {
  public:

    struct Data {
      std::vector<UWBDetection> detections;
      MSGPACK_DEFINE_ARRAY(detections)
    };

    template <typename SensorT>
    static Buffer Serialize(
        const SensorT &sensor,
        std::vector<UWBDetection> detections);

    static Data DeserializeRawData(const RawData &message) {
      return MsgPack::UnPack<Data>(message.begin(), message.size());
    }

    static SharedPtr<SensorData> Deserialize(RawData DESERIALIZE_DECL_DATA(data));
  };

  template <typename SensorT>
  inline Buffer UWBSerializer::Serialize(
      const SensorT &,
      std::vector<UWBDetection> detections)
  {
    return MsgPack::Pack(Data{std::move(detections)});
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
