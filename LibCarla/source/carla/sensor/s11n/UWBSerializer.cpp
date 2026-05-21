// CARLA UWB sensor extension
// https://carla.org
//
// Author: dh0508 (GitHub: https://github.com/dh0508)
// Date: 2026
//
// Added UWB sensor to CARLA

#include "carla/sensor/s11n/UWBSerializer.h"
#include "carla/sensor/data/UWBMeasurement.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> UWBSerializer::Deserialize(RawData DESERIALIZE_DECL_DATA(data)) {
    return SharedPtr<SensorData>(new data::UWBMeasurement(DESERIALIZE_MOVE_DATA(data)));
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
