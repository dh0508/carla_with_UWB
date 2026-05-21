// CARLA UWB sensor extension
// https://carla.org
//
// Author: dh0508 (GitHub: https://github.com/dh0508)
// Date: 2026
//
// Added UWB sensor to CARLA

#pragma once

#include "carla/sensor/s11n/UWBSerializer.h"
#include "carla/sensor/SensorData.h"

#include <cstdint>
#include <string>
#include <vector>

namespace carla {
namespace sensor {
namespace data {

  /// Immutable view of one UWB ranging result from a remote anchor.
  class UWBDetectionView {
  public:

    UWBDetectionView() = default;

    explicit UWBDetectionView(const s11n::UWBDetection &d)
      : _anchor_id(d.anchor_id),
        _distance(d.distance),
        _payload(d.payload) {}

    uint64_t           GetAnchorId() const { return _anchor_id; }
    float              GetDistance() const { return _distance; }
    const std::string &GetPayload()  const { return _payload; }

  private:

    uint64_t    _anchor_id = 0u;
    float       _distance  = 0.0f;
    std::string _payload;
  };

  /// Measurement produced by the UWB sensor each tick.
  /// Contains one UWBDetectionView per anchor detected within range.
  class UWBMeasurement : public SensorData {
  protected:

    using Super      = SensorData;
    using Serializer = s11n::UWBSerializer;

    friend Serializer;

    explicit UWBMeasurement(const RawData &data)
      : Super(data)
    {
      auto raw = Serializer::DeserializeRawData(data);
      _detections.reserve(raw.detections.size());
      for (const auto &d : raw.detections)
        _detections.emplace_back(d);
    }

  public:

    using value_type     = UWBDetectionView;
    using iterator       = std::vector<UWBDetectionView>::const_iterator;
    using const_iterator = std::vector<UWBDetectionView>::const_iterator;

    size_t         size()  const { return _detections.size(); }
    const_iterator begin() const { return _detections.begin(); }
    const_iterator end()   const { return _detections.end(); }

    const UWBDetectionView &at(size_t i) const { return _detections.at(i); }

    size_t GetDetectionCount() const { return _detections.size(); }

  private:

    std::vector<UWBDetectionView> _detections;
  };

} // namespace data
} // namespace sensor
} // namespace carla
