// CARLA UWB sensor extension
// https://carla.org
//
// Author: dh0508 (GitHub: https://github.com/dh0508)
// Date: 2026
//
// Added UWB sensor to CARLA

#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "UWBSensor.generated.h"

/// UWB (Ultra-Wideband) sensor.
///
/// Each tick the sensor finds all other AUWBSensor actors within MaxRange
/// metres, computes the ToF distance, and reports the (anchor_id, distance,
/// payload) triplets to the client listen() callback.
///
/// The payload this sensor broadcasts is set by calling
/// sensor.send_uwb_payload(obj) from Python, where obj is any JSON-serialisable
/// object.  The sensor handles ranging and delivery independently of V2X.
UCLASS()
class CARLA_API AUWBSensor : public ASensor
{
  GENERATED_BODY()

public:

  AUWBSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  // ---- Configuration -------------------------------------------------------

  void SetMaxRange(float Range)        { MaxRange = Range; }
  void SetLOSStdDev(float S)           { LOSStdDev = S; }
  void SetNLOSBiasMin(float B)         { NLOSBiasMin = B; }
  void SetNLOSBiasMax(float B)         { NLOSBiasMax = B; }
  void SetNLOSStdDev(float S)          { NLOSStdDev = S; }

  float GetMaxRange()    const { return MaxRange; }
  float GetLOSStdDev()   const { return LOSStdDev; }
  float GetNLOSBiasMin() const { return NLOSBiasMin; }
  float GetNLOSBiasMax() const { return NLOSBiasMax; }
  float GetNLOSStdDev()  const { return NLOSStdDev; }

  // ---- Payload (called from CarlaServer when client calls sensor.send_uwb_payload()) ---

  void SetPayload(std::string Payload);

protected:

  static constexpr float CM_TO_M = 1e-2f;

  float MaxRange    = 100.0f;
  float LOSStdDev   = 0.05f;   // σ_LOS  [m]
  float NLOSBiasMin = 0.2f;    // b_NLOS min [m]
  float NLOSBiasMax = 2.0f;    // b_NLOS max [m]
  float NLOSStdDev  = 0.3f;    // σ_NLOS [m]

private:

  // Per-sensor payload storage — keyed by the AUWBSensor pointer.
  // Written by Send() (RPC thread) and read by PostPhysTick() (game thread).
  static std::mutex                               sPayloadMutex;
  static std::unordered_map<AUWBSensor*, std::string> sPayloadMap;
};
