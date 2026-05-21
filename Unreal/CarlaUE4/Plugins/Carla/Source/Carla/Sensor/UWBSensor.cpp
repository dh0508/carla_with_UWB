// CARLA UWB sensor extension
// https://carla.org
//
// Author: dh0508 (GitHub: https://github.com/dh0508)
// Date: 2026
//
// Added UWB sensor to CARLA

#include "Carla.h"
#include "Carla/Sensor/UWBSensor.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "EngineUtils.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/sensor/s11n/UWBSerializer.h"
#include <compiler/enable-ue4-macros.h>

// Static member definitions
std::mutex                                    AUWBSensor::sPayloadMutex;
std::unordered_map<AUWBSensor*, std::string>  AUWBSensor::sPayloadMap;

// =============================================================================

AUWBSensor::AUWBSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PostPhysics;
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
}

FActorDefinition AUWBSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeUWBDefinition();
}

void AUWBSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetUWB(ActorDescription, this);
}

// =============================================================================
// -- Send (called from CarlaServer on RPC thread) -----------------------------
// =============================================================================

void AUWBSensor::SetPayload(std::string Payload)
{
  std::lock_guard<std::mutex> Lock(sPayloadMutex);
  sPayloadMap[this] = std::move(Payload);
}

// =============================================================================
// -- Tick ---------------------------------------------------------------------
// =============================================================================

void AUWBSensor::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AUWBSensor::PostPhysTick);

  const float   RangeCm = MaxRange / CM_TO_M;
  const FVector SelfLoc = GetActorLocation();

  std::vector<carla::sensor::s11n::UWBDetection> Detections;

  for (TActorIterator<AUWBSensor> It(World); It; ++It)
  {
    AUWBSensor *Other = *It;
    if (Other == this || !IsValid(Other))
      continue;

    const float DistCm = FVector::Dist(SelfLoc, Other->GetActorLocation());
    if (DistCm > RangeCm)
      continue;

    float DistM = DistCm * CM_TO_M;

    // LOS/NLOS 판단: 두 센서 사이에 장애물이 있으면 NLOS
    FHitResult Hit;
    FCollisionQueryParams QueryParams(TEXT("UWBLOSCheck"), true);
    QueryParams.AddIgnoredActor(this);
    QueryParams.AddIgnoredActor(Other);
    const bool bNLOS = World->LineTraceSingleByChannel(
        Hit, SelfLoc, Other->GetActorLocation(),
        ECC_Visibility, QueryParams);

    if (bNLOS)
    {
      // NLOS: 양의 바이어스 + 큰 노이즈
      const float Bias = RandomEngine->GetUniformFloatInRange(NLOSBiasMin, NLOSBiasMax);
      DistM += Bias + RandomEngine->GetNormalDistribution(0.0f, NLOSStdDev);
    }
    else
    {
      // LOS: 제로-평균 가우시안
      DistM += RandomEngine->GetNormalDistribution(0.0f, LOSStdDev);
    }

    carla::sensor::s11n::UWBDetection Det;
    AActor *OtherOwner = Other->GetOwner() ? Other->GetOwner() : Other;
    Det.anchor_id = static_cast<uint64_t>(OtherOwner->GetUniqueID());
    Det.distance  = DistM;

    // Look up the payload the remote sensor has registered via Send()
    {
      std::lock_guard<std::mutex> Lock(sPayloadMutex);
      auto Found = sPayloadMap.find(Other);
      if (Found != sPayloadMap.end())
        Det.payload = Found->second;
    }

    Detections.push_back(std::move(Det));
  }

  auto Stream = GetDataStream(*this);
  Stream.SerializeAndSend(*this, std::move(Detections));
}
