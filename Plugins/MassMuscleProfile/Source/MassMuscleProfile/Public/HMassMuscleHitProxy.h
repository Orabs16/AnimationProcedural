#pragma once
#include "CoreMinimal.h"
#include "HitProxies.h"

struct HMuscleArcProxy : public HHitProxy
{
    DECLARE_HIT_PROXY();

    FName MuscleName;

    HMuscleArcProxy(FName InMuscleName)
        : HHitProxy(HPP_World)
        , MuscleName(InMuscleName)
    {}
};
struct HBoneLineProxy : public HHitProxy
{
    DECLARE_HIT_PROXY();

    FName BoneName;
    int32 LineIndex;

    HBoneLineProxy(FName InBoneName, int32 InLineIndex)
        : HHitProxy(HPP_World)
        , BoneName(InBoneName)
        , LineIndex(InLineIndex)
    {}
};