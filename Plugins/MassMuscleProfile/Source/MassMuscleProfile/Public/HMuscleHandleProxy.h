#pragma once
#include "HitProxies.h"

enum class EMuscleHandle : uint8 { Min, Max };

struct HMuscleHandleProxy : public HHitProxy
{
    DECLARE_HIT_PROXY()

    EMuscleHandle HandleType;

    HMuscleHandleProxy(EMuscleHandle InType)
        : HHitProxy(HPP_Foreground)
        , HandleType(InType)
    {}
};

struct HMassMuscleRotationHandle : public HHitProxy
{
    DECLARE_HIT_PROXY();

    EAxis::Type Axis;

    HMassMuscleRotationHandle(EAxis::Type InAxis)
        : HHitProxy(HPP_Foreground)
        , Axis(InAxis)
    {}
};