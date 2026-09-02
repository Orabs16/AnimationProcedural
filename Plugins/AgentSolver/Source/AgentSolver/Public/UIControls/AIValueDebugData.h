#pragma once

// Plain data record for one bipolar gauge in the AI debug window (see
// SAgentSolverAIDebugPanel.h) -- one AI observation (Input) or action
// (Output) component. Deliberately not a UStruct: this is per-tick,
// game-thread-only display data (built fresh every Tick(), never
// serialized, never crossing a UObject boundary), same treatment
// SAgentSolverLineGraph's own float samples get.

#include "CoreMinimal.h"

/** Which side of the debug window a value belongs on. */
enum class EAIValueDebugCategory : uint8
{
	Input,
	Output
};

struct FAIValueDebugData
{
	FName VariableName;

	/** Clamped to [-1,1] on construction -- see SAIBipolarGaugeBar, which assumes its Value input is already in this range. */
	float NormalizedValue = 0.0f;

	EAIValueDebugCategory Category = EAIValueDebugCategory::Input;

	FAIValueDebugData() = default;

	FAIValueDebugData(FName InVariableName, float InNormalizedValue, EAIValueDebugCategory InCategory)
		: VariableName(InVariableName)
		, NormalizedValue(FMath::Clamp(InNormalizedValue, -1.0f, 1.0f))
		, Category(InCategory)
	{
	}
};
