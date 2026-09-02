#pragma once

// A saved, swappable bundle of everything one AMutoRLTrainingDriver setup
// needs: its preview environment, rig assets, network Load/Save slots, and
// tuning knobs -- lets you keep several distinct training
// configurations (e.g. "baseline", "high muscle penalty experiment") as
// separate assets and swap between them instead of re-entering a dozen
// values by hand every time.
//
// Double-clicking one of these assets in the Content Browser opens the
// Agent Solver tool with it loaded (see FAgentSolverPresetAssetActions,
// mirrors MassMuscleProfile's own OpenAssetEditor -> OpenToolForAsset
// pattern -- FMassMuscleProfile.cpp). Manually opening the tool (Window
// menu) with no preset already active checks the asset registry and loads
// the first UAgentSolverPreset it finds (see
// FAgentSolverModule::SpawnControlPanelTab). It is also swappable from the
// Viewport Parameters tab (UAgentSolverViewportSettings::ActivePreset).
//
// Live edits to any of the tuning fields below, made through the driver
// while a preset is active, get written back into this asset and mark it
// dirty (see SAgentSolverControlPanel::SyncActivePresetFromDriver) -- but do
// NOT auto-save to disk. Save it the normal way (Ctrl+S / Save All) once
// you're happy with a change.
//
// Not WITH_EDITOR-gated -- like AMutoRLTrainingDriver's own UCLASS/
// UPROPERTY declarations, this is plain data with no Editor-only types in
// it (every referenced class below is forward-declared and only ever
// touched through a TObjectPtr/TSoftObjectPtr, which doesn't need the full
// type). Loading/applying a preset IS Editor-only logic, but that lives in
// SAgentSolverControlPanel.cpp, not here.

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "UObject/SoftObjectPtr.h"
#include "AgentSolverPreset.generated.h"

class UWorld;
class USkeletalMesh;
class UMassMuscleProfileAssetMass;
class UMassMuscleProfileAssetMuscle;
class ULearningAgentsNeuralNetwork;
class UAnimSequence;
class AMutoRLTrainingDriver;

// EditInlineNew: lets any TObjectPtr<UAgentSolverPreset> UPROPERTY (e.g.
// ULearningProgramNode::Params) be edited inline in the Details panel --
// without it, the property widget only offers an asset-reference picker,
// which for a per-node Instanced subobject (never itself a saved Content
// Browser asset) looked permanently empty with no way to create or assign
// one. With it, the widget also gains an inline expand/edit (for whatever
// preset -- asset or instanced -- is currently assigned), a "New" button to
// create a fresh instanced one, and an asset picker to point at an existing
// saved UAgentSolverPreset asset instead.
UCLASS(BlueprintType, EditInlineNew)
class AGENTSOLVER_API UAgentSolverPreset : public UDataAsset
{
	GENERATED_BODY()

public:
	// ----- Environment -----
	/**
	 * Mirrors UAgentSolverViewportSettings::EnvironmentLevel -- the cosmetic
	 * preview-scene level the embedded viewport copies static geometry
	 * (floors, walls, dressing) from for visual context around the rig.
	 * Loading a preset restores this into the Viewport tab, so reopening the
	 * project with a preset active gets your last preview environment back.
	 *
	 * NOT the level containing the AMutoRLTrainingDriver actor, and does NOT
	 * open/switch the editor's actual current level -- an earlier version of
	 * this field meant that instead (via FEditorFileUtils::LoadMap in
	 * SAgentSolverControlPanel::LoadPreset), which turned out both unwanted
	 * (forcibly switching the editor's open level as a side effect of
	 * picking a preset) and unsafe (LoadMap is a heavy, world-destroying
	 * call, and running it synchronously from inside a tab-spawn/
	 * double-click callback caused real problems -- see LoadPreset's
	 * comment). Removed 2026-08-25; this field now does only what its name
	 * on UAgentSolverViewportSettings already promises.
	 */
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Environment")
	TSoftObjectPtr<UWorld> EnvironmentLevel;

	// ----- Rig assets -- mirrors AMutoRLTrainingDriver's own "Muto RL|Rig" fields -----
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Rig")
	TObjectPtr<USkeletalMesh> SkeletalMesh;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Rig")
	TObjectPtr<UMassMuscleProfileAssetMass> MassAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Rig")
	TObjectPtr<UMassMuscleProfileAssetMuscle> MuscleAsset;

	// ----- Network assets -- mirrors AMutoRLTrainingDriver's own 6 "Muto RL|Learning" slots -----
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadEncoderNetworkAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadPolicyNetworkAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadDecoderNetworkAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> SaveEncoderNetworkAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> SavePolicyNetworkAsset;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Networks")
	TObjectPtr<ULearningAgentsNeuralNetwork> SaveDecoderNetworkAsset;

	/** See AMutoRLTrainingDriver::GlobalMuscleStrengthScale's comment. */
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Rig")
	float GlobalMuscleStrengthScale = 1.0f;

	/** See AMutoRLTrainingDriver::MuscleActivationThresholdMultiplier's comment. */
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Rig")
	float MuscleActivationThresholdMultiplier = 1.0f;

	// ----- Tuning parameters -- mirrors the driver's "Muto RL|Reward" fields,
	// its "Muto RL|Tuning" (Reward Settings pane) fields, and Gravity 1:1. Kept
	// as a flat list of scalars, not a shared struct, so this asset has zero
	// compile-time dependency on CreatureRLEnvironment.h or the driver header. -----
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float MaxTorquePerDOF = 5.0e7f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float MinUprightDot = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float MinHeightFraction = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float AliveBonus = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float UprightWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float BalanceWeight = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float TorquePenaltyWeight = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float RewardHeightTarget = 100.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float RewardHeightMultiplier = 1.0f;

	/** See AMutoRLTrainingDriver::RewardEnergyConsumptionMultiplier's comment for why this has a DisplayName override -- kept as the same field name for preset-value compatibility. */
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning", meta = (DisplayName = "Energy Consumption Malus Multiplier"))
	float RewardEnergyConsumptionMultiplier = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning", meta = (DisplayName = "Muscles Use Malus Multiplier"))
	float RewardMusclesUseMultiplier = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float GlobalRewardScale = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	float GlobalRewardOffset = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Tuning")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	// ----- Imitation -- mirrors the driver's "Muto RL|Imitation" and
	// "Muto RL|Tuning|Imitation" fields 1:1, same flat-scalars-no-shared-struct
	// rule as the block above. ObjectiveMode is stored as a bool rather than
	// the EMutoObjectiveMode enum for the same reason: that enum lives in the
	// driver header, and this asset deliberately does not include it. -----

	/** Whether this preset trains the imitation objective rather than standing/balance. Maps to AMutoRLTrainingDriver::ObjectiveMode. */
	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	bool bImitationObjective = false;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	TObjectPtr<UAnimSequence> ReferenceMotion;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ReferencePoseTime = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	bool bImitateFullClip = false;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ReferenceSampleRate = 30.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	bool bReferenceMotionLoops = true;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	bool bResetToReferencePose = true;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	TArray<FName> EndEffectorBoneNames;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationPoseWeight = 0.65f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationVelocityWeight = 0.10f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationEndEffectorWeight = 0.15f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationRootWeight = 0.10f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationPoseErrorScale = 2.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationVelocityErrorScale = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationEndEffectorErrorScale = 40.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationRootErrorScale = 10.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	float ImitationMaxPoseErrorRad = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Agent Solver Preset|Imitation")
	bool bImitationTerminateOnUprightAndHeight = true;

	/**
	 * Copies every Rig/Networks/Tuning/Imitation field above onto Driver,
	 * exactly as SAgentSolverControlPanel::LoadPreset used to do inline (see
	 * that function's comment for why Rig assets are skipped when empty
	 * rather than nulling the driver's current assignment, and why
	 * EnvironmentLevel is NOT copied here -- that one is UI-only viewport
	 * state, not driver state). Extracted so ULearningProgramNode's
	 * per-stage Params (an instanced UAgentSolverPreset) can push its values
	 * onto the driver the same way a manually-loaded preset does, without a
	 * second copy of this field list to keep in sync.
	 *
	 * Plain runtime logic -- both classes involved are ordinary runtime
	 * UObjects, so unlike SAgentSolverControlPanel this is NOT WITH_EDITOR-
	 * gated, and can be called from the training driver itself while a PIE
	 * session is running (e.g. a Learning Program transition firing on the
	 * background training thread).
	 */
	void ApplyToDriver(AMutoRLTrainingDriver* Driver) const;
};
