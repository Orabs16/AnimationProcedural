#include "AgentSolver/AgentSolverPreset.h"
#include "AgentSolver/MutoRLTrainingDriver.h"

void UAgentSolverPreset::ApplyToDriver(AMutoRLTrainingDriver* Driver) const
{
	if (!Driver)
	{
		return;
	}

	// Rig assets are NOT unconditionally overwritten -- SkeletalMesh/
	// MassAsset/MuscleAsset are structurally REQUIRED for StartTraining() to
	// do anything at all, so nulling them from an incompletely-filled preset
	// (or Learning Program node) would silently break training for the rest
	// of the PIE session. See SAgentSolverControlPanel::LoadPreset's original
	// comment (2026-08-25 bug report) for the full incident this guards
	// against.
	if (SkeletalMesh) { Driver->SkeletalMesh = SkeletalMesh; }
	if (MassAsset) { Driver->MassAsset = MassAsset; }
	if (MuscleAsset) { Driver->MuscleAsset = MuscleAsset; }
	Driver->LoadEncoderNetworkAsset = LoadEncoderNetworkAsset;
	Driver->LoadPolicyNetworkAsset = LoadPolicyNetworkAsset;
	Driver->LoadDecoderNetworkAsset = LoadDecoderNetworkAsset;
	Driver->SaveEncoderNetworkAsset = SaveEncoderNetworkAsset;
	Driver->SavePolicyNetworkAsset = SavePolicyNetworkAsset;
	Driver->SaveDecoderNetworkAsset = SaveDecoderNetworkAsset;
	Driver->MaxTorquePerDOF = MaxTorquePerDOF;
	Driver->MinUprightDot = MinUprightDot;
	Driver->MinHeightFraction = MinHeightFraction;
	Driver->AliveBonus = AliveBonus;
	Driver->UprightWeight = UprightWeight;
	Driver->BalanceWeight = BalanceWeight;
	Driver->TorquePenaltyWeight = TorquePenaltyWeight;
	Driver->RewardHeightTarget = RewardHeightTarget;
	Driver->RewardHeightMultiplier = RewardHeightMultiplier;
	Driver->RewardEnergyConsumptionMultiplier = RewardEnergyConsumptionMultiplier;
	Driver->RewardMusclesUseMultiplier = RewardMusclesUseMultiplier;
	Driver->GlobalRewardScale = GlobalRewardScale;
	Driver->GlobalRewardOffset = GlobalRewardOffset;
	Driver->GlobalMuscleStrengthScale = GlobalMuscleStrengthScale;
	Driver->MuscleActivationThresholdMultiplier = MuscleActivationThresholdMultiplier;
	Driver->Gravity = Gravity;
	Driver->ObjectiveMode = bImitationObjective ? EMutoObjectiveMode::Imitation : EMutoObjectiveMode::Standing;
	Driver->ReferenceMotion = ReferenceMotion;
	Driver->ReferencePoseTime = ReferencePoseTime;
	Driver->bImitateFullClip = bImitateFullClip;
	Driver->ReferenceSampleRate = ReferenceSampleRate;
	Driver->bReferenceMotionLoops = bReferenceMotionLoops;
	Driver->bResetToReferencePose = bResetToReferencePose;
	Driver->EndEffectorBoneNames = EndEffectorBoneNames;
	Driver->ImitationPoseWeight = ImitationPoseWeight;
	Driver->ImitationVelocityWeight = ImitationVelocityWeight;
	Driver->ImitationEndEffectorWeight = ImitationEndEffectorWeight;
	Driver->ImitationRootWeight = ImitationRootWeight;
	Driver->ImitationPoseErrorScale = ImitationPoseErrorScale;
	Driver->ImitationVelocityErrorScale = ImitationVelocityErrorScale;
	Driver->ImitationEndEffectorErrorScale = ImitationEndEffectorErrorScale;
	Driver->ImitationRootErrorScale = ImitationRootErrorScale;
	Driver->ImitationMaxPoseErrorRad = ImitationMaxPoseErrorRad;
	Driver->bImitationTerminateOnUprightAndHeight = bImitationTerminateOnUprightAndHeight;
}
