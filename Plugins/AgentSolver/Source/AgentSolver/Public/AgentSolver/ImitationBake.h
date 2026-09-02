#pragma once

// Editor-only: turns a UAnimSequence into a CreatureImitation::FReferenceMotion,
// i.e. samples an animation and converts every frame into the solver's own
// joint coordinates.
//
// Split out of CreatureImitation.h on purpose. This is the only part of the
// imitation work that needs USkeletalMesh/UAnimSequence and the animation
// evaluation machinery; keeping it here leaves the reward math itself free of
// any asset dependency, so it stays exercisable against the synthetic test rigs
// the rest of the plugin uses (see Tests/CreatureImitationTest.cpp).
//
// Baking happens ONCE, on the game thread, in AMutoRLTrainingDriver::
// StartTraining -- never per step and never from the background training
// thread. Animation evaluation is not thread-safe against editor asset state,
// and the result is a few hundred KB of plain floats that every one of the 256
// parallel envs then shares read-only.
//
// No new module dependency: GetAnimationPose (AnimSequence.h), FBoneContainer::
// InitializeTo (BoneContainer.h), FAnimExtractContext (AnimationAsset.h) and
// FAnimationPoseData (Animation/AnimationPoseData.h) all live in the Engine
// module, which AgentSolver.Build.cs already lists.
#if WITH_EDITOR

#include "CoreMinimal.h"
#include "AgentSolver/CreatureImitation.h"
#include "PhysicsSolver/CreatureBatchState.h"
#include "Engine/SkeletalMesh.h"
#include "ReferenceSkeleton.h"
#include "Animation/AnimSequence.h"
#include "Animation/AnimationPoseData.h"
#include "Animation/AnimationAsset.h"
#include "Animation/Skeleton.h"
#include "BonePose.h"
#include "BoneContainer.h"
#include "AnimationRuntime.h"

namespace ImitationBake
{
	/**
	 * Evaluates one time of an animation into COMPONENT-space bone transforms,
	 * indexed by the skeletal mesh's own bone indices (which is what
	 * FCreatureTopology::BodyBoneIndex holds).
	 *
	 * The bone container is initialized over EVERY bone, so a compact pose
	 * index equals a mesh bone index and no remapping table is needed. A
	 * reference skeleton is stored parent-before-child, so local->component
	 * space is a single forward pass with no recursion.
	 */
	inline bool EvaluateComponentSpacePose(
		const USkeletalMesh& Mesh,
		const UAnimSequence& AnimSequence,
		float Time,
		TArray<FTransform>& OutComponentSpace)
	{
		const FReferenceSkeleton& RefSkel = Mesh.GetRefSkeleton();
		const int32 NumBones = RefSkel.GetNum();
		if (NumBones == 0)
		{
			return false;
		}

		TArray<FBoneIndexType> RequiredBones;
		RequiredBones.Reserve(NumBones);
		for (int32 Bone = 0; Bone < NumBones; ++Bone)
		{
			RequiredBones.Add((FBoneIndexType)Bone);
		}

		// FMemMark/FMemStack is required: FCompactPose allocates from the
		// per-thread stack allocator, and without the mark that memory is never
		// released.
		FMemMark Mark(FMemStack::Get());

		FBoneContainer BoneContainer;
		BoneContainer.InitializeTo(RequiredBones, UE::Anim::FCurveFilterSettings(), Mesh);

		FCompactPose Pose;
		Pose.SetBoneContainer(&BoneContainer);
		FBlendedCurve Curve;
		UE::Anim::FStackAttributeContainer Attributes;
		FAnimationPoseData PoseData(Pose, Curve, Attributes);

		AnimSequence.GetAnimationPose(PoseData, FAnimExtractContext((double)Time));

		OutComponentSpace.SetNum(NumBones);
		for (int32 Bone = 0; Bone < NumBones; ++Bone)
		{
			const FCompactPoseBoneIndex CompactIndex(Bone);
			const FTransform Local = Pose.IsValidIndex(CompactIndex) ? Pose[CompactIndex] : RefSkel.GetRefBonePose()[Bone];
			const int32 Parent = RefSkel.GetParentIndex(Bone);
			OutComponentSpace[Bone] = (Parent == INDEX_NONE) ? Local : (Local * OutComponentSpace[Parent]);
		}
		return true;
	}

	/** The reference skeleton's own rest pose in component space -- the baseline RootHeightAboveRest is measured against, and the pose the rest-pose retarget invariant is checked with. */
	inline void BuildRestComponentSpacePose(const USkeletalMesh& Mesh, TArray<FTransform>& OutComponentSpace)
	{
		const FReferenceSkeleton& RefSkel = Mesh.GetRefSkeleton();
		const int32 NumBones = RefSkel.GetNum();
		const TArray<FTransform>& RefPose = RefSkel.GetRefBonePose();

		OutComponentSpace.SetNum(NumBones);
		for (int32 Bone = 0; Bone < NumBones; ++Bone)
		{
			const int32 Parent = RefSkel.GetParentIndex(Bone);
			OutComponentSpace[Bone] = (Parent == INDEX_NONE) ? RefPose[Bone] : (RefPose[Bone] * OutComponentSpace[Parent]);
		}
	}

	/**
	 * Bone-indexed component-space transforms -> BODY-indexed ones, via
	 * FCreatureTopology::BodyBoneIndex. This is the only place the bone/body
	 * distinction is resolved; everything downstream is body-indexed.
	 *
	 * A body whose bone index is missing keeps its parent's transform rather
	 * than an identity, so it retargets to a zero joint angle instead of to a
	 * wild one -- an unmapped body should read as "not moving", never as "moved
	 * to the origin".
	 */
	inline bool GatherBodyTransforms(
		const FCreatureTopology& Topo,
		const TArray<FTransform>& BoneComponentSpace,
		TArray<FTransform>& OutBodyCS,
		TArray<FString>* OutWarnings = nullptr)
	{
		OutBodyCS.SetNum(Topo.NumBodies);
		bool bAllMapped = true;

		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 BoneIdx = Topo.BodyBoneIndex.IsValidIndex(Body) ? Topo.BodyBoneIndex[Body] : INDEX_NONE;
			if (BoneComponentSpace.IsValidIndex(BoneIdx))
			{
				OutBodyCS[Body] = BoneComponentSpace[BoneIdx];
			}
			else
			{
				bAllMapped = false;
				const int32 Parent = Topo.BodyParent.IsValidIndex(Body) ? Topo.BodyParent[Body] : INDEX_NONE;
				OutBodyCS[Body] = (Parent != INDEX_NONE && Parent < Body) ? OutBodyCS[Parent] : FTransform::Identity;
				if (OutWarnings)
				{
					OutWarnings->Add(FString::Printf(
						TEXT("ImitationBake: body %d has no valid BodyBoneIndex (%d) — it will retarget to a zero joint angle."), Body, BoneIdx));
				}
			}
		}
		return bAllMapped;
	}

	/**
	 * Bakes a whole clip (or a single time, when StartTime == EndTime) into an
	 * FReferenceMotion.
	 *
	 * SampleRate is the bake rate, not the source clip's own frame rate --
	 * sampling denser than the source just interpolates, which is harmless, and
	 * sampling at a round rate keeps the finite-differenced velocities clean.
	 *
	 * bLooping controls both the phase wrap and the velocity finite-difference
	 * at the clip's ends. Set it false for a one-shot motion, or the first and
	 * last frames get velocities that pretend the clip teleports back to its
	 * start.
	 */
	inline bool BakeReferenceMotion(
		const USkeletalMesh& Mesh,
		const UAnimSequence& AnimSequence,
		const FCreatureTopology& Topo,
		const TArray<int32>& EndEffectorBodies,
		float StartTime,
		float EndTime,
		float SampleRate,
		bool bLooping,
		CreatureImitation::FReferenceMotion& OutMotion,
		TArray<FString>& OutWarnings)
	{
		OutMotion = CreatureImitation::FReferenceMotion();
		OutMotion.EndEffectorBodies = EndEffectorBodies;
		OutMotion.bLooping = bLooping;
		OutMotion.SampleRate = FMath::Max(SampleRate, 1.0f);

		if (Topo.NumBodies == 0)
		{
			OutWarnings.Add(TEXT("ImitationBake: topology is empty."));
			return false;
		}

		// The clip must be authored against this mesh's skeleton, or every bone
		// index below refers to a different bone than intended -- a silent,
		// extremely confusing failure if it goes unchecked.
		if (AnimSequence.GetSkeleton() != Mesh.GetSkeleton())
		{
			OutWarnings.Add(FString::Printf(
				TEXT("ImitationBake: '%s' is authored against skeleton '%s' but the mesh uses '%s' — bone indices will not correspond."),
				*AnimSequence.GetName(),
				AnimSequence.GetSkeleton() ? *AnimSequence.GetSkeleton()->GetName() : TEXT("<none>"),
				Mesh.GetSkeleton() ? *Mesh.GetSkeleton()->GetName() : TEXT("<none>")));
			return false;
		}

		// RootHeightAboveRest is measured against this rig's own rest pose, so
		// that the frame's height is comparable against the batch's world Z
		// (which stands at a contact-derived TargetTorsoHeight, an unrelated
		// number). See FReferenceFrame::RootHeightAboveRest.
		TArray<FTransform> RestBoneCS;
		BuildRestComponentSpacePose(Mesh, RestBoneCS);
		TArray<FTransform> RestBodyCS;
		GatherBodyTransforms(Topo, RestBoneCS, RestBodyCS, &OutWarnings);
		const double RestRootZ = RestBodyCS.IsValidIndex(0) ? RestBodyCS[0].GetTranslation().Z : 0.0;

		const float ClipLength = AnimSequence.GetPlayLength();
		const float ClampedStart = FMath::Clamp(StartTime, 0.0f, ClipLength);
		const float ClampedEnd = FMath::Clamp(EndTime, ClampedStart, ClipLength);
		const float Span = ClampedEnd - ClampedStart;

		// A zero span is the single-pose case (phase 1) and is fully supported:
		// one frame, zero velocities everywhere, which is exactly the right
		// reference for "hold this pose".
		const int32 NumFrames = (Span <= KINDA_SMALL_NUMBER)
			? 1
			: FMath::Max(2, FMath::RoundToInt(Span * OutMotion.SampleRate));

		OutMotion.Duration = Span;
		OutMotion.Frames.SetNum(NumFrames);

		TArray<FTransform> BoneCS;
		TArray<FTransform> BodyCS;
		float MaxResidual = 0.0f;

		for (int32 F = 0; F < NumFrames; ++F)
		{
			// A looping clip's last sample must NOT land back on the first
			// (that would duplicate a frame and stall the phase for one step),
			// so divide the span by NumFrames when looping and NumFrames-1
			// when not.
			const float Denominator = (NumFrames <= 1) ? 1.0f : (bLooping ? (float)NumFrames : (float)(NumFrames - 1));
			const float Time = ClampedStart + (NumFrames <= 1 ? 0.0f : Span * ((float)F / Denominator));

			if (!EvaluateComponentSpacePose(Mesh, AnimSequence, Time, BoneCS))
			{
				OutWarnings.Add(FString::Printf(TEXT("ImitationBake: failed to evaluate '%s' at t=%.3f."), *AnimSequence.GetName(), Time));
				return false;
			}
			GatherBodyTransforms(Topo, BoneCS, BodyCS);

			float FrameResidual = 0.0f;
			CreatureImitation::RetargetPoseToJointSpace(Topo, BodyCS, EndEffectorBodies, OutMotion.Frames[F], &FrameResidual);
			OutMotion.Frames[F].RootHeightAboveRest = (float)(BodyCS[0].GetTranslation().Z - RestRootZ);
			MaxResidual = FMath::Max(MaxResidual, FrameResidual);
		}

		OutMotion.MaxRevoluteResidualRad = MaxResidual;
		CreatureImitation::FillVelocitiesCentralDifference(Topo, OutMotion);

		// Not an error -- a rig whose revolutes cannot represent the clip's
		// off-axis motion still trains, it just cannot reach a pose reward of
		// 1.0. Surfacing the number is what lets that be diagnosed instead of
		// mistaken for a learning failure.
		if (MaxResidual > FMath::DegreesToRadians(20.0f))
		{
			OutWarnings.Add(FString::Printf(
				TEXT("ImitationBake: '%s' asks a 1-DOF joint to rotate up to %.1f° off its own axis — that motion cannot be represented by this rig and is discarded, capping the achievable pose reward."),
				*AnimSequence.GetName(), FMath::RadiansToDegrees(MaxResidual)));
		}

		return true;
	}

	/**
	 * Resolves end-effector BONE names to body indices. Names that match no
	 * body are reported and skipped rather than failing the bake -- an
	 * end-effector list is a tuning choice, and a typo in it should not stop
	 * training.
	 */
	inline void ResolveEndEffectorBodies(
		const FCreatureTopology& Topo,
		const TArray<FName>& BodyDebugNames,
		const TArray<FName>& EndEffectorNames,
		TArray<int32>& OutBodies,
		TArray<FString>& OutWarnings)
	{
		OutBodies.Reset();
		for (const FName& Name : EndEffectorNames)
		{
			const int32 Body = BodyDebugNames.IndexOfByKey(Name);
			if (Body != INDEX_NONE && Body < Topo.NumBodies)
			{
				OutBodies.AddUnique(Body);
			}
			else
			{
				OutWarnings.Add(FString::Printf(TEXT("ImitationBake: end-effector bone '%s' matches no body in this topology — skipping."), *Name.ToString()));
			}
		}
	}
}

#endif // WITH_EDITOR
