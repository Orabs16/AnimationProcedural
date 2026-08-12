#pragma once

// Editor-only: builds an FCreatureTopology from Muto's authored rig/profile
// assets (USkeletalMesh + UMassMuscleProfileAssetMuscle/Mass, both from the
// MassMuscleProfile plugin, an Editor-type module — see AgentSolver.Build.cs).
// This is a Muto-specific data-prep step, not a generic rig importer; a
// generic FReferenceSkeleton-walking tool is a separate, later piece of work.
#if WITH_EDITOR

#include "CoreMinimal.h"
#include "CreatureBatchState.h"
#include "UMassMuscleProfileAsset.h"
#include "Engine/SkeletalMesh.h"
#include "ReferenceSkeleton.h"

namespace MutoTopology
{
	/**
	 * One limb chain as it exists in Muto's skeleton, right down to the joint
	 * bone names — see the comment on BuildMutoTopology() for how this was
	 * derived and what's authored vs. inferred.
	 *
	 * MountBone IS its own ABA body — a 3-DOF ball joint relative to the
	 * torso (e.g. "FShoulder_muscle_Roll/Yaw/Pitch_L", BoneName=FShoulder_L,
	 * is the muscle triple that drives FShoulder's OWN rotation; see the
	 * Bone/Child semantics correction on the muscle-lookup code below for how
	 * this was confirmed — CORRECTS an earlier, wrong assumption that the
	 * mount bone was unarticulated/fused into the torso, based on a
	 * misreading of which of {BoneName, ChildBoneName} is the body that
	 * actually rotates).
	 *
	 * JointBones is every bone in the chain past the mount, base name without
	 * the _L/_R suffix, in parent-to-child order. EVERY entry becomes its own
	 * 1-DOF Yaw revolute body EXCEPT THE LAST ONE: the final entry in each
	 * archetype is the limb's "Tip" bone (FTip/BTip/MTip/FeetTip) — confirmed
	 * against the real muscle data (2026-08-11, user-reported and verified)
	 * that no muscle is ever authored with BoneName==<Tip bone>, i.e. Tip has
	 * no independent joint at all and is meant to ride along rigidly with its
	 * real parent (e.g. Feet/FHand) — so BuildMutoTopology does not create an
	 * ABA body for it (see the per-limb loop below).
	 */
	struct FLimbArchetype
	{
		FName MountBone;
		TArray<FName> JointBones;
	};

	inline TArray<FLimbArchetype> GetMutoLimbArchetypes()
	{
		return {
			{ TEXT("FShoulder"), { TEXT("FElbow"), TEXT("FElbow2"), TEXT("FElbow3"), TEXT("FHand"), TEXT("FTip") } },
			{ TEXT("BShoulder"), { TEXT("BElbow1"), TEXT("BElbow2"), TEXT("BElbow3"), TEXT("BHand"), TEXT("BTip") } },
			{ TEXT("MShoulder"), { TEXT("MElbow"), TEXT("MHand"), TEXT("MTip") } },
			{ TEXT("Hips"), { TEXT("Knee1"), TEXT("Knee2"), TEXT("Feet"), TEXT("FeetTip") } },
		};
	}

	/** Non-limb bones fused into the torso (body 0) — the whole spine/head/mouth chain. */
	inline TArray<FName> GetMutoTorsoBones()
	{
		return {
			TEXT("Pelvis"), TEXT("Back1"), TEXT("Back2"), TEXT("Back3"),
			TEXT("Head1"), TEXT("Head2"), TEXT("UpperMouth"), TEXT("LowerMouth"), TEXT("Chin"),
		};
	}

	/**
	 * Composes RefSkel's rest-pose local bone transforms from BoneIndex up to
	 * (not including) RelativeToBoneIndex, which must be an ancestor of
	 * BoneIndex. Returns BoneIndex's rest transform expressed in
	 * RelativeToBoneIndex's rest frame.
	 */
	inline FTransform GetRestTransformRelativeTo(const FReferenceSkeleton& RefSkel, int32 BoneIndex, int32 RelativeToBoneIndex)
	{
		FTransform Composite = FTransform::Identity;
		int32 Idx = BoneIndex;
		while (Idx != INDEX_NONE && Idx != RelativeToBoneIndex)
		{
			Composite = Composite * RefSkel.GetRawRefBonePose()[Idx];
			Idx = RefSkel.GetParentIndex(Idx);
		}
		ensureMsgf(Idx == RelativeToBoneIndex, TEXT("GetRestTransformRelativeTo: RelativeToBoneIndex is not an ancestor of BoneIndex"));
		return Composite;
	}

	/**
	 * Builds an FCreatureTopology for Muto from its rig + authored profile data.
	 *
	 * Body 0 (torso) fuses the whole spine/head/mouth chain (GetMutoTorsoBones)
	 * only — every limb's mount bone is its own real ABA body now (see
	 * FLimbArchetype's comment on the Bone/Child semantics correction). Each
	 * of the 8 limbs (GetMutoLimbArchetypes, x2 for _L/_R) becomes a chain of
	 * ABA bodies: the mount bone as a 3-DOF ball joint, then every remaining
	 * JointBones entry EXCEPT THE LAST as a 1-DOF Yaw revolute — the last
	 * entry (the limb's Tip bone) gets no body at all, since no muscle is
	 * ever authored to independently rotate it (see FLimbArchetype). Total
	 * body/DOF count per limb is unchanged by this — it's still "one 3-DOF
	 * ball + N 1-DOF revolutes" either way, just shifted by one link (mount
	 * in, Tip out).
	 *
	 * Per-body mass comes directly from MassAsset (currently a uniform 1.0
	 * placeholder for every bone in the asset — this function doesn't invent
	 * different numbers, just sums/reads what's there so it stays traceable
	 * back to the source asset once real mass values get authored). Inertia
	 * has no source at all yet, so it's a thin-uniform-rod-about-its-own-
	 * length approximation from each body's own mass and offset — a
	 * placeholder, not a measurement; revisit once real mass/inertia data
	 * exists (see the "Mass source" decision in project memory/roadmap).
	 *
	 * MuscleAsset is optional and used only for a defensive cross-check: if
	 * F's authored axes turn out not to be Yaw where we assumed, this warns
	 * instead of silently building a topology that no longer matches the
	 * source data.
	 */
	inline bool BuildMutoTopology(
		USkeletalMesh* SkeletalMesh,
		UMassMuscleProfileAssetMass* MassAsset,
		UMassMuscleProfileAssetMuscle* MuscleAsset,
		FCreatureTopology& OutTopology,
		TArray<FString>& OutWarnings,
		TArray<FName>* OutBodyDebugNames = nullptr)
	{
		if (!SkeletalMesh)
		{
			OutWarnings.Add(TEXT("BuildMutoTopology: SkeletalMesh is null."));
			return false;
		}

		const FReferenceSkeleton& RefSkel = SkeletalMesh->GetRefSkeleton();
		const int32 PelvisIdx = RefSkel.FindBoneIndex(TEXT("Pelvis"));
		if (PelvisIdx == INDEX_NONE)
		{
			OutWarnings.Add(TEXT("BuildMutoTopology: could not find bone 'Pelvis'."));
			return false;
		}

		// Pelvis's own bind-pose rotation in COMPONENT space (composed all the
		// way up to the skeleton's true root, not stopping at Pelvis like every
		// other GetRestTransformRelativeTo call below does — RelativeToBoneIndex
		// = INDEX_NONE walks to the top). Everything else in this function
		// treats Pelvis as body 0's own frame (identity, by construction) for
		// composing its CHILDREN's offsets/rotations, which is fine and
		// self-consistent for the limbs — but it means body 0's OWN rest
		// rotation was never captured anywhere. Callers that reset body 0 to
		// "standing" (AMutoRLTrainingDriver/AMutoRLVisualizerActor) need this:
		// the root/master bone above Pelvis is often not identity-oriented
		// (e.g. rotated to point down the spine toward Pelvis rather than
		// aligned to world axes), so assuming "Pelvis at world identity
		// rotation" means "standing upright" is only true if that root bone
		// happens to be identity too — not a safe assumption. Stored in the
		// otherwise-unused root slot of BodyRestRotInParent (see below).
		const FTransform TorsoRestInComponentSpace = GetRestTransformRelativeTo(RefSkel, PelvisIdx, INDEX_NONE);

		auto GetMass = [MassAsset](FName BoneName) -> float
		{
			if (!MassAsset) return 1.0f;
			const int32 Idx = MassAsset->FindBoneByName(BoneName);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].Mass : 1.0f;
		};
		auto GetRadius = [MassAsset](FName BoneName) -> float
		{
			if (!MassAsset) return 15.0f;
			const int32 Idx = MassAsset->FindBoneByName(BoneName);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].Radius : 15.0f;
		};
		auto GetCapsuleHalfHeight = [MassAsset](FName BoneName) -> float
		{
			if (!MassAsset) return 0.0f;
			const int32 Idx = MassAsset->FindBoneByName(BoneName);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].CapsuleHalfHeight : 0.0f;
		};

		// Sanity-check every limb's authored data against this builder's
		// assumption (the joint pivoting AT the mount bone = 3-DOF ball,
		// every subsequent joint = 1-DOF Yaw revolute) — all 4 archetypes
		// are authored now, not just F, so check all of them instead of
		// just F. Restricted to limb bones (mount + chain, both sides) so
		// the still-fused, differently-shaped torso/spine muscle data
		// (Back*/Head*/Mouth, out of scope for this topology) doesn't spam
		// unrelated warnings.
		if (MuscleAsset)
		{
			TSet<FName> BallJointPivotBoneNames; // e.g. FShoulder_L, Hips_L — the mount bone itself IS the ball joint (see FLimbArchetype's comment)
			TSet<FName> LimbRelevantBoneNames;
			for (const FLimbArchetype& Archetype : GetMutoLimbArchetypes())
			{
				for (const TCHAR* Side : { TEXT("_L"), TEXT("_R") })
				{
					const FName MountBoneName(Archetype.MountBone.ToString() + Side);
					BallJointPivotBoneNames.Add(MountBoneName);
					LimbRelevantBoneNames.Add(MountBoneName);
					for (const FName& JointBone : Archetype.JointBones)
					{
						LimbRelevantBoneNames.Add(FName(JointBone.ToString() + Side));
					}
				}
			}

			for (const FMassMuscleDataMuscle& M : MuscleAsset->Muscles)
			{
				if (!LimbRelevantBoneNames.Contains(M.BoneName))
				{
					continue; // torso/spine muscle, out of scope for this topology
				}
				const bool bExpectBallJoint = BallJointPivotBoneNames.Contains(M.BoneName);
				if (!bExpectBallJoint && M.Orientation != ERotationType::Yaw)
				{
					OutWarnings.Add(FString::Printf(TEXT("Muscle '%s' (bone %s, child %s) is not Yaw — BuildMutoTopology assumes all non-ball-joint revolutes are Yaw; axis may now be wrong."),
						*M.Name.ToString(), *M.BoneName.ToString(), *M.ChildBoneName.ToString()));
				}
			}
		}

		// ---- Body list: index 0 = torso, then each limb's chain in order ----
		TArray<int32> BodyParent = { 0 }; // root's own parent entry is unused
		TArray<int32> BodyDOFCount = { 0 };
		TArray<int32> BodyLimbIndex = { INDEX_NONE };
		TArray<FVector> JointOffsetInParent = { FVector::ZeroVector };
		TArray<FQuat> RestRotInParent = { TorsoRestInComponentSpace.GetRotation() }; // body 0 slot repurposed — see comment above
		TArray<FVector> JointAxisLocal = { FVector::UpVector };
		TArray<float> Mass = { 0.0f }; // filled in after torso-fusion sum, below
		TArray<FName> DebugBoneName = { TEXT("Torso") };
		TMap<int32, FVector> FusedTipOffsetByBody; // see BodyFusedTipOffset's comment — applied to OutTopology after Build()

		const TArray<FLimbArchetype> Archetypes = GetMutoLimbArchetypes();
		int32 LimbIndex = 0;
		for (const FLimbArchetype& Archetype : Archetypes)
		{
			for (const TCHAR* Side : { TEXT("_L"), TEXT("_R") })
			{
				// Claim this physical limb's slot (0..7: one per archetype x
				// side) up front, before any early-continue below, so a
				// missing bone on one side never shifts the other side's or
				// a later archetype's index — every archetype x side
				// combination always gets a stable, distinct slot matching
				// NumLimbs = Archetypes.Num() * 2.
				const int32 ThisLimbIndex = LimbIndex++;

				const FName MountBoneName(Archetype.MountBone.ToString() + Side);
				const int32 MountBoneIdx = RefSkel.FindBoneIndex(MountBoneName);
				if (MountBoneIdx == INDEX_NONE)
				{
					OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: could not find mount bone '%s' — skipping limb."), *MountBoneName.ToString()));
					continue;
				}

				// ---- Mount bone: the limb's own 3-DOF ball joint, relative
				// to the torso — see FLimbArchetype's comment on the
				// Bone/Child semantics correction (a "MountBone_muscle_
				// Roll/Yaw/Pitch" triple, BoneName==MountBone, is what
				// actually drives this). ----
				const FTransform MountRestRelToPelvis = GetRestTransformRelativeTo(RefSkel, MountBoneIdx, PelvisIdx);
				const int32 MountBody = BodyParent.Num();
				BodyParent.Add(0);
				BodyDOFCount.Add(3);
				BodyLimbIndex.Add(ThisLimbIndex);
				JointOffsetInParent.Add(MountRestRelToPelvis.GetTranslation());
				RestRotInParent.Add(MountRestRelToPelvis.GetRotation());
				JointAxisLocal.Add(FVector::ZeroVector); // unused for ball joints
				Mass.Add(GetMass(MountBoneName));
				DebugBoneName.Add(MountBoneName);

				int32 ParentBody = MountBody;
				int32 ParentBoneIdx = MountBoneIdx;

				// ---- Remaining chain: every JointBones entry EXCEPT THE
				// LAST becomes its own 1-DOF Yaw revolute body. The last
				// entry — the limb's "Tip" bone (FTip/BTip/MTip/FeetTip) —
				// deliberately gets NO body: confirmed against the real
				// muscle data (2026-08-11, user-reported and verified with a
				// throwaway dump test) that no muscle is ever authored with
				// BoneName==<Tip bone>; only the SECOND-TO-LAST bone's own
				// muscle exists (e.g. "Feet_muscle_Yaw_R", BoneName=Feet_R,
				// ChildBoneName=FeetTip_R), and per the corrected semantics
				// (BoneName is the body that rotates, ChildBoneName is just
				// a downstream reference) that muscle drives FEET's own
				// rotation, not an independent Tip joint. Tip rides along
				// rigidly with its real parent — the same "never explicitly
				// posed, keeps its authored rest-pose local transform"
				// pattern already used for the spine/mouth bones fused into
				// the torso. CanTouchGround-flagged Tip bones (Hips'
				// FeetTip) no longer get their own ground-contact point in
				// CreatureGroundContact.h, since that body no longer exists
				// to iterate over — but the Tip's rest-pose offset from its
				// real parent (Feet/Hand) is still captured below into
				// FusedTipOffsetByBody, so that parent's OWN contact point
				// can be extended out to the Tip's actual visual extent
				// (confirmed necessary in practice: without this, Feet/
				// Hand's contact point calibrates ground height to its own
				// joint origin only, and the un-corrected Tip geometry
				// hanging past it visibly pokes through the ground). ----
				for (int32 JointOrdinal = 0; JointOrdinal < Archetype.JointBones.Num() - 1; ++JointOrdinal)
				{
					const FName JointBoneName(Archetype.JointBones[JointOrdinal].ToString() + Side);
					const int32 JointBoneIdx = RefSkel.FindBoneIndex(JointBoneName);
					if (JointBoneIdx == INDEX_NONE)
					{
						OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: could not find joint bone '%s' — truncating limb here."), *JointBoneName.ToString()));
						break;
					}

					const FTransform RestRelToParentBody = GetRestTransformRelativeTo(RefSkel, JointBoneIdx, ParentBoneIdx);

					const int32 ThisBody = BodyParent.Num();
					BodyParent.Add(ParentBody);
					BodyDOFCount.Add(1);
					BodyLimbIndex.Add(ThisLimbIndex);
					JointOffsetInParent.Add(RestRelToParentBody.GetTranslation());
					RestRotInParent.Add(RestRelToParentBody.GetRotation());
					// Yaw = local -Y in the parent bone's own rest frame, per
					// the rig's joint orient convention: +X points toward the
					// child bone, +Z points into the interior of the range of
					// motion, +Y flips sign between the _L/_R sides. The axis
					// is negated (-Y, not +Y) because FQuat(Axis,Angle)'s
					// rotation direction is fixed by UE's cross-product
					// convention (X x Y = Z), which rotates +X TOWARD -Z for
					// a positive angle about +Y — the opposite of "+X toward
					// +Z" above. Using -Y makes increasing JointPos rotate
					// +X toward +Z instead, matching both
					// FMassMuscleViewportClient::DrawSelectedArc's Yaw arc
					// (ArcXAxis=local X, ArcYAxis=local Z, angle sweeps X->Z)
					// and ComputeMuscleMultipliers's use of Batch.JointPos
					// (converted straight to degrees) against the SAME
					// MinRange/MaxRange authored in that same tool/convention
					// — those two only make sense if JointPos's sign matches
					// the editor's angle sign exactly.
					JointAxisLocal.Add(FVector(0.0f, -1.0f, 0.0f));
					Mass.Add(GetMass(JointBoneName));
					DebugBoneName.Add(JointBoneName);

					ParentBody = ThisBody;
					ParentBoneIdx = JointBoneIdx;
				}

				// ---- Fused Tip: capture its rest-pose offset from the last
				// real body created above (ParentBody, e.g. Feet/FHand) —
				// see BodyFusedTipOffset's comment on FCreatureTopology. ----
				{
					const FName TipBoneName(Archetype.JointBones.Last().ToString() + Side);
					const int32 TipBoneIdx = RefSkel.FindBoneIndex(TipBoneName);
					if (TipBoneIdx != INDEX_NONE)
					{
						FusedTipOffsetByBody.Add(ParentBody, GetRestTransformRelativeTo(RefSkel, TipBoneIdx, ParentBoneIdx).GetTranslation());
					}
				}
			}
		}

		OutTopology.NumLimbs = Archetypes.Num() * 2;
		OutTopology.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

		for (const TPair<int32, FVector>& Pair : FusedTipOffsetByBody)
		{
			OutTopology.BodyFusedTipOffset[Pair.Key] = Pair.Value;
		}

		// ---- Torso (body 0): fuse spine/head/mouth only — limb mount bones
		// are now their own real ABA bodies, not fused in here (see
		// FLimbArchetype's comment). ----
		{
			float TorsoMass = 0.0f;
			for (const FName& BoneName : GetMutoTorsoBones())
			{
				TorsoMass += GetMass(BoneName);
			}
			Mass[0] = TorsoMass;
			// See the comment on TorsoRestInComponentSpace above — this is the
			// only place body 0's own BodyRestRotInParent slot gets set (the
			// per-body loop below starts at Body=1).
			OutTopology.BodyRestRotInParent[0] = RestRotInParent[0];

			// Placeholder torso inertia, same thin-rod approximation as the
			// limbs below, using the spine's overall length (Pelvis -> Head1) as
			// the proxy "L". See the function comment: no real inertia source
			// exists yet.
			const int32 Head1Idx = RefSkel.FindBoneIndex(TEXT("Head1"));
			const float SpineLength = Head1Idx != INDEX_NONE
				? static_cast<float>(GetRestTransformRelativeTo(RefSkel, Head1Idx, PelvisIdx).GetTranslation().Size())
				: 100.0f;
			const float IPerp = (1.0f / 12.0f) * TorsoMass * SpineLength * SpineLength;
			OutTopology.BodyMass[0] = TorsoMass;
			OutTopology.BodyLocalCoMOffset[0] = FVector::ZeroVector;
			OutTopology.BodyInertiaDiagLocal[0] = FVector(0.05f * IPerp, IPerp, IPerp);
		}

		// ---- Limb bodies: per-body topology fields + CoM/inertia from each body's own offset ----
		for (int32 Body = 1; Body < OutTopology.NumBodies; ++Body)
		{
			OutTopology.BodyJointOffsetInParent[Body] = JointOffsetInParent[Body];
			OutTopology.BodyRestRotInParent[Body] = RestRotInParent[Body];
			OutTopology.BodyJointAxisLocal[Body] = JointAxisLocal[Body];
			OutTopology.BodyMass[Body] = Mass[Body];
			OutTopology.BodyRadius[Body] = GetRadius(DebugBoneName[Body]);
			OutTopology.BodyCapsuleHalfHeight[Body] = GetCapsuleHalfHeight(DebugBoneName[Body]);

			// CoM offset placeholder: half the offset to this body's own child if
			// it has exactly one (the common case — a chain), or half of its own
			// incoming offset for the leaf at the end of each limb (no child to
			// measure against). Both are expressed in this body's own rest frame,
			// which is the same frame BodyJointOffsetInParent[Child] is already in
			// (see the function comment on why that equivalence holds).
			int32 ChildBody = INDEX_NONE;
			for (int32 Candidate = Body + 1; Candidate < OutTopology.NumBodies; ++Candidate)
			{
				if (OutTopology.BodyParent[Candidate] == Body) { ChildBody = Candidate; break; }
			}
			const FVector LengthVec = (ChildBody != INDEX_NONE) ? JointOffsetInParent[ChildBody] : JointOffsetInParent[Body];
			OutTopology.BodyLocalCoMOffset[Body] = 0.5f * LengthVec;

			const float BodyLength = static_cast<float>(LengthVec.Size());
			const float IPerp = (1.0f / 12.0f) * Mass[Body] * BodyLength * BodyLength;
			OutTopology.BodyInertiaDiagLocal[Body] = FVector(0.05f * IPerp, IPerp, IPerp);
		}

		// ---- Per-DOF muscle strength curves, one authored FMassMuscleDataMuscle
		// per DOF: BoneName IS this body — CORRECTED 2026-08-11 (was
		// ChildBoneName, i.e. backwards). User-confirmed directly (concrete
		// example: "Feet_muscle_Yaw_R" has Attached bone=Feet_R, Linked to
		// bone=FeetTip_R, and is meant to rotate FEET around its own local Y
		// axis — moving FeetTip in the world as a rigid consequence, never
		// rotating FeetTip itself, since FeetTip has no muscle attached to it
		// at all) and independently verified against the full data for both
		// the F and Hips chains (a throwaway dump test, deleted after use):
		// every real body's OWN muscle is the one named/BoneName'd after
		// itself (e.g. Body=Feet_L's driving muscle has BoneName=Feet_L), not
		// the one whose ChildBoneName matches. ChildBoneName is purely a
		// downstream reference (which bone the arc-drawing tool points
		// toward) with no bearing on which body the muscle actually drives.
		// This also explains why the mount bone is now its own ball-joint
		// body (see FLimbArchetype) and why the Tip bone at the end of each
		// chain never gets one (see the per-limb loop above) — both are
		// direct, structural consequences of this same correction, not
		// separate fixes. Orientation picks which axis — Roll/Yaw/Pitch =
		// local X/Y/Z (this rig's joint orient convention), matching the
		// ball joint's 3 DOF slots (0=X,1=Y,2=Z, see CreatureBatchState.h's
		// JointRelRot comment) or the revolute's 1 slot (always Yaw, checked
		// above). ----
		if (MuscleAsset)
		{
			auto FindMuscle = [MuscleAsset](FName ThisBoneName, ERotationType Orient) -> const FMassMuscleDataMuscle*
			{
				for (const FMassMuscleDataMuscle& M : MuscleAsset->Muscles)
				{
					if (M.BoneName == ThisBoneName && M.Orientation == Orient)
					{
						return &M;
					}
				}
				return nullptr;
			};
			auto ApplyMuscleToDOF = [&OutTopology](int32 DOFIdx, const FMassMuscleDataMuscle* M)
			{
				if (!M) return;
				OutTopology.DOFExtensionCurve[DOFIdx] = M->ExtensionStrengthCurve;
				OutTopology.DOFFlexionCurve[DOFIdx] = M->FlexionStrengthCurve;
				OutTopology.DOFRangeMinDeg[DOFIdx] = M->MinRange;
				// Ranges that cross the 0/360 boundary are authored with Min >
				// Max (e.g. [315.8, 52.2] meaning -44.2..52.2) — unwrap Max so
				// it's always > Min, same convention DrawSelectedArc uses.
				OutTopology.DOFRangeMaxDeg[DOFIdx] = (M->MaxRange > M->MinRange) ? M->MaxRange : M->MaxRange + 360.0f;
				OutTopology.DOFHasMuscleCurve[DOFIdx] = 1;
			};

			for (int32 Body = 1; Body < OutTopology.NumBodies; ++Body)
			{
				const FName& ThisBoneName = DebugBoneName[Body];
				const int32 DOFOffset = OutTopology.BodyDOFOffset[Body];
				if (OutTopology.BodyDOFCount[Body] == 3)
				{
					ApplyMuscleToDOF(DOFOffset + 0, FindMuscle(ThisBoneName, ERotationType::Roll));
					ApplyMuscleToDOF(DOFOffset + 1, FindMuscle(ThisBoneName, ERotationType::Yaw));
					ApplyMuscleToDOF(DOFOffset + 2, FindMuscle(ThisBoneName, ERotationType::Pitch));
				}
				else
				{
					ApplyMuscleToDOF(DOFOffset, FindMuscle(ThisBoneName, ERotationType::Yaw));
				}
			}
		}

		if (OutBodyDebugNames)
		{
			*OutBodyDebugNames = MoveTemp(DebugBoneName);
		}

		return true;
	}
}

#endif // WITH_EDITOR
