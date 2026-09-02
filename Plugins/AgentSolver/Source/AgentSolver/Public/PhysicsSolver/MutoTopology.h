#pragma once

// Editor-only: builds an FCreatureTopology from Muto's authored rig/profile
// assets (USkeletalMesh + UMassMuscleProfileAssetMuscle/Mass, both from the
// MassMuscleProfile plugin, an Editor-type module — see AgentSolver.Build.cs).
// This is a Muto-specific data-prep step, not a generic rig importer; a
// generic FReferenceSkeleton-walking tool is a separate, later piece of work.
#if WITH_EDITOR

#include "CoreMinimal.h"
#include "PhysicsSolver/CreatureBatchState.h"
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

		/**
		 * The mount bone's REAL anatomical parent, now that the spine is
		 * independently articulated (see BuildMutoTopology's spine-body
		 * construction phase) instead of being one fused torso body. F/M
		 * mount onto the neck/back, not directly onto Pelvis -- this is what
		 * makes the spine's flex actually reach the limbs (the front legs
		 * follow the neck as it moves) rather than leaving them locked to
		 * the pelvis while the neck articulates independently.
		 */
		FName ParentBone;
	};

	inline TArray<FLimbArchetype> GetMutoLimbArchetypes()
	{
		return {
			{ TEXT("FShoulder"), { TEXT("FElbow"), TEXT("FElbow2"), TEXT("FElbow3"), TEXT("FHand"), TEXT("FTip") }, TEXT("Head1") },
			{ TEXT("BShoulder"), { TEXT("BElbow1"), TEXT("BElbow2"), TEXT("BElbow3"), TEXT("BHand"), TEXT("BTip") }, TEXT("Back3") },
			{ TEXT("MShoulder"), { TEXT("MElbow"), TEXT("MHand"), TEXT("MTip") }, TEXT("Back2") },
			{ TEXT("Hips"), { TEXT("Knee1"), TEXT("Knee2"), TEXT("Feet"), TEXT("FeetTip") }, TEXT("Pelvis") },
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
	 * Torso-chain bones with no real skeletal child of their own — the end of
	 * their branch, same structural role as a limb's "Tip" bone (see
	 * FLimbArchetype's comment). No muscle is ever authored to drive them
	 * independently (confirmed: this is why they'd otherwise get zero muscle
	 * curves and, per ClampJointLimits, no joint-angle limit at all — a real
	 * bug, found 2026-08-16 when UpperMouth/Chin free-spun to ~50 rad/s under
	 * passive gravity alone and dragged the rest of the chain into a
	 * divergence). User-confirmed (2026-08-16): treat them exactly like a
	 * limb's Tip bone — no independent ABA body, fused rigidly (mass + rest
	 * offset) into their real parent instead.
	 */
	inline TArray<FName> GetMutoTorsoLeafBones()
	{
		return { TEXT("UpperMouth"), TEXT("Chin") };
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
	 * Diagonal inertia about the CoM of a SOLID CAPSULE of uniform density: a
	 * cylinder of radius R and length L, capped by a hemisphere of the same radius
	 * at each end, lying along its own LOCAL +X axis. Returns (Iaxial, Iperp, Iperp).
	 *
	 * This replaced a thin-rod approximation on 2026-08-21. The rod form had no
	 * thickness at all, so its axial term was identically zero and was faked as
	 * `0.05 * Iperp` "for numerical safety" -- a magic number standing in for a
	 * shape nobody had measured. Solving 0.5*m*r^2 == 0.05*(1/12)*m*L^2 shows what
	 * that fudge was actually claiming: **every bone is exactly L/sqrt(120) thick,
	 * i.e. r = 0.0913*L**. The authored collision radii disagree with that badly and
	 * in both directions -- Back3 is r/L = 0.85 (9x fatter than assumed) while Feet
	 * is 0.061 (1.5x thinner) -- so the fudge was not a conservative approximation,
	 * it was a different creature.
	 *
	 * R comes from FMassMuscleDataMass::Radius, the SAME authored value ground and
	 * limb collision already use to place contact points. That is the point: one
	 * authored number now decides how thick a bone is for collision AND for
	 * inertia, instead of collision reading real geometry while the mass model
	 * silently assumed a different one.
	 *
	 * L is the BONE length (the distance to this body's own child joint), not the
	 * collision capsule's length. Those are deliberately different:
	 * BodyCapsuleHalfHeight defaults to 0, which collapses the COLLISION capsule to
	 * a sphere at the tip, but the bone's MASS still spans the whole segment. Using
	 * the collision length here would give most bones a point-like mass
	 * distribution, which is worse than the rod it replaced.
	 *
	 * The cylinder-length + two-caps convention matches MutoMassAuthoringDumpTest's
	 * volume formula exactly (pi*r^2*L + (4/3)*pi*r^3), so the shape this derives
	 * inertia from is the same shape that test derives its suggested MASS from.
	 *
	 * A non-positive radius falls back to the OLD THIN ROD VERBATIM, magic 0.05
	 * and all, so a bone with no authored radius (Pelvis, at time of writing) does
	 * not silently change behaviour just because this function was introduced.
	 *
	 * The first attempt at that fallback picked the equivalent radius L/sqrt(120)
	 * instead, on the reasoning that it solves 0.5*m*r^2 == 0.05*(1/12)*m*L^2. It
	 * does -- for a CYLINDER. Feeding it to a capsule misses by 2.2% axially and
	 * 29% transversely, because the caps take a share of the mass and then sit
	 * outside L where they contribute a large parallel-axis term. Caught by the
	 * test that asserts this paragraph rather than trusting it.
	 */
	inline FVector CapsuleInertiaDiagLocal(float MassKg, float RadiusCm, float LengthCm)
	{
		const float L = FMath::Max(LengthCm, 0.0f);
		if (MassKg <= 0.0f || (RadiusCm <= 0.0f && L <= 0.0f))
		{
			return FVector::ZeroVector;
		}

		if (RadiusCm <= 0.0f)
		{
			// Legacy path, exact by construction rather than by an equivalence
			// argument. Reached only by a bone whose collision radius nobody has
			// authored; it disappears the moment one is. The caller is expected to
			// warn -- silently substituting a made-up thickness is how the 0.05 got
			// to live here unexamined for as long as it did.
			const float RodPerp = MassKg * L * L / 12.0f;
			return FVector(0.05f * RodPerp, RodPerp, RodPerp);
		}

		const float R = RadiusCm;

		// Split the mass between the cylinder and the two caps by VOLUME, i.e.
		// assume uniform density -- the authored Mass is a single number for the
		// whole bone, so there is nothing better to go on.
		const float R2 = R * R;
		const float CylVolume = PI * R2 * L;
		const float CapVolume = (4.0f / 3.0f) * PI * R2 * R;
		const float TotalVolume = CylVolume + CapVolume;
		if (TotalVolume <= SMALL_NUMBER)
		{
			return FVector::ZeroVector;
		}
		const float CylMass = MassKg * (CylVolume / TotalVolume);
		const float CapMass = MassKg * (CapVolume / TotalVolume); // both hemispheres together = one sphere

		// Axial: cylinder (1/2)m r^2, plus the two caps which together form a full
		// sphere about that same axis, (2/5)m r^2.
		const float IAxial = 0.5f * CylMass * R2 + 0.4f * CapMass * R2;

		// Transverse: cylinder about its own centre, plus the caps carried out to
		// the ends. The 3*L*R/8 cross term is the hemisphere's own centroid offset
		// (3R/8 from its flat face) beating against the L/2 half-length -- dropping
		// it would under-report the transverse inertia of every stubby bone, which
		// on this rig is most of the spine.
		const float IPerp =
			CylMass * (L * L / 12.0f + R2 * 0.25f)
			+ CapMass * (0.4f * R2 + L * L * 0.25f + 0.375f * L * R);

		return FVector(IAxial, IPerp, IPerp);
	}

	/**
	 * Builds an FCreatureTopology for Muto from its rig + authored profile data.
	 *
	 * Body 0 is Pelvis alone (2026-08-16: previously fused the whole spine/
	 * head/mouth chain into body 0; that chain is now independently
	 * articulated -- see the spine-body construction phase below). Every
	 * limb's mount bone is its own real ABA body (see FLimbArchetype's
	 * comment on the Bone/Child semantics correction). Each of the 8 limbs
	 * (GetMutoLimbArchetypes, x2 for _L/_R) becomes a chain of ABA bodies:
	 * the mount bone as a 3-DOF ball joint (parented to its real anatomical
	 * spine bone per FLimbArchetype::ParentBone, not always Pelvis), then
	 * every remaining JointBones entry EXCEPT THE LAST as a 1-DOF Yaw
	 * revolute — the last entry (the limb's Tip bone) gets no body at all,
	 * since no muscle is ever authored to independently rotate it (see
	 * FLimbArchetype). Total body/DOF count per limb is unchanged by this —
	 * it's still "one 3-DOF ball + N 1-DOF revolutes" either way, just
	 * shifted by one link (mount in, Tip out).
	 *
	 * The spine/head/jaw chain (GetMutoTorsoBones, minus Pelvis) is built as
	 * 6 more 3-DOF ball-joint bodies (Back1-3, Head1-2, LowerMouth), each
	 * parented to its REAL skeletal parent (derived from
	 * RefSkel.GetParentIndex, not assumed from GetMutoTorsoBones' authoring
	 * order — that list is flat/unordered and does not itself encode the
	 * branch at Head2 -> {UpperMouth, LowerMouth}). Built BEFORE the limb
	 * loop below, since F/B/M's mount bones need their real spine parent's
	 * ABA body index to already exist. UpperMouth and Chin (GetMutoTorsoLeafBones)
	 * are the end of their branch with no muscle ever authored to drive them
	 * independently — same structural role as a limb's Tip bone, and now
	 * treated identically: no ABA body, fused rigidly into their real parent
	 * (Head2, LowerMouth) instead. (2026-08-16: they were briefly built as
	 * real, unconstrained bodies — zero muscle curve meant zero joint-limit
	 * too, per ClampJointLimits — and free-spun to ~50 rad/s under passive
	 * gravity alone, dragging Head1 and the front arm mounted on it into a
	 * divergence. User-identified root cause: they're leaves with no child,
	 * exactly like Tip/FeetTip, and should never have been independent
	 * bodies in the first place.)
	 *
	 * Per-body mass comes directly from MassAsset (currently a uniform 1.0
	 * placeholder for every bone in the asset — this function doesn't invent
	 * different numbers, just sums/reads what's there so it stays traceable
	 * back to the source asset once real mass values get authored).
	 *
	 * Inertia is derived (CapsuleInertiaDiagLocal) from three things that ARE
	 * authored: the bone's mass, its collision radius, and its rest-pose length.
	 * Still a derivation rather than a measurement — nobody has authored an
	 * inertia tensor — but as of 2026-08-21 it no longer contains a magic
	 * constant, and the thickness it assumes is the same one collision uses
	 * instead of a different, implicit one.
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

		// Authored per-bone data is keyed on the REFERENCE-SKELETON BONE INDEX,
		// not the bone name. Every call site below already resolved a name to an
		// index to walk the skeleton, so passing that index on costs nothing and
		// makes FMassMuscleDataMass::BoneIndex load-bearing -- it was previously
		// written by InitializeFromSkeletalMesh and read by nothing, i.e. free to
		// drift out of date after a mesh re-import with no symptom at all.
		// UMassMuscleProfileAssetMass::PostLoad now repairs it on load.
		auto GetMass = [MassAsset](int32 BoneIdx) -> float
		{
			if (!MassAsset) return 1.0f;
			const int32 Idx = MassAsset->FindBoneByIndex(BoneIdx);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].Mass : 1.0f;
		};
		auto GetRadius = [MassAsset](int32 BoneIdx) -> float
		{
			if (!MassAsset) return 15.0f;
			const int32 Idx = MassAsset->FindBoneByIndex(BoneIdx);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].Radius : 15.0f;
		};
		auto GetCapsuleHalfHeight = [MassAsset](int32 BoneIdx) -> float
		{
			if (!MassAsset) return 0.0f;
			const int32 Idx = MassAsset->FindBoneByIndex(BoneIdx);
			return Idx != INDEX_NONE ? MassAsset->Mass[Idx].CapsuleHalfHeight : 0.0f;
		};

		// Sanity-check every limb's authored data against this builder's
		// assumption (the joint pivoting AT the mount bone = 3-DOF ball,
		// every subsequent joint = 1-DOF Yaw revolute) — all 4 archetypes
		// are authored now, not just F, so check all of them instead of
		// just F. Restricted to limb bones (mount + chain, both sides) —
		// the spine/head/jaw chain (Back*/Head*/Mouth) has no revolute
		// bodies at all (every spine body is a 3-DOF ball), so this
		// particular check has nothing to validate there; that data is
		// picked up generically by the per-body muscle-curve lookup below
		// instead, same as any other ball-joint body.
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
		TArray<float> Mass = { 0.0f }; // Pelvis's own mass filled in below
		TArray<FName> DebugBoneName = { TEXT("Pelvis") }; // body 0 is now just Pelvis, not a fused "Torso" blob
		TArray<int32> DebugBoneIdx = { PelvisIdx };          // parallel to DebugBoneName -- the lookup key for authored per-bone data
		TMap<int32, FVector> FusedTipOffsetByBody; // see BodyFusedTipOffset's comment — applied to OutTopology after Build()
		TMap<int32, float> FusedTipMassByBody;    // the Tip's authored mass, fused into its parent body below

		// ---- Spine/head/jaw: independently-articulated 3-DOF ball joints ----
		// Built BEFORE the limb loop below: F/B/M's mount bones need their
		// real spine parent's ABA body index to already exist (see
		// FLimbArchetype::ParentBone). Bone index Pelvis -> ABA body 0 seeds
		// the map; every other torso bone's REAL skeletal parent (via
		// RefSkel.GetParentIndex, not an assumed chain — see the function
		// comment on why) must already be in the map when its own turn
		// comes, which holds because GetMutoTorsoBones()' authoring order
		// already happens to be a valid parent-before-child order (Pelvis,
		// Back1, Back2, Back3, Head1, Head2, UpperMouth, LowerMouth, Chin) —
		// verified against the real skeleton, not assumed (a lookup failure
		// below would warn rather than silently mis-parent a bone).
		TMap<int32, int32> SpineBodyIndexByBoneIdx;
		SpineBodyIndexByBoneIdx.Add(PelvisIdx, 0);
		const TArray<FName> TorsoLeafBones = GetMutoTorsoLeafBones();
		for (const FName& BoneName : GetMutoTorsoBones())
		{
			if (BoneName == TEXT("Pelvis"))
			{
				continue; // already body 0
			}
			const int32 BoneIdx = RefSkel.FindBoneIndex(BoneName);
			if (BoneIdx == INDEX_NONE)
			{
				OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: could not find spine bone '%s' — skipping."), *BoneName.ToString()));
				continue;
			}
			const int32 ParentBoneIdx = RefSkel.GetParentIndex(BoneIdx);
			const int32* ParentBodyIdxPtr = (ParentBoneIdx != INDEX_NONE) ? SpineBodyIndexByBoneIdx.Find(ParentBoneIdx) : nullptr;
			if (!ParentBodyIdxPtr)
			{
				OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: spine bone '%s' has no already-built parent body — skipping (its real skeletal parent isn't one of the fused torso bones, or GetMutoTorsoBones() no longer lists it in parent-before-child order)."), *BoneName.ToString()));
				continue;
			}

			if (TorsoLeafBones.Contains(BoneName))
			{
				// No independent joint (see GetMutoTorsoLeafBones' comment) --
				// fuse rigidly into the real parent body instead, same
				// mechanism a limb's Tip bone already uses below.
				FusedTipOffsetByBody.Add(*ParentBodyIdxPtr, GetRestTransformRelativeTo(RefSkel, BoneIdx, ParentBoneIdx).GetTranslation());
				FusedTipMassByBody.Add(*ParentBodyIdxPtr, GetMass(BoneIdx));
				continue;
			}

			const FTransform RestRelToParentBody = GetRestTransformRelativeTo(RefSkel, BoneIdx, ParentBoneIdx);
			const int32 ThisBody = BodyParent.Num();
			BodyParent.Add(*ParentBodyIdxPtr);
			BodyDOFCount.Add(3);
			BodyLimbIndex.Add(INDEX_NONE); // not a limb
			JointOffsetInParent.Add(RestRelToParentBody.GetTranslation());
			RestRotInParent.Add(RestRelToParentBody.GetRotation());
			JointAxisLocal.Add(FVector::ZeroVector); // unused for ball joints
			Mass.Add(GetMass(BoneIdx));
			DebugBoneName.Add(BoneName);
			DebugBoneIdx.Add(BoneIdx);

			SpineBodyIndexByBoneIdx.Add(BoneIdx, ThisBody);
		}

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
				// to its REAL anatomical spine parent (FLimbArchetype::
				// ParentBone — F/M mount on the neck/back, not Pelvis; see
				// that field's comment) — see FLimbArchetype's comment on
				// the Bone/Child semantics correction (a "MountBone_muscle_
				// Roll/Yaw/Pitch" triple, BoneName==MountBone, is what
				// actually drives this). ----
				const int32 MountParentBoneIdx = RefSkel.FindBoneIndex(Archetype.ParentBone);
				const int32* MountParentBodyIdxPtr = (MountParentBoneIdx != INDEX_NONE) ? SpineBodyIndexByBoneIdx.Find(MountParentBoneIdx) : nullptr;
				if (!MountParentBodyIdxPtr)
				{
					OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: could not find mount parent body '%s' for '%s' — skipping limb."), *Archetype.ParentBone.ToString(), *MountBoneName.ToString()));
					continue;
				}

				const FTransform MountRestRelToParent = GetRestTransformRelativeTo(RefSkel, MountBoneIdx, MountParentBoneIdx);
				const int32 MountBody = BodyParent.Num();
				BodyParent.Add(*MountParentBodyIdxPtr);
				BodyDOFCount.Add(3);
				BodyLimbIndex.Add(ThisLimbIndex);
				JointOffsetInParent.Add(MountRestRelToParent.GetTranslation());
				RestRotInParent.Add(MountRestRelToParent.GetRotation());
				JointAxisLocal.Add(FVector::ZeroVector); // unused for ball joints
				Mass.Add(GetMass(MountBoneIdx));
				DebugBoneName.Add(MountBoneName);
				DebugBoneIdx.Add(MountBoneIdx);

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
					// Yaw = local -Y in THIS BONE'S OWN rest frame, per the
					// rig's joint orient convention: +X points toward the
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
					//
					// ROTATED INTO THE PARENT'S FRAME (2026-08-14). The muscle
					// that drives this joint is authored as
					// "<ThisBone>_muscle_Yaw_*" with BoneName==ThisBone, so
					// "Yaw" names an axis of THIS bone, not of its parent —
					// but BodyJointAxisLocal is consumed as
					// ParentRot.RotateVector(axis) (CreatureBatchSolver.h
					// Pass 1), i.e. in the PARENT's frame. The two frames
					// differ by exactly RestRelToParentBody, which on this rig
					// is a large rotation (Knee1: 235°), so storing a bare
					// (0,-1,0) here made the solver rotate about the PARENT's
					// yaw axis instead of the bone's own. On Knee1 those axes
					// are nearly perpendicular, and 99.3% of every commanded
					// yaw came out as ROLL of a joint that has no roll DOF —
					// user-observed in the editor, then measured. Every one of
					// the 26 revolutes leaked, 2.5% to 99.3%; the ones that
					// looked fine did so only because their rest rotation
					// happened to be nearly a pure Y rotation, which leaves Y
					// invariant. See SOLVER_DEBUG_LOG.md entry 022.
					//
					// The ball-joint path was always correct here — it builds
					// JointFrame = ParentRot * BodyRestRotInParent (the bone's
					// own frame) before interpreting its DOFs. This brings the
					// revolute path onto that same frame convention.
					JointAxisLocal.Add(RestRelToParentBody.GetRotation().RotateVector(FVector(0.0f, -1.0f, 0.0f)));
					Mass.Add(GetMass(JointBoneIdx));
					DebugBoneName.Add(JointBoneName);
					DebugBoneIdx.Add(JointBoneIdx);

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
						// The Tip has no body of its own (no muscle drives it), but
						// it still has MASS, and that mass sits out at the end of a
						// long lever where it contributes substantially to the
						// parent's inertia and shifts its CoM. Capturing it here so
						// the per-body loop below can fuse it in — without this its
						// authored mass was silently discarded (measured: 8 kg over
						// the 8 Tip bones, see SOLVER_DEBUG_LOG.md entry 008).
						FusedTipMassByBody.Add(ParentBody, GetMass(TipBoneIdx));
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

		// ---- Pelvis (body 0): its OWN mass/inertia only now — the rest of
		// the spine/head/mouth chain has its own real ABA bodies (built
		// above), not fused in here. ----
		{
			const float PelvisMass = GetMass(PelvisIdx);
			Mass[0] = PelvisMass;
			// See the comment on TorsoRestInComponentSpace above — this is the
			// only place body 0's own BodyRestRotInParent slot gets set (the
			// per-body loop below starts at Body=1).
			OutTopology.BodyRestRotInParent[0] = RestRotInParent[0];

			// Pelvis inertia, same capsule model as every other body below, using
			// the distance to Back1 (Pelvis's real spine child now, same role Head1
			// played as a rough "how big is this body" proxy when the whole chain was
			// fused) as the proxy "L".
			//
			// Pelvis is the ONE body with no authored collision radius (it has no
			// ground-contact geometry, so nothing ever needed one), which is exactly
			// the case CapsuleInertiaDiagLocal's fallback exists for: it reproduces
			// the old thin-rod numbers here rather than inventing a thickness. If a
			// radius is ever authored for Pelvis this picks it up with no code change.
			const int32 Back1Idx = RefSkel.FindBoneIndex(TEXT("Back1"));
			const float SpineLength = Back1Idx != INDEX_NONE
				? static_cast<float>(GetRestTransformRelativeTo(RefSkel, Back1Idx, PelvisIdx).GetTranslation().Size())
				: 100.0f;
			OutTopology.BodyMass[0] = PelvisMass;
			OutTopology.BodyRadius[0] = GetRadius(PelvisIdx);
			OutTopology.BodyCapsuleHalfHeight[0] = GetCapsuleHalfHeight(PelvisIdx);
			OutTopology.BodyLocalCoMOffset[0] = FVector::ZeroVector;
			if (OutTopology.BodyRadius[0] <= 0.0f)
			{
				OutWarnings.Add(FString::Printf(TEXT("BuildMutoTopology: Pelvis has no authored collision radius, so its inertia falls back to the legacy thin-rod approximation (axial term = 0.05 * transverse, a magic number with no physical source). Author FMassMuscleDataMass::Radius for Pelvis to derive it from real geometry like every other body.")));
			}
			OutTopology.BodyInertiaDiagLocal[0] = CapsuleInertiaDiagLocal(PelvisMass, OutTopology.BodyRadius[0], SpineLength);
		}

		// ---- Limb bodies: per-body topology fields + CoM/inertia from each body's own offset ----
		for (int32 Body = 1; Body < OutTopology.NumBodies; ++Body)
		{
			OutTopology.BodyJointOffsetInParent[Body] = JointOffsetInParent[Body];
			OutTopology.BodyRestRotInParent[Body] = RestRotInParent[Body];
			OutTopology.BodyJointAxisLocal[Body] = JointAxisLocal[Body];
			OutTopology.BodyMass[Body] = Mass[Body];
			OutTopology.BodyRadius[Body] = GetRadius(DebugBoneIdx[Body]);
			OutTopology.BodyCapsuleHalfHeight[Body] = GetCapsuleHalfHeight(DebugBoneIdx[Body]);

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
			const FVector OwnCoM = 0.5f * LengthVec;

			// Inertia from this body's own mass, its AUTHORED collision radius and its
			// bone length -- see CapsuleInertiaDiagLocal, which replaced a thin rod
			// whose zero-thickness axial term was faked as 0.05 * Iperp.
			const float BodyLength = static_cast<float>(LengthVec.Size());
			const FVector OwnInertiaDiag = CapsuleInertiaDiagLocal(Mass[Body], OutTopology.BodyRadius[Body], BodyLength);

			// ---- Fuse the unarticulated Tip bone, if this body has one ----
			// The Tip is not its own ABA body (no muscle drives it), but it has
			// real mass sitting at the far end of a long lever — Feet's Tip is
			// 106 cm out — so it both shifts the parent's CoM and adds a large
			// parallel-axis term to its inertia. It also carries the contact
			// point (see BuildMutoContactPoints), which means it was previously
			// pushing on the ground while contributing no mass at all: the
			// ground reaction torqued a body that did not include the geometry
			// generating it. Treated as a POINT mass at BodyFusedTipOffset —
			// the Tip's own shape is unknown and its own spin inertia is second
			// order next to m*r^2 at this lever.
			const float* TipMassPtr = FusedTipMassByBody.Find(Body);
			const FVector* TipOffsetPtr = FusedTipOffsetByBody.Find(Body);
			const float TipMass = TipMassPtr ? *TipMassPtr : 0.0f;

			if (TipMassPtr && TipOffsetPtr && TipMass > 0.0f)
			{
				const FVector TipPos = *TipOffsetPtr;
				const float TotalMass = Mass[Body] + TipMass;
				const FVector FusedCoM = (Mass[Body] * OwnCoM + TipMass * TipPos) / TotalMass;

				// Shift each part's inertia from its own CoM to the FUSED CoM.
				// Parallel axis: I += m * (|d|^2 * Identity - d (x) d). Only the
				// diagonal is kept, since FCreatureTopology stores a diagonal
				// inertia — the off-diagonal terms are small while the limb axis
				// stays roughly aligned with the offsets. See A-04 for what that
				// discards; the capsule model this now fuses into is a real shape
				// rather than the thin rod it used to be, but the STORAGE is still
				// diagonal-only, so this approximation did not go away with it.
				auto ParallelAxisDiag = [](float M, const FVector& D)
				{
					return FVector(
						M * (D.Y * D.Y + D.Z * D.Z),
						M * (D.X * D.X + D.Z * D.Z),
						M * (D.X * D.X + D.Y * D.Y));
				};

				const FVector FusedInertia =
					OwnInertiaDiag
					+ ParallelAxisDiag(Mass[Body], OwnCoM - FusedCoM)
					+ ParallelAxisDiag(TipMass, TipPos - FusedCoM);

				OutTopology.BodyMass[Body] = TotalMass;
				OutTopology.BodyLocalCoMOffset[Body] = FusedCoM;
				OutTopology.BodyInertiaDiagLocal[Body] = FusedInertia;
			}
			else
			{
				OutTopology.BodyLocalCoMOffset[Body] = OwnCoM;
				OutTopology.BodyInertiaDiagLocal[Body] = OwnInertiaDiag;
			}
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
				// The scalar strengths, which the MassMuscleProfile tool has always
				// exposed (spinboxes, dirty marking, L/R mirror propagation) but which
				// stopped here until 2026-08-21 -- only the curves were read, so every
				// per-muscle strength anyone authored had no effect on training.
				OutTopology.DOFExtensionStrength[DOFIdx] = M->ExtensionStrength;
				OutTopology.DOFFlexionStrength[DOFIdx] = M->FlexionStrength;
				OutTopology.DOFMuscleActivationThreshold[DOFIdx] = FMath::Max(M->MuscleActivationThreshold, 0.0f);
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

		// Carried on the topology itself rather than handed back as another
		// out-param, so downstream consumers (CreatureGroundContact's contact-point
		// builder) can resolve authored per-bone data without every caller having
		// to remember to thread a parallel array through.
		checkf(DebugBoneIdx.Num() == OutTopology.NumBodies, TEXT("BuildMutoTopology: DebugBoneIdx (%d) fell out of step with NumBodies (%d)"), DebugBoneIdx.Num(), OutTopology.NumBodies);
		OutTopology.BodyBoneIndex = DebugBoneIdx;

		if (OutBodyDebugNames)
		{
			*OutBodyDebugNames = MoveTemp(DebugBoneName);
		}

		return true;
	}
}

#endif // WITH_EDITOR
