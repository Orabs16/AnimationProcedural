// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 008.
//
// Audits the skeleton against the ABA topology to answer: does any bone that
// is NOT an ABA body still influence the simulation, and is any authored mass
// being silently dropped?
//
// Three concrete risks this checks, all of which would be invisible at runtime:
//
//  1. DROPPED MASS. BuildMutoTopology creates a body for each mount bone and
//     each chain bone EXCEPT THE LAST (the Tip bones get none). Torso mass is
//     the sum over GetMutoTorsoBones() only. Any bone carrying authored mass
//     that is neither an ABA body nor a summed torso bone contributes nothing —
//     the creature is lighter than the asset says, silently.
//
//  2. THE ROOT CHAIN ABOVE PELVIS. TorsoRestInComponentSpace is built with
//     GetRestTransformRelativeTo(..., INDEX_NONE), which composes all the way
//     to the skeleton root. A null/dummy bone above Pelvis therefore bakes its
//     rotation (and any scale) into body 0's rest orientation, i.e. into what
//     "standing" means.
//
//  3. NON-UNIT SCALE. Every offset/inertia in the solver assumes bone rest
//     transforms are pure rotation+translation. A scaled bone silently changes
//     lengths, and inertia scales with length SQUARED.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.SkeletonAudit; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=skelaudit.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "CreatureBatchState.h"
#include "CreatureGroundContact.h"

#if WITH_EDITOR
#include "MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoSkeletonAudit,
	"AgentSolver.TEMP.SkeletonAudit",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoSkeletonAudit::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Topo;
	TArray<FString> Warnings;
	TArray<FName> BodyDebugNames;
	if (!TestTrue(TEXT("BuildMutoTopology succeeded"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))) return false;

	const FReferenceSkeleton& RefSkel = SkeletalMesh->GetRefSkeleton();
	const int32 NumBones = RefSkel.GetNum();
	const int32 PelvisIdx = RefSkel.FindBoneIndex(TEXT("Pelvis"));

	// ---- 1. Full hierarchy, flagging what the solver does and does not use ----
	TSet<FName> BodyBones;
	for (const FName& N : BodyDebugNames) BodyBones.Add(N);
	TSet<FName> TorsoBones;
	for (const FName& N : MutoTopology::GetMutoTorsoBones()) TorsoBones.Add(N);

	// Tip bones are not bodies, but their mass IS fused into their parent body
	// (see MutoTopology.h). Count them as simulated, or this audit reports its
	// own fix as still-dropped mass.
	TSet<FName> FusedTipBones;
	for (const MutoTopology::FLimbArchetype& A : MutoTopology::GetMutoLimbArchetypes())
	{
		for (const TCHAR* Side : { TEXT("_L"), TEXT("_R") })
		{
			FusedTipBones.Add(FName(A.JointBones.Last().ToString() + Side));
		}
	}

	AddInfo(FString::Printf(TEXT("Skeleton has %d bones; ABA topology has %d bodies, %d DOF."),
		NumBones, Topo.NumBodies, Topo.NumDOF));
	AddInfo(FString::Printf(TEXT("Pelvis is bone index %d."), PelvisIdx));

	// ---- 2. The chain ABOVE Pelvis (bones that are ancestors of Pelvis) ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- ANCESTORS OF PELVIS (these compose into body 0's rest rotation) ----"));
	{
		TArray<int32> Chain;
		int32 Walk = PelvisIdx;
		while (Walk != INDEX_NONE)
		{
			Chain.Insert(Walk, 0);
			Walk = RefSkel.GetParentIndex(Walk);
		}
		for (int32 Idx : Chain)
		{
			const FTransform& T = RefSkel.GetRefBonePose()[Idx];
			const FRotator R = T.GetRotation().Rotator();
			AddInfo(FString::Printf(
				TEXT("  [%3d] %-16s parent=%3d  T=(%8.2f %8.2f %8.2f)  R=(%7.2f %7.2f %7.2f)  S=(%.3f %.3f %.3f)%s"),
				Idx, *RefSkel.GetBoneName(Idx).ToString(), RefSkel.GetParentIndex(Idx),
				T.GetTranslation().X, T.GetTranslation().Y, T.GetTranslation().Z,
				R.Pitch, R.Yaw, R.Roll,
				T.GetScale3D().X, T.GetScale3D().Y, T.GetScale3D().Z,
				(Idx == PelvisIdx) ? TEXT("   <- Pelvis (body 0)") : TEXT("   <- ANCESTOR")));
		}
		const FTransform Composed = MutoTopology::GetRestTransformRelativeTo(RefSkel, PelvisIdx, INDEX_NONE);
		const FRotator CR = Composed.GetRotation().Rotator();
		AddInfo(FString::Printf(
			TEXT("  => composed Pelvis-in-component: T=(%.2f %.2f %.2f) R=(%.2f %.2f %.2f) S=(%.3f %.3f %.3f)"),
			Composed.GetTranslation().X, Composed.GetTranslation().Y, Composed.GetTranslation().Z,
			CR.Pitch, CR.Yaw, CR.Roll,
			Composed.GetScale3D().X, Composed.GetScale3D().Y, Composed.GetScale3D().Z));
		AddInfo(TEXT("  (this IS Topo.BodyRestRotInParent[0] — what 'standing upright' means)"));
	}

	// ---- 3. Mass accounting: authored vs actually simulated ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- MASS ACCOUNTING: every bone with authored mass ----"));
	float AuthoredTotal = 0.0f, SimulatedTotal = 0.0f, DroppedTotal = 0.0f;
	TArray<FString> DroppedRows;

	for (int32 Idx = 0; Idx < NumBones; ++Idx)
	{
		const FName BoneName = RefSkel.GetBoneName(Idx);
		const int32 MassIdx = MassAsset->FindBoneByName(BoneName);
		if (MassIdx == INDEX_NONE) continue;

		const float M = MassAsset->Mass[MassIdx].Mass;
		AuthoredTotal += M;

		const bool bIsBody = BodyBones.Contains(BoneName);
		const bool bIsTorso = TorsoBones.Contains(BoneName);
		const bool bIsFusedTip = FusedTipBones.Contains(BoneName);
		if (bIsBody || bIsTorso || bIsFusedTip)
		{
			SimulatedTotal += M;
		}
		else if (M > 0.0f)
		{
			DroppedTotal += M;
			DroppedRows.Add(FString::Printf(
				TEXT("  [%3d] %-16s mass=%8.2f  canTouchGround=%d   <- NOT a body, NOT torso, NOT a fused tip"),
				Idx, *BoneName.ToString(), M, MassAsset->Mass[MassIdx].CanTouchGround ? 1 : 0));
		}
	}

	if (DroppedRows.Num() > 0)
	{
		AddInfo(FString::Printf(TEXT("  %d bone(s) carry authored mass that the solver NEVER SEES:"), DroppedRows.Num()));
		for (const FString& Row : DroppedRows) AddInfo(Row);
	}
	else
	{
		AddInfo(TEXT("  No dropped mass — every authored bone is either an ABA body or summed into the torso."));
	}

	float TopoTotal = 0.0f;
	for (int32 B = 0; B < Topo.NumBodies; ++B) TopoTotal += Topo.BodyMass[B];

	AddInfo(FString::Printf(TEXT("  authored total  = %10.2f kg"), AuthoredTotal));
	AddInfo(FString::Printf(TEXT("  simulated total = %10.2f kg  (Topo.BodyMass sum = %.2f)"), SimulatedTotal, TopoTotal));
	AddInfo(FString::Printf(TEXT("  DROPPED         = %10.2f kg  (%.1f%% of authored)"),
		DroppedTotal, AuthoredTotal > 0.0f ? 100.0f * DroppedTotal / AuthoredTotal : 0.0f));

	// ---- 4. Non-unit scale anywhere in the skeleton ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- NON-UNIT SCALE CHECK (inertia scales with length^2) ----"));
	int32 ScaledCount = 0;
	for (int32 Idx = 0; Idx < NumBones; ++Idx)
	{
		const FVector S = RefSkel.GetRefBonePose()[Idx].GetScale3D();
		if (!FMath::IsNearlyEqual((float)S.X, 1.0f, 1e-4f) ||
			!FMath::IsNearlyEqual((float)S.Y, 1.0f, 1e-4f) ||
			!FMath::IsNearlyEqual((float)S.Z, 1.0f, 1e-4f))
		{
			++ScaledCount;
			AddInfo(FString::Printf(TEXT("  [%3d] %-16s scale=(%.4f %.4f %.4f)"),
				Idx, *RefSkel.GetBoneName(Idx).ToString(), S.X, S.Y, S.Z));
		}
	}
	if (ScaledCount == 0) AddInfo(TEXT("  All bones have unit scale — offsets and inertia are safe."));

	// ---- 5. Bones with ZERO or missing mass that ARE simulated ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- ABA BODIES WITH ZERO / MISSING AUTHORED MASS ----"));
	int32 ZeroCount = 0;
	for (int32 B = 0; B < Topo.NumBodies; ++B)
	{
		if (Topo.BodyMass[B] > KINDA_SMALL_NUMBER) continue;
		++ZeroCount;
		AddInfo(FString::Printf(TEXT("  body %2d %-16s mass=%.4f  inertia=%s"),
			B, BodyDebugNames.IsValidIndex(B) ? *BodyDebugNames[B].ToString() : TEXT("?"),
			Topo.BodyMass[B], *Topo.BodyInertiaDiagLocal[B].ToString()));
	}
	if (ZeroCount == 0) AddInfo(TEXT("  None — every simulated body has positive mass."));

	// ---- 6. Contact points whose body is NOT the bone that owns the geometry ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- CONTACT POINTS (offset comes from a FUSED tip bone, not the body itself) ----"));
	const TArray<CreatureGroundContact::FContactPointDef> Points =
		CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	for (const CreatureGroundContact::FContactPointDef& P : Points)
	{
		AddInfo(FString::Printf(
			TEXT("  body %2d %-14s localOffset=%9.2f  radius=%6.2f  (offset is the fused Tip's rest position)"),
			P.BodyIndex, *P.DebugName.ToString(), P.LocalOffset.Size(), P.Radius));
	}

	// ---- 7. DOF RANGE AUDIT ----
	// ClampJointLimits only clamps when Wrapped > Width + eps, and Wrapped is
	// always in [0,360). So any DOF whose Width >= 360 can NEVER be clamped —
	// that joint is completely unconstrained. Width comes straight from the
	// authored values via
	//     MaxDeg = (MaxRange > MinRange) ? MaxRange : MaxRange + 360
	// with no upper bound, so a wide or degenerate authored pair silently
	// disables the limit rather than erroring.
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- DOF RANGE AUDIT (Width>=360 => limit can NEVER fire) ----"));
	{
		int32 NoCurve = 0, BadWidth = 0, WideWidth = 0, OutOfBand = 0;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 Off = Topo.BodyDOFOffset[Body];
			for (int32 k = 0; k < Topo.BodyDOFCount[Body]; ++k)
			{
				const int32 DOF = Off + k;
				if (!Topo.DOFHasMuscleCurve[DOF]) { ++NoCurve; continue; }

				const float MinD = Topo.DOFRangeMinDeg[DOF];
				const float MaxD = Topo.DOFRangeMaxDeg[DOF];
				const float Width = MaxD - MinD;

				const bool bBad = (Width <= 0.0f);
				const bool bWide = (Width >= 360.0f);
				const bool bBand = (MinD < -360.0f || MinD > 360.0f || MaxD < -360.0f || MaxD > 720.0f);
				if (bBad) ++BadWidth;
				if (bWide) ++WideWidth;
				if (bBand) ++OutOfBand;

				if (bBad || bWide || bBand)
				{
					AddInfo(FString::Printf(
						TEXT("  !! DOF %2d body %2d %-14s min=%8.2f max=%8.2f width=%8.2f %s%s%s"),
						DOF, Body, BodyDebugNames.IsValidIndex(Body) ? *BodyDebugNames[Body].ToString() : TEXT("?"),
						MinD, MaxD, Width,
						bBad ? TEXT("[WIDTH<=0] ") : TEXT(""),
						bWide ? TEXT("[NEVER CLAMPS] ") : TEXT(""),
						bBand ? TEXT("[OUT OF BAND]") : TEXT("")));
				}
			}
		}
		AddInfo(FString::Printf(
			TEXT("  summary: %d DOF total | %d without curves (UNLIMITED by design) | %d width<=0 | %d width>=360 | %d out of band"),
			Topo.NumDOF, NoCurve, BadWidth, WideWidth, OutOfBand));

		// Min/max/mean width across the DOFs that DO have ranges.
		float MinW = TNumericLimits<float>::Max(), MaxW = 0.0f, SumW = 0.0f; int32 N = 0;
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			if (!Topo.DOFHasMuscleCurve[DOF]) continue;
			const float W = Topo.DOFRangeMaxDeg[DOF] - Topo.DOFRangeMinDeg[DOF];
			MinW = FMath::Min(MinW, W); MaxW = FMath::Max(MaxW, W); SumW += W; ++N;
		}
		if (N > 0)
		{
			AddInfo(FString::Printf(TEXT("  widths over %d ranged DOF: min=%.2f max=%.2f mean=%.2f"),
				N, MinW, MaxW, SumW / N));
		}

		// Raw authored pairs, to see whether Min<Max holds BEFORE unwrapping.
		AddInfo(TEXT("  ---- raw authored MinRange/MaxRange (pre-unwrap) ----"));
		int32 RawInverted = 0, RawOutside = 0;
		for (const FMassMuscleDataMuscle& M : MuscleAsset->Muscles)
		{
			const bool bInv = (M.MaxRange <= M.MinRange);
			const bool bOut = (M.MinRange < -360.0f || M.MinRange > 360.0f ||
			                   M.MaxRange < -360.0f || M.MaxRange > 360.0f);
			if (bInv) ++RawInverted;
			if (bOut) ++RawOutside;
			if (bOut)
			{
				AddInfo(FString::Printf(TEXT("  !! muscle %-24s bone %-14s min=%8.2f max=%8.2f  OUTSIDE [-360,360]"),
					*M.Name.ToString(), *M.BoneName.ToString(), M.MinRange, M.MaxRange));
			}
		}
		AddInfo(FString::Printf(
			TEXT("  %d muscles authored | %d with MaxRange<=MinRange (unwrapped by +360) | %d outside [-360,360]"),
			MuscleAsset->Muscles.Num(), RawInverted, RawOutside));
	}

	if (Warnings.Num() > 0)
	{
		AddInfo(TEXT(""));
		AddInfo(TEXT("---- BuildMutoTopology WARNINGS ----"));
		for (const FString& W : Warnings) AddInfo(FString::Printf(TEXT("  %s"), *W));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
