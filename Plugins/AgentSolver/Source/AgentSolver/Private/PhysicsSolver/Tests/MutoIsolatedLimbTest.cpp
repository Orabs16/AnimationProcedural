// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 009.
//
// Isolates the remaining contact instability on the smallest rigs that can
// still exhibit it, instead of debugging on a 34-body collapsing ragdoll.
//
//   RIG A : torso + ONE F limb          (6 bodies)
//   RIG B : torso + BOTH F limbs        (11 bodies)
//   RIG C : torso + ONE leg (Hips)      (5 bodies)  — the limb that actually
//                                                     bears weight at rest
//   RIG D : torso + BOTH legs           (9 bodies)
//   FULL  : the whole creature          (35 bodies) — reference
//
// Sub-topologies are EXTRACTED from the real Muto topology rather than
// hand-built, so bone offsets, masses, inertia, joint axes, rest rotations and
// muscle ranges are all the authentic authored data. Only the body set changes.
//
// Each rig is run through the same battery:
//   1. contact-free, passive, under gravity  -> finite? energy bounded?
//   2. with ground contact, passive          -> finite?
// at 60 Hz single substep, and at 240 Hz.
//
// Reading it: if the small rigs stay finite where the full rig dies, the bug
// scales with body count / chain depth. If even RIG A dies, we have a minimal
// reproduction to debug directly.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.IsolatedLimb; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=isolimb.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureRLEnvironment.h"
#include "AgentSolver/MutoRLTrainingDriver.h" // ComputeDefaultStandingHeight

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoIsolatedLimb,
	"AgentSolver.TEMP.IsolatedLimb",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace
{
	/**
	 * Build a topology containing only KeepBodies (ascending, must include 0,
	 * and every kept body's parent must also be kept). All physical data is
	 * copied verbatim from Full — this changes WHICH bodies exist, nothing else.
	 */
	FCreatureTopology ExtractSubTopology(
		const FCreatureTopology& Full,
		const TArray<int32>& KeepBodies,
		const TArray<FName>& FullNames,
		TArray<FName>& OutNames)
	{
		TMap<int32, int32> OldToNew;
		for (int32 i = 0; i < KeepBodies.Num(); ++i) OldToNew.Add(KeepBodies[i], i);

		TArray<int32> Parent, DOFCount, LimbIndex;
		for (int32 i = 0; i < KeepBodies.Num(); ++i)
		{
			const int32 Old = KeepBodies[i];
			const int32 OldParent = Full.BodyParent[Old];
			Parent.Add(i == 0 ? 0 : OldToNew.FindChecked(OldParent));
			DOFCount.Add(i == 0 ? 0 : Full.BodyDOFCount[Old]);
			LimbIndex.Add(Full.BodyLimbIndex[Old]);
		}

		FCreatureTopology Sub;
		// One limb slot per kept body is more than enough; LimbStrengthScale /
		// LimbActive are indexed by it and default to 1 / active.
		Sub.NumLimbs = FMath::Max(1, KeepBodies.Num());
		Sub.Build(Parent, DOFCount, LimbIndex);

		// Remap limb indices into [0, NumLimbs) so LimbIndex() stays in range.
		for (int32 i = 0; i < KeepBodies.Num(); ++i)
		{
			Sub.BodyLimbIndex[i] = (i == 0) ? INDEX_NONE : (i % Sub.NumLimbs);
		}

		OutNames.Reset();
		for (int32 i = 0; i < KeepBodies.Num(); ++i)
		{
			const int32 Old = KeepBodies[i];
			Sub.BodyJointAxisLocal[i]      = Full.BodyJointAxisLocal[Old];
			Sub.BodyJointOffsetInParent[i] = Full.BodyJointOffsetInParent[Old];
			Sub.BodyRestRotInParent[i]     = Full.BodyRestRotInParent[Old];
			Sub.BodyLocalCoMOffset[i]      = Full.BodyLocalCoMOffset[Old];
			Sub.BodyMass[i]                = Full.BodyMass[Old];
			Sub.BodyInertiaDiagLocal[i]    = Full.BodyInertiaDiagLocal[Old];
			Sub.BodyFusedTipOffset[i]      = Full.BodyFusedTipOffset[Old];
			Sub.BodyRadius[i]              = Full.BodyRadius[Old];
			Sub.BodyCapsuleHalfHeight[i]   = Full.BodyCapsuleHalfHeight[Old];
			OutNames.Add(FullNames.IsValidIndex(Old) ? FullNames[Old] : NAME_None);

			// Per-DOF data moves with the body; offsets differ between rigs.
			for (int32 k = 0; k < Sub.BodyDOFCount[i]; ++k)
			{
				const int32 NewDOF = Sub.BodyDOFOffset[i] + k;
				const int32 OldDOF = Full.BodyDOFOffset[Old] + k;
				Sub.DOFExtensionCurve[NewDOF] = Full.DOFExtensionCurve[OldDOF];
				Sub.DOFFlexionCurve[NewDOF]   = Full.DOFFlexionCurve[OldDOF];
				Sub.DOFRangeMinDeg[NewDOF]    = Full.DOFRangeMinDeg[OldDOF];
				Sub.DOFRangeMaxDeg[NewDOF]    = Full.DOFRangeMaxDeg[OldDOF];
				Sub.DOFHasMuscleCurve[NewDOF] = Full.DOFHasMuscleCurve[OldDOF];
			}
		}
		return Sub;
	}

	/** Bodies of the chain rooted at RootBody, inclusive, in ascending order. */
	TArray<int32> CollectChain(const FCreatureTopology& Full, int32 RootBody)
	{
		TArray<int32> Out;
		TSet<int32> InChain{ RootBody };
		for (int32 B = RootBody; B < Full.NumBodies; ++B)
		{
			if (B == RootBody || InChain.Contains(Full.BodyParent[B]))
			{
				InChain.Add(B);
				Out.Add(B);
			}
		}
		return Out;
	}

	/**
	 * Builds a KeepBodies set for ExtractSubTopology: body 0, every ANCESTOR
	 * of each root (so every kept body's parent is also kept -- required by
	 * ExtractSubTopology, see its own comment), and every DESCENDANT of each
	 * root. Needed since 2026-08-16: limb mount bones no longer parent
	 * directly to body 0 (the spine is now independently articulated, F/M/B
	 * mount on Head1/Back2/Back3), so picking "body 0 + a limb's own chain"
	 * alone -- what this used to do inline -- silently omits the real spine
	 * bodies in between, and ExtractSubTopology's OldToNew.FindChecked(OldParent)
	 * asserts when it can't find them.
	 */
	TArray<int32> BuildRigKeepSet(const FCreatureTopology& Full, const TArray<int32>& RootBodies)
	{
		TSet<int32> KeepSet;
		KeepSet.Add(0);
		for (int32 Root : RootBodies)
		{
			for (int32 B = Root; B != INDEX_NONE && !KeepSet.Contains(B); B = (B == 0) ? INDEX_NONE : Full.BodyParent[B])
			{
				KeepSet.Add(B);
			}
			for (int32 Descendant : CollectChain(Full, Root))
			{
				KeepSet.Add(Descendant);
			}
		}
		TArray<int32> Keep = KeepSet.Array();
		Keep.Sort();
		return Keep;
	}

	struct FRunResult
	{
		bool bFinite = true;
		float FailTime = 0.0f;
		FString FailWhat;
		double E0 = 0.0, EEnd = 0.0;
		float TorsoZEnd = 0.0f;
	};

	double TotalEnergy(const FCreatureBatchState& B, const FCreatureTopology& T, const FVector& G, bool& bFinite)
	{
		double E = 0.0; bFinite = true;
		for (int32 Body = 0; Body < T.NumBodies; ++Body)
		{
			const int32 I = B.BodyIndex(Body, 0);
			const FQuat Rot = B.GetBodyRot(Body, 0);
			const FVector Pos = B.GetBodyPos(Body, 0);
			const FVector W(B.AngVelX[I], B.AngVelY[I], B.AngVelZ[I]);
			const FVector V(B.LinVelX[I], B.LinVelY[I], B.LinVelZ[I]);
			const FVector CoMOff = Rot.RotateVector(T.BodyLocalCoMOffset[Body]);
			const FVector CoMVel = V + FVector::CrossProduct(W, CoMOff);
			const FMat3 R = FMat3::FromRotation(Rot);
			const FMat3 IW = R * FMat3::Diagonal(T.BodyInertiaDiagLocal[Body]) * R.Transpose();
			const double M = T.BodyMass[Body];
			const double KE = 0.5 * M * CoMVel.SizeSquared() + 0.5 * FVector::DotProduct(W, IW * W);
			const double PE = -M * FVector::DotProduct(G, Pos + CoMOff);
			if (!FMath::IsFinite(KE) || !FMath::IsFinite(PE)) { bFinite = false; return E; }
			E += KE + PE;
		}
		return E;
	}

	FRunResult RunRig(const FCreatureTopology& T, const TArray<FContactPointDef>& Points,
	                  bool bContact, int32 Hz, float Seconds,
	                  const FVector& StartPos, const FQuat& StartRot,
	                  const FImpulseContactParams& Params, const FVector& Gravity)
	{
		FRunResult R;
		constexpr int32 NumEnvs = 8;
		FCreatureBatchState B;
		B.Init(T, NumEnvs);
		FCreatureABASolver Solver;

		FRandomStream S(1234);
		for (int32 Env = 0; Env < NumEnvs; ++Env)
		{
			CreatureRLEnvironment::ResetEnv(B, Env, StartPos, StartRot, S, 0.0f, 0.0f);
		}
		Solver.Step(B, 0.0f, Gravity);
		for (int32 D = 0; D < T.NumDOF; ++D)
			for (int32 Env = 0; Env < NumEnvs; ++Env)
				B.JointTorque[B.DOFIndex(D, Env)] = 0.0f;

		bool bFin = true;
		R.E0 = TotalEnergy(B, T, Gravity, bFin);

		FImpulseContactCache Cache; // persists across the run — warm starting
		const float Dt = 1.0f / Hz;
		const int32 Steps = FMath::RoundToInt(Seconds * Hz);
		for (int32 Step = 0; Step < Steps; ++Step)
		{
			Solver.Step(B, Dt, Gravity);
			if (bContact) ResolveGroundContactImpulses(B, T, Points, Params, Solver, Dt, Cache, nullptr);

			// Cheap finite check on the quantities that failed historically.
			bool bBad = false;
			for (int32 i = 0; i < B.PosX.Num() && !bBad; ++i)
				if (!FMath::IsFinite(B.PosX[i]) || !FMath::IsFinite(B.LinVelX[i])) bBad = true;
			for (int32 i = 0; i < B.JointVel.Num() && !bBad; ++i)
				if (!FMath::IsFinite(B.JointVel[i]) || !FMath::IsFinite(B.JointPos[i])) bBad = true;

			if (bBad)
			{
				R.bFinite = false;
				R.FailTime = Step * Dt;
				R.FailWhat = TEXT("state non-finite");
				return R;
			}
		}
		R.EEnd = TotalEnergy(B, T, Gravity, bFin);
		if (!bFin) { R.bFinite = false; R.FailTime = Seconds; R.FailWhat = TEXT("energy non-finite"); }
		R.TorsoZEnd = static_cast<float>(B.GetBodyPos(0, 0).Z);
		return R;
	}
}

bool FMutoIsolatedLimb::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Full;
	TArray<FString> Warnings;
	TArray<FName> FullNames;
	if (!TestTrue(TEXT("BuildMutoTopology succeeded"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Full, Warnings, &FullNames))) return false;

	// Locate each limb's root body by bone name.
	auto FindBody = [&FullNames](const TCHAR* Name) -> int32
	{
		for (int32 i = 0; i < FullNames.Num(); ++i) if (FullNames[i].ToString() == Name) return i;
		return INDEX_NONE;
	};
	const int32 FShoulderL = FindBody(TEXT("FShoulder_L"));
	const int32 FShoulderR = FindBody(TEXT("FShoulder_R"));
	const int32 HipsL = FindBody(TEXT("Hips_L"));
	const int32 HipsR = FindBody(TEXT("Hips_R"));

	struct FRig { const TCHAR* Label; TArray<int32> Keep; };
	TArray<FRig> Rigs;
	{
		TArray<int32> A = BuildRigKeepSet(Full, { FShoulderL });
		TArray<int32> Bo = BuildRigKeepSet(Full, { FShoulderL, FShoulderR });
		TArray<int32> C = BuildRigKeepSet(Full, { HipsL });
		TArray<int32> D = BuildRigKeepSet(Full, { HipsL, HipsR });
		TArray<int32> FullSet; for (int32 i = 0; i < Full.NumBodies; ++i) FullSet.Add(i);
		Rigs.Add({ TEXT("A torso+1 F limb "), A });
		Rigs.Add({ TEXT("B torso+2 F limbs"), Bo });
		Rigs.Add({ TEXT("C torso+1 leg    "), C });
		Rigs.Add({ TEXT("D torso+2 legs   "), D });
		Rigs.Add({ TEXT("FULL creature    "), FullSet });
	}

	const FVector Gravity(0.0f, 0.0f, -980.0f);

	AddInfo(TEXT("Sub-topologies extracted from the real rig — authentic offsets, masses, inertia, ranges."));
	AddInfo(TEXT("Passive (zero torque), zero reset noise, 3 simulated seconds."));
	AddInfo(TEXT(""));

	for (const FRig& Rig : Rigs)
	{
		TArray<FName> SubNames;
		const FCreatureTopology Sub = ExtractSubTopology(Full, Rig.Keep, FullNames, SubNames);

		const TArray<FContactPointDef> SubPoints = BuildMutoContactPoints(Sub, MassAsset, SubNames);

		float SubMass = 0.0f;
		for (int32 B = 0; B < Sub.NumBodies; ++B) SubMass += Sub.BodyMass[B];

		const FQuat StandRot = Sub.BodyRestRotInParent[0];
		const float StandH = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(Sub, SubPoints, StandRot);
		const FVector StandPos(0.0f, 0.0f, StandH);

		// Identical for every rig regardless of its mass — the impulse model
		// derives its own effective mass, so unlike the penalty model this needs
		// no per-rig sizing at all. That is exactly what makes a rig-scaling
		// sweep like this one meaningful: the only variable is the body count.
		FImpulseContactParams P;
		P.GroundZ = 0.0f;
		P.ContactHertz = 15.0f;
		P.DampingRatio = 10.0f;
		P.Slop = 0.5f;
		P.FrictionCoefficient = 0.8f;
		P.Iterations = 8;
		P.RelaxIterations = 2;

		AddInfo(FString::Printf(TEXT("---- %s : %d bodies, %d DOF, %.0f kg, %d contact pts, standH=%.1f"),
			Rig.Label, Sub.NumBodies, Sub.NumDOF, SubMass, SubPoints.Num(), StandH));

		struct FVariant { const TCHAR* What; bool bContact; int32 Hz; };
		const FVariant Variants[] = {
			{ TEXT("no contact @ 60Hz "), false,  60 },
			{ TEXT("no contact @240Hz "), false, 240 },
			{ TEXT("CONTACT    @ 60Hz "), true,   60 },
			{ TEXT("CONTACT    @240Hz "), true,  240 },
		};

		for (const FVariant& V : Variants)
		{
			const FRunResult R = RunRig(Sub, SubPoints, V.bContact, V.Hz, 3.0f, StandPos, StandRot, P, Gravity);
			if (!R.bFinite)
			{
				AddInfo(FString::Printf(TEXT("     %s  DIVERGED at t=%.3fs (%s)"),
					V.What, R.FailTime, *R.FailWhat));
			}
			else
			{
				const double Ratio = (FMath::Abs(R.E0) > 1e-9) ? R.EEnd / R.E0 : 0.0;
				AddInfo(FString::Printf(TEXT("     %s  finite 3.0s | E/E0=%+.4g | torsoZ=%.1f"),
					V.What, Ratio, R.TorsoZEnd));
			}
		}
		AddInfo(TEXT(""));
	}

	AddInfo(TEXT("READ: small rigs finite where FULL dies => the bug scales with body count or"));
	AddInfo(TEXT("chain depth. RIG A dying => a minimal reproduction to debug directly."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
