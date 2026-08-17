// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 003.
//
// Entry 002 established that the passive, CONTACT-FREE rig diverges at a
// dt-invariant time (~2.67s across a 64x range of substep rates), i.e. the
// continuous system being integrated is itself divergent. The leading
// hypothesis is that the omitted velocity-product ("Coriolis") term in
// SpatialAlgebra.h's TranslateMotion breaks the power balance of the
// articulated chain, letting a passive system gain energy without bound.
//
// This measures that directly.
//
// Case ZG is the decisive one: gravity OFF, contact OFF, no actuation, with the
// joints given an initial velocity. That system is CLOSED — total mechanical
// energy must be exactly conserved. There is no PE reference-height ambiguity
// and no huge free-fall PE term to swamp float precision, so any growth at all
// is unambiguously energy injected by the solver.
//
// Case G is the real failing scenario (gravity on, standing pose) traced for
// comparison, so the two can be correlated against the ~2.67s failure.
//
// Energy is accumulated in DOUBLE precision: with 6162 kg at ~400 cm and
// g=980, PE lands near 2.4e9, where float32 spacing is ~128 — large enough to
// hide the very drift being measured.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.EnergyTrace; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=energy.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h"
#include "CreatureGroundContact.h"
#include "MutoRLTrainingDriver.h"
#include "CreatureRLEnvironment.h"

#if WITH_EDITOR
#include "MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoEnergyTrace,
	"AgentSolver.TEMP.EnergyTrace",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace
{
	/**
	 * Total mechanical energy of one env, in double precision.
	 * Same CoM/inertia convention the solver itself uses (see
	 * CreatureBatchSolver.h's "own rigid-body inertia" pass) and the same one
	 * CreaturePendulumEnergyTest uses, so a real dynamics bug shows up here
	 * instead of being masked by a mismatched convention. Unlike that test,
	 * body 0 IS included — here it is a genuine free body, not a fixed anchor.
	 */
	double ComputeTotalEnergy(const FCreatureBatchState& Batch, const FCreatureTopology& Topo,
	                          int32 Env, const FVector& Gravity, bool& bOutFinite)
	{
		double Total = 0.0;
		bOutFinite = true;

		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const FQuat Rot = Batch.GetBodyRot(Body, Env);
			const FVector Pos = Batch.GetBodyPos(Body, Env);
			const FVector AngVel(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
			const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);

			// LinVel is the velocity of the body's REFERENCE point (its joint
			// origin), not its CoM — shift it before forming KE.
			const FVector CoMOffsetWorld = Rot.RotateVector(Topo.BodyLocalCoMOffset[Body]);
			const FVector CoMPos = Pos + CoMOffsetWorld;
			const FVector CoMVel = LinVel + FVector::CrossProduct(AngVel, CoMOffsetWorld);

			const FMat3 RotM = FMat3::FromRotation(Rot);
			const FMat3 IComWorld = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[Body]) * RotM.Transpose();

			const double Mass = Topo.BodyMass[Body];
			const double KE = 0.5 * Mass * CoMVel.SizeSquared()
				+ 0.5 * FVector::DotProduct(AngVel, IComWorld * AngVel);
			const double PE = -Mass * FVector::DotProduct(Gravity, CoMPos);

			if (!FMath::IsFinite(KE) || !FMath::IsFinite(PE)) { bOutFinite = false; return Total; }
			Total += KE + PE;
		}
		return Total;
	}
}

bool FMutoEnergyTrace::RunTest(const FString& Parameters)
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

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	const FQuat StandingTorsoRot = Topo.BodyRestRotInParent[0];
	const float TargetTorsoHeight = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(Topo, Points, StandingTorsoRot);
	const FVector StandingTorsoPos(0.0f, 0.0f, TargetTorsoHeight);

	constexpr int32 NumEnvs = 8;

	struct FCase
	{
		const TCHAR* Label;
		bool bGravity;
		float InitialJointVel; // rad/s applied to every DOF at t=0
		int32 Hz;
		float Seconds;
	};

	const FCase Cases[] = {
		// Kick magnitude sweep. Setting EVERY DOF spinning at once is not a
		// physical pose — velocities compound down a 5-link, 2m-per-segment
		// chain, so a large kick drives joints hard into their limits, where
		// the inelastic clamp dominates. Sweeping the magnitude separates "the
		// dynamics inject energy" from "this initial condition is violent".
		{ TEXT("ZG-tiny zero gravity, joint kick 0.01 rad/s @60Hz"),  false, 0.01f,  60, 5.0f },
		{ TEXT("ZG-small zero gravity, joint kick 0.05 rad/s @60Hz"), false, 0.05f,  60, 5.0f },
		// DECISIVE: closed system. Energy MUST be exactly conserved.
		{ TEXT("ZG-60   zero gravity, joint kick 0.5 rad/s @60Hz"),   false, 0.5f,   60, 5.0f },
		{ TEXT("ZG-960  zero gravity, joint kick 0.5 rad/s @960Hz"),  false, 0.5f,  960, 5.0f },
		// Control: closed system AT REST. Nothing should ever move.
		{ TEXT("ZG-REST zero gravity, zero velocity @60Hz"),          false, 0.0f,   60, 5.0f },
		// The real failing scenario, traced for correlation.
		{ TEXT("G-60    gravity on, passive, no contact @60Hz"),      true,  0.0f,   60, 5.0f },
		{ TEXT("G-960   gravity on, passive, no contact @960Hz"),     true,  0.0f,  960, 5.0f },
	};

	for (const FCase& Case : Cases)
	{
		const FVector Gravity = Case.bGravity ? FVector(0.0f, 0.0f, -980.0f) : FVector::ZeroVector;
		const float Dt = 1.0f / Case.Hz;
		const int32 NumSteps = FMath::RoundToInt(Case.Seconds * Case.Hz);

		FCreatureBatchState Batch;
		Batch.Init(Topo, NumEnvs);
		FCreatureABASolver Solver;

		FRandomStream S(1234);
		for (int32 Env = 0; Env < NumEnvs; ++Env)
		{
			CreatureRLEnvironment::ResetEnv(Batch, Env, StandingTorsoPos, StandingTorsoRot, S, 0.0f, 0.0f);
		}
		Solver.Step(Batch, 0.0f, Gravity); // FK refresh — ResetEnv writes the root only

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const int32 Idx = Batch.DOFIndex(DOF, Env);
				Batch.JointTorque[Idx] = 0.0f;                 // passive throughout
				Batch.JointVel[Idx] = Case.InitialJointVel;    // the kick
			}
		}

		bool bFinite = true;
		const double E0 = ComputeTotalEnergy(Batch, Topo, 0, Gravity, bFinite);

		AddInfo(FString::Printf(TEXT("---- %s ----"), Case.Label));
		AddInfo(FString::Printf(TEXT("     E0 = %+.6e"), E0));

		// Sample roughly 10 times per simulated second regardless of rate.
		const int32 SampleEvery = FMath::Max(1, Case.Hz / 10);
		double PeakRatio = 1.0;
		int32 FirstDoubleStep = INDEX_NONE;
		int32 NonFiniteStep = INDEX_NONE;

		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			Solver.Step(Batch, Dt, Gravity);

			if ((Step % SampleEvery) != 0 && Step != NumSteps - 1) continue;

			const double E = ComputeTotalEnergy(Batch, Topo, 0, Gravity, bFinite);
			if (!bFinite || !FMath::IsFinite(E))
			{
				NonFiniteStep = Step;
				AddInfo(FString::Printf(TEXT("     t=%6.3fs  ENERGY NON-FINITE"), Step * Dt));
				break;
			}

			// Ratio against |E0| — for the zero-gravity cases E0 is pure KE and
			// strictly positive, so this is a clean multiplier.
			const double Ratio = (FMath::Abs(E0) > 1e-9) ? (E / E0) : 0.0;
			PeakRatio = FMath::Max(PeakRatio, FMath::Abs(Ratio));
			if (FirstDoubleStep == INDEX_NONE && FMath::Abs(Ratio) > 2.0) FirstDoubleStep = Step;

			// Keep the log readable: print every half second only.
			if ((Step % (SampleEvery * 5)) == 0 || Step == NumSteps - 1)
			{
				AddInfo(FString::Printf(TEXT("     t=%6.3fs  E=%+.6e   E/E0=%+.4g"), Step * Dt, E, Ratio));
			}
		}

		if (NonFiniteStep != INDEX_NONE)
		{
			AddInfo(FString::Printf(TEXT("     VERDICT: diverged, energy non-finite by t=%.3fs"), NonFiniteStep * Dt));
		}
		else if (FirstDoubleStep != INDEX_NONE)
		{
			AddInfo(FString::Printf(TEXT("     VERDICT: ENERGY GROWTH — exceeded 2x E0 at t=%.3fs, peak %.4g x"),
				FirstDoubleStep * Dt, PeakRatio));
		}
		else
		{
			AddInfo(FString::Printf(TEXT("     VERDICT: bounded, peak |E/E0| = %.6g"), PeakRatio));
		}
		AddInfo(TEXT(""));
	}

	AddInfo(TEXT("READING THIS: the ZG cases are closed systems — energy MUST be conserved."));
	AddInfo(TEXT("Growth there is energy the SOLVER created, and is conclusive on its own."));
	AddInfo(TEXT("If ZG-60 and ZG-960 grow at a similar RATE PER SECOND (not per step), the"));
	AddInfo(TEXT("injection is a modeling error, not an integration error."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
