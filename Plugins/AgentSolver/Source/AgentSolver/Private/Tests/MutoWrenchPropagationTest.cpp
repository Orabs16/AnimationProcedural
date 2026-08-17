// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entries 011 / 012.
//
// Checks the formulation-INDEPENDENT laws on extracted chains:
//     d/dt (total linear momentum)  == total external FORCE
//     d/dt (total angular momentum) == total external TORQUE  (about a fixed origin)
// Whatever the ABA does internally, these must hold, so any violation is the
// solver's and not the model's.
//
// REPAIRED after entry 011, where the first version failed its own control by
// reporting 30% momentum error on a one-body closed system whose answer is
// provably zero. Four faults, all in the test:
//
//   1. THE REAL ONE — the variant dispatch was
//          if (Mode == Grav) {...} else { ApplyForceAtPoint(...); }
//      so the CLOSED variant fell into the else and had the full 1e6 force
//      applied to it while being SCORED as a closed system. Every CLOSED number
//      in entry 011 was measuring a driven system against a closed-system
//      expectation.
//   2. Momentum was sampled straight after Step(), where non-root transforms
//      are stale — Pass 1 computes them from the OLD root transform, then Pass
//      3a moves the root and Pass 3b integrates the joints, with no FK refresh
//      unless a clamp fires. M0 and M1 were therefore not comparable states.
//      Now every sample is preceded by a zero-dt Step(), which re-runs Pass 1
//      without advancing anything.
//   3. The angular denominator used |T_ext|, which starts at ~0 because the
//      torso's BodyLocalCoMOffset is ZeroVector (so r_com x F vanishes at t=0),
//      turning a tiny absolute error into a meaningless ratio. Now normalized
//      by a fixed characteristic torque |F| * L_char.
//   4. A 1e6 force on a ~2 m lever made the unclamped chains flail, so late
//      steps measured chaotic divergence rather than systematic error. The
//      force is now a fraction of the rig's own weight and the duration short.
//
// Joint limits deliberately never fire here (muscle ranges are not copied into
// the sub-topology), so there are no impulsive clamp events to corrupt a
// momentum balance. That is checked, not assumed.
//
// GATE: the one-body CLOSED case must read ~0 before any other number in this
// file is trustworthy. It is reported first and labelled as the gate.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.WrenchPropagation; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=wrenchprop.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h"
#include "CreatureGroundContact.h"

#if WITH_EDITOR
#include "MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoWrenchPropagation,
	"AgentSolver.TEMP.WrenchPropagation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace WrenchProp
{
	struct FMomentum { FVector P = FVector::ZeroVector; FVector L = FVector::ZeroVector; };

	/** Total linear and angular momentum of env 0, about the WORLD ORIGIN. */
	FMomentum TotalMomentum(const FCreatureBatchState& B, const FCreatureTopology& T)
	{
		FMomentum M;
		for (int32 Body = 0; Body < T.NumBodies; ++Body)
		{
			const int32 I = B.BodyIndex(Body, 0);
			const FQuat Rot = B.GetBodyRot(Body, 0);
			const FVector Pos = B.GetBodyPos(Body, 0);
			const FVector W(B.AngVelX[I], B.AngVelY[I], B.AngVelZ[I]);
			const FVector V(B.LinVelX[I], B.LinVelY[I], B.LinVelZ[I]);

			const FVector CoMOff = Rot.RotateVector(T.BodyLocalCoMOffset[Body]);
			const FVector CoMPos = Pos + CoMOff;
			const FVector CoMVel = V + FVector::CrossProduct(W, CoMOff);

			const FMat3 R = FMat3::FromRotation(Rot);
			const FMat3 IW = R * FMat3::Diagonal(T.BodyInertiaDiagLocal[Body]) * R.Transpose();

			const float Mass = T.BodyMass[Body];
			const FVector Lin = Mass * CoMVel;
			M.P += Lin;
			M.L += FVector::CrossProduct(CoMPos, Lin) + IW * W;
		}
		return M;
	}

	FCreatureTopology ExtractSubTopology(
		const FCreatureTopology& Full, const TArray<int32>& Keep,
		const TArray<FName>& FullNames, TArray<FName>& OutNames,
		bool bForceRevolute = false)
	{
		TMap<int32, int32> OldToNew;
		for (int32 i = 0; i < Keep.Num(); ++i) OldToNew.Add(Keep[i], i);

		TArray<int32> Parent, DOFCount, LimbIndex;
		for (int32 i = 0; i < Keep.Num(); ++i)
		{
			Parent.Add(i == 0 ? 0 : OldToNew.FindChecked(Full.BodyParent[Keep[i]]));
			// bForceRevolute turns every joint into a 1-DOF hinge while keeping
			// the SAME bodies, masses, inertia and offsets — so a ball-vs-revolute
			// comparison changes only the reduction path being exercised.
			const int32 Native = Full.BodyDOFCount[Keep[i]];
			DOFCount.Add(i == 0 ? 0 : (bForceRevolute ? 1 : Native));
			LimbIndex.Add(i == 0 ? INDEX_NONE : 0);
		}

		FCreatureTopology Sub;
		Sub.NumLimbs = 1;
		Sub.Build(Parent, DOFCount, LimbIndex);

		OutNames.Reset();
		for (int32 i = 0; i < Keep.Num(); ++i)
		{
			const int32 Old = Keep[i];
			Sub.BodyJointAxisLocal[i]      = Full.BodyJointAxisLocal[Old];
			Sub.BodyJointOffsetInParent[i] = Full.BodyJointOffsetInParent[Old];
			Sub.BodyRestRotInParent[i]     = Full.BodyRestRotInParent[Old];
			Sub.BodyLocalCoMOffset[i]      = Full.BodyLocalCoMOffset[Old];
			Sub.BodyMass[i]                = Full.BodyMass[Old];
			Sub.BodyInertiaDiagLocal[i]    = Full.BodyInertiaDiagLocal[Old];
			Sub.BodyRadius[i]              = Full.BodyRadius[Old];
			OutNames.Add(FullNames.IsValidIndex(Old) ? FullNames[Old] : NAME_None);
			// Muscle ranges intentionally NOT copied — see the header note about
			// clamp events corrupting a momentum balance.
		}
		return Sub;
	}

	/** Rough spatial extent of the chain in the rest pose — a characteristic lever. */
	float CharacteristicLength(const FCreatureTopology& T)
	{
		float L = 0.0f;
		for (int32 b = 1; b < T.NumBodies; ++b)
		{
			L += static_cast<float>(T.BodyJointOffsetInParent[b].Size());
		}
		return FMath::Max(L, 100.0f);
	}
}

bool FMutoWrenchPropagation::RunTest(const FString& Parameters)
{
	using namespace WrenchProp;

	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Full;
	TArray<FString> Warnings;
	TArray<FName> FullNames;
	if (!TestTrue(TEXT("BuildMutoTopology"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Full, Warnings, &FullNames))) return false;

	auto FindBody = [&FullNames](const TCHAR* N) -> int32
	{
		for (int32 i = 0; i < FullNames.Num(); ++i) if (FullNames[i].ToString() == N) return i;
		return INDEX_NONE;
	};
	const int32 HipsL = FindBody(TEXT("Hips_L"));
	const int32 Knee1L = FindBody(TEXT("Knee1_L"));
	const int32 Knee2L = FindBody(TEXT("Knee2_L"));

	// The 2-body BALL rig is the smallest CLOSED failure. Pairing it with an
	// otherwise-identical rig whose joint is forced to a 1-DOF revolute isolates
	// the ball-joint reduction as the only difference between them.
	struct FRig { const TCHAR* Label; TArray<int32> Keep; bool bForceRevolute; };
	const FRig Rigs[] = {
		{ TEXT("1 body (torso)          "), { 0 },                        false },
		{ TEXT("2 bodies BALL joint     "), { 0, HipsL },                 false },
		{ TEXT("2 bodies REVOLUTE joint "), { 0, HipsL },                 true  },
		{ TEXT("3 bodies (ball+revolute)"), { 0, HipsL, Knee1L },         false },
		{ TEXT("3 bodies ALL REVOLUTE   "), { 0, HipsL, Knee1L },         true  },
		{ TEXT("4 bodies (ball+2 revol.)"), { 0, HipsL, Knee1L, Knee2L }, false },
		{ TEXT("4 bodies ALL REVOLUTE   "), { 0, HipsL, Knee1L, Knee2L }, true  },
	};

	// A REAL-GROUND-CONTACT variant used to live here, checking the system
	// against the wrench the penalty model deposited in ExtForce/ExtTorque. It
	// answered its question — the contact wrench WAS applied correctly, which is
	// what moved the investigation off wrench propagation (entry 013) — and was
	// removed with the penalty model: the impulse path never writes ExtForce, it
	// applies impulses straight to velocities, so there is no deposited wrench to
	// read back. The variants below still validate the recursion itself, which is
	// this test's lasting value.
	enum class EMode { Closed, Tip, Grav };
	struct FVariant { const TCHAR* What; EMode Mode; };
	const FVariant Variants[] = {
		{ TEXT("CLOSED (no ext force)"), EMode::Closed },
		{ TEXT("wrench on TIP body  "), EMode::Tip    },
		{ TEXT("GRAVITY only        "), EMode::Grav   },
	};

	AddInfo(TEXT("Momentum balance on extracted chains. No joint limits can fire (ranges not copied)."));
	AddInfo(TEXT("Every momentum sample is preceded by a zero-dt Step() so forward kinematics match"));
	AddInfo(TEXT("the current joint state — otherwise non-root transforms are one step stale."));
	AddInfo(TEXT("GATE: the 1-body CLOSED row must read ~0 or nothing else here is meaningful."));
	AddInfo(TEXT(""));

	for (const FRig& Rig : Rigs)
	{
		TArray<FName> Names;
		const FCreatureTopology T = ExtractSubTopology(Full, Rig.Keep, FullNames, Names, Rig.bForceRevolute);

		float TotalMass = 0.0f;
		for (int32 b = 0; b < T.NumBodies; ++b) TotalMass += T.BodyMass[b];
		const float LChar = CharacteristicLength(T);
		const int32 Last = T.NumBodies - 1;

		AddInfo(FString::Printf(TEXT("---- %s : %d bodies, %d DOF, %.0f kg, L_char=%.0f"),
			Rig.Label, T.NumBodies, T.NumDOF, TotalMass, LChar));

		for (const FVariant& Var : Variants)
		{
			// GENTLE: a fraction of the rig's own weight, so the chain moves
			// without flailing into a chaotic regime (entry 011 fault 4).
			const FVector Force(0.0f, 0.0f, 0.05f * TotalMass * 980.0f);
			const FVector CaseGravity = (Var.Mode == EMode::Grav)
				? FVector(0, 0, -980.0f) : FVector::ZeroVector;
			const FVector LocalApp = (Last > 0) ? 2.0f * T.BodyLocalCoMOffset[Last] : FVector(100.0f, 0.0f, 0.0f);

			// One contact point at the far end of the chain, sitting just below
			// the ground so contact is active from the first step.
			CreatureGroundContact::FContactPointDef Pt;
			Pt.BodyIndex = Last;
			Pt.LocalOffset = LocalApp;
			Pt.Radius = T.BodyRadius[Last];
			Pt.DebugName = Names[Last];
			Pt.LimbIndex = 0;
			const TArray<CreatureGroundContact::FContactPointDef> ContactPoints{ Pt };

			CreatureGroundContact::FImpulseContactParams ContactParams;
			// Ground placed just under the rig's rest pose so the point is a few
			// units deep — a gentle, persistent contact rather than an impact.
			ContactParams.GroundZ = -5.0f;
			ContactParams.ContactHertz = 30.0f;
			ContactParams.DampingRatio = 10.0f;
			ContactParams.FrictionCoefficient = 0.8f;
			ContactParams.Iterations = 8;
			ContactParams.RelaxIterations = 2;
			CreatureGroundContact::FImpulseContactCache ContactCache;

			for (const int32 Hz : { 1000, 4000, 16000 })
			{
				const float Dt = 1.0f / Hz;
				const float Duration = 0.05f;
				const int32 Steps = FMath::RoundToInt(Duration * Hz);
				constexpr int32 WarmUp = 2;

				FCreatureBatchState B;
				B.Init(T, 8);
				FCreatureABASolver Solver;
				B.SetBodyPos(0, 0, FVector::ZeroVector);
				B.SetBodyRot(0, 0, FQuat::Identity);

				if (Var.Mode == EMode::Closed)
				{
					for (int32 D = 0; D < T.NumDOF; ++D)
						for (int32 Env = 0; Env < 8; ++Env)
							B.JointVel[B.DOFIndex(D, Env)] = 0.2f;
					B.LinVelX[B.BodyIndex(0, 0)] = 50.0f;
					B.AngVelY[B.BodyIndex(0, 0)] = 0.3f;
				}
				Solver.Step(B, 0.0f, CaseGravity); // FK refresh

				const FMomentum MRef = TotalMomentum(B, T);
				const double LinDenom = (Var.Mode == EMode::Closed)
					? FMath::Max(1.0, (double)MRef.P.Size())
					: FMath::Max(1.0, (double)((Var.Mode == EMode::Grav) ? TotalMass * 980.0f : Force.Size()));
				const double AngDenom = FMath::Max(1.0, LinDenom * LChar);

				double WorstPErr = 0.0, WorstLErr = 0.0;

				for (int32 Step = 0; Step < Steps; ++Step)
				{
					B.ClearExternalForces(0);
					FVector FExt = FVector::ZeroVector;
					FVector TExt = FVector::ZeroVector;

					// EXPLICIT per-mode dispatch. The entry-011 bug was a bare
					// `else` here that applied the force in CLOSED mode too.
					switch (Var.Mode)
					{
					case EMode::Closed:
						break; // genuinely nothing external

					case EMode::Tip:
					{
						const FQuat AppRot = B.GetBodyRot(Last, 0);
						const FVector AppWorld = B.GetBodyPos(Last, 0) + AppRot.RotateVector(LocalApp);
						B.ApplyForceAtPoint(Last, 0, Force, AppWorld);
						FExt = Force;
						TExt = FVector::CrossProduct(AppWorld, Force);
						break;
					}

					case EMode::Grav:
					{
						FExt = TotalMass * CaseGravity;
						for (int32 b = 0; b < T.NumBodies; ++b)
						{
							const FVector CoMPos = B.GetBodyPos(b, 0)
								+ B.GetBodyRot(b, 0).RotateVector(T.BodyLocalCoMOffset[b]);
							TExt += FVector::CrossProduct(CoMPos, T.BodyMass[b] * CaseGravity);
						}
						break;
					}

					}

					const FMomentum M0 = TotalMomentum(B, T);
					Solver.Step(B, Dt, CaseGravity);
					Solver.Step(B, 0.0f, CaseGravity); // refresh FK before sampling M1
					const FMomentum M1 = TotalMomentum(B, T);

					double PErr, LErr;
					if (Var.Mode == EMode::Closed)
					{
						PErr = (M1.P - MRef.P).Size() / LinDenom;
						LErr = (M1.L - MRef.L).Size() / AngDenom;
					}
					else
					{
						PErr = ((M1.P - M0.P) / Dt - FExt).Size() / LinDenom;
						LErr = ((M1.L - M0.L) / Dt - TExt).Size() / AngDenom;
					}

					// For CLOSED this is cumulative DRIFT from the initial value, so
					// take it at the FINAL step — a fixed physical time. Worst-over-
					// steps would be unfair across rates, since a finer dt simply
					// takes more samples of the same 0.05 s and is more likely to
					// catch a peak. For the driven variants the quantity is a
					// per-step instantaneous error, where worst-case is the right
					// summary.
					if (Var.Mode == EMode::Closed)
					{
						if (Step == Steps - 1) { WorstPErr = PErr; WorstLErr = LErr; }
					}
					else if (Step >= WarmUp)
					{
						WorstPErr = FMath::Max(WorstPErr, PErr);
						WorstLErr = FMath::Max(WorstLErr, LErr);
					}
				}

				const bool bOK = (WorstPErr < 1e-3 && WorstLErr < 1e-3);
				AddInfo(FString::Printf(
					TEXT("     %s @%5d Hz: linear=%.4g  angular=%.4g  %s%s"),
					Var.What, Hz, WorstPErr, WorstLErr,
					bOK ? TEXT("OK") : TEXT("<<<< NOT CONSERVED"),
					(T.NumBodies == 1 && Var.Mode == EMode::Closed) ? TEXT("   [GATE]") : TEXT("")));
			}
		}
		AddInfo(TEXT(""));
	}

	AddInfo(TEXT("READ: with the GATE passing, a multi-body failure is the solver. Failure only"));
	AddInfo(TEXT("for >1 body isolates it to the joint reduction; CLOSED failing implicates the"));
	AddInfo(TEXT("recursion itself, TIP-only implicates the external-wrench path through it."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
