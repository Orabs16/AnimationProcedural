// PERMANENT INVARIANT GUARD. Started as the diagnostic for a user-reported bug
// (SOLVER_DEBUG_LOG.md entry 022) and kept afterwards, because the invariant it
// checks -- a 1-DOF revolute rotates about its authored axis and nothing else --
// is otherwise invisible. A rig with every revolute axis wrong still animates,
// still conserves momentum, still passes every other test in this directory; it
// just bends about the wrong axes, and only a human watching the viewport
// notices.
//
// User-reported symptom: Knee1_R has only a Yaw muscle authored, so it is built
// as a 1-DOF revolute and must be incapable of rolling about its own long axis.
// In the editor it visibly rolls.
//
// The visualizer is already ruled out by inspection: MutoRLVisualizer.cpp:233-234
// writes Batch.GetBodyRot(Body,0) straight into SetBoneTransformByName in world
// space, with no remapping. So whatever roll is on screen is roll the solver's
// forward kinematics produced.
//
// That leaves the revolute axis itself. CreatureBatchState.h:42 declares
// BodyJointAxisLocal as "revolute axis, in PARENT's rest frame", and
// BuildMutoTopology hardcodes (0,-1,0) for every chain joint, and
// CreatureBatchSolver.h:127 consumes it as ParentRot.RotateVector(AxisLocal).
// But the authored muscle "Knee1_muscle_Yaw_R" has BoneName=Knee1_R -- it names
// the axis in KNEE1's own frame, not its parent's. Those two frames differ by
// exactly BodyRestRotInParent, the bind-pose rotation, which for a knee is not
// identity.
//
// Two measurements, both pure kinematics -- no dynamics, no contact, no
// timestep, so nothing here can be blamed on integration:
//
//  A. AXIS AUDIT. For every 1-DOF body, express the axis the solver ACTUALLY
//     rotates about in that bone's OWN frame. A correct Yaw joint must come
//     back as (0,-1,0): zero X (roll) and zero Z (pitch) component. The X
//     component IS the roll leakage, in radians of roll per radian of
//     commanded yaw.
//
//  B. FUNCTIONAL FK CHECK. Bind pose, command Knee1_R's single DOF to +30deg,
//     nothing else touched. Take the body's world rotation delta, and resolve
//     its axis in the bone's own frame. Reports the roll actually delivered.
//     This is the same quantity as A, arrived at through the real FK path
//     rather than from the topology -- if they disagree, the bug is in FK
//     rather than in the axis.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.Muto.JointAxisAudit; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=jointaxis.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

namespace
{
	// The rig's joint-orient convention, per BuildMutoTopology's comment at
	// MutoTopology.h:311-327: +X toward the child bone, +Z into the interior of
	// the range of motion, +Y flipping sign between sides. So in a bone's own
	// frame, X is roll (the long axis), Y is yaw, Z is pitch.
	FString DescribeAxisInBoneFrame(const FVector& A)
	{
		return FString::Printf(TEXT("roll(X)=%+.4f  yaw(Y)=%+.4f  pitch(Z)=%+.4f"), A.X, A.Y, A.Z);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoJointAxisAudit,
	"AgentSolver.Muto.JointAxisAudit",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoJointAxisAudit::RunTest(const FString& Parameters)
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
	for (const FString& Warning : Warnings)
	{
		AddInfo(FString::Printf(TEXT("BuildMutoTopology warning: %s"), *Warning));
	}

	// ---------------------------------------------------------------------
	// A. AXIS AUDIT
	// ---------------------------------------------------------------------
	AddInfo(TEXT("=== A. Revolute axis, resolved in each bone's OWN frame ==="));
	AddInfo(TEXT("    A correct Yaw revolute reads roll(X)=0, yaw(Y)=-1, pitch(Z)=0."));

	float WorstRollLeak = 0.0f;
	FName WorstRollLeakBone = NAME_None;
	int32 NumRevolutes = 0;
	int32 NumLeaking = 0;
	constexpr float LeakToleranceRad = 0.01f; // ~0.6% of commanded rotation

	for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
	{
		if (Topo.BodyDOFCount[Body] != 1)
		{
			continue;
		}
		++NumRevolutes;

		const FQuat Rest = Topo.BodyRestRotInParent[Body];
		const FVector AxisInParentFrame = Topo.BodyJointAxisLocal[Body];

		// The solver rotates about ParentRot * AxisLocal. Relative to the body's
		// own frame -- which at rest is ParentRot * Rest -- that same axis is
		// Rest^-1 * AxisLocal.
		const FVector AxisInBoneFrame = Rest.UnrotateVector(AxisInParentFrame);

		const float RollLeak = FMath::Abs(static_cast<float>(AxisInBoneFrame.X));
		if (RollLeak > WorstRollLeak)
		{
			WorstRollLeak = RollLeak;
			WorstRollLeakBone = BodyDebugNames[Body];
		}
		if (RollLeak > LeakToleranceRad)
		{
			++NumLeaking;
		}

		FVector RestAxis; float RestAngleRad;
		Rest.ToAxisAndAngle(RestAxis, RestAngleRad);

		AddInfo(FString::Printf(
			TEXT("  %-14s parent=%-14s restRot=%.2fdeg  axis-in-bone-frame: %s   => %.1f%% of commanded yaw becomes ROLL"),
			*BodyDebugNames[Body].ToString(),
			*(Topo.BodyParent[Body] == 0 ? FName(TEXT("Pelvis")) : BodyDebugNames[Topo.BodyParent[Body]]).ToString(),
			FMath::RadiansToDegrees(RestAngleRad),
			*DescribeAxisInBoneFrame(AxisInBoneFrame),
			RollLeak * 100.0f));
	}

	AddInfo(FString::Printf(TEXT("  -> %d of %d revolutes leak roll beyond %.3f; worst is %s at %.4f (%.1f%%)"),
		NumLeaking, NumRevolutes, LeakToleranceRad,
		*WorstRollLeakBone.ToString(), WorstRollLeak, WorstRollLeak * 100.0f));

	// ---------------------------------------------------------------------
	// B. FUNCTIONAL FK CHECK on Knee1_R specifically
	// ---------------------------------------------------------------------
	AddInfo(TEXT("=== B. Functional FK: command Knee1_R +30deg from bind pose ==="));

	const int32 KneeBody = BodyDebugNames.IndexOfByKey(FName(TEXT("Knee1_R")));
	if (!TestTrue(TEXT("Knee1_R exists as an ABA body"), KneeBody != INDEX_NONE)) return false;
	if (!TestEqual(TEXT("Knee1_R is a 1-DOF revolute"), Topo.BodyDOFCount[KneeBody], 1)) return false;

	FCreatureBatchState Batch;
	Batch.Init(Topo, 1);
	FCreatureABASolver Solver;

	// Bind pose: every joint at zero, root at identity.
	Solver.RecomputeKinematics(Batch);
	const FQuat KneeRotBefore = Batch.GetBodyRot(KneeBody, 0);
	const FQuat ParentRotBefore = Batch.GetBodyRot(Topo.BodyParent[KneeBody], 0);

	constexpr float CommandedDeg = 30.0f;
	const int32 KneeDOF = Topo.BodyDOFOffset[KneeBody];
	Batch.JointPos[Batch.DOFIndex(KneeDOF, 0)] = FMath::DegreesToRadians(CommandedDeg);
	Solver.RecomputeKinematics(Batch);
	const FQuat KneeRotAfter = Batch.GetBodyRot(KneeBody, 0);
	const FQuat ParentRotAfter = Batch.GetBodyRot(Topo.BodyParent[KneeBody], 0);

	// Nothing upstream moved, so the whole delta belongs to this joint. Assert
	// it rather than assume it -- otherwise a parent that drifted would be
	// misread as roll leaking out of the knee.
	const float ParentMoved = FMath::RadiansToDegrees(ParentRotBefore.AngularDistance(ParentRotAfter));
	TestTrue(TEXT("Parent body did not move (delta is attributable to Knee1_R alone)"), ParentMoved < 1e-4f);

	const FQuat WorldDelta = (KneeRotAfter * KneeRotBefore.Inverse()).GetNormalized();
	FVector DeltaAxisWorld; float DeltaAngleRad;
	WorldDelta.ToAxisAndAngle(DeltaAxisWorld, DeltaAngleRad);

	// Resolve the delivered rotation axis in the bone's own (pre-move) frame.
	const FVector DeltaAxisInBoneFrame = KneeRotBefore.UnrotateVector(DeltaAxisWorld);
	const float DeliveredDeg = FMath::RadiansToDegrees(DeltaAngleRad);
	const float RollDeg = DeliveredDeg * static_cast<float>(DeltaAxisInBoneFrame.X);
	const float YawDeg = DeliveredDeg * static_cast<float>(DeltaAxisInBoneFrame.Y);
	const float PitchDeg = DeliveredDeg * static_cast<float>(DeltaAxisInBoneFrame.Z);

	AddInfo(FString::Printf(TEXT("  commanded          %+.3f deg of YAW"), CommandedDeg));
	AddInfo(FString::Printf(TEXT("  delivered rotation %+.3f deg about %s"), DeliveredDeg, *DescribeAxisInBoneFrame(DeltaAxisInBoneFrame)));
	AddInfo(FString::Printf(TEXT("  resolved in bone frame:  roll %+.3f deg   yaw %+.3f deg   pitch %+.3f deg"), RollDeg, YawDeg, PitchDeg));

	// The headline assertion. A 1-DOF Yaw revolute may not roll, at all.
	TestTrue(
		FString::Printf(TEXT("Knee1_R produces no roll about its own long axis (got %.3f deg of roll for %.1f deg of commanded yaw)"), RollDeg, CommandedDeg),
		FMath::Abs(RollDeg) < 0.05f);

	// Reported separately: pitch leakage is the same defect on a different axis
	// and would be just as wrong, but it is not what was observed on screen, so
	// it must not be able to mask or substitute for the roll result above.
	TestTrue(
		FString::Printf(TEXT("Knee1_R produces no pitch either (got %.3f deg)"), PitchDeg),
		FMath::Abs(PitchDeg) < 0.05f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
