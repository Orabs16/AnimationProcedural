// TEMPORARY DIAGNOSTIC — DELETE AFTER THE MASS VALUES ARE AUTHORED.
//
// Suspect 1 (see SOLVER_DEBUG_LOG.md, entry 001) established that the rig's
// MASS scale is inconsistent with its LENGTH scale: limb segments are 56-106 cm
// (a creature several metres tall) carrying a uniform placeholder mass of 1.0 kg
// per bone, 43 kg total. Because I = (1/12)*m*L^2 carries mass linearly, that
// under-scaled mass lands directly in the effective mass seen at a contact
// point (m_eff = I/r^2 ~ 0.105 kg), and m_eff is what sets the explicit-
// integration stability bound dt < 2*m_eff/c for the velocity-proportional
// contact terms. That is the whole reason the solver "needs" 960 Hz.
//
// This test AUTHORS NOTHING. It reads the geometry already present in
// MassProfile_Muto (Radius, CapsuleHalfHeight) plus each body's own bone length
// from the built topology, and prints a suggested mass per bone so the values
// can be typed back into the asset by hand — keeping the mass data traceable to
// the asset rather than invented in code.
//
// Suggested mass = capsule volume * density:
//   V = pi*r^2*L_cyl + (4/3)*pi*r^3        (cylinder + two hemispherical caps)
//   density = 1.0e-3 kg/cm^3               (~1000 kg/m^3, i.e. flesh ~ water)
// These are a STARTING POINT sized to the real geometry, not a measurement.
// Adjust individual bones freely — the point is that the totals land in the
// right order of magnitude, not that any single bone is exact.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.MassAuthoringDump; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=massdump.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureGroundContact.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoMassAuthoringDump,
	"AgentSolver.TEMP.MassAuthoringDump",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoMassAuthoringDump::RunTest(const FString& Parameters)
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

	constexpr float DensityKgPerCm3 = 1.0e-3f; // ~1000 kg/m^3

	AddInfo(TEXT("=================================================================================="));
	AddInfo(TEXT("PER-BONE MASS AUTHORING TABLE — type the 'suggest' column into MassProfile_Muto"));
	AddInfo(TEXT("  boneLen  = 2 * |BodyLocalCoMOffset| (the length the inertia model already uses)"));
	AddInfo(TEXT("  radius   = authored FMassMuscleDataMass::Radius"));
	AddInfo(TEXT("  capsuleV = pi*r^2*L + (4/3)*pi*r^3   [cm^3]"));
	AddInfo(FString::Printf(TEXT("  suggest  = capsuleV * %.1e kg/cm^3"), DensityKgPerCm3));
	AddInfo(TEXT("=================================================================================="));
	AddInfo(TEXT("  body  bone            current    boneLen   radius     capsuleV     suggest"));

	float CurrentTotal = 0.0f;
	float SuggestedTotal = 0.0f;

	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const FName BoneName = BodyDebugNames.IsValidIndex(Body) ? BodyDebugNames[Body] : FName(TEXT("<unnamed>"));
		const float CurrentMass = Topo.BodyMass[Body];
		const float BoneLen = 2.0f * static_cast<float>(Topo.BodyLocalCoMOffset[Body].Size());
		const float Radius = Topo.BodyRadius[Body];

		// A zero radius means nothing was authored for this bone (the torso is
		// the expected case — it's a fusion of several bones, not one capsule).
		// Fall back to a slenderness-ratio guess so the row is still useful
		// rather than printing a zero mass that would obviously be wrong.
		const float EffRadius = (Radius > KINDA_SMALL_NUMBER) ? Radius : FMath::Max(BoneLen * 0.15f, 1.0f);
		const float CapsuleVol = PI * EffRadius * EffRadius * BoneLen + (4.0f / 3.0f) * PI * EffRadius * EffRadius * EffRadius;
		const float Suggested = DensityKgPerCm3 * CapsuleVol;

		CurrentTotal += CurrentMass;
		SuggestedTotal += Suggested;

		AddInfo(FString::Printf(
			TEXT("  %4d  %-14s %8.2f %10.2f %8.2f %12.0f %11.2f%s"),
			Body, *BoneName.ToString(), CurrentMass, BoneLen, Radius, CapsuleVol, Suggested,
			(Radius > KINDA_SMALL_NUMBER) ? TEXT("") : TEXT("   <- radius not authored, guessed")));
	}

	AddInfo(TEXT("----------------------------------------------------------------------------------"));
	AddInfo(FString::Printf(TEXT("  TOTAL current = %.1f kg     TOTAL suggested = %.1f kg     (%.1fx)"),
		CurrentTotal, SuggestedTotal, (CurrentTotal > 0.0f) ? SuggestedTotal / CurrentTotal : 0.0f));

	// ---- What this buys, stated in the terms the instability is actually
	// measured in: effective mass at a contact point and the resulting
	// explicit-integration stability bound for each velocity-proportional term.
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- projected effect on contact stability (per contact body) ----"));
	AddInfo(TEXT("  m_eff = I/r^2 at the contact lever arm; bound is dt < 2*m_eff/c"));

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	const float MassScale = (CurrentTotal > 0.0f) ? SuggestedTotal / CurrentTotal : 1.0f;

	// The shipped contact gains these bounds are evaluated against.
	constexpr float ShippedDamperK = 20.0f;
	constexpr float ShippedFrictionK = 150.0f;

	float WorstHzNow = 0.0f;
	float WorstHzAfter = 0.0f;

	for (const FContactPointDef& P : Points)
	{
		const float LeverArm = static_cast<float>(P.LocalOffset.Size());
		if (LeverArm <= 1.0f)
		{
			continue; // contact sits on the joint origin; no lever, no reduction
		}
		const float I = static_cast<float>(Topo.BodyInertiaDiagLocal[P.BodyIndex].GetMax());
		const float MEffNow = I / (LeverArm * LeverArm);
		// Inertia is linear in mass, so m_eff scales with the mass change.
		const float MEffAfter = MEffNow * MassScale;

		// The binding term is whichever gain gives the smaller dt.
		const float WorstGain = FMath::Max(ShippedDamperK, ShippedFrictionK);
		const float HzNow = WorstGain / (2.0f * MEffNow);
		const float HzAfter = WorstGain / (2.0f * MEffAfter);
		WorstHzNow = FMath::Max(WorstHzNow, HzNow);
		WorstHzAfter = FMath::Max(WorstHzAfter, HzAfter);

		AddInfo(FString::Printf(
			TEXT("  %-12s lever=%7.2f  m_eff now=%8.4f -> after=%8.4f  |  min rate now=%8.0f Hz -> after=%7.0f Hz"),
			*P.DebugName.ToString(), LeverArm, MEffNow, MEffAfter, HzNow, HzAfter));
	}

	AddInfo(TEXT("----------------------------------------------------------------------------------"));
	AddInfo(FString::Printf(
		TEXT("  WORST-CASE required rate: %.0f Hz now  ->  %.0f Hz after authoring (target: <= 60 Hz)"),
		WorstHzNow, WorstHzAfter));
	AddInfo(TEXT("  NOTE: this bounds the DAMPING terms only. The normal spring must additionally be"));
	AddInfo(TEXT("  re-derived from the new weight (SpringK ~ Weight/(ends*desiredSag)) — that is"));
	AddInfo(TEXT("  Suspect 3's job, deliberately NOT bundled into this change."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
