#pragma once

// PASSIVE RAGDOLL PLAYBACK. No policy, no inference, no muscles, no reward,
// no episode termination -- gravity and ground contact acting on the
// articulated rig, and nothing else.
//
// Why this exists as its own actor rather than a flag on AMutoRLVisualizerActor:
// that one shows what the POLICY is doing, which right now means every muscle
// firing in a different direction at once, at high speed. Motion that fast and
// that uncorrelated is unreadable -- you cannot tell a solver defect from a bad
// policy, because both look like thrashing. Removing actuation removes the
// entire confound: whatever this actor shows is the SOLVER, because there is no
// other input. If the ragdoll behaves and the trained one doesn't, the problem
// is the policy or the reward; if the ragdoll misbehaves, the problem is
// underneath both, and every RL question can wait.
//
// This is also the natural place to watch the things that are currently open:
// - a joint bending about an axis it has no business bending about (that class
//   of bug was real -- SOLVER_DEBUG_LOG.md entry 022 -- and was caught by eye,
//   in the viewport, not by any test),
// - sustained ground contact diverging after ~1.5s (entry 022),
// so it draws joint axes and contact points, and it FREEZES on the first
// non-finite value instead of letting the rig vanish to infinity, which is
// otherwise the only thing you get to see.
//
// Playback defaults to quarter speed. The point is legibility, not realism --
// PlaybackSpeed scales only how much SIMULATED time one real frame advances, so
// slow motion changes nothing about the physics (identical substep dt, identical
// results). It is a camera shutter, not a solver setting.
//
// Usage: drop one in the level, assign the three rig assets (SkeletalMesh /
// MassAsset / MuscleAsset), press Play. Nothing else needs assigning -- in
// particular there is NO SourceTrainingDriver, because there is no policy. All
// the inherited "Muto RL|Training" properties are unused here; the ones that do
// matter are contact (ContactHertz/DampingRatio/Iterations/Slop/Friction),
// PhysicsSubstepDt, and Gravity.

#include "CoreMinimal.h"
#include "MutoRLTrainingDriver.h"
#include "MutoRagdollVisualizer.generated.h"

class UPoseableMeshComponent;

/**
 * Subclasses AMutoRLTrainingDriver for the same reason AMutoRLVisualizerActor
 * does -- code reuse of the rig-asset UPROPERTYs, the Batch/Solver/ContactPoints
 * simulation state, StepPhysicsSubstepped() and ComputeDefaultStandingHeight().
 * It overrides StartTraining()/Tick() entirely and creates no Learning Agents
 * objects whatsoever (no Manager, Interactor, Policy, Critic or Trainer), so
 * "StartTraining" here is a misnomer inherited from the base class: it starts
 * playback.
 */
UCLASS()
class AGENTSOLVER_API AMutoRagdollVisualizerActor : public AMutoRLTrainingDriver
{
	GENERATED_BODY()

public:
	AMutoRagdollVisualizerActor();

	virtual void StartTraining() override;
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(VisibleAnywhere, Category = "Muto Ragdoll")
	TObjectPtr<UPoseableMeshComponent> MeshComponent;

	// ----- Time control. The whole point of this actor: you cannot read
	// motion you cannot slow down. None of this touches the physics -- see
	// PlaybackSpeed. -----

	/**
	 * Simulated seconds per real second. 1.0 is real time, 0.25 (default) is
	 * quarter speed, 0.05 is very slow.
	 *
	 * This scales only how much simulated time each frame advances. Substep dt
	 * is PhysicsSubstepDt regardless, so the trajectory at 0.05 is the SAME
	 * trajectory as at 1.0, just delivered over 20x the wall-clock. Changing it
	 * cannot mask or introduce a solver bug.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Time", meta = (ClampMin = "0.01", ClampMax = "4.0"))
	float PlaybackSpeed = 0.25f;

	/** Freeze the simulation. The pose still updates and debug draw still runs, so you can orbit the camera around a stopped frame. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Time")
	bool bPaused = false;

	/** How many PhysicsSubstepDt substeps one StepOnce() advances. 1 = the finest possible increment, which is what you want when watching a divergence build. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Time", meta = (ClampMin = "1", ClampMax = "240"))
	int32 SubstepsPerManualStep = 1;

	// ----- Initial pose -----

	/** Torso height at reset. <= 0 auto-derives the standing height from the rig's own rest-pose leg length (ComputeDefaultStandingHeight), i.e. feet just touching. Raise it to drop the rig from a height instead. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Reset")
	float ResetTorsoHeight = -1.0f;

	/** Extra rotation applied to the torso at reset, on top of its bind-pose orientation. Tip it over to watch contact catch it on one side. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Reset")
	FRotator ResetTorsoTilt = FRotator::ZeroRotator;

	/** Uniform random perturbation applied to every joint at reset, in degrees. 0 = exact bind pose. Non-zero gives the rig something to fall FROM, which is usually more informative than a perfectly symmetric collapse. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Reset", meta = (ClampMin = "0.0"))
	float ResetJointJitterDeg = 0.0f;

	/** Re-seeded into ResetStream on every ResetRagdoll(), so the same seed always gives the same fall -- a divergence you can watch twice is worth far more than one you can't reproduce. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Reset")
	int32 ResetSeed = 12345;

	/** Automatically reset once the rig has been at rest for this long (seconds of simulated time). <= 0 never auto-resets, which is the default: an unexplained settle is something to look at, not something to clear away. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Reset")
	float AutoResetAfterRestSeconds = -1.0f;

	// ----- Divergence -----

	/**
	 * On the first non-finite value anywhere in the state, roll back to the
	 * last good frame, pause, and log which body or DOF went first.
	 *
	 * Without this the rig's coordinates become NaN and the mesh simply stops
	 * being drawn -- the single least informative outcome available, and
	 * exactly what makes "it diverges at ~1.5s" hard to look at. With it you
	 * keep the frame BEFORE the blow-up on screen, which is the frame that
	 * actually contains the evidence.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Divergence")
	bool bFreezeOnDivergence = true;

	/** Also treat "finite but absurd" as divergence -- a body further than this from the origin, or faster than MaxPlausibleSpeed. Catches the blow-up a frame or two earlier, while the pose is still readable. 0 disables. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Divergence")
	float MaxPlausibleDistance = 100000.0f;

	/** See MaxPlausibleDistance. In cm/s. 0 disables. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Divergence")
	float MaxPlausibleSpeed = 500000.0f;

	// ----- Debug draw -----

	/** Contact points: green when touching (radius scaled by normal force), grey when clear. These are spheres, not points -- drawn at their authored Radius, so you can see the actual collision geometry rather than guessing. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawContactPoints = true;

	/** Every revolute's rotation axis, at its joint origin. This is the picture that would have made SOLVER_DEBUG_LOG.md entry 022 obvious on sight: a knee's axis should be perpendicular to the leg, and a wrong one visibly points along it. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawJointAxes = false;

	/** Each body's own frame (red/green/blue = X/Y/Z), which is the frame joint axes and inertia are authored in. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawBodyFrames = false;

	/** Per-body linear velocity vectors. The first body to run away is usually the first to be wrong. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawVelocities = false;

	/** The skeleton as lines parent->child, drawn even where the mesh is hard to read from. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawBones = false;

	/** A reference grid on the ground plane at Config.GroundZ, so "is it sinking?" is answerable by eye. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bDrawGroundGrid = true;

	/** Length in cm of the drawn axis/velocity indicators. Velocities are additionally normalised, so this is their full length at MaxPlausibleSpeed-ish scale, not a per-cm/s scale factor. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw", meta = (ClampMin = "1.0"))
	float DebugDrawScale = 25.0f;

	/** Live readout in the top-left: sim time, contacts, deepest penetration, fastest body/joint, divergence state. */
	UPROPERTY(EditAnywhere, Category = "Muto Ragdoll|Draw")
	bool bShowOnScreenStats = true;

	// ----- Buttons (usable in the Details panel while PIE runs) -----

	/** Return to the initial pose and clear the divergence state. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto Ragdoll")
	void ResetRagdoll();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto Ragdoll")
	void TogglePause();

	/** Advance exactly SubstepsPerManualStep substeps while paused. Does not unpause. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto Ragdoll")
	void StepOnce();

	/** Dump the current per-body and per-DOF state to the log -- the numeric counterpart to whatever you are looking at. Most useful on a frozen divergence frame. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto Ragdoll")
	void DumpState();

private:
	/** Poses MeshComponent from the current single-env Batch. Same mapping as AMutoRLVisualizerActor::UpdateMeshPose -- body 0 is the fused torso and drives "Pelvis". */
	void UpdateMeshPose();

	/** Runs NumSubsteps of passive physics, with the rollback/divergence check around them. Returns false if it diverged (and, if bFreezeOnDivergence, left the state rolled back). */
	bool AdvancePhysics(int32 NumSubsteps);

	/** Scans the whole state for non-finite or implausible values. Returns a human-readable reason, or an empty string if the state is good. */
	FString FindBadState() const;

	void DrawDebug() const;
	void DrawStats() const;

	/** Last known-good state, kept so a divergence can be rolled back one substep and looked at. Copied before each substep -- a full FCreatureBatchState copy, which at 1 env is a few KB. */
	FCreatureBatchState LastGoodBatch;
	bool bHasLastGood = false;

	/** Set once divergence is detected; suppresses further stepping and is what the on-screen readout reports. Cleared by ResetRagdoll(). */
	bool bDiverged = false;
	FString DivergenceReason;
	float DivergedAtSimTime = 0.0f;

	float SimTime = 0.0f;
	float RestTimer = 0.0f;

	/** Populated by AdvancePhysics for the readout, so DrawStats does no work of its own. */
	int32 NumTouchingContacts = 0;
	float DeepestPenetration = 0.0f;
	float FastestBodySpeed = 0.0f;
	int32 FastestBodyIndex = INDEX_NONE;
	float FastestJointSpeedDeg = 0.0f;
	int32 FastestJointDOF = INDEX_NONE;
};
