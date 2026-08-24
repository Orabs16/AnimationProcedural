#pragma once

// Editor-only preview-scene viewport client for SAgentSolverControlPanel's
// embedded viewport -- poses a UPoseableMeshComponent from the Ragdoll/
// Visualizer actor found in the current world (AgentSolverUI::
// FindViewportSourceActor), one env row at a time (see
// UAgentSolverViewportSettings::EnvIndexToVisualize), the same
// BodyDebugNames/Batch-transform technique AMutoRLVisualizerActor::
// UpdateMeshPose already uses in-level. Mirrors MassMuscleProfile's
// FMassMuscleViewportClient (preview scene + realtime ticker forcing
// continuous redraw).
//
// Whole file is WITH_EDITOR-gated, unlike this module's UCLASS headers --
// FEditorViewportClient itself lives in UnrealEd (an Editor-only module, see
// AgentSolver.Build.cs), and there's no UHT reflection markup here forcing
// this to stay parseable unconditionally the way MutoRLTrainingDriver.h does.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "EditorViewportClient.h"
#include "Containers/Ticker.h"
#include "UObject/WeakObjectPtr.h"
#include "UObject/SoftObjectPtr.h"

class UPoseableMeshComponent;
class UStaticMeshComponent;
class USceneComponent;
class UAgentSolverViewportSettings;
class USkeletalMesh;
class UWorld;
class AMutoRLTrainingDriver;
class FAdvancedPreviewScene;

class FAgentSolverViewportClient : public FEditorViewportClient, public TSharedFromThis<FAgentSolverViewportClient>
{
public:
	FAgentSolverViewportClient(FAdvancedPreviewScene* InPreviewScene, UAgentSolverViewportSettings* InSettings);
	virtual ~FAgentSolverViewportClient() override;

	virtual void Tick(float DeltaSeconds) override;

private:
	/**
	 * Re-poses PreviewMeshComponent from whichever actor EAgentSolverViewportSource
	 * currently selects (AgentSolverUI::FindViewportSourceActor -- Ragdoll/
	 * Visualizer are both subclasses of AMutoRLTrainingDriver sharing the same
	 * Batch/BodyDebugNames/SkeletalMesh layout, so everything here is uniform
	 * across both). No lock needed: both only ever mutate their own Batch
	 * from their own Tick() on this same game thread. No-ops, leaving the
	 * mesh at its last pose, if nothing is found / no SkeletalMesh assigned /
	 * zero envs right now. Also explicitly calls PreviewMeshComponent->
	 * RefreshBoneTransforms() after posing -- see the .cpp for why that's
	 * required here specifically (a preview-scene component has no ticking
	 * UWorld to call it automatically the way a normal level component would).
	 */
	void RefreshPoseFromSource();

	/**
	 * Copies the StaticMeshComponents AND light components (ULightComponentBase --
	 * covers Directional/Point/Spot/Rect lights and Sky Light in one pass,
	 * since USkyLightComponent doesn't derive from the punctual-light-specific
	 * ULightComponent) of every actor in Settings->EnvironmentLevel's
	 * PersistentLevel into the preview scene, as plain transient duplicates --
	 * see that property's comment. Lights included because copying only the
	 * static geometry left everything lit solely by the generic Advanced
	 * Preview Scene default light, which looks flat/grey next to whatever the
	 * source level actually authored. Re-run only when EnvironmentLevel
	 * actually changes (LastAppliedEnvironmentLevel), not every tick:
	 * loading/scanning a whole level's actor list is not something to repeat
	 * 30-60 times a second.
	 *
	 * Also toggles off, whenever a custom EnvironmentLevel is set: PreviewScene's
	 * default floor/environment mesh (SetProfileIndex(0) in
	 * SAgentSolverViewport::Construct -- otherwise the copied level geometry
	 * and the generic Advanced Preview Scene floor/sky render superimposed on
	 * each other), AND its default DirectionalLight/SkyLight (a SEPARATE light
	 * source from the floor/environment MESH visibility toggle above -- see
	 * this function's .cpp comment; left on, it fully lit the scene regardless
	 * of whatever the copied level's own lights were doing).
	 */
	void RefreshEnvironmentFromLevel();

	/**
	 * DuplicateObject's Source (preserving every property -- mesh/materials,
	 * or light color/intensity/radius/angle/etc., whichever concrete class it
	 * is), clears the copy's stale AttachParent (SetupAttachment(nullptr) --
	 * DuplicateObject copies that reference too, and it points at a component
	 * in the SOURCE level's own hierarchy, which FPreviewScene::AddComponent
	 * would otherwise defer to instead of the world transform passed in),
	 * marks it transient, and adds it to PreviewScene at Source's exact world
	 * transform. Returns the copy (also already appended to
	 * EnvironmentComponents) or nullptr if DuplicateObject failed.
	 */
	USceneComponent* DuplicateEnvironmentComponent(USceneComponent* Source);

	TObjectPtr<UPoseableMeshComponent> PreviewMeshComponent = nullptr;
	TWeakObjectPtr<USkeletalMesh> CurrentSkeletalMesh;
	FAdvancedPreviewScene* PreviewScene = nullptr;
	TWeakObjectPtr<UAgentSolverViewportSettings> Settings;

	TArray<TObjectPtr<USceneComponent>> EnvironmentComponents;
	TSoftObjectPtr<UWorld> LastAppliedEnvironmentLevel;

	/**
	 * Own plain black inverted-sphere sky dome (same sphere mesh Advanced
	 * Preview Scene itself uses, "/Engine/EditorMeshes/AssetViewer/
	 * Sphere_inversenormals", material "/Engine/EngineDebugMaterials/
	 * BlackUnlitMaterial" -- both real, always-shipped engine assets).
	 * Created once, hidden by default; RefreshEnvironmentFromLevel shows it
	 * only when a custom EnvironmentLevel is active AND that level copied in
	 * no USkyLightComponent of its own -- otherwise, with the Advanced
	 * Preview Scene's own default sky hidden (see that function) and no sky
	 * light in the copied level either, the scene would have no sky
	 * reference at all instead of a clean black one.
	 */
	TObjectPtr<UStaticMeshComponent> BlackSkyDomeComponent = nullptr;

	/**
	 * Logged reason RefreshPoseFromSource last bailed early (or empty string
	 * if it last succeeded) -- only re-logged when this actually CHANGES, so
	 * "still frame" while training is genuinely stuck/misconfigured shows up
	 * once in the log instead of being silently invisible (this is a polling
	 * function running every tick; without this, "why isn't it moving" had
	 * no diagnostic trail at all).
	 */
	FString LastPoseFailureReason;

	/**
	 * Counts calls to RefreshPoseFromSource -- gates the "[AS-TRACE]" heartbeat
	 * log to roughly once a second (see TraceHeartbeatInterval in the .cpp)
	 * instead of every tick, since this is a WITH_EDITOR polling function that
	 * runs 30-60x/sec. Shares the "[AS-TRACE]" prefix with the physics/agent
	 * tick heartbeats in MutoRagdollVisualizer.cpp/MutoRLVisualizer.cpp/
	 * MutoRLTrainingDriver.cpp so the whole mesh-show -> physics-tick ->
	 * agent-tick chain for a given viewport source can be grepped as one
	 * ordered trace when tracking down "at which point does this stop moving".
	 */
	int32 TraceHeartbeatCounter = 0;

	FTSTicker::FDelegateHandle TickHandle;
};

#endif // WITH_EDITOR
