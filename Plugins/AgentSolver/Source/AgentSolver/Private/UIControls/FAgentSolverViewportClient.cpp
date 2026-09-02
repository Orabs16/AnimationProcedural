#include "UIControls/FAgentSolverViewportClient.h"

#if WITH_EDITOR

#include "UIControls/AgentSolverUIUtils.h"
#include "UIControls/AgentSolverViewportSettings.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
// Full definitions needed (not just AgentSolverUIUtils.h's forward
// declarations) so the compiler can see these actually derive from
// AMutoRLTrainingDriver -- see RefreshPoseFromSource's comment.
#include "AgentSolver/MutoRLVisualizer.h"
#include "PhysicsSolver/MutoRagdollVisualizer.h"

#include "Components/PoseableMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/LightComponentBase.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "AdvancedPreviewScene.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "GameFramework/Actor.h"

FAgentSolverViewportClient::FAgentSolverViewportClient(FAdvancedPreviewScene* InPreviewScene, UAgentSolverViewportSettings* InSettings)
	: FEditorViewportClient(nullptr, InPreviewScene)
	, PreviewScene(InPreviewScene)
	, Settings(InSettings)
{
	PreviewMeshComponent = NewObject<UPoseableMeshComponent>(GetTransientPackage(), NAME_None, RF_Transient);
	SetViewLocation(FVector(-300.0f, 0.0f, 100.0f));
	SetViewRotation(FRotator(-10.0f, 0.0f, 0.0f));

	bSetListenerPosition = false;
	SetRealtime(true);

	if (PreviewScene)
	{
		PreviewScene->AddComponent(PreviewMeshComponent, FTransform::Identity);
	}

	// See BlackSkyDomeComponent's comment -- both assets are real, always-
	// shipped engine content (Sphere_inversenormals is loaded the same way,
	// unconditionally, by FAdvancedPreviewScene's own constructor; verified
	// BlackUnlitMaterial.uasset exists on disk under Engine/Content/
	// EngineDebugMaterials). Hidden until RefreshEnvironmentFromLevel decides
	// it's actually needed.
	BlackSkyDomeComponent = NewObject<UStaticMeshComponent>(GetTransientPackage(), NAME_None, RF_Transient);
	if (UStaticMesh* SkySphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/EditorMeshes/AssetViewer/Sphere_inversenormals.Sphere_inversenormals")))
	{
		BlackSkyDomeComponent->SetStaticMesh(SkySphere);
	}
	if (UMaterialInterface* BlackMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/EngineDebugMaterials/BlackUnlitMaterial.BlackUnlitMaterial")))
	{
		BlackSkyDomeComponent->SetMaterial(0, BlackMaterial);
	}
	BlackSkyDomeComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BlackSkyDomeComponent->CastShadow = false;
	BlackSkyDomeComponent->SetVisibility(false);
	if (PreviewScene)
	{
		// Large scale (matches FAdvancedPreviewScene's own sky sphere) so it
		// fully encloses the view regardless of camera position.
		const FTransform SphereTransform(FRotator::ZeroRotator, FVector::ZeroVector, FVector(2000.0f));
		PreviewScene->AddComponent(BlackSkyDomeComponent, SphereTransform);
	}

	// See FMassMuscleViewportClient's identical ticker -- this project's
	// established way of forcing a realtime viewport client to actually
	// redraw every frame while docked in a Slate panel rather than the level
	// viewport.
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
		[this](float DeltaTime) -> bool
		{
			if (Viewport)
			{
				Viewport->Invalidate();
				Viewport->Draw();
			}
			return true;
		}));
}

FAgentSolverViewportClient::~FAgentSolverViewportClient()
{
	FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
}

void FAgentSolverViewportClient::Tick(float DeltaSeconds)
{
	FEditorViewportClient::Tick(DeltaSeconds);
	RefreshEnvironmentFromLevel();
	RefreshPoseFromSource();
}

namespace
{
	const TCHAR* LexViewportSource(EAgentSolverViewportSource Source)
	{
		switch (Source)
		{
		case EAgentSolverViewportSource::Ragdoll: return TEXT("Ragdoll");
		case EAgentSolverViewportSource::Visualizer: default: return TEXT("Visualizer");
		}
	}

	// Every "[AS-TRACE]" heartbeat below fires at most this often (in calls,
	// not seconds -- RefreshPoseFromSource runs once per viewport Tick, so at
	// a typical 30-60fps this is roughly once every 0.5-1s), so the trace is
	// readable over a long session instead of flooding the log every frame.
	constexpr int32 TraceHeartbeatInterval = 30;
}

void FAgentSolverViewportClient::RefreshPoseFromSource()
{
	// See LastPoseFailureReason's comment -- only logs when the reason
	// actually changes, so a genuinely stuck state (e.g. the Python trainer
	// hung and Batch never advances) leaves a trail instead of looking
	// identical to "everything's fine, just not visibly moving this frame".
	// Shares this same dedup mechanism with the lock-busy case below (not
	// just the no-actor/no-mesh/zero-envs cases it originally covered) so
	// EVERY reason posing can stall shows up exactly once when it starts and
	// once when it clears, never silently.
	auto ReportFailure = [this](const FString& Reason)
	{
		if (LastPoseFailureReason != Reason)
		{
			LastPoseFailureReason = Reason;
			UE_LOG(LogTemp, Warning, TEXT("[AS-TRACE] FAgentSolverViewportClient: %s"), *Reason);
		}
	};
	auto ReportSuccess = [this]()
	{
		if (!LastPoseFailureReason.IsEmpty())
		{
			LastPoseFailureReason.Reset();
			UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] FAgentSolverViewportClient: pose source recovered, posing normally again."));
		}
	};

	if (!PreviewMeshComponent)
	{
		return;
	}

	const EAgentSolverViewportSource Source = Settings.IsValid() ? Settings->ViewportSource : EAgentSolverViewportSource::Ragdoll;

	// FindRagdollVisualizer/FindRLVisualizer both return an
	// AMutoRLTrainingDriver* -- they're subclasses of it that exist purely to
	// reuse its Batch/BodyDebugNames/SkeletalMesh/rig plumbing (see their own
	// class comments), so everything below is identical regardless of which
	// source is selected.
	AMutoRLTrainingDriver* SourceActor = AgentSolverUI::FindViewportSourceActor(Source);

	if (!SourceActor)
	{
		ReportFailure(FString::Printf(TEXT("no %s actor found -- pose frozen at its last value."), LexViewportSource(Source)));
		return;
	}
	if (!SourceActor->SkeletalMesh)
	{
		ReportFailure(FString::Printf(TEXT("%s actor '%s' found but has no SkeletalMesh assigned -- pose frozen."), LexViewportSource(Source), *SourceActor->GetName()));
		return;
	}

	if (CurrentSkeletalMesh.Get() != SourceActor->SkeletalMesh)
	{
		CurrentSkeletalMesh = SourceActor->SkeletalMesh;
		PreviewMeshComponent->SetSkinnedAssetAndUpdate(SourceActor->SkeletalMesh);
		//UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] FAgentSolverViewportClient: mesh-show -- SetSkinnedAssetAndUpdate('%s') on %s actor '%s'."),
		//	*SourceActor->SkeletalMesh->GetName(), LexViewportSource(Source), *SourceActor->GetName());
	}

	// No lock needed here: Ragdoll/Visualizer only ever mutate their own
	// Batch from their own Tick() on this same game thread -- the
	// TrainingStepLock this used to take (needed only for the now-removed
	// LiveTrainingDriver source, whose background training thread was the
	// one other writer of Batch) is gone along with it.
	const int32 NumEnvs = SourceActor->Batch.GetNumEnvs();
	if (NumEnvs <= 0)
	{
		ReportFailure(FString::Printf(TEXT("%s actor '%s' has zero envs (Batch.GetNumEnvs()==0) -- training likely hasn't started stepping yet."), LexViewportSource(Source), *SourceActor->GetName()));
		return;
	}

	const int32 EnvIndex = Settings.IsValid() ? FMath::Clamp(Settings->EnvIndexToVisualize, 0, NumEnvs - 1) : 0;
	const FCreatureTopology& Topo = SourceActor->Batch.GetTopology();

	// Same fused-body -> bone mapping as AMutoRLVisualizerActor::UpdateMeshPose
	// (body 0 is the synthetic fused "Torso", whose real bone is Pelvis).
	int32 BodiesPosed = 0;
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		if (!SourceActor->BodyDebugNames.IsValidIndex(Body))
		{
			continue;
		}
		const FName BoneName = (Body == 0) ? TEXT("Pelvis") : SourceActor->BodyDebugNames[Body];
		const FTransform WorldTransform(SourceActor->Batch.GetBodyRot(Body, EnvIndex), SourceActor->Batch.GetBodyPos(Body, EnvIndex));
		PreviewMeshComponent->SetBoneTransformByName(BoneName, WorldTransform, EBoneSpaces::WorldSpace);
		++BodiesPosed;
	}

	// SetBoneTransformByName only sets BoneSpaceTransforms and calls
	// MarkRefreshTransformDirty() (bNeedsRefreshTransform=true) -- the actual
	// work that propagates a new pose to rendering (FillComponentSpaceTransforms,
	// UpdateBounds, MarkRenderTransformDirty, MarkRenderDynamicDataDirty) lives
	// in the SEPARATE RefreshBoneTransforms(), which in a normal level gets
	// called automatically by the component's own registered tick as part of
	// a running world. PreviewMeshComponent lives in an FAdvancedPreviewScene,
	// which has no ticking UWorld behind it -- nothing was ever calling this,
	// so every SetBoneTransformByName call above was writing real data (this
	// is exactly why the [AS-TRACE] heartbeat's torsoZ, which reads Batch
	// directly, showed correct changing values) that never once reached the
	// render proxy -- the mesh was provably being re-posed every tick while
	// looking completely frozen. Must be called explicitly here instead.
	PreviewMeshComponent->RefreshBoneTransforms(nullptr);

	ReportSuccess();

	// Heartbeat: proves the mesh-show step is actually running AND the
	// underlying Batch data is actually changing frame to frame (TorsoZ is
	// the cheapest single number that should visibly move whenever the
	// source's physics/agent tick is doing anything) -- if BodiesPosed==0
	// here despite SourceActor/SkeletalMesh/NumEnvs all being valid, that
	// means BodyDebugNames didn't match Topo.NumBodies (a topology/rig
	// mismatch, not a lock/threading issue) and NOTHING above actually posed
	// a single bone this call, which the pass/fail reporter alone can't show
	// (it only tracks whether SourceActor/mesh/envs were found, not whether
	// posing then did anything).
	if (++TraceHeartbeatCounter >= TraceHeartbeatInterval)
	{
		TraceHeartbeatCounter = 0;
		const float TorsoZ = (float)SourceActor->Batch.GetBodyPos(0, EnvIndex).Z;
		UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] FAgentSolverViewportClient: mesh-show heartbeat -- source=%s actor='%s' env=%d/%d bodiesPosed=%d/%d torsoZ=%.2f."),
			LexViewportSource(Source), *SourceActor->GetName(), EnvIndex, NumEnvs, BodiesPosed, Topo.NumBodies, TorsoZ);
	}
}

void FAgentSolverViewportClient::RefreshEnvironmentFromLevel()
{
	if (!Settings.IsValid() || !PreviewScene)
	{
		return;
	}

	const TSoftObjectPtr<UWorld>& DesiredLevel = Settings->EnvironmentLevel;
	if (DesiredLevel == LastAppliedEnvironmentLevel)
	{
		return;
	}
	LastAppliedEnvironmentLevel = DesiredLevel;

	// See this function's header comment -- without this, a custom level's
	// copied-in geometry renders superimposed on the generic Advanced Preview
	// Scene floor/sky (SetProfileIndex(0) in SAgentSolverViewport::Construct),
	// which is what made a genuinely successful copy look like nothing had
	// happened. bDirect=true so this never touches the user's global Advanced
	// Preview Scene profile settings, only this one client's live state.
	// Restored (shown again) once EnvironmentLevel is cleared.
	const bool bHasCustomEnvironment = !DesiredLevel.IsNull();
	PreviewScene->SetFloorVisibility(!bHasCustomEnvironment, /*bDirect=*/true);
	PreviewScene->SetEnvironmentVisibility(!bHasCustomEnvironment, /*bDirect=*/true);

	// SetFloorVisibility/SetEnvironmentVisibility above only toggle the
	// VISIBLE floor/sky-dome MESH geometry (FAdvancedPreviewScene::
	// SetEnvironmentVisibility literally just calls SkyComponent->SetVisibility)
	// -- they do NOT touch actual illumination. FPreviewScene's own
	// unconditional default DirectionalLight+SkyLight (added because
	// SAgentSolverViewport::Construct passes bDefaultLighting=true) is a
	// SEPARATE light source that was still fully lighting the scene
	// regardless, which is what a copied level's own (much dimmer/different)
	// lights were fighting against -- "way stronger... completely smash my
	// lighting". Both members are public on FPreviewScene (the base class),
	// so toggled directly here rather than through some higher-level API.
	if (PreviewScene->DirectionalLight)
	{
		PreviewScene->DirectionalLight->SetVisibility(!bHasCustomEnvironment);
	}
	if (PreviewScene->SkyLight)
	{
		PreviewScene->SkyLight->SetVisibility(!bHasCustomEnvironment);
	}

	// Clear whatever the previous level (if any) copied in before applying the new one.
	for (USceneComponent* Component : EnvironmentComponents)
	{
		if (Component)
		{
			PreviewScene->RemoveComponent(Component);
		}
	}
	EnvironmentComponents.Reset();

	if (!bHasCustomEnvironment)
	{
		// Restore Advanced Preview Scene's own sky as the reference again --
		// otherwise, if a previous custom level had no sky of its own, the
		// black dome shown for IT would incorrectly stay visible now too.
		if (BlackSkyDomeComponent)
		{
			BlackSkyDomeComponent->SetVisibility(false);
		}
		return;
	}

	UWorld* Level = DesiredLevel.LoadSynchronous();
	if (!Level || !Level->PersistentLevel)
	{
		UE_LOG(LogTemp, Warning, TEXT("FAgentSolverViewportClient: EnvironmentLevel '%s' failed to load as a UWorld -- nothing copied into the preview scene."), *DesiredLevel.ToString());
		if (BlackSkyDomeComponent)
		{
			BlackSkyDomeComponent->SetVisibility(true);
		}
		return;
	}

	// Static meshes + lights (see this function's header comment for why
	// lights are included) -- this never opens or simulates the level, just
	// scans its actor list once for visual dressing to sit the rig in.
	int32 NumMeshesCopied = 0;
	int32 NumLightsCopied = 0;
	int32 NumSkyLightsCopied = 0;
	for (AActor* Actor : Level->PersistentLevel->Actors)
	{
		if (!Actor)
		{
			continue;
		}

		TArray<UStaticMeshComponent*> SourceMeshes;
		Actor->GetComponents<UStaticMeshComponent>(SourceMeshes);
		for (UStaticMeshComponent* SourceComponent : SourceMeshes)
		{
			if (!SourceComponent || !SourceComponent->GetStaticMesh())
			{
				continue;
			}
			if (DuplicateEnvironmentComponent(SourceComponent))
			{
				++NumMeshesCopied;
			}
		}

		TArray<ULightComponentBase*> SourceLights;
		Actor->GetComponents<ULightComponentBase>(SourceLights);
		for (ULightComponentBase* SourceComponent : SourceLights)
		{
			if (!SourceComponent)
			{
				continue;
			}
			if (DuplicateEnvironmentComponent(SourceComponent))
			{
				++NumLightsCopied;
				if (SourceComponent->IsA<USkyLightComponent>())
				{
					++NumSkyLightsCopied;
				}
			}
		}
	}

	// See BlackSkyDomeComponent's comment -- the copied level provided no sky
	// of its own (no USkyLightComponent among what got copied above), and
	// Advanced Preview Scene's own default sky is hidden per this function's
	// earlier SetEnvironmentVisibility(false) call, so without this the scene
	// would have no sky reference at all.
	if (BlackSkyDomeComponent)
	{
		BlackSkyDomeComponent->SetVisibility(NumSkyLightsCopied == 0);
	}

	UE_LOG(LogTemp, Log, TEXT("FAgentSolverViewportClient: copied %d static mesh component(s) and %d light component(s) (%d sky light(s)) from EnvironmentLevel '%s' into the preview scene."), NumMeshesCopied, NumLightsCopied, NumSkyLightsCopied, *DesiredLevel.ToString());
}

USceneComponent* FAgentSolverViewportClient::DuplicateEnvironmentComponent(USceneComponent* Source)
{
	if (!Source)
	{
		return nullptr;
	}

	USceneComponent* Copy = DuplicateObject<USceneComponent>(Source, GetTransientPackage());
	if (!Copy)
	{
		return nullptr;
	}

	// See this function's header comment -- DuplicateObject copies AttachParent
	// too, which still points at a component in the SOURCE level's hierarchy;
	// FPreviewScene::AddComponent only applies the world transform it's given
	// when AttachParent is null, so this must be cleared first or the copy
	// would silently keep the source's RELATIVE transform instead.
	Copy->SetupAttachment(nullptr);
	Copy->ClearFlags(RF_AllFlags);
	Copy->SetFlags(RF_Transient);

	PreviewScene->AddComponent(Copy, Source->GetComponentTransform());
	EnvironmentComponents.Add(Copy);
	return Copy;
}

#endif // WITH_EDITOR
