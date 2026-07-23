#pragma once

#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"
#include "EditorViewportClient.h"
#include "FMassMuscleEditorModel.h"
#include "FMassMuscleEditorSettings.h"
#include "HMuscleHandleProxy.h"

class FMassMuscleViewportClient : public FEditorViewportClient, public TSharedFromThis<FMassMuscleViewportClient>
{
public:
	void SetSkeletalMesh(USkeletalMesh* Mesh);
	void SetSelectedBone(FName BoneName);
	FMassMuscleViewportClient(FPreviewScene* InPreviewScene, TSharedPtr<FMassMuscleEditorModel> InModel = nullptr, TSharedPtr<FMassMuscleEditorSettings> InSettings = nullptr);

	void SetModel(TSharedPtr<FMassMuscleEditorModel> InModel)
	{
		Model = InModel;
	}
	void SetSettings(TSharedPtr<FMassMuscleEditorSettings> InSettings)
	{
		Settings = InSettings;
	}
	virtual ~FMassMuscleViewportClient() override;

protected:
	virtual void Draw(const FSceneView* View, FPrimitiveDrawInterface* PDI) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void ProcessClick(FSceneView& View, HHitProxy* HitProxy, FKey Key, EInputEvent Event, uint32 HitX, uint32 HitY) override;
	virtual bool InputKey(const FInputKeyEventArgs& EventArgs) override;
	virtual void CapturedMouseMove(FViewport* InViewport, int32 InMouseX, int32 InMouseY) override;

private:
	TObjectPtr<USkeletalMesh> PreviewSkeletalMesh = nullptr;

	TObjectPtr<USkeletalMeshComponent> PreviewMeshComponent = nullptr;

	FPreviewScene* PreviewScene = nullptr;

	TSharedPtr<FMassMuscleEditorModel> Model;
	TSharedPtr<FMassMuscleEditorSettings> Settings;

	void DrawSkeleton(FReferenceSkeleton RefSkeleton, FPrimitiveDrawInterface* PDI);
	void DrawSelectedArc(FPrimitiveDrawInterface* PDI, FMassMuscleDataMuscle* Muscle);
	void DrawSingleArc(FPrimitiveDrawInterface* PDI, FMassMuscleDataMuscle* Muscle);
	void DrawAllMuscleArcs(FPrimitiveDrawInterface* PDI, UMassMuscleProfileAssetMuscle* profile);
	void DrawSelectedBone(FPrimitiveDrawInterface *PDI, const FReferenceSkeleton &RefSkeleton);
	float GetGizmoScale(const FVector& Origin, const FSceneView* View) const;
	void DrawRotationGizmo(FPrimitiveDrawInterface* PDI, const FVector& Origin, float Scale);

	float ComputeAngleFromMouse(FViewport* InViewport, int32 X, int32 Y) const;
	bool bIsDraggingHandle = false;
	EMuscleHandle DraggedHandle = EMuscleHandle::Min;
    FVector ArcCenter = FVector::ZeroVector;
    FVector ArcXAxis  = FVector::ForwardVector;
    FVector ArcYAxis  = FVector::RightVector;
    float   ArcRadius = 30.f;



	FTSTicker::FDelegateHandle TickHandle;
};