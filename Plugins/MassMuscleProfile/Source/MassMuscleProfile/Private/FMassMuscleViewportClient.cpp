#include "FMassMuscleViewportClient.h"
#include "CoreGlobals.h"
#include "DrawDebugHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "FMassMuscleData.h"
#include "FMassMuscleEditorSettings.h"
#include "FMassMuscleTreeItem.h"
#include "Logging/LogMacros.h"
#include "Logging/LogVerbosity.h"
#include "Math/MathFwd.h"
#include "Misc/CoreMiscDefines.h"
#include "PreviewScene.h"
#include "PrimitiveDrawingUtils.h"
#include "ReferenceSkeleton.h"
#include "Templates/SharedPointer.h"
#include "UObject/SoftObjectPath.h"
#include "Engine/World.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "HMassMuscleHitProxy.h"
#include "FMassMuscleEditorModel.h"
#include "FMassMuscleStyle.h"
#include "HMuscleHandleProxy.h"
#include "UMassMuscleProfileAsset.h"
#include "Containers/Ticker.h"

FMassMuscleViewportClient::FMassMuscleViewportClient(FPreviewScene* InPreviewScene, TSharedPtr<FMassMuscleEditorModel> InModel, TSharedPtr<FMassMuscleEditorSettings> InSettings)
	: FEditorViewportClient(nullptr, InPreviewScene)
{
	PreviewMeshComponent = NewObject<USkeletalMeshComponent>(GetTransientPackage(), NAME_None, RF_Transient);
	this->SetViewLocation(FVector(-150, 0, 80));
	this->SetViewRotation(FRotator(-10, 0, 0));

	bSetListenerPosition = false;
	this->SetRealtime(true);
	SetRealtime(true);
	PreviewScene = InPreviewScene;
	if (InPreviewScene)
	{
		InPreviewScene->AddComponent(PreviewMeshComponent, FTransform::Identity);
	}

	if (InModel)
	{
		SetModel(InModel);

		Model->OnSkeletalMeshChanged.AddRaw(this, &FMassMuscleViewportClient::SetSkeletalMesh);
		if (InModel->GetSkeletalMesh())
		{
			SetSkeletalMesh(InModel->GetSkeletalMesh());
		}
	}
	if (InSettings)
	{
		Settings = InSettings;
	}
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
		[this](float DeltaTime) -> bool
		{
			if (Viewport)
			{
				Viewport->Invalidate();
				Viewport->Draw(); // force an actual redraw
			}
			return true; // return true to keep ticking
		}
	));
}

void FMassMuscleViewportClient::Tick(float DeltaSeconds)
{
	FEditorViewportClient::Tick(DeltaSeconds);
}

void FMassMuscleViewportClient::SetSkeletalMesh(USkeletalMesh* Mesh)
{
	PreviewSkeletalMesh = Mesh;

	if (PreviewMeshComponent)
	{
		PreviewMeshComponent->SetSkeletalMesh(Mesh);
	}
}
void FMassMuscleViewportClient::Draw(const FSceneView* View, FPrimitiveDrawInterface* PDI)
{
	FEditorViewportClient::Draw(View, PDI);

	if (!PreviewMeshComponent || !PreviewMeshComponent->GetSkeletalMeshAsset())
		return;

	const FReferenceSkeleton& RefSkeleton = PreviewMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();


	DrawSkeleton(RefSkeleton, PDI);
	DrawAllMuscleArcs(PDI, Model->GetMuscleProfile());


	if (Model->GetSelectedType() == EItemType::Muscle)
	{
		UMassMuscleProfileAssetMuscle* profile = Model->GetMuscleProfile();
		if (profile)
		{
			int32 index = profile->FindMuscleByName(Model->GetSelected());
			if (index != INDEX_NONE)
			{
				DrawSelectedArc(PDI, &profile->Muscles[index]);
			}
		}
	}
	else
	{
		DrawSelectedBone(PDI, RefSkeleton);
		DrawBoneCapsule(PDI, RefSkeleton);
		if (Settings->XRayCapsules)
		{
			DrawAllBoneCapsules(PDI, RefSkeleton, Model->GetMassProfile());
		}
		/*
		UMassMuscleProfileAssetMuscle* profile = Model->GetMuscleProfile();
		int32 BoneIndex = Model->GetSelectedBoneIndex();
		if (profile && BoneIndex != INDEX_NONE)
		{
			FTransform boneTransform = RefSkeleton.GetBoneAbsoluteTransform(BoneIndex);
			DrawRotationGizmo(PDI, boneTransform.GetLocation(), 20);
		}*/
	}
}

void FMassMuscleViewportClient::DrawSelectedBone(FPrimitiveDrawInterface* PDI, const FReferenceSkeleton& RefSkeleton)
{
	if (Model->GetSelectedBoneIndex() == INDEX_NONE)
	{
		return;
	}

	const FTransform BoneTransform = PreviewMeshComponent->GetBoneTransform(Model->GetSelectedBoneIndex());

	const FVector Pos = BoneTransform.GetLocation();

	DrawWireSphere(PDI, Pos, FMassMuscleStyle::BoneColor, 5.0f, 12, SDPG_Foreground);
	//DrawAllMuscleArcs(PDI);
	const int32 ParentIndex = RefSkeleton.GetParentIndex(Model->GetSelectedBoneIndex());
	if (ParentIndex != INDEX_NONE)
	{
		const FTransform ParentTransform = PreviewMeshComponent->GetBoneTransform(ParentIndex);

		PDI->DrawLine(BoneTransform.GetLocation(), ParentTransform.GetLocation(), FColor::Orange, -1, 2.0f);
	}
	TArray<int32> Children;
	RefSkeleton.GetDirectChildBones(Model->GetSelectedBoneIndex(), Children);
	for (int32 ChildIndex : Children)
	{
		const FTransform ChildTransform = PreviewMeshComponent->GetBoneTransform(ChildIndex);

		PDI->DrawLine(BoneTransform.GetLocation(), ChildTransform.GetLocation(), FColor::Green, -1, 2.0f);
	}
}

void FMassMuscleViewportClient::DrawBoneCapsule(FPrimitiveDrawInterface* PDI, const FReferenceSkeleton& RefSkeleton)
{
	if (!Model.IsValid() || Model->GetSelectedBoneIndex() == INDEX_NONE)
		return;

	UMassMuscleProfileAssetMass* MassProfile = Model->GetMassProfile();
	if (!MassProfile)
		return;

	const int32 BoneIndex = Model->GetSelectedBoneIndex();
	const FName BoneName = RefSkeleton.GetBoneName(BoneIndex);
	const int32 MassIndex = MassProfile->FindBoneByName(BoneName);
	if (MassIndex == INDEX_NONE)
		return;

	DrawSingleBoneCapsule(PDI, RefSkeleton, BoneIndex, MassProfile->Mass[MassIndex]);
}

void FMassMuscleViewportClient::DrawAllBoneCapsules(FPrimitiveDrawInterface* PDI, const FReferenceSkeleton& RefSkeleton, UMassMuscleProfileAssetMass* MassProfile)
{
	if (!MassProfile)
		return;

	for (const FMassMuscleDataMass& BoneMass : MassProfile->Mass)
	{
		const int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneMass.BoneName);
		if (BoneIndex == INDEX_NONE)
			continue;

		DrawSingleBoneCapsule(PDI, RefSkeleton, BoneIndex, BoneMass);
	}
}

void FMassMuscleViewportClient::DrawSingleBoneCapsule(FPrimitiveDrawInterface* PDI, const FReferenceSkeleton& RefSkeleton, int32 BoneIndex, const FMassMuscleDataMass& BoneMass)
{
	if (BoneMass.Radius <= 0.0f)
		return;

	const FVector BonePos = PreviewMeshComponent->GetBoneTransform(BoneIndex).GetLocation();

	// Tip = first direct child's joint position (matches AgentSolver's
	// BodyFusedTipOffset convention — see CreatureGroundContact.h). Leaf
	// bones (e.g. this rig's own unarticulated Tip bones) have no child to
	// point toward, so extrapolate along the incoming parent->bone direction
	// instead, just so a capsule still previews.
	TArray<int32> Children;
	RefSkeleton.GetDirectChildBones(BoneIndex, Children);
	FVector TipPos;
	if (Children.Num() > 0)
	{
		TipPos = PreviewMeshComponent->GetBoneTransform(Children[0]).GetLocation();
	}
	else
	{
		const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
		const FVector InDirection = ParentIndex != INDEX_NONE
			? (BonePos - PreviewMeshComponent->GetBoneTransform(ParentIndex).GetLocation()).GetSafeNormal()
			: FVector::UpVector;
		TipPos = BonePos + InDirection * FMath::Max(BoneMass.Radius * 2.0f, 1.0f);
	}

	FVector Axis = (TipPos - BonePos).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::UpVector;
	}

	// End cap is fixed at the tip (matches the old sphere-at-tip behavior
	// when CapsuleHalfHeight == 0); HalfHeight pulls the start cap back
	// along the bone toward — and potentially past — its own origin.
	const FVector EndCenter = TipPos;
	const FVector StartCenter = TipPos - Axis * (BoneMass.CapsuleHalfHeight * 2.0f);
	const FVector Center = (StartCenter + EndCenter) * 0.5f;
	const float SegmentHalfLen = FVector::Dist(StartCenter, EndCenter) * 0.5f;

	FVector XAxis = FVector::CrossProduct(Axis, FVector::UpVector).GetSafeNormal();
	if (XAxis.IsNearlyZero())
	{
		XAxis = FVector::CrossProduct(Axis, FVector::ForwardVector).GetSafeNormal();
	}
	const FVector YAxis = FVector::CrossProduct(Axis, XAxis).GetSafeNormal();

	DrawWireCapsule(PDI, Center, XAxis, YAxis, Axis, FMassMuscleStyle::MassCapsuleColor, BoneMass.Radius, SegmentHalfLen + BoneMass.Radius, 16, -1);
}

void FMassMuscleViewportClient::DrawSkeleton(FReferenceSkeleton RefSkeleton, FPrimitiveDrawInterface* PDI)
{
	const int32 NumBones = RefSkeleton.GetNum();

	for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
	{
		const FName BoneName = RefSkeleton.GetBoneName(BoneIndex);
		const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);

		if (ParentIndex != INDEX_NONE)
		{
			const FTransform BoneTransform = PreviewMeshComponent->GetBoneTransform(BoneIndex);
			const FTransform ParentTransform = PreviewMeshComponent->GetBoneTransform(ParentIndex);
			const FVector Direction = (BoneTransform.GetLocation() - ParentTransform.GetLocation()).GetSafeNormal();
			const FVector RightVector = FVector::CrossProduct(Direction, FVector::UpVector).GetSafeNormal();
			const FVector UpVector = FVector::CrossProduct(Direction, RightVector).GetSafeNormal();

			PDI->SetHitProxy(new HBoneLineProxy(BoneName, 0));

			PDI->DrawLine(BoneTransform.GetLocation(), ParentTransform.GetLocation(), FMassMuscleStyle::BoneColor, Settings->XRaySkeleton ? -1 : 0, 1.0f);

			PDI->SetHitProxy(nullptr);
		}
	}
}

void FMassMuscleViewportClient::DrawAllMuscleArcs(FPrimitiveDrawInterface* PDI, UMassMuscleProfileAssetMuscle* profile)
{
	if (!Settings->XRayMuscles)
		return;
	if (!profile)
		return;
	const int32 NumBones = profile->Muscles.Num();

	for (int32 MuscleIndex = 0; MuscleIndex < NumBones; ++MuscleIndex)
	{
		DrawSingleArc(PDI, &profile->Muscles[MuscleIndex]);
	}
}

void FMassMuscleViewportClient::DrawSingleArc(FPrimitiveDrawInterface* PDI, FMassMuscleDataMuscle* Muscle)
{
	int32 BoneIndex = PreviewMeshComponent->GetBoneIndex(Muscle->ChildBoneName);
	if (BoneIndex == INDEX_NONE)
		return;

	FTransform BoneTM = PreviewMeshComponent->GetBoneTransform(BoneIndex);
	ArcCenter = BoneTM.GetLocation();
	ArcXAxis = BoneTM.GetUnitAxis(EAxis::X);
	ArcYAxis = BoneTM.GetUnitAxis(EAxis::Y);

	//ArcXAxis = Muscle->Orientation.RotateVector(ArcXAxis);
	//ArcYAxis = Muscle->Orientation.RotateVector(ArcYAxis);

	ArcRadius = 30.f;
	float MaxAngle = Muscle->MaxRange;
	MaxAngle = MaxAngle > Muscle->MinRange ? MaxAngle : MaxAngle + 360;

	PDI->SetHitProxy(new HMuscleArcProxy(Muscle->Name));
	DrawArc(PDI, ArcCenter, ArcXAxis, ArcYAxis, Muscle->MinRange, MaxAngle, ArcRadius, 32, FLinearColor::Yellow, SDPG_Foreground);
	PDI->SetHitProxy(nullptr);
}

void FMassMuscleViewportClient::DrawSelectedArc(FPrimitiveDrawInterface* PDI, FMassMuscleDataMuscle* Muscle)
{
	int32 BoneIndex = PreviewMeshComponent->GetBoneIndex(Muscle->BoneName);
	if (BoneIndex == INDEX_NONE)
		return;
	int32 ChildBoneIndex = PreviewMeshComponent->GetBoneIndex(Muscle->ChildBoneName);
	if (ChildBoneIndex == INDEX_NONE)
		return;

	FTransform BoneTM = PreviewMeshComponent->GetBoneTransform(BoneIndex);
	FTransform ChildBoneTM = PreviewMeshComponent->GetBoneTransform(ChildBoneIndex);
	ArcCenter = BoneTM.GetLocation();
	switch (Muscle->Orientation) {
		case ERotationType::Pitch:
			ArcXAxis = BoneTM.GetUnitAxis(EAxis::X);
			ArcYAxis = BoneTM.GetUnitAxis(EAxis::Y);
			break;

		case ERotationType::Roll:
			ArcXAxis = BoneTM.GetUnitAxis(EAxis::Z);
			ArcYAxis = BoneTM.GetUnitAxis(EAxis::Y);
			break;

		case ERotationType::Yaw:
			ArcXAxis = BoneTM.GetUnitAxis(EAxis::X);
			ArcYAxis = BoneTM.GetUnitAxis(EAxis::Z);
			break;
			
	}
	ArcRadius = 30.f;
	float MaxAngle = Muscle->MaxRange;
	MaxAngle = MaxAngle > Muscle->MinRange ? MaxAngle : MaxAngle + 360;
	DrawArc(PDI, ArcCenter, ArcXAxis, ArcYAxis, Muscle->MinRange, MaxAngle, ArcRadius, 32, FLinearColor::Yellow, SDPG_Foreground);

	auto HandlePos = [&](float AngleDeg) -> FVector
	{
		float Rad = FMath::DegreesToRadians(AngleDeg);
		return ArcCenter + (ArcXAxis * FMath::Cos(Rad) + ArcYAxis * FMath::Sin(Rad)) * ArcRadius;
	};

	PDI->SetHitProxy(new HMuscleHandleProxy(EMuscleHandle::Min));
	PDI->DrawPoint(HandlePos(Muscle->MinRange), (bIsDraggingHandle && DraggedHandle == EMuscleHandle::Min) ? FLinearColor::White : FLinearColor::Red, 14.f, SDPG_Foreground);
	PDI->SetHitProxy(nullptr);

	PDI->SetHitProxy(new HMuscleHandleProxy(EMuscleHandle::Max));
	PDI->DrawPoint(HandlePos(Muscle->MaxRange), (bIsDraggingHandle && DraggedHandle == EMuscleHandle::Max) ? FLinearColor::White : FLinearColor::Blue, 14.f, SDPG_Foreground);
	PDI->SetHitProxy(nullptr);

	const FTransform BoneTransform = PreviewMeshComponent->GetBoneTransform(BoneIndex);
	const FTransform ParentTransform = PreviewMeshComponent->GetBoneTransform(PreviewMeshComponent->GetParentBone(Muscle->BoneName));
	PDI->DrawLine(BoneTransform.GetLocation(), ParentTransform.GetLocation(), FColor::Orange, -1, 2.0f);
	const FTransform ChildTransform = PreviewMeshComponent->GetBoneTransform(ChildBoneIndex);
	PDI->DrawLine(BoneTransform.GetLocation(), ChildTransform.GetLocation(), FColor::Green, -1, 2.0f);
}


void FMassMuscleViewportClient::ProcessClick(FSceneView& View, HHitProxy* HitProxy, FKey Key, EInputEvent Event, uint32 HitX, uint32 HitY)
{
	if (!HitProxy)
	{
		FEditorViewportClient::ProcessClick(View, HitProxy, Key, Event, HitX, HitY);
		return;
	}
	if (HitProxy->IsA(HBoneLineProxy::StaticGetType()))
	{
		HBoneLineProxy* LineProxy = static_cast<HBoneLineProxy*>(HitProxy);

		// write back to the shared model → notifies all widgets
		Model->SetSelected(LineProxy->BoneName, EItemType::Bone);
		return;
	}
	if (HitProxy->IsA(HMuscleArcProxy::StaticGetType()))
	{
		HMuscleArcProxy* ArcProxy = static_cast<HMuscleArcProxy*>(HitProxy);

		// write back to the shared model → notifies all widgets
		Model->SetSelected(ArcProxy->MuscleName, EItemType::Muscle);
		return;
	}
	
	// fall through for default camera behaviour
	FEditorViewportClient::ProcessClick(View, HitProxy, Key, Event, HitX, HitY);
}

bool FMassMuscleViewportClient::InputKey(const FInputKeyEventArgs& EventArgs)
{
	if (EventArgs.Key == EKeys::LeftMouseButton)
	{
		if (EventArgs.Event == IE_Pressed)
		{
			HHitProxy* HitProxy = EventArgs.Viewport->GetHitProxy(EventArgs.Viewport->GetMouseX(), EventArgs.Viewport->GetMouseY());

			if (HitProxy && HitProxy->IsA(HMuscleHandleProxy::StaticGetType()))
			{
				HMuscleHandleProxy* HandleProxy = static_cast<HMuscleHandleProxy*>(HitProxy);

				bIsDraggingHandle = true;
				DraggedHandle = HandleProxy->HandleType;

				return true; // ✅ consume — stops camera orbit while dragging
			}
		}
		else if (EventArgs.Event == IE_Released && bIsDraggingHandle)
		{
			bIsDraggingHandle = false;
			return true;
		}
	}

	return FEditorViewportClient::InputKey(EventArgs);
}

void FMassMuscleViewportClient::CapturedMouseMove(FViewport* InViewport, int32 InMouseX, int32 InMouseY)
{
	if (!Model.IsValid())
	{
		FEditorViewportClient::CapturedMouseMove(InViewport, InMouseX, InMouseY);
		return;
	}
	if (bIsDraggingHandle)
	{

		UMassMuscleProfileAssetMuscle* profile = Model->GetMuscleProfile();
		int32 MuscleIndex = profile->FindMuscleByName(Model->GetSelected());
		if (MuscleIndex == INDEX_NONE)
			return;
		FMassMuscleDataMuscle* Muscle = &profile->Muscles[MuscleIndex];
		if (!Muscle)
			return;

		float NewAngle = ComputeAngleFromMouse(InViewport, InMouseX, InMouseY);

		if (DraggedHandle == EMuscleHandle::Min)
			Muscle->MinRange = NewAngle;
		else
			Muscle->MaxRange = NewAngle;

		profile->NotifyMuscleChange(); // ✅ tells details panel etc. to refresh
		Invalidate();				   // ✅ request viewport redraw
		
	}
}

float FMassMuscleViewportClient::ComputeAngleFromMouse(FViewport* InViewport, int32 X, int32 Y) const
{
	FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(InViewport, GetScene(), EngineShowFlags).SetRealtimeUpdate(IsRealtime()));

	FSceneView* View = const_cast<FMassMuscleViewportClient*>(this)->CalcSceneView(&ViewFamily);

	FViewportCursorLocation CursorLoc(View, const_cast<FMassMuscleViewportClient*>(this), X, Y);

	// ── Plane defined by the arc's center and normal ───────────────────────────
	FVector PlaneNormal = FVector::CrossProduct(ArcXAxis, ArcYAxis).GetSafeNormal();
	FPlane Plane(ArcCenter, PlaneNormal);

	FVector Intersection = FMath::RayPlaneIntersection(CursorLoc.GetOrigin(), CursorLoc.GetDirection(), Plane);

	FVector LocalDir = (Intersection - ArcCenter).GetSafeNormal();

	float X2 = FVector::DotProduct(LocalDir, ArcXAxis);
	float Y2 = FVector::DotProduct(LocalDir, ArcYAxis);

	float AngleDeg = FMath::RadiansToDegrees(FMath::Atan2(Y2, X2));
	if (AngleDeg < 0.f)
		AngleDeg += 360.f;

	return AngleDeg;
}

FMassMuscleViewportClient::~FMassMuscleViewportClient()
{
	if (Model.IsValid())
	{
		Model->OnSkeletalMeshChanged.RemoveAll(this);
	}
	FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
}