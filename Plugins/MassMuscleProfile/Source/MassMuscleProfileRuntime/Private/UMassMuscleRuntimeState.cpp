#include "UMassMuscleRuntimeState.h"
#include "Engine/SkeletalMesh.h"

namespace
{
    void LogChunked(const FString& Message)
    {
        constexpr int32 MaxChunkLen = 900;
        if (Message.Len() <= MaxChunkLen)
        {
            UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
            return;
        }

        int32 StartIndex = 0;
        while (StartIndex < Message.Len())
        {
            const int32 Remaining = Message.Len() - StartIndex;
            const int32 ChunkLen = FMath::Min(MaxChunkLen, Remaining);
            UE_LOG(LogTemp, Log, TEXT("%s"), *Message.Mid(StartIndex, ChunkLen));
            StartIndex += ChunkLen;
        }
    }
}

namespace
{
    void NormalizeAngleRangeForWrap(float& InOutMinAngle, float& InOutMaxAngle)
    {
        while (InOutMinAngle > InOutMaxAngle)
        {
            InOutMinAngle -= 360.0f;
        }
    }

    FVector GetRotationAxis(ERotationType RotationType)
    {
        switch (RotationType)
        {
        case ERotationType::Roll:
            return FVector::RightVector;
        case ERotationType::Pitch:
            return FVector::UpVector;
        case ERotationType::Yaw:
        default:
            return FVector::ForwardVector;
        }
    }
}

void UMassMuscleRuntimeState::Initialize(USkeletalMesh* SkeletalMesh, const TArray<FMassMuscleDataMuscle>& MuscleData, const TArray<FMassMuscleDataMass>& MassData)
{
    Muscles.Empty();
    BoneStates.Empty();
    BoneMasses.Empty();
    BoneIndexToMuscleIndices.Empty();

    if (!SkeletalMesh)
    {
        return;
    }

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();
    const int32 NumBones = RefSkeleton.GetNum();

    BoneStates.SetNum(NumBones);
    for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
    {
        FBoneKinematicState& BoneState = BoneStates[BoneIndex];
        BoneState.BoneIndex = BoneIndex;
        BoneState.CurrentTransform = FTransform::Identity;
        BoneState.PreviousRotation = FQuat::Identity;
        BoneState.AngularVelocity = FVector::ZeroVector;
    }

    BoneMasses.Init(1.0f, NumBones);

    for (const FMassMuscleDataMass& MassEntry : MassData)
    {
        int32 BoneIndex = MassEntry.BoneIndex;
        if (BoneIndex == INDEX_NONE && MassEntry.BoneName != NAME_None)
        {
            BoneIndex = RefSkeleton.FindBoneIndex(MassEntry.BoneName);
        }

        if (BoneIndex != INDEX_NONE && BoneMasses.IsValidIndex(BoneIndex))
        {
            BoneMasses[BoneIndex] = FMath::Max(0.0f, MassEntry.Mass);
        }
    }

    if (MuscleData.Num() == 0)
    {
        return;
    }

    Muscles.Reserve(MuscleData.Num());

    for (const FMassMuscleDataMuscle& MuscleEntry : MuscleData)
    {
        FMuscleConstraintData Constraint;
        Constraint.MuscleName = MuscleEntry.Name;
        Constraint.BoneName = MuscleEntry.BoneName;
        Constraint.BoneIndex = RefSkeleton.FindBoneIndex(MuscleEntry.BoneName);
        Constraint.RotationAxis = GetRotationAxis(MuscleEntry.Orientation);
        Constraint.MinAngle = MuscleEntry.MinRange;
        Constraint.MaxAngle = MuscleEntry.MaxRange;
        NormalizeAngleRangeForWrap(Constraint.MinAngle, Constraint.MaxAngle);
        Constraint.BaseStrength = MuscleEntry.Strength;

        const int32 MuscleIndex = Muscles.Add(Constraint);

        if (Constraint.BoneIndex != INDEX_NONE)
        {
            FMuscleIndexArray* ExistingIndices = BoneIndexToMuscleIndices.Find(Constraint.BoneIndex);
            if (ExistingIndices == nullptr)
            {
                FMuscleIndexArray NewIndexArray;
                NewIndexArray.Indices.Add(MuscleIndex);
                BoneIndexToMuscleIndices.Add(Constraint.BoneIndex, NewIndexArray);
            }
            else
            {
                ExistingIndices->Indices.Add(MuscleIndex);
            }
        }
    }
}

UMassMuscleRuntimeState* UMassMuscleRuntimeState::CreateInitializedRuntimeState(UObject* Outer, USkeletalMesh* SkeletalMesh, const TArray<FMassMuscleDataMuscle>& MuscleData, const TArray<FMassMuscleDataMass>& MassData)
{
    UObject* EffectiveOuter = Outer ? Outer : GetTransientPackage();
    UMassMuscleRuntimeState* RuntimeState = NewObject<UMassMuscleRuntimeState>(EffectiveOuter);
    if (RuntimeState)
    {
        RuntimeState->Initialize(SkeletalMesh, MuscleData, MassData);
    }

    return RuntimeState;
}

FString UMassMuscleRuntimeState::DebugDumpRuntimeState(bool bPrintToLog) const
{
    FString Dump;
    Dump += TEXT("\n============================================================\n");
    Dump += TEXT("= UMassMuscleRuntimeState Debug Dump\n");
    Dump += TEXT("============================================================\n");

    Dump += FString::Printf(TEXT("Muscles: %d\n"), Muscles.Num());
    Dump += FString::Printf(TEXT("BoneStates: %d\n"), BoneStates.Num());
    Dump += FString::Printf(TEXT("BoneMasses: %d\n"), BoneMasses.Num());
    Dump += FString::Printf(TEXT("BoneIndexToMuscleIndices: %d\n"), BoneIndexToMuscleIndices.Num());

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("MUSCLES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (Muscles.Num() == 0)
    {
        Dump += TEXT("(empty)\n");
    }
    else
    {
        for (int32 i = 0; i < Muscles.Num(); ++i)
        {
            const FMuscleConstraintData& Muscle = Muscles[i];
            Dump += TEXT("------------------------------\n");
            Dump += FString::Printf(TEXT("Muscle[%d]\n"), i);
            Dump += FString::Printf(TEXT("  MuscleName: %s\n"), *Muscle.MuscleName.ToString());
            Dump += FString::Printf(TEXT("  BoneName: %s\n"), *Muscle.BoneName.ToString());
            Dump += FString::Printf(TEXT("  BoneIndex: %d\n"), Muscle.BoneIndex);
            Dump += FString::Printf(TEXT("  RotationAxis: %s\n"), *Muscle.RotationAxis.ToString());
            Dump += FString::Printf(TEXT("  MinAngle: %.4f\n"), Muscle.MinAngle);
            Dump += FString::Printf(TEXT("  MaxAngle: %.4f\n"), Muscle.MaxAngle);
            Dump += FString::Printf(TEXT("  BaseStrength: %.4f\n"), Muscle.BaseStrength);
            Dump += FString::Printf(TEXT("  StrengthByAngleCurve Keys: %d\n"), Muscle.StrengthByAngleCurve.GetRichCurveConst()->GetNumKeys());
            Dump += FString::Printf(TEXT("  ExplosivityCurve Keys: %d\n"), Muscle.ExplosivityCurve.GetRichCurveConst()->GetNumKeys());
        }
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE STATES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (BoneStates.Num() == 0)
    {
        Dump += TEXT("(empty)\n");
    }
    else
    {
        for (int32 i = 0; i < BoneStates.Num(); ++i)
        {
            const FBoneKinematicState& BoneState = BoneStates[i];
            Dump += TEXT("------------------------------\n");
            Dump += FString::Printf(TEXT("BoneState[%d]\n"), i);
            Dump += FString::Printf(TEXT("  BoneIndex: %d\n"), BoneState.BoneIndex);
            Dump += FString::Printf(TEXT("  CurrentTransform.Location: %s\n"), *BoneState.CurrentTransform.GetLocation().ToString());
            Dump += FString::Printf(TEXT("  CurrentTransform.Rotation(Euler): %s\n"), *BoneState.CurrentTransform.GetRotation().Euler().ToString());
            Dump += FString::Printf(TEXT("  CurrentTransform.Scale3D: %s\n"), *BoneState.CurrentTransform.GetScale3D().ToString());
            Dump += FString::Printf(TEXT("  PreviousRotation(Euler): %s\n"), *BoneState.PreviousRotation.Euler().ToString());
            Dump += FString::Printf(TEXT("  AngularVelocity: %s\n"), *BoneState.AngularVelocity.ToString());
        }
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE MASSES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (BoneMasses.Num() == 0)
    {
        Dump += TEXT("(empty)\n");
    }
    else
    {
        for (int32 i = 0; i < BoneMasses.Num(); ++i)
        {
            Dump += FString::Printf(TEXT("BoneMass[%d] = %.4f\n"), i, BoneMasses[i]);
        }
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE INDEX -> MUSCLE INDICES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (BoneIndexToMuscleIndices.Num() == 0)
    {
        Dump += TEXT("(empty)\n");
    }
    else
    {
        TArray<int32> SortedBoneIndices;
        BoneIndexToMuscleIndices.GetKeys(SortedBoneIndices);
        SortedBoneIndices.Sort();

        for (const int32 BoneIndex : SortedBoneIndices)
        {
            const FMuscleIndexArray* Indices = BoneIndexToMuscleIndices.Find(BoneIndex);
            Dump += FString::Printf(TEXT("BoneIndex[%d]: "), BoneIndex);

            if (!Indices || Indices->Indices.Num() == 0)
            {
                Dump += TEXT("(empty)\n");
                continue;
            }

            for (int32 Idx = 0; Idx < Indices->Indices.Num(); ++Idx)
            {
                Dump += FString::Printf(TEXT("%d"), Indices->Indices[Idx]);
                if (Idx < Indices->Indices.Num() - 1)
                {
                    Dump += TEXT(", ");
                }
            }
            Dump += TEXT("\n");
        }
    }

    Dump += TEXT("============================================================\n");

    if (bPrintToLog)
    {
        LogChunked(Dump);
    }

    return Dump;
}

FString UMassMuscleRuntimeState::DebugDumpRuntimeStateForBones(const TArray<int32>& BoneIndices, bool bPrintToLog) const
{
    TSet<int32> RequestedBones;
    RequestedBones.Reserve(BoneIndices.Num());
    for (const int32 BoneIndex : BoneIndices)
    {
        RequestedBones.Add(BoneIndex);
    }

    TArray<int32> SortedRequestedBones = RequestedBones.Array();
    SortedRequestedBones.Sort();

    TArray<int32> InvalidBoneIndices;
    TSet<int32> ValidBoneSet;
    for (const int32 BoneIndex : SortedRequestedBones)
    {
        const bool bValidForState = BoneStates.IsValidIndex(BoneIndex);
        const bool bValidForMass = BoneMasses.IsValidIndex(BoneIndex);
        const bool bValidForMap = BoneIndexToMuscleIndices.Contains(BoneIndex);
        if (bValidForState || bValidForMass || bValidForMap)
        {
            ValidBoneSet.Add(BoneIndex);
        }
        else
        {
            InvalidBoneIndices.Add(BoneIndex);
        }
    }

    FString Dump;
    Dump += TEXT("\n============================================================\n");
    Dump += TEXT("= UMassMuscleRuntimeState Debug Dump (Filtered By Bone)\n");
    Dump += TEXT("============================================================\n");
    Dump += FString::Printf(TEXT("Requested Bone Indices: %d\n"), SortedRequestedBones.Num());
    Dump += FString::Printf(TEXT("Valid Bone Indices: %d\n"), ValidBoneSet.Num());
    Dump += FString::Printf(TEXT("Invalid Bone Indices: %d\n"), InvalidBoneIndices.Num());

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("REQUESTED BONE INDICES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (SortedRequestedBones.Num() == 0)
    {
        Dump += TEXT("(empty)\n");
    }
    else
    {
        for (const int32 BoneIndex : SortedRequestedBones)
        {
            Dump += FString::Printf(TEXT("BoneIndex[%d]\n"), BoneIndex);
        }
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("INVALID BONE INDICES\n");
    Dump += TEXT("------------------------------------------------------------\n");
    if (InvalidBoneIndices.Num() == 0)
    {
        Dump += TEXT("(none)\n");
    }
    else
    {
        for (const int32 InvalidBoneIndex : InvalidBoneIndices)
        {
            Dump += FString::Printf(TEXT("BoneIndex[%d] (not found in states/masses/map)\n"), InvalidBoneIndex);
        }
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("MUSCLES (MATCHING BONE INDEX)\n");
    Dump += TEXT("------------------------------------------------------------\n");
    int32 MatchingMuscleCount = 0;
    for (int32 i = 0; i < Muscles.Num(); ++i)
    {
        const FMuscleConstraintData& Muscle = Muscles[i];
        if (!ValidBoneSet.Contains(Muscle.BoneIndex))
        {
            continue;
        }

        ++MatchingMuscleCount;
        Dump += TEXT("------------------------------\n");
        Dump += FString::Printf(TEXT("Muscle[%d]\n"), i);
        Dump += FString::Printf(TEXT("  MuscleName: %s\n"), *Muscle.MuscleName.ToString());
        Dump += FString::Printf(TEXT("  BoneName: %s\n"), *Muscle.BoneName.ToString());
        Dump += FString::Printf(TEXT("  BoneIndex: %d\n"), Muscle.BoneIndex);
        Dump += FString::Printf(TEXT("  RotationAxis: %s\n"), *Muscle.RotationAxis.ToString());
        Dump += FString::Printf(TEXT("  MinAngle: %.4f\n"), Muscle.MinAngle);
        Dump += FString::Printf(TEXT("  MaxAngle: %.4f\n"), Muscle.MaxAngle);
        Dump += FString::Printf(TEXT("  BaseStrength: %.4f\n"), Muscle.BaseStrength);
        Dump += FString::Printf(TEXT("  StrengthByAngleCurve Keys: %d\n"), Muscle.StrengthByAngleCurve.GetRichCurveConst()->GetNumKeys());
        Dump += FString::Printf(TEXT("  ExplosivityCurve Keys: %d\n"), Muscle.ExplosivityCurve.GetRichCurveConst()->GetNumKeys());
    }
    if (MatchingMuscleCount == 0)
    {
        Dump += TEXT("(none)\n");
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE STATES (MATCHING BONE INDEX)\n");
    Dump += TEXT("------------------------------------------------------------\n");
    int32 MatchingStateCount = 0;
    for (const int32 BoneIndex : SortedRequestedBones)
    {
        if (!BoneStates.IsValidIndex(BoneIndex))
        {
            continue;
        }

        const FBoneKinematicState& BoneState = BoneStates[BoneIndex];
        ++MatchingStateCount;
        Dump += TEXT("------------------------------\n");
        Dump += FString::Printf(TEXT("BoneState[%d]\n"), BoneIndex);
        Dump += FString::Printf(TEXT("  BoneIndex: %d\n"), BoneState.BoneIndex);
        Dump += FString::Printf(TEXT("  CurrentTransform.Location: %s\n"), *BoneState.CurrentTransform.GetLocation().ToString());
        Dump += FString::Printf(TEXT("  CurrentTransform.Rotation(Euler): %s\n"), *BoneState.CurrentTransform.GetRotation().Euler().ToString());
        Dump += FString::Printf(TEXT("  CurrentTransform.Scale3D: %s\n"), *BoneState.CurrentTransform.GetScale3D().ToString());
        Dump += FString::Printf(TEXT("  PreviousRotation(Euler): %s\n"), *BoneState.PreviousRotation.Euler().ToString());
        Dump += FString::Printf(TEXT("  AngularVelocity: %s\n"), *BoneState.AngularVelocity.ToString());
    }
    if (MatchingStateCount == 0)
    {
        Dump += TEXT("(none)\n");
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE MASSES (MATCHING BONE INDEX)\n");
    Dump += TEXT("------------------------------------------------------------\n");
    int32 MatchingMassCount = 0;
    for (const int32 BoneIndex : SortedRequestedBones)
    {
        if (!BoneMasses.IsValidIndex(BoneIndex))
        {
            continue;
        }

        ++MatchingMassCount;
        Dump += FString::Printf(TEXT("BoneMass[%d] = %.4f\n"), BoneIndex, BoneMasses[BoneIndex]);
    }
    if (MatchingMassCount == 0)
    {
        Dump += TEXT("(none)\n");
    }

    Dump += TEXT("------------------------------------------------------------\n");
    Dump += TEXT("BONE INDEX -> MUSCLE INDICES (MATCHING BONE INDEX)\n");
    Dump += TEXT("------------------------------------------------------------\n");
    int32 MatchingMapCount = 0;
    for (const int32 BoneIndex : SortedRequestedBones)
    {
        const FMuscleIndexArray* Indices = BoneIndexToMuscleIndices.Find(BoneIndex);
        if (!Indices)
        {
            continue;
        }

        ++MatchingMapCount;
        Dump += FString::Printf(TEXT("BoneIndex[%d]: "), BoneIndex);
        if (Indices->Indices.Num() == 0)
        {
            Dump += TEXT("(empty)\n");
            continue;
        }

        for (int32 Idx = 0; Idx < Indices->Indices.Num(); ++Idx)
        {
            Dump += FString::Printf(TEXT("%d"), Indices->Indices[Idx]);
            if (Idx < Indices->Indices.Num() - 1)
            {
                Dump += TEXT(", ");
            }
        }
        Dump += TEXT("\n");
    }
    if (MatchingMapCount == 0)
    {
        Dump += TEXT("(none)\n");
    }

    Dump += TEXT("============================================================\n");

    if (bPrintToLog)
    {
        LogChunked(Dump);
    }

    return Dump;
}
