#include "FRigUnit_MuscleCCDIK.h"
#include "Modules/ModuleManager.h"
#include "Rigs/RigHierarchy.h"
#include "Algo/Reverse.h"

namespace
{
    float NormalizeAngleToConstraintRange(float AngleDegrees, float MinDegrees, float MaxDegrees)
    {
        while (MinDegrees > MaxDegrees)
        {
            MinDegrees -= 360.0f;
        }

        while (AngleDegrees < MinDegrees)
        {
            AngleDegrees += 360.0f;
        }

        while (AngleDegrees > MaxDegrees)
        {
            AngleDegrees -= 360.0f;
        }

        return FMath::Clamp(AngleDegrees, MinDegrees, MaxDegrees);
    }

    float GetSignedTwistAngleDegrees(const FQuat& RelativeRotation, const FVector& AxisLocal)
    {
        FVector TwistAxis = AxisLocal.GetSafeNormal();
        if (TwistAxis.IsNearlyZero())
        {
            return 0.0f;
        }

        FQuat SafeRelative = RelativeRotation.GetNormalized();
        FVector RelativeVector(SafeRelative.X, SafeRelative.Y, SafeRelative.Z);
        const FVector ProjectedVector = TwistAxis * FVector::DotProduct(RelativeVector, TwistAxis);
        FQuat TwistQuat(ProjectedVector.X, ProjectedVector.Y, ProjectedVector.Z, SafeRelative.W);
        TwistQuat.Normalize();

        const FVector TwistVector(TwistQuat.X, TwistQuat.Y, TwistQuat.Z);
        const float SignedRadians = 2.0f * FMath::Atan2(FVector::DotProduct(TwistVector, TwistAxis), TwistQuat.W);
        return FMath::RadiansToDegrees(SignedRadians);
    }

    void ClampLocalRotationToAxis(
        FQuat& InOutLocalRotation,
        const FQuat& InitialLocalRotation,
        const FVector& AxisCleanLocal,
        const float MinAngleDegrees,
        const float MaxAngleDegrees)
    {
        const FVector SafeAxis = AxisCleanLocal.GetSafeNormal();
        if (SafeAxis.IsNearlyZero())
        {
            return;
        }

        const FQuat SafeInitial = InitialLocalRotation.GetNormalized();
        const FQuat SafeLocal = InOutLocalRotation.GetNormalized();
        const FQuat CleanLocalRotation = (SafeLocal * SafeInitial.Inverse()).GetNormalized();

        const float CurrentAngleDegrees = GetSignedTwistAngleDegrees(CleanLocalRotation, SafeAxis);
        const float ClampedAngleDegrees = NormalizeAngleToConstraintRange(CurrentAngleDegrees, MinAngleDegrees, MaxAngleDegrees);

        const FQuat ClampedCleanRotation(SafeAxis, FMath::DegreesToRadians(ClampedAngleDegrees));
        InOutLocalRotation = (ClampedCleanRotation * SafeInitial).GetNormalized();
    }

    bool SolveChainWithCustomCCD(
        URigHierarchy* Hierarchy,
        const TArray<FCachedRigElement>& CachedChain,
        const FVector& EffectorTarget,
        const float Precision,
        const int32 MaxIterations,
        const bool bUseAxisConstraints,
        const TArray<FVector>& RotationAxisLocalPerItem,
        const TArray<FQuat>& InitialLocalRotationPerItem,
        const TArray<float>& MinAnglePerItem,
        const TArray<float>& MaxAnglePerItem,
        const bool bUseRotationLimits,
        const TArray<float>& RotationLimitsPerItem,
        TArray<FTransform>& OutChainLocalTransforms)
    {
        if (CachedChain.Num() < 2 || Hierarchy == nullptr)
        {
            return false;
        }

        OutChainLocalTransforms.Empty();
        OutChainLocalTransforms.Reserve(CachedChain.Num());
        for (FCachedRigElement CachedElement : CachedChain)
        {
            if (!CachedElement.UpdateCache(Hierarchy))
            {
                return false;
            }
            OutChainLocalTransforms.Add(Hierarchy->GetLocalTransform(CachedElement));
        }

        const float PrecisionSq = FMath::Square(FMath::Max(Precision, 0.001f));
        const int32 LastIndex = OutChainLocalTransforms.Num() - 1;
        const int32 Iterations = FMath::Max(MaxIterations, 1);
        const float MaxStepRadians = FMath::DegreesToRadians(25.0f);
        const float DeltaDamping = 0.6f;

        TArray<FTransform> ChainWorldTransforms;
        ChainWorldTransforms.Reserve(OutChainLocalTransforms.Num());

        FTransform RootParentWorld = FTransform::Identity;
        if (CachedChain.Num() >= 2)
        {
            const FRigElementKey RootKey = CachedChain[0].GetResolvedKey();
            const FRigElementKey ParentOfRoot = Hierarchy->GetFirstParent(RootKey);
            if (ParentOfRoot.IsValid())
            {
                RootParentWorld = Hierarchy->GetGlobalTransform(ParentOfRoot);
            }
        }

        auto RebuildWorldTransforms = [&](TArray<FTransform>& LocalTransforms, TArray<FTransform>& OutWorldTransforms)
        {
            OutWorldTransforms.Reset();
            OutWorldTransforms.Reserve(LocalTransforms.Num());
            FTransform CurrentParentWorld = RootParentWorld;
            for (int32 Index = 0; Index < LocalTransforms.Num(); ++Index)
            {
                const FTransform WorldTransform = CurrentParentWorld * LocalTransforms[Index];
                OutWorldTransforms.Add(WorldTransform);
                CurrentParentWorld = WorldTransform;
            }
        };

        RebuildWorldTransforms(OutChainLocalTransforms, ChainWorldTransforms);

        auto GetTipErrorSq = [&]() -> float
        {
            if (ChainWorldTransforms.Num() == 0)
            {
                return TNumericLimits<float>::Max();
            }
            return FVector::DistSquared(ChainWorldTransforms.Last().GetLocation(), EffectorTarget);
        };

        float BestErrorSq = GetTipErrorSq();
        TArray<FTransform> BestLocalTransforms = OutChainLocalTransforms;

        if (BestErrorSq <= PrecisionSq)
        {
            return true;
        }

        for (int32 Iteration = 0; Iteration < Iterations; ++Iteration)
        {
            for (int32 JointIndex = LastIndex - 1; JointIndex >= 0; --JointIndex)
            {
                const FVector JointPos = ChainWorldTransforms[JointIndex].GetLocation();
                const FVector TipPos = ChainWorldTransforms[LastIndex].GetLocation();

                FVector ToTip = TipPos - JointPos;
                FVector ToTarget = EffectorTarget - JointPos;

                if (ToTip.IsNearlyZero() || ToTarget.IsNearlyZero())
                {
                    continue;
                }

                ToTip.Normalize();
                ToTarget.Normalize();

                FQuat DeltaRotation = FQuat::Identity;
                bool bHasDeltaRotation = false;
                bool bJointHasAxisConstraint = false;
                bool bUsedSignedAxisStep = false;
                const FTransform ParentWorldTransform = (JointIndex == 0) ? RootParentWorld : ChainWorldTransforms[JointIndex - 1];

                if (bUseAxisConstraints &&
                    RotationAxisLocalPerItem.IsValidIndex(JointIndex) &&
                    !RotationAxisLocalPerItem[JointIndex].IsNearlyZero())
                {
                    bJointHasAxisConstraint = true;
                    FVector AxisLocalAtRest = RotationAxisLocalPerItem[JointIndex];
                    if (InitialLocalRotationPerItem.IsValidIndex(JointIndex))
                    {
                        AxisLocalAtRest = InitialLocalRotationPerItem[JointIndex].RotateVector(AxisLocalAtRest);
                    }

                    FVector AxisWorld = ParentWorldTransform.TransformVectorNoScale(AxisLocalAtRest);
                    if (AxisWorld.Normalize())
                    {
                        const FVector TipPlanar = (ToTip - FVector::DotProduct(ToTip, AxisWorld) * AxisWorld).GetSafeNormal();
                        const FVector TargetPlanar = (ToTarget - FVector::DotProduct(ToTarget, AxisWorld) * AxisWorld).GetSafeNormal();

                        if (!TipPlanar.IsNearlyZero() && !TargetPlanar.IsNearlyZero())
                        {
                            const float DotValue = FMath::Clamp(FVector::DotProduct(TipPlanar, TargetPlanar), -1.0f, 1.0f);
                            float SignedDeltaAngle = FMath::Atan2(
                                FVector::DotProduct(AxisWorld, FVector::CrossProduct(TipPlanar, TargetPlanar)),
                                DotValue);

                            SignedDeltaAngle *= DeltaDamping;

                            if (bUseRotationLimits && RotationLimitsPerItem.IsValidIndex(JointIndex))
                            {
                                const float LimitDegrees = FMath::Clamp(RotationLimitsPerItem[JointIndex], 1.0f, 179.0f);
                                const float LimitRadians = FMath::DegreesToRadians(LimitDegrees);
                                SignedDeltaAngle = FMath::Clamp(SignedDeltaAngle, -LimitRadians, LimitRadians);
                            }

                            SignedDeltaAngle = FMath::Clamp(SignedDeltaAngle, -MaxStepRadians, MaxStepRadians);

                            if (!FMath::IsNearlyZero(SignedDeltaAngle))
                            {
                                DeltaRotation = FQuat(AxisWorld, SignedDeltaAngle);
                                bHasDeltaRotation = true;
                                bUsedSignedAxisStep = true;
                            }
                        }
                    }
                }

                if (!bHasDeltaRotation)
                {
                    if (bJointHasAxisConstraint)
                    {
                        continue;
                    }

                    DeltaRotation = FQuat::FindBetweenNormals(ToTip, ToTarget);
                    if (!DeltaRotation.IsNormalized())
                    {
                        DeltaRotation.Normalize();
                    }

                    if (!DeltaRotation.IsNormalized())
                    {
                        continue;
                    }
                }

                if (!bUsedSignedAxisStep)
                {
                    float EffectiveLimitRadians = MaxStepRadians;
                    if (bUseRotationLimits && RotationLimitsPerItem.IsValidIndex(JointIndex))
                    {
                        const float LimitDegrees = FMath::Clamp(RotationLimitsPerItem[JointIndex], 1.0f, 179.0f);
                        EffectiveLimitRadians = FMath::Min(EffectiveLimitRadians, FMath::DegreesToRadians(LimitDegrees));
                    }

                    const float DeltaAngle = DeltaRotation.GetAngle();
                    if (!FMath::IsNearlyZero(DeltaAngle))
                    {
                        FVector DeltaAxis = DeltaRotation.GetRotationAxis();
                        if (!DeltaAxis.Normalize())
                        {
                            DeltaAxis = FVector::UpVector;
                        }

                        const float ClampedDeltaAngle = FMath::Min(DeltaAngle, EffectiveLimitRadians);
                        DeltaRotation = FQuat(DeltaAxis, ClampedDeltaAngle);
                    }
                }

                const float PreviousErrorSq = GetTipErrorSq();
                const FQuat PreviousLocalRotation = OutChainLocalTransforms[JointIndex].GetRotation();

                const FQuat CurrentWorldRotation = ChainWorldTransforms[JointIndex].GetRotation();
                const FQuat NewWorldRotation = DeltaRotation * CurrentWorldRotation;
                FQuat NewLocalRotation = ParentWorldTransform.GetRotation().Inverse() * NewWorldRotation;
                if (!NewLocalRotation.IsNormalized())
                {
                    NewLocalRotation.Normalize();
                }

                if (bJointHasAxisConstraint &&
                    InitialLocalRotationPerItem.IsValidIndex(JointIndex) &&
                    MinAnglePerItem.IsValidIndex(JointIndex) &&
                    MaxAnglePerItem.IsValidIndex(JointIndex))
                {
                    ClampLocalRotationToAxis(
                        NewLocalRotation,
                        InitialLocalRotationPerItem[JointIndex],
                        RotationAxisLocalPerItem[JointIndex],
                        MinAnglePerItem[JointIndex],
                        MaxAnglePerItem[JointIndex]);
                }

                FTransform& JointLocalTransform = OutChainLocalTransforms[JointIndex];
                JointLocalTransform.SetRotation(NewLocalRotation.GetNormalized());

                RebuildWorldTransforms(OutChainLocalTransforms, ChainWorldTransforms);
                const float NewErrorSq = GetTipErrorSq();

                if (NewErrorSq > PreviousErrorSq)
                {
                    JointLocalTransform.SetRotation(PreviousLocalRotation);
                    RebuildWorldTransforms(OutChainLocalTransforms, ChainWorldTransforms);
                    continue;
                }

                if (NewErrorSq < BestErrorSq)
                {
                    BestErrorSq = NewErrorSq;
                    BestLocalTransforms = OutChainLocalTransforms;
                }

                if (NewErrorSq <= PrecisionSq)
                {
                    return true;
                }
            }
        }

        if (BestLocalTransforms.Num() == OutChainLocalTransforms.Num())
        {
            OutChainLocalTransforms = BestLocalTransforms;
            RebuildWorldTransforms(OutChainLocalTransforms, ChainWorldTransforms);
        }

        return BestErrorSq <= PrecisionSq;
    }
}

FRigUnit_MuscleCCDIK_Execute()
{
    URigHierarchy* Hierarchy = ExecuteContext.Hierarchy;
    bReachedEffector = false;
    if (Hierarchy == nullptr)
    {
        CachedChain.Empty();
        AffectedBoneIndices.Empty();
        return;
    }

    AffectedBoneIndices.Empty();

    if (!RootBone.IsValid() || !TipBone.IsValid())
    {
        CachedChain.Empty();
        return;
    }

    FRigElementKey RootKey = RootBone;
    FRigElementKey TipKey = TipBone;
    RootKey.Type = ERigElementType::Bone;
    TipKey.Type = ERigElementType::Bone;

    if (!Hierarchy->Contains(RootKey) || !Hierarchy->Contains(TipKey))
    {
        CachedChain.Empty();
        return;
    }

    TArray<FRigElementKey> ChainKeys;
    ChainKeys.Reserve(32);

    FRigElementKey Current = TipKey;
    while (Current.IsValid())
    {
        ChainKeys.Add(Current);
        if (Current == RootKey)
        {
            break;
        }

        Current = Hierarchy->GetFirstParent(Current);
    }

    if (ChainKeys.Num() < 2 || ChainKeys.Last() != RootKey)
    {
        CachedChain.Empty();
        return;
    }

    Algo::Reverse(ChainKeys);

    CachedChain.Empty();
    CachedChain.Reserve(ChainKeys.Num());

    for (const FRigElementKey& ChainKey : ChainKeys)
    {
        CachedChain.Add(FCachedRigElement(ChainKey, Hierarchy));
    }

    if (CachedChain.Num() < 2)
    {
        return;
    }

    FTransform RootParentWorldTransform = FTransform::Identity;
    const FRigElementKey ParentOfRoot = Hierarchy->GetFirstParent(CachedChain[0].GetResolvedKey());
    if (ParentOfRoot.IsValid())
    {
        RootParentWorldTransform = Hierarchy->GetGlobalTransform(ParentOfRoot);
    }

    const FTransform RootWorldTransform = Hierarchy->GetGlobalTransform(CachedChain[0]);
    FVector ResolvedEffectorTarget = EffectorTarget;
    switch (EffectorSpace)
    {
        case EMuscleCCDIKEffectorSpace::RootParentLocal:
            ResolvedEffectorTarget = RootParentWorldTransform.TransformPosition(EffectorTarget);
            break;
        case EMuscleCCDIKEffectorSpace::RootLocal:
            ResolvedEffectorTarget = RootWorldTransform.TransformPosition(EffectorTarget);
            break;
        case EMuscleCCDIKEffectorSpace::Global:
        default:
            break;
    }

    TArray<FTransform> SolverChainLocal;
    FQuat OriginalTipLocalRotation = FQuat::Identity;
    FQuat InitialTipLocalRotation = FQuat::Identity;

    if (bPreserveTipLocalRotation)
    {
        OriginalTipLocalRotation = Hierarchy->GetLocalTransform(CachedChain.Last()).GetRotation();
        InitialTipLocalRotation = Hierarchy->GetInitialLocalTransform(CachedChain.Last()).GetRotation();
    }

    TArray<float> RotationLimitsPerItem;
    RotationLimitsPerItem.Init(180.0f, CachedChain.Num());
    bool bUseRotationLimits = false;
    TArray<FVector> RotationAxisLocalPerItem;
    RotationAxisLocalPerItem.Init(FVector::ZeroVector, CachedChain.Num());
    TArray<FQuat> InitialLocalRotationPerItem;
    InitialLocalRotationPerItem.Init(FQuat::Identity, CachedChain.Num());
    TArray<float> MinAnglePerItem;
    MinAnglePerItem.Init(-179.0f, CachedChain.Num());
    TArray<float> MaxAnglePerItem;
    MaxAnglePerItem.Init(179.0f, CachedChain.Num());
    bool bUseAxisConstraints = false;

    for (int32 ChainIndex = 0; ChainIndex < CachedChain.Num(); ++ChainIndex)
    {
        InitialLocalRotationPerItem[ChainIndex] =
            Hierarchy->GetInitialLocalTransform(CachedChain[ChainIndex]).GetRotation().GetNormalized();
    }

    if (MuscleData)
    {
        TMap<FName, float> LimitPerBoneName;
        TMap<FName, FVector> AxisPerBoneName;
        TMap<FName, FVector2D> AngleRangePerBoneName;
        for (const FMuscleConstraintData& Constraint : MuscleData->Muscles)
        {
            if (Constraint.BoneName == NAME_None)
            {
                continue;
            }

            const float ConstraintLimit = FMath::Max(FMath::Abs(Constraint.MinAngle), FMath::Abs(Constraint.MaxAngle));
            const float ClampedLimit = FMath::Clamp(ConstraintLimit, 1.0f, 179.0f);

            float* ExistingLimit = LimitPerBoneName.Find(Constraint.BoneName);
            if (ExistingLimit)
            {
                *ExistingLimit = FMath::Max(*ExistingLimit, ClampedLimit);
            }
            else
            {
                LimitPerBoneName.Add(Constraint.BoneName, ClampedLimit);
            }

            if (!Constraint.RotationAxis.IsNearlyZero())
            {
                FVector AxisLocal = Constraint.RotationAxis.GetSafeNormal();
                if (!AxisLocal.IsNearlyZero())
                {
                    FVector* ExistingAxis = AxisPerBoneName.Find(Constraint.BoneName);
                    if (ExistingAxis == nullptr)
                    {
                        AxisPerBoneName.Add(Constraint.BoneName, AxisLocal);
                    }
                    else
                    {
                        *ExistingAxis = AxisLocal;
                    }
                }
            }

            FVector2D* ExistingRange = AngleRangePerBoneName.Find(Constraint.BoneName);
            if (ExistingRange == nullptr)
            {
                AngleRangePerBoneName.Add(Constraint.BoneName, FVector2D(Constraint.MinAngle, Constraint.MaxAngle));
            }
            else
            {
                ExistingRange->X = FMath::Min(ExistingRange->X, Constraint.MinAngle);
                ExistingRange->Y = FMath::Max(ExistingRange->Y, Constraint.MaxAngle);
            }
        }

        for (int32 ChainIndex = 0; ChainIndex < CachedChain.Num(); ++ChainIndex)
        {
            const FRigElementKey ChainKey = CachedChain[ChainIndex].GetResolvedKey();
            if (const float* Limit = LimitPerBoneName.Find(ChainKey.Name))
            {
                RotationLimitsPerItem[ChainIndex] = *Limit;
                bUseRotationLimits = true;
            }

            if (const FVector* Axis = AxisPerBoneName.Find(ChainKey.Name))
            {
                FVector AxisLocal = Axis->GetSafeNormal();
                RotationAxisLocalPerItem[ChainIndex] = AxisLocal;
                bUseAxisConstraints = true;
            }

            if (const FVector2D* Range = AngleRangePerBoneName.Find(ChainKey.Name))
            {
                MinAnglePerItem[ChainIndex] = Range->X;
                MaxAnglePerItem[ChainIndex] = Range->Y;
            }
        }
    }

    bReachedEffector = SolveChainWithCustomCCD(
        Hierarchy,
        CachedChain,
        ResolvedEffectorTarget,
        Precision,
        MaxIterations,
        bUseAxisConstraints,
        RotationAxisLocalPerItem,
        InitialLocalRotationPerItem,
        MinAnglePerItem,
        MaxAnglePerItem,
        bUseRotationLimits,
        RotationLimitsPerItem,
        SolverChainLocal);

    for (int32 ChainIndex = 0; ChainIndex < CachedChain.Num(); ++ChainIndex)
    {
        Hierarchy->SetLocalTransform(CachedChain[ChainIndex], SolverChainLocal[ChainIndex], true);
    }

    if (bPreserveTipLocalRotation)
    {
        FTransform TipLocalTransform = Hierarchy->GetLocalTransform(CachedChain.Last());
        const FQuat PreservedTipLocal = bPreserveTipInitialLocalRotation ? InitialTipLocalRotation : OriginalTipLocalRotation;
        TipLocalTransform.SetRotation(PreservedTipLocal.GetNormalized());
        Hierarchy->SetLocalTransform(CachedChain.Last(), TipLocalTransform, true);
    }

    if (MuscleData)
    {
        TSet<FName> SolvedBoneNames;
        for (const FRigElementKey& ChainKey : ChainKeys)
        {
            SolvedBoneNames.Add(ChainKey.Name);
        }

        TSet<int32> UniqueAffectedBoneIndices;
        for (const FMuscleConstraintData& Constraint : MuscleData->Muscles)
        {
            if (Constraint.BoneIndex != INDEX_NONE && SolvedBoneNames.Contains(Constraint.BoneName))
            {
                UniqueAffectedBoneIndices.Add(Constraint.BoneIndex);
            }
        }

        AffectedBoneIndices = UniqueAffectedBoneIndices.Array();
        AffectedBoneIndices.Sort();
    }
}

IMPLEMENT_MODULE(FDefaultModuleImpl, MassMuscleProfileRuntime)
