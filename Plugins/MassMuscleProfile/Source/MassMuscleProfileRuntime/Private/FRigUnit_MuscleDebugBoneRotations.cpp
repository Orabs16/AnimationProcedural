#include "FRigUnit_MuscleDebugBoneRotations.h"
#include "Rigs/RigHierarchy.h"

FRigUnit_MuscleDebugBoneRotationsV2_Execute()
{
    DebuggedBoneCount = 0;

    URigHierarchy* Hierarchy = ExecuteContext.Hierarchy;
    if (Hierarchy == nullptr)
    {
        if (bPrintToLog)
        {
            UE_LOG(LogTemp, Warning, TEXT("Muscle Debug Bone Rotations V2: Hierarchy is null."));
        }
        return;
    }

    FString AccumulatedDebug;
    AccumulatedDebug += TEXT("=== Muscle Debug Bone Rotations ===\n");

    for (const FRigElementKey& BoneInputKey : Bones)
    {
        FRigElementKey BoneKey = BoneInputKey;
        BoneKey.Type = ERigElementType::Bone;

        const bool bBoneIsValid = Hierarchy->Contains(BoneKey);
        FVector WorldEuler = FVector::ZeroVector;
        FVector LocalEuler = FVector::ZeroVector;
        FVector CleanLocalEuler = FVector::ZeroVector;

        if (bBoneIsValid)
        {
            const FTransform WorldTransform = Hierarchy->GetGlobalTransform(BoneKey);
            const FTransform LocalTransform = Hierarchy->GetLocalTransform(BoneKey);
            const FTransform InitialLocalTransform = Hierarchy->GetInitialLocalTransform(BoneKey);

            const FQuat CleanLocalQuat = (LocalTransform.GetRotation() * InitialLocalTransform.GetRotation().Inverse()).GetNormalized();

            WorldEuler = WorldTransform.GetRotation().Euler();
            LocalEuler = LocalTransform.GetRotation().Euler();
            CleanLocalEuler = CleanLocalQuat.Euler();
            ++DebuggedBoneCount;
        }

        if (bBoneIsValid)
        {
            AccumulatedDebug += FString::Printf(
                TEXT("%s | World=(P=%.3f Y=%.3f R=%.3f) | Local=(P=%.3f Y=%.3f R=%.3f) | CleanLocal=(P=%.3f Y=%.3f R=%.3f)\n"),
                *BoneKey.Name.ToString(),
                WorldEuler.Y,
                WorldEuler.Z,
                WorldEuler.X,
                LocalEuler.Y,
                LocalEuler.Z,
                LocalEuler.X,
                CleanLocalEuler.Y,
                CleanLocalEuler.Z,
                CleanLocalEuler.X);
        }
        else
        {
            AccumulatedDebug += FString::Printf(
                TEXT("%s | INVALID BONE\n"),
                *BoneInputKey.Name.ToString());
        }
    }

    if (bPrintToLog)
    {
        UE_LOG(LogTemp, Log, TEXT("%s"), *AccumulatedDebug);
    }
}
