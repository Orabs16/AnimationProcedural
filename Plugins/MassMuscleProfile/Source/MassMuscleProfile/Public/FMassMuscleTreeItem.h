#pragma once

#include "CoreMinimal.h"

enum class EItemType
{
    Bone,
    Muscle,
    Mass
};

struct FMassMuscleTreeItem
{
    FName Name;

    int32 BoneIndex = INDEX_NONE;

    int32 ParentIndex = INDEX_NONE;

    TArray<TSharedPtr<FMassMuscleTreeItem>> Children;
    EItemType ItemType;

    FMassMuscleTreeItem(const FName& InName,
                        int32 InBoneIndex,
                        int32 InParentIndex,
                        EItemType InItemType)
        : Name(InName)
        , BoneIndex(InBoneIndex)
        , ParentIndex(InParentIndex)
        , ItemType(InItemType)
    {
    }
};
