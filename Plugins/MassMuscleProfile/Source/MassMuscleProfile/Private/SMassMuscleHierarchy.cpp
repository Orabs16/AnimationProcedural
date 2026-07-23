#include "SMassMuscleHierarchy.h"

#include "Containers/Array.h"
#include "CoreGlobals.h"
#include "FMassMuscleData.h"
#include "FMassMuscleStyle.h"
#include "FMassMuscleTreeItem.h"
#include "FMassMuscleEditorModel.h"
#include "Logging/LogMacros.h"
#include "Logging/LogVerbosity.h"
#include "Math/Color.h"
#include "MeshTypes.h"
#include "Misc/CoreMiscDefines.h"
#include "Templates/SharedPointer.h"
#include "UMassMuscleProfileAsset.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Views/STableRow.h"
#include "Widgets/Views/STableViewBase.h"
#include "Engine/SkeletalMesh.h"
#include "Widgets/Views/STreeView.h"
#include <cstddef>
#include "FMassMuscleStyle.h"

void SMassMuscleHierarchy::Construct(const FArguments& InArgs)
{
	Model = InArgs._Model;
	Model->OnSkeletalMeshChanged.AddSP(this, &SMassMuscleHierarchy::OnMeshChanged);
	Model->OnSelectionChanged.AddSP(this, &SMassMuscleHierarchy::OnExteriorSelectionChanged);
	Model->OnMuscleAdd.AddSP(this, &SMassMuscleHierarchy::OnAddMuscle);

	Settings = InArgs._Settings;
	Settings->OnHierarchyExtend.AddSP(this, &SMassMuscleHierarchy::ExtendHierarchy);
	Settings->OnHierarchyCollapse.AddSP(this, &SMassMuscleHierarchy::CollapseHierarchy);


	ChildSlot[SAssignNew(TreeView, STreeView<TSharedPtr<FMassMuscleTreeItem>>)
				  	.TreeItemsSource(&RootItems)

				  	.OnGenerateRow(this, &SMassMuscleHierarchy::OnGenerateRow)

				  	.OnGetChildren(this, &SMassMuscleHierarchy::OnGetChildren)
				  	.OnSelectionChanged(this, &SMassMuscleHierarchy::OnSelectionChanged)];

	if (Model->GetSkeletalMesh())
	{
		SetSkeletalMesh(Model->GetSkeletalMesh());
	}
	ExtendHierarchy();
}

void SMassMuscleHierarchy::OnGetChildren(TSharedPtr<FMassMuscleTreeItem> Item, TArray<TSharedPtr<FMassMuscleTreeItem>>& OutChildren)
{
	OutChildren = Item->Children;
}

TSharedRef<ITableRow> SMassMuscleHierarchy::OnGenerateRow(TSharedPtr<FMassMuscleTreeItem> Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	return SNew(STableRow<TSharedPtr<FMassMuscleTreeItem>>, OwnerTable)
    [
        SNew(STextBlock).Text_Lambda([Item](){ return FText::FromName(Item->Name); })
        .ColorAndOpacity(Item->ItemType==EItemType::Muscle? 
			FMassMuscleStyle::MuscleColor : 
			Item->Children.Num()>0? FLinearColor::White : FLinearColor::White*0.3f)
			
    ];
}

void SMassMuscleHierarchy::SetSkeletalMesh(USkeletalMesh* InMesh)
{
	SkeletalMesh = InMesh;

	BuildTree();
	if (TreeView.IsValid())
	{
		TreeView->RequestTreeRefresh();
	}
}

void SMassMuscleHierarchy::OnMeshChanged(USkeletalMesh* NewMesh)
{
	SetSkeletalMesh(NewMesh);
}

void SMassMuscleHierarchy::OnSelectionChanged(TSharedPtr<FMassMuscleTreeItem> Item, ESelectInfo::Type SelectInfo)
{
	if (Item.IsValid())
	{
		Model->SetSelected(Item->Name, Item->ItemType);
	}
}

void SMassMuscleHierarchy::BuildTree()
{
	RootItems.Empty();

	if (!SkeletalMesh)
	{
		return;
	}

	const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();

	const int32 NumBones = RefSkeleton.GetNum();

	TreeItems.SetNum(NumBones);

	// Create nodes
	for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
	{
		const FName BoneName = RefSkeleton.GetBoneName(BoneIndex);

		const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);

		TreeItems[BoneIndex] = MakeShared<FMassMuscleTreeItem>(BoneName, BoneIndex, ParentIndex, EItemType::Bone);
	}

	// Build hierarchy
	for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
	{
		const int32 ParentIndex = TreeItems[BoneIndex]->ParentIndex;

		if (ParentIndex == INDEX_NONE)
		{
			RootItems.Add(TreeItems[BoneIndex]);
		}
		else
		{
			TreeItems[ParentIndex]->Children.Add(TreeItems[BoneIndex]);
		}
	}

	UMassMuscleProfileAssetMuscle* profile = Model->GetMuscleProfile();
	if (!profile)
	{
		return;
	}
	const int32 musclesNum = profile->Muscles.Num();
	for (int32 MuscleIndex = 0; MuscleIndex < musclesNum; ++MuscleIndex)
	{
		AddMuscleToHierarchy(profile->Muscles[MuscleIndex]);
	}
}

void SMassMuscleHierarchy::ExtendHierarchy()
{
	SMassMuscleHierarchy::SetItemExpansionRecursive(RootItems[0], true);
}

void SMassMuscleHierarchy::CollapseHierarchy()
{
	SMassMuscleHierarchy::SetItemExpansionRecursive(RootItems[0], false);
}

void SMassMuscleHierarchy::SetItemExpansionRecursive(TSharedPtr<FMassMuscleTreeItem> Item, bool bExpand)
{
	if (!Item.IsValid() || !TreeView.IsValid())
	{
		return;
	}

	TreeView->SetItemExpansion(Item, bExpand);

	for (const TSharedPtr<FMassMuscleTreeItem>& Child : Item->Children)
	{
		SetItemExpansionRecursive(Child, bExpand);
	}
}

void SMassMuscleHierarchy::OnExteriorSelectionChanged(FName NewSelectionName, EItemType Type)
{
	const TSharedPtr<FMassMuscleTreeItem>& NewSelection = FindItemByName(NewSelectionName);
	if (!NewSelection)
	{
		return;
	}
	TreeView->SetSelection(NewSelection);
}

const TSharedPtr<FMassMuscleTreeItem>& SMassMuscleHierarchy::FindItemByName(FName ToFind)
{
	const TSharedPtr<FMassMuscleTreeItem>* PointerToSelection = (TreeItems.FindByPredicate(
		[&](const TSharedPtr<FMassMuscleTreeItem> inItem)
		{
			return inItem.IsValid() && inItem->Name == ToFind;
		}
	));

	if (PointerToSelection == nullptr || !PointerToSelection->IsValid())
	{
		static const TSharedPtr<FMassMuscleTreeItem> EmptySelection;
		return EmptySelection;
	}
	return *PointerToSelection;
}

void SMassMuscleHierarchy::AddMuscleToHierarchy(FMassMuscleDataMuscle Muscle)
{
	const TSharedPtr<FMassMuscleTreeItem> parent = FindItemByName(Muscle.BoneName);
	if (!parent)
		return;

	TSharedPtr<FMassMuscleTreeItem> item = MakeShared<FMassMuscleTreeItem>(Muscle.Name, NULL, parent->BoneIndex, EItemType::Muscle);

	TreeItems[parent->BoneIndex]->Children.Insert(item, 0);
	TreeItems.Add(item);
}

void SMassMuscleHierarchy::OnAddMuscle(FMassMuscleDataMuscle NewMuscle)
{
	AddMuscleToHierarchy(NewMuscle);
	TreeView->RequestTreeRefresh();
	Model->SetSelected(NewMuscle.Name, EItemType::Muscle);
}


void SMassMuscleHierarchy::OnDeleteMuscle(FMassMuscleDataMuscle MuscleToDelete)
{
	const TSharedPtr<FMassMuscleTreeItem> parent = FindItemByName(MuscleToDelete.BoneName);
	if (!parent)
		return;
	const TSharedPtr<FMassMuscleTreeItem> item = FindItemByName(MuscleToDelete.Name);
	if (!item)
		return;
	TreeItems.Remove(item);
	TreeItems[parent->BoneIndex]->Children.Remove(item);
	TreeView->ClearSelection();
	TreeView->RequestTreeRefresh();
}

void SMassMuscleHierarchy::OnItemNameChanged(FName Muscle, FName NewName)
{
    int32 index = TreeItems.IndexOfByPredicate(
        [&](const TSharedPtr<FMassMuscleTreeItem> I){ return I->Name == Muscle; });
	if(index == INDEX_NONE) {
		return;
	}
	TreeItems[index]->Name = NewName;
}

TArray<TSharedPtr<FMassMuscleTreeItem>> SMassMuscleHierarchy::GetSelectedItems() const
{
	TArray<TSharedPtr<FMassMuscleTreeItem>> SelectedItems;
	if (TreeView.IsValid())
	{
		TreeView->GetSelectedItems(SelectedItems);
	}
	return SelectedItems;
}