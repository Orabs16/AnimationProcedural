#pragma once

#include "CoreMinimal.h"
#include "FMassMuscleTreeItem.h"
#include "FMassMuscleEditorModel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STableViewBase.h"
#include "Widgets/Views/STreeView.h"
#include "FMassMuscleEditorSettings.h"

class SMassMuscleHierarchy : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SMassMuscleHierarchy) {}
    SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
    SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorSettings>, Settings)
    SLATE_END_ARGS()

    void Construct(const FArguments &InArgs);

    void SetSkeletalMesh(USkeletalMesh *InMesh);

    void OnItemNameChanged(FName Muscle, FName NewName);
    TArray<TSharedPtr<FMassMuscleTreeItem>> GetSelectedItems() const;
    void OnAddMuscle(FMassMuscleDataMuscle Muscle);
    void OnDeleteMuscle(FMassMuscleDataMuscle Muscle);
private:
    void BuildTree();
// NOLINTNEXTLINE
    TSharedRef<ITableRow> OnGenerateRow(TSharedPtr<FMassMuscleTreeItem> Item, const TSharedRef<STableViewBase> &OwnerTable); 

    void OnGetChildren(TSharedPtr<FMassMuscleTreeItem> Item, TArray<TSharedPtr<FMassMuscleTreeItem>> &OutChildren);

    void OnSelectionChanged(TSharedPtr<FMassMuscleTreeItem> Item, ESelectInfo::Type SelectInfo);

    void OnMeshChanged(USkeletalMesh *NewMesh);

    void SetItemExpansionRecursive(TSharedPtr<FMassMuscleTreeItem> Item, bool bExpand);

    void OnExteriorSelectionChanged(FName NewSelectionName, EItemType Type);

    void ExtendHierarchy();
    void CollapseHierarchy();
    
    const TSharedPtr<FMassMuscleTreeItem>& FindItemByName(FName ToFind);

    void AddMuscleToHierarchy(FMassMuscleDataMuscle NewMuscle);


private:
    USkeletalMesh *SkeletalMesh = nullptr;

    TArray<TSharedPtr<FMassMuscleTreeItem>> RootItems;
    
    TArray<TSharedPtr<FMassMuscleTreeItem>> TreeItems;

    TSharedPtr<STreeView<TSharedPtr<FMassMuscleTreeItem>>> TreeView;

    TSharedPtr<FMassMuscleEditorModel> Model;
    TSharedPtr<FMassMuscleEditorSettings> Settings;
};