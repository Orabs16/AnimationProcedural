#include "SMassMuscleSettingsPannel.h"
#include "Engine/DataAsset.h"
#include "PropertyCustomizationHelpers.h"
#include "ThumbnailRendering/ThumbnailManager.h"
#include "UMassMuscleProfileAsset.h"
#include "Widgets/Input/SSearchBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "SMassMuscleExpandableSection.h"
#include "SMassMuscleItemRow.h"
#include "FMassMuscleEditorModel.h"

void SMassMuscleSettingsPannel::Construct(const FArguments &InArgs)
{
    Model = InArgs._Model;

    ChildSlot[
        SNew(SMassMuscleExpandableSection)
        .Title(FText::FromString("Data"))
        .InitiallyExpanded(true)
        +SMassMuscleExpandableSection::Slot()
        [
            SNew(SMassMuscleItemRow)
            +SMassMuscleItemRow::Slot()
            [
                 SNew(STextBlock).Text(FText::FromString("Skeletal Mesh"))
            ]
            +SMassMuscleItemRow::Slot()
            [
                SNew(SObjectPropertyEntryBox)
                .AllowedClass(USkeletalMesh::StaticClass())
                .ObjectPath(this, &SMassMuscleSettingsPannel::GetMeshPath)
                .OnObjectChanged(this, &SMassMuscleSettingsPannel::OnMeshChanged)
                .AllowClear(true)
                .DisplayThumbnail(true) // ← the small preview image
                .ThumbnailPool(UThumbnailManager::Get().GetSharedThumbnailPool())
            ]
        ]
            
        +SMassMuscleExpandableSection::Slot()
        [
            SNew(SMassMuscleItemRow)
            +SMassMuscleItemRow::Slot()
            [
                 SNew(STextBlock).Text(FText::FromString("Muscle Data"))
            ]
            +SMassMuscleItemRow::Slot()
            [
                SNew(SObjectPropertyEntryBox)
                .AllowedClass(UMassMuscleProfileAssetMuscle::StaticClass())
                .ObjectPath(this, &SMassMuscleSettingsPannel::GetMuscelDataPath)
                .OnObjectChanged(this, &SMassMuscleSettingsPannel::OnMuscleDataChanged)
                .AllowClear(true)
                .DisplayThumbnail(true) // ← the small preview image
                .ThumbnailPool(UThumbnailManager::Get().GetSharedThumbnailPool())
                .OnShouldFilterAsset_Lambda([this](const FAssetData& AssetData) -> bool
                {
                    UMassMuscleProfileAssetMuscle* profile = Cast<UMassMuscleProfileAssetMuscle>(AssetData.GetAsset());
                    if (!profile || !profile->SkeletalMesh) return false;
                    return profile->SkeletalMesh->GetPathName() != (Model->GetSkeletalMesh() ? Model->GetSkeletalMesh()->GetPathName() : FString());
                })
            ]
        ]
        +SMassMuscleExpandableSection::Slot()
        [
            SNew(SMassMuscleItemRow)
            +SMassMuscleItemRow::Slot()
            [
                 SNew(STextBlock).Text(FText::FromString("Mass Data"))
            ]
            +SMassMuscleItemRow::Slot()
            [
                SNew(SObjectPropertyEntryBox)
                .AllowedClass(UMassMuscleProfileAssetMass::StaticClass())
                .ObjectPath(this, &SMassMuscleSettingsPannel::GetMassDataPath)
                .OnObjectChanged(this, &SMassMuscleSettingsPannel::OnMassDataChanged)
                .AllowClear(true)
                .DisplayThumbnail(true) // ← the small preview image
                .ThumbnailPool(UThumbnailManager::Get().GetSharedThumbnailPool())
                .OnShouldFilterAsset_Lambda([this](const FAssetData& AssetData) -> bool
                {
                    UMassMuscleProfileAssetMass* profile = Cast<UMassMuscleProfileAssetMass>(AssetData.GetAsset());
                    if (!profile || !profile->SkeletalMesh) return false;
                    return profile->SkeletalMesh->GetPathName() != (Model->GetSkeletalMesh() ? Model->GetSkeletalMesh()->GetPathName() : FString());
                })
            ]
        ]
    ];
}
void SMassMuscleSettingsPannel::OnMeshChanged(const FAssetData &AssetData)
{
    USkeletalMesh *SelectedMesh = Cast<USkeletalMesh>(AssetData.GetAsset());
    if (Model.IsValid())
    {
        Model->SetSkeletalMesh(SelectedMesh);
    }
}

// Feeds the current path back into the widget so it stays in sync
FString SMassMuscleSettingsPannel::GetMeshPath() const
{
    return Model->GetSkeletalMesh() ? Model->GetSkeletalMesh()->GetPathName() : FString();
}

void SMassMuscleSettingsPannel::OnMuscleDataChanged(const FAssetData &AssetData)
{
    UMassMuscleProfileAssetMuscle *muscleData = Cast<UMassMuscleProfileAssetMuscle>(AssetData.GetAsset());
    if (Model.IsValid())
    {
        Model->SetMuscleProfile(muscleData);
    }
}
void SMassMuscleSettingsPannel::OnMassDataChanged(const FAssetData &AssetData)
{
    UMassMuscleProfileAssetMass *massData = Cast<UMassMuscleProfileAssetMass>(AssetData.GetAsset());
    if (Model.IsValid())
    {
        Model->SetMassProfile(massData);
    }
}

// Feeds the current path back into the widget so it stays in sync
FString SMassMuscleSettingsPannel::GetMuscelDataPath() const
{
    return Model->GetMuscleProfile() ? Model->GetMuscleProfile()->GetPathName() : FString();
}
FString SMassMuscleSettingsPannel::GetMassDataPath() const
{
    return Model->GetMassProfile() ? Model->GetMassProfile()->GetPathName() : FString();
}
