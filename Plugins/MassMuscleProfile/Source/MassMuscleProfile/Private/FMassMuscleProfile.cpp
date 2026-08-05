#include "FMassMuscleProfile.h"

#include "BlueprintActionDatabase.h"
#include "FMassMuscleProfileAssetActions.h"
#include "SMassMuscleMainWidget.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "UMassMuscleProfileAsset.h"
#include "UObject/UObjectGlobals.h"
#include "Widgets/Docking/SDockTab.h"
#include "LevelEditor.h"
#include "FMassMuscleEditorModel.h"
#include "FMassMuscleStyle.h"
#include "ToolMenus.h"

#include "Framework/Application/SlateApplication.h"
#define LOCTEXT_NAMESPACE "FMassMuscleProfileModule"

static const FName MassMuscleTabName("MassMuscleProfile");

void FMassMuscleProfileModule::StartupModule()
{
    FGlobalTabmanager::Get()
        ->RegisterNomadTabSpawner(MassMuscleTabName,
                                  FOnSpawnTab::CreateRaw(this, &FMassMuscleProfileModule::SpawnPluginTab))
        .SetDisplayName(LOCTEXT("MassMuscleTabTitle", "Mass Muscle Profile"))
        .SetMenuType(ETabSpawnerMenuType::Hidden);
    UToolMenus::RegisterStartupCallback(
        FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FMassMuscleProfileModule::RegisterMenus));

    IAssetTools &AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

    AssetActionsMuscle = MakeShared<FMassMuscleProfileAssetActionsMuscle>();
    AssetTools.RegisterAssetTypeActions(AssetActionsMuscle.ToSharedRef());
    AssetActionsMass = MakeShared<FMassMuscleProfileAssetActionsMass>();
    AssetTools.RegisterAssetTypeActions(AssetActionsMass.ToSharedRef());

    FMassMuscleStyle::Initialize();

    if (FBlueprintActionDatabase* ActionDatabase = FBlueprintActionDatabase::TryGet())
    {
        if (UClass* ControlRigBlueprintClass = FindObject<UClass>(nullptr, TEXT("/Script/ControlRigDeveloper.ControlRigBlueprint")))
        {
            ActionDatabase->RefreshClassActions(ControlRigBlueprintClass);
        }
    }
}

void FMassMuscleProfileModule::ShutdownModule()
{
    FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(MassMuscleTabName);
    if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
    {
        IAssetTools& AssetTools =
            FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();

        AssetTools.UnregisterAssetTypeActions(AssetActionsMuscle.ToSharedRef());
        AssetTools.UnregisterAssetTypeActions(AssetActionsMass.ToSharedRef());
    }

    FMassMuscleStyle::Shutdown();
}

TSharedRef<SDockTab> FMassMuscleProfileModule::SpawnPluginTab(const FSpawnTabArgs &SpawnTabArgs)
{
    ActiveModel = MakeShared<FMassMuscleEditorModel>();
    if (PendingAssetToOpen)
    {
        RefreshModelForAsset(PendingAssetToOpen);
        PendingAssetToOpen = nullptr;
    }
    else
    {
        // your existing "grab first asset in registry" fallback logic stays here unchanged
        FAssetRegistryModule& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
        TArray<FAssetData> Assets;
        AssetRegistry.Get().GetAssetsByClass(UMassMuscleProfileAssetMuscle::StaticClass()->GetClassPathName(), Assets);
        if (Assets.Num() < 1)
        {
            ActiveModel->SetSkeletalMesh(LoadObject<USkeletalMesh>(nullptr, TEXT("/Engine/EngineMeshes/SkeletalCube.SkeletalCube")));
        }
        else
        {
            RefreshModelForAsset(Assets[0].GetAsset());
        }
    }
    return SNew(SDockTab)[SNew(SMassMuscleMainWidget).Model(ActiveModel)];
}

void FMassMuscleProfileModule::RegisterMenus()
{
    FToolMenuOwnerScoped OwnerScoped(this);

    UToolMenu *Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Window");

    FToolMenuSection &Section = Menu->FindOrAddSection("WindowLayout");

    Section.AddMenuEntry(NAME_None, LOCTEXT("MassMuscleProfile", "Mass Muscle Profile"),
                         LOCTEXT("MassMuscleProfileTooltip", "Open Mass Muscle Editor"), FSlateIcon(),
                         FToolUIActionChoice(FExecuteAction::CreateLambda(
                             []() { FGlobalTabmanager::Get()->TryInvokeTab(FName("MassMuscleProfile")); })));
}

FMassMuscleProfileModule& FMassMuscleProfileModule::Get()
{
    return FModuleManager::LoadModuleChecked<FMassMuscleProfileModule>("MassMuscleProfile");
}

void FMassMuscleProfileModule::OpenToolForAsset(UObject* Asset)
{
    PendingAssetToOpen = Asset;

    // If the tab doesn't exist yet, this calls SpawnPluginTab, which will
    // consume PendingAssetToOpen. If it already exists, this just focuses it.
    FGlobalTabmanager::Get()->TryInvokeTab(MassMuscleTabName);

    // Tab already existed -> SpawnPluginTab didn't run -> update the live model ourselves.
    if (ActiveModel.IsValid() && PendingAssetToOpen)
    {
        RefreshModelForAsset(PendingAssetToOpen);
        PendingAssetToOpen = nullptr;
    }
}

void FMassMuscleProfileModule::RefreshModelForAsset(UObject* Asset)
{
    if (!ActiveModel.IsValid())
    {
        return;
    }

    FAssetRegistryModule& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

    if (UMassMuscleProfileAssetMuscle* MuscleProfile = Cast<UMassMuscleProfileAssetMuscle>(Asset))
    {
        ActiveModel->SetMuscleProfile(MuscleProfile);
        ActiveModel->SetSkeletalMesh(MuscleProfile->SkeletalMesh);

        TArray<FAssetData> MassAssets;
        AssetRegistry.Get().GetAssetsByClass(UMassMuscleProfileAssetMass::StaticClass()->GetClassPathName(), MassAssets);
        FAssetData* Found = MassAssets.FindByPredicate([MuscleProfile](const FAssetData& Data)
        {
            UMassMuscleProfileAssetMass* MassProfile = Cast<UMassMuscleProfileAssetMass>(Data.GetAsset());
            return MassProfile && MassProfile->SkeletalMesh == MuscleProfile->SkeletalMesh;
        });
        ActiveModel->SetMassProfile(Found ? Cast<UMassMuscleProfileAssetMass>(Found->GetAsset()) : nullptr);
    }
    else if (UMassMuscleProfileAssetMass* MassProfile = Cast<UMassMuscleProfileAssetMass>(Asset))
    {
        ActiveModel->SetMassProfile(MassProfile);
        ActiveModel->SetSkeletalMesh(MassProfile->SkeletalMesh);

        TArray<FAssetData> MuscleAssets;
        AssetRegistry.Get().GetAssetsByClass(UMassMuscleProfileAssetMuscle::StaticClass()->GetClassPathName(), MuscleAssets);
        FAssetData* Found = MuscleAssets.FindByPredicate([MassProfile](const FAssetData& Data)
        {
            UMassMuscleProfileAssetMuscle* MuscleProfile = Cast<UMassMuscleProfileAssetMuscle>(Data.GetAsset());
            return MuscleProfile && MuscleProfile->SkeletalMesh == MassProfile->SkeletalMesh;
        });
        ActiveModel->SetMuscleProfile(Found ? Cast<UMassMuscleProfileAssetMuscle>(Found->GetAsset()) : nullptr);
    }
}


#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FMassMuscleProfileModule, MassMuscleProfile)