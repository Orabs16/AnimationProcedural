// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"
#include "UMassMuscleProfileAsset.h"
#include "FMassMuscleData.h"
#include "FMassMuscleEditorModel.h"

class SDockTab;
class FMassMuscleProfileAssetActionsMuscle;
class FMassMuscleProfileAssetActionsMass;

class FMassMuscleProfileModule : public IModuleInterface
{
public:
  virtual void StartupModule() override;
  virtual void ShutdownModule() override;

  TSharedRef<SDockTab> SpawnPluginTab(const class FSpawnTabArgs& SpawnTabArgs);
  TSharedPtr<FMassMuscleProfileAssetActionsMuscle> AssetActionsMuscle;
  TSharedPtr<FMassMuscleProfileAssetActionsMass> AssetActionsMass;
  void RegisterMenus();

  static FMassMuscleProfileModule& Get();
  void OpenToolForAsset(UObject* Asset);

private:
   void RefreshModelForAsset(UObject* Asset);
   TSharedPtr<FMassMuscleEditorModel> ActiveModel;
   UObject* PendingAssetToOpen = nullptr;
};