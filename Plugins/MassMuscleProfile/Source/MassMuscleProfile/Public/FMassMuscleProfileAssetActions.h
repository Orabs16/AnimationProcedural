#pragma once
#include "AssetTypeActions_Base.h"
#include "UMassMuscleProfileAsset.h"
#include "FMassMuscleProfile.h"

class FMassMuscleProfileAssetActionsMuscle : public FAssetTypeActions_Base
{
public:
	virtual FText GetName() const override
	{
		return FText::FromString("Muscle Profile");
	}


	virtual FColor GetTypeColor() const override
	{
		return FColor(220, 80, 60);
	} // your brand colour

	virtual UClass* GetSupportedClass() const override
	{
		return UMassMuscleProfileAssetMuscle::StaticClass();
	}


	virtual uint32 GetCategories() override
	{
		return EAssetTypeCategories::Animation;
	}

	virtual void OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<IToolkitHost> EditWithinLevelEditor = TSharedPtr<IToolkitHost>()) override
	{
		for (UObject* Obj : InObjects)
		{
			FMassMuscleProfileModule::Get().OpenToolForAsset(Obj);
		}
	}
};


class FMassMuscleProfileAssetActionsMass : public FAssetTypeActions_Base
{
public:
	virtual FText GetName() const override
	{
		return FText::FromString("Mass Profile");
	}


	virtual FColor GetTypeColor() const override
	{
		return FColor(220, 80, 60);
	} // your brand colour

	virtual UClass* GetSupportedClass() const override
	{
		return UMassMuscleProfileAssetMass::StaticClass();
	}


	virtual uint32 GetCategories() override
	{
		return EAssetTypeCategories::Animation;
	}
	virtual void OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<IToolkitHost> EditWithinLevelEditor = TSharedPtr<IToolkitHost>()) override
	{
		for (UObject* Obj : InObjects)
		{
		    FMassMuscleProfileModule::Get().OpenToolForAsset(Obj);
		}
	}
};