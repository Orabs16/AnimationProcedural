#pragma once
#include "CoreMinimal.h"
#include "Styling/SlateStyle.h"

class FMassMuscleStyle
{
public:
	static void Initialize();
	static void Shutdown();
	static const ISlateStyle& Get();
	static FName GetStyleSetName();

	inline static const FLinearColor UnrealDarkBackGround               = FLinearColor(FColor(21,21,21,255));
	inline static const FLinearColor UnrealBackGround            		= FLinearColor(FColor(36,36,36,255));
	inline static const FLinearColor UnrealLightBackGround              = FLinearColor(FColor(47,47,47,255));
	inline static const FLinearColor UnrealButtonBackGround             	= FLinearColor(FColor(56,56,56,255));
	inline static const FLinearColor UnrealSmallHighLight             	= FLinearColor(FColor(87,87,87,255));
	inline static const FLinearColor UnrealHighLight 					= FLinearColor(FColor(134,134,134,255));
	inline static const FLinearColor UnrealText                         = FLinearColor(FColor(255,255,255,255));
	inline static const FLinearColor MuscleText                         = FLinearColor(FColor(255,130,130,255));
	inline static const FLinearColor MuscleColor                        = FLinearColor(FColor(255,130,130,120));
	inline static const FLinearColor BoneColor							= FLinearColor(FColor(105, 181, 205, 120));
	inline static const FLinearColor MassCapsuleColor						= FLinearColor(FColor(120, 220, 140, 160));


private:
	static TSharedRef<class FSlateStyleSet> Create();
	static TSharedPtr<class FSlateStyleSet> StyleInstance;
};
