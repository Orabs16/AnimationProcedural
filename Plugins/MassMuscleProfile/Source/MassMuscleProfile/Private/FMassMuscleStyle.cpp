#include "FMassMuscleStyle.h"

#include "Styling/SlateStyleRegistry.h"
#include "Styling/SlateStyle.h"

#include "Brushes/SlateRoundedBoxBrush.h"
#include "Brushes/SlateColorBrush.h"

TSharedPtr<FSlateStyleSet> FMassMuscleStyle::StyleInstance = nullptr;

void FMassMuscleStyle::Initialize()
{
    if (!StyleInstance.IsValid())
    {
        StyleInstance = Create();
        FSlateStyleRegistry::RegisterSlateStyle(*StyleInstance);
    }
}

void FMassMuscleStyle::Shutdown()
{
    if (StyleInstance.IsValid())
    {
        FSlateStyleRegistry::UnRegisterSlateStyle(*StyleInstance);
        StyleInstance.Reset();
    }
}

FName FMassMuscleStyle::GetStyleSetName()
{
    static FName StyleSetName(TEXT("MassMuscleProfileStyle"));
    return StyleSetName;
}

const ISlateStyle& FMassMuscleStyle::Get()
{
    return *StyleInstance;
}

TSharedRef<FSlateStyleSet> FMassMuscleStyle::Create()
{
    const float rond = 8.f;
    TSharedRef<FSlateStyleSet> Style = MakeShared<FSlateStyleSet>(GetStyleSetName());

    Style->Set("MassMuscle.Tab.Active", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealBackGround,
        FVector4(rond, rond, 0.f, 0.f)));

    Style->Set("MassMuscle.Tab.Inactive", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealDarkBackGround,
        FVector4(rond, rond, 0.f, 0.f)));

    Style->Set("MassMuscle.BackGround", new FSlateColorBrush(
        FMassMuscleStyle::UnrealBackGround));

    Style->Set("MassMuscle.DarkBackGround", new FSlateColorBrush(
        FMassMuscleStyle::UnrealDarkBackGround));
        
    Style->Set("MassMuscle.LightBackGround", new FSlateColorBrush(
        FMassMuscleStyle::UnrealLightBackGround));

    Style->Set("MassMuscle.HighLight", new FSlateColorBrush(
        FMassMuscleStyle::UnrealHighLight));

    Style->Set("MassMuscle.Tab.TextActive", FLinearColor::White);
    Style->Set("MassMuscle.Tab.TextInactive", FLinearColor(0.5f, 0.5f, 0.5f));

    
    Style->Set("LeftDarkRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealBackGround,
        FVector4(rond, 0, 0,rond)));
    Style->Set("LeftRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealLightBackGround,
        FVector4(rond, 0, 0,rond)));
    Style->Set("LeftLightRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealButtonBackGround,
        FVector4(rond, 0, 0,rond)));
    Style->Set("RightDarkRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealBackGround,
        FVector4(0,rond ,rond, 0)));
    Style->Set("RightRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealLightBackGround,
        FVector4(0,rond, rond, 0)));
    Style->Set("RightLightRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealButtonBackGround,
        FVector4(0,rond, rond, 0)));

        
    Style->Set("DarkRoundBrush", new FSlateRoundedBoxBrush(
        FMassMuscleStyle::UnrealDarkBackGround,
        FVector4(rond, rond, rond,rond)));

    return Style;
}