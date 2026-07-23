#pragma once

#include "CoreMinimal.h"
#include "SEditorViewport.h"
#include "AdvancedPreviewScene.h"
#include "FMassMuscleEditorModel.h"
#include "FMassMuscleEditorSettings.h"

class FPreviewScene;
class FMassMuscleViewportClient;

class SMassMuscleViewport : public SEditorViewport
{
public:

    SLATE_BEGIN_ARGS(SMassMuscleViewport) {}
        SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
        SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorSettings>, Settings)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

    FPreviewScene* GetPreviewScene() const { return MyAdvancedPreviewScene.Get(); }

    TSharedPtr<FMassMuscleViewportClient> GetViewportClient() const;

protected:

    virtual TSharedRef<FEditorViewportClient>
    MakeEditorViewportClient() override;

private:

    TSharedPtr<FAdvancedPreviewScene> MyAdvancedPreviewScene;
    TSharedPtr<FMassMuscleViewportClient> ViewportClient;
    TSharedPtr<FMassMuscleEditorModel> Model;
    TSharedPtr<FMassMuscleEditorSettings> Settings;
};