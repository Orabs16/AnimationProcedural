#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/SPanel.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "FMassMuscleEditorModel.h"

class SMassMuscleSettingsPannel : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SMassMuscleSettingsPannel) {}
        SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
    SLATE_END_ARGS()

    void Construct(const FArguments &InArgs);

private:
    FString GetMeshPath() const;
    void OnMeshChanged(const FAssetData &AssetData);
    FString GetMuscelDataPath() const;
    void OnMuscleDataChanged(const FAssetData &AssetData);
    FString GetMassDataPath() const;
    void OnMassDataChanged(const FAssetData &AssetData);

    TSharedPtr<FMassMuscleEditorModel> Model;

    bool DataIsExpanded = true;
};
