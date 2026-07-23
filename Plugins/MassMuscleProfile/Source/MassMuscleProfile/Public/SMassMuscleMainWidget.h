#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "SMassMuscleViewport.h"
#include "FMassMuscleEditorModel.h"
#include "SMassMuscleHierarchy.h"
#include "SMassMuscleButtonPannel.h"
#include "SMassMuscleDetailsPannel.h"
#include "SMassMuscleSettingsPannel.h"
#include "FMassMuscleEditorSettings.h"

class SMassMuscleMainWidget : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SMassMuscleMainWidget) {}
    SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
    SLATE_END_ARGS()

    void Construct(const FArguments &InArgs);


private:
    TSharedPtr<class SMassMuscleViewport> ViewportWidget;
    TSharedPtr<class SMassMuscleHierarchy> HierarchyWidget;
    TSharedPtr<class SMassMuscleButtonPanel> ButtonPanelWidget;
    TSharedPtr<class SMassMuscleDetailsPannel> DetailsPanelWidget;
    TSharedPtr<class SMassMuscleSettingsPannel> SettingsPanelWidget;

    TObjectPtr<USkeletalMesh> SelectedMesh = nullptr;
    TSharedPtr<FMassMuscleEditorModel> Model;
    TSharedPtr<FMassMuscleEditorSettings> Settings;

    FReply OnTabClicked(int32 TabIndex);
    TSharedPtr<SWidgetSwitcher> RightPanelSwitcher;
    int32 ActiveTabIndex = 0;
};