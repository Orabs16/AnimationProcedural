#pragma once

#include "Widgets/SCompoundWidget.h"
#include "FMassMuscleEditorSettings.h"
#include "FMassMuscleEditorModel.h"
#include "SMassMuscleHierarchy.h"

class SMassMuscleButtonPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMassMuscleButtonPanel){}
		SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
		SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorSettings>, Settings)
		SLATE_ARGUMENT(TSharedPtr<SMassMuscleHierarchy>, HierarchyWidget)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	
    TSharedPtr<class SButton> extend;
    TSharedPtr<class SButton> collapse;

private:
	FReply OnXRaySkeleton();
	FReply OnXRayMuscles();
	FReply OnXRayCapsules();
	FReply OnCollapse();
	FReply OnExtend();
	FReply OnNewMuscle();
	FReply OnSaveClicked();
	FReply OnMirrorMuscles();
	FReply OnMirrorCapsules();
	FReply OnDeleteMuscles();
	bool IsHierarchyCollapsed = false;
    TSharedPtr<FMassMuscleEditorSettings> Settings;
    TSharedPtr<FMassMuscleEditorModel> Model;
	TSharedPtr<SMassMuscleHierarchy> HierarchyWidget;
	
};
