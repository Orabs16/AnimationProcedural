#include "LearningProgramAssetTypeActions.h"
#include "LearningProgramAssetEditorToolkit.h"

void FLearningProgramAssetTypeActions::OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<IToolkitHost> EditWithinLevelEditor)
{
	const EToolkitMode::Type Mode = EditWithinLevelEditor.IsValid() ? EToolkitMode::WorldCentric : EToolkitMode::Standalone;

	for (UObject* Obj : InObjects)
	{
		if (ULearningProgram* Program = Cast<ULearningProgram>(Obj))
		{
			TSharedRef<FLearningProgramAssetEditorToolkit> Toolkit = MakeShared<FLearningProgramAssetEditorToolkit>();
			Toolkit->InitLearningProgramEditor(Mode, EditWithinLevelEditor, Program);
		}
	}
}
