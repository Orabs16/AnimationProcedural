#include "UIControls/SAgentSolverViewport.h"

#if WITH_EDITOR

#include "UIControls/FAgentSolverViewportClient.h"
#include "AdvancedPreviewScene.h"

void SAgentSolverViewport::Construct(const FArguments& InArgs)
{
	Settings = InArgs._Settings;

	FPreviewScene::ConstructionValues CVS;
	CVS.bCreatePhysicsScene = false;
	CVS.bDefaultLighting = true;

	PreviewScene = MakeShareable(new FAdvancedPreviewScene(CVS));
	PreviewScene->SetProfileIndex(0);

	SEditorViewport::Construct(SEditorViewport::FArguments());
}

TSharedRef<FEditorViewportClient> SAgentSolverViewport::MakeEditorViewportClient()
{
	ViewportClient = MakeShared<FAgentSolverViewportClient>(PreviewScene.Get(), Settings);
	return ViewportClient.ToSharedRef();
}

#endif // WITH_EDITOR
