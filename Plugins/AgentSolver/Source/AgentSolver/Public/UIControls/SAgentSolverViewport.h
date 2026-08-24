#pragma once

// Editor-only embedded viewport for SAgentSolverControlPanel's middle pane.
// Mirrors MassMuscleProfile's SMassMuscleViewport (SEditorViewport +
// FAdvancedPreviewScene + a custom FEditorViewportClient) almost exactly --
// see FAgentSolverViewportClient.h for what actually gets drawn.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "SEditorViewport.h"

class FAdvancedPreviewScene;
class FAgentSolverViewportClient;
class UAgentSolverViewportSettings;

class SAgentSolverViewport : public SEditorViewport
{
public:
	SLATE_BEGIN_ARGS(SAgentSolverViewport) {}
		SLATE_ARGUMENT(UAgentSolverViewportSettings*, Settings)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

protected:
	virtual TSharedRef<FEditorViewportClient> MakeEditorViewportClient() override;

private:
	TSharedPtr<FAdvancedPreviewScene> PreviewScene;
	TSharedPtr<FAgentSolverViewportClient> ViewportClient;
	UAgentSolverViewportSettings* Settings = nullptr;
};

#endif // WITH_EDITOR
