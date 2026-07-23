#include "SMassMuscleViewport.h"

#include "PreviewScene.h"
#include "AdvancedPreviewScene.h"
#include "FMassMuscleViewportClient.h"
#include "FMassMuscleEditorModel.h"
#include "Widgets/Input/IVirtualKeyboardEntry.h"

void SMassMuscleViewport::Construct(const FArguments& InArgs)
{
    Model = InArgs._Model;
    Settings = InArgs._Settings;
    
    FPreviewScene::ConstructionValues CVS;
    CVS.bCreatePhysicsScene = false; // Set to true if you need physics/collisions
    CVS.bDefaultLighting = true;     // This enables the native default lighting setup

    // 2. Instantiate the scene
    MyAdvancedPreviewScene = MakeShareable(new FAdvancedPreviewScene(CVS));

    // 3. Optional: Map it to the standard Asset Viewer settings 
    // This lets your tool share the same background/lighting profiles as the rest of UE
    MyAdvancedPreviewScene->SetProfileIndex(0);
    SEditorViewport::Construct(SEditorViewport::FArguments());
}

TSharedRef<FEditorViewportClient>
SMassMuscleViewport::MakeEditorViewportClient()
{
    ViewportClient = MakeShared<FMassMuscleViewportClient>(MyAdvancedPreviewScene.Get(), Model, Settings);
    return ViewportClient.ToSharedRef();
}
TSharedPtr<FMassMuscleViewportClient>
SMassMuscleViewport::GetViewportClient() const
{
    return ViewportClient;
}
