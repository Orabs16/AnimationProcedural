#include "UIControls/SAgentSolverControlPanel.h"

#if WITH_EDITOR

#include "UIControls/AgentSolverUIUtils.h"
#include "UIControls/SAgentSolverViewport.h"
#include "UIControls/SAgentSolverLineGraph.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
#include "Components/PoseableMeshComponent.h"
#include "UIControls/LearningAgentsNeuralNetworkFactory.h"
#include "LearningAgentsNeuralNetwork.h"

#include "DetailsViewArgs.h"
#include "IDetailsView.h"
#include "IDetailPropertyRow.h"
#include "IDetailCustomization.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "PropertyCustomizationHelpers.h"
#include "PropertyEditorModule.h"
#include "PropertyEditorDelegates.h"
#include "Modules/ModuleManager.h"
#include "Editor.h"
#include "PlayInEditorDataTypes.h"
#include "LevelEditor.h"
#include "IAssetViewport.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/AppStyle.h"

#define LOCTEXT_NAMESPACE "SAgentSolverControlPanel"

namespace
{
	// "Muto RL|Rig"/"Muto RL|Reward"/"Muto RL|Reset"/"Muto RL|Training"/
	// "Muto RL|Learning" -> Agent tab. "Muto RL|Simulation"/"Muto RL|Contact"
	// (|*)/"Muto RL|Diagnostics" -> Physics tab. Prefix match on purpose: e.g.
	// "Muto RL|Contact|Joint Limits" must land under the bare "Muto RL|Contact"
	// allowance. "Muto RL|Learning" (the 6 Load*/Save*NetworkAsset properties)
	// deliberately NOT built as bespoke Slate rows -- they show through the
	// Agent tab's details view like everything else, since IDetailsView
	// already renders object-reference properties as asset pickers for free;
	// building bespoke rows on top just duplicated them.
	const TArray<FString> AgentParameterCategoryPrefixes = { TEXT("Muto RL|Rig"), TEXT("Muto RL|Reward"), TEXT("Muto RL|Reset"), TEXT("Muto RL|Training"), TEXT("Muto RL|Learning") };
	const TArray<FString> PhysicsParameterCategoryPrefixes = { TEXT("Muto RL|Simulation"), TEXT("Muto RL|Contact"), TEXT("Muto RL|Diagnostics") };
	// Deliberately "Muto RL|Tuning", not "Muto RL|Reward" -- see
	// AMutoRLTrainingDriver::RewardHeightTarget's comment: nesting under
	// "Muto RL|Reward|..." would ALSO match AgentParameterCategoryPrefixes'
	// "Muto RL|Reward" entry (StartsWith is a raw string prefix check, not
	// pipe-segment-aware), showing every field in both the Agent tab AND here.
	// "Muto RL|Simulation|Gravity" is Gravity specifically (see its own
	// comment) -- deliberately NOT the bare "Muto RL|Simulation", which would
	// also pull in bAutoStartOnBeginPlay/NumEnvs/etc. from the Physics tab.
	const TArray<FString> RewardSettingsCategoryPrefixes = { TEXT("Muto RL|Tuning"), TEXT("Muto RL|Simulation|Gravity") };

	// How often Tick() pushes a new sample into the reward/throughput graphs.
	// Sub-second would just make the graph noisy without adding information
	// (AverageReward is already a slow EMA; step-rate over <0.5s is mostly jitter).
	constexpr double GraphSampleIntervalSeconds = 0.5;

	// Local reproduction of MassMuscleProfile's dark palette (FMassMuscleStyle,
	// see the class comment) -- not reused directly, since none of that
	// module's Slate classes or FMassMuscleStyle itself carry a
	// MASSMUSCLEPROFILE_API export macro, so linking against them from this
	// module's own DLL would fail. Same literal color values, so the two
	// panels still look alike.
	const FLinearColor PanelBackgroundColor = FLinearColor(FColor(21, 21, 21, 255));
	const FLinearColor SectionHeaderColor = FLinearColor(FColor(200, 200, 200, 255));
	const FLinearColor TabActiveColor = FLinearColor(FColor(56, 56, 56, 255));
	const FLinearColor TabInactiveColor = PanelBackgroundColor;

	/**
	 * Registered on AgentParametersView/PhysicsParametersView/RewardSettingsView
	 * to hide the built-in engine categories these actors/components carry --
	 * these aren't reached by SetIsPropertyVisibleDelegate below at all
	 * (that's a per-FProperty filter; these category headers come from a
	 * different part of IDetailsView's default layout, same class of gap as
	 * the CallInEditor buttons discovered earlier -- see
	 * MutoRLTrainingDriver.h's history there). Unlike an empty-customization
	 * attempt (confirmed not to work, same history), HideCategory calls made
	 * HERE do work regardless of what else populated the layout: it's a
	 * direct suppression instruction the details panel honors from any
	 * registered customization, not a "replaces the default" mechanism.
	 *
	 * Registered against TWO different classes (see CreateFilteredDetailsView):
	 * "Actor" and "Physics" (a few actor-level physics-replication properties)
	 * come from AMutoRLTrainingDriver itself, but "Transform"/"Material"/
	 * "Skin Weights" come from the actor's ROOT COMPONENT -- both
	 * AMutoRagdollVisualizerActor and AMutoRLVisualizerActor set
	 * RootComponent = a UPoseableMeshComponent (see their constructors) --
	 * which IDetailsView lays out via a SEPARATE customization pass keyed off
	 * the COMPONENT's own class, not the actor's. A customization registered
	 * only for AMutoRLTrainingDriver never reaches that pass at all, which is
	 * exactly why an earlier attempt using just that one registration hid
	 * Actor/Physics but left Transform/Material/Skin Weights showing.
	 */
	class FAgentSolverCategoryFilterCustomization : public IDetailCustomization
	{
	public:
		static TSharedRef<IDetailCustomization> MakeInstance()
		{
			return MakeShared<FAgentSolverCategoryFilterCustomization>();
		}

		virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override
		{
			// HideCategory takes the CATEGORY'S INTERNAL FName, not its display
			// label -- confirmed by reading the engine customizations that
			// build each of these (Editor/DetailCustomizations/Private/):
			// SceneComponentDetails.cpp's FSceneComponentDetails registers
			// EditCategory("TransformCommon", displayed as "Transform");
			// SkinnedMeshComponentDetails.cpp's FSkinnedMeshComponentDetails
			// registers EditCategory("SkinWeights", displayed as "Skin
			// Weights") -- "Transform"/"Skin Weights" (the display text) never
			// matched either HideCategory call.
			DetailBuilder.HideCategory(TEXT("Material"));
			DetailBuilder.HideCategory(TEXT("Materials"));
			DetailBuilder.HideCategory(TEXT("TransformCommon"));
			DetailBuilder.HideCategory(TEXT("Physics"));
			DetailBuilder.HideCategory(TEXT("SkinWeights"));
			DetailBuilder.HideCategory(TEXT("Actor"));

			// Gives the Learning category's 6 Load*/Save*NetworkAsset rows a
			// "create new asset" (+) button, same as any other asset-reference
			// property that HAS a registered UFactory -- ULearningAgentsNeuralNetwork
			// had none at all (see LearningAgentsNeuralNetworkFactory.h's
			// comment), so the property's default SObjectPropertyEntryBox never
			// got one either. One shared factory instance for all 6 rows --
			// stateless, safe to reuse. GetProperty() on a property this object
			// doesn't have (e.g. when this same customization runs for a
			// UPoseableMeshComponent instead of an AMutoRLTrainingDriver, see
			// this class's header comment) returns an invalid handle;
			// EditDefaultProperty on that returns nullptr, guarded below.
			static TStrongObjectPtr<ULearningAgentsNeuralNetworkFactory> NeuralNetworkFactory(NewObject<ULearningAgentsNeuralNetworkFactory>());
			const TArray<FName> NetworkAssetPropertyNames = {
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, LoadEncoderNetworkAsset),
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, LoadPolicyNetworkAsset),
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, LoadDecoderNetworkAsset),
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, SaveEncoderNetworkAsset),
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, SavePolicyNetworkAsset),
				GET_MEMBER_NAME_CHECKED(AMutoRLTrainingDriver, SaveDecoderNetworkAsset),
			};
			for (const FName& PropertyName : NetworkAssetPropertyNames)
			{
				TSharedRef<IPropertyHandle> Handle = DetailBuilder.GetProperty(PropertyName, AMutoRLTrainingDriver::StaticClass());
				IDetailPropertyRow* Row = DetailBuilder.EditDefaultProperty(Handle);
				if (!Row)
				{
					continue;
				}
				Row->CustomWidget()
					.NameContent()
					[
						Handle->CreatePropertyNameWidget()
					]
					.ValueContent()
					[
						SNew(SObjectPropertyEntryBox)
						.PropertyHandle(Handle)
						.AllowedClass(ULearningAgentsNeuralNetwork::StaticClass())
						.NewAssetFactories(TArray<UFactory*>{ NeuralNetworkFactory.Get() })
						.AllowCreate(true)
						.AllowClear(true)
						.DisplayThumbnail(true)
					];
			}
		}
	};
}

FText SAgentSolverControlPanel::GetStatusText() const
{
	const AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	if (!AgentSolverUI::IsPIERunning())
	{
		return Driver
			? LOCTEXT("StatusNoPIE", "Driver found in the editor level. Press Start to launch PIE and begin training.")
			: LOCTEXT("StatusNoDriver", "No AMutoRLTrainingDriver found in the current level.");
	}
	if (!Driver)
	{
		return LOCTEXT("StatusNoDriverPIE", "PIE is running, but no AMutoRLTrainingDriver was found in it.");
	}
	if (!Driver->GetPolicy())
	{
		return LOCTEXT("StatusNotStarted", "Driver found. Training has not been started yet -- press Start.");
	}
	return Driver->IsTrainingActive()
		? LOCTEXT("StatusTraining", "Training is RUNNING.")
		: LOCTEXT("StatusStopped", "Driver found. Training is stopped.");
}

FText SAgentSolverControlPanel::GetStatsText() const
{
	AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	if (!Driver)
	{
		return FText::GetEmpty();
	}

	FNumberFormattingOptions RewardFormat;
	RewardFormat.SetMaximumFractionalDigits(3);

	return FText::Format(
		LOCTEXT("StatsFormat", "Envs: {0}\nTraining steps: {1}\nEpisodes completed: {2}\nAvg reward (EMA): {3}"),
		FText::AsNumber(Driver->NumEnvs),
		FText::AsNumber(Driver->GetTrainingStepCount()),
		FText::AsNumber(Driver->GetEpisodeCount()),
		FText::AsNumber(Driver->GetAverageReward(), &RewardFormat));
}

FReply SAgentSolverControlPanel::OnStartTrainingClicked()
{
	if (!AgentSolverUI::IsPIERunning())
	{
		// Matches the real editor Play button: launches a PIE session rather
		// than running StartTraining() directly (see AgentSolverUI::
		// IsPIERunning's comment for why that's unsafe against the editor-
		// world instance). AMutoRLTrainingDriver::BeginPlay then auto-calls
		// StartTraining() once PIE actually starts, via bAutoStartOnBeginPlay
		// (default true) -- nothing else is needed here for the common case.
		//
		// DestinationSlateViewport is set to the level editor's currently
		// active viewport so PIE plays there instead of opening a new
		// floating window. An earlier attempt at this forced
		// ULevelEditorPlaySettings::LastExecutedPlayModeType=PlayMode_InViewPort
		// via a throwaway settings object passed through
		// FRequestPlaySessionParams::EditorPlaySettings -- that setting isn't
		// actually consulted for the new-window-vs-viewport decision (see
		// UEditorEngine::StartPlayInEditorSession, PlayLevel.cpp), only
		// DestinationSlateViewport is (checked directly, PlayLevel.cpp
		// ~line 3081-3124: falls through to spawning a new window only when
		// this is unset). Training itself is headless (see
		// AMutoRLTrainingDriver's class comment -- nothing to look at in the
		// PIE world regardless; this panel's own embedded viewport is what
		// actually shows it), so a whole extra window for it is pure clutter.
		if (GEditor)
		{
			FRequestPlaySessionParams Params;
			FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
			if (TSharedPtr<IAssetViewport> ActiveViewport = LevelEditorModule.GetFirstActiveViewport())
			{
				Params.DestinationSlateViewport = ActiveViewport;
			}
			GEditor->RequestPlaySession(Params);
		}
		return FReply::Handled();
	}

	// Already in PIE: manual (re)start -- covers bAutoStartOnBeginPlay being
	// off, or clicking again after a StopTraining() call. StartTraining()
	// itself is idempotent (bStartTrainingCalled guard), so this is harmless
	// even if BeginPlay already started it.
	if (AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver())
	{
		Driver->StartTraining();
	}
	return FReply::Handled();
}

FReply SAgentSolverControlPanel::OnStopTrainingClicked()
{
	if (AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver())
	{
		Driver->StopTraining();
	}
	return FReply::Handled();
}

bool SAgentSolverControlPanel::IsStartTrainingEnabled() const
{
	// Not in PIE is still "enabled" -- clicking it launches PIE (see
	// OnStartTrainingClicked), it doesn't no-op. Only greyed out once
	// training is already actively running, matching Stop's own condition.
	const AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	return !Driver || !Driver->IsTrainingActive();
}

bool SAgentSolverControlPanel::IsStopTrainingEnabled() const
{
	const AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	return Driver && Driver->IsTrainingActive();
}

FReply SAgentSolverControlPanel::OnSaveToAssetsClicked()
{
	if (AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver())
	{
		Driver->SaveTrainedNetworksToAssets();
	}
	return FReply::Handled();
}

FReply SAgentSolverControlPanel::OnLoadFromAssetsClicked()
{
	if (AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver())
	{
		Driver->LoadTrainedNetworksFromAssets();
	}
	return FReply::Handled();
}

FReply SAgentSolverControlPanel::OnSelectParameterTab(int32 TabIndex)
{
	ActiveParameterTabIndex = TabIndex;
	if (ParameterSwitcher.IsValid())
	{
		ParameterSwitcher->SetActiveWidgetIndex(TabIndex);
	}
	return FReply::Handled();
}

TSharedRef<SWidget> SAgentSolverControlPanel::MakeTabButton(const FText& Label, TAttribute<bool> bIsActive, FOnClicked OnClicked)
{
	// Same visual structure as SMassMuscleMainWidget's Details/Settings tab
	// header (see class comment): a NoBorder button so only the inner SBorder
	// paints, tinted by whether this tab is the active one, wrapping a
	// fixed-height centered label.
	return SNew(SButton)
		.ButtonStyle(FAppStyle::Get(), "NoBorder")
		.ContentPadding(0.0f)
		.OnClicked(OnClicked)
		[
			SNew(SBorder)
			.BorderImage(FAppStyle::GetBrush("WhiteBrush"))
			.BorderBackgroundColor_Lambda([bIsActive]() { return bIsActive.Get() ? TabActiveColor : TabInactiveColor; })
			.Padding(FMargin(0.0f, 0.0f, 0.0f, 2.0f))
			[
				SNew(SBox)
				.HAlign(HAlign_Center)
				.VAlign(VAlign_Center)
				.HeightOverride(28.0f)
				[
					SNew(STextBlock).Text(Label)
				]
			]
		];
}

TSharedRef<IDetailsView> SAgentSolverControlPanel::CreateFilteredDetailsView(const TArray<FString>& AllowedCategoryPrefixes)
{
	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.bAllowSearch = true;
	DetailsViewArgs.bShowOptions = false;
	DetailsViewArgs.bShowObjectLabel = false;
	DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;

	TSharedRef<IDetailsView> View = PropertyEditorModule.CreateDetailView(DetailsViewArgs);

	// See FAgentSolverCategoryFilterCustomization's comment -- this is what
	// actually hides the built-in Transform/Physics/Material/Skin Weights/
	// Actor categories; the property-visible filter below never sees them.
	// Registered against BOTH the actor class (Actor/Physics) AND
	// UPoseableMeshComponent (Transform/Material/Skin Weights -- both
	// Ragdoll's and Visualizer's actual RootComponent class), since
	// IDetailsView lays those two out via separate customization passes.
	View->RegisterInstancedCustomPropertyLayout(AMutoRLTrainingDriver::StaticClass(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FAgentSolverCategoryFilterCustomization::MakeInstance));
	View->RegisterInstancedCustomPropertyLayout(UPoseableMeshComponent::StaticClass(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FAgentSolverCategoryFilterCustomization::MakeInstance));

	// Category metadata is the pipe-separated string as declared on the
	// UPROPERTY (e.g. "Muto RL|Contact|Joint Limits") -- StartsWith against
	// each allowed prefix so a sub-category like that one still lands under
	// its parent's tab without having to allowlist every leaf individually.
	View->SetIsPropertyVisibleDelegate(FIsPropertyVisible::CreateLambda(
		[AllowedCategoryPrefixes](const FPropertyAndParent& PropertyAndParent)
		{
			const FString* Category = PropertyAndParent.Property.FindMetaData(TEXT("Category"));
			if (!Category)
			{
				return false;
			}
			for (const FString& Prefix : AllowedCategoryPrefixes)
			{
				if (Category->StartsWith(Prefix))
				{
					return true;
				}
			}
			return false;
		}));

	return View;
}

void SAgentSolverControlPanel::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);

	// See LastKnownSourceActor's comment -- SetObject is skipped unless the
	// resolved actor instance actually changed (e.g. across PIE start/stop or
	// a viewport-source switch), not re-run every frame. Agent/Physics tabs
	// follow the VIEWPORT'S current source (Ragdoll/Visualizer), not
	// necessarily the plain training driver -- FindTrainingDriver() below is
	// used separately, only for the Play/Stop/stats/graphs, which are
	// specifically about the plain driver's own training loop.
	const EAgentSolverViewportSource Source = ViewportSettings.IsValid() ? ViewportSettings->ViewportSource : EAgentSolverViewportSource::Ragdoll;
	AMutoRLTrainingDriver* SourceActor = AgentSolverUI::FindViewportSourceActor(Source);
	if (SourceActor != LastKnownSourceActor.Get())
	{
		LastKnownSourceActor = SourceActor;

		if (AgentParametersView.IsValid())
		{
			AgentParametersView->SetObject(SourceActor);
		}
		if (PhysicsParametersView.IsValid())
		{
			PhysicsParametersView->SetObject(SourceActor);
		}
	}

	// IsPIERunning() specifically, not just Driver -- FindTrainingDriver()
	// deliberately falls back to the plain editor-world actor when PIE isn't
	// running (so the Agent/Physics tabs stay inspectable pre-Play), but that
	// instance's StartTraining() never legitimately runs outside PIE, so
	// sampling it produces nothing but an endless run of meaningless zero
	// samples that fill the graphs' ring buffers before real data ever shows up.
	AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();

	// RewardSettingsView tracks the SAME actor Driver resolves to (see this
	// class's header comment for why it's FindTrainingDriver() and not the
	// viewport source) -- separate change-tracking from LastKnownSourceActor
	// above since the two views can legitimately point at different actors.
	if (Driver != LastKnownDriverForReward.Get())
	{
		LastKnownDriverForReward = Driver;
		if (RewardSettingsView.IsValid())
		{
			RewardSettingsView->SetObject(Driver);
		}
	}

	if (Driver && AgentSolverUI::IsPIERunning() && (InCurrentTime - LastGraphSampleTime) >= GraphSampleIntervalSeconds)
	{
		const int32 CurrentStepCount = Driver->GetTrainingStepCount();
		// LastGraphSampleTime == 0.0 means "first sample" -- no rate to
		// compute yet (there's no prior step count to diff against), so
		// report 0 rather than a spurious spike from CurrentStepCount - 0.
		const bool bHasPriorSample = LastGraphSampleTime > 0.0;
		const double Elapsed = bHasPriorSample ? (InCurrentTime - LastGraphSampleTime) : GraphSampleIntervalSeconds;
		const float StepsPerSecond = bHasPriorSample ? (float)((CurrentStepCount - LastGraphStepCount) / FMath::Max(Elapsed, KINDA_SMALL_NUMBER)) : 0.0f;
		const float CurrentReward = Driver->GetAverageReward();

		if (RewardGraph.IsValid())
		{
			RewardGraph->AddSample(CurrentReward);
		}
		if (ThroughputGraph.IsValid() && bHasPriorSample)
		{
			ThroughputGraph->AddSample(StepsPerSecond);
		}
		if (TorsoHeightBonusGraph.IsValid())
		{
			TorsoHeightBonusGraph->AddSample(Driver->GetLastTorsoHeightBonus());
		}
		if (EnergyConsumptionMalusGraph.IsValid())
		{
			EnergyConsumptionMalusGraph->AddSample(Driver->GetLastEnergyConsumptionMalus());
		}
		if (MusclesUseMalusGraph.IsValid())
		{
			MusclesUseMalusGraph->AddSample(Driver->GetLastMusclesUseMalus());
		}

		// Diagnostic for "the graphs show a flat line" -- confirms samples are
		// actually being taken (proves Tick() is firing and Driver is found,
		// which a flat/empty-looking graph alone can't distinguish from), and
		// shows the RAW values being sampled so a genuinely flat/near-zero
		// signal (e.g. AverageReward barely having moved yet) is visible as
		// such rather than looking identical to "nothing is being sampled at
		// all". Throttled to once every ~10 samples (~5s at the default
		// GraphSampleIntervalSeconds) to stay readable.
		++GraphSampleCount;
		if (GraphSampleCount == 1 || GraphSampleCount % 10 == 0)
		{
			UE_LOG(LogTemp, Log, TEXT("SAgentSolverControlPanel: graph sample #%d -- TrainingStepCount=%d (delta=%d over %.2fs), AverageReward=%.4f, StepsPerSecond=%.2f"),
				GraphSampleCount, CurrentStepCount, CurrentStepCount - LastGraphStepCount, Elapsed, CurrentReward, StepsPerSecond);
		}

		LastGraphSampleTime = InCurrentTime;
		LastGraphStepCount = CurrentStepCount;
	}
}

void SAgentSolverControlPanel::Construct(const FArguments& InArgs)
{
	ViewportSettings = TStrongObjectPtr<UAgentSolverViewportSettings>(NewObject<UAgentSolverViewportSettings>());

	// Plain, unfiltered details view over UAgentSolverViewportSettings --
	// ViewportSource (an enum) and EnvironmentLevel (a TSoftObjectPtr<UWorld>)
	// render as a native combo box and asset picker respectively, for free,
	// same as EnvIndexToVisualize/bShowFloor already did. No bespoke rows on
	// top of this view -- see AgentParameterCategoryPrefixes' comment for why.
	{
		FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		FDetailsViewArgs DetailsViewArgs;
		DetailsViewArgs.bAllowSearch = false;
		DetailsViewArgs.bShowOptions = false;
		DetailsViewArgs.bShowObjectLabel = false;
		DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
		ViewportParametersView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
		ViewportParametersView->SetObject(ViewportSettings.Get());
	}

	AgentParametersView = CreateFilteredDetailsView(AgentParameterCategoryPrefixes);
	PhysicsParametersView = CreateFilteredDetailsView(PhysicsParameterCategoryPrefixes);
	// Same filtered-details-onto-the-real-actor pattern as Agent/Physics above
	// -- see this class's header comment for why this is wired to the actual
	// AMutoRLTrainingDriver instance (via RewardSettingsPrefixes) instead of a
	// separate disconnected settings object.
	RewardSettingsView = CreateFilteredDetailsView(RewardSettingsCategoryPrefixes);

	SAssignNew(RewardGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.3f, 0.8f, 0.4f)).MaxSamples(150);
	SAssignNew(ThroughputGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.3f, 0.6f, 1.0f)).MaxSamples(150);
	SAssignNew(TorsoHeightBonusGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.8f, 0.7f, 0.2f)).MaxSamples(150);
	SAssignNew(EnergyConsumptionMalusGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.9f, 0.4f, 0.2f)).MaxSamples(150);
	SAssignNew(MusclesUseMalusGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.8f, 0.3f, 0.6f)).MaxSamples(150);

	const FMargin ButtonPadding(2.0f, 4.0f);
	const FMargin IconButtonPadding(4.0f);

	// clang-format off
	ChildSlot
	[
		SNew(SVerticalBox)

		// ---- Toolbar ----
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(4.0f)
		[
			SNew(SHorizontalBox)

			+ SHorizontalBox::Slot().AutoWidth().Padding(ButtonPadding)
			[
				SNew(SButton)
				.ButtonStyle(FAppStyle::Get(), "NoBorder")
				.ContentPadding(IconButtonPadding)
				.IsEnabled(this, &SAgentSolverControlPanel::IsStartTrainingEnabled)
				.ToolTipText(LOCTEXT("StartTrainingTooltip", "Launches PIE if it isn't running yet (training then auto-starts via bAutoStartOnBeginPlay), or manually (re)starts training if PIE is already running. Never runs training against the editor-world instance."))
				.OnClicked(this, &SAgentSolverControlPanel::OnStartTrainingClicked)
				[
					SNew(SBox).WidthOverride(24.0f).HeightOverride(24.0f)
					[
						SNew(SImage)
						.Image(FAppStyle::GetBrush("PlayWorld.PlayInViewport"))
						.ColorAndOpacity(FLinearColor(0.15f, 0.85f, 0.15f))
					]
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(ButtonPadding)
			[
				SNew(SButton)
				.ButtonStyle(FAppStyle::Get(), "NoBorder")
				.ContentPadding(IconButtonPadding)
				.IsEnabled(this, &SAgentSolverControlPanel::IsStopTrainingEnabled)
				.ToolTipText(LOCTEXT("StopTrainingTooltip", "Stops the background training thread and the Python trainer process without ending PIE."))
				.OnClicked(this, &SAgentSolverControlPanel::OnStopTrainingClicked)
				[
					SNew(SBox).WidthOverride(24.0f).HeightOverride(24.0f)
					[
						SNew(SImage)
						.Image(FAppStyle::GetBrush("PlayWorld.StopPlaySession.Small"))
						.ColorAndOpacity(FLinearColor(0.9f, 0.2f, 0.2f))
					]
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(ButtonPadding * 2.0f)[ SNew(SSeparator).Orientation(Orient_Vertical) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(ButtonPadding)
			[
				SNew(SButton)
				.Text(LOCTEXT("SaveToAssets", "Save To Assets"))
				.ToolTipText(LOCTEXT("SaveToAssetsTooltip", "Copies the live Policy's weights into the Save*NetworkAsset slots (see the Learning category) -- the only thing that writes into a network asset's NeuralNetworkData."))
				.OnClicked(this, &SAgentSolverControlPanel::OnSaveToAssetsClicked)
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(ButtonPadding)
			[
				SNew(SButton)
				.Text(LOCTEXT("LoadFromAssets", "Load From Assets"))
				.ToolTipText(LOCTEXT("LoadFromAssetsTooltip", "Hot-swaps the live Policy's weights from the Load*NetworkAsset slots (see the Learning category)."))
				.OnClicked(this, &SAgentSolverControlPanel::OnLoadFromAssetsClicked)
			]
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(4.0f, 0.0f, 4.0f, 0.0f)[ SNew(SSeparator) ]

		// ---- Statistics + Graphs / Reward Settings | Viewport | Parameters ----
		+ SVerticalBox::Slot()
		.FillHeight(1.0f)
		[
			SNew(SSplitter)

			+ SSplitter::Slot().Value(0.22f)
			[
				SNew(SSplitter)
				.Orientation(Orient_Vertical)

				+ SSplitter::Slot().Value(0.6f)
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f)
						[
							SNew(STextBlock)
							.AutoWrapText(true)
							.Text(this, &SAgentSolverControlPanel::GetStatusText)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 6.0f)
						[
							SNew(STextBlock)
							.AutoWrapText(true)
							.Text(this, &SAgentSolverControlPanel::GetStatsText)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("RewardGraphTitle", "Reward (EMA) over time")).ColorAndOpacity(SectionHeaderColor)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							RewardGraph.ToSharedRef()
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("ThroughputGraphTitle", "Training steps/sec")).ColorAndOpacity(SectionHeaderColor)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							ThroughputGraph.ToSharedRef()
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("TorsoHeightBonusGraphTitle", "Torso height bonus (EMA)")).ColorAndOpacity(SectionHeaderColor)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							TorsoHeightBonusGraph.ToSharedRef()
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("EnergyMalusGraphTitle", "Energy consumption malus (EMA)")).ColorAndOpacity(SectionHeaderColor)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							EnergyConsumptionMalusGraph.ToSharedRef()
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("MusclesUseMalusGraphTitle", "Muscles use malus (EMA)")).ColorAndOpacity(SectionHeaderColor)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							MusclesUseMalusGraph.ToSharedRef()
						]
					]
				]

				+ SSplitter::Slot().Value(0.4f)
				[
					RewardSettingsView.ToSharedRef()
				]
			]

			+ SSplitter::Slot().Value(0.56f)
			[
				SAssignNew(ViewportWidget, SAgentSolverViewport)
				.Settings(ViewportSettings.Get())
			]

			+ SSplitter::Slot().Value(0.22f)
			[
				SNew(SVerticalBox)

				+ SVerticalBox::Slot().AutoHeight().Padding(2.0f)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(1.0f)
					[
						MakeTabButton(LOCTEXT("ViewportParamsTab", "Viewport"),
							TAttribute<bool>::CreateLambda([this]() { return ActiveParameterTabIndex == 0; }),
							FOnClicked::CreateSP(this, &SAgentSolverControlPanel::OnSelectParameterTab, 0))
					]
					+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(1.0f)
					[
						MakeTabButton(LOCTEXT("AgentParamsTab", "Agent"),
							TAttribute<bool>::CreateLambda([this]() { return ActiveParameterTabIndex == 1; }),
							FOnClicked::CreateSP(this, &SAgentSolverControlPanel::OnSelectParameterTab, 1))
					]
					+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(1.0f)
					[
						MakeTabButton(LOCTEXT("PhysicsParamsTab", "Physics"),
							TAttribute<bool>::CreateLambda([this]() { return ActiveParameterTabIndex == 2; }),
							FOnClicked::CreateSP(this, &SAgentSolverControlPanel::OnSelectParameterTab, 2))
					]
				]

				+ SVerticalBox::Slot().FillHeight(1.0f)
				[
					SAssignNew(ParameterSwitcher, SWidgetSwitcher)
					+ SWidgetSwitcher::Slot()[ ViewportParametersView.ToSharedRef() ]
					+ SWidgetSwitcher::Slot()[ AgentParametersView.ToSharedRef() ]
					+ SWidgetSwitcher::Slot()[ PhysicsParametersView.ToSharedRef() ]
				]
			]
		]
	];
	// clang-format on
}

#undef LOCTEXT_NAMESPACE

#endif // WITH_EDITOR
