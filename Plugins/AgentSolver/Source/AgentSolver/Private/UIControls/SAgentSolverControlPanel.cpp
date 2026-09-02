#include "UIControls/SAgentSolverControlPanel.h"

#if WITH_EDITOR

#include "UIControls/AgentSolverUIUtils.h"
#include "UIControls/SAgentSolverViewport.h"
#include "UIControls/SAgentSolverLineGraph.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
#include "AgentSolver/AgentSolverPreset.h"
#include "Components/PoseableMeshComponent.h"
#include "UIControls/LearningAgentsNeuralNetworkFactory.h"
#include "UIControls/AgentSolverPresetFactory.h"
#include "LearningAgentsNeuralNetwork.h"

#include "AssetRegistry/AssetRegistryModule.h"

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
	// "Muto RL|Imitation" is the STRUCTURAL imitation block (objective mode,
	// the clip, bake settings, end-effector names) -- read once by
	// StartTraining, so it belongs with the other setup values here rather
	// than in the always-visible Reward Settings pane, which is for knobs that
	// respond live. The imitation WEIGHTS live under "Muto RL|Tuning|Imitation"
	// and are picked up by RewardSettingsCategoryPrefixes below instead.
	const TArray<FString> AgentParameterCategoryPrefixes = { TEXT("Muto RL|Rig"), TEXT("Muto RL|Reward"), TEXT("Muto RL|Reset"), TEXT("Muto RL|Training"), TEXT("Muto RL|Learning"), TEXT("Muto RL|Imitation") };
	const TArray<FString> PhysicsParameterCategoryPrefixes = { TEXT("Muto RL|Simulation"), TEXT("Muto RL|Contact"), TEXT("Muto RL|Diagnostics") };
	// Deliberately "Muto RL|Tuning", not "Muto RL|Reward" -- see
	// AMutoRLTrainingDriver::RewardHeightTarget's comment: nesting under
	// "Muto RL|Reward|..." would ALSO match AgentParameterCategoryPrefixes'
	// "Muto RL|Reward" entry (StartsWith is a raw string prefix check, not
	// pipe-segment-aware), showing every field in both the Agent tab AND here.
	// "Muto RL|Simulation|Gravity" is Gravity specifically (see its own
	// comment) -- deliberately NOT the bare "Muto RL|Simulation", which would
	// also pull in bAutoStartOnBeginPlay/NumEnvs/etc. from the Physics tab.
	// "Muto RL|Reward|Core Weights"/"Torque"/"Termination" (2026-08-25) are
	// AliveBonus/UprightWeight/BalanceWeight/TorquePenaltyWeight/
	// MaxTorquePerDOF/MinUprightDot/MinHeightFraction specifically -- same
	// Gravity-style dual-tab trick, deliberately NOT the bare "Muto
	// RL|Reward", which would also pull in TargetTorsoHeightOverride (a
	// structural setup value, not a tuning knob) from the Agent tab.
	const TArray<FString> RewardSettingsCategoryPrefixes = {
		TEXT("Muto RL|Tuning"), TEXT("Muto RL|Simulation|Gravity"),
		TEXT("Muto RL|Reward|Core Weights"), TEXT("Muto RL|Reward|Torque"), TEXT("Muto RL|Reward|Termination")
	};

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

			// Same "create new" (+) treatment for the Viewport tab's preset
			// picker row (UAgentSolverViewportSettings::ActivePreset) --
			// GetProperty() on this object's-own-class-only lookup returns an
			// invalid handle when this customization runs for
			// AMutoRLTrainingDriver/UPoseableMeshComponent instead, same
			// guard as the network-asset loop above.
			static TStrongObjectPtr<UAgentSolverPresetFactory> PresetFactory(NewObject<UAgentSolverPresetFactory>());
			TSharedRef<IPropertyHandle> PresetHandle = DetailBuilder.GetProperty(
				GET_MEMBER_NAME_CHECKED(UAgentSolverViewportSettings, ActivePreset), UAgentSolverViewportSettings::StaticClass());
			if (IDetailPropertyRow* PresetRow = DetailBuilder.EditDefaultProperty(PresetHandle))
			{
				PresetRow->CustomWidget()
					.NameContent()
					[
						PresetHandle->CreatePropertyNameWidget()
					]
					.ValueContent()
					[
						SNew(SObjectPropertyEntryBox)
						.PropertyHandle(PresetHandle)
						.AllowedClass(UAgentSolverPreset::StaticClass())
						.NewAssetFactories(TArray<UFactory*>{ PresetFactory.Get() })
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

EVisibility SAgentSolverControlPanel::GetImitationGraphVisibility() const
{
	const AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	return (Driver && Driver->HasReferenceMotion()) ? EVisibility::Visible : EVisibility::Collapsed;
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

void SAgentSolverControlPanel::LoadPreset(UAgentSolverPreset* Preset)
{
	if (!Preset)
	{
		return;
	}
	if (ViewportSettings.IsValid())
	{
		ViewportSettings->ActivePreset = Preset;

		// EnvironmentLevel here is UAgentSolverViewportSettings::
		// EnvironmentLevel -- the cosmetic preview-scene level the embedded
		// viewport copies static geometry from (see that property's own
		// comment), NOT a request to switch the actual editor's currently
		// open level. An earlier version of this function did exactly that
		// via FEditorFileUtils::LoadMap, based on a misreading of what
		// "environment level" meant here -- LoadMap tears down and rebuilds
		// the whole world, and calling it synchronously from inside a
		// tab-spawn/double-click callback was unsafe, which is the likely
		// cause of the 2026-08-25 "preset doesn't load at tool-open" and
		// "doesn't switch levels" reports. Removed entirely: restoring the
		// preview-scene level is enough to make "reopen the project, get my
		// last environment back" work without ever touching the editor's
		// actual open level.
		ViewportSettings->EnvironmentLevel = Preset->EnvironmentLevel;
	}

	// Everything else applies to whatever driver is in the currently open
	// level. Network/tuning fields are copied unconditionally
	// -- an empty slot or a default-scale weight is a perfectly meaningful,
	// deliberate value there. Rig assets are NOT: SkeletalMesh/MassAsset/
	// MuscleAsset are structurally REQUIRED for StartTraining() to do
	// anything at all (it bails out at the very first check if any of the
	// three is null) -- unconditionally nulling them from an incompletely-
	// filled preset (e.g. one just created via "+" and not populated yet)
	// silently broke training for the rest of the PIE session: StartTraining
	// bailed before ever touching Policy's network assets, so Policy existed
	// but GetEncoderNetworkAsset() etc. stayed null, and stopping PIE then
	// crashed inside SaveNetworkToSnapshot calling through that null pointer
	// (2026-08-25 bug report -- matches EXCEPTION_ACCESS_VIOLATION reading
	// 0x38, LearningAgentsNeuralNetwork.cpp:49's `this->NeuralNetworkData`
	// with `this` null). So: only overwrite a Rig slot when the preset
	// actually has something in it; an empty preset field leaves the
	// driver's current assignment alone instead of wiping it.
	AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();
	if (!Driver)
	{
		return;
	}

	// See UAgentSolverPreset::ApplyToDriver's comment -- this used to be a
	// field-by-field copy inline here; extracted so ULearningProgramNode's
	// per-stage Params can reuse the exact same logic.
	Preset->ApplyToDriver(Driver);
}

void SAgentSolverControlPanel::LoadFirstAvailablePreset()
{
	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	TArray<FAssetData> Assets;
	AssetRegistryModule.Get().GetAssetsByClass(UAgentSolverPreset::StaticClass()->GetClassPathName(), Assets);
	if (Assets.Num() > 0)
	{
		if (UAgentSolverPreset* Preset = Cast<UAgentSolverPreset>(Assets[0].GetAsset()))
		{
			LoadPreset(Preset);
		}
	}
}

void SAgentSolverControlPanel::SyncActivePresetFromDriver(const FPropertyChangedEvent& Event)
{
	UAgentSolverPreset* Preset = ViewportSettings.IsValid() ? ViewportSettings->ActivePreset : nullptr;
	if (!Preset)
	{
		return;
	}

	// EnvironmentLevel lives on ViewportSettings, not the driver -- synced
	// unconditionally here, independent of whether a driver is currently
	// found, so "reopen the project, get my last environment back" works
	// even when edited from a context with no driver in the open level.
	Preset->EnvironmentLevel = ViewportSettings->EnvironmentLevel;

	if (AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver())
	{
		Preset->SkeletalMesh = Driver->SkeletalMesh;
		Preset->MassAsset = Driver->MassAsset;
		Preset->MuscleAsset = Driver->MuscleAsset;
		Preset->LoadEncoderNetworkAsset = Driver->LoadEncoderNetworkAsset;
		Preset->LoadPolicyNetworkAsset = Driver->LoadPolicyNetworkAsset;
		Preset->LoadDecoderNetworkAsset = Driver->LoadDecoderNetworkAsset;
		Preset->SaveEncoderNetworkAsset = Driver->SaveEncoderNetworkAsset;
		Preset->SavePolicyNetworkAsset = Driver->SavePolicyNetworkAsset;
		Preset->SaveDecoderNetworkAsset = Driver->SaveDecoderNetworkAsset;
		Preset->MaxTorquePerDOF = Driver->MaxTorquePerDOF;
		Preset->MinUprightDot = Driver->MinUprightDot;
		Preset->MinHeightFraction = Driver->MinHeightFraction;
		Preset->AliveBonus = Driver->AliveBonus;
		Preset->UprightWeight = Driver->UprightWeight;
		Preset->BalanceWeight = Driver->BalanceWeight;
		Preset->TorquePenaltyWeight = Driver->TorquePenaltyWeight;
		Preset->RewardHeightTarget = Driver->RewardHeightTarget;
		Preset->RewardHeightMultiplier = Driver->RewardHeightMultiplier;
		Preset->RewardEnergyConsumptionMultiplier = Driver->RewardEnergyConsumptionMultiplier;
		Preset->RewardMusclesUseMultiplier = Driver->RewardMusclesUseMultiplier;
		Preset->GlobalRewardScale = Driver->GlobalRewardScale;
		Preset->GlobalRewardOffset = Driver->GlobalRewardOffset;
		Preset->GlobalMuscleStrengthScale = Driver->GlobalMuscleStrengthScale;
		Preset->MuscleActivationThresholdMultiplier = Driver->MuscleActivationThresholdMultiplier;
		Preset->Gravity = Driver->Gravity;
		Preset->bImitationObjective = (Driver->ObjectiveMode == EMutoObjectiveMode::Imitation);
		Preset->ReferenceMotion = Driver->ReferenceMotion;
		Preset->ReferencePoseTime = Driver->ReferencePoseTime;
		Preset->bImitateFullClip = Driver->bImitateFullClip;
		Preset->ReferenceSampleRate = Driver->ReferenceSampleRate;
		Preset->bReferenceMotionLoops = Driver->bReferenceMotionLoops;
		Preset->bResetToReferencePose = Driver->bResetToReferencePose;
		Preset->EndEffectorBoneNames = Driver->EndEffectorBoneNames;
		Preset->ImitationPoseWeight = Driver->ImitationPoseWeight;
		Preset->ImitationVelocityWeight = Driver->ImitationVelocityWeight;
		Preset->ImitationEndEffectorWeight = Driver->ImitationEndEffectorWeight;
		Preset->ImitationRootWeight = Driver->ImitationRootWeight;
		Preset->ImitationPoseErrorScale = Driver->ImitationPoseErrorScale;
		Preset->ImitationVelocityErrorScale = Driver->ImitationVelocityErrorScale;
		Preset->ImitationEndEffectorErrorScale = Driver->ImitationEndEffectorErrorScale;
		Preset->ImitationRootErrorScale = Driver->ImitationRootErrorScale;
		Preset->ImitationMaxPoseErrorRad = Driver->ImitationMaxPoseErrorRad;
		Preset->bImitationTerminateOnUprightAndHeight = Driver->bImitationTerminateOnUprightAndHeight;
	}
	Preset->MarkPackageDirty();
}

void SAgentSolverControlPanel::OnActivePresetPicked(const FPropertyChangedEvent& Event)
{
	if (!ViewportSettings.IsValid())
	{
		return;
	}
	if (Event.GetPropertyName() == GET_MEMBER_NAME_CHECKED(UAgentSolverViewportSettings, ActivePreset))
	{
		if (UAgentSolverPreset* Preset = ViewportSettings->ActivePreset)
		{
			LoadPreset(Preset);
		}
		return;
	}
	// Any other Viewport tab edit -- right now just EnvironmentLevel -- gets
	// written back into the active preset, same "keep the preset in sync"
	// rule SyncActivePresetFromDriver applies to the Agent/Physics/Reward
	// Settings tabs.
	SyncActivePresetFromDriver(Event);
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
			auto MatchesAllowedPrefix = [&AllowedCategoryPrefixes](const FProperty& Property)
			{
				const FString* Category = Property.FindMetaData(TEXT("Category"));
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
			};

			if (MatchesAllowedPrefix(PropertyAndParent.Property))
			{
				return true;
			}

			// Fall back to the parent chain -- a struct-VALUED UPROPERTY's own
			// Category (e.g. AMutoRLTrainingDriver::TrainingSettings' "Muto
			// RL|Training") only tags the struct property itself. Every LEAF
			// field inside FLearningAgentsPPOTrainingSettings/
			// FLearningAgentsPPOTrainerSettings/FLearningAgentsPolicySettings/
			// FLearningAgentsCriticSettings carries Epic's OWN "LearningAgents"
			// Category tag, which never matched any prefix here on its own --
			// so EVERY field of those 4 structs (EpsilonClip, GaeLambda,
			// bAdvantageNormalization, MinimumAdvantage/MaximumAdvantage,
			// bUseGradNormMaxClipping, GradNormMax, learning rates, batch
			// sizes, IterationsPerGather, etc.) was silently invisible in
			// every tab despite TrainingSettings itself living in "Muto
			// RL|Training" -- confirmed 2026-08-25 chasing why
			// bUseGradNormMaxClipping wasn't reachable anywhere in the panel.
			for (const FProperty* Parent : PropertyAndParent.ParentProperties)
			{
				if (Parent && MatchesAllowedPrefix(*Parent))
				{
					return true;
				}
			}
			return false;
		}));

	// Any edit finished on this view (Rig/Networks/Reward tuning, depending
	// on which of the 3 filtered views this is) writes the driver's current
	// preset-tracked fields back into the active preset, if any -- see
	// SyncActivePresetFromDriver's own comment.
	View->OnFinishedChangingProperties().AddSP(this, &SAgentSolverControlPanel::SyncActivePresetFromDriver);

	return View;
}

void SAgentSolverControlPanel::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);

	// IsPIERunning() specifically, not just Driver -- FindTrainingDriver()
	// deliberately falls back to the plain editor-world actor when PIE isn't
	// running (so the Agent/Physics tabs stay inspectable pre-Play), but that
	// instance's StartTraining() never legitimately runs outside PIE, so
	// sampling it produces nothing but an endless run of meaningless zero
	// samples that fill the graphs' ring buffers before real data ever shows up.
	AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();

	// Physics tab (Simulation/Contact/Diagnostics) normally follows the
	// VIEWPORT'S current preview source (Ragdoll/Visualizer), so it stays
	// useful for previewing/tuning physics against a cheap single-env
	// preview actor before committing to a full training run -- but while
	// training is actually running it switches to the real training driver
	// instead, so e.g. Gravity reads/writes the SAME actor training does
	// (2026-08-25 bug report -- editing it here used to silently edit the
	// PREVIEW actor's own copy while training read a completely different
	// actor's Gravity, with no visible link between the two).
	const bool bTrainingActive = Driver && Driver->IsTrainingActive();
	const EAgentSolverViewportSource Source = ViewportSettings.IsValid() ? ViewportSettings->ViewportSource : EAgentSolverViewportSource::Ragdoll;
	AMutoRLTrainingDriver* PhysicsSourceActor = bTrainingActive ? Driver : AgentSolverUI::FindViewportSourceActor(Source);
	if (PhysicsSourceActor != LastKnownSourceActor.Get())
	{
		LastKnownSourceActor = PhysicsSourceActor;
		if (PhysicsParametersView.IsValid())
		{
			PhysicsParametersView->SetObject(PhysicsSourceActor);
		}
	}

	// Agent tab (Rig/Reward/Reset/Training/Learning) and Reward Settings
	// ALWAYS track the real training driver, unconditionally -- unlike the
	// Physics tab above, there is no legitimate reason for these to ever
	// follow the viewport source. AMutoRLVisualizerActor/
	// AMutoRagdollVisualizerActor INHERIT from AMutoRLTrainingDriver (see
	// their class declarations), so they carry their OWN separate copies of
	// EVERY one of these fields -- SkeletalMesh, MassAsset, MuscleAsset,
	// AliveBonus, the 6 Load*/Save*NetworkAsset slots, all of it. Following
	// the viewport source here (as this tab used to, before 2026-08-25) meant
	// that before training ever started, inspecting or editing the Agent tab
	// silently showed/edited the PREVIEW actor's copies instead of the real
	// driver's -- worse than the Gravity trap, since it could make the
	// driver's OWN rig assets look configured while actually being unset,
	// and StartTraining() would then refuse to run with no visible reason
	// why (confirmed 2026-08-25: SkeletalMesh/MassAsset/MuscleAsset appeared
	// assigned in this tab while the real driver had none set, and PIE
	// crashed on stop trying to save a Policy that never finished setting up).
	if (Driver != LastKnownDriverForReward.Get())
	{
		LastKnownDriverForReward = Driver;
		if (AgentParametersView.IsValid())
		{
			AgentParametersView->SetObject(Driver);
		}
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
		// Only sampled while a reference is actually in play -- otherwise these
		// four would draw a flat line at whatever their EMAs were last left at,
		// which reads as "imitation is failing" when the run simply isn't
		// imitating anything.
		if (Driver->HasReferenceMotion())
		{
			if (PoseRewardGraph.IsValid()) { PoseRewardGraph->AddSample(Driver->GetLastPoseReward()); }
			if (VelocityRewardGraph.IsValid()) { VelocityRewardGraph->AddSample(Driver->GetLastVelocityReward()); }
			if (EndEffectorRewardGraph.IsValid()) { EndEffectorRewardGraph->AddSample(Driver->GetLastEndEffectorReward()); }
			if (RootRewardGraph.IsValid()) { RootRewardGraph->AddSample(Driver->GetLastRootReward()); }
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
	// Except ActivePreset (added 2026-08-25): that gets the SAME "create new
	// asset" (+) button as the 6 Load*/Save*NetworkAsset rows, via the same
	// FAgentSolverCategoryFilterCustomization registered against
	// UAgentSolverViewportSettings here -- its HideCategory calls are all
	// no-ops for this class (it has none of those categories), so this stays
	// otherwise unfiltered exactly as before.
	{
		FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		FDetailsViewArgs DetailsViewArgs;
		DetailsViewArgs.bAllowSearch = false;
		DetailsViewArgs.bShowOptions = false;
		DetailsViewArgs.bShowObjectLabel = false;
		DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
		ViewportParametersView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
		ViewportParametersView->RegisterInstancedCustomPropertyLayout(UAgentSolverViewportSettings::StaticClass(),
			FOnGetDetailCustomizationInstance::CreateStatic(&FAgentSolverCategoryFilterCustomization::MakeInstance));
		ViewportParametersView->SetObject(ViewportSettings.Get());

		// Picking a different preset here applies it -- see OnActivePresetPicked.
		ViewportParametersView->OnFinishedChangingProperties().AddSP(this, &SAgentSolverControlPanel::OnActivePresetPicked);
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
	SAssignNew(PoseRewardGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.3f, 0.8f, 0.9f)).MaxSamples(150);
	SAssignNew(VelocityRewardGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.4f, 0.6f, 0.9f)).MaxSamples(150);
	SAssignNew(EndEffectorRewardGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.5f, 0.9f, 0.5f)).MaxSamples(150);
	SAssignNew(RootRewardGraph, SAgentSolverLineGraph).LineColor(FLinearColor(0.7f, 0.7f, 0.9f)).MaxSamples(150);

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
						// Imitation terms. Visible only while a reference is
						// baked -- on a standing run they would draw four
						// permanently-empty graphs, which is noise, not
						// information.
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("PoseRewardGraphTitle", "Imitation: pose (EMA)")).ColorAndOpacity(SectionHeaderColor)
								.Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							SNew(SBox).Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
							[
								PoseRewardGraph.ToSharedRef()
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("VelocityRewardGraphTitle", "Imitation: joint velocity (EMA)")).ColorAndOpacity(SectionHeaderColor)
								.Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							SNew(SBox).Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
							[
								VelocityRewardGraph.ToSharedRef()
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("EndEffectorRewardGraphTitle", "Imitation: end effectors (EMA)")).ColorAndOpacity(SectionHeaderColor)
								.Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							SNew(SBox).Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
							[
								EndEffectorRewardGraph.ToSharedRef()
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 12.0f, 6.0f, 2.0f)
						[
							SNew(STextBlock).Text(LOCTEXT("RootRewardGraphTitle", "Imitation: root (EMA)")).ColorAndOpacity(SectionHeaderColor)
								.Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(6.0f, 0.0f)
						[
							SNew(SBox).Visibility(this, &SAgentSolverControlPanel::GetImitationGraphVisibility)
							[
								RootRewardGraph.ToSharedRef()
							]
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
