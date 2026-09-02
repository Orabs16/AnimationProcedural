#pragma once

// Editor-only Window-menu panel for AMutoRLTrainingDriver -- a toolbar
// (Play/Stop icon buttons) above a 3-pane layout: statistics + reward/
// throughput/reward-component graphs on top-left / reward-tuning settings on
// bottom-left, a live embedded viewport filling the middle
// (SAgentSolverViewport), and a parameter switcher on the right (Viewport /
// Agent / Physics), each tab just an IDetailsView -- Viewport unfiltered
// (UAgentSolverViewportSettings, so ViewportSource/EnvironmentLevel show as
// their normal native combo box / asset picker rows), Agent filtered to the
// real AMutoRLTrainingDriver ALWAYS (never the viewport source -- see
// Tick()'s comment for why that used to be a real bug), Physics filtered to
// the viewport-source actor normally but the real driver while training is
// active (same reasoning Gravity's own comment covers).
// Deliberately NOT duplicated with bespoke Slate rows on top of these views
// -- IDetailsView already renders enum properties as combo boxes and object
// properties as asset pickers for free, so anything hand-built here would
// just be the same control twice.
//
// Reward Settings (bottom-left) is the SAME pattern as Agent/Physics -- a
// CreateFilteredDetailsView onto the actual AMutoRLTrainingDriver (Category
// "Muto RL|Tuning", see that class's RewardHeightTarget/RewardHeightMultiplier/
// RewardEnergyConsumptionMultiplier/RewardMusclesUseMultiplier), not a
// separate disconnected settings object -- editing it here IS editing the
// live actor's real UPROPERTYs, which CreatureRLEnvironment::ComputeReward
// actually reads via Driver->Config (see AMutoRLTrainingDriver::StartTraining's
// Config-mirroring). Tracks AgentSolverUI::FindTrainingDriver() specifically
// (not the viewport source) since these values are only ever consumed by the
// plain driver's own training loop -- Ragdoll/Visualizer inherit the same
// UPROPERTYs but never call ComputeReward, so editing them there would look
// like it does nothing.
//
// Styling mirrors MassMuscleProfile's SMassMuscleMainWidget for the tab
// header strip specifically: NoBorder SButton -> tinted SBorder -> fixed-
// height SBox -> centered STextBlock, active/inactive tracked by a plain
// int32 alongside the SWidgetSwitcher (see OnTabClicked/ActiveTabIndex
// there) -- reused here via MakeTabButton.
//
// Not WITH_EDITOR-gated itself -- like MutoRLTrainingDriver.h's own
// UCLASS/UPROPERTY declarations, this header is fine to parse unconditionally
// (only forward declarations for the Editor-only widget/view types it holds
// as TSharedPtr members -- TSharedPtr doesn't need a complete type to be
// declared, only to be dereferenced); only the .cpp's actual GEditor/
// TActorIterator/viewport/details-view logic is gated (UnrealEd/
// PropertyEditor/AdvancedPreviewScene are Editor-only modules -- see
// AgentSolver.Build.cs).

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Framework/SlateDelegates.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/WeakObjectPtr.h"
#include "UIControls/AgentSolverViewportSettings.h"

class AMutoRLTrainingDriver;
class SAgentSolverViewport;
class SAgentSolverLineGraph;
class IDetailsView;
class SWidgetSwitcher;
class UAgentSolverPreset;
struct FPropertyChangedEvent;

class SAgentSolverControlPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAgentSolverControlPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/**
	 * Applies a preset: restores its EnvironmentLevel into ViewportSettings
	 * (the cosmetic preview-scene level, NOT a switch of the editor's actual
	 * open level -- see UAgentSolverPreset::EnvironmentLevel's comment for
	 * why an earlier version of this function did that and why it was
	 * removed), then copies every Rig/Network/Tuning field onto whatever
	 * AMutoRLTrainingDriver AgentSolverUI::FindTrainingDriver() finds in the
	 * currently open level. Rig assets (SkeletalMesh/MassAsset/MuscleAsset)
	 * are only overwritten when the preset actually has one set -- an empty
	 * preset field leaves the driver's current assignment alone rather than
	 * nulling it out, since StartTraining() hard-requires all three (see
	 * this function's own comment in the .cpp for the crash that motivated
	 * this). Sets ViewportSettings->ActivePreset, which is what
	 * SyncActivePresetFromDriver below and the Viewport tab's own picker row
	 * both read. Called by FAgentSolverModule (double-click on a preset
	 * asset, or the "load first found" fallback when the tool opens with
	 * nothing pending) and by OnActivePresetPicked below when the Viewport
	 * tab's picker row itself changes.
	 */
	void LoadPreset(UAgentSolverPreset* Preset);

	/** Asset-registry scan for the first UAgentSolverPreset found, applied via LoadPreset if one exists. Called once by FAgentSolverModule::SpawnControlPanelTab when the tool opens with no specific preset pending. */
	void LoadFirstAvailablePreset();

	/**
	 * Re-points PhysicsParametersView at whichever actor AgentSolverUI::
	 * FindViewportSourceActor(ViewportSettings->ViewportSource) currently
	 * returns (or the real training driver instead, while training is
	 * active -- see this method's own .cpp comment), and AgentParametersView/
	 * RewardSettingsView at AgentSolverUI::FindTrainingDriver() ALWAYS (a
	 * separate lookup, tracked separately -- see this class's header
	 * comment for why Agent must never follow the viewport source), both
	 * only on actual actor-instance CHANGE (e.g. across PIE start/stop, a
	 * viewport-source switch, or a training start/stop), and, at most once
	 * every GraphSampleIntervalSeconds, pushes a new sample into the reward/
	 * throughput/reward-component graphs (also from FindTrainingDriver()).
	 * All cheap (a couple of actor-class iterators; the per-actor-change work
	 * is skipped entirely on unchanged frames).
	 */
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;

private:
	FText GetStatusText() const;
	FText GetStatsText() const;

	/** Visible only while the active driver has a baked reference pose/motion -- on a standing run the four imitation graphs would just be permanently-empty boxes. */
	EVisibility GetImitationGraphVisibility() const;

	FReply OnStartTrainingClicked();
	FReply OnStopTrainingClicked();
	/** Requires AgentSolverUI::IsPIERunning() -- see that function's comment for why StartTraining must never run against the editor-world instance. */
	bool IsStartTrainingEnabled() const;
	bool IsStopTrainingEnabled() const;

	/** Copies Policy's current weights into Save*NetworkAsset (see the Learning category) -- the ONLY thing that actually writes into a network asset's NeuralNetworkData; nothing else calls AMutoRLTrainingDriver::SaveTrainedNetworksToAssets. */
	FReply OnSaveToAssetsClicked();
	/** Hot-swaps Policy's live weights from Load*NetworkAsset (see the Learning category). */
	FReply OnLoadFromAssetsClicked();

	/**
	 * One of the right-hand parameter tabs (Agent/Physics): an IDetailsView
	 * on the found actor, showing only properties whose UPROPERTY Category
	 * starts with one of AllowedCategoryPrefixes. Object to inspect is set
	 * later, per-frame, by Tick().
	 */
	TSharedRef<IDetailsView> CreateFilteredDetailsView(const TArray<FString>& AllowedCategoryPrefixes);

	/** Which of the 3 right-hand parameter tabs is showing -- see ActiveParameterTabIndex. */
	FReply OnSelectParameterTab(int32 TabIndex);

	/** One tab-strip button matching SMassMuscleMainWidget::OnTabClicked's Details/Settings header look (NoBorder button, tinted border, fixed-height centered label). */
	static TSharedRef<SWidget> MakeTabButton(const FText& Label, TAttribute<bool> bIsActive, FOnClicked OnClicked);

	/**
	 * Bound to AgentParametersView/PhysicsParametersView/RewardSettingsView's
	 * OnFinishedChangingProperties -- whenever an edit finishes on any of
	 * those (Rig assets, network Load/Save slots, or a tuning knob), copies
	 * the driver's current values for every preset-tracked field back into
	 * ViewportSettings->ActivePreset and marks it dirty. No-ops if no preset
	 * is active. Deliberately mark-dirty only, not an auto-disk-save --
	 * unlike AMutoRLTrainingDriver::SaveTrainedNetworksToAssets (a deliberate
	 * one-off "save my trained weights" click), this can fire many times a
	 * second while dragging a slider, so forcing a disk write on every call
	 * would be excessive; Ctrl+S / Save All still persists it same as any
	 * other dirty asset.
	 */
	void SyncActivePresetFromDriver(const FPropertyChangedEvent& Event);

	/** Bound to ViewportParametersView's OnFinishedChangingProperties -- if the edit was to ViewportSettings->ActivePreset (the Viewport tab's preset picker row), applies the newly picked preset via LoadPreset. */
	void OnActivePresetPicked(const FPropertyChangedEvent& Event);

	/** Backing object for the Viewport tab -- see AgentSolverViewportSettings.h. Owned here (kept alive for the panel's lifetime); the embedded viewport client only holds a weak reference. */
	TStrongObjectPtr<UAgentSolverViewportSettings> ViewportSettings;

	TSharedPtr<SAgentSolverViewport> ViewportWidget;
	TSharedPtr<IDetailsView> ViewportParametersView;
	TSharedPtr<IDetailsView> AgentParametersView;
	TSharedPtr<IDetailsView> PhysicsParametersView;
	TSharedPtr<IDetailsView> RewardSettingsView;
	TSharedPtr<SWidgetSwitcher> ParameterSwitcher;
	/** Tracked alongside ParameterSwitcher, same pattern as SMassMuscleMainWidget::ActiveTabIndex -- lets MakeTabButton's border-tint lambda read "am I the active tab" without querying the switcher. */
	int32 ActiveParameterTabIndex = 0;

	TSharedPtr<SAgentSolverLineGraph> RewardGraph;
	TSharedPtr<SAgentSolverLineGraph> ThroughputGraph;
	/** Visualization for the wired-in Reward Settings terms -- see AMutoRLTrainingDriver::GetLastTorsoHeightBonus/GetLastEnergyConsumptionMalus/GetLastMusclesUseMalus. */
	TSharedPtr<SAgentSolverLineGraph> TorsoHeightBonusGraph;
	TSharedPtr<SAgentSolverLineGraph> EnergyConsumptionMalusGraph;
	TSharedPtr<SAgentSolverLineGraph> MusclesUseMalusGraph;
	/**
	 * The four imitation terms -- see AMutoRLTrainingDriver::GetLastPoseReward
	 * and friends. Each is in (0,1] by construction, so they share a natural
	 * common scale the maluses above do not.
	 *
	 * The pose graph is the one to watch first: immediately after a reset with
	 * bResetToReferencePose on, it should sit at ~1.0. If it does not, the
	 * retarget or the bake is wrong and nothing else on this panel means
	 * anything yet.
	 */
	TSharedPtr<SAgentSolverLineGraph> PoseRewardGraph;
	TSharedPtr<SAgentSolverLineGraph> VelocityRewardGraph;
	TSharedPtr<SAgentSolverLineGraph> EndEffectorRewardGraph;
	TSharedPtr<SAgentSolverLineGraph> RootRewardGraph;

	/** So Tick() only calls SetObject on an actor change, not every frame -- tracks PhysicsParametersView's target only (the one view that can legitimately follow the viewport source). */
	TWeakObjectPtr<AMutoRLTrainingDriver> LastKnownSourceActor;
	/** Same purpose as LastKnownSourceActor, but for AgentParametersView AND RewardSettingsView, which both ALWAYS track FindTrainingDriver() -- never the viewport source, see this class's header comment for why. */
	TWeakObjectPtr<AMutoRLTrainingDriver> LastKnownDriverForReward;

	/** Wall-clock time (FSlateApplication::GetCurrentTime()-scale) of the last graph sample; see GraphSampleIntervalSeconds. */
	double LastGraphSampleTime = 0.0;
	/** GetTrainingStepCount() as of the last graph sample -- diffed against the current count to turn a monotonically-increasing counter into a "steps/sec" rate, which is what's actually informative to look at over time. */
	int32 LastGraphStepCount = 0;
	/** Total graph samples taken since this panel was constructed -- purely to throttle the diagnostic log in Tick() to roughly once every 5 seconds instead of every sample. */
	int32 GraphSampleCount = 0;
};
