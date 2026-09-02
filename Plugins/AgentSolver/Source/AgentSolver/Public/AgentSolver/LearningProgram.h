#pragma once

// A curriculum for AMutoRLTrainingDriver: a graph of stages (nodes), each
// carrying its own bundle of training parameters (an instanced
// UAgentSolverPreset -- reward weights, muscle malus, gravity, imitation
// settings, exactly the same fields a standalone preset asset has, see
// ULearningProgramNode::Params below), connected by transitions that fire
// once a condition on live training progress is met.
//
// Authored visually in the Learning Program node editor (AgentSolverEditor
// module -- ULearningProgramEdGraph/Schema/GraphNode, and
// FLearningProgramAssetEditorToolkit for the double-click-to-open asset
// editor). This header is plain runtime data with no dependency on any of
// that -- the editor-only graph lives behind EdGraph below, referenced only
// through the base UEdGraph* type (itself part of the Engine module, not an
// editor module), the same way UBlueprint keeps its UberGraphPages.
//
// At runtime, AMutoRLTrainingDriver::ActiveLearningProgram walks Nodes
// starting at EntryNodeId, applying each node's Params to itself
// (UAgentSolverPreset::ApplyToDriver) and advancing along a node's
// Transitions when one of them triggers -- see
// AMutoRLTrainingDriver::TickLearningProgramTransitions.

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AgentSolver/AgentSolverPreset.h"
#include "LearningProgram.generated.h"

class UEdGraph;

/**
 * What a transition compares against. StepsSinceNodeEntry measures progress
 * SINCE THE CURRENT NODE WAS ENTERED, not since training started -- see
 * AMutoRLTrainingDriver::NodeEntryTrainingStepCount. AverageRewardTarget
 * compares against AMutoRLTrainingDriver::GetAverageReward(), which is a
 * whole-run EMA (not reset per node) -- its own decay is what makes it track
 * "recent" performance, so no separate per-node accumulator is needed.
 */
UENUM(BlueprintType)
enum class ELearningProgramConditionType : uint8
{
	/** Fires once GetAverageReward() reaches at least ThresholdValue. */
	AverageRewardTarget UMETA(DisplayName = "Average Reward Target"),

	/** Fires once GetTrainingStepCount() has advanced by at least ThresholdValue steps since this node was entered. */
	StepsSinceNodeEntry UMETA(DisplayName = "Steps Since Node Entry"),
};

/** One outgoing edge from a ULearningProgramNode. A node can have several -- only the first ENABLED one whose condition is met on a given step fires (see TickLearningProgramTransitions). */
USTRUCT()
struct FLearningProgramTransition
{
	GENERATED_BODY()

	/** Disabled transitions are skipped entirely, as if they didn't exist -- lets you keep a transition wired up (and its target chosen) without it firing, e.g. while tuning ThresholdValue. */
	UPROPERTY(EditAnywhere, Category = "Transition")
	bool bEnabled = true;

	UPROPERTY(EditAnywhere, Category = "Transition")
	ELearningProgramConditionType Condition = ELearningProgramConditionType::StepsSinceNodeEntry;

	/** Meaning depends on Condition -- a target average reward, or a step count. */
	UPROPERTY(EditAnywhere, Category = "Transition", meta = (ClampMin = "0.0"))
	float ThresholdValue = 1000.0f;

	/** The node this transition leads to. Resolved to a pointer via ULearningProgram::FindNode at evaluation time -- kept as a Guid (not a raw pointer) so the graph editor can rewrite it freely (dragging a pin link) without dangling any runtime state. */
	UPROPERTY()
	FGuid TargetNodeId;
};

/**
 * One curriculum stage. Params reuses UAgentSolverPreset wholesale (Instanced
 * -- so each node gets its own private preset object, editable inline in the
 * node's details) instead of duplicating its ~40 reward/rig/imitation fields
 * a second time; ApplyLearningProgramNode below just calls
 * Params->ApplyToDriver(Driver), same as loading a standalone preset.
 */
UCLASS(EditInlineNew, DefaultToInstanced, BlueprintType)
class AGENTSOLVER_API ULearningProgramNode : public UObject
{
	GENERATED_BODY()

public:
	ULearningProgramNode();

	/** Short label shown on the node in the graph editor -- purely cosmetic, does not affect training. */
	UPROPERTY(EditAnywhere, Category = "Learning Program Node")
	FString DisplayName;

	/**
	 * This stage's training parameters -- see UAgentSolverPreset for the full
	 * field list (torso reward, muscle malus, gravity, imitation weights,
	 * ...). Deliberately NOT marked Instanced: that specifier is what made
	 * the property editor hide the asset picker and show only "None" / a
	 * "create new instance" entry, with no way to browse to one of your
	 * existing saved UAgentSolverPreset assets -- Instanced means "privately
	 * owned data, always duplicated with its outer," which is exactly the
	 * opposite of "point this stage at one of my saved presets." Without it,
	 * the widget offers both: a normal asset picker (existing content
	 * browser assets), AND -- since UAgentSolverPreset is still
	 * EditInlineNew -- a "New" button plus inline expand/edit for whatever
	 * is currently assigned, including the private default instance
	 * CreateDefaultSubobject below creates before you ever touch this field.
	 */
	UPROPERTY(EditAnywhere, Category = "Learning Program Node")
	TObjectPtr<UAgentSolverPreset> Params;

	/**
	 * Conditions that advance training away from this node once met. See
	 * FLearningProgramTransition. EditFixedSize: entries are only ever
	 * added/removed via the graph editor's "Add Transition"/"Remove
	 * Transition" node actions (ULearningProgramGraphNode::AddTransitionPin/
	 * RemoveTransitionPin, AgentSolverEditor module), which keep this array
	 * and the node's output pins in lockstep -- editing the count from a
	 * details panel instead would desync the two. Condition/ThresholdValue on
	 * an existing entry are still freely editable here.
	 */
	UPROPERTY(EditAnywhere, EditFixedSize, Category = "Learning Program Node")
	TArray<FLearningProgramTransition> Transitions;

	/** Stable identity for this node, independent of its array index -- referenced by FLearningProgramTransition::TargetNodeId and ULearningProgram::EntryNodeId. Assigned once, on construction, and never reassigned. */
	UPROPERTY()
	FGuid NodeId;

	/** Graph-editor canvas position. Lives here (not on the editor-only graph node) so layout survives a recompile of the graph without any editor-only storage on ULearningProgram itself. */
	UPROPERTY()
	FVector2D EditorPosition = FVector2D::ZeroVector;
};

/**
 * The double-clickable asset the user builds a curriculum in. See this
 * file's top comment for the runtime/editor split.
 */
UCLASS(BlueprintType)
class AGENTSOLVER_API ULearningProgram : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Instanced, Category = "Learning Program")
	TArray<TObjectPtr<ULearningProgramNode>> Nodes;

	/** Which of Nodes training starts at. Must match one entry's NodeId -- see GetEntryNode(). */
	UPROPERTY(EditAnywhere, Category = "Learning Program")
	FGuid EntryNodeId;

	/** O(n) linear search -- Nodes is expected to stay small (a handful of curriculum stages), not indexed. Returns nullptr if NodeId is not found (e.g. a dangling TargetNodeId left over from a deleted node). */
	ULearningProgramNode* FindNode(const FGuid& NodeId) const;

	/** FindNode(EntryNodeId), or nullptr if EntryNodeId is unset/stale. */
	ULearningProgramNode* GetEntryNode() const;

#if WITH_EDITORONLY_DATA
	/**
	 * The visual graph FLearningProgramAssetEditorToolkit edits. Typed as the
	 * base UEdGraph (Engine module, available outside Editor targets) rather
	 * than the AgentSolverEditor module's own ULearningProgramEdGraph
	 * subclass, so this runtime-module header never needs to reference an
	 * Editor-only module -- same reasoning as UBlueprint::UbergraphPages.
	 */
	UPROPERTY()
	TObjectPtr<UEdGraph> EdGraph;
#endif
};
