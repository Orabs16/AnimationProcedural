#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class SMassMuscleExpandableSection : public SCompoundWidget
{
public:

    struct FContentSlot : public TSlotBase<FContentSlot>
    {
        SLATE_SLOT_BEGIN_ARGS(FContentSlot, TSlotBase<FContentSlot>)
            // add per-slot args here if needed, e.g:
            // SLATE_ARGUMENT(FMargin, Padding)
        SLATE_SLOT_END_ARGS()

        void Construct(const FChildren& SlotOwner, FSlotArguments&& InArgs)
        {
            TSlotBase<FContentSlot>::Construct(SlotOwner, MoveTemp(InArgs));
        }
    };

    
    static FContentSlot::FSlotArguments Slot()
    {
        return FContentSlot::FSlotArguments(MakeUnique<FContentSlot>());
    }

    SLATE_BEGIN_ARGS(SMassMuscleExpandableSection)
        : _InitiallyExpanded(true)
        {}
        SLATE_ARGUMENT(FText,                   Title)
        SLATE_ARGUMENT(bool,                    InitiallyExpanded)
        SLATE_SLOT_ARGUMENT(SMassMuscleExpandableSection::FContentSlot, Slots)
    SLATE_END_ARGS()


    SMassMuscleExpandableSection()
        : ContentSlots(this)    // TPanelChildren must be initialized with owner
    {}

    void Construct(const FArguments& InArgs);

private:

    FReply          OnHeaderClicked();
    EVisibility     GetContentVisibility() const;
    const FSlateBrush* GetArrowBrush() const;

    bool bIsExpanded = true;
    TPanelChildren<FContentSlot>    ContentSlots;
};