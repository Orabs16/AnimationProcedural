#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/SPanel.h"

class SMassMuscleItemRow : public SCompoundWidget
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
    
    SLATE_BEGIN_ARGS(SMassMuscleItemRow){}
        SLATE_SLOT_ARGUMENT(SMassMuscleItemRow::FContentSlot, Slots)
    SLATE_END_ARGS()
    
    SMassMuscleItemRow()
        : ContentSlots(this)    // TPanelChildren must be initialized with owner
    {}

    void Construct(const FArguments& InArgs);

private:

    TPanelChildren<FContentSlot>    ContentSlots;
};