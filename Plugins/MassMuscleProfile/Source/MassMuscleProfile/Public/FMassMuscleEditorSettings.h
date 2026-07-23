#pragma once
#include "CoreMinimal.h"


DECLARE_MULTICAST_DELEGATE(FOnHierarchyExtend);
DECLARE_MULTICAST_DELEGATE(FOnHierarchyCollapse);
class FMassMuscleEditorSettings
{
public:
    bool XRaySkeleton = true;
    bool XRayMuscles = false;
    void SetHierarchy(bool value){
        HierarchyIsExtended = value;
        if(value) OnHierarchyExtend.Broadcast();
        else OnHierarchyCollapse.Broadcast();
    }
    FOnHierarchyCollapse OnHierarchyCollapse;
    FOnHierarchyExtend OnHierarchyExtend;
private:
    bool HierarchyIsExtended = true;
};