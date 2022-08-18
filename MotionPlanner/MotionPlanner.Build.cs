using UnrealBuildTool;

public class MotionPlanner: ModuleRules
{
    public MotionPlanner(ReadOnlyTargetRules Target) : base(Target) 
    {
        PrivateDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject", "Engine"}); 
    }
}