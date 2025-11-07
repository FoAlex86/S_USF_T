// FurnaceRaymarch.Build.cs
using System.IO;
using EpicGames.Core;
using UnrealBuildTool;
using static EpicGames.Core.JsonObject;

public class FurnaceRaymarch : ModuleRules
{
    public FurnaceRaymarch(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(
            new string[] {
                // Accès aux headers privés du Renderer, conforme au modèle
                Path.Combine(GetModuleDirectory("Renderer"), "Private"),
            }
        );

        PrivateIncludePaths.AddRange(
            new string[] {
                // ...
            }
        );

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "Engine",
                "RenderCore",
                "RHI",
                "ProceduralMeshComponent"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "Projects",
                "RHI",
                "Renderer",
                "RenderCore"
            }
        );

        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
                // ...
            }
        );
    }
}
