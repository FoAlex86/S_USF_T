// FurnaceRaymarch.cpp
#include "FurnaceRaymarch.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FFurnaceRaymarchModule"

void FFurnaceRaymarchModule::StartupModule()
{
	// Mapping du répertoire Shaders (plugin root /Shaders) vers /FurnaceRaymarch
	const FString PluginShaderDir = FPaths::Combine(
		IPluginManager::Get().FindPlugin(TEXT("FurnaceRaymarch"))->GetBaseDir(),
		TEXT("Shaders")
	);

	if (!AllShaderSourceDirectoryMappings().Contains(TEXT("/FurnaceRaymarch")))
	{
		AddShaderSourceDirectoryMapping(TEXT("/FurnaceRaymarch"), PluginShaderDir);
	}

	// NOTE : pas de création de FSceneViewExtension ici.
	// Elle est créée par UCustomShaderSubsystem (EngineSubsystem) au bon moment.
}

void FFurnaceRaymarchModule::ShutdownModule()
{
	// Rien de spécial ici (Subsystem gère la ViewExtension)
}

#undef LOCTEXT_NAMESPACE
IMPLEMENT_MODULE(FFurnaceRaymarchModule, FurnaceRaymarch)

