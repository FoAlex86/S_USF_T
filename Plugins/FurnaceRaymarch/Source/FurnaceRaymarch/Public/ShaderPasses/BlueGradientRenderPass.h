#pragma once

#include "CoreMinimal.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "SceneTexturesConfig.h"
#include "PostProcess/PostProcessInputs.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "ShaderParameters.h"
#include "RenderGraphResources.h"
#include "SceneView.h"

BEGIN_SHADER_PARAMETER_STRUCT(FBlueGradientParams, )
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, SceneColorTexture)

	SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
	SHADER_PARAMETER_STRUCT_INCLUDE(FSceneTextureShaderParameters, SceneTextures)

	RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

class FBlueGradientPS : public FGlobalShader
{
	DECLARE_EXPORTED_SHADER_TYPE(FBlueGradientPS, Global, );
	using FParameters = FBlueGradientParams;

	SHADER_USE_PARAMETER_STRUCT(FBlueGradientPS, FGlobalShader);

public:
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		// pas de defines spécifiques
	}
};


