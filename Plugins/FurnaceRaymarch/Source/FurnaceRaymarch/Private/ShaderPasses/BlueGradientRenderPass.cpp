// BlueGradientRenderPass.cpp
#include "ShaderPasses/BlueGradientRenderPass.h"

#include "SceneRenderTargetParameters.h"
#include "SceneTexturesConfig.h"

// Unique .usf localisé à: Plugins/FurnaceRaymarch/Shaders/Private/BlueGradient.usf
IMPLEMENT_SHADER_TYPE(, FBlueGradientPS, TEXT("/FurnaceRaymarch/Private/BlueGradient.usf"), TEXT("MainPS"), SF_Pixel);

