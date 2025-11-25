#include "ShaderPasses/VTURaymarchCS.h"
#include "SceneRenderTargetParameters.h"
#include "SceneTexturesConfig.h"

IMPLEMENT_SHADER_TYPE(, FVTURaymarchCS, TEXT("/FurnaceRaymarch/Private/VTURaymarch.usf"), TEXT("MainCS"), SF_Compute);


