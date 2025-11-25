// CustomShaderSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "VTU/VTUCore.h"
#include "VTU/VTUOctree.h"
#include "Subsystems/EngineSubsystem.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Rendering/CustomSceneViewExtension.h"
#include "Engine/TextureRenderTarget2D.h"
#include "CustomShaderSubsystem.generated.h"

UCLASS()
class FURNACERAYMARCH_API UCustomShaderSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

private:
	TSharedPtr<class FCustomSceneViewExtension, ESPMode::ThreadSafe> CustomSceneViewExtension;
	
	UPROPERTY()                 // (garbage collector)
	UTextureRenderTarget2D* RaymarchRT = nullptr;

	int32 DebugDrawMode_GT = 0; // côté GameThread, pour le Cycle

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	UFUNCTION(BlueprintCallable, Category = "FurnaceRaymarch")
	void SetRaymarchTargetRT(UTextureRenderTarget2D* RT);

	//lier la RT dans un matériau / blueprint
	UFUNCTION(BlueprintPure, Category = "FurnaceRaymarch")
	UTextureRenderTarget2D * GetRaymarchRT() const { return RaymarchRT; }

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void ClearVTUOnGPU();

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetRaymarchVolumeTransform(const FTransform& ActorToWorld);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	/*void SetRaymarchBoundsWS(const FVector& MinWS, const FVector& MaxWS);*/
	void SetRaymarchBoundsLS(const FVector& MinLS, const FVector& MaxLS);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetWorldToLocal(const FMatrix& W2L);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	bool UploadFeatureVals(const TArray<float>& Values);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetFeatureWindow(float InVMin, float InVMax);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetRaymarchEpsilon(float InEpsCm);

	// --- C++ pur (prend le vrai FVTUGrid) ---
	bool UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm = 1.f);

	bool UploadVTUOctree(const FVTUCellOctree& Octree);

	// Set explicite depuis Blueprint (0..2)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void SetDebugDrawMode(int32 Mode);

	// Cycle 0 -> 1 -> 2 -> 0
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void CycleDebugDrawMode();

	//// Option: fixe l’AABB de debug (monde) utilisé par l’USF pour l’overlay/UVW
	//UFUNCTION(BlueprintCallable, Category = "VTU")
	//void SetDebugBoundsWS(const FVector& MinWS, const FVector& MaxWS);
};
