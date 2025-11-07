// CustomShaderSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "VTU/VTUCore.h"
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

	UFUNCTION(BlueprintCallable, Category = "FurnaceRaymarch")
	void UploadDummyVTU();

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetRaymarchVolumeTransform(const FTransform& ActorToWorld);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	void SetRaymarchBoundsWS(const FVector& MinWS, const FVector& MaxWS);

	// --- C++ pur (prend le vrai FVTUGrid) ---
	bool UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm = 1.f);

	//// Option: fixe l’AABB de debug (monde) utilisé par l’USF pour l’overlay/UVW
	//UFUNCTION(BlueprintCallable, Category = "VTU")
	//void SetDebugBoundsWS(const FVector& MinWS, const FVector& MaxWS);
};
