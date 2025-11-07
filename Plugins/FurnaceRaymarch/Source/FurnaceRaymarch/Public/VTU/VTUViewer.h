#pragma once

#include "CoreMinimal.h"
#include "VTU/VTUOctree.h"
#include "GameFramework/Actor.h"
#include "VTU/VTUCore.h"
#include "VTUViewer.generated.h"


UENUM(BlueprintType)
enum class EVTUCellFeature : uint8
{
	NOx, CO, OH, H2, H2O, CO2, O2, CH4, T
};

// --- Un clone complet : points transformés (LOCAL cm), bbox, et son octree ---
USTRUCT()
struct FVTUGridClone
{
	GENERATED_BODY()

	// Points transformés (déjà en LOCAL cm)
	TArray<FVector3f> PointsLS;

	// BBox de tout le clone (LOCAL cm)
	FBox BoundsLS = FBox(ForceInitToZero);

	// Octree des cellules du clone (chaque élément = index de cellule du 1/8)
	TSharedPtr<FVTUCellOctree> Octree;

	// Mnémonique (debug)
	FString Label;
};

UCLASS()
class AVTUViewer : public AActor
{
	GENERATED_BODY()
public:
	UPROPERTY(VisibleAnywhere) UProceduralMeshComponent* PMC;

	FVTUGrid GridCache;
	UPROPERTY(EditAnywhere, Category = "VTU") float ScaleCm = 100.f;

	TArray<float> PointScalarsCache; // size = Points.Num()
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features")
	FVTUCellFeatures CellFeatures;  // 9 tableaux alignés aux cellules
	//--------------------------------Test---------------------------------
	// Activer/désactiver l’usage des clones
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Clones")
	bool bUseClonedOctrees = false;

	// Tous les clones (Id, FlipX, FlipY, FlipXY, SwapXY * leurs flips) — jusqu’à 8
	TArray<FVTUGridClone> GridClones;

	// Pivot LOCAL (cm) des plans X=0 / Y=0 (souvent (0,0,0) chez toi)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Clones")
	FVector ClonePivotLS = FVector::ZeroVector;
	//--------------------------------Test---------------------------------
	AVTUViewer();
	//--------------------------------Test----------------------------------
	// --- Mémo du dernier hit (alimenté par RaymarchDebug) ---
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	int32 LastHitCloneIndex = -999;   // -1 = base (1/8), >=0 = index clone, -999 = aucun

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	int32 LastHitCellIndex = -1;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	FVector LastHitPointWS = FVector::ZeroVector;
	//--------------------------------Test----------------------------------

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_FileInfo(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseHeaders(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseHeaders_Light(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseSamples(const FString& Path, int32 MaxPointTokens = 30000, int32 MaxConnTokens = 60000);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ReadOffsetsLast(const FString& Path, int32 NumCells);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ScanCountsAndOffsets(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	bool LoadAndShowVTU_ASCII_Streaming(const FString& AbsolutePath);

	UFUNCTION(BlueprintCallable, Category = "VTU")
	bool LoadOrParseVTUAndShow(const FString& VtuPath, const FString& CacheFilePath);

	UFUNCTION(BlueprintCallable, Category = "VTU|Features")
	bool LoadPredictedVectorNPY(const FString& NpyPath);

	UFUNCTION(BlueprintCallable, Category = "VTU|Features")
	bool ApplyFeatureToSurface(EVTUCellFeature Feature, float VMin, float VMax, int32 SectionIndex = 0);

	UFUNCTION(BlueprintCallable, Category = "VTU|Materials")
	bool SetSurfaceMaterial(UMaterialInterface* Material, int32 SectionIndex = -1); // -1 => toutes les sections utiles

	UFUNCTION(BlueprintCallable, Category = "VTU|Materials")
	bool SetSurfaceMaterialByPath(const FString& AssetPath, int32 SectionIndex = -1); // ex: "/Game/MeshProcess/M_VTU_TwoSided.M_VTU_TwoSided"

	UFUNCTION(BlueprintCallable, Category = "VTU|Octree")
	bool BuildCellOctree();  // à appeler après LoadOrParseVTUAndShow

	UFUNCTION(BlueprintCallable, Category = "VTU|Octree")
	void ClearCellOctree();
	// Raymarch CPU de debug (retourne une couleur pour un rayon)
	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	FLinearColor RaymarchDebug(EVTUCellFeature Feature, const FVector& RayOriginWS, const FVector& RayDirWS, int32& OutCellIndex, float StepCm, /*int32 MaxSteps, float Density,*/ float VMin, float VMax, float EpsCm, bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float ClipThicknessCm, bool bKeepFront); // VMin/VMax = 0 => autoscale

	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	UTexture2D* RaymarchToTexture2D(int32 Width, int32 Height, EVTUCellFeature Feature, float StepCm, float VMin, float VMax, float EpsCm, bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float ClipThicknessCm, bool bKeepFront, bool bCropToOBB = true, bool bClearTransparent = true);

	/*UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawWorldBounds(float Duration = 2.f, float Thickness = 3.f);*/

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawWorldBounds(float Duration = 2.f, float Thickness = 3.f, FLinearColor Color = FLinearColor(0.f, 255.f, 255.f));

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawCellBounds(int32 CellIndex, float Duration = 2.f, float Thickness = 1.f, FLinearColor Color = FLinearColor(255.f, 0.f, 255.f));

	//--------------------------------Test---------------------------------
	// API
	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	void SetClonePivotLocal(const FVector& PivotLS);

	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	void ClearClonedOctrees();

	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	// Construit 8 clones (Id, FlipX, FlipY, FlipXY, SwapXY + leurs flips)
	bool BuildClonedOctrees_XY(bool bIncludeIdentity = true,
		int32 MaxPerLeaf = 64, int32 MaxDepth = 12);

	bool BuildOneClone_XY(bool bFlipX, bool bFlipY, bool bSwapXY,
		const FString& Label,
		int32 MaxPerLeaf, int32 MaxDepth,
		FVTUGridClone& OutClone);

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawWorldBoundsAll(bool bIncludeBase = true, float Duration = 3.f, float Thickness = 2.f);

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawCellBoundsAny(int32 CellIndex, bool bIncludeBase = true, float Duration = 5.f, float Thickness = 2.f);

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawCellBoundsInClone(int32 CloneIndex, int32 CellIndex, float Duration = 5.f, float Thickness = 2.f);

	// --- Outils debug ciblés ---
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawHitWorldBounds(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Yellow) const;

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawHitCellBounds(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Blue) const;

	// Bounds “actifs” du volume pour le rendu (union clones si actifs)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	FBox GetActiveVolumeBoundsLS() const;

	// Dessine l’OBB du volume actif (union des clones si actifs, sinon le 1/8)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawActiveVolumeOBB(float Duration = 5.f,
		float Thickness = 2.f,
		FLinearColor Color = FLinearColor::Red) const;

	// Dessine l’OBB du clone i (0..GridClones.Num()-1)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawCloneBoundsOBB(int32 CloneIndex,
		float Duration = 5.f,
		float Thickness = 2.f,
		FLinearColor Color = FLinearColor::Yellow) const;

	// (optionnel) Dessine l’OBB du 1/8 de base
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawBaseBoundsOBB(float Duration = 5.f,
		float Thickness = 2.f,
		FLinearColor Color = FLinearColor::White) const;

	//----------------cube
	// == Proxy cube pour afficher la texture de raymarch ==
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Proxy")
	UProceduralMeshComponent* RayVolumePMC = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	UMaterialInterface* RayVolumeMaterial = nullptr; // ex: M_RayVolume (à créer)

	UPROPERTY(Transient)
	UMaterialInstanceDynamic* RayVolumeMID = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultDepthEpsCm = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultOpacity = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultGain = 1.0f;

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SetRayVolumeBaseMaterial(UMaterialInterface* Mat);

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SpawnRayVolumeProxy(bool bTwoSided = true);

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void DestroyRayVolumeProxy();

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SetRayVolumeTexture(UTexture2D* RayTex, float Opacity = 1.0f, float Gain = 1.0f);

	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	bool RaycastFirstCell(EVTUCellFeature Feature,
		const FVector& RayOriginWS,
		const FVector& RayDirWS,
		int32& OutCellIndex,
		FVector& OutHitWS,
		FLinearColor& OutColor,
		float StepCm,
		float VMin, float VMax, float EpsCm,
		bool  bEnableClip,
		FVector ClipOriginWS, FVector ClipNormalWS,
		float  ClipThicknessCm, bool bKeepFront);

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SetRayVolumeParams(/*float InMaxDepthCm, float InDepthEpsCm = 1.0f,*/
		float InOpacity = 1.0f, float InGain = 1.0f);

	//--------------------------------Test---------------------------------
	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void PushProxyBoundsToGPU_FromPMC();

	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void BindRaymarchRTToMID();

	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void PushGridToGPU();

	private:
	TUniquePtr<FVTUCellOctree> CellOctree;
};