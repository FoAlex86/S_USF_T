// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "VTUCore.generated.h"


// Types de cellules VTK courants
UENUM()
enum class EVTKCellType : uint8 {
	VTK_EMPTY = 0, VTK_VERTEX = 1, VTK_LINE = 3, VTK_TRIANGLE = 5, VTK_QUAD = 9,
	VTK_TETRA = 10, VTK_HEXAHEDRON = 12, VTK_WEDGE = 13, VTK_PYRAMID = 14,
	VTK_POLYHEDRON = 42
};

// Conteneur brut VTU
USTRUCT()
struct FVTUGrid {
	GENERATED_BODY()
	TArray<FVector3f> Points;       // unités VTU 
	TArray<int32>     Connectivity; // concat indices de sommets par cellule
	TArray<int32>     Offsets;      // fin exclusive par cellule (taille = NumCells)
	TArray<uint8>     Types;        // type VTK par cellule

	TArray<int32>     Faces;
	TArray<int32>     FaceOffsets;
	FBox              Bounds = FBox(ForceInitToZero);
	int32 NumCells() const { return Types.Num(); }
	bool  HasPolyFaces() const { return Faces.Num() > 0 && FaceOffsets.Num() == Types.Num(); }
};

USTRUCT(BlueprintType)
struct FVTUCellFeatures
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> NOx;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> CO;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> OH;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> H2;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> H2O;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> CO2;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> O2;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> CH4;
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") TArray<float> T;

	void Reset()
	{
		NOx.Reset(); CO.Reset(); OH.Reset(); H2.Reset(); H2O.Reset();
		CO2.Reset(); O2.Reset(); CH4.Reset(); T.Reset();
	}

	void SetNum(int32 N)
	{
		NOx.SetNumUninitialized(N); CO.SetNumUninitialized(N); OH.SetNumUninitialized(N);
		H2.SetNumUninitialized(N);  H2O.SetNumUninitialized(N); CO2.SetNumUninitialized(N);
		O2.SetNumUninitialized(N);  CH4.SetNumUninitialized(N); T.SetNumUninitialized(N);
	}
};


// Namespace "core" (parsing + build surface)
namespace VTUCore
{
	// 1) Lecture .vtu (ASCII) -> FVTUGrid
	bool LoadVTU_ASCII(const FString& FilePath, FVTUGrid& Out);

	// 2) Construction de la surface (frontière) -> ProceduralMeshComponent
	//    ScaleCm = 100 si points en metres (UE en cm)
	bool BuildSurfaceToPMC(const FVTUGrid& G, UProceduralMeshComponent* PMC, float ScaleCm = 100.f);

	// 0) Optionnel: un helper
	void SummarizeProbe(const TCHAR* Tag, const FString& Line);

	// 1) Probe minimal: taille, drapeaux de format (sans parser tout)
	bool Probe_FileInfo(const FString& Path, int64& OutFileSizeBytes, bool& bHasUnstructuredGridTag, bool& bHasAsciiTag, FString& OutFirstKB);

	bool Probe_ParseHeaders_Light(const FString& Path, int32& OutNumPoints, int32& OutNumCells, bool& bHasConnectivity, bool& bHasOffsets, bool& bHasTypes, bool& bHasFaces, bool& bHasFaceOffsets, float& OutSeconds);
	// 2) Probe headers: parse complet mais SANS construire de mesh
	bool Probe_ParseHeaders_ASCII(const FString& Path, int32& OutNumPoints, int32& OutNumCells, int32& OutConnCount, int32& OutFacesCount, int32& OutFaceOffsetsCount, bool& bHasPolyFaces, float& OutSeconds);

	// 3) Probe “sample”: parse d’un petit échantillon pour estimer le coût
	bool Probe_ParseSamples_ASCII(const FString& Path, int32 MaxPointTokens, int32 MaxConnTokens, int32& OutParsedPointTokens, int32& OutParsedConnTokens, float& OutSecondsPoints, float& OutSecondsConn);

	// Lit offsets (Int64 ASCII) en streaming, sans charger tout le XML.
	// Renvoie le dernier offset (=> longueur de connectivity).
	bool ReadOffsetsLast_ASCII(const FString& Path, int32 NumCells, int64& OutLastOffset, float& OutSeconds);

	// Retourne les infos clés en 1 appel : counts + length(connectivity)
	bool Probe_ScanCountsAndOffsets(const FString& Path, int32& OutNumPoints, int32& OutNumCells, bool& bHasConnectivity, bool& bHasOffsets, bool& bHasTypes, bool& bHasFaces, bool& bHasFaceOffsets, int64& OutConnectivityLength, float& OutSecsHeaders, float& OutSecsOffsets);

	// Charge un .vtu ASCII en streaming (Points/Offsets/Types/Connectivity [+Faces si polyhedra]) sans surcharge mémoire.
	bool LoadVTU_ASCII_Streaming(const FString& FilePath, FVTUGrid& Out, bool bLoadFacesIfPoly = true);

	// Sauvegarde/lecture d’un FVTUGrid en binaire (format interne, versionné)
	bool SaveGridBinary(const FString& Path, const FVTUGrid& G);
	bool LoadGridBinary(const FString& Path, FVTUGrid& Out);

	// Charge un .npy (double <f8> shape=(9*NumCells, 1|None)) et remplit les 9 tableaux de FVTUCellFeatures.
	// Retourne false si shape/dtype non conformes ou taille incompatible avec NumCells.
	bool LoadNPY_CellFeatures(const FString& NpyPath, int32 NumCellsExpected, FVTUCellFeatures& Out);

	// Applique un des 9 champs de FVTUCellFeatures comme scalaires sur la surface (section SectionIndex) du PMC.
	bool BuildSurfaceToPMC_WithCellScalars(const FVTUGrid& G, UProceduralMeshComponent* PMC, float ScaleCm, const TArray<float>& CellScalars, float VMin, float VMax, int32 SectionIndex);
}