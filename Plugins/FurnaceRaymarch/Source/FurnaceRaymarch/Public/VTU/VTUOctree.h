#pragma once
#include "CoreMinimal.h"

// Élément indexé : AABB monde (cm) + index de cellule
struct FVTUCellProxy
{
    FBox   Bounds;
    int32  Cell = -1;
};

// Noeud d'octree (léger)
class FVTUOctreeNode
{
    public:
        explicit FVTUOctreeNode(const FBox& InBounds) : Bounds(InBounds) {}

        FBox Bounds;                           // boîte couverte par ce noeud
        TArray<int32> Elements;                // indices dans le tableau global 'Proxies'
        TUniquePtr<FVTUOctreeNode> Child[8];   // 8 enfants (si subdivisé)
        bool bLeaf = true;
};

// Octree complet
class FVTUCellOctree
{
    public:
        void Build(const TArray<FVTUCellProxy>& InProxies, const FBox& InWorldBounds, int32 InMaxPerLeaf = 32, int32 InMaxDepth = 10);
        void QueryAABB(const FBox& Box, TArray<int32>& OutCells) const;
        int32 NumElements() const { return Proxies.Num(); }
        FBox  GetWorldBounds() const { return WorldBounds; }
        bool  IsBuilt() const { return Root.IsValid(); }
        bool GetCellBounds(int32 Cell, FBox& Out) const;
        const FBox* BoundsOf(int32 Cell) const;
        // remettre l’octree à zéro
        void Clear();

        // compatibilité avec le code des clones ; simple wrapper + retour bool
        bool BuildFromProxies(const TArray<FVTUCellProxy>& InProxies, const FBox& InWorldBounds, int32 InMaxPerLeaf = 32, int32 InMaxDepth = 10)
        {
            Build(InProxies, InWorldBounds, InMaxPerLeaf, InMaxDepth);
            return IsBuilt();
        }
        bool FlattenForGPU( /*out*/ TArray<FVector3f>& OutNodeCenter, TArray<FVector3f>& OutNodeExtent, TArray<int32>& OutNodeFirstChild, TArray<uint32>& OutNodeChildCount, TArray<int32>& OutNodeFirstElem, TArray<uint32>& OutNodeElemCount, TArray<int32>& OutChildIndex,/* liste contiguë des enfants*/ TArray<FVector3f>& OutElemMin, TArray<FVector3f>& OutElemMax, TArray<int32>& OutElemCell, uint32& OutRootIndex) const;


    private:
        void Insert(int32 ElemIndex, FVTUOctreeNode* Node, int32 Depth);
        void Subdivide(FVTUOctreeNode* Node);
        static FBox ChildBoundsFor(const FBox& Parent, int32 ChildIdx);
        void QueryNodeAABB(const FVTUOctreeNode* Node, const FBox& Box, TArray<int32>& OutCells) const;

        TUniquePtr<FVTUOctreeNode> Root;
        TArray<FVTUCellProxy> Proxies;
        FBox   WorldBounds;
        int32  MaxPerLeaf = 32;
        int32  MaxDepth = 10;
        TArray<FBox> CellBounds;
};
