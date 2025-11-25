#include "VTU/VTUOctree.h"

void FVTUCellOctree::Clear()
{
    Root.Reset();
    Proxies.Reset();
    WorldBounds = FBox(ForceInitToZero);
    CellBounds.Reset();
}

void FVTUCellOctree::Build(const TArray<FVTUCellProxy>& InProxies, const FBox& InWorldBounds, int32 InMaxPerLeaf, int32 InMaxDepth)
{
    Clear();

    Proxies = InProxies;
    WorldBounds = InWorldBounds;
    MaxPerLeaf = FMath::Max(1, InMaxPerLeaf);
    MaxDepth = FMath::Clamp(InMaxDepth, 1, 24);

    // table directe Cell -> Bounds pour accès O(1)
    int32 MaxCell = -1;
    for (const auto& P : Proxies) MaxCell = FMath::Max(MaxCell, P.Cell);
    CellBounds.SetNum(MaxCell + 1);          // (ajoute TArray<FBox> CellBounds dans la classe)
    for (int32 i = 0; i < CellBounds.Num(); ++i) CellBounds[i] = FBox(ForceInitToZero);

    Root = TUniquePtr<FVTUOctreeNode>(new FVTUOctreeNode(WorldBounds));
    for (int32 i = 0; i < Proxies.Num(); ++i)
    {
        if (Proxies[i].Bounds.IsValid) {
            Insert(i, Root.Get(), 0);
            //  mémorise l’AABB de la cellule
            const int32 c = Proxies[i].Cell;
            if (CellBounds.IsValidIndex(c)) CellBounds[c] = Proxies[i].Bounds;
        }
    }
}

const FBox* FVTUCellOctree::BoundsOf(int32 Cell) const
{
    return (CellBounds.IsValidIndex(Cell) && CellBounds[Cell].IsValid) ? &CellBounds[Cell] : nullptr;
}

//  profite de la LUT, sinon fallback (sécuritée)
bool FVTUCellOctree::GetCellBounds(int32 Cell, FBox& Out) const
{
    if (CellBounds.IsValidIndex(Cell) && CellBounds[Cell].IsValid)
    {
        Out = CellBounds[Cell];
        return true;
    }
    // fallback 
    for (const FVTUCellProxy& P : Proxies)
    {
        if (P.Cell == Cell) { Out = P.Bounds; return true; }
    }
    return false;
}

void FVTUCellOctree::Insert(int32 ElemIndex, FVTUOctreeNode* Node, int32 Depth)
{
    // Si on est en feuille et sous le seuil, stocke ici
    if (Node->bLeaf && (Node->Elements.Num() < MaxPerLeaf || Depth >= MaxDepth)) {
        Node->Elements.Add(ElemIndex);
        return;
    }

    // Sinon, subdivise si pas déjà fait
    if (Node->bLeaf) {
        Subdivide(Node);
    }

    const FBox& EB = Proxies[ElemIndex].Bounds;
    bool bPushedToChild = false;

    // Essaie de pousser dans un enfant si l'élément est entièrement contenu dans UNE boîte enfant
    for (int32 c = 0; c < 8; ++c)
    {
        const FBox CB = ChildBoundsFor(Node->Bounds, c);
        if (CB.IsInside(EB)) {
            Insert(ElemIndex, Node->Child[c].Get(), Depth + 1);
            bPushedToChild = true;
            break;
        }
    }

    // Sinon, stocke au niveau courant (évite duplication quand l'AABB chevauche plusieurs enfants)
    if (!bPushedToChild) {
        Node->Elements.Add(ElemIndex);
    }
}

void FVTUCellOctree::Subdivide(FVTUOctreeNode* Node)
{
    Node->bLeaf = false;
    for (int32 i = 0; i < 8; ++i) {
        Node->Child[i] = MakeUnique<FVTUOctreeNode>(ChildBoundsFor(Node->Bounds, i));
    }

    // Redispatch les éléments existants si possible (sinon on les garde dans ce noeud)
    TArray<int32> Old = MoveTemp(Node->Elements);
    Node->Elements.Reset();

    for (int32 idx : Old)
    {
        const FBox& EB = Proxies[idx].Bounds;
        bool bPushed = false;
        for (int32 c = 0; c < 8; ++c) {
            const FBox CB = ChildBoundsFor(Node->Bounds, c);
            if (CB.IsInside(EB)) {
                Insert(idx, Node->Child[c].Get(), /*Depth dummy*/ 1024); // on ne redescend pas profond inutilement
                bPushed = true;
                break;
            }
        }
        if (!bPushed) {
            Node->Elements.Add(idx);
        }
    }
}

FBox FVTUCellOctree::ChildBoundsFor(const FBox& Parent, int32 ChildIdx)
{
    const FVector C = Parent.GetCenter();
    const FVector E = Parent.GetExtent();

    const bool px = (ChildIdx & 1) != 0;
    const bool py = (ChildIdx & 2) != 0;
    const bool pz = (ChildIdx & 4) != 0;

    const FVector Min(
        px ? C.X : Parent.Min.X,
        py ? C.Y : Parent.Min.Y,
        pz ? C.Z : Parent.Min.Z
    );
    const FVector Max(
        px ? Parent.Max.X : C.X,
        py ? Parent.Max.Y : C.Y,
        pz ? Parent.Max.Z : C.Z
    );
    return FBox(Min, Max);
}

void FVTUCellOctree::QueryAABB(const FBox& Box, TArray<int32>& OutCells) const
{
    OutCells.Reset();
    if (!Root.IsValid() || !Box.IsValid) return;
    QueryNodeAABB(Root.Get(), Box, OutCells);
}

void FVTUCellOctree::QueryNodeAABB(const FVTUOctreeNode* Node, const FBox& Box, TArray<int32>& OutCells) const
{
    if (!Node || !Node->Bounds.Intersect(Box)) return;

    // Collecte les éléments du noeud courant qui intersectent Box
    for (int32 idx : Node->Elements)
    {
        if (Proxies[idx].Bounds.Intersect(Box))
            OutCells.Add(Proxies[idx].Cell);
    }

    if (!Node->bLeaf)
    {
        for (int32 c = 0; c < 8; ++c)
            QueryNodeAABB(Node->Child[c].Get(), Box, OutCells);
    }
}

bool FVTUCellOctree::FlattenForGPU( TArray<FVector3f>& OutNodeCenter, TArray<FVector3f>& OutNodeExtent, TArray<int32>& OutNodeFirstChild, TArray<uint32>& OutNodeChildCount, TArray<int32>& OutNodeFirstElem, TArray<uint32>& OutNodeElemCount, TArray<int32>& OutChildIndex, TArray<FVector3f>& OutElemMin, TArray<FVector3f>& OutElemMax, TArray<int32>& OutElemCell, uint32& OutRootIndex) const
{
    if (!IsBuilt() || !Root.IsValid())
        return false;

    OutNodeCenter.Reset();
    OutNodeExtent.Reset();
    OutNodeFirstChild.Reset();
    OutNodeChildCount.Reset();
    OutNodeFirstElem.Reset();
    OutNodeElemCount.Reset();
    OutChildIndex.Reset();
    OutElemMin.Reset();
    OutElemMax.Reset();
    OutElemCell.Reset();

    // 1) numérote tous les noeuds (BFS) et stocke des pointeurs
    TArray<const FVTUOctreeNode*> NodePtrs;
    TMap<const FVTUOctreeNode*, int32> NodeIndex;
    {
        TArray<const FVTUOctreeNode*> Q;
        Q.Add(Root.Get());
        while (Q.Num() > 0)
        {
            const FVTUOctreeNode* N = Q[0]; Q.RemoveAt(0, 1, false);
            int32 NewIdx = NodePtrs.Add(N);
            NodeIndex.Add(N, NewIdx);

            for (int32 i = 0; i < 8; ++i)
            {
                if (N->Child[i].IsValid())
                    Q.Add(N->Child[i].Get());
            }
        }
    }

    const int32 N = NodePtrs.Num();
    if (N <= 0) return false;

    OutNodeCenter.SetNum(N);
    OutNodeExtent.SetNum(N);
    OutNodeFirstChild.SetNum(N);
    OutNodeChildCount.SetNum(N);
    OutNodeFirstElem.SetNum(N);
    OutNodeElemCount.SetNum(N);

    // 2) pour chaque noeud: bounds, enfants, éléments
    for (int32 ni = 0; ni < N; ++ni)
    {
        const FVTUOctreeNode* Np = NodePtrs[ni];
        const FBox& B = Np->Bounds;
        const FVector3f C = (FVector3f)B.GetCenter();
        const FVector3f E = (FVector3f)B.GetExtent();

        OutNodeCenter[ni] = C;
        OutNodeExtent[ni] = E;

        // enfants -> OutChildIndex
        int32 firstChild = -1;
        uint32 childCount = 0;
        for (int32 i = 0; i < 8; ++i)
        {
            if (Np->Child[i].IsValid())
            {
                const int32 cidx = NodeIndex[Np->Child[i].Get()];
                if (firstChild < 0) firstChild = OutChildIndex.Num();
                OutChildIndex.Add(cidx);
                ++childCount;
            }
        }
        OutNodeFirstChild[ni] = firstChild;
        OutNodeChildCount[ni] = childCount;

        // éléments (cellules) -> OutElem*
        int32 firstElem = -1;
        uint32 elemCount = 0;
        if (Np->Elements.Num() > 0)
        {
            firstElem = OutElemCell.Num();
            for (int32 cell : Np->Elements)
            {
                FBox CB;
                if (!GetCellBounds(cell, CB))
                {
                    // fallback si pas de bounds par cellule
                    const FBox* P = BoundsOf(cell);
                    CB = P ? *P : FBox(B.Min, B.Max);
                }
                OutElemMin.Add((FVector3f)CB.Min);
                OutElemMax.Add((FVector3f)CB.Max);
                OutElemCell.Add(cell);
                ++elemCount;
            }
        }
        OutNodeFirstElem[ni] = firstElem;
        OutNodeElemCount[ni] = elemCount;
    }

    OutRootIndex = NodeIndex[Root.Get()];
    return true;
}
