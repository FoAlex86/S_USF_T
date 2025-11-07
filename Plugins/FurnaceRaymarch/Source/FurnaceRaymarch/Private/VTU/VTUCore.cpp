#include "VTU/VTUCore.h"
#include "HAL/PlatformFilemanager.h"

// ---------- helpers parsing ----------
static void SplitWS(const FString& S, TArray<FString>& Out) {
    Out.Reset(); Out.Reserve(S.Len() / 3);
    const TCHAR* p = *S;
    while (*p) {
        while (*p && FChar::IsWhitespace(*p)) ++p;
        if (!*p) break;
        const TCHAR* b = p;
        while (*p && !FChar::IsWhitespace(*p)) ++p;
        Out.Emplace(b, p - b);
    }
}
static bool ParseFloats(const FString& Body, TArray<float>& Out) {
    TArray<FString> T; SplitWS(Body, T); Out.Reset(T.Num());
    for (const FString& x : T) Out.Add(FCString::Atof(*x));
    return Out.Num() > 0;
}
static bool ParseInt32s(const FString& Body, TArray<int32>& Out) {
    TArray<FString> T; SplitWS(Body, T); Out.Reset(T.Num());
    for (const FString& x : T) Out.Add(FCString::Atoi(*x));
    return Out.Num() > 0;
}
static bool ParseInt64s(const FString& Body, TArray<int64>& Out) {
    TArray<FString> T; SplitWS(Body, T); Out.Reset(T.Num());
    for (const FString& x : T) Out.Add(FCString::Atoi64(*x));
    return Out.Num() > 0;
}

// extrait <Tag>...</Tag>
static bool ExtractTagBody(const FString& Xml, const FString& Tag, int32 from, FString& out, int32& endPos) {
    const FString open = TEXT("<") + Tag;
    const FString close = TEXT("</") + Tag + TEXT(">");
    int32 p = Xml.Find(open, ESearchCase::IgnoreCase, ESearchDir::FromStart, from);
    if (p < 0) return false;
    int32 gt = Xml.Find(TEXT(">"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
    if (gt < 0) return false;
    int32 q = Xml.Find(close, ESearchCase::IgnoreCase, ESearchDir::FromStart, gt + 1);
    if (q < 0) return false;
    out = Xml.Mid(gt + 1, q - (gt + 1));
    endPos = q + close.Len();
    return true;
}

// récupère le 1er <DataArray ... format="ascii">...</DataArray> (option Name=...)
// renvoie aussi l'entête 'head' pour lire type="..."
static bool ExtractDataArrayASCII(const FString& Block, FString& outBody, FString& outHead, const FString& withName = TEXT("")) {
    int32 pos = 0;
    while (true) {
        int32 da = Block.Find(TEXT("<DataArray"), ESearchCase::IgnoreCase, ESearchDir::FromStart, pos);
        if (da < 0) break;
        int32 gt = Block.Find(TEXT(">"), ESearchCase::IgnoreCase, ESearchDir::FromStart, da);
        if (gt < 0) break;
        FString head = Block.Mid(da, gt - da);
        const bool ascii =
            head.Contains(TEXT("format=\"ascii\"")) || head.Contains(TEXT("format='ascii'"));
        const bool nameOk = withName.IsEmpty()
            || head.Contains(TEXT("Name=\"") + withName + TEXT("\""))
            || head.Contains(TEXT("Name='") + withName + TEXT("'"));
        int32 end = Block.Find(TEXT("</DataArray>"), ESearchCase::IgnoreCase, ESearchDir::FromStart, gt + 1);
        if (end < 0) break;
        if (ascii && nameOk) {
            outHead = head;
            outBody = Block.Mid(gt + 1, end - (gt + 1));
            return true;
        }
        pos = end + 12;
    }
    return false;
}

static bool HeadIs(const FString& Head, const TCHAR* TypeName) {
    return Head.Contains(FString::Printf(TEXT("type=\"%s\""), TypeName), ESearchCase::IgnoreCase)
        || Head.Contains(FString::Printf(TEXT("type='%s'"), TypeName), ESearchCase::IgnoreCase);
}

// ---------- loader ASCII ----------
bool VTUCore::LoadVTU_ASCII(const FString& FilePath, FVTUGrid& Out)
{
    Out = FVTUGrid();

    FString Xml;
    if (!FFileHelper::LoadFileToString(Xml, *FilePath)) return false;
    if (!Xml.Contains(TEXT("<VTKFile")) || !Xml.Contains(TEXT("<UnstructuredGrid"))) return false;

    // <Piece>
    FString Piece; int32 afterPiece = 0;
    if (!ExtractTagBody(Xml, TEXT("Piece"), 0, Piece, afterPiece)) return false;

    // ---- Points (Float32) ----
    {
        FString PointsBlock; int32 afterPoints = 0;
        if (!ExtractTagBody(Piece, TEXT("Points"), 0, PointsBlock, afterPoints)) return false;

        FString head, body;
        if (!ExtractDataArrayASCII(PointsBlock, body, head)) return false;
        if (!HeadIs(head, TEXT("Float32")) && !HeadIs(head, TEXT("Float64"))) {
            // accepte Float64 mais cast en float
        }
        TArray<float> flat;
        if (!ParseFloats(body, flat) || flat.Num() % 3 != 0) return false;
        const int32 N = flat.Num() / 3;
        Out.Points.SetNumUninitialized(N);
        FBox box(ForceInitToZero);
        for (int32 i = 0; i < N; ++i) {
            FVector3f p(flat[3 * i + 0], flat[3 * i + 1], flat[3 * i + 2]);
            Out.Points[i] = p; box += FVector(p);
        }
        Out.Bounds = box;
    }

    // ---- Cells ----
    FString CellsBlock; int32 afterCells = 0;
    if (!ExtractTagBody(Piece, TEXT("Cells"), 0, CellsBlock, afterCells)) return false;

    // connectivity (Int64), offsets (Int64), types (UInt8), faces(faceoffsets) (Int64) optionnels
    FString hConn, bConn; if (!ExtractDataArrayASCII(CellsBlock, bConn, hConn, TEXT("connectivity"))) return false;
    FString hOffs, bOffs; if (!ExtractDataArrayASCII(CellsBlock, bOffs, hOffs, TEXT("offsets")))      return false;
    FString hType, bType; if (!ExtractDataArrayASCII(CellsBlock, bType, hType, TEXT("types")))        return false;

    // CONNECTIVITY
    if (HeadIs(hConn, TEXT("Int64"))) {
        TArray<int64> tmp64; if (!ParseInt64s(bConn, tmp64)) return false;
        Out.Connectivity.SetNumUninitialized(tmp64.Num());
        for (int32 i = 0; i < tmp64.Num(); ++i) {
            if (tmp64[i] < 0 || tmp64[i] > INT32_MAX) { UE_LOG(LogTemp, Error, TEXT("VTU: connectivity > INT32_MAX")); return false; }
            Out.Connectivity[i] = (int32)tmp64[i];
        }
    }
    else {
        if (!ParseInt32s(bConn, Out.Connectivity)) return false;
    }

    // OFFSETS
    if (HeadIs(hOffs, TEXT("Int64"))) {
        TArray<int64> tmp64; if (!ParseInt64s(bOffs, tmp64)) return false;
        Out.Offsets.SetNumUninitialized(tmp64.Num());
        for (int32 i = 0; i < tmp64.Num(); ++i) {
            if (tmp64[i] < 0 || tmp64[i] > INT32_MAX) { UE_LOG(LogTemp, Error, TEXT("VTU: offset > INT32_MAX")); return false; }
            Out.Offsets[i] = (int32)tmp64[i];
        }
    }
    else {
        if (!ParseInt32s(bOffs, Out.Offsets)) return false;
    }

    // TYPES (UInt8)
    {
        TArray<int32> tmp; if (!ParseInt32s(bType, tmp)) return false;
        Out.Types.SetNumUninitialized(tmp.Num());
        for (int32 i = 0; i < tmp.Num(); ++i) Out.Types[i] = (uint8)tmp[i];
    }

    // FACEs (optionnel)
    {
        FString hFaces, bFaces, hFoffs, bFoffs;
        const bool hasFaces = ExtractDataArrayASCII(CellsBlock, bFaces, hFaces, TEXT("faces"));
        const bool hasFOffs = ExtractDataArrayASCII(CellsBlock, bFoffs, hFoffs, TEXT("faceoffsets"));
        if (hasFaces && hasFOffs)
        {
            if (HeadIs(hFaces, TEXT("Int64"))) {
                TArray<int64> t64; if (!ParseInt64s(bFaces, t64)) return false;
                Out.Faces.SetNumUninitialized(t64.Num());
                for (int32 i = 0; i < t64.Num(); ++i) {
                    if (t64[i] < 0 || t64[i] > INT32_MAX) { UE_LOG(LogTemp, Error, TEXT("VTU: faces > INT32_MAX")); return false; }
                    Out.Faces[i] = (int32)t64[i];
                }
            }
            else {
                if (!ParseInt32s(bFaces, Out.Faces)) return false;
            }
            if (HeadIs(hFoffs, TEXT("Int64"))) {
                TArray<int64> t64; if (!ParseInt64s(bFoffs, t64)) return false;
                Out.FaceOffsets.SetNumUninitialized(t64.Num());
                for (int32 i = 0; i < t64.Num(); ++i) {
                    if (t64[i] < 0 || t64[i] > INT32_MAX) { UE_LOG(LogTemp, Error, TEXT("VTU: faceoffsets > INT32_MAX")); return false; }
                    Out.FaceOffsets[i] = (int32)t64[i];
                }
            }
            else {
                if (!ParseInt32s(bFoffs, Out.FaceOffsets)) return false;
            }
            // Sanity faible : taille doit coller au nb de cellules
            if (Out.FaceOffsets.Num() != Out.Types.Num()) {
                UE_LOG(LogTemp, Warning, TEXT("VTU: faceoffsets size != NumCells (%d vs %d)"), Out.FaceOffsets.Num(), Out.Types.Num());
            }
        }
    }

    // Sanity générique cells
    if (Out.Offsets.Num() != Out.Types.Num()) return false;
    if (Out.Offsets.Num() == 0 || Out.Connectivity.Num() == 0) return false;
    if (Out.Offsets.Last() != Out.Connectivity.Num()) return false;

    return true;
}

// ---------- surface builder ----------
struct FTriKey {
    int32 A, B, C;
    friend uint32 GetTypeHash(const FTriKey& K) {
        return HashCombine(HashCombine(::GetTypeHash(K.A), ::GetTypeHash(K.B)), ::GetTypeHash(K.C));
    }
    bool operator==(const FTriKey& O) const { return A == O.A && B == O.B && C == O.C; }
    static FTriKey Make(int32 i0, int32 i1, int32 i2) {
        int32 a = i0, b = i1, c = i2; if (a > b)Swap(a, b); if (b > c)Swap(b, c); if (a > b)Swap(a, b); return { a,b,c };
    }
};
struct FQuadKey {
    int32 A, B, C, D;
    friend uint32 GetTypeHash(const FQuadKey& K) {
        return HashCombine(HashCombine(::GetTypeHash(K.A), ::GetTypeHash(K.B)),
            HashCombine(::GetTypeHash(K.C), ::GetTypeHash(K.D)));
    }
    bool operator==(const FQuadKey& O) const { return A == O.A && B == O.B && C == O.C && D == O.D; }
    static FQuadKey Make(int32 v0, int32 v1, int32 v2, int32 v3) {
        int32 a = v0, b = v1, c = v2, d = v3; if (a > b)Swap(a, b); if (c > d)Swap(c, d);
        if (a > c) { Swap(a, c); Swap(b, d); } if (b > d) Swap(b, d);
        return { a,b,c,d };
    }
};

// Clé pour un polygone (polyèdre) : liste triée d’indices -> dédoublonnage face/face
struct FPolyKey {
    TArray<int32> Sorted;
    bool operator==(const FPolyKey& O) const {
        if (Sorted.Num() != O.Sorted.Num()) return false;
        for (int32 i = 0; i < Sorted.Num(); ++i) if (Sorted[i] != O.Sorted[i]) return false;
        return true;
    }
    friend uint32 GetTypeHash(const FPolyKey& K) {
        uint32 h = 1469598103u;
        for (int32 v : K.Sorted) h = (h ^ ::GetTypeHash(v)) * 16777619u;
        return h;
    }
};
struct FPolyRec {
    TArray<int32> Ordered; // ordre rencontré sur la 1ère occurrence (pour trianguler)
    int32 Count = 0;
};

static void AddTetFaces(const int32* v, TMap<FTriKey, int32>& T) {
    const int F[4][3] = { {0,2,1},{0,1,3},{1,2,3},{2,0,3} };
    for (int f = 0; f < 4; ++f) T.FindOrAdd(FTriKey::Make(v[F[f][0]], v[F[f][1]], v[F[f][2]]))++;
}
static void AddHexFaces(const int32* v, TMap<FQuadKey, int32>& Q) {
    const int F[6][4] = { {0,1,2,3},{4,5,6,7},{0,4,5,1},{1,5,6,2},{2,6,7,3},{3,7,4,0} };
    for (int f = 0; f < 6; ++f) Q.FindOrAdd(FQuadKey::Make(v[F[f][0]], v[F[f][1]], v[F[f][2]], v[F[f][3]]))++;
}
static void AddWedgeFaces(const int32* v, TMap<FTriKey, int32>& T, TMap<FQuadKey, int32>& Q) {
    const int TF[2][3] = { {0,1,2},{3,4,5} };
    const int QF[3][4] = { {0,1,4,3},{1,2,5,4},{2,0,3,5} };
    for (int i = 0; i < 2; ++i) T.FindOrAdd(FTriKey::Make(v[TF[i][0]], v[TF[i][1]], v[TF[i][2]]))++;
    for (int i = 0; i < 3; ++i) Q.FindOrAdd(FQuadKey::Make(v[QF[i][0]], v[QF[i][1]], v[QF[i][2]], v[QF[i][3]]))++;
}
static void AddPyramidFaces(const int32* v, TMap<FTriKey, int32>& T, TMap<FQuadKey, int32>& Q) {
    Q.FindOrAdd(FQuadKey::Make(v[0], v[1], v[2], v[3]))++;
    const int TF[4][3] = { {0,1,4},{1,2,4},{2,3,4},{3,0,4} };
    for (int i = 0; i < 4; ++i) T.FindOrAdd(FTriKey::Make(v[TF[i][0]], v[TF[i][1]], v[TF[i][2]]))++;
}

bool VTUCore::BuildSurfaceToPMC(const FVTUGrid& G, UProceduralMeshComponent* PMC, float ScaleCm)
{
    if (!PMC || G.Points.Num() == 0 || G.NumCells() == 0) return false;

    TMap<FTriKey, int32>   CountTri;
    TMap<FQuadKey, int32>  CountQuad;
    TMap<FPolyKey, FPolyRec> CountPoly; // pour polyèdres (faces n-gones)

    // 1) Comptage des faces
    int32 start = 0;
    int32 startF = 0;
    for (int32 ci = 0; ci < G.NumCells(); ++ci)
    {
        const int32 end = G.Offsets[ci];
        const uint8 ty = G.Types[ci];
        const int32 n = end - start;
        const int32* v = G.Connectivity.GetData() + start;

        switch ((EVTKCellType)ty) {
        case EVTKCellType::VTK_TETRA:      if (n == 4) AddTetFaces(v, CountTri); break;
        case EVTKCellType::VTK_HEXAHEDRON: if (n == 8) AddHexFaces(v, CountQuad); break;
        case EVTKCellType::VTK_WEDGE:      if (n == 6) AddWedgeFaces(v, CountTri, CountQuad); break;
        case EVTKCellType::VTK_PYRAMID:    if (n == 5) AddPyramidFaces(v, CountTri, CountQuad); break;
        case EVTKCellType::VTK_TRIANGLE:   if (n == 3) CountTri.FindOrAdd(FTriKey::Make(v[0], v[1], v[2]))++; break;
        case EVTKCellType::VTK_QUAD:       if (n == 4) CountQuad.FindOrAdd(FQuadKey::Make(v[0], v[1], v[2], v[3]))++; break;
        case EVTKCellType::VTK_POLYHEDRON:
        {
            if (!G.HasPolyFaces()) break; // rien à compter si on n'a pas faces/faceoffsets
            const int32 endF = G.FaceOffsets.IsValidIndex(ci) ? G.FaceOffsets[ci] : startF;
            int32 idx = startF;
            if (endF <= idx) break;

            // Layout par cellule polyèdre :
            // [nFaces, (n0, v0...v{n0-1}), (n1, ...), ... ]
            if (idx >= endF) break;
            const int32 nFaces = G.Faces[idx++];

            for (int32 f = 0; f < nFaces && idx < endF; ++f) {
                if (idx >= endF) break;
                const int32 nv = G.Faces[idx++];
                if (idx + nv > endF) break;

                // face = nv indices
                const int32* fv = G.Faces.GetData() + idx;
                idx += nv;

                // clé dédoublonnée = triee
                FPolyKey key;
                key.Sorted.SetNumUninitialized(nv);
                for (int32 k = 0; k < nv; ++k) key.Sorted[k] = fv[k];
                key.Sorted.Sort();

                FPolyRec* rec = CountPoly.Find(key);
                if (!rec) {
                    FPolyRec r; r.Count = 1;
                    r.Ordered.SetNumUninitialized(nv);
                    for (int32 k = 0; k < nv; ++k) r.Ordered[k] = fv[k];
                    CountPoly.Add(key, MoveTemp(r));
                }
                else {
                    rec->Count++;
                }
            }

            startF = endF; // avance dans le flux faces pour la cellule suivante
            break;
        }
        default: break;
        }

        start = end;
    }

    // 2) Sommets (cm si besoin)
    TArray<FVector> V; V.SetNumUninitialized(G.Points.Num());
    for (int32 i = 0; i < G.Points.Num(); ++i) V[i] = FVector(G.Points[i]) * ScaleCm;

    // 3) Indices triangles de frontière
    TArray<int32> I; I.Reserve(CountTri.Num() * 3 + CountQuad.Num() * 6 + CountPoly.Num() * 6);

    // Triangles
    for (const auto& kv : CountTri)
        if (kv.Value == 1) { I.Add(kv.Key.A); I.Add(kv.Key.B); I.Add(kv.Key.C); }

    // Quads -> 2 triangles
    for (const auto& kv : CountQuad)
        if (kv.Value == 1) {
            I.Add(kv.Key.A); I.Add(kv.Key.B); I.Add(kv.Key.C);
            I.Add(kv.Key.A); I.Add(kv.Key.C); I.Add(kv.Key.D);
        }

    // Polygones -> triangulation en éventail (fan) selon l'ordre rencontré
    for (const auto& kv : CountPoly)
        if (kv.Value.Count == 1) {
            const TArray<int32>& F = kv.Value.Ordered;
            if (F.Num() >= 3) {
                for (int32 t = 1; t + 1 < F.Num(); ++t) {
                    I.Add(F[0]); I.Add(F[t]); I.Add(F[t + 1]);
                }
            }
        }

    // 4) Envoi PMC
    TArray<FVector> N; TArray<FVector2D> UV; TArray<FLinearColor> C; TArray<FProcMeshTangent> T;
    PMC->CreateMeshSection_LinearColor(0, V, I, N, UV, C, T, false);
    PMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    return true;
}

// ---------- prob ----------
void VTUCore::SummarizeProbe(const TCHAR* Tag, const FString& Line) {
    UE_LOG(LogTemp, Display, TEXT("[VTU:%s] %s"), Tag, *Line);
    if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 25.f, FColor::Yellow,
        FString::Printf(TEXT("[VTU:%s] %s"), Tag, *Line));
}

bool VTUCore::Probe_FileInfo(const FString& Path, int64& OutSize, bool& bUG, bool& bAscii, FString& OutFirstKB)
{
    OutSize = 0; bUG = false; bAscii = false; OutFirstKB.Reset();

    if (Path.IsEmpty() || !FPaths::FileExists(Path)) {
        SummarizeProbe(TEXT("ProbeA"), FString::Printf(TEXT("NOT FOUND: %s"), *Path));
        return false;
    }

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    TUniquePtr<IFileHandle> H(PF.OpenRead(*Path));
    if (!H) {
        SummarizeProbe(TEXT("ProbeA"), TEXT("OpenRead failed"));
        return false;
    }
    OutSize = H->Size();

    const int64 ReadSize = FMath::Min<int64>(OutSize, 64 * 1024);
    TArray<uint8> Buf; Buf.SetNumUninitialized(ReadSize);
    H->Read(Buf.GetData(), ReadSize);

    // best effort pour lire comme texte
    OutFirstKB = FString(ReadSize / sizeof(ANSICHAR), (ANSICHAR*)Buf.GetData());
    bUG = OutFirstKB.Contains(TEXT("<UnstructuredGrid"));
    bAscii = OutFirstKB.Contains(TEXT("format=\"ascii\"")) || OutFirstKB.Contains(TEXT("format='ascii'"));

    SummarizeProbe(TEXT("ProbeA"), FString::Printf(TEXT("Size=%lld bytes, UG=%s, asciiTag=%s"),
        OutSize, bUG ? TEXT("yes") : TEXT("no"), bAscii ? TEXT("yes") : TEXT("no")));
    return true;
}

// --- utilitaires parsing très légers (sans tout charger) ---
static bool FindAttribute(const FString& TagHead, const TCHAR* Attr, int32& OutVal)
{
    // cherche Attr="NNN" ou Attr='NNN'
    const FString A1 = FString::Printf(TEXT("%s=\""), Attr);
    const FString A2 = FString::Printf(TEXT("%s='"), Attr);
    int32 p = TagHead.Find(A1, ESearchCase::IgnoreCase);
    bool dq = true;
    if (p < 0) { p = TagHead.Find(A2, ESearchCase::IgnoreCase); dq = false; }
    if (p < 0) return false;
    p += dq ? A1.Len() : A2.Len();
    int32 q = TagHead.Find(dq ? TEXT("\"") : TEXT("'"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
    if (q < 0) return false;
    OutVal = FCString::Atoi(*TagHead.Mid(p, q - p));
    return true;
}

static bool FindTagHeadStreaming(IFileHandle* H, const TCHAR* OpenTag, FString& OutHead, int64 MaxScanBytes = 8 * 1024 * 1024)
{
    // Lit par blocs et s'arrête dès rencontrE "<Tag ...>" (jusqu'au '>')
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    FString Acc; Acc.Reserve(256 * 1024);
    int64 Read = 0;
    while (Read < MaxScanBytes) {
        const int64 ToRead = FMath::Min<int64>(CHUNK, MaxScanBytes - Read);
        if (!H->Read(Buf.GetData(), ToRead)) break;
        Read += ToRead;
        FString S = FString(ToRead, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
        Acc += S;
        int32 p = Acc.Find(OpenTag, ESearchCase::IgnoreCase);
        if (p >= 0) {
            int32 gt = Acc.Find(TEXT(">"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
            if (gt >= 0) {
                OutHead = Acc.Mid(p, gt - p); // "<Tag ..."(sans '>')
                return true;
            }
            // pas encore '>' -> continue lire
        }
        // garde une fenêtre raisonnable pour le cas où le tag est coupé
        if (Acc.Len() > 2 * CHUNK) Acc.RightInline(CHUNK, false);
    }
    return false;
}

static void ScanDataArrayHeadsStreaming(IFileHandle* H, bool& bHasConn, bool& bHasOffs, bool& bHasTypes, bool& bHasFaces, bool& bHasFoffs, int64 MaxScanBytes = 32 * 1024 * 1024)
{
    bHasConn = bHasOffs = bHasTypes = bHasFaces = bHasFoffs = false;
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    FString Acc;
    int64 Read = 0;
    while (Read < MaxScanBytes && !(bHasConn && bHasOffs && bHasTypes && bHasFaces && bHasFoffs)) {
        const int64 ToRead = FMath::Min<int64>(CHUNK, MaxScanBytes - Read);
        if (!H->Read(Buf.GetData(), ToRead)) break;
        Read += ToRead;
        FString S = FString(ToRead, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
        Acc += S;

        // Chercher les têtes <DataArray ... Name="xxx"
        auto mark = [&](const TCHAR* name) { return Acc.Contains(FString::Printf(TEXT("Name=\"%s\""), name)) ||
            Acc.Contains(FString::Printf(TEXT("Name='%s'"), name)); };
        if (!bHasConn && mark(TEXT("connectivity"))) bHasConn = true;
        if (!bHasOffs && mark(TEXT("offsets")))      bHasOffs = true;
        if (!bHasTypes && mark(TEXT("types")))       bHasTypes = true;
        if (!bHasFaces && mark(TEXT("faces")))       bHasFaces = true;
        if (!bHasFoffs && mark(TEXT("faceoffsets"))) bHasFoffs = true;

        if (Acc.Len() > 2 * CHUNK) Acc.RightInline(CHUNK, false);
    }
}

bool VTUCore::Probe_ParseHeaders_Light(const FString& Path, int32& OutNumPoints, int32& OutNumCells, bool& bHasConnectivity, bool& bHasOffsets, bool& bHasTypes, bool& bHasFaces, bool& bHasFaceOffsets, float& OutSeconds)
{
    OutNumPoints = OutNumCells = 0;
    bHasConnectivity = bHasOffsets = bHasTypes = bHasFaces = bHasFaceOffsets = false;

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    TUniquePtr<IFileHandle> H(PF.OpenRead(*Path));
    if (!H) { SummarizeProbe(TEXT("ProbeB"), TEXT("OpenRead failed")); return false; }

    const double t0 = FPlatformTime::Seconds();

    // 1) Trouver l'entête <Piece ...>
    FString PieceHead;
    if (!FindTagHeadStreaming(H.Get(), TEXT("<Piece"), PieceHead)) {
        SummarizeProbe(TEXT("ProbeB"), TEXT("No <Piece> head found"));
        return false;
    }
    FindAttribute(PieceHead, TEXT("NumberOfPoints"), OutNumPoints);
    FindAttribute(PieceHead, TEXT("NumberOfCells"), OutNumCells);

    // 2) Chercher <Cells> puis scruter les DataArray Name="..."
    // On scanne le reste du fichier jusqu'à trouver "<Cells"
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    FString Acc; Acc.Reserve(256 * 1024);
    bool bInCells = false;

    while (!H->Tell() || H->Tell() < H->Size())
    {
        int64 Remaining = H->Size() - H->Tell();
        int64 ToRead = FMath::Min<int64>(CHUNK, Remaining);
        if (ToRead <= 0) break;

        if (!H->Read(Buf.GetData(), ToRead)) break;
        FString S = FString(ToRead, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
        Acc += S;

        if (!bInCells)
        {
            int32 pCells = Acc.Find(TEXT("<Cells"), ESearchCase::IgnoreCase);
            if (pCells >= 0)
            {
                bInCells = true;
                // garde uniquement la fenêtre à partir de <Cells> pour limiter la mémoire
                Acc.RightInline(Acc.Len() - pCells, false);
            }
            else
            {
                // garder une fenêtre glissante pour ne pas grossir Acc
                if (Acc.Len() > 2 * CHUNK) Acc.RightInline(CHUNK, false);
            }
            continue;
        }

        // Nous sommes dans la fenêtre où <Cells> a été trouvé : détectons les 5 DataArray
        auto mark = [&](const TCHAR* name) { return Acc.Contains(FString::Printf(TEXT("Name=\"%s\""), name)) ||
            Acc.Contains(FString::Printf(TEXT("Name='%s'"), name)); };
        if (!bHasConnectivity && mark(TEXT("connectivity"))) bHasConnectivity = true;
        if (!bHasOffsets && mark(TEXT("offsets")))      bHasOffsets = true;
        if (!bHasTypes && mark(TEXT("types")))        bHasTypes = true;
        if (!bHasFaces && mark(TEXT("faces")))        bHasFaces = true;
        if (!bHasFaceOffsets && mark(TEXT("faceoffsets")))  bHasFaceOffsets = true;

        // s'arrêter tôt si on a tout vu
        if (bHasConnectivity && bHasOffsets && bHasTypes && bHasFaces && bHasFaceOffsets) break;

        // Compacter Acc pour contenir les derniers blocs près de <Cells>
        if (Acc.Len() > 4 * CHUNK) Acc.RightInline(2 * CHUNK, false);
    }

    OutSeconds = float(FPlatformTime::Seconds() - t0);
    SummarizeProbe(TEXT("ProbeB"), FString::Printf(
        TEXT("OK in %.2fs | NumPoints=%d NumCells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d"),
        OutSeconds, OutNumPoints, OutNumCells,
        bHasConnectivity ? 1 : 0, bHasOffsets ? 1 : 0, bHasTypes ? 1 : 0, bHasFaces ? 1 : 0, bHasFaceOffsets ? 1 : 0));

    return true;
}

bool VTUCore::Probe_ParseHeaders_ASCII(const FString& Path, int32& OutNumPoints, int32& OutNumCells, int32& OutConnCount, int32& OutFacesCount, int32& OutFaceOffsetsCount, bool& bHasPolyFaces, float& OutSeconds)
{
    const double t0 = FPlatformTime::Seconds();

    FVTUGrid G;
    if (!LoadVTU_ASCII(Path, G)) {
        SummarizeProbe(TEXT("ProbeB"), TEXT("LoadVTU_ASCII FAILED"));
        return false;
    }

    OutNumPoints = G.Points.Num();
    OutNumCells = G.NumCells();
    OutConnCount = G.Connectivity.Num();
    OutFacesCount = G.Faces.Num();
    OutFaceOffsetsCount = G.FaceOffsets.Num();
    bHasPolyFaces = G.HasPolyFaces();
    OutSeconds = float(FPlatformTime::Seconds() - t0);

    SummarizeProbe(TEXT("ProbeB"),
        FString::Printf(TEXT("OK in %.2fs | Pts=%d Cells=%d Conn=%d Faces=%d FOffs=%d PolyFaces=%s"),
            OutSeconds, OutNumPoints, OutNumCells, OutConnCount, OutFacesCount, OutFaceOffsetsCount,
            bHasPolyFaces ? TEXT("yes") : TEXT("no")));

    return true;
}

// Petit utilitaire: tronquer un bloc texte à N tokens blancs
static FString TakeFirstTokens(const FString& Body, int32 MaxTokens)
{
    int32 tokens = 0; int32 i = 0, N = Body.Len();
    auto IsWS = [](TCHAR c) { return FChar::IsWhitespace(c); };
    while (i < N && tokens < MaxTokens) {
        while (i < N && IsWS(Body[i])) ++i;
        if (i >= N) break;
        ++tokens;
        while (i < N && !IsWS(Body[i])) ++i;
    }
    return Body.Left(i);
}

bool VTUCore::Probe_ParseSamples_ASCII(const FString& Path, int32 MaxPointTokens, int32 MaxConnTokens, int32& OutTokPts, int32& OutTokConn, float& OutSecPts, float& OutSecConn)
{
    OutTokPts = OutTokConn = 0; OutSecPts = OutSecConn = 0;

    FString Xml;
    if (!FFileHelper::LoadFileToString(Xml, *Path)) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("cannot read file"));
        return false;
    }

    FString Piece; int32 after = 0;
    if (!ExtractTagBody(Xml, TEXT("Piece"), 0, Piece, after)) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("missing <Piece>"));
        return false;
    }

    // Points
    FString Pblk; int32 ap = 0;
    if (!ExtractTagBody(Piece, TEXT("Points"), 0, Pblk, ap)) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("missing <Points>"));
        return false;
    }
    FString PHead, PBody;
    if (!ExtractDataArrayASCII(Pblk, PBody, PHead)) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("Points DataArray ascii not found"));
        return false;
    }
    PBody = TakeFirstTokens(PBody, MaxPointTokens);
    {
        TArray<float> tmp;
        const double t0 = FPlatformTime::Seconds();
        if (!ParseFloats(PBody, tmp)) {
            SummarizeProbe(TEXT("ProbeC"), TEXT("ParseFloats failed"));
            return false;
        }
        OutSecPts = float(FPlatformTime::Seconds() - t0);
        OutTokPts = tmp.Num();
        SummarizeProbe(TEXT("ProbeC"), FString::Printf(TEXT("Points tokens=%d in %.2fs"), OutTokPts, OutSecPts));
    }

    // Cells/connectivity
    FString Cblk; int32 ac = 0;
    if (!ExtractTagBody(Piece, TEXT("Cells"), 0, Cblk, ac)) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("missing <Cells>"));
        return false;
    }
    FString HConn, BConn;
    if (!ExtractDataArrayASCII(Cblk, BConn, HConn, TEXT("connectivity"))) {
        SummarizeProbe(TEXT("ProbeC"), TEXT("no connectivity"));
        return false;
    }
    BConn = TakeFirstTokens(BConn, MaxConnTokens);
    {
        TArray<int64> tmp;
        const double t0 = FPlatformTime::Seconds();
        if (!ParseInt64s(BConn, tmp)) {
            SummarizeProbe(TEXT("ProbeC"), TEXT("ParseInt64s failed"));
            return false;
        }
        OutSecConn = float(FPlatformTime::Seconds() - t0);
        OutTokConn = tmp.Num();
        SummarizeProbe(TEXT("ProbeC"), FString::Printf(TEXT("Conn tokens=%d in %.2fs"), OutTokConn, OutSecConn));
    }

    return true;
}

// Cherche la tête du DataArray avec Name="offsets" et renvoie la position juste après '>' (début des nombres)
static bool FindDataArrayStart(IFileHandle* H, const TCHAR* NameAttr, int64& OutDataPos, int64 MaxScan = 256ll * 1024 * 1024)
{
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    FString Acc;
    int64 Read = 0;
    while (Read < MaxScan && (H->Tell() < H->Size()))
    {
        int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
        if (ToRead <= 0) break;
        if (!H->Read(Buf.GetData(), ToRead)) break;
        Read += ToRead;

        FString S = FString(ToRead, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
        Acc += S;

        // trouve <DataArray ... Name="offsets" ... >
        int32 p = Acc.Find(TEXT("<DataArray"), ESearchCase::IgnoreCase);
        while (p >= 0)
        {
            int32 gt = Acc.Find(TEXT(">"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
            if (gt < 0) break; // tête coupée -> lire plus

            const FString Head = Acc.Mid(p, gt - p); // sans '>'
            const bool hasName = Head.Contains(FString::Printf(TEXT("Name=\"%s\""), NameAttr)) ||
                Head.Contains(FString::Printf(TEXT("Name='%s'"), NameAttr));
            const bool ascii = Head.Contains(TEXT("format=\"ascii\"")) || Head.Contains(TEXT("format='ascii'"));
            if (hasName && ascii)
            {
                // Position de début des données = position du file à ce moment - longueur restante après '>'
                const int64 FilePosAfterChunk = H->Tell();
                const int64 AccTailAfterGt = Acc.Len() - (gt + 1);
                OutDataPos = FilePosAfterChunk - FTCHARToUTF8(*Acc.Right(AccTailAfterGt)).Length();
                // Recalage simple : on repositionne le curseur à OutDataPos
                H->Seek(OutDataPos);
                return true;
            }

            // Cherche prochain <DataArray> dans Acc
            int32 nextSearchStart = p + 10;
            p = Acc.Find(TEXT("<DataArray"), ESearchCase::IgnoreCase, ESearchDir::FromStart, nextSearchStart);
        }

        // Compacter la fenêtre (éviter la croissance infinie)
        if (Acc.Len() > 2 * CHUNK) Acc.RightInline(CHUNK, false);
    }
    return false;
}

// Parse N entiers Int64 ASCII depuis la position courante du fichier
static bool ParseN_Int64_ASCII(IFileHandle* H, int32 N, int64& OutLast, float& OutSeconds)
{
    const double t0 = FPlatformTime::Seconds();
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);

    // état du tokenizer
    bool  inNum = false;
    bool  neg = false;
    int64 val = 0;
    int32 count = 0;

    auto flush = [&]() {
        if (inNum) {
            int64 final = neg ? -val : val;
            OutLast = final; // on ne garde que le dernier
            ++count;
            inNum = false; neg = false; val = 0;
        }
        };

    while (count < N && H->Tell() < H->Size())
    {
        int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
        if (ToRead <= 0) break;
        if (!H->Read(Buf.GetData(), ToRead)) break;

        const char* p = (const char*)Buf.GetData();
        const char* e = p + ToRead;

        while (p < e && count < N)
        {
            const char c = *p++;
            if (c == '-' && !inNum) { inNum = true; neg = true; val = 0; continue; }
            if (c >= '0' && c <= '9') {
                if (!inNum) { inNum = true; neg = false; val = (c - '0'); }
                else { val = val * 10 + (c - '0'); }
            }
            else {
                // séparateur -> flush si besoin
                if (inNum) flush();
            }
        }
    }
    // fin de flux
    if (count < N && inNum) flush();

    OutSeconds = float(FPlatformTime::Seconds() - t0);
    return count == N;
}

bool VTUCore::ReadOffsetsLast_ASCII(const FString& Path,
    int32 NumCells,
    int64& OutLastOffset,
    float& OutSeconds)
{
    OutLastOffset = 0; OutSeconds = 0.f;

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    TUniquePtr<IFileHandle> H(PF.OpenRead(*Path));
    if (!H) { SummarizeProbe(TEXT("ProbeOffs"), TEXT("OpenRead failed")); return false; }

    // Trouver le DataArray Name="offsets"
    int64 dataPos = 0;
    if (!FindDataArrayStart(H.Get(), TEXT("offsets"), dataPos)) {
        SummarizeProbe(TEXT("ProbeOffs"), TEXT("Offsets DataArray not found"));
        return false;
    }

    // Parse N entiers 64-bits
    float secs = 0;
    if (!ParseN_Int64_ASCII(H.Get(), NumCells, OutLastOffset, secs)) {
        SummarizeProbe(TEXT("ProbeOffs"), TEXT("Failed to parse N Int64 offsets"));
        return false;
    }
    OutSeconds = secs;

    SummarizeProbe(TEXT("ProbeOffs"), FString::Printf(TEXT("Parsed %d offsets, last=%lld (%.2fs)"),
        NumCells, OutLastOffset, OutSeconds));
    return true;
}

bool VTUCore::Probe_ScanCountsAndOffsets(const FString& Path,
    int32& P, int32& C,
    bool& hasConn, bool& hasOffs, bool& hasTypes, bool& hasFaces, bool& hasFoffs,
    int64& ConnLen,
    float& secsHead, float& secsOffs)
{
    if (!Probe_ParseHeaders_Light(Path, P, C, hasConn, hasOffs, hasTypes, hasFaces, hasFoffs, secsHead))
        return false;
    ConnLen = 0; secsOffs = 0.f;
    if (hasOffs && C > 0) {
        if (!ReadOffsetsLast_ASCII(Path, C, ConnLen, secsOffs)) return false;
    }
    SummarizeProbe(TEXT("ProbeALL"), FString::Printf(
        TEXT("Counts OK in %.2fs, Offs OK in %.2fs | Points=%d Cells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d | ConnLen=%lld"),
        secsHead, secsOffs, P, C, hasConn, hasOffs, hasTypes, hasFaces, hasFoffs, ConnLen));
    return true;
}

// Parse M floats ASCII depuis la position courante -> callback 'sink(val, idx)' (évite les gros arrays temporaires)
template<typename Sink>
static bool ParseN_Float_ASCII(IFileHandle* H, int64 M, Sink sink, float& OutSeconds)
{
    const double t0 = FPlatformTime::Seconds();
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);

    FString Tok; Tok.Reserve(64);
    int64 count = 0;

    auto flush = [&]() {
        if (!Tok.IsEmpty()) {
            const float v = FCString::Atof(*Tok); // gère -, ., e/E
            sink(v, count);
            Tok.Reset();
            ++count;
        }
        };

    while (count < M && H->Tell() < H->Size())
    {
        const int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
        if (ToRead <= 0) break;
        if (!H->Read(Buf.GetData(), ToRead)) break;

        const char* p = reinterpret_cast<const char*>(Buf.GetData());
        const char* e = p + ToRead;

        while (p < e && count < M)
        {
            const char c = *p++;
            // séparateurs ASCII usuels
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') { flush(); }
            else { Tok.AppendChar((TCHAR)c); } // on stocke le token en TCHAR, conversion bon marché
        }
    }
    if (count < M && !Tok.IsEmpty()) flush();

    OutSeconds = float(FPlatformTime::Seconds() - t0);
    return count == M;
}

// Parse N entiers non signés petits (types) en ASCII -> uint8
static bool ParseN_UInt8_ASCII(IFileHandle* H, int32 N, TArray<uint8>& Out, float& OutSeconds)
{
    const double t0 = FPlatformTime::Seconds();
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    Out.SetNumUninitialized(N);
    int32 count = 0; int32 val = 0; bool inNum = false;

    auto flush = [&]() {
        if (inNum) {
            Out[count++] = (uint8)FMath::Clamp(val, 0, 255);
            inNum = false; val = 0;
        }
        };

    while (count < N && H->Tell() < H->Size()) {
        int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
        if (ToRead <= 0) break;
        if (!H->Read(Buf.GetData(), ToRead)) break;
        const char* p = (const char*)Buf.GetData(); const char* e = p + ToRead;
        while (p < e && count < N) {
            const char c = *p++;
            if (c >= '0' && c <= '9') { inNum = true; val = val * 10 + (c - '0'); }
            else { flush(); }
        }
    }
    if (count < N && inNum) flush();

    OutSeconds = float(FPlatformTime::Seconds() - t0);
    return count == N;
}

// Seek vers le 1er <DataArray ... format="ascii"> à l'intérieur de <Points>
// Positionne le file handle juste après '>' => prêt à parser les floats.
static bool SeekToPointsDataArrayASCII(IFileHandle* H, int64& OutDataPos, int64 MaxScanBytes = 256ll * 1024 * 1024)
{
    const int32 CHUNK = 1024 * 1024;
    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
    FString Acc;
    Acc.Reserve(256 * 1024);

    bool bSawPoints = false;
    int64 Read = 0;

    while (Read < MaxScanBytes && H->Tell() < H->Size())
    {
        const int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
        if (ToRead <= 0) break;
        if (!H->Read(Buf.GetData(), ToRead)) break;

        Read += ToRead;
        const FString S = FString(ToRead, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf.GetData())));
        Acc += S;

        if (!bSawPoints)
        {
            int32 pPts = Acc.Find(TEXT("<Points"), ESearchCase::IgnoreCase);
            if (pPts >= 0)
            {
                bSawPoints = true;
                // garde surtout la fenêtre depuis <Points>
                Acc.RightInline(Acc.Len() - pPts, /*bAllowShrinking*/ false);
            }
            else
            {
                if (Acc.Len() > 2 * CHUNK) Acc.RightInline(CHUNK, false);
                continue;
            }
        }

        // On a vu <Points> : cherche le 1er <DataArray ... format="ascii">
        int32 pDA = Acc.Find(TEXT("<DataArray"), ESearchCase::IgnoreCase);
        while (pDA >= 0)
        {
            const int32 gt = Acc.Find(TEXT(">"), ESearchCase::IgnoreCase, ESearchDir::FromStart, pDA);
            if (gt < 0)
            {
                // header coupé entre 2 chunks -> lire plus
                break;
            }

            const FString Head = Acc.Mid(pDA, gt - pDA);
            const bool bAscii = Head.Contains(TEXT("format=\"ascii\"")) || Head.Contains(TEXT("format='ascii'"));

            if (bAscii)
            {
                // file pos après ce chunk
                const int64 FilePosAfterChunk = H->Tell();
                // longueur UTF8 de la "queue" conservée après le '>'
                const FString Tail = Acc.Right(Acc.Len() - (gt + 1));
                const int32   TailBytes = FTCHARToUTF8(*Tail).Length();
                // position exacte du début de données dans le fichier
                OutDataPos = FilePosAfterChunk - TailBytes;
                H->Seek(OutDataPos);
                return true;
            }

            // sinon: chercher le prochain <DataArray> dans la fenêtre courante
            pDA = Acc.Find(TEXT("<DataArray"), ESearchCase::IgnoreCase, ESearchDir::FromStart, pDA + 10);
        }

        // compacter la fenêtre pour ne pas grossir en RAM
        if (Acc.Len() > 4 * CHUNK) Acc.RightInline(2 * CHUNK, false);
    }

    return false;
}

bool VTUCore::LoadVTU_ASCII_Streaming(const FString& FilePath, FVTUGrid& Out, bool bLoadFacesIfPoly)
{
    Out = FVTUGrid();

    // 1) Comptes & présence
    int32 NumPts = 0, NumCells = 0; bool hasConn = false, hasOffs = false, hasTypes = false, hasFaces = false, hasFoffs = false;
    float tHead = 0, tOffs = 0; int64 ConnLen = 0;
    if (!Probe_ScanCountsAndOffsets(FilePath, NumPts, NumCells, hasConn, hasOffs, hasTypes, hasFaces, hasFoffs, ConnLen, tHead, tOffs))
        return false;
    if (NumPts <= 0 || NumCells <= 0 || !hasConn || !hasOffs || !hasTypes) return false;

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    // 2) POINTS (3*NumPts floats) — recherche correcte dans <Points> sans s'appuyer sur Name=""
    {
        TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
        if (!H) return false;

        int64 dataPos = 0;
        if (!SeekToPointsDataArrayASCII(H.Get(), dataPos)) {
            SummarizeProbe(TEXT("LoadStream"), TEXT("Failed to locate <Points>/<DataArray ascii>"));
            return false;
        }

        Out.Points.SetNumUninitialized(NumPts);
        FBox box(ForceInitToZero);

        float secs = 0.f;
        const int64 M = int64(NumPts) * 3;

        int64 k = 0;
        auto sink = [&](float v, int64 idx) {
            const int64 i = idx / 3;
            const int64 c = idx % 3;
            if (c == 0) Out.Points[i].X = v;
            else if (c == 1) Out.Points[i].Y = v;
            else             Out.Points[i].Z = v;
            if (c == 2) box += FVector(Out.Points[i]); // bounds quand triplet complet
            };

        if (!ParseN_Float_ASCII(H.Get(), M, sink, secs)) {
            SummarizeProbe(TEXT("LoadStream"), TEXT("ParseN_Float_ASCII(points) failed"));
            return false;
        }

        Out.Bounds = box;
        SummarizeProbe(TEXT("LoadStream"), FString::Printf(TEXT("Points OK: %d (%.2fs)"), NumPts, secs));
    }

    // 3) OFFSETS (NumCells Int64) -> Out.Offsets (int32) & ConnLen (déjà connu)
    {
        TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
        if (!H) return false;
        int64 dataPos = 0; if (!FindDataArrayStart(H.Get(), TEXT("offsets"), dataPos)) return false;
        float secs = 0; int64 last = 0;
        // on parse et on stocke
        Out.Offsets.SetNumUninitialized(NumCells);
        const double t0 = FPlatformTime::Seconds();
        // réutilise ParseN_Int64_ASCII, mais copie chaque valeur
        int32 parsed = 0;
        const int32 CHUNK = 1024 * 1024;
        TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
        bool inNum = false, neg = false; int64 val = 0;

        auto flushOne = [&]() {
            if (inNum) {
                int64 final = neg ? -val : val;
                if (final < 0 || final > INT32_MAX) { SummarizeProbe(TEXT("LoadStream"), TEXT("offset overflow")); parsed = -1; }
                else {
                    Out.Offsets[parsed++] = (int32)final;
                    last = final;
                }
                inNum = false; neg = false; val = 0;
            }
            };

        while (parsed >= 0 && parsed < NumCells && H->Tell() < H->Size()) {
            int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
            if (ToRead <= 0) break;
            if (!H->Read(Buf.GetData(), ToRead)) break;
            const char* p = (const char*)Buf.GetData(); const char* e = p + ToRead;
            while (p < e && parsed >= 0 && parsed < NumCells) {
                const char c = *p++;
                if (c == '-' && !inNum) { inNum = true; neg = true; val = 0; }
                else if (c >= '0' && c <= '9') { if (!inNum) { inNum = true; neg = false; val = (c - '0'); } else { val = val * 10 + (c - '0'); } }
                else { flushOne(); }
            }
        }
        if (parsed >= 0 && parsed < NumCells && inNum) flushOne();
        secs = float(FPlatformTime::Seconds() - t0);
        if (parsed != NumCells) return false;
        if (ConnLen == 0) ConnLen = last;

        SummarizeProbe(TEXT("LoadStream"), FString::Printf(TEXT("Offsets OK: %d (last=%d)"), NumCells, Out.Offsets.Last()));
    }

    // 4) TYPES (NumCells UInt8 ASCII)
    {
        TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
        if (!H) return false;
        int64 dataPos = 0; if (!FindDataArrayStart(H.Get(), TEXT("types"), dataPos)) return false;
        float secs = 0;
        if (!ParseN_UInt8_ASCII(H.Get(), NumCells, Out.Types, secs)) return false;

        SummarizeProbe(TEXT("LoadStream"), FString::Printf(TEXT("Types OK: %d"), Out.Types.Num()));
    }

    // 5) CONNECTIVITY (ConnLen Int64) -> Out.Connectivity (int32)
    {
        if (ConnLen <= 0 || ConnLen > INT32_MAX) return false;
        Out.Connectivity.SetNumUninitialized((int32)ConnLen);

        TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
        if (!H) return false;
        int64 dataPos = 0; if (!FindDataArrayStart(H.Get(), TEXT("connectivity"), dataPos)) return false;

        const int32 CHUNK = 1024 * 1024;
        TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
        int32 count = 0; bool inNum = false, neg = false; int64 val = 0;

        auto flush = [&]() {
            if (inNum) {
                int64 final = neg ? -val : val;
                if (final < 0 || final > INT32_MAX) { count = -1; }
                else Out.Connectivity[count++] = (int32)final;
                inNum = false; neg = false; val = 0;
            }
            };

        while (count >= 0 && count < ConnLen && H->Tell() < H->Size()) {
            int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
            if (ToRead <= 0) break;
            if (!H->Read(Buf.GetData(), ToRead)) break;
            const char* p = (const char*)Buf.GetData(); const char* e = p + ToRead;
            while (p < e && count >= 0 && count < ConnLen) {
                const char c = *p++;
                if (c == '-' && !inNum) { inNum = true; neg = true; val = 0; }
                else if (c >= '0' && c <= '9') { if (!inNum) { inNum = true; neg = false; val = (c - '0'); } else { val = val * 10 + (c - '0'); } }
                else { flush(); }
            }
        }
        if (count >= 0 && count < ConnLen && inNum) flush();
        if (count != ConnLen) return false;

        SummarizeProbe(TEXT("LoadStream"), FString::Printf(TEXT("Connectivity OK: %d"), Out.Connectivity.Num()));
    }

    // 6) FACEs / FACEOFFSETs si nécessaire (polyhedra)
    if (bLoadFacesIfPoly)
    {
        bool hasPoly = false;
        for (uint8 t : Out.Types) { if (t == (uint8)EVTKCellType::VTK_POLYHEDRON) { hasPoly = true; break; } }
        if (hasPoly && hasFaces && hasFoffs)
        {
            // FaceOffsets
            {
                TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
                if (!H) return false;
                int64 pos = 0; if (!FindDataArrayStart(H.Get(), TEXT("faceoffsets"), pos)) return false;

                // parse NumCells int64 -> int32
                Out.FaceOffsets.SetNumUninitialized(NumCells);
                const int32 CHUNK = 1024 * 1024;
                TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
                int32 parsed = 0; bool inNum = false, neg = false; int64 val = 0;

                auto flush = [&]() {
                    if (inNum) {
                        int64 final = neg ? -val : val;
                        if (final < 0 || final > INT32_MAX) { parsed = -1; }
                        else Out.FaceOffsets[parsed++] = (int32)final;
                        inNum = false; neg = false; val = 0;
                    }
                    };

                while (parsed >= 0 && parsed < NumCells && H->Tell() < H->Size()) {
                    int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
                    if (ToRead <= 0) break;
                    if (!H->Read(Buf.GetData(), ToRead)) break;
                    const char* p = (const char*)Buf.GetData(); const char* e = p + ToRead;
                    while (p < e && parsed >= 0 && parsed < NumCells) {
                        const char c = *p++;
                        if (c == '-' && !inNum) { inNum = true; neg = true; val = 0; }
                        else if (c >= '0' && c <= '9') { if (!inNum) { inNum = true; neg = false; val = (c - '0'); } else { val = val * 10 + (c - '0'); } }
                        else { flush(); }
                    }
                }
                if (parsed >= 0 && parsed < NumCells && inNum) flush();
                if (parsed != NumCells) return false;
            }

            // Faces (longueur = FaceOffsets.Last())
            if (Out.FaceOffsets.Num() > 0) {
                const int32 FacesLen = Out.FaceOffsets.Last();
                if (FacesLen > 0) {
                    Out.Faces.SetNumUninitialized(FacesLen);
                    TUniquePtr<IFileHandle> H(PF.OpenRead(*FilePath));
                    if (!H) return false;
                    int64 pos = 0; if (!FindDataArrayStart(H.Get(), TEXT("faces"), pos)) return false;

                    const int32 CHUNK = 1024 * 1024;
                    TArray<uint8> Buf; Buf.SetNumUninitialized(CHUNK);
                    int32 count = 0; bool inNum = false, neg = false; int64 val = 0;

                    auto flush = [&]() {
                        if (inNum) {
                            int64 final = neg ? -val : val;
                            if (final < 0 || final > INT32_MAX) { count = -1; }
                            else Out.Faces[count++] = (int32)final;
                            inNum = false; neg = false; val = 0;
                        }
                        };

                    while (count >= 0 && count < FacesLen && H->Tell() < H->Size()) {
                        int64 ToRead = FMath::Min<int64>(CHUNK, H->Size() - H->Tell());
                        if (ToRead <= 0) break;
                        if (!H->Read(Buf.GetData(), ToRead)) break;
                        const char* p = (const char*)Buf.GetData(); const char* e = p + ToRead;
                        while (p < e && count >= 0 && count < FacesLen) {
                            const char c = *p++;
                            if (c == '-' && !inNum) { inNum = true; neg = true; val = 0; }
                            else if (c >= '0' && c <= '9') { if (!inNum) { inNum = true; neg = false; val = (c - '0'); } else { val = val * 10 + (c - '0'); } }
                            else { flush(); }
                        }
                    }
                    if (count >= 0 && count < FacesLen && inNum) flush();
                    if (count != FacesLen) return false;
                }
            }
        }
        SummarizeProbe(TEXT("LoadStream"), FString::Printf(TEXT("Faces OK: %d  FaceOffsets OK: %d"), Out.Faces.Num(), Out.FaceOffsets.Num()));
    }

    if (Out.Offsets.Num() != NumCells || Out.Types.Num() != NumCells) {
        SummarizeProbe(TEXT("LoadStream"), TEXT("Sanity fail: counts mismatch with NumCells"));
        return false;
    }
    if (Out.Offsets.Last() != Out.Connectivity.Num()) {
        SummarizeProbe(TEXT("LoadStream"), TEXT("Sanity fail: OffsetsLast != Connectivity.Num()"));
        return false;
    }

    return true;
}

static constexpr uint32 VTU_MAGIC = 0x42555456u; // 'VTUB' LE
static constexpr uint32 VTU_VER = 1;

static uint32 CRC_Update(uint32 Prev, const void* Data, SIZE_T Bytes)
{
    return FCrc::MemCrc32(Data, Bytes, Prev);
}

template<typename T>
static bool BinWriteArray(IFileHandle* W, const TArray<T>& A)
{
    if (A.Num() == 0) return true;
    return W->Write(reinterpret_cast<const uint8*>(A.GetData()), sizeof(T) * A.Num());
}
template<typename T>
static bool BinReadArray(IFileHandle* R, TArray<T>& A, int32 N)
{
    if (N <= 0) { A.Reset(); return true; }
    A.SetNumUninitialized(N);
    return R->Read(reinterpret_cast<uint8*>(A.GetData()), sizeof(T) * N);
}

bool VTUCore::SaveGridBinary(const FString& Path, const FVTUGrid& G)
{
    // Sanity minimale
    if (G.Points.Num() == 0 || G.Offsets.Num() != G.Types.Num()) return false;
    if (G.Offsets.Last() != G.Connectivity.Num()) return false;

    const bool bHasFaces = (G.FaceOffsets.Num() == G.Types.Num() && G.Faces.Num() > 0);
    const int32 FacesLen = bHasFaces ? G.Faces.Num() : 0;

    // CRC sur les zones data
    uint32 Crc = 0;
    Crc = CRC_Update(Crc, G.Points.GetData(), sizeof(FVector3f) * G.Points.Num());
    Crc = CRC_Update(Crc, G.Offsets.GetData(), sizeof(int32) * G.Offsets.Num());
    Crc = CRC_Update(Crc, G.Types.GetData(), sizeof(uint8) * G.Types.Num());
    Crc = CRC_Update(Crc, G.Connectivity.GetData(), sizeof(int32) * G.Connectivity.Num());
    if (bHasFaces) {
        Crc = CRC_Update(Crc, G.FaceOffsets.GetData(), sizeof(int32) * G.FaceOffsets.Num());
        Crc = CRC_Update(Crc, G.Faces.GetData(), sizeof(int32) * G.Faces.Num());
    }

    // Ouverture + dossiers
    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    PF.CreateDirectoryTree(*FPaths::GetPath(Path));
    TUniquePtr<IFileHandle> W(PF.OpenWrite(*Path, /*append*/false));
    if (!W) return false;

    // Header
    const uint32 Magic = VTU_MAGIC;
    const uint32 Version = VTU_VER;
    const int32  NumPts = G.Points.Num();
    const int32  NumCel = G.Types.Num();
    const int32  ConnLen = G.Connectivity.Num();
    const uint8  HasFaces = bHasFaces ? 1 : 0;
    const int32  FacesN = FacesLen;

    FVector MinB = G.Bounds.Min;
    FVector MaxB = G.Bounds.Max;

    auto WU32 = [&](uint32 v) { return W->Write(reinterpret_cast<const uint8*>(&v), sizeof(v)); };
    auto WI32 = [&](int32 v) {  return W->Write(reinterpret_cast<const uint8*>(&v), sizeof(v)); };
    auto WU8 = [&](uint8 v) {  return W->Write(reinterpret_cast<const uint8*>(&v), sizeof(v)); };
    auto WVec3 = [&](const FVector& v) {
        float tmp[3] = { (float)v.X, (float)v.Y, (float)v.Z };
        return W->Write(reinterpret_cast<const uint8*>(tmp), sizeof(tmp));
        };

    if (!WU32(Magic) || !WU32(Version)) return false;
    if (!WI32(NumPts) || !WI32(NumCel) || !WI32(ConnLen)) return false;
    if (!WU8(HasFaces) || !WI32(FacesN)) return false;
    if (!WVec3(MinB) || !WVec3(MaxB)) return false;
    if (!WU32(Crc)) return false;

    // Data
    if (!BinWriteArray(W.Get(), G.Points))       return false;
    if (!BinWriteArray(W.Get(), G.Offsets))      return false;
    if (!BinWriteArray(W.Get(), G.Types))        return false;
    if (!BinWriteArray(W.Get(), G.Connectivity)) return false;
    if (bHasFaces) {
        if (!BinWriteArray(W.Get(), G.FaceOffsets)) return false;
        if (!BinWriteArray(W.Get(), G.Faces))       return false;
    }
    return true;
}

bool VTUCore::LoadGridBinary(const FString& Path, FVTUGrid& Out)
{
    Out = FVTUGrid();

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    TUniquePtr<IFileHandle> R(PF.OpenRead(*Path));
    if (!R) return false;

    auto RU32 = [&](uint32& v) { return R->Read(reinterpret_cast<uint8*>(&v), sizeof(v)); };
    auto RI32 = [&](int32& v) {  return R->Read(reinterpret_cast<uint8*>(&v), sizeof(v)); };
    auto RU8 = [&](uint8& v) {  return R->Read(reinterpret_cast<uint8*>(&v), sizeof(v)); };
    auto RVec3 = [&](FVector& v) {
        float tmp[3] = { 0,0,0 };
        if (!R->Read(reinterpret_cast<uint8*>(tmp), sizeof(tmp))) return false;
        v = FVector(tmp[0], tmp[1], tmp[2]); return true;
        };

    uint32 Magic = 0, Version = 0, CrcStored = 0;
    int32 NumPts = 0, NumCel = 0, ConnLen = 0, FacesN = 0;
    uint8 HasFaces = 0;
    FVector MinB(0), MaxB(0);

    if (!RU32(Magic) || Magic != VTU_MAGIC) return false;
    if (!RU32(Version) || Version != VTU_VER) return false;
    if (!RI32(NumPts) || !RI32(NumCel) || !RI32(ConnLen)) return false;
    if (!RU8(HasFaces) || !RI32(FacesN)) return false;
    if (!RVec3(MinB) || !RVec3(MaxB)) return false;
    if (!RU32(CrcStored)) return false;

    if (NumPts <= 0 || NumCel <= 0 || ConnLen <= 0) return false;

    // Lire les data
    if (!BinReadArray(R.Get(), Out.Points, NumPts)) return false;
    if (!BinReadArray(R.Get(), Out.Offsets, NumCel)) return false;
    if (!BinReadArray(R.Get(), Out.Types, NumCel)) return false;
    if (!BinReadArray(R.Get(), Out.Connectivity, ConnLen)) return false;

    if (HasFaces) {
        if (!BinReadArray(R.Get(), Out.FaceOffsets, NumCel)) return false;
        if (!BinReadArray(R.Get(), Out.Faces, FacesN)) return false;
    }
    else {
        Out.FaceOffsets.Reset(); Out.Faces.Reset();
    }

    // CRC
    uint32 Crc = 0;
    Crc = CRC_Update(Crc, Out.Points.GetData(), sizeof(FVector3f) * Out.Points.Num());
    Crc = CRC_Update(Crc, Out.Offsets.GetData(), sizeof(int32) * Out.Offsets.Num());
    Crc = CRC_Update(Crc, Out.Types.GetData(), sizeof(uint8) * Out.Types.Num());
    Crc = CRC_Update(Crc, Out.Connectivity.GetData(), sizeof(int32) * Out.Connectivity.Num());
    if (HasFaces) {
        Crc = CRC_Update(Crc, Out.FaceOffsets.GetData(), sizeof(int32) * Out.FaceOffsets.Num());
        Crc = CRC_Update(Crc, Out.Faces.GetData(), sizeof(int32) * Out.Faces.Num());
    }
    if (Crc != CrcStored) {
        UE_LOG(LogTemp, Error, TEXT("VTU cache CRC mismatch (file %s)"), *Path);
        return false;
    }

    // Sanity
    if (Out.Offsets.Num() != Out.Types.Num()) return false;
    if (Out.Offsets.Last() != Out.Connectivity.Num()) return false;

    Out.Bounds = FBox(MinB, MaxB);
    return true;
}
//------------------------------------------------------------------------------------------------CELLS-----------------------------------------------------------------------------------------------
// ---- Lecteur .npy minimal pour '<f8', fortran_order=False ----

static bool Npy_ReadHeader(IFileHandle* H, int64& OutDataOffset, int64& OutCount, bool& bLittleEndian, bool& bFOrder, FString& DType)
{
    OutDataOffset = 0; OutCount = 0; bLittleEndian = false; bFOrder = false; DType.Reset();

    // Magic (6) + version (2)
    uint8 hdr8[8];
    if (!H->Read(hdr8, 8)) return false;
    const uint8 expectedMagic[6] = { 0x93,'N','U','M','P','Y' };
    for (int i = 0; i < 6; ++i) if (hdr8[i] != expectedMagic[i]) return false;

    const uint8 vMajor = hdr8[6];
    const uint8 vMinor = hdr8[7];

    uint32 headerLen = 0;
    if (vMajor == 1) {
        uint8 l2[2]; if (!H->Read(l2, 2)) return false;
        headerLen = (uint32)l2[0] | ((uint32)l2[1] << 8);
    }
    else { // v2/v3
        uint8 l4[4]; if (!H->Read(l4, 4)) return false;
        headerLen = (uint32)l4[0] | ((uint32)l4[1] << 8) | ((uint32)l4[2] << 16) | ((uint32)l4[3] << 24);
    }

    TArray<uint8> hdr; hdr.SetNumUninitialized(headerLen);
    if (!H->Read(hdr.GetData(), headerLen)) return false;
    const FString HdrStr = FString(headerLen, UTF8_TO_TCHAR((const char*)hdr.GetData()));

    // dtype
    {
        int32 p = HdrStr.Find(TEXT("'descr':"));
        if (p < 0) p = HdrStr.Find(TEXT("\"descr\":"));
        if (p < 0) return false;
        int32 q = HdrStr.Find(TEXT("'"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p + 8);
        if (q < 0) q = HdrStr.Find(TEXT("\""), ESearchCase::IgnoreCase, ESearchDir::FromStart, p + 8);
        if (q < 0) return false;
        const TCHAR quote = HdrStr[q];
        int32 r = HdrStr.Find(FString::Chr(quote), ESearchCase::IgnoreCase, ESearchDir::FromStart, q + 1);
        if (r < 0) return false;
        DType = HdrStr.Mid(q + 1, r - q - 1); // ex: "<f8"
        bLittleEndian = DType.Len() > 0 && DType[0] == '<';
    }

    // fortran_order
    {
        int32 p = HdrStr.Find(TEXT("fortran_order"));
        if (p < 0) return false;
        int32 t = HdrStr.Find(TEXT("True"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
        int32 f = HdrStr.Find(TEXT("False"), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
        bFOrder = (t >= 0 && (f < 0 || t < f)); // True avant False
    }

    // shape: (N,) ou (N,1)
    {
        int32 p = HdrStr.Find(TEXT("shape"));
        if (p < 0) return false;
        int32 lp = HdrStr.Find(TEXT("("), ESearchCase::IgnoreCase, ESearchDir::FromStart, p);
        int32 rp = HdrStr.Find(TEXT(")"), ESearchCase::IgnoreCase, ESearchDir::FromStart, lp + 1);
        if (lp < 0 || rp < 0) return false;
        const FString tuple = HdrStr.Mid(lp + 1, rp - lp - 1); // ex: "1723599, 1"
        // récupère 1er entier
        int64 n0 = 0, n1 = 1;
        {
            FString A = tuple; A.ReplaceInline(TEXT(" "), TEXT(""));
            TArray<FString> parts; A.ParseIntoArray(parts, TEXT(","), /*CullEmpty=*/true);
            if (parts.Num() >= 1) n0 = FCString::Atoi64(*parts[0]);
            if (parts.Num() >= 2 && !parts[1].IsEmpty()) n1 = FCString::Atoi64(*parts[1]);
            if (n1 <= 0) n1 = 1;
        }
        OutCount = n0 * n1;
    }

    OutDataOffset = 8 + ((vMajor == 1) ? 2 : 4) + headerLen;
    return true;
}

bool VTUCore::LoadNPY_CellFeatures(const FString& NpyPath, int32 NumCellsExpected, FVTUCellFeatures& Out)
{
    Out.Reset();
    if (NumCellsExpected <= 0) { UE_LOG(LogTemp, Error, TEXT("NPY: NumCellsExpected <= 0")); return false; }

    IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
    TUniquePtr<IFileHandle> H(PF.OpenRead(*NpyPath));
    if (!H) { UE_LOG(LogTemp, Error, TEXT("NPY: cannot open %s"), *NpyPath); return false; }

    int64 dataOffset = 0, count = 0; bool little = false, fOrder = false; FString dtype;
    if (!Npy_ReadHeader(H.Get(), dataOffset, count, little, fOrder, dtype)) {
        UE_LOG(LogTemp, Error, TEXT("NPY: header parse failed"));
        return false;
    }
    if (!little || !dtype.Contains(TEXT("f8"))) {
        UE_LOG(LogTemp, Error, TEXT("NPY: only little-endian '<f8' supported (got %s)"), *dtype);
        return false;
    }
    if (fOrder) {
        UE_LOG(LogTemp, Error, TEXT("NPY: fortran_order=True not supported"));
        return false;
    }

    const int64 needed = int64(NumCellsExpected) * 9; // 9 features
    if (count != needed) {
        UE_LOG(LogTemp, Error, TEXT("NPY: count mismatch. got=%lld expected=%lld (NumCells=%d * 9)"),
            count, needed, NumCellsExpected);
        return false;
    }

    const int64 dataBytes = count * 8; // f8
    const int64 fileSize = H->Size();
    if (dataOffset + dataBytes > fileSize) {
        UE_LOG(LogTemp, Error, TEXT("NPY: truncated file"));
        return false;
    }

    // Lecture en une fois + conversion double->float
    TArray<uint8> buf; buf.SetNumUninitialized(dataBytes);
    H->Seek(dataOffset);
    if (!H->Read(buf.GetData(), dataBytes)) {
        UE_LOG(LogTemp, Error, TEXT("NPY: failed to read data block"));
        return false;
    }
    const double* D = reinterpret_cast<const double*>(buf.GetData());

    // Allocation des 9 buffers (float)
    Out.SetNum(NumCellsExpected);

    // Découpe par features: [NOx(0..N-1), CO(N..2N-1), ..., T(8N..9N-1)]
    const int32 N = NumCellsExpected;
    auto copyBlock = [&](float* dst, const double* src) {
        for (int32 i = 0; i < N; ++i) dst[i] = (float)src[i];
        };

    copyBlock(Out.NOx.GetData(), D + 0 * N);
    copyBlock(Out.CO.GetData(), D + 1 * N);
    copyBlock(Out.OH.GetData(), D + 2 * N);
    copyBlock(Out.H2.GetData(), D + 3 * N);
    copyBlock(Out.H2O.GetData(), D + 4 * N);
    copyBlock(Out.CO2.GetData(), D + 5 * N);
    copyBlock(Out.O2.GetData(), D + 6 * N);
    copyBlock(Out.CH4.GetData(), D + 7 * N);
    copyBlock(Out.T.GetData(), D + 8 * N);

    UE_LOG(LogTemp, Display, TEXT("NPY: loaded features OK (%d cells, 9 arrays)"), N);
    return true;
}


//------------------------------------------------------------------------------------------------Color Feature-----------------------------------------------------------------------------------------------
// === Color map simple bleu->rouge ===
static inline float Clamp01(float x){ return FMath::Clamp(x, 0.f, 1.f); }
static FLinearColor ScalarToColor(float v, float vmin, float vmax)
{
  if (!FMath::IsFinite(v)) return FLinearColor::Black;
  if (vmax <= vmin)        return FLinearColor::Gray;
  const float t = Clamp01((v - vmin) / (vmax - vmin));
  // Bleu -> Rouge avec un vert au milieu (visual simple)
  return FLinearColor(t, 1.f - FMath::Abs(2*t - 1.f), 1.f - t, 1.f);
}

// Clé de face (ordre insensible) pour détection des frontières
struct FFaceKey
{
  // on stocke une version triée des indices de la face
  TArray<int32> V; // trié croissant
  bool operator==(const FFaceKey& o) const
  {
    if (V.Num()!=o.V.Num()) return false;
    for (int32 i=0;i<V.Num();++i) if (V[i]!=o.V[i]) return false;
    return true;
  }
};
static uint32 GetTypeHash(const FFaceKey& K)
{
  uint32 h = 1469598103u;
  for (int32 x : K.V) { h = (h ^ ::GetTypeHash(x)) * 16777619u; }
  return h;
}

bool VTUCore::BuildSurfaceToPMC_WithCellScalars(const FVTUGrid& G,
                                                UProceduralMeshComponent* PMC,
                                                float ScaleCm,
                                                const TArray<float>& CellScalars,
                                                float VMin, float VMax,
                                                int32 SectionIndex)
{
  if (!PMC) return false;
  if (!G.HasPolyFaces()) {
    UE_LOG(LogTemp, Warning, TEXT("BuildSurfaceToPMC_WithCellScalars: no poly faces; returning false"));
    return false;
  }
  if (CellScalars.Num() != G.NumCells()) {
    UE_LOG(LogTemp, Error, TEXT("BuildSurfaceToPMC_WithCellScalars: CellScalars size mismatch (%d vs %d)"),
           CellScalars.Num(), G.NumCells());
    return false;
  }

  // 1) Première passe : parcourir toutes les faces de toutes les cellules, détecter les frontières
  struct FFaceRec
  {
    int32 Cell = -1;          // cellule porteuse (si frontière, une seule)
    TArray<int32> FaceVerts;  // ordre original (pour triangulation)
    bool bShared = false;     // si true => face interne (vue 2 fois)
  };

  TMap<FFaceKey, FFaceRec> FaceMap;
  FaceMap.Reserve(FMath::Min(G.Faces.Num()/3, 800000)); // heuristique

  int32 cursor = 0;
  for (int32 ci=0; ci<G.NumCells(); ++ci)
  {
    const int32 endF = G.FaceOffsets[ci];
    if (endF <= cursor) { cursor = endF; continue; }

    if (cursor >= G.Faces.Num()) break;
    const int32 nFaces = G.Faces[cursor++];

    for (int32 f=0; f<nFaces; ++f)
    {
      if (cursor >= endF) break;
      const int32 nv = G.Faces[cursor++];

      if (nv <= 0 || cursor + nv > endF) { cursor = endF; break; }

      // lire les indices de sommets de la face
      const int32* fv = G.Faces.GetData() + cursor;
      cursor += nv;

      // clé triée (ordre insensible)
      FFaceKey K;
      K.V.SetNumUninitialized(nv);
      for (int32 k=0;k<nv;++k) K.V[k] = fv[k];
      K.V.Sort();

      FFaceRec* Rec = FaceMap.Find(K);
      if (!Rec)
      {
        FFaceRec R;
        R.Cell = ci;
        R.FaceVerts.SetNumUninitialized(nv);
        // on copie l'ordre ORIGINAL (pour trianguler en fan)
        for (int32 k=0;k<nv;++k) R.FaceVerts[k] = fv[k];
        FaceMap.Add(MoveTemp(K), MoveTemp(R));
      }
      else
      {
        // déjà vue : face interne -> marquer comme partagée
        Rec->bShared = true;
      }
    }
  }

  // 2) Construire les triangles pour les faces frontières (non partagées)
  TArray<FVector>         Vertices;
  TArray<int32>           Indices;
  TArray<FLinearColor>    Colors;
  TArray<FVector>         Normals;
  TArray<FVector2D>       UV0;
  TArray<FProcMeshTangent>Tangents;

  Vertices.Reserve(500000);
  Indices.Reserve(1000000);
  Colors.Reserve(500000);

  for (auto& It : FaceMap)
  {
    const FFaceRec& R = It.Value;
    if (R.bShared || R.Cell<0 || R.FaceVerts.Num()<3) continue; // interne ou invalide

    const int32 base = Vertices.Num();
    const float val  = CellScalars[R.Cell];
    const FLinearColor col = ScalarToColor(val, VMin, VMax);

    // ajouter les sommets (dans l'ordre face)
    for (int32 idx : R.FaceVerts)
    {
      const FVector P = FVector(G.Points[idx]) * ScaleCm;
      Vertices.Add(P);
      Colors.Add(col);
    }

    // triangulation en fan : (0,k,k+1)
    for (int32 k=1; k+1 < R.FaceVerts.Num(); ++k)
    {
      Indices.Add(base + 0);
      Indices.Add(base + k);
      Indices.Add(base + k + 1);
    }
  }

  if (Indices.Num() < 3)
  {
    UE_LOG(LogTemp, Warning, TEXT("BuildSurfaceToPMC_WithCellScalars: no boundary triangles"));
    PMC->ClearMeshSection(SectionIndex);
    return false;
  }

  PMC->CreateMeshSection_LinearColor(SectionIndex, Vertices, Indices, Normals, UV0, Colors, Tangents, /*collision*/false);
  PMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  return true;
}