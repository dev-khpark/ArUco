using System;
using System.Collections.Generic;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ObjdetectModule;
using OpenCVForUnity.UnityUtils;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// Physical stand-in for the real table: a board with corner markers and seven tangram pieces with
/// ArUco stickers, built at real-world scale far away from the game scene.
///
/// Board coordinates (u, v) are seen from the player: u = player's right, v = away from the player
/// (toward the device). They map to the sim root's local (x, z); y is up from the table.
/// Piece shapes come from the answer diagram meshes, and each sticker sits on the diagram's 'center'
/// child with marker +X along 'center'→'direction' — the same convention the matcher uses, so this is
/// also the sticker placement guide for the real pieces.
/// </summary>
public sealed class SimTangramWorld
{
    public sealed class Piece
    {
        public TangramMatcher.TangramShapeType type;
        public int markerId;
        public Transform transform;
        public Vector2[] outline; // local meters, sticker origin at (0,0), +X = guide direction
        public Transform sticker;
        public Vector2 stickerOffset;
        public float stickerAngleDeg;
        public Vector2 pos;
        public float angleDeg;
        public bool solved;
    }

    public struct Target
    {
        public TangramMatcher.TangramShapeType type;
        public Vector2 pos;
        public float angleDeg;
        public Transform piece; // the answer-diagram piece this slot belongs to
    }

    public Transform Root { get; private set; }
    public readonly List<Piece> Pieces = new List<Piece>();
    public Transform Hand { get; private set; }
    public float BoardSize { get; private set; }
    public float PieceThickness { get; private set; }
    private readonly List<Vector2> cornerCenters = new List<Vector2>();
    private readonly List<Transform> cornerStickers = new List<Transform>();
    private readonly List<Vector2> cornerBase = new List<Vector2>();
    private float cornerClearance;

    private const float BoardHeight = 0.0004f; // above the table to avoid z-fighting
    private Dictionary dictionary;
    private Shader litShader;

    /// <summary>
    /// Builds the world. Returns warnings (e.g. a sticker that overhangs its piece) for the caller to log.
    /// </summary>
    public List<string> Build(Transform parent, TangramMatcher matcher, SimSetup setup)
    {
        var warnings = new List<string>();
        dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50);
        litShader = Shader.Find("Universal Render Pipeline/Lit");
        if (litShader == null) litShader = Shader.Find("Standard");
        BoardSize = setup.boardSizeMeters;
        PieceThickness = setup.pieceThicknessMeters;

        Root = new GameObject("SimTable").transform;
        Root.SetParent(parent, false);

        var table = Quad("Table", Root, new Vector3(0f, 0f, 0f), 1.4f, MakeMaterial(new Color(0.55f, 0.42f, 0.3f), 0.15f, MakeWoodTexture()));
        table.GetComponent<Renderer>().sharedMaterial.mainTextureScale = new Vector2(6f, 6f);
        Quad("Board", Root, new Vector3(0f, BoardHeight, 0f), setup.boardSizeMeters, MakeMaterial(new Color(0.93f, 0.93f, 0.9f), 0.05f, null));

        foreach (var (id, pos) in CornerLayout(setup))
        {
            cornerStickers.Add(Sticker(Root, id, setup));
            cornerBase.Add(pos);
        }
        SetBoardRotation(setup.boardRotationDeg);
        cornerClearance = setup.markerSizeMeters * (0.5f + setup.stickerQuietZoneRatio) * 1.42f + 0.005f;

        var shapes = ReadDiagramShapes(matcher, setup.setSizeMeters, out _);
        var ids = AssignMarkerIds(shapes);
        for (int i = 0; i < shapes.Count; i++)
        {
            var shape = shapes[i];
            int id = ids[i];
            var go = new GameObject($"Sim_{shape.name}_id{id}");
            go.transform.SetParent(Root, false);
            go.AddComponent<MeshFilter>().sharedMesh = Prism(shape.outline, setup.pieceThicknessMeters);
            go.AddComponent<MeshRenderer>().sharedMaterial = MakeMaterial(shape.color, 0.35f, null);
            var piece = new Piece { type = shape.type, markerId = id, transform = go.transform, outline = shape.outline };
            piece.sticker = Sticker(go.transform, id, setup);
            Pieces.Add(piece);

            float half = setup.markerSizeMeters * (0.5f + setup.stickerQuietZoneRatio);
            if (!SquareFits(shape.outline, half, setup.stickerRotationOffsetDeg))
                warnings.Add($"{shape.name}: a {setup.markerSizeMeters * 100f:0.#}cm marker sticker overhangs the piece at this set size. Use a smaller marker or a larger set.");
        }

        Hand = BuildHand(Root);
        Hand.gameObject.SetActive(false);
        RestickMarkers(setup);
        Scatter(setup);
        return warnings;
    }

    // ---------- Arrangement ----------

    /// <summary>Rotates the corner-marker layout (which corner is TL/TR/…) relative to the player's view.</summary>
    public void SetBoardRotation(float deg)
    {
        cornerCenters.Clear();
        for (int i = 0; i < cornerStickers.Count; i++)
        {
            Vector2 p = Rotate(cornerBase[i], deg);
            cornerStickers[i].localPosition = new Vector3(p.x, BoardHeight + 0.0002f, p.y);
            cornerStickers[i].localRotation = Quaternion.Euler(0f, -deg, 0f) * Quaternion.Euler(90f, 0f, 0f);
            cornerCenters.Add(p);
        }
    }

    public void SetPose(Piece p, Vector2 pos, float angleDeg)
    {
        p.pos = pos;
        p.angleDeg = angleDeg;
        // Tiny per-piece lift so overlapping pieces stack instead of z-fighting.
        p.transform.localPosition = new Vector3(pos.x, BoardHeight + 0.0001f + 0.0002f * Pieces.IndexOf(p), pos.y);
        // Unity's +Y rotation is clockwise seen from above, while angleDeg is counter-clockwise in (u, v).
        p.transform.localRotation = Quaternion.Euler(0f, -angleDeg, 0f);
    }

    public void Scatter(SimSetup setup)
    {
        float r = BoardSize * 0.5f - 0.03f;
        var placed = new List<Vector2>();
        foreach (var p in Pieces)
        {
            float radius = 0f;
            foreach (var o in p.outline) radius = Mathf.Max(radius, o.magnitude);
            Vector2 best = Vector2.zero; float bestGap = -1f;
            for (int attempt = 0; attempt < 60; attempt++)
            {
                var c = new Vector2(Random.Range(-r, r), Random.Range(-r, r));
                // A player doesn't park pieces on the board's corner markers (that would just hide the board).
                bool coversCorner = false;
                foreach (var cc in cornerCenters) if (Vector2.Distance(c, cc) < radius + cornerClearance) { coversCorner = true; break; }
                if (coversCorner) continue;
                float gap = float.MaxValue;
                foreach (var q in placed) gap = Mathf.Min(gap, Vector2.Distance(c, q));
                if (gap > bestGap) { bestGap = gap; best = c; }
                if (gap > setup.setSizeMeters * 0.38f) break;
            }
            placed.Add(best);
            SetPose(p, best, Random.Range(0f, 360f));
            p.solved = false;
        }
    }

    /// <summary>Moves one unsolved piece (or all) to its answer pose with hand-placement noise.</summary>
    public int Solve(TangramMatcher matcher, SimSetup setup, bool all)
    {
        var targets = ComputeTargets(matcher, setup.setSizeMeters);
        var used = new bool[targets.Count];
        if (all) foreach (var p in Pieces) p.solved = false;
        // Same-type pieces are interchangeable: keep the slots already taken by solved pieces.
        foreach (var p in Pieces)
            if (p.solved) { int ti = NearestFreeTarget(targets, used, p); if (ti >= 0) used[ti] = true; }

        var order = new List<Piece>(Pieces);
        if (!all) Shuffle(order);
        int moved = 0;
        foreach (var p in order)
        {
            if (p.solved) continue;
            int ti = NearestFreeTarget(targets, used, p);
            if (ti < 0) continue;
            used[ti] = true;
            Vector2 noise = Random.insideUnitCircle * setup.placementNoiseMm * 0.001f;
            float angNoise = Random.Range(-1f, 1f) * setup.placementNoiseDeg;
            SetPose(p, targets[ti].pos + noise, targets[ti].angleDeg + angNoise);
            p.solved = true;
            moved++;
            if (!all) break;
        }
        return moved;
    }

    private static int NearestFreeTarget(List<Target> targets, bool[] used, Piece p)
    {
        int best = -1; float bestD = float.MaxValue;
        for (int i = 0; i < targets.Count; i++)
        {
            if (used[i] || targets[i].type != p.type) continue;
            float d = (targets[i].pos - p.pos).sqrMagnitude;
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    /// <summary>Re-applies stickers with fresh placement error (like peeling and re-sticking by hand).</summary>
    public void RestickMarkers(SimSetup setup)
    {
        foreach (var p in Pieces)
        {
            p.stickerOffset = Random.insideUnitCircle * setup.stickerErrorMm * 0.001f;
            p.stickerAngleDeg = setup.stickerRotationOffsetDeg + Random.Range(-1f, 1f) * setup.stickerErrorDeg;
            p.sticker.localPosition = new Vector3(p.stickerOffset.x, setup.pieceThicknessMeters + 0.0002f, p.stickerOffset.y);
            p.sticker.localRotation = Quaternion.Euler(0f, -p.stickerAngleDeg, 0f) * Quaternion.Euler(90f, 0f, 0f);
        }
    }

    public Piece PickPiece(Vector2 boardPoint)
    {
        Piece best = null;
        float bestD = float.MaxValue;
        foreach (var p in Pieces)
        {
            Vector2 local = Rotate(boardPoint - p.pos, -p.angleDeg);
            if (PointInConvex(local, p.outline))
            {
                float d = local.sqrMagnitude;
                if (d < bestD) { bestD = d; best = p; }
            }
        }
        return best;
    }

    /// <summary>Hovers a simple hand + forearm over a board point; the arm comes from the player's side.</summary>
    public void PlaceHand(Vector2 boardPoint, float height)
    {
        Hand.localPosition = new Vector3(boardPoint.x, height, boardPoint.y);
    }

    // ---------- Layout shared with the printable guide (StickerGuideExporter) ----------

    /// <summary>
    /// Corner marker centers 10..13 = TL, TR, BR, BL from the player's view, in board (u, v) meters before
    /// boardRotationDeg. Same size as the piece markers; the sticker edge sits 6 mm inside the board edge.
    /// </summary>
    public static (int id, Vector2 pos)[] CornerLayout(SimSetup setup)
    {
        float inset = setup.markerSizeMeters * (0.5f + setup.stickerQuietZoneRatio) + 0.006f;
        float h = setup.boardSizeMeters * 0.5f - inset;
        return new[] { (10, new Vector2(-h, h)), (11, new Vector2(h, h)), (12, new Vector2(h, -h)), (13, new Vector2(-h, -h)) };
    }

    /// <summary>ArUco ID per shape, in diagram order. Same table as ArUcoMobileTracker.TryMapArucoIdToShape (7 stays unused).</summary>
    public static List<int> AssignMarkerIds(List<ShapeInfo> shapes)
    {
        var nextId = new Dictionary<TangramMatcher.TangramShapeType, int>
        {
            { TangramMatcher.TangramShapeType.LargeTriangle, 0 },
            { TangramMatcher.TangramShapeType.MediumTriangle, 2 },
            { TangramMatcher.TangramShapeType.SmallTriangle, 3 },
            { TangramMatcher.TangramShapeType.Square, 5 },
            { TangramMatcher.TangramShapeType.Parallelogram, 6 },
        };
        var ids = new List<int>();
        foreach (var s in shapes) ids.Add(nextId[s.type]++);
        return ids;
    }

    // ---------- Diagram → physical shapes ----------

    public struct ShapeInfo
    {
        public string name;
        public TangramMatcher.TangramShapeType type;
        public Vector2[] outline;
        public Color color;
        public Vector2 center;   // diagram units
        public float angleDeg;
        public Transform piece;
    }

    /// <summary>Answer poses in board meters for the matcher's current diagram, centered on the board.</summary>
    public static List<Target> ComputeTargets(TangramMatcher matcher, float setSizeMeters)
    {
        var shapes = ReadDiagramShapes(matcher, setSizeMeters, out float scale);
        var result = new List<Target>();
        if (shapes.Count == 0) return result;
        Vector2 min = new Vector2(float.MaxValue, float.MaxValue), max = new Vector2(float.MinValue, float.MinValue);
        foreach (var s in shapes)
        {
            foreach (var o in s.outline)
            {
                Vector2 w = s.center * scale + Rotate(o, s.angleDeg);
                min = Vector2.Min(min, w); max = Vector2.Max(max, w);
            }
        }
        Vector2 mid = (min + max) * 0.5f;
        foreach (var s in shapes)
            result.Add(new Target { type = s.type, pos = s.center * scale - mid, angleDeg = s.angleDeg, piece = s.piece });
        return result;
    }

    private static List<ShapeInfo> ReadDiagramShapes(TangramMatcher matcher, float setSizeMeters, out float scale)
        => ReadDiagramShapes(matcher, matcher.GetDiagramPieces(), setSizeMeters, out scale);

    /// <summary>
    /// Projects each diagram piece onto the diagram plane (world XY, as the matcher's useXYPlane), takes the
    /// convex hull and expresses it in the piece's sticker frame. Scale makes the assembled set setSizeMeters wide
    /// (the seven pieces always tile a square of side S, so S = sqrt(total area)).
    /// Pass the diagram root's children as <paramref name="pieces"/> in Edit mode, where the matcher has no cache yet.
    /// </summary>
    public static List<ShapeInfo> ReadDiagramShapes(TangramMatcher matcher, IEnumerable<Transform> pieces, float setSizeMeters, out float scale)
    {
        var list = new List<ShapeInfo>();
        float area = 0f;
        foreach (var t in pieces)
        {
            if (!TangramMatcher.TryGetShapeTypeFromPieceName(t.name, out var type)) continue;
            var mf = t.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) continue;
            var pts = new List<Vector2>();
            foreach (var v in mf.sharedMesh.vertices) { Vector3 w = t.TransformPoint(v); pts.Add(new Vector2(w.x, w.y)); }
            var hull = ConvexHull(pts);
            area += PolygonArea(hull);

            Vector3 c3 = matcher.GetPieceCenter(t);
            Vector2 c = new Vector2(c3.x, c3.y);
            var dirT = t.Find("direction");
            Vector2 dir = dirT != null ? new Vector2(dirT.position.x, dirT.position.y) - c : new Vector2(t.right.x, t.right.y);
            float ang = Mathf.Atan2(dir.y, dir.x) * Mathf.Rad2Deg;

            var r = t.GetComponent<Renderer>();
            var m = r != null ? r.sharedMaterial : null;
            Color col = m != null && m.HasProperty("_BaseColor") ? m.GetColor("_BaseColor") : Color.white;
            list.Add(new ShapeInfo { name = t.name, type = type, outline = hull.ToArray(), color = col, center = c, angleDeg = ang, piece = t });
        }
        scale = area > 1e-12f ? setSizeMeters / Mathf.Sqrt(area) : 1f;
        for (int i = 0; i < list.Count; i++)
        {
            var s = list[i];
            var local = new Vector2[s.outline.Length];
            for (int k = 0; k < local.Length; k++) local[k] = Rotate((s.outline[k] - s.center) * scale, -s.angleDeg);
            s.outline = local;
            list[i] = s;
        }
        return list;
    }

    // ---------- Geometry helpers ----------

    public static Vector2 Rotate(Vector2 p, float deg)
    {
        float r = deg * Mathf.Deg2Rad, c = Mathf.Cos(r), s = Mathf.Sin(r);
        return new Vector2(c * p.x - s * p.y, s * p.x + c * p.y);
    }

    /// <summary>Andrew's monotone chain; counter-clockwise, no duplicate end point.</summary>
    private static List<Vector2> ConvexHull(List<Vector2> pts)
    {
        pts.Sort((a, b) => a.x != b.x ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y));
        var hull = new List<Vector2>();
        for (int pass = 0; pass < 2; pass++)
        {
            int start = hull.Count;
            for (int i = 0; i < pts.Count; i++)
            {
                var p = pts[pass == 0 ? i : pts.Count - 1 - i];
                while (hull.Count >= start + 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 1e-12f) hull.RemoveAt(hull.Count - 1);
                hull.Add(p);
            }
            hull.RemoveAt(hull.Count - 1);
        }
        return hull;
    }

    private static float Cross(Vector2 o, Vector2 a, Vector2 b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);

    private static float PolygonArea(List<Vector2> poly)
    {
        float a = 0f;
        for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++) a += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
        return Mathf.Abs(a) * 0.5f;
    }

    public static bool PointInConvex(Vector2 p, Vector2[] ccw)
    {
        for (int i = 0; i < ccw.Length; i++)
            if (Cross(ccw[i], ccw[(i + 1) % ccw.Length], p) < 0f) return false;
        return true;
    }

    public static bool SquareFits(Vector2[] outline, float half, float angleDeg)
    {
        foreach (var c in new[] { new Vector2(-half, -half), new Vector2(half, -half), new Vector2(half, half), new Vector2(-half, half) })
            if (!PointInConvex(Rotate(c, angleDeg), outline)) return false;
        return true;
    }

    private static void Shuffle<T>(List<T> list)
    {
        for (int i = list.Count - 1; i > 0; i--) { int j = Random.Range(0, i + 1); (list[i], list[j]) = (list[j], list[i]); }
    }

    // ---------- Object builders ----------

    private Transform Quad(string name, Transform parent, Vector3 localPos, float size, Material mat)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Quad);
        go.name = name;
        UnityEngine.Object.Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.Euler(90f, 0f, 0f); // face up; texture up = +v (away from player)
        go.transform.localScale = new Vector3(size, size, 1f);
        go.GetComponent<Renderer>().sharedMaterial = mat;
        return go.transform;
    }

    /// <summary>White paper sticker with the marker in the middle and a quiet zone around it.</summary>
    private Transform Sticker(Transform parent, int id, SimSetup setup)
    {
        int markerPx = 120, quietPx = Mathf.RoundToInt(markerPx * setup.stickerQuietZoneRatio);
        using (var marker = new Mat())
        using (var sticker = new Mat())
        {
            Objdetect.generateImageMarker(dictionary, id, markerPx, marker, 1);
            Core.copyMakeBorder(marker, sticker, quietPx, quietPx, quietPx, quietPx, Core.BORDER_CONSTANT, new Scalar(255));
            var tex = new Texture2D(sticker.cols(), sticker.rows(), TextureFormat.RGBA32, true);
            // The camera sees the sticker small, so lower mip levels are what gets sampled: fill them too.
            Utils.matToTexture2D(sticker, tex, true, 0, true);
            tex.filterMode = FilterMode.Trilinear;
            tex.anisoLevel = 8;
            var t = Quad($"Marker_{id}", parent, Vector3.zero, setup.markerSizeMeters * (1f + 2f * setup.stickerQuietZoneRatio),
                MakeMaterial(Color.white, setup.stickerSmoothness, tex));
            return t;
        }
    }

    private Transform BuildHand(Transform parent)
    {
        var skin = MakeMaterial(new Color(0.87f, 0.68f, 0.56f), 0.25f, null);
        var root = new GameObject("Hand").transform;
        root.SetParent(parent, false);
        var palm = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        UnityEngine.Object.Destroy(palm.GetComponent<Collider>());
        palm.transform.SetParent(root, false);
        palm.transform.localScale = new Vector3(0.085f, 0.03f, 0.1f);
        palm.GetComponent<Renderer>().sharedMaterial = skin;
        var arm = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        UnityEngine.Object.Destroy(arm.GetComponent<Collider>());
        arm.transform.SetParent(root, false);
        arm.transform.localPosition = new Vector3(0f, 0.03f, -0.16f);
        arm.transform.localRotation = Quaternion.Euler(-78f, 0f, 0f); // reaches in from the player's side, rising toward the elbow
        arm.transform.localScale = new Vector3(0.065f, 0.12f, 0.06f);
        arm.GetComponent<Renderer>().sharedMaterial = skin;
        return root;
    }

    private static Mesh Prism(Vector2[] ccw, float thickness)
    {
        int n = ccw.Length;
        var verts = new List<Vector3>();
        var tris = new List<int>();
        for (int i = 0; i < n; i++) verts.Add(new Vector3(ccw[i].x, thickness, ccw[i].y));
        for (int i = 1; i < n - 1; i++) { tris.Add(0); tris.Add(i + 1); tris.Add(i); } // clockwise seen from above
        for (int i = 0; i < n; i++)
        {
            var a = ccw[i]; var b = ccw[(i + 1) % n];
            int s = verts.Count;
            verts.Add(new Vector3(a.x, thickness, a.y)); verts.Add(new Vector3(b.x, thickness, b.y));
            verts.Add(new Vector3(b.x, 0f, b.y)); verts.Add(new Vector3(a.x, 0f, a.y));
            tris.Add(s); tris.Add(s + 1); tris.Add(s + 2); tris.Add(s); tris.Add(s + 2); tris.Add(s + 3);
        }
        var mesh = new Mesh { name = "SimPiece" };
        mesh.SetVertices(verts);
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    private Material MakeMaterial(Color color, float smoothness, Texture tex)
    {
        var m = new Material(litShader);
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", color); else m.color = color;
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smoothness);
        if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", smoothness);
        if (tex != null)
        {
            if (m.HasProperty("_BaseMap")) m.SetTexture("_BaseMap", tex);
            m.mainTexture = tex;
        }
        return m;
    }

    private static Texture2D MakeWoodTexture()
    {
        var t = new Texture2D(128, 128, TextureFormat.RGBA32, true) { wrapMode = TextureWrapMode.Repeat };
        for (int y = 0; y < 128; y++)
            for (int x = 0; x < 128; x++)
            {
                float n = Mathf.PerlinNoise(x * 0.05f, y * 0.4f) * 0.35f + Mathf.PerlinNoise(x * 0.3f, y * 0.9f) * 0.1f;
                t.SetPixel(x, y, new Color(0.75f + n * 0.4f, 0.6f + n * 0.3f, 0.45f + n * 0.2f));
            }
        t.Apply(true);
        return t;
    }
}

/// <summary>Physical dimensions of the set, board and stickers (meters).</summary>
[Serializable]
public class SimSetup
{
    [Tooltip("Side of the square the seven pieces form when assembled.")]
    public float setSizeMeters = 0.15f;
    [Tooltip("Edge of the black marker square. Keep corner and piece markers the same size: the tracker assumes one markerLength.")]
    public float markerSizeMeters = 0.02f;
    [Tooltip("White margin around the marker on the sticker, per side, as a fraction of the marker size.")]
    public float stickerQuietZoneRatio = 0.2f;
    public float boardSizeMeters = 0.24f;
    [Tooltip("Rotation of the corner-marker layout (which corner is TL=10, TR=11, …) relative to the player's view.")]
    public float boardRotationDeg = 0f;
    public float pieceThicknessMeters = 0.004f;
    [Tooltip("0 = matte paper. Raise to simulate glossy stickers (glare at oblique angles washes out the black).")]
    [Range(0f, 1f)] public float stickerSmoothness = 0.05f;

    [Header("Sticker placement (the guide)")]
    [Tooltip("Sticker rotation relative to the piece's center→direction axis. 0 = marker +X points along it.")]
    public float stickerRotationOffsetDeg = 0f;
    [Tooltip("Hand-sticking error even with a printed guide (1σ-ish).")]
    public float stickerErrorMm = 1.0f;
    public float stickerErrorDeg = 2f;

    [Header("Player placement")]
    [Tooltip("How far from the exact answer pose a player typically sets a piece.")]
    public float placementNoiseMm = 1.5f;
    public float placementNoiseDeg = 2f;
}
