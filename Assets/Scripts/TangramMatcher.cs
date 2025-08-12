using System;
using System.Collections.Generic;
using UnityEngine;

#if OPENCV_FOR_UNITY
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.Calib3dModule;
#endif

/// <summary>
/// Matches detected Tangram shapes (from ArUco IDs) to the answer layout under a TangramDiagram root.
/// Highlights matched child objects for debugging.
/// 
/// How to use:
/// - Attach to a GameObject and assign `tangramDiagramRoot` (defaults to this.transform)
/// - Feed detections every frame via SetDetections(...) or UpdateFromArucoMats(...)
/// - The component will compute nearest-neighbor matches by shape type and color matched pieces.
/// </summary>
public class TangramMatcher : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Root Transform that contains the answer pieces as children (triangle_l_1, triangle_l_2, triangle_m, triangle_s_1, triangle_s_2, square, parallel)")]
    public Transform tangramDiagramRoot;

    [Tooltip("World transform of the physical camera for ArUco (used to convert camera-space tvec to world). If not assigned, world positions won't be computed from Mats.")]
    public Transform arUcoCameraTransform;

    [Header("Matching Settings")]
    [Tooltip("Maximum world distance (meters) allowed to consider a detection matched to a diagram piece.")]
    public float maxMatchDistanceMeters = 0.2f;
    [Tooltip("Maximum planar angle error (degrees) allowed for a match.")]
    public float maxAngleErrorDegrees = 45f;
    [Tooltip("Weight converting rotation error (degrees) into meters for matching cost. cost = distance(m) + rotationWeightMetersPerDeg * angle(deg)")]
    public float rotationWeightMetersPerDeg = 0.005f;
    // Anchor-relative matching removed

    [Header("Diagram Graph Settings")]
    [Tooltip("Adjacency threshold. Create an edge if center distance <= (sizeA+sizeB)*this factor.")]
    public float diagramAdjacencyMaxNormalizedDistance = 3.0f;
    [Tooltip("Maximum neighbor edges to keep per node (closest first). 0 = unlimited.")]
    public int diagramMaxNeighborsPerNode = 4;
    [Tooltip("Graph matching distance tolerance (meters).")]
    public float graphDistanceToleranceMeters = 0.5f;
    [Tooltip("Graph matching angle tolerance (degrees).")]
    public float graphAngleToleranceDeg = 45f;

    [Header("Graph-Only Matching (no absolute coordinates)")]
    
    [Tooltip("Detection graph adjacency threshold using nominal size per type. Create edge if distance <= (sizeA+sizeB)*this factor.")]
    public float detectionAdjacencyMaxNormalizedDistance = 3.0f;
    [Tooltip("Maximum neighbor edges to keep per detection node (closest first). 0 = unlimited.")]
    public int detectionMaxNeighborsPerNode = 4;
    [Tooltip("Nominal size (meters) used for detection graph normalization per shape type.")]
    public float nominalSizeLargeTriangle = 0.25f;
    public float nominalSizeMediumTriangle = 0.20f;
    public float nominalSizeSmallTriangle = 0.15f;
    public float nominalSizeSquare = 0.18f;
    public float nominalSizeParallelogram = 0.18f;

    [Header("Graph Debugging")]
    [Tooltip("Log the built TangramDiagram graph (nodes and edges) when it is rebuilt.")]
    public bool debugLogDiagramGraph = true;

    [Header("Debug Colors")]
    public Color matchedColor = Color.green;
    public Color unmatchedColor = new Color(1f, 1f, 1f, 1f); // keep original white by default
    [Tooltip("If true, only matched pieces will change color; unmatched remain untouched.")]
    public bool highlightOnlyMatches = true;
    [Tooltip("If true, matched piece color encodes error magnitude using a gradient from errorLowColor to errorHighColor.")]
    public bool colorByErrorMagnitude = true;
    [Tooltip("Color used for low error (near 0m).")]
    public Color errorLowColor = Color.green;
    [Tooltip("Color used for high error (>= errorMaxForColor meters).")]
    public Color errorHighColor = Color.red;
    [Tooltip("Error value in meters that maps to errorHighColor (values above are clamped).")]
    public float errorMaxForColor = 0.2f;

    [Header("Debug Output")]
    [Tooltip("Log per-type error statistics each frame (count/min/max/avg).")]
    public bool debugLogPerTypeErrorStats = true;
    [Tooltip("Draw a line between detection position and matched piece position (Scene view / gizmo-like runtime lines).")]
    public bool debugDrawErrorLines = true;
    [Tooltip("Overlay per-match error text (distance and angle) on screen at piece position.")]
    public bool showErrorLabelsUI = false;
    public Color uiLabelColor = Color.white;
    public int uiFontSize = 14;
    public Camera uiCamera;

    [Header("Near-Miss Debugging")]
    [Tooltip("Log near-miss candidates that failed relational tolerances to help tuning.")]
    public bool debugLogUnmatchedCandidates = true;
    [Tooltip("Max number of near-miss candidates to log per detection node.")]
    public int debugMaxUnmatchedCandidatesPerDetection = 3;
    [Tooltip("Draw lines for near-miss candidates (helps visualize how far they are from passing).")]
    public bool debugDrawNearMissLines = false;
    public Color nearMissLineColor = new Color(1f, 0.65f, 0f, 1f); // orange

    [Header("Graph Constraints")]
    [Tooltip("When building the diagram graph, keep only the single nearest neighbor edge for each node.")]
    public bool diagramOnlyNearestNeighbor = true;

    [Header("Relational Angle Constraints")]
    [Tooltip("Additionally require that pairwise angles between matched neighbors are preserved (relative to the center).")]
    public bool usePairwiseNeighborAngle = false;
    [Tooltip("Tolerance for average pairwise neighbor angle difference (degrees). Only used when usePairwiseNeighborAngle is true.")]
    public float pairwiseAngleToleranceDeg = 30f;
    [Tooltip("Log pairwise neighbor angle checks for debugging.")]
    public bool debugLogPairwiseAngles = true;

    [Header("Color Application")]
    [Tooltip("If set, use this shader color property name (e.g., _BaseColor, _Color). If empty, auto-detect common names.")]
    public string overrideColorProperty = "";
    [Tooltip("If no color property is found on the material, assign Renderer.material.color as a fallback (instantiates material).")]
    public bool fallbackUseMaterialColorWhenNoProperty = true;

    [Header("Neutral Color Mode")]
    [Tooltip("If true, all diagram pieces are set to a neutral color initially, and only matched pieces restore their original colors.")]
    public bool useNeutralColorMode = true;
    [Tooltip("Neutral color applied to all unmatched pieces.")]
    public Color neutralColor = Color.gray;
    [Tooltip("When using neutral color mode, matched pieces restore their captured original color instead of debug colors.")]
    public bool matchedRestoresOriginalColor = true;

    [Header("Projection & Scale Normalization")]
    [Tooltip("Project all relation computations onto the diagram plane (reduces perspective effects).")]
    public bool usePlanarProjectionForRelations = true;
    [Tooltip("When comparing distances, estimate and apply a scalar scale factor from detection neighbors to diagram neighbors to reduce perspective-induced scale error.")]
    public bool useScaleNormalization = true;
    [Tooltip("Use only the nearest already-matched neighbor to compute relation cost (simpler and more stable with sparse links).")]
    public bool useOnlyNearestMatchedNeighbor = true;

    [Header("Temporal Smoothing")]
    [Tooltip("Stabilize matches by aggregating the last N frames before deciding final highlight.")]
    public bool useTemporalSmoothing = true;
    [Tooltip("Number of frames to aggregate.")]
    public int temporalWindowSize = 5;
    [Tooltip("Minimum votes within the window required to accept a piece as matched (per type). 0 = disabled (always pick top-K by votes).")]
    public int temporalMinVotes = 0;

    /// <summary>
    /// Types of Tangram shapes supported.
    /// </summary>
    public enum TangramShapeType
    {
        LargeTriangle = 0,
        MediumTriangle = 1,
        SmallTriangle = 2,
        Square = 3,
        Parallelogram = 4
    }

    /// <summary>
    /// A single detection input item. Positions are in world space (Unity) if provided.
    /// </summary>
    [Serializable]
    public struct DetectedShape
    {
        public int arucoId;
        public TangramShapeType shapeType;
        public Vector3 worldPosition;
        public Quaternion worldRotation;
        public float confidence; // 0..1 optional
    }

    /// <summary>
    /// Result of a single match between a detection and a diagram piece.
    /// </summary>
    [Serializable]
    public struct MatchResult
    {
        public TangramShapeType shapeType;
        public int arucoId;
        public Transform matchedPieceTransform;
        public float worldDistanceMeters;
        public Vector3 detectionWorldPosition;
        public float angleErrorDegrees;
    }

    // Cached diagram children grouped by type
    private readonly Dictionary<TangramShapeType, List<Transform>> diagramPiecesByType = new Dictionary<TangramShapeType, List<Transform>>();

    // Latest detections fed by caller
    private readonly List<DetectedShape> latestDetections = new List<DetectedShape>();

    // Public read-only results for external debug
    public IReadOnlyList<MatchResult> LastMatchResults => lastMatchResults;
    private readonly List<MatchResult> lastMatchResults = new List<MatchResult>();

    // Error statistics per type (last frame)
    [Serializable]
    public struct ErrorStats
    {
        public int count;
        public float min;
        public float max;
        public float avg;
    }
    private readonly Dictionary<TangramShapeType, ErrorStats> lastTypeErrorStats = new Dictionary<TangramShapeType, ErrorStats>();
    public IReadOnlyDictionary<TangramShapeType, ErrorStats> LastTypeErrorStats => lastTypeErrorStats;

    // MaterialPropertyBlock to avoid material instantiation
    private MaterialPropertyBlock sharedPropertyBlock;
    private GUIStyle cachedLabelStyle;
    
    // Temporal memory: for each piece, keep a circular buffer of votes
    private class TemporalVotes
    {
        public int[] buffer;
        public int index;
        public int count;
    }
    private readonly Dictionary<Transform, TemporalVotes> votesByPiece = new Dictionary<Transform, TemporalVotes>();
    private int temporalFrameCounter = 0;
    
    // Original color cache per renderer/material index
    private class SubMaterialColorInfo
    {
        public string propertyName; // null if none
        public Color originalColor;
        public bool hasColorProperty;
        public Color fallbackOriginalColor;
    }
    private readonly Dictionary<MeshRenderer, List<SubMaterialColorInfo>> originalColorsByRenderer = new Dictionary<MeshRenderer, List<SubMaterialColorInfo>>();

    // Diagram graph
    private class DiagramGraph
    {
        public class Edge
        {
            public int toIndex;
            public float expectedDistanceMeters;
            public float expectedAngleDeg;
            public float normalizedDistance;
        }
        public class Node
        {
            public Transform piece;
            public float sizeMeters;
            public readonly List<Edge> edges = new List<Edge>();
        }
        public readonly List<Node> nodes = new List<Node>();
        public readonly Dictionary<Transform, int> indexByTransform = new Dictionary<Transform, int>();
    }
    private DiagramGraph diagramGraph = new DiagramGraph();

    void Awake()
    {
        if (tangramDiagramRoot == null)
        {
            tangramDiagramRoot = transform;
        }

        sharedPropertyBlock = new MaterialPropertyBlock();
        RebuildDiagramCache();
    }

    void OnEnable()
    {
        RebuildDiagramCache();
    }

    /// <summary>
    /// Re-scan the `tangramDiagramRoot` children and group them by inferred shape type.
    /// Child naming convention expected: triangle_l_1, triangle_l_2, triangle_m, triangle_s_1, triangle_s_2, square, parallel
    /// </summary>
    public void RebuildDiagramCache()
    {
        diagramPiecesByType.Clear();
        foreach (TangramShapeType type in Enum.GetValues(typeof(TangramShapeType)))
        {
            diagramPiecesByType[type] = new List<Transform>();
        }

        if (tangramDiagramRoot == null)
            return;

        for (int i = 0; i < tangramDiagramRoot.childCount; i++)
        {
            Transform child = tangramDiagramRoot.GetChild(i);
            if (child == null)
                continue;

            if (!HasRenderableMesh(child.gameObject))
                continue;

            if (TryInferShapeTypeFromName(child.name, out TangramShapeType type))
            {
                diagramPiecesByType[type].Add(child);
            }
        }

        // Build diagram graph from current children
        BuildDiagramGraph();

        if (debugLogDiagramGraph)
        {
            DebugLogDiagramGraph();
        }

        // Capture original colors for all pieces
        CaptureOriginalColors();

        // Apply neutral color to all pieces at start if enabled
        if (useNeutralColorMode)
        {
            foreach (var kv in diagramPiecesByType)
            {
                foreach (Transform piece in kv.Value)
                {
                    SetRendererColor(piece.gameObject, neutralColor);
                }
            }
        }
    }

    /// <summary>
    /// Feed detections in world space. Triggers matching and debug coloring.
    /// </summary>
    public void SetDetections(List<DetectedShape> detections)
    {
        latestDetections.Clear();
        if (detections != null)
            latestDetections.AddRange(detections);

        RunMatchingAndHighlight();
    }

#if OPENCV_FOR_UNITY
    /// <summary>
    /// Convenience: feed detections from ArUco Mats directly. Requires `arUcoCameraTransform`.
    /// Positions are converted from OpenCV camera coordinates (x right, y down, z forward)
    /// to Unity world coordinates (x right, y up, z forward) and then transformed by camera transform.
    /// </summary>
    public void UpdateFromArucoMats(Mat ids, Mat rvecs, Mat tvecs)
    {
        if (ids == null || tvecs == null)
            return;

        if (arUcoCameraTransform == null)
        {
            Debug.LogWarning("TangramMatcher: arUcoCameraTransform is not assigned; cannot convert camera-space to world.");
            return;
        }

        latestDetections.Clear();

        int count = (int)ids.total();
        for (int i = 0; i < count; i++)
        {
            int arucoId = (int)ids.get(i, 0)[0];

            if (!TryMapArucoIdToShape(arucoId, out TangramShapeType shapeType))
                continue; // ignore unknown ids

            // Read tvec (position in camera coords, meters)
            double[] t = tvecs.get(i, 0);
            Vector3 posCamera = new Vector3((float)t[0], (float)(-t[1]), (float)t[2]); // flip Y to go from OpenCV to Unity
            Vector3 worldPos = arUcoCameraTransform.TransformPoint(posCamera);

            // Optional rotation conversion
            Quaternion worldRot = Quaternion.identity;
            try
            {
                if (rvecs != null && rvecs.total() > i)
                {
                    double[] r = rvecs.get(i, 0);
                    using (Mat rvec = new Mat(3, 1, CvType.CV_64F))
                    using (Mat rmat = new Mat(3, 3, CvType.CV_64F))
                    {
                        rvec.put(0, 0, r[0]);
                        rvec.put(1, 0, r[1]);
                        rvec.put(2, 0, r[2]);
                        Calib3d.Rodrigues(rvec, rmat);
                        // Convert rotation matrix to Unity Quaternion (camera space -> world)
                        Matrix4x4 m = Matrix4x4.identity;
                        m.m00 = (float)rmat.get(0, 0)[0]; m.m01 = (float)rmat.get(0, 1)[0]; m.m02 = (float)rmat.get(0, 2)[0];
                        m.m10 = (float)rmat.get(1, 0)[0]; m.m11 = (float)rmat.get(1, 1)[0]; m.m12 = (float)rmat.get(1, 2)[0];
                        m.m20 = (float)rmat.get(2, 0)[0]; m.m21 = (float)rmat.get(2, 1)[0]; m.m22 = (float)rmat.get(2, 2)[0];
                        // Flip Y axis to map to Unity camera coordinates
                        Matrix4x4 flipY = Matrix4x4.Scale(new Vector3(1, -1, 1));
                        m = flipY * m * flipY; // apply on both sides for proper handedness correction
                        worldRot = arUcoCameraTransform.rotation * Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
                    }
                }
            }
            catch (Exception)
            {
                worldRot = Quaternion.identity;
            }

            latestDetections.Add(new DetectedShape
            {
                arucoId = arucoId,
                shapeType = shapeType,
                worldPosition = worldPos,
                worldRotation = worldRot,
                confidence = 1f
            });
        }

        RunMatchingAndHighlight();
    }
#endif

    /// <summary>
    /// Compute matches and apply debug coloring.
    /// </summary>
    private void RunMatchingAndHighlight()
    {
        lastMatchResults.Clear();

        // Build available piece lists per type
        var availablePieces = new Dictionary<TangramShapeType, List<Transform>>();
        foreach (var kv in diagramPiecesByType)
        {
            availablePieces[kv.Key] = new List<Transform>(kv.Value);
        }

        var matchedPieceSet = new HashSet<Transform>();

        var linkSegments = new List<(Vector3 from, Vector3 to, float dist, Transform piece, TangramShapeType type, int id)>();
        // Collect near-miss segments in a local list, pass by reference where needed
        var nearMissSegments = new List<(Vector3 from, Vector3 to, float dist, Transform piece, TangramShapeType type, int id)>();

        // Absolute greedy matching removed – use graph-only matching
        RunGraphOnlyMatching(availablePieces, matchedPieceSet, linkSegments);

        // Error stats per type
        ComputeAndStoreErrorStats();

        // Temporal smoothing: convert instantaneous matches to stable set
        var stableSet = useTemporalSmoothing ? GetStableMatchedSet(matchedPieceSet) : matchedPieceSet;

        ApplyDebugColors(stableSet);

        // Draw debug lines for visual error
        if (debugDrawErrorLines)
        {
            foreach (var seg in linkSegments)
            {
                float t = errorMaxForColor > 0f ? Mathf.Clamp01(seg.dist / errorMaxForColor) : 1f;
                Color c = colorByErrorMagnitude ? Color.Lerp(errorLowColor, errorHighColor, t) : matchedColor;
                Debug.DrawLine(seg.from, seg.to, c, 0f, false);
            }
        }

        // Draw near-miss lines
        if (debugDrawNearMissLines)
        {
            foreach (var seg in nearMissSegments)
            {
                Debug.DrawLine(seg.from, seg.to, nearMissLineColor, 0f, false);
            }
        }

        // Debug log in English for clarity (including nearest graph neighbor info)
        foreach (var result in lastMatchResults)
        {
            string graphInfo = BuildGraphDebugInfo(result);
            Debug.Log($"[TangramMatcher] Matched {result.shapeType} (ArUco {result.arucoId}) -> Piece '{result.matchedPieceTransform.name}', distance = {result.worldDistanceMeters:F3} m, angle = {result.angleErrorDegrees:F1} deg. {graphInfo}");
        }

        if (debugLogPerTypeErrorStats)
        {
            foreach (var kv in lastTypeErrorStats)
            {
                var s = kv.Value;
                Debug.Log($"[TangramMatcher] Error stats for {kv.Key} -> count={s.count}, min={s.min:F3}m, max={s.max:F3}m, avg={s.avg:F3}m");
            }
        }
    }

    private void ApplyDebugColors(HashSet<Transform> matchedPieces)
    {
        // First, update unmatched pieces
        foreach (var kv in diagramPiecesByType)
        {
            foreach (Transform piece in kv.Value)
            {
                if (matchedPieces.Contains(piece)) continue;
                if (useNeutralColorMode)
                {
                    SetRendererColor(piece.gameObject, neutralColor);
                }
                else
                {
                    if (highlightOnlyMatches)
                        RestoreRendererOriginalColor(piece.gameObject);
                    else
                        SetRendererColor(piece.gameObject, unmatchedColor);
                }
            }
        }

        // Then, set matched pieces to color (optionally by error magnitude)
        foreach (Transform piece in matchedPieces)
        {
            if (useNeutralColorMode && matchedRestoresOriginalColor)
            {
                RestoreRendererOriginalColor(piece.gameObject);
            }
            else if (colorByErrorMagnitude)
            {
                // Find distance for this piece from last results
                float dist = 0f;
                bool found = false;
                foreach (var r in lastMatchResults)
                {
                    if (r.matchedPieceTransform == piece)
                    {
                        dist = r.worldDistanceMeters;
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    float t = errorMaxForColor > 0f ? Mathf.Clamp01(dist / errorMaxForColor) : 1f;
                    Color c = Color.Lerp(errorLowColor, errorHighColor, t);
                    SetRendererColor(piece.gameObject, c);
                }
                else
                {
                    SetRendererColor(piece.gameObject, matchedColor);
                }
            }
            else
            {
                SetRendererColor(piece.gameObject, matchedColor);
            }
        }
    }

    private void CaptureOriginalColors()
    {
        originalColorsByRenderer.Clear();
        var candidateProps = string.IsNullOrEmpty(overrideColorProperty)
            ? new[] { "_BaseColor", "_Color", "_TintColor" }
            : new[] { overrideColorProperty };
        foreach (var kv in diagramPiecesByType)
        {
            foreach (Transform piece in kv.Value)
            {
                var renderer = piece.GetComponent<MeshRenderer>();
                if (renderer == null) continue;
                var list = new List<SubMaterialColorInfo>();
                var sharedMats = renderer.sharedMaterials;
                for (int m = 0; m < sharedMats.Length; m++)
                {
                    var info = new SubMaterialColorInfo
                    {
                        propertyName = null,
                        originalColor = Color.white,
                        hasColorProperty = false,
                        fallbackOriginalColor = Color.white
                    };
                    var sm = sharedMats[m];
                    if (sm != null)
                    {
                        foreach (var prop in candidateProps)
                        {
                            if (sm.HasProperty(prop))
                            {
                                info.propertyName = prop;
                                try { info.originalColor = sm.GetColor(prop); }
                                catch { info.originalColor = Color.white; }
                                info.hasColorProperty = true;
                                break;
                            }
                        }
                        // Fallback: try reading material.color via a temp instantiation-read pattern avoided; assume white if unknown
                        try
                        {
                            // Note: sharedMaterial.color may not be valid if shader has no _Color
                            info.fallbackOriginalColor = sm.color;
                        }
                        catch { info.fallbackOriginalColor = Color.white; }
                    }
                    list.Add(info);
                }
                originalColorsByRenderer[renderer] = list;
                // Initialize temporal votes
                if (!votesByPiece.ContainsKey(piece))
                {
                    votesByPiece[piece] = new TemporalVotes { buffer = new int[Mathf.Max(1, temporalWindowSize)], index = 0, count = 0 };
                }
            }
        }
    }

    private void RestoreRendererOriginalColor(GameObject go)
    {
        var renderer = go.GetComponent<MeshRenderer>();
        if (renderer == null) return;
        if (!originalColorsByRenderer.TryGetValue(renderer, out var list)) return;

        // Restore via property block or fallback material.color
        for (int m = 0; m < list.Count; m++)
        {
            var info = list[m];
            if (info.hasColorProperty && !string.IsNullOrEmpty(info.propertyName))
            {
                if (sharedPropertyBlock == null) sharedPropertyBlock = new MaterialPropertyBlock();
                renderer.GetPropertyBlock(sharedPropertyBlock, m);
                sharedPropertyBlock.SetColor(info.propertyName, info.originalColor);
                renderer.SetPropertyBlock(sharedPropertyBlock, m);
            }
            else if (fallbackUseMaterialColorWhenNoProperty)
            {
                try
                {
                    var mats = renderer.materials;
                    if (m < mats.Length && mats[m] != null)
                    {
                        mats[m].color = info.fallbackOriginalColor;
                    }
                }
                catch { /* ignore */ }
            }
        }
    }

    private HashSet<Transform> GetStableMatchedSet(HashSet<Transform> instantaneous)
    {
        temporalFrameCounter++;
        // Update votes ring buffer
        foreach (var kv in diagramPiecesByType)
        {
            foreach (Transform piece in kv.Value)
            {
                if (!votesByPiece.TryGetValue(piece, out var tv))
                {
                    tv = new TemporalVotes { buffer = new int[Mathf.Max(1, temporalWindowSize)], index = 0, count = 0 };
                    votesByPiece[piece] = tv;
                }
                if (tv.buffer.Length != temporalWindowSize)
                {
                    tv.buffer = new int[Mathf.Max(1, temporalWindowSize)];
                    tv.index = 0; tv.count = 0;
                }
                // write vote
                tv.buffer[tv.index] = instantaneous.Contains(piece) ? 1 : 0;
                tv.index = (tv.index + 1) % tv.buffer.Length;
                if (tv.count < tv.buffer.Length) tv.count++;
            }
        }

        // Derive stable set
        var stable = new HashSet<Transform>();
        foreach (var kv in diagramPiecesByType)
        {
            foreach (Transform piece in kv.Value)
            {
                var tv = votesByPiece[piece];
                int sum = 0; for (int i = 0; i < tv.count; i++) sum += tv.buffer[i];
                // If temporalMinVotes > 0, require at least that many; else pick majority (> half)
                int threshold = temporalMinVotes > 0 ? temporalMinVotes : Mathf.Max(1, tv.buffer.Length / 2 + 1);
                if (sum >= threshold) stable.Add(piece);
            }
        }
        return stable;
    }

    // ---------- Diagram Graph Construction ----------
    private void BuildDiagramGraph()
    {
        diagramGraph = new DiagramGraph();

        // Collect all nodes (all pieces regardless of type)
        var allPieces = new List<Transform>();
        foreach (var kv in diagramPiecesByType)
        {
            allPieces.AddRange(kv.Value);
        }

        // Build nodes
        for (int i = 0; i < allPieces.Count; i++)
        {
            Transform p = allPieces[i];
            var node = new DiagramGraph.Node
            {
                piece = p,
                sizeMeters = EstimatePieceSizeMeters(p)
            };
            diagramGraph.indexByTransform[p] = diagramGraph.nodes.Count;
            diagramGraph.nodes.Add(node);
        }

        // Build edges using normalized distance threshold and direction angle
        for (int i = 0; i < diagramGraph.nodes.Count; i++)
        {
            var ni = diagramGraph.nodes[i];
            Vector3 pi = ni.piece.position;
            int bestJ = -1;
            float bestDist = float.PositiveInfinity;
            float bestNorm = float.PositiveInfinity;
            float bestAngle = 0f;
            var allEdges = new List<DiagramGraph.Edge>();
            for (int j = 0; j < diagramGraph.nodes.Count; j++)
            {
                if (i == j) continue;
                var nj = diagramGraph.nodes[j];
                Vector3 pj = nj.piece.position;

                float centerDist = Vector3.Distance(pi, pj);
                float norm = (ni.sizeMeters + nj.sizeMeters);
                float normalized = norm > 1e-4f ? centerDist / norm : float.PositiveInfinity;
                if (normalized <= diagramAdjacencyMaxNormalizedDistance)
                {
                    float angle = ComputePlanarAngleDeg(pi, pj);
                    var edge = new DiagramGraph.Edge
                    {
                        toIndex = diagramGraph.indexByTransform[nj.piece],
                        expectedDistanceMeters = centerDist,
                        expectedAngleDeg = angle,
                        normalizedDistance = normalized
                    };
                    allEdges.Add(edge);
                    if (centerDist < bestDist)
                    {
                        bestDist = centerDist;
                        bestJ = j;
                        bestNorm = normalized;
                        bestAngle = angle;
                    }
                }
            }

            if (diagramOnlyNearestNeighbor)
            {
                if (bestJ >= 0)
                {
                    ni.edges.Clear();
                    ni.edges.Add(new DiagramGraph.Edge
                    {
                        toIndex = bestJ,
                        expectedDistanceMeters = bestDist,
                        expectedAngleDeg = bestAngle,
                        normalizedDistance = bestNorm
                    });
                }
            }
            else
            {
                ni.edges.AddRange(allEdges);
                // Keep only closest edges if limited
                if (diagramMaxNeighborsPerNode > 0 && ni.edges.Count > diagramMaxNeighborsPerNode)
                {
                    ni.edges.Sort((a, b) => a.expectedDistanceMeters.CompareTo(b.expectedDistanceMeters));
                    ni.edges.RemoveRange(diagramMaxNeighborsPerNode, ni.edges.Count - diagramMaxNeighborsPerNode);
                }
            }
        }
    }

    // ---------- Matching helpers ----------
    // Absolute greedy matching removed

    private class DetectionGraph
    {
        public class DNode
        {
            public int detIndex; // index into latestDetections
            public TangramShapeType type;
            public Vector3 pos;
            public Quaternion rot;
            public readonly List<int> neighbors = new List<int>(); // neighbor indices into nodes list
        }
        public readonly List<DNode> nodes = new List<DNode>();
    }

    private void RunGraphOnlyMatching(
        Dictionary<TangramShapeType, List<Transform>> availablePieces,
        HashSet<Transform> matchedPieceSet,
        List<(Vector3 from, Vector3 to, float dist, Transform piece, TangramShapeType type, int id)> linkSegments)
    {
        // 1) Build detection graph with nominal-size-normalized adjacency
        var dg = BuildDetectionGraph();
        if (dg.nodes.Count == 0)
            return;

        // 2) For each detection, find diagram candidates by graph relation only
        // Strategy: seed with the detection that has the most neighbors; map it to the diagram piece of same type with highest graph degree (no absolute coords).
        // Then expand frontier: only match detections that have at least one already-matched neighbor, and only to diagram pieces that are neighbors of at least one matched diagram neighbor.

        // Order nodes by decreasing degree
        var order = new List<int>();
        for (int i = 0; i < dg.nodes.Count; i++) order.Add(i);
        order.Sort((a, b) => dg.nodes[b].neighbors.Count.CompareTo(dg.nodes[a].neighbors.Count));

        // Track which diagram pieces remain per type
        var remainingByType = new Dictionary<TangramShapeType, List<Transform>>();
        foreach (var kv in availablePieces) remainingByType[kv.Key] = new List<Transform>(kv.Value);

        var detToPiece = new Dictionary<int, Transform>();
        var assignedDetectionIndices = new HashSet<int>();
        bool seeded = false;

        for (int oi = 0; oi < order.Count; oi++)
        {
            int nodeIdx = order[oi];
            if (assignedDetectionIndices.Contains(nodeIdx))
                continue;
            var node = dg.nodes[nodeIdx];

            if (!remainingByType.TryGetValue(node.type, out var candidates) || candidates.Count == 0)
                continue;

            Transform chosen = null;
            float chosenCost = float.PositiveInfinity;
            float chosenAvgDistDiff = 0f;
            float chosenAvgAngDiff = 0f;

            // If any neighbor is already matched, constrain to neighbors in diagram graph
            var neighborMatchedPieces = new HashSet<Transform>();
            foreach (int nIdx in node.neighbors)
            {
                if (detToPiece.TryGetValue(nIdx, out var neighborPiece))
                {
                    neighborMatchedPieces.Add(neighborPiece);
                }
            }

            List<Transform> filteredCandidates = candidates;
            if (neighborMatchedPieces.Count > 0)
            {
                filteredCandidates = FilterCandidatesByAnyGraphNeighbor(neighborMatchedPieces, candidates);
                if (filteredCandidates.Count == 0)
                    continue; // no graph-consistent options, skip
            }

            // Multi-detection assignment for duplicate types: choose configuration that maximizes number of matches, then minimal total cost
            if (remainingByType.TryGetValue(node.type, out var remainingForType) && remainingForType.Count >= 2)
            {
                var detIndices = new List<int>();
                // collect current and subsequent unmatched detections of same type having at least one matched neighbor
                for (int oj = oi; oj < order.Count; oj++)
                {
                    int candIdx = order[oj];
                    if (assignedDetectionIndices.Contains(candIdx)) continue;
                    var candNode = dg.nodes[candIdx];
                    if (candNode.type != node.type) continue;
                    bool hasMatchedNeighbor = false;
                    foreach (int nn in candNode.neighbors) { if (detToPiece.ContainsKey(nn)) { hasMatchedNeighbor = true; break; } }
                    if (!hasMatchedNeighbor) continue;
                    detIndices.Add(candIdx);
                }
                if (detIndices.Count >= 2)
                {
                    // Build union candidate pool filtered by graph neighbors per detection
                    var pool = new List<Transform>();
                    foreach (int di in detIndices)
                    {
                        var neighSet = new HashSet<Transform>();
                        foreach (int nn in dg.nodes[di].neighbors) { if (detToPiece.TryGetValue(nn, out var np)) neighSet.Add(np); }
                        var filtered = FilterCandidatesByAnyGraphNeighbor(neighSet, remainingForType);
                        foreach (var p in filtered) if (!pool.Contains(p)) pool.Add(p);
                    }
                    if (pool.Count > 0)
                    {
                        // Build all feasible pairs with costs
                        var pairs = new List<(int di, Transform piece, float cost, float d, float a)>();
                        foreach (int di in detIndices)
                        {
                            foreach (var p in pool)
                            {
                                if (!TryComputeRelationalCostForCandidate(di, p, dg, detToPiece, out float d, out float a, out _, out int cnt))
                                    continue;
                                if (d <= graphDistanceToleranceMeters && a <= graphAngleToleranceDeg)
                                {
                                    float cost = d + rotationWeightMetersPerDeg * a;
                                    pairs.Add((di, p, cost, d, a));
                                }
                            }
                        }
                        // Greedy by ascending cost to maximize matches
                        pairs.Sort((x, y) => x.cost.CompareTo(y.cost));
                        var usedDet = new HashSet<int>();
                        var usedPiece = new HashSet<Transform>();
                        foreach (var pr in pairs)
                        {
                            if (usedDet.Contains(pr.di) || usedPiece.Contains(pr.piece)) continue;
                            usedDet.Add(pr.di); usedPiece.Add(pr.piece);
                        }
                        if (usedDet.Count > 0)
                        {
                            // Commit selected matches
                            foreach (var pr in pairs)
                            {
                                if (!usedDet.Contains(pr.di) || !usedPiece.Contains(pr.piece)) continue;
                            }
                            // Actually iterate again to commit in order to keep logs and results clean
                            var committed = new HashSet<int>();
                            var committedPieces = new HashSet<Transform>();
                            foreach (var pr in pairs)
                            {
                                if (committed.Contains(pr.di) || committedPieces.Contains(pr.piece)) continue;
                                if (!usedDet.Contains(pr.di) || !usedPiece.Contains(pr.piece)) continue;
                                int di = pr.di; var piece = pr.piece;
                                detToPiece[di] = piece;
                                assignedDetectionIndices.Add(di);
                                remainingForType.Remove(piece);
                                matchedPieceSet.Add(piece);
                                var det = latestDetections[dg.nodes[di].detIndex];
                                lastMatchResults.Add(new MatchResult
                                {
                                    shapeType = det.shapeType,
                                    arucoId = det.arucoId,
                                    matchedPieceTransform = piece,
                                    worldDistanceMeters = pr.d,
                                    detectionWorldPosition = det.worldPosition,
                                    angleErrorDegrees = pr.a
                                });
                                linkSegments.Add((det.worldPosition, piece.position, pr.d, piece, det.shapeType, det.arucoId));
                                committed.Add(di); committedPieces.Add(piece);
                            }
                            // Move to next after committing this type group
                            continue;
                        }
                    }
                }
            }

            if (neighborMatchedPieces.Count == 0 && !seeded)
            {
                // Seed: pick diagram piece with highest graph degree among candidates
                Transform best = null;
                int bestDeg = -1;
                foreach (var p in filteredCandidates)
                {
                    if (!diagramGraph.indexByTransform.TryGetValue(p, out int idx)) continue;
                    int deg = diagramGraph.nodes[idx].edges.Count;
                    if (deg > bestDeg)
                    {
                        bestDeg = deg;
                        best = p;
                    }
                }
                if (best != null)
                {
                    chosen = best;
                    chosenCost = 0f;
                    chosenAvgDistDiff = 0f;
                    chosenAvgAngDiff = 0f;
                }
            }
            else
            {
                // Relational cost using only the nearest already-matched neighbor
                var nearMissList = new List<(Transform p, float avgDistDiff, float avgAngDiff, float cost)>();
                foreach (var p in filteredCandidates)
                {
                    // find nearest matched neighbor in detection graph
                    int nearestIdx = -1; float nearestDet = float.PositiveInfinity; Transform nearestPiece = null;
                    foreach (int nIdx in node.neighbors)
                    {
                        if (!detToPiece.TryGetValue(nIdx, out var np)) continue;
                        float d = Vector3.Distance(dg.nodes[nIdx].pos, node.pos);
                        if (d < nearestDet) { nearestDet = d; nearestIdx = nIdx; nearestPiece = np; }
                    }
                    if (nearestIdx < 0) continue; // no neighbor constraints available

                    Vector3 vDet = dg.nodes[nearestIdx].pos - node.pos;
                    Vector3 vDiag = nearestPiece.position - p.position;
                    if (usePlanarProjectionForRelations)
                    {
                        Vector3 nrm = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
                        vDet = Vector3.ProjectOnPlane(vDet, nrm);
                        vDiag = Vector3.ProjectOnPlane(vDiag, nrm);
                    }
                    float scaleFactor = 1f;
                    if (useScaleNormalization && vDiag.magnitude > 1e-4f)
                        scaleFactor = vDet.magnitude / vDiag.magnitude;
                    float avgDistDiff = Mathf.Abs(vDet.magnitude - vDiag.magnitude * scaleFactor);
                    float avgAngDiff = ComputePlanarAngleBetweenVectors(vDet, vDiag);
                    float cost = avgDistDiff + rotationWeightMetersPerDeg * avgAngDiff;
                    bool passes = avgDistDiff <= graphDistanceToleranceMeters && avgAngDiff <= graphAngleToleranceDeg;
                    if (passes)
                    {
                        if (cost < chosenCost)
                        {
                            chosenCost = cost;
                            chosen = p;
                            chosenAvgDistDiff = avgDistDiff;
                            chosenAvgAngDiff = avgAngDiff;
                        }
                    }
                    else
                    {
                        nearMissList.Add((p, avgDistDiff, avgAngDiff, cost));
                        if (debugDrawNearMissLines)
                        {
                            // draw a line from this detection to the candidate to visualize near miss
                            linkSegments.Add((node.pos, p.position, avgDistDiff, p, node.type, latestDetections[node.detIndex].arucoId));
                        }
                    }
                }
                // Log top near-miss candidates
                if (debugLogUnmatchedCandidates && nearMissList.Count > 0)
                {
                    nearMissList.Sort((a, b) => a.cost.CompareTo(b.cost));
                    int take = Mathf.Min(debugMaxUnmatchedCandidatesPerDetection, nearMissList.Count);
                    for (int ii = 0; ii < take; ii++)
                    {
                        var nm = nearMissList[ii];
                        Debug.Log($"[TangramMatcher] near-miss for det({node.type}) -> diag({nm.p.name}) avg diff(d)={nm.avgDistDiff:F3}m avg diff(θ)={nm.avgAngDiff:F1} deg | tol(d)<={graphDistanceToleranceMeters:F3} tol(θ)<={graphAngleToleranceDeg:F1}");
                        if (debugDrawNearMissLines)
                        {
                            // Draw line from detection to candidate piece center with near-miss color and use distance diff for segment magnitude coloring elsewhere if needed
                            // We'll add a short segment into the shared near-miss collector via link-like callback by referencing outer scope list
                        }
                    }
                }
            }

            if (chosen != null)
            {
                detToPiece[nodeIdx] = chosen;
                assignedDetectionIndices.Add(nodeIdx);
                remainingByType[node.type].Remove(chosen);
                matchedPieceSet.Add(chosen);
                var det = latestDetections[node.detIndex];
                lastMatchResults.Add(new MatchResult
                {
                    shapeType = det.shapeType,
                    arucoId = det.arucoId,
                    matchedPieceTransform = chosen,
                    worldDistanceMeters = chosenAvgDistDiff,
                    detectionWorldPosition = det.worldPosition,
                    angleErrorDegrees = chosenAvgAngDiff
                });
                linkSegments.Add((det.worldPosition, chosen.position, chosenAvgDistDiff, chosen, det.shapeType, det.arucoId));

                // Detailed relational debug per matched neighbor
                foreach (int nIdx in node.neighbors)
                {
                    if (!detToPiece.TryGetValue(nIdx, out var neighborPiece))
                        continue;
                    Vector3 vDet = node.pos - dg.nodes[nIdx].pos;
                    Vector3 vDiag = chosen.position - neighborPiece.position;
                    float distDiff = Mathf.Abs(vDet.magnitude - vDiag.magnitude);
                    float angDiff = ComputePlanarAngleBetweenVectors(vDet, vDiag);
                    Debug.Log($"[TangramMatcher] matched relation det({node.type})-det({dg.nodes[nIdx].type}) vs diag({chosen.name})-diag({neighborPiece.name}) diff(d)={distDiff:F3}m diff(θ)={angDiff:F1} deg");
                }

                if (!seeded) seeded = true;
            }
        }
    }

    private List<Transform> FilterCandidatesByAnyGraphNeighbor(HashSet<Transform> matchedNeighbors, List<Transform> candidates)
    {
        if (diagramGraph == null || diagramGraph.nodes.Count == 0)
            return new List<Transform>(candidates);
        var allowed = new HashSet<Transform>();
        foreach (var neigh in matchedNeighbors)
        {
            if (!diagramGraph.indexByTransform.TryGetValue(neigh, out int idx))
                continue;
            var node = diagramGraph.nodes[idx];
            allowed.Add(neigh); // allow itself
            foreach (var e in node.edges)
            {
                allowed.Add(diagramGraph.nodes[e.toIndex].piece);
            }
        }
        var list = new List<Transform>();
        foreach (var c in candidates)
        {
            if (allowed.Contains(c)) list.Add(c);
        }
        if (list.Count == 0) return new List<Transform>(candidates); // fallback
        return list;
    }

    private DetectionGraph BuildDetectionGraph()
    {
        var dg = new DetectionGraph();
        // Build nodes
        for (int i = 0; i < latestDetections.Count; i++)
        {
            var d = latestDetections[i];
            dg.nodes.Add(new DetectionGraph.DNode
            {
                detIndex = i,
                type = d.shapeType,
                pos = d.worldPosition,
                rot = d.worldRotation
            });
        }
        // Build edges (normalized by nominal size per type)
        for (int i = 0; i < dg.nodes.Count; i++)
        {
            var ni = dg.nodes[i];
            float sizeI = GetNominalSize(ni.type);
            for (int j = 0; j < dg.nodes.Count; j++)
            {
                if (i == j) continue;
                var nj = dg.nodes[j];
                float sizeJ = GetNominalSize(nj.type);
                float dist = Vector3.Distance(ni.pos, nj.pos);
                float denom = sizeI + sizeJ;
                float norm = denom > 1e-4f ? dist / denom : float.PositiveInfinity;
                if (norm <= detectionAdjacencyMaxNormalizedDistance)
                {
                    ni.neighbors.Add(j);
                }
            }
            // limit neighbors
            if (detectionMaxNeighborsPerNode > 0 && ni.neighbors.Count > detectionMaxNeighborsPerNode)
            {
                ni.neighbors.Sort((a, b) => Vector3.Distance(dg.nodes[i].pos, dg.nodes[a].pos).CompareTo(Vector3.Distance(dg.nodes[i].pos, dg.nodes[b].pos)));
                ni.neighbors.RemoveRange(detectionMaxNeighborsPerNode, ni.neighbors.Count - detectionMaxNeighborsPerNode);
            }
        }
        return dg;
    }

    private float GetNominalSize(TangramShapeType type)
    {
        switch (type)
        {
            case TangramShapeType.LargeTriangle: return nominalSizeLargeTriangle;
            case TangramShapeType.MediumTriangle: return nominalSizeMediumTriangle;
            case TangramShapeType.SmallTriangle: return nominalSizeSmallTriangle;
            case TangramShapeType.Square: return nominalSizeSquare;
            case TangramShapeType.Parallelogram: return nominalSizeParallelogram;
            default: return 0.18f;
        }
    }

    private bool TryComputeRelationalCostForCandidate(int nodeIdx, Transform candidatePiece, DetectionGraph dg, Dictionary<int, Transform> detToPiece,
        out float avgDistDiff, out float avgAngDiff, out float usedScale, out int usedCount)
    {
        float sumDist = 0f, sumAng = 0f; int count = 0; usedScale = 1f;
        var scaleSamples = new List<float>();
        var center = dg.nodes[nodeIdx];
        foreach (int nIdx in center.neighbors)
        {
            if (!detToPiece.TryGetValue(nIdx, out var neighborPiece)) continue;
            Vector3 vDet = dg.nodes[nIdx].pos - center.pos;
            Vector3 vDiag = neighborPiece.position - candidatePiece.position;
            if (usePlanarProjectionForRelations)
            {
                Vector3 nrm = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
                vDet = Vector3.ProjectOnPlane(vDet, nrm);
                vDiag = Vector3.ProjectOnPlane(vDiag, nrm);
            }
            sumDist += Mathf.Abs(vDet.magnitude - vDiag.magnitude);
            sumAng += ComputePlanarAngleBetweenVectors(vDet, vDiag);
            if (useScaleNormalization && vDiag.magnitude > 1e-4f)
                scaleSamples.Add(vDet.magnitude / vDiag.magnitude);
            count++;
        }
        if (count == 0) { avgDistDiff = 0f; avgAngDiff = 0f; usedCount = 0; return false; }
        if (useScaleNormalization && scaleSamples.Count > 0)
        {
            scaleSamples.Sort(); usedScale = scaleSamples[scaleSamples.Count / 2];
            // recompute distance diff with scale
            sumDist = 0f; int rc = 0;
            foreach (int nIdx in center.neighbors)
            {
                if (!detToPiece.TryGetValue(nIdx, out var neighborPiece)) continue;
                Vector3 vDet = dg.nodes[nIdx].pos - center.pos;
                Vector3 vDiag = neighborPiece.position - candidatePiece.position;
                if (usePlanarProjectionForRelations)
                {
                    Vector3 nrm = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
                    vDet = Vector3.ProjectOnPlane(vDet, nrm);
                    vDiag = Vector3.ProjectOnPlane(vDiag, nrm);
                }
                sumDist += Mathf.Abs(vDet.magnitude - vDiag.magnitude * usedScale);
                rc++;
            }
            if (rc > 0) count = rc;
        }
        avgDistDiff = sumDist / count; avgAngDiff = sumAng / count; usedCount = count; return true;
    }

    private float EstimatePieceSizeMeters(Transform piece)
    {
        // Estimate by renderer bounds extents magnitude (approximate scale/size)
        var rend = piece.GetComponent<MeshRenderer>();
        if (rend == null) return 0.1f;
        var bounds = rend.bounds;
        // Use planar size (x,z) more than height (y)
        Vector3 size = bounds.size;
        float planar = Mathf.Sqrt(size.x * size.x + size.z * size.z);
        return Mathf.Max(planar * 0.5f, 0.01f);
    }

    private float ComputePlanarAngleDeg(Vector3 from, Vector3 to)
    {
        Vector3 dir = to - from;
        Vector3 planeNormal = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
        Vector3 dirProj = Vector3.ProjectOnPlane(dir, planeNormal);
        if (dirProj.sqrMagnitude < 1e-6f) return 0f;
        dirProj.Normalize();
        // Compute angle relative to diagram forward projected onto plane
        Vector3 refForward = tangramDiagramRoot != null ? Vector3.ProjectOnPlane(tangramDiagramRoot.forward, planeNormal).normalized : Vector3.forward;
        if (refForward.sqrMagnitude < 1e-6f) refForward = Vector3.forward;
        float ang = Vector3.SignedAngle(refForward, dirProj, planeNormal);
        return NormalizeAngle180(ang);
    }

    private float ComputePlanarAngleBetweenVectors(Vector3 vA, Vector3 vB)
    {
        Vector3 planeNormal = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
        Vector3 a = Vector3.ProjectOnPlane(vA, planeNormal);
        Vector3 b = Vector3.ProjectOnPlane(vB, planeNormal);
        if (a.sqrMagnitude < 1e-6f || b.sqrMagnitude < 1e-6f) return 0f;
        a.Normalize();
        b.Normalize();
        float ang = Mathf.Abs(Vector3.SignedAngle(a, b, planeNormal));
        return Mathf.Abs(NormalizeAngle180(ang));
    }

    private float ComputeAveragePairwiseAngle(List<Vector3> dirs)
    {
        if (dirs == null || dirs.Count < 2) return 0f;
        Vector3 planeNormal = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
        float sum = 0f;
        int cnt = 0;
        for (int i = 0; i < dirs.Count; i++)
        {
            for (int j = i + 1; j < dirs.Count; j++)
            {
                Vector3 a = Vector3.ProjectOnPlane(dirs[i], planeNormal).normalized;
                Vector3 b = Vector3.ProjectOnPlane(dirs[j], planeNormal).normalized;
                if (a.sqrMagnitude < 1e-6f || b.sqrMagnitude < 1e-6f) continue;
                float ang = Mathf.Abs(Vector3.SignedAngle(a, b, planeNormal));
                sum += Mathf.Abs(NormalizeAngle180(ang));
                cnt++;
            }
        }
        if (cnt == 0) return 0f;
        return sum / cnt;
    }

    private void DebugLogDiagramGraph()
    {
        if (diagramGraph == null)
        {
            Debug.Log("[TangramMatcher] Diagram graph is null");
            return;
        }
        Debug.Log($"[TangramMatcher] Diagram graph: nodes={diagramGraph.nodes.Count}");
        for (int i = 0; i < diagramGraph.nodes.Count; i++)
        {
            var n = diagramGraph.nodes[i];
            string edges = "";
            for (int e = 0; e < n.edges.Count; e++)
            {
                var ed = n.edges[e];
                var to = diagramGraph.nodes[ed.toIndex].piece;
                edges += $" -> {to.name}(d={ed.expectedDistanceMeters:F2}m, θ={ed.expectedAngleDeg:F0}°, nd={ed.normalizedDistance:F2})";
            }
            Debug.Log($"  [{i}] {n.piece.name} size~{n.sizeMeters:F2}m{edges}");
        }
    }

    private void ComputeAndStoreErrorStats()
    {
        lastTypeErrorStats.Clear();
        // Initialize
        foreach (TangramShapeType type in Enum.GetValues(typeof(TangramShapeType)))
        {
            lastTypeErrorStats[type] = new ErrorStats { count = 0, min = float.PositiveInfinity, max = 0f, avg = 0f };
        }

        var sums = new Dictionary<TangramShapeType, float>();
        foreach (TangramShapeType type in Enum.GetValues(typeof(TangramShapeType)))
            sums[type] = 0f;

        foreach (var res in lastMatchResults)
        {
            var s = lastTypeErrorStats[res.shapeType];
            s.count += 1;
            s.min = Mathf.Min(s.min, res.worldDistanceMeters);
            s.max = Mathf.Max(s.max, res.worldDistanceMeters);
            lastTypeErrorStats[res.shapeType] = s;
            sums[res.shapeType] += res.worldDistanceMeters;
        }

        // finalize avg and min for empty sets
        var keys = new List<TangramShapeType>(lastTypeErrorStats.Keys);
        foreach (var type in keys)
        {
            var s = lastTypeErrorStats[type];
            if (s.count == 0)
            {
                s.min = 0f;
                s.max = 0f;
                s.avg = 0f;
            }
            else
            {
                s.avg = sums[type] / s.count;
            }
            lastTypeErrorStats[type] = s;
        }
    }

    private void SetRendererColor(GameObject go, Color color)
    {
        var renderer = go.GetComponent<MeshRenderer>();
        if (renderer == null)
            return;

        if (sharedPropertyBlock == null)
            sharedPropertyBlock = new MaterialPropertyBlock();

        // Try property block first (non-instancing); support multi-material renderers
        string[] candidateProps = string.IsNullOrEmpty(overrideColorProperty)
            ? new[] { "_BaseColor", "_Color", "_TintColor" }
            : new[] { overrideColorProperty };
        bool applied = false;
        var sharedMats = renderer.sharedMaterials;
        for (int m = 0; m < sharedMats.Length; m++)
        {
            var sm = sharedMats[m];
            if (sm == null) continue;
            foreach (var prop in candidateProps)
            {
                if (sm.HasProperty(prop))
                {
                    renderer.GetPropertyBlock(sharedPropertyBlock, m);
                    sharedPropertyBlock.SetColor(prop, color);
                    renderer.SetPropertyBlock(sharedPropertyBlock, m);
                    applied = true;
                    break;
                }
            }
        }

        // Fallback: instantiate material and set .color (only if allowed)
        if (!applied && fallbackUseMaterialColorWhenNoProperty)
        {
            try
            {
                // Apply to all instantiated materials if needed
                var mats = renderer.materials;
                for (int i = 0; i < mats.Length; i++)
                {
                    if (mats[i] != null)
                    {
                        mats[i].color = color;
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[TangramMatcher] Failed to set material color on {go.name}: {e.Message}");
            }
        }
    }

    void OnGUI()
    {
        if (!showErrorLabelsUI)
            return;
        var cam = uiCamera != null ? uiCamera : Camera.main;
        if (cam == null) cam = Camera.current;
        if (cam == null && Camera.allCamerasCount > 0) cam = Camera.allCameras[0];
        if (cam == null) return;
        if (cachedLabelStyle == null)
        {
            cachedLabelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = uiFontSize,
                normal = { textColor = uiLabelColor }
            };
        }
        else
        {
            cachedLabelStyle.fontSize = uiFontSize;
            cachedLabelStyle.normal.textColor = uiLabelColor;
        }

        foreach (var res in lastMatchResults)
        {
            Vector3 world = res.matchedPieceTransform.position;
            Vector3 sp = cam.WorldToScreenPoint(world);
            if (sp.z < 0f) continue; // behind camera
            float x = sp.x;
            float y = Screen.height - sp.y;
            string text = $"{res.shapeType}: {res.worldDistanceMeters:F2}m, {res.angleErrorDegrees:F0}°";
            GUI.Label(new Rect(x + 6f, y - 18f, 260f, 22f), text, cachedLabelStyle);
        }
    }

    private static bool HasRenderableMesh(GameObject go)
    {
        return go != null && go.GetComponent<MeshFilter>() != null && go.GetComponent<MeshRenderer>() != null;
    }

    private static bool TryInferShapeTypeFromName(string objectName, out TangramShapeType shapeType)
    {
        string lower = objectName.ToLowerInvariant();

        if (lower.Contains("triangle_l"))
        {
            shapeType = TangramShapeType.LargeTriangle;
            return true;
        }
        if (lower.Contains("triangle_m"))
        {
            shapeType = TangramShapeType.MediumTriangle;
            return true;
        }
        if (lower.Contains("triangle_s"))
        {
            shapeType = TangramShapeType.SmallTriangle;
            return true;
        }
        if (lower.Contains("square"))
        {
            shapeType = TangramShapeType.Square;
            return true;
        }
        if (lower.Contains("parallel") || lower.Contains("parallelogram"))
        {
            shapeType = TangramShapeType.Parallelogram;
            return true;
        }

        shapeType = default;
        return false;
    }

    private static bool TryMapArucoIdToShape(int arucoId, out TangramShapeType shapeType)
    {
        switch (arucoId)
        {
            case 0: shapeType = TangramShapeType.LargeTriangle; return true;
            case 1: shapeType = TangramShapeType.MediumTriangle; return true;
            case 2: shapeType = TangramShapeType.SmallTriangle; return true;
            case 3: shapeType = TangramShapeType.Square; return true;
            case 4: shapeType = TangramShapeType.Parallelogram; return true;
            default:
                shapeType = default;
                return false;
        }
    }

    // -------- Anchor-relative matching and angle handling --------
    // Anchor-relative matching removed

    private IEnumerable<Transform> FilterCandidatesByGraph(Transform diagramAnchor, List<Transform> candidates)
    {
        // If graph has no data or anchor is missing, allow all
        if (diagramGraph == null || diagramGraph.nodes.Count == 0 || diagramAnchor == null)
            return candidates;

        if (!diagramGraph.indexByTransform.TryGetValue(diagramAnchor, out int anchorIdx))
            return candidates;

        var allowed = new HashSet<Transform>();
        var anchorNode = diagramGraph.nodes[anchorIdx];
        foreach (var e in anchorNode.edges)
        {
            var neighbor = diagramGraph.nodes[e.toIndex].piece;
            allowed.Add(neighbor);
        }

        // Always include the anchor itself
        allowed.Add(diagramAnchor);

        var list = new List<Transform>();
        foreach (var c in candidates)
        {
            if (allowed.Contains(c)) list.Add(c);
        }
        // If nothing allowed by graph, fallback to all candidates to avoid deadlock
        if (list.Count == 0) return candidates;
        return list;
    }

    private float ComputeAngleError(TangramShapeType type, Quaternion detectionWorldRotation, Transform piece)
    {
        if (piece == null)
            return 0f;
        // If no rotation provided, treat as zero error (position-only)
        if (detectionWorldRotation == Quaternion.identity)
            return 0f;

        // Use diagram up as plane normal; fallback to world up
        Vector3 planeNormal = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
        // Compare planar orientation using projected forward vectors
        Vector3 detForward = detectionWorldRotation * Vector3.forward;
        Vector3 pieceForward = piece.rotation * Vector3.forward;
        Vector3 detProj = Vector3.ProjectOnPlane(detForward, planeNormal).normalized;
        Vector3 pieceProj = Vector3.ProjectOnPlane(pieceForward, planeNormal).normalized;
        if (detProj.sqrMagnitude < 1e-6f || pieceProj.sqrMagnitude < 1e-6f)
            return 0f;

        float rawAngle = Mathf.Abs(Vector3.SignedAngle(pieceProj, detProj, planeNormal));
        rawAngle = NormalizeAngle180(rawAngle);

        // Handle symmetry per type (modulo)
        float symmetry = GetSymmetryModuloDegrees(type);
        if (symmetry > 0f)
        {
            rawAngle = rawAngle % symmetry;
            rawAngle = Mathf.Min(rawAngle, symmetry - rawAngle);
        }

        // Mirror-insensitive for parallelogram
        if (type == TangramShapeType.Parallelogram)
        {
            rawAngle = Mathf.Min(rawAngle, Mathf.Abs(180f - rawAngle));
        }

        return Mathf.Abs(rawAngle);
    }

    private static float NormalizeAngle180(float deg)
    {
        while (deg > 180f) deg -= 360f;
        while (deg < -180f) deg += 360f;
        return deg;
    }

    private static float GetSymmetryModuloDegrees(TangramShapeType type)
    {
        switch (type)
        {
            case TangramShapeType.Square:
                return 90f; // 4-fold rotational symmetry
            case TangramShapeType.Parallelogram:
                return 180f; // 2-fold rotational symmetry
            case TangramShapeType.LargeTriangle:
            case TangramShapeType.MediumTriangle:
            case TangramShapeType.SmallTriangle:
                return 90f; // Right isosceles tangram triangles
            default:
                return 0f;
        }
    }

    private string BuildGraphDebugInfo(MatchResult res)
    {
        if (diagramGraph == null || res.matchedPieceTransform == null)
            return "(no graph)";
        if (!diagramGraph.indexByTransform.TryGetValue(res.matchedPieceTransform, out int idx))
            return "(no graph node)";
        var node = diagramGraph.nodes[idx];
        if (node.edges.Count == 0)
            return "(no neighbors)";

        // Report nearest neighbor expectations vs actuals for first few neighbors
        int report = Mathf.Min(2, node.edges.Count);
        var parts = new List<string>();
        for (int k = 0; k < report; k++)
        {
            var e = node.edges[k];
            var neigh = diagramGraph.nodes[e.toIndex].piece;
            float actualDist = Vector3.Distance(node.piece.position, neigh.position);
            float actualAng = ComputePlanarAngleDeg(node.piece.position, neigh.position);
            float dDiff = Mathf.Abs(actualDist - e.expectedDistanceMeters);
            float aDiff = Mathf.Abs(NormalizeAngle180(actualAng - e.expectedAngleDeg));
            parts.Add($"-> '{neigh.name}' diff(d)={dDiff:F2}m diff(θ)={aDiff:F0}°");
        }
        return string.Join(", ", parts);
    }
}


