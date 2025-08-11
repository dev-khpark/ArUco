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

    [Header("Debug Colors")]
    public Color matchedColor = Color.green;
    public Color unmatchedColor = new Color(1f, 1f, 1f, 1f); // keep original white by default
    [Tooltip("If true, only matched pieces will change color; unmatched remain untouched.")]
    public bool highlightOnlyMatches = true;

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
    }

    // Cached diagram children grouped by type
    private readonly Dictionary<TangramShapeType, List<Transform>> diagramPiecesByType = new Dictionary<TangramShapeType, List<Transform>>();

    // Latest detections fed by caller
    private readonly List<DetectedShape> latestDetections = new List<DetectedShape>();

    // Public read-only results for external debug
    public IReadOnlyList<MatchResult> LastMatchResults => lastMatchResults;
    private readonly List<MatchResult> lastMatchResults = new List<MatchResult>();

    // MaterialPropertyBlock to avoid material instantiation
    private MaterialPropertyBlock sharedPropertyBlock;

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

        // Greedy nearest-neighbor per detection, scoped by shape type
        foreach (DetectedShape detection in latestDetections)
        {
            if (!availablePieces.TryGetValue(detection.shapeType, out List<Transform> candidates) || candidates.Count == 0)
                continue;

            Transform bestPiece = null;
            float bestDistance = float.MaxValue;

            foreach (Transform piece in candidates)
            {
                float d = Vector3.Distance(detection.worldPosition, piece.position);
                if (d < bestDistance)
                {
                    bestDistance = d;
                    bestPiece = piece;
                }
            }

            if (bestPiece != null && bestDistance <= maxMatchDistanceMeters)
            {
                matchedPieceSet.Add(bestPiece);
                lastMatchResults.Add(new MatchResult
                {
                    shapeType = detection.shapeType,
                    arucoId = detection.arucoId,
                    matchedPieceTransform = bestPiece,
                    worldDistanceMeters = bestDistance
                });

                // Remove matched piece from candidate pool for this type
                candidates.Remove(bestPiece);
            }
        }

        ApplyDebugColors(matchedPieceSet);

        // Debug log in English for clarity
        foreach (var result in lastMatchResults)
        {
            Debug.Log($"[TangramMatcher] Matched {result.shapeType} (ArUco {result.arucoId}) -> Piece '{result.matchedPieceTransform.name}', distance = {result.worldDistanceMeters:F3} m");
        }
    }

    private void ApplyDebugColors(HashSet<Transform> matchedPieces)
    {
        // First, optionally reset all to unmatched color
        if (!highlightOnlyMatches)
        {
            foreach (var kv in diagramPiecesByType)
            {
                foreach (Transform piece in kv.Value)
                {
                    SetRendererColor(piece.gameObject, unmatchedColor);
                }
            }
        }

        // Then, set matched pieces to matchedColor
        foreach (Transform piece in matchedPieces)
        {
            SetRendererColor(piece.gameObject, matchedColor);
        }
    }

    private void SetRendererColor(GameObject go, Color color)
    {
        var renderer = go.GetComponent<MeshRenderer>();
        if (renderer == null)
            return;

        if (sharedPropertyBlock == null)
            sharedPropertyBlock = new MaterialPropertyBlock();

        renderer.GetPropertyBlock(sharedPropertyBlock);
        sharedPropertyBlock.SetColor("_Color", color);
        renderer.SetPropertyBlock(sharedPropertyBlock);
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
}


