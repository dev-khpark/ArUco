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
/// 
/// RECENT IMPROVEMENTS (2024):
/// - Fixed inconsistent angle calculation methods in error reporting and relation checking
/// - Error reporting now uses the same ComputeRelativeAngleDeg/ComputeRelativeAngleDegForDiagram functions
/// - This ensures consistency between matching logic and error reporting
/// - Added debug logging for the new angle calculation methods when debugLogRelativeAngles is enabled
/// </summary>
public class TangramMatcher : MonoBehaviour
{
    [Header("Angle Offset Settings")]
    [Tooltip("Global angle offset applied to connection angles and/or orientations (degrees). Default: 180")]
    [Range(0f, 360f)]
    public float angleOffset = 180f;

    [Tooltip("Apply angle offset to connection line angles between shapes")]
    public bool applyOffsetToConnectionAngles = true;

    [Tooltip("Apply angle offset to individual shape orientations")]
    public bool applyOffsetToOrientations = true;
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
    public float rotationWeightMetersPerDeg = 0.01f;

    [Tooltip("한 조각이 여러 이웃 관계 중 첫 번째 'PASS' 관계를 만족하면 즉시 매칭을 확정합니다.")]
    public bool matchOnFirstPassingRelation = true;

    [Tooltip("직전 프레임에 매칭됐던 조각이 잠시 매칭에 실패하는 경우(Flicker), 'MATCH FAILED' 경고를 억제합니다.")]
    public bool suppressFlickerWarnings = true;

    // 직전 프레임에 매칭되었던 조각들을 추적하기 위한 변수
    private readonly HashSet<Transform> _matchedPiecesLastFrame = new HashSet<Transform>();
    // Anchor-relative matching removed

    [Header("Distance Cost Model")]
    [Tooltip("Include absolute detection-to-piece center distance in candidate cost (uses world meters).")]
    public bool useAbsoluteDistanceInCost = false;
    [Tooltip("Weight for absolute distance contribution (meters -> meters). Only used when useAbsoluteDistanceInCost is true.")]
    public float absoluteDistanceWeightMeters = 1.0f;
    [Tooltip("Use normalized relation distance ratios instead of raw meter differences. Ratios are computed relative to a baseline.")]
    public bool useNormalizedRelationDistance = true;
    [Tooltip("Weight converting pairwise center-angle error (degrees) into meters for two-detection fallback and general costs.")]
    public float pairwiseAngleWeightMetersPerDeg = 0.01f;
    [Tooltip("If true, use a unified baseline within the currently matched subset (max diagram distance among matched pieces) for ratio normalization.")]
    public bool useSubsetUnifiedBaseline = true;

    [Header("First-Pass Relative Scale")]
    [Tooltip("첫 PASS 관계에서 전역 스케일 s=d_actual/d_expected 를 정하고, 이후 거리는 s에 대한 ±허용비율로 평가합니다.")]
    public bool useFirstPassRelativeScale = true;

    [Range(0f, 1f)]
    [Tooltip("상대 허용 비율 (0.25 = ±25%)")]
    public float firstPassScaleTolerance = 2.25f;

    // Internal state for first-pass relative scale
    private bool firstPassScaleValid = false;
    private float firstPassScale = 1f;
    private int firstPassScaleFrame = -1;

    [Header("First-Pass Direction Offset")]
    [Tooltip("첫 PASS 관계에서 도형 간 연결 각도의 오프셋(사용자 배치 기준)을 고정하고, 이후엔 기대 각도에 이 오프셋을 더해 비교합니다.")]
    public bool useFirstPassDirectionOffset = true;
    [Tooltip("연결 각도 오프셋을 캡처하기 전에는 연결 각도를 무시하고 ORI만으로 첫 PASS를 허용합니다.")]
    public bool ignoreConnectionAngleUntilOffsetLocked = true;
    private bool firstPassDirOffsetValid = false;
    private float firstPassDirOffsetDeg = 0f;
    private int firstPassDirOffsetFrame = -1;

    [Header("Diagram Graph Settings")]
    [Tooltip("Adjacency threshold as a ratio of the maximum pairwise center distance among diagram pieces. Create an edge if (centerDist / maxDiagramPairDist) <= this value.")]
    public float diagramAdjacencyMaxNormalizedDistance = 0.8f;
    [Tooltip("If true, build diagram graph edges by 2D mesh intersection on the diagram plane instead of distance thresholds.")]
    public bool diagramEdgesUseMeshIntersection = true;
    [Tooltip("When using mesh-intersection edges, do a fast Renderer.bounds.Intersects pre-check before polygon overlap test.")]
    public bool useBoundsPrecheckForMeshEdges = true;
    [Tooltip("Maximum neighbor edges to keep per node (closest first). 0 = unlimited.")]
    public int diagramMaxNeighborsPerNode = 4;
    [Tooltip("Graph matching distance tolerance (meters).")]
    public float graphDistanceToleranceMeters = 0.5f;
    [Tooltip("Graph matching angle tolerance (degrees).")]
    public float graphAngleToleranceDeg = 45f;

    [Header("Graph-Only Matching (no absolute coordinates)")]
    
    [Tooltip("Detection graph adjacency threshold as a ratio of the maximum pairwise center distance among detections. Create edge if (centerDist / maxDetectionPairDist) <= this value.")]
    public float detectionAdjacencyMaxNormalizedDistance = 0.8f;
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

    [Header("Relative Angle Debugging")]
    [Tooltip("Enable detailed debugging logs for relative angle calculations.")]
    public bool debugLogRelativeAngles = true;
    [Tooltip("Log relative angle calculation process during diagram graph construction.")]
    public bool debugLogDiagramGraphAngles = true;
    [Tooltip("Log relative angle calculation process between detected shapes.")]
    public bool debugLogDetectionAngles = true;
    [Tooltip("Log angle comparison results between diagram and detected shapes.")]
    public bool debugLogAngleComparison = true;

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
    [Tooltip("Overlay per-detection ArUco Euler angles (XYZ in degrees) near each detected marker.")]
    public bool showDetectionEulerUI = true;
    [Header("Detection Orientation Debug")]
    [Tooltip("Draw a small world-space arrow over each detected marker, indicating its local +X direction projected to the diagram plane.")]
    public bool drawDetectionOrientationArrows = true;
    [Tooltip("Arrow length in meters.")]
    public float detectionOrientationArrowLength = 0.08f;
    [Tooltip("Arrow width in meters.")]
    public float detectionOrientationArrowWidth = 0.004f;
    public Color detectionOrientationArrowColor = Color.green;
    private readonly Dictionary<int, LineRenderer> orientationArrowById = new Dictionary<int, LineRenderer>();
    public Color uiLabelColor = Color.white;
    public int uiFontSize = 14;
    public Camera uiCamera;

    
    [Header("Lock Debugging")]
    [Tooltip("Log current locked pieces and their states each frame (piece, type, locked, confidence, last seen, matched det)")]
    public bool debugLogLockedStates = true;
    [Tooltip("Log pre-assignment decisions that attach detections to locked pieces before matching")]
    public bool debugLogPreAssignments = true;
    [Tooltip("Log reasons when locked pieces fail to match this frame.")]
    public bool debugLockFailures = true;

    [Header("Lock Policy")]
    [Tooltip("Require at least one passing relation (vs matched neighbors on diagram edges) for a piece to gain lock confidence.")]
    public bool requireRelationPassForLock = true;
    [Tooltip("When requiring relation pass for lock, demand all diagram-edge relations to matched neighbors to pass (true) or any one to pass (false). If false, use minimum neighbor count requirement instead.")]
    public bool relationLockRequireAllNeighbors = false;
    
    [Tooltip("Minimum number of neighbors that must pass relation checks (when relationLockRequireAllNeighbors is false)")]
    [Range(1, 5)]
    public int minNeighborsForLocked = 1;
    
    [Tooltip("Log relation pass/fail decisions that affect locking.")]
    public bool debugLogLockRelationDecision = true;
    [Tooltip("Immediately set lock (confidence=1.0) when relation pass condition is satisfied, instead of gradual confidence gain.")]
    public bool immediateLockOnRelationPass = true;
    [Header("Relation Best-Fit Options")]
    [Tooltip("When a neighbor's matched piece is not a direct diagram edge to the anchor, allow using the anchor's best-fit diagram neighbor (min expected diff) for REL logging.")]
    public bool allowBestFitEdgeForRelation = true;
    [Tooltip("When evaluating relation PASS for locking, if the specific diagram neighbor piece is not matched this frame, allow using any matched detection of the neighbor's type as a proxy.")]
    public bool allowCrossMatchedNeighborForRelationPass = true;

    [Header("Preserve Existing Matches")]
    [Tooltip("When enabled, keep previous matches unless the detection moves beyond an extreme threshold. Greatly reduces interference when new shapes appear.")]
    public bool preserveLockUnlessExtreme = true;
    [Tooltip("Extreme distance (meters) beyond which we allow re-matching away from the previously assigned/locked piece.")]
    public float extremeReassignDistanceMeters = 0.30f;
    [Tooltip("If true, do not unlock or decay confidence for locked pieces when not seen; they remain locked until an extreme change is observed.")]
    public bool stickyLocks = true;
    [Tooltip("Maximum frames a locked piece can remain unseen before unlocking, even with stickyLocks=true. 0 = unlimited.")]
    public int maxUnseenFrames = 50;

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

    [Header("Absolute Orientation Check")]
    [Tooltip("If true, compare each detection's local +X direction to the diagram piece direction defined by child 'direction' from 'center'.")]
    public bool useAbsoluteOrientation = true;
    [Tooltip("Tolerance for absolute orientation difference (degrees). Candidates exceeding this are rejected during relation matching.")]
    public float absoluteOrientationToleranceDeg = 35f;
    [Tooltip("Weight converting absolute orientation error (degrees) into meters for relational cost.")]
    public float absoluteOrientationWeightMetersPerDeg = 0.005f;

    [Header("Orientation Debugging")]
    [Tooltip("Log absolute orientation differences (center->direction vs marker +X) in the console.")]
    public bool debugLogOrientationDiff = true;
    [Tooltip("Log when candidates are rejected due to exceeding absolute orientation tolerance.")]
    public bool debugLogOrientationRejections = true;
    [Tooltip("When only one detection of a type exists but multiple diagram pieces of that type are present, assign to the piece with the smallest absolute orientation difference (ignoring relations).")]
    public bool forceOrientationForSingleDetection = true;
    [Tooltip("Log per-piece orientation sweep (angle to every candidate piece) for each detection.")]
    public bool debugLogOrientationSweep = false;
    [Tooltip("Log per detection the absolute orientation difference to its assigned or best-matching diagram piece (independent of relations).")]
    [System.NonSerialized]
    public bool debugLogOrientationPerDetection = true;

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

    [Header("Plane Normal Override")]
    [Tooltip("If true, force planar computations to XY plane (normal = +Z). Else, use tangramDiagramRoot.up or world up Y.")]
    public bool useXYPlane = true;

    [Header("Marker Plane (4-corner gating)")]
    [Tooltip("Require 4 plane-corner ArUco markers to be detected to run matching.")]
    public bool requirePlaneCorners = true;
    [Tooltip("Corner marker IDs in order: Top-Left, Top-Right, Bottom-Right, Bottom-Left")]
    public int planeCornerIdTL = 10;
    public int planeCornerIdTR = 11;
    public int planeCornerIdBR = 12;
    public int planeCornerIdBL = 13;
    [Tooltip("Draw debug axes/edges for the detected marker plane.")]
    public bool debugDrawMarkerPlane = true;
    [Tooltip("If true and corners missing, show an on-screen message.")]
    public bool showMissingCornersUI = true;

    // Runtime corner plane state
    private bool planeCornersReady = false;
    private Vector3 planeTLWorld, planeTRWorld, planeBRWorld, planeBLWorld;
    private int planeCornersFrame = -1000;

    [Tooltip("If true, map detections from marker plane (TL,TR,BL) into diagram plane via 2D similarity (translation/rotation/scale) using target width/height.")]
    public bool useMarkerPlaneSimilarity = true;
    [Tooltip("Target width (TL->TR) in diagram meters for marker plane mapping.")]
    public float markerTargetWidthMeters = 1.0f;
    [Tooltip("Target height (TL->BL) in diagram meters for marker plane mapping.")]
    public float markerTargetHeightMeters = 1.0f;

    [Header("Prediction (Kalman CV)")]
    [Tooltip("Use constant-velocity Kalman prediction for locked pieces to improve temporal coherence.")]
    public bool useKalmanPrediction = true;
    [Tooltip("Process noise (Q) for Kalman per axis.")]
    public float kalmanProcessNoise = 1e-2f;
    [Tooltip("Measurement noise (R) for Kalman per axis.")]
    public float kalmanMeasurementNoise = 1e-1f;

    [Header("Optimal Assignment")]
    [Tooltip("Use Hungarian algorithm for per-type multi-detection multi-piece assignment.")]
    public bool useHungarianForTypeMatching = true;

    // ---------- Gating ----------
    [Header("Gating")]
    [Tooltip("Use Mahalanobis gating with Kalman covariance for pre-assignment of locked pieces.")]
    public bool useMahalanobisGating = true;
    [Tooltip("Gate width in sigmas (typical 2.5~3.0).")]
    public float gatingSigma = 2.8f;
    [Tooltip("If true, gating is done in the diagram plane (X/Z). If false, 3D.")]
    public bool gatingPlanar = true;

    // ---------- Relation Filtering ----------
    [Header("Relation Filtering")]
    [Tooltip("Compare detection relations only against diagram GRAPH EDGES (adjacent nodes).")]
    public bool restrictRelationsToGraphEdges = true;
    [Tooltip("Maximum allowed |Δdistance| in meters for a relation candidate to be considered.")]
    public float relationMaxDistDiff = 0.12f;
    [Tooltip("Maximum allowed |Δangle| in degrees for a relation candidate to be considered.")]
    public float relationMaxAngleDiffDeg = 30f;
    [Tooltip("If true, log only PASSED relations; candidates are logged with a different tag.")]
    public bool verboseRelationDebug = false;

    [Header("Relation Debugging")]
    [Tooltip("If true, after matching, report diffs for ALL diagram edges touching matched pieces (even if relation was not used in matching).")]
    public bool debugAllDiagramEdges = true;
    [Tooltip("Limit number of diagram edges to report per node. 0 = all.")]
    public int debugEdgesReportPerNode = 0;
    [Tooltip("Draw debug lines for diagram edges during relation reporting.")]
    public bool drawRelationEdgeLines = false;

    [Header("Detection Relation Debugging")]
    [Tooltip("If true, also report distances/angles between DETECTED shapes (detection graph edges).")]
    public bool debugDetectionRelations = true;
    [Tooltip("Limit number of detection relations to report per node. 0 = all.")]
    public int debugDetectionRelationsPerNode = 0;
    [Tooltip("Draw debug lines for detection relations during reporting.")]
    public bool drawDetectionRelationLines = false;
    [Tooltip("Restrict relation comparisons to pairs that are connected by an edge in the diagram graph (expected relations only).")]
    public bool restrictRelationsToDiagramEdges = true;
    [Tooltip("If true, when a detection lacks a current-frame match, fall back to the last assigned piece for expected relation lookup (reduces '(no expected)' flicker).")]
    public bool useExpectedFallbackFromLastAssignment = true;
    [Tooltip("If true, suppress logging of '(no expected)' relation lines; only log when expectations are available.")]
    public bool suppressNoExpectedRelationLogs = true;

    [Header("Detection Buffering")]
    [Tooltip("Number of recent frames to buffer for corner gating and detection fusion.")]
    public int detectionBufferWindow = 4;
    private readonly Queue<(int frame, List<DetectedShape> items)> recentDetectionsBuffer = new Queue<(int frame, List<DetectedShape> items)>();
    // Optional external corner feed buffer (raw positions by ID)
    private readonly Queue<(int frame, Dictionary<int, Vector3> corners)> recentCornerBuffer = new Queue<(int frame, Dictionary<int, Vector3> corners)>();

    /// <summary>
    /// Optional: externally register a single corner marker observation (world position) for fusion over the buffer window.
    /// Call this when ArUco 10/11/12/13 are detected, even if they are not included in SetDetections.
    /// </summary>
    public void RegisterCornerObservation(int arucoId, Vector3 worldPosition)
    {
        var dict = new Dictionary<int, Vector3> { { arucoId, worldPosition } };
        recentCornerBuffer.Enqueue((frameCounter, dict));
        while (recentCornerBuffer.Count > Mathf.Max(1, detectionBufferWindow)) recentCornerBuffer.Dequeue();
    }

    /// <summary>
    /// Optional: externally register multiple corner marker observations in a single call.
    /// Provide a map of arucoId -> worldPosition for any subset of {TL, TR, BR, BL}.
    /// </summary>
    public void RegisterCornerObservations(Dictionary<int, Vector3> corners)
    {
        if (corners == null || corners.Count == 0) return;
        recentCornerBuffer.Enqueue((frameCounter, new Dictionary<int, Vector3>(corners)));
        while (recentCornerBuffer.Count > Mathf.Max(1, detectionBufferWindow)) recentCornerBuffer.Dequeue();
    }

    [Header("Hysteresis / Stickiness")]
    [Tooltip("Prefer the previously assigned diagram piece for a detection unless a new candidate improves cost beyond a threshold.")]
    public bool preferPreviousAssignment = true;
    [Tooltip("Minimal cost improvement (meters-equivalent) required to switch assignment away from the previous piece.")]
    public float switchCostImprovementThreshold = 0.10f;
    [Tooltip("Cost bias in meters-equivalent to prefer previous assignment during multi-detection assignment.")]
    public float previousAssignmentBiasMeters = 0.05f;
    [Tooltip("Prevent switching within this many frames unless improvement beats threshold.")]
    public int assignmentCooldownFrames = 20;
    [Tooltip("Extra penalty (meters-equivalent) applied to switching during cooldown.")]
    public float cooldownPenaltyMeters = 0.3f;

    [Header("Stateful Matching & Confidence")]
    [Tooltip("Enable locking of stable matches across frames for temporal coherence.")]
    public bool enableStatefulMatching = true;
    [Tooltip("Confidence gain per successful re-detection (0..1).")]
    public float confidenceGainPerHit = 0.2f;
    [Tooltip("Confidence decay per frame when not seen (0..1).")]
    public float confidenceDecayPerFrame = 0.0f;
    [Tooltip("Confidence threshold to lock a piece match.")]
    public float lockConfidenceThreshold = 0.7f;
    [Tooltip("Confidence threshold to unlock a previously locked piece.")]
    public float unlockConfidenceThreshold = 0.2f;
    [Tooltip("Max distance to re-associate a detection to a locked piece (meters).")]
    public float relockMaxDistanceMeters = 1.5f;

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
        public bool isCorner; // true if this detection is one of the 4 plane corner markers
        // Cached planar metrics for this frame (computed in SetDetections)
        public Vector2 planeCoord; // coordinates on the diagram plane basis (axisU, axisV) relative to plane origin
        public Vector3 projectedWorldPosition; // world-space point projected onto the diagram plane
        public float planeAngleDeg; // detection's local +X projected onto plane, angle w.r.t. axisU
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
        public Quaternion detectionWorldRotation; // ★ 추가: 감지된 도형의 월드 회전을 저장하여 상대 각도 계산에 사용
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
    // Queue REL logs and flush at end of frame to avoid (no expected) timing artifacts
    private readonly List<string> relLogQueue = new List<string>();
    
    // Corner-based projection system with stabilization
    private class CornerPlaneSystem
    {
        public Vector3 origin;           // Origin point of the plane (first corner)
        public Vector3 planeNormal;     // Normal vector of the plane
        public Vector3 uAxis;           // U axis (right direction) in 2D plane
        public Vector3 vAxis;           // V axis (up direction) in 2D plane
        public bool isValid;            // Whether the plane system is valid
        public int[] cornerIds;         // Which corner IDs were used (for debugging)
        public float stability;         // Stability metric (higher = more stable)
        public int frameCount;          // Number of consecutive frames this plane has been valid
        
        public CornerPlaneSystem()
        {
            origin = Vector3.zero;
            planeNormal = Vector3.up;
            uAxis = Vector3.right;
            vAxis = Vector3.forward;
            isValid = false;
            cornerIds = new int[3];
            stability = 0f;
            frameCount = 0;
        }
        
        /// <summary>
        /// Creates a copy of this corner plane system
        /// </summary>
        public CornerPlaneSystem Clone()
        {
            var copy = new CornerPlaneSystem();
            copy.origin = this.origin;
            copy.planeNormal = this.planeNormal;
            copy.uAxis = this.uAxis;
            copy.vAxis = this.vAxis;
            copy.isValid = this.isValid;
            copy.stability = this.stability;
            copy.frameCount = this.frameCount;
            for (int i = 0; i < 3; i++) copy.cornerIds[i] = this.cornerIds[i];
            return copy;
        }
    }
    
    private CornerPlaneSystem currentCornerPlane = new CornerPlaneSystem();
    private CornerPlaneSystem previousCornerPlane = new CornerPlaneSystem();
    
    // Stabilized corner positions (keyed by ArUco ID)
    private readonly Dictionary<int, Vector3> stabilizedCornerPositions = new Dictionary<int, Vector3>();
    private readonly Dictionary<int, Vector3> rawCornerPositions = new Dictionary<int, Vector3>();
    private readonly Dictionary<int, int> cornerSeenFrames = new Dictionary<int, int>();
    private int currentFrame = 0;
    
    // Corner stabilization settings
    [Header("Corner Stabilization")]
    [Tooltip("Enable corner coordinate stabilization to reduce frame-to-frame jitter in plane calculation")]
    public bool enableCornerStabilization = true;
    [Tooltip("Smoothing factor for corner positions (0=no smoothing, 1=max smoothing)")]
    [Range(0f, 0.95f)]
    public float cornerSmoothingFactor = 0.7f;
    [Tooltip("Minimum change in corner position (meters) required to trigger plane recalculation")]
    public float cornerMovementThreshold = 0.01f;
    [Tooltip("Minimum angle change (degrees) in plane normal required to trigger plane recalculation")]
    public float planeAngleThreshold = 2.0f;
    [Tooltip("Minimum number of consecutive frames a plane must be stable before it can be replaced")]
    public int minStableFrames = 3;
    [Tooltip("Maximum allowed angular deviation (degrees) from previous plane normal")]
    public float maxPlaneAngleDeviation = 15.0f;
    
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
    
    // Fast diagram edge membership set
    private readonly HashSet<ulong> diagramEdgeSet = new HashSet<ulong>();
    private static ulong EdgeKey(int a, int b) => ((ulong)(uint)a << 32) | (uint)(b);
    private bool IsDiagramEdge(int i, int j)
    {
        return diagramEdgeSet.Contains(EdgeKey(i, j)) || diagramEdgeSet.Contains(EdgeKey(j, i));
    }

    // ---------- Geometry helpers for mesh-intersection edges ----------
    // Project each piece's mesh vertices onto the diagram plane and compute a convex hull in 2D.
    private List<Vector2> GetProjectedConvexHull2D(Transform piece, Vector3 planeNormal, Vector3 axisU, Vector3 axisV)
    {
        var result = new List<Vector2>();
        if (piece == null) return result;
        var meshFilter = piece.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null) return result;
        var verts = meshFilter.sharedMesh.vertices;
        if (verts == null || verts.Length == 0) return result;

        // Collect projected points
        var pts = new List<Vector2>(verts.Length);
        for (int i = 0; i < verts.Length; i++)
        {
            Vector3 world = piece.TransformPoint(verts[i]);
            Vector3 p = Vector3.ProjectOnPlane(world, planeNormal);
            float u = Vector3.Dot(p, axisU);
            float v = Vector3.Dot(p, axisV);
            pts.Add(new Vector2(u, v));
        }

        // Compute convex hull (Monotone chain)
        pts.Sort((a, b) => a.x == b.x ? a.y.CompareTo(b.y) : a.x.CompareTo(b.x));
        var hull = new List<Vector2>();
        // lower
        for (int i = 0; i < pts.Count; i++)
        {
            while (hull.Count >= 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], pts[i]) <= 0f) hull.RemoveAt(hull.Count - 1);
            hull.Add(pts[i]);
        }
        // upper
        int t = hull.Count + 1;
        for (int i = pts.Count - 2; i >= 0; i--)
        {
            while (hull.Count >= t && Cross(hull[hull.Count - 2], hull[hull.Count - 1], pts[i]) <= 0f) hull.RemoveAt(hull.Count - 1);
            hull.Add(pts[i]);
        }
        if (hull.Count > 1) hull.RemoveAt(hull.Count - 1);
        return hull;
    }

    private float Cross(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    // Convex polygon intersection test in 2D using Separating Axis Theorem
    private bool ConvexPolygonsIntersect(List<Vector2> hullA, List<Vector2> hullB)
    {
        if (hullA == null || hullB == null || hullA.Count < 3 || hullB.Count < 3) return false;
        if (!OverlapOnAllAxes(hullA, hullB)) return false;
        if (!OverlapOnAllAxes(hullB, hullA)) return false;
        return true;
    }

    private bool OverlapOnAllAxes(List<Vector2> polyA, List<Vector2> polyB)
    {
        for (int i = 0; i < polyA.Count; i++)
        {
            Vector2 p1 = polyA[i];
            Vector2 p2 = polyA[(i + 1) % polyA.Count];
            Vector2 edge = p2 - p1;
            Vector2 axis = new Vector2(-edge.y, edge.x).normalized;
            ProjectPolygon(polyA, axis, out float minA, out float maxA);
            ProjectPolygon(polyB, axis, out float minB, out float maxB);
            if (maxA < minB || maxB < minA) return false;
        }
        return true;
    }

    private void ProjectPolygon(List<Vector2> poly, Vector2 axis, out float min, out float max)
    {
        float d = Vector2.Dot(poly[0], axis);
        min = d; max = d;
        for (int i = 1; i < poly.Count; i++)
        {
            d = Vector2.Dot(poly[i], axis);
            if (d < min) min = d;
            else if (d > max) max = d;
        }
    }

    // Stateful matching data
    private int frameCounter = 0;
    private class LockedState
    {
        public Transform piece;
        public TangramShapeType type;
        public float confidence;
        public Vector3 lastPosition;
        public Vector3 lastVelocity;
        public int lastSeenFrame;
        public bool isLocked;
        // Kalman state per axis: position/velocity and covariance terms
        public bool kalmanInitialized;
        public Vector3 kPos;
        public Vector3 kVel;
        // Covariance for each axis (2x2): [p11 p12; p21 p22]
        public Vector4 kCovX; // (p11, p12, p21, p22)
        public Vector4 kCovY;
        public Vector4 kCovZ;
    }
    private readonly Dictionary<Transform, LockedState> lockedByPiece = new Dictionary<Transform, LockedState>();
    private readonly Dictionary<int, Transform> preAssignedDetections = new Dictionary<int, Transform>();
    // Remember last assignment per detection type/key (single-id tracking)
    private readonly Dictionary<(TangramShapeType type, int arucoId), Transform> lastAssignedPieceByKey = new Dictionary<(TangramShapeType, int), Transform>();
    private readonly HashSet<string> _gatingFailuresThisFrame = new HashSet<string>();
    // Each frame, store detailed failure reason per locked piece name
    private readonly Dictionary<string, string> _detailedFailureReasons = new Dictionary<string, string>();
    // Dynamic graph distance tolerance updated from successful relation checks
    private float _dynamicGraphDistanceTolerance;

    // Cached per-frame normalization baselines (subset reference)
    private float cachedBaselineDetRef = 1f;
    private float cachedBaselineExpRef = 1f;
    private int cachedBaselineFrame = -1;
    private bool cachedBaselineValid = false;
    private readonly HashSet<string> previousDetectionKeySet = new HashSet<string>();

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

        // Build fast edge set for relation filtering
        diagramEdgeSet.Clear();
        if (diagramGraph != null)
        {
            for (int i = 0; i < diagramGraph.nodes.Count; i++)
            {
                var ni = diagramGraph.nodes[i];
                for (int e = 0; e < ni.edges.Count; e++)
                {
                    int j = ni.edges[e].toIndex;
                    diagramEdgeSet.Add(EdgeKey(i, j));
                }
            }
        }

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
        // Reset dynamic tolerance at frame start
        _dynamicGraphDistanceTolerance = graphDistanceToleranceMeters;
        _detailedFailureReasons.Clear();
        _gatingFailuresThisFrame.Clear();
        latestDetections.Clear();
        if (detections != null)
        {
            // First pass: collect all detections and identify corners
            foreach (var d in detections)
            {
                // Pass through corner IDs even if TryMapArucoIdToShape would ignore them upstream
                bool isCornerId = (d.arucoId == planeCornerIdTL || d.arucoId == planeCornerIdTR || d.arucoId == planeCornerIdBR || d.arucoId == planeCornerIdBL);
                // Also check for corner IDs 10,11,12,13 for new corner plane system
                bool isNewCornerId = (d.arucoId >= 10 && d.arucoId <= 13);
                var dd = d;
                if (isCornerId || isNewCornerId) dd.isCorner = true;
                latestDetections.Add(dd);
            }
            
            // Build corner plane system using ArUco IDs 10,11,12,13
            BuildCornerPlaneSystem();
            
            // Second pass: compute planar coordinates using corner-based projection
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var dd = latestDetections[i];
                
                // Project to 2D using corner plane system
                dd.planeCoord = ProjectTo2DCornerPlane(dd.worldPosition);
                
                // For projectedWorldPosition, project onto the corner plane in 3D space
                if (currentCornerPlane.isValid)
                {
                    Vector3 relativePos = dd.worldPosition - currentCornerPlane.origin;
                    Vector3 projectedOnPlane = dd.worldPosition - Vector3.Dot(relativePos, currentCornerPlane.planeNormal) * currentCornerPlane.planeNormal;
                    dd.projectedWorldPosition = projectedOnPlane;
                }
                else
                {
                    // Fallback projection if corner plane not available
                    Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 planeOrigin = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
                    dd.projectedWorldPosition = dd.worldPosition - Vector3.Dot(dd.worldPosition - planeOrigin, planeN) * planeN;
                }
                
                // Calculate plane angle for individual shape orientation (keep existing logic)
                Vector3 orientPlaneN = currentCornerPlane.isValid ? currentCornerPlane.planeNormal : 
                    (useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up));
                Vector3 orientAxisU = currentCornerPlane.isValid ? currentCornerPlane.uAxis : 
                    Vector3.Cross(orientPlaneN, Vector3.up).normalized;
                if (orientAxisU.sqrMagnitude < 1e-6f) orientAxisU = Vector3.Cross(orientPlaneN, Vector3.right).normalized;
                
                Vector3 dirProj = Vector3.ProjectOnPlane(dd.worldRotation * Vector3.right, orientPlaneN).normalized;
                float aDeg = 0f;
                if (dirProj.sqrMagnitude > 1e-6f)
                {
                    aDeg = NormalizeAngle180(Vector3.SignedAngle(orientAxisU, dirProj, orientPlaneN));
                }
                dd.planeAngleDeg = aDeg;
                
                latestDetections[i] = dd;
            }
        }

        // push into buffer
        recentDetectionsBuffer.Enqueue((frameCounter, new List<DetectedShape>(latestDetections)));
        while (recentDetectionsBuffer.Count > Mathf.Max(1, detectionBufferWindow)) recentDetectionsBuffer.Dequeue();

        // union of detections over buffer window for robust gating
        var fusedDetections = FuseBufferedDetections();

        // 4-corner gating: ensure all plane markers are present
        if (requirePlaneCorners)
        {
            UpdatePlaneCornersFromList(fusedDetections);
            if (!planeCornersReady)
            {
                // Skip matching; optionally draw UI message
                if (showMissingCornersUI)
                {
                    int tl=0,tr=0,br=0,bl=0;
                    foreach (var d in fusedDetections)
                    {
                        if (d.arucoId == planeCornerIdTL) tl=1;
                        else if (d.arucoId == planeCornerIdTR) tr=1;
                        else if (d.arucoId == planeCornerIdBR) br=1;
                        else if (d.arucoId == planeCornerIdBL) bl=1;
                    }
                    Debug.Log($"[TangramMatcher] Corner markers not detected – skipping matching. present(TL={tl}, TR={tr}, BR={br}, BL={bl}) window={detectionBufferWindow}");
                }
                return;
            }
            // Overwrite detection positions with median-fused positions (non-corner) to reduce jitter
            var medById = new Dictionary<int, Vector3>();
            foreach (var d in fusedDetections) if (!d.isCorner) medById[d.arucoId] = d.worldPosition;
            for (int i = 0; i < latestDetections.Count; i++)
            {
                if (latestDetections[i].isCorner) continue;
                if (medById.TryGetValue(latestDetections[i].arucoId, out var mp))
                {
                    var tmp = latestDetections[i]; tmp.worldPosition = mp; latestDetections[i] = tmp;
                }
            }
            // Apply similarity mapping to align to diagram plane/scale
            if (useMarkerPlaneSimilarity)
            {
                ApplyMarkerPlaneSimilarityMapping();
            }
            else
            {
                // simple projection only
                Vector3 planeNormal = Vector3.Normalize(Vector3.Cross(planeTRWorld - planeTLWorld, planeBLWorld - planeTLWorld));
                for (int i = 0; i < latestDetections.Count; i++)
                {
                    var d = latestDetections[i];
                    Vector3 p = d.worldPosition;
                    Vector3 pProj = p - Vector3.Dot(p - planeTLWorld, planeNormal) * planeNormal;
                    d.worldPosition = pProj;
                    latestDetections[i] = d;
                }
            }
            // Recompute planar caches using corner-based projection after potential worldPosition updates
            // Rebuild corner plane system in case positions changed
            BuildCornerPlaneSystem();
            
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var dd = latestDetections[i];
                
                // Use corner-based projection (don't overwrite with old method)
                dd.planeCoord = ProjectTo2DCornerPlane(dd.worldPosition);
                
                // For projectedWorldPosition, project onto the corner plane in 3D space
                if (currentCornerPlane.isValid)
                {
                    Vector3 relativePos = dd.worldPosition - currentCornerPlane.origin;
                    Vector3 projectedOnPlane = dd.worldPosition - Vector3.Dot(relativePos, currentCornerPlane.planeNormal) * currentCornerPlane.planeNormal;
                    dd.projectedWorldPosition = projectedOnPlane;
                }
                else
                {
                    // Fallback projection if corner plane not available
                    Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 planeOrigin = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
                    dd.projectedWorldPosition = dd.worldPosition - Vector3.Dot(dd.worldPosition - planeOrigin, planeN) * planeN;
                }
                
                // Calculate plane angle for individual shape orientation using corner plane
                Vector3 orientPlaneN = currentCornerPlane.isValid ? currentCornerPlane.planeNormal : 
                    (useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up));
                Vector3 orientAxisU = currentCornerPlane.isValid ? currentCornerPlane.uAxis : 
                    Vector3.Cross(orientPlaneN, Vector3.up).normalized;
                if (orientAxisU.sqrMagnitude < 1e-6f) orientAxisU = Vector3.Cross(orientPlaneN, Vector3.right).normalized;
                
                Vector3 dirProj = Vector3.ProjectOnPlane(dd.worldRotation * Vector3.right, orientPlaneN).normalized;
                float aDeg = 0f;
                if (dirProj.sqrMagnitude > 1e-6f)
                {
                    aDeg = NormalizeAngle180(Vector3.SignedAngle(orientAxisU, dirProj, orientPlaneN));
                }
                dd.planeAngleDeg = aDeg;
                
                latestDetections[i] = dd;
            }
            InvalidateBaselinesIfNewDetectionsPresent();
        }

        frameCounter++;
        if (enableStatefulMatching)
        {
            // Unlock long-unseen locks even when stickyLocks is enabled
            if (stickyLocks && maxUnseenFrames > 0)
            {
                var toUnlock = new List<Transform>();
                foreach (var kv in lockedByPiece)
                {
                    var ls = kv.Value;
                    if (!ls.isLocked) continue;
                    int unseenFrames = frameCounter - ls.lastSeenFrame;
                    if (unseenFrames > maxUnseenFrames)
                    {
                        ls.isLocked = false;
                        // Force below unlock threshold to ensure immediate unlock semantics
                        ls.confidence = Mathf.Min(ls.confidence, unlockConfidenceThreshold - 0.01f);
                        ls.kalmanInitialized = false;
                        toUnlock.Add(kv.Key);
                        if (debugLogLockedStates)
                        {
                            Debug.Log($"[TangramMatcher] Unlocked {ls.piece.name} ({ls.type}) after {unseenFrames} unseen frames (max={maxUnseenFrames})");
                        }
                    }
                }
                foreach (var t in toUnlock) lockedByPiece.Remove(t);
            }
            // Pre-assignment: try to re-associate detections to locked pieces of same type within distance
            preAssignedDetections.Clear();
            foreach (var kv in lockedByPiece)
            {
                var ls = kv.Value;
                if (!ls.isLocked) continue;
                // predict with Kalman CV
                Vector3 predicted;
                if (useKalmanPrediction)
                {
                    predicted = KalmanPredict(ls, Time.deltaTime);
                }
                else
                {
                    predicted = ls.lastPosition + ls.lastVelocity * Time.deltaTime;
                }
                // pre-assign to locked pieces with gating
                int bestIdx = -1;
                float bestScore = float.PositiveInfinity;

                for (int i = 0; i < latestDetections.Count; i++)
                {
                    if (preAssignedDetections.ContainsKey(i)) continue;
                    var d = latestDetections[i];
                    if (d.isCorner) continue; // skip plane corner markers entirely
                    if (d.shapeType != ls.type) continue;

                    bool pass;
                    float score;

                    if (useMahalanobisGating && ls.kalmanInitialized)
                    {
                        score = MahalanobisDistanceSquared(ls, d.worldPosition, gatingPlanar);
                        pass = score <= gatingSigma * gatingSigma;
                    }
                    else
                    {
                        // Euclidean fallback gate
                        score = Vector3.Distance(d.worldPosition, predicted);
                        pass = score <= relockMaxDistanceMeters;
                    }

                    // If we preserve lock unless extreme, only accept if not an extreme deviation
                    bool extremeOk = true;
                    if (preserveLockUnlessExtreme)
                    {
                        float extremeDist = Vector3.Distance(d.worldPosition, ls.lastPosition);
                        if (extremeDist > extremeReassignDistanceMeters)
                            extremeOk = true; // allow re-attach if extremely far
                        // else prefer staying with current lock; we'll only accept if this is the locked piece
                        if (kv.Key != ls.piece && extremeDist <= extremeReassignDistanceMeters)
                            extremeOk = false;
                    }

                    if (pass && extremeOk && score < bestScore)
                    {
                        bestScore = score;
                        bestIdx = i;
                    }
                }

                if (bestIdx >= 0)
                {
                    preAssignedDetections[bestIdx] = ls.piece;
                    // Keep filter tight
                    if (useKalmanPrediction)
                    {
                        KalmanUpdate(ls, latestDetections[bestIdx].worldPosition);
                    }
                    if (debugLogPreAssignments)
                    {
                        var d = latestDetections[bestIdx];
                        Debug.Log($"[TangramMatcher] PREASSIGN det({d.shapeType}:{d.arucoId}) -> lock({ls.type}:{ls.piece.name}) score={bestScore:F3}");
                    }
                }
                else
                {
                    // No acceptable candidate for this locked piece; if any candidates of the same type existed this frame, record gating failure
                    int candidatesOfType = 0;
                    float minCandidateDist = float.PositiveInfinity;
                    for (int i = 0; i < latestDetections.Count; i++)
                    {
                        var d = latestDetections[i];
                        if (d.isCorner || d.shapeType != ls.type) continue;
                        candidatesOfType++;
                        Vector3 planeN_abs = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                        Vector3 vAbs = Vector3.ProjectOnPlane(d.worldPosition - predicted, planeN_abs);
                        float dist = vAbs.magnitude;
                        if (dist < minCandidateDist) minCandidateDist = dist;
                    }
                    if (candidatesOfType > 0)
                    {
                        Debug.LogWarning($"[TangramMatcher] GATING FAILED for locked piece '{ls.piece.name}'. Found {candidatesOfType} detection(s) of type {ls.type}, but the closest one was {minCandidateDist:F3}m away (gate threshold: {relockMaxDistanceMeters:F3}m).");
                        _gatingFailuresThisFrame.Add(ls.piece.name);
                    }
                }
            }
        }

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

            bool isCornerId = (arucoId == planeCornerIdTL || arucoId == planeCornerIdTR || arucoId == planeCornerIdBR || arucoId == planeCornerIdBL);
            if (!TryMapArucoIdToShape(arucoId, out TangramShapeType shapeType))
            {
                if (!isCornerId)
                    continue; // ignore unknown non-corner ids
                // Allow corner ids through with a placeholder shape type (won't be used for matching)
                shapeType = TangramShapeType.Square;
            }

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

            // Also check for corner IDs 10,11,12,13 for new corner plane system
            bool isNewCornerId = (arucoId >= 10 && arucoId <= 13);
            
            latestDetections.Add(new DetectedShape
            {
                arucoId = arucoId,
                shapeType = shapeType,
                worldPosition = worldPos,
                worldRotation = worldRot,
                confidence = 1f,
                isCorner = isCornerId || isNewCornerId
            });
        }
        // 4-corner gating and homography-based 2D mapping
        if (requirePlaneCorners)
        {
            // push into buffer and fuse
            recentDetectionsBuffer.Enqueue((frameCounter, new List<DetectedShape>(latestDetections)));
            while (recentDetectionsBuffer.Count > Mathf.Max(1, detectionBufferWindow)) recentDetectionsBuffer.Dequeue();
            var fused = FuseBufferedDetections();
            UpdatePlaneCornersFromList(fused);
            if (!planeCornersReady)
            {
                int tl=0,tr=0,br=0,bl=0;
                foreach (var d in fused)
                {
                    if (d.arucoId == planeCornerIdTL) tl=1;
                    else if (d.arucoId == planeCornerIdTR) tr=1;
                    else if (d.arucoId == planeCornerIdBR) br=1;
                    else if (d.arucoId == planeCornerIdBL) bl=1;
                }
                Debug.Log($"[TangramMatcher] Corner markers not detected – skipping matching. present(TL={tl}, TR={tr}, BR={br}, BL={bl}) window={detectionBufferWindow}");
                return;
            }
            // Optional: draw plane edges for debug
            if (debugDrawMarkerPlane)
            {
                Debug.DrawLine(planeTLWorld, planeTRWorld, Color.cyan, 0f, false);
                Debug.DrawLine(planeTRWorld, planeBRWorld, Color.cyan, 0f, false);
                Debug.DrawLine(planeBRWorld, planeBLWorld, Color.cyan, 0f, false);
                Debug.DrawLine(planeBLWorld, planeTLWorld, Color.cyan, 0f, false);
            }
            if (useMarkerPlaneSimilarity)
            {
                ApplyMarkerPlaneSimilarityMapping();
            }
            else
            {
                // simple projection only
                Vector3 planeNormal = Vector3.Normalize(Vector3.Cross(planeTRWorld - planeTLWorld, planeBLWorld - planeTLWorld));
                for (int i = 0; i < latestDetections.Count; i++)
                {
                    var d = latestDetections[i];
                    Vector3 p = d.worldPosition;
                    Vector3 pProj = p - Vector3.Dot(p - planeTLWorld, planeNormal) * planeNormal;
                    d.worldPosition = pProj;
                    latestDetections[i] = d;
                }
            }
            
            // Apply corner-based projection to all detections
            BuildCornerPlaneSystem();
            
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var dd = latestDetections[i];
                
                // Project to 2D using corner plane system
                dd.planeCoord = ProjectTo2DCornerPlane(dd.worldPosition);
                
                // For projectedWorldPosition, project onto the corner plane in 3D space
                if (currentCornerPlane.isValid)
                {
                    Vector3 relativePos = dd.worldPosition - currentCornerPlane.origin;
                    Vector3 projectedOnPlane = dd.worldPosition - Vector3.Dot(relativePos, currentCornerPlane.planeNormal) * currentCornerPlane.planeNormal;
                    dd.projectedWorldPosition = projectedOnPlane;
                }
                else
                {
                    // Fallback projection if corner plane not available
                    Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 planeOrigin = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
                    dd.projectedWorldPosition = dd.worldPosition - Vector3.Dot(dd.worldPosition - planeOrigin, planeN) * planeN;
                }
                
                // Calculate plane angle for individual shape orientation using corner plane
                Vector3 orientPlaneN = currentCornerPlane.isValid ? currentCornerPlane.planeNormal : 
                    (useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up));
                Vector3 orientAxisU = currentCornerPlane.isValid ? currentCornerPlane.uAxis : 
                    Vector3.Cross(orientPlaneN, Vector3.up).normalized;
                if (orientAxisU.sqrMagnitude < 1e-6f) orientAxisU = Vector3.Cross(orientPlaneN, Vector3.right).normalized;
                
                Vector3 dirProj = Vector3.ProjectOnPlane(dd.worldRotation * Vector3.right, orientPlaneN).normalized;
                float aDeg = 0f;
                if (dirProj.sqrMagnitude > 1e-6f)
                {
                    aDeg = NormalizeAngle180(Vector3.SignedAngle(orientAxisU, dirProj, orientPlaneN));
                }
                dd.planeAngleDeg = aDeg;
                
                latestDetections[i] = dd;
            }
            
            InvalidateBaselinesIfNewDetectionsPresent();
        }

        RunMatchingAndHighlight();
    }
#endif

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

        // Build edges
        for (int i = 0; i < diagramGraph.nodes.Count; i++)
        {
            var ni = diagramGraph.nodes[i];
            Vector3 pi = GetPieceCenterWorld(ni.piece);
            var allEdges = new List<DiagramGraph.Edge>();

            bool usingOverlapEdges = diagramEdgesUseMeshIntersection;

            // Precompute plane basis for 2D projection once
            Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
            Vector3 axisU = Vector3.Cross(planeN, Vector3.up);
            if (axisU.sqrMagnitude < 1e-6f) axisU = Vector3.Cross(planeN, Vector3.right);
            axisU.Normalize();
            Vector3 axisV = Vector3.Cross(planeN, axisU);

            // Cache convex hulls for all nodes
            var hullCache = new Dictionary<int, List<Vector2>>();
            for (int h = 0; h < diagramGraph.nodes.Count; h++)
            {
                hullCache[h] = GetProjectedConvexHull2D(diagramGraph.nodes[h].piece, planeN, axisU, axisV);
            }

            // Build edges either by mesh overlap on plane or legacy distance threshold
            int bestJ = -1;
            float bestDist = float.PositiveInfinity;
            float bestNorm = float.PositiveInfinity;
            float bestAngle = 0f;
            for (int j = 0; j < diagramGraph.nodes.Count; j++)
            {
                if (i == j) continue;
                var nj = diagramGraph.nodes[j];
                Vector3 pj = GetPieceCenterWorld(nj.piece);

                bool connect = false;
                float normalized = float.PositiveInfinity;
                if (usingOverlapEdges)
                {
                    // Optional fast 3D bounds precheck
                    if (useBoundsPrecheckForMeshEdges)
                    {
                        var ri = ni.piece.GetComponent<MeshRenderer>();
                        var rj = nj.piece.GetComponent<MeshRenderer>();
                        if (ri != null && rj != null && !ri.bounds.Intersects(rj.bounds))
                        {
                            connect = false;
                        }
                        else
                        {
                            connect = ConvexPolygonsIntersect(hullCache[i], hullCache[j]);
                        }
                    }
                    else
                    {
                        connect = ConvexPolygonsIntersect(hullCache[i], hullCache[j]);
                    }
                }
                else
                {
                    float centerDist = Vector3.Distance(pi, pj);
                    // Normalize by maximum diagram pairwise center distance for consistency
                    float maxPair = GetDiagramMaxEdgeLength();
                    normalized = maxPair > 1e-4f ? centerDist / maxPair : float.PositiveInfinity;
                    connect = (normalized <= diagramAdjacencyMaxNormalizedDistance);
                }

                if (connect)
                {
                    float centerDist = Vector3.Distance(pi, pj);
                    // ★ 변경점: 새로운 상대 각도 계산 방식을 사용합니다.
                    // ArUco ID를 기준으로 기준(from)과 목표(to) 조각을 일관되게 결정합니다.
                    int id_i = GetArucoIdForDiagramPiece(ni.piece);
                    int id_j = GetArucoIdForDiagramPiece(nj.piece);
                    Transform fromPiece, toPiece;
                    if (id_i >= 0 && id_j >= 0 && id_i > id_j)
                    {
                        fromPiece = nj.piece;
                        toPiece = ni.piece;
                    }
                    else
                    {
                        fromPiece = ni.piece;
                        toPiece = nj.piece;
                    }

                    // 디버깅 컨텍스트 생성
                    string debugContext = debugLogDiagramGraphAngles ? 
                        $"DIAGRAM {fromPiece.name}(ID:{id_i}) → {toPiece.name}(ID:{id_j})" : "";

                    // ★ 변경점: 다이어그램 전용 상대 각도 계산 함수('center'->'direction' 기반)를 호출합니다.
                    float angle = ComputeRelativeAngleDegForDiagram(fromPiece,
                        toPiece,
                        debugContext);

                    // 기존 방식과의 비교를 위한 로그 (선택적)
                    if (debugLogDiagramGraphAngles)
                    {
                        float legacyAngle = ComputePlanarAngleDeg(GetPieceCenterWorld(fromPiece), GetPieceCenterWorld(toPiece));
                        // Debug output in English for diagram graph angle comparison between new and legacy methods
                        Debug.Log($"[DiagramGraph] {fromPiece.name} → {toPiece.name}: " +
                                  $"New method = {angle:F2}°, Legacy method = {legacyAngle:F2}°, Difference = {Mathf.Abs(angle - legacyAngle):F2}°");
                    }
                    var edge = new DiagramGraph.Edge
                    {
                        toIndex = diagramGraph.indexByTransform[nj.piece],
                        expectedDistanceMeters = centerDist,
                        expectedAngleDeg = angle,
                        // Store normalized by max pair distance for logging symmetry
                        normalizedDistance = (GetDiagramMaxEdgeLength() > 1e-4f) ? centerDist / GetDiagramMaxEdgeLength() : 0f
                    };
                    allEdges.Add(edge);

                    if (!usingOverlapEdges)
                    {
                        if (centerDist < bestDist)
                        {
                            bestDist = centerDist;
                            bestJ = j;
                            bestNorm = edge.normalizedDistance;
                            bestAngle = angle;
                        }
                    }
                }
            }

            // If using mesh-overlap edges, ignore nearest-neighbor pruning and keep all intersecting edges
            if (diagramOnlyNearestNeighbor && !usingOverlapEdges)
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
                // If using mesh-overlap edges, keep all intersecting connections as requested (do not trim)
                if (!usingOverlapEdges)
                {
                    if (diagramMaxNeighborsPerNode > 0 && ni.edges.Count > diagramMaxNeighborsPerNode)
                    {
                        ni.edges.Sort((a, b) => a.expectedDistanceMeters.CompareTo(b.expectedDistanceMeters));
                        ni.edges.RemoveRange(diagramMaxNeighborsPerNode, ni.edges.Count - diagramMaxNeighborsPerNode);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Gets the ArUco ID associated with a diagram piece by looking up its shape type and position.
    /// This is used to determine the consistent direction for relative angle calculations.
    /// </summary>
    private int GetArucoIdForDiagramPiece(Transform piece)
    {
        if (piece == null) return -1;

        // First try name-based mapping for backward compatibility
        string n = piece.name != null ? piece.name.ToLower() : "";
        if (n.Contains("triangle_l"))
        {
            return n.EndsWith("_2") ? 1 : 0; // LargeTriangle IDs: 0,1
        }
        if (n.Contains("triangle_m"))
        {
            return 2; // MediumTriangle ID
        }
        if (n.Contains("triangle_s"))
        {
            return n.EndsWith("_2") ? 4 : 3; // SmallTriangle IDs: 3,4
        }
        if (n.Contains("square"))
        {
            return 5; // Square ID
        }
        if (n.Contains("parallel") || n.Contains("parallelogram"))
        {
            return n.EndsWith("_2") ? 7 : 6; // Parallelogram IDs: 6,7
        }

        // Fallback: Find the shape type of this piece and determine ID by order
        TangramShapeType shapeType = TangramShapeType.LargeTriangle;
        bool found = false;
        foreach (var kv in diagramPiecesByType)
        {
            if (kv.Value.Contains(piece))
            {
                shapeType = kv.Key;
                found = true;
                break;
            }
        }
        if (!found) return -1;

        // Map shape type to ArUco ID ranges and determine which ID based on piece order
        List<Transform> piecesOfType = diagramPiecesByType[shapeType];
        int indexInType = piecesOfType.IndexOf(piece);
        if (indexInType < 0) return -1;

        switch (shapeType)
        {
            case TangramShapeType.LargeTriangle:
                return indexInType == 0 ? 0 : 1; // IDs 0, 1
            case TangramShapeType.MediumTriangle:
                return 2; // ID 2
            case TangramShapeType.SmallTriangle:
                return indexInType == 0 ? 3 : 4; // IDs 3, 4
            case TangramShapeType.Square:
                return 5; // ID 5
            case TangramShapeType.Parallelogram:
                return indexInType == 0 ? 6 : 7; // IDs 6, 7
            default:
                return -1;
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
        // 1) Build detection graph with max-distance-normalized adjacency (no size-based normalization)
        var dg = BuildDetectionGraph();
        if (dg.nodes.Count == 0)
            return;

        // 2) For each detection, find diagram candidates by graph relation only
        // Strategy: seed with the detection that has the most neighbors; map it to the diagram piece of same type with highest graph degree (no absolute coords).
        // Then expand frontier: only match detections that have at least one already-matched neighbor, and only to diagram pieces that are neighbors of at least one matched diagram neighbor.

        // Special handling: when exactly 2 non-corner detections are present, ignore relation distance terms
        // and use only center-to-center angle and absolute orientation for matching.
        int nonCornerCount = 0;
        for (int i = 0; i < latestDetections.Count; i++) if (!latestDetections[i].isCorner) nonCornerCount++;
        bool angleOnlyForTwoDetections = (nonCornerCount == 2);

        // Order nodes by decreasing degree
        var order = new List<int>();
        for (int i = 0; i < dg.nodes.Count; i++) order.Add(i);
        order.Sort((a, b) => dg.nodes[b].neighbors.Count.CompareTo(dg.nodes[a].neighbors.Count));

        // Track which diagram pieces remain per type, skipping locked ones
        var remainingByType = new Dictionary<TangramShapeType, List<Transform>>();
        foreach (var kv in availablePieces)
        {
            var list = new List<Transform>();
            foreach (var p in kv.Value)
            {
                if (enableStatefulMatching && lockedByPiece.TryGetValue(p, out var ls) && ls.isLocked)
                {
                    // keep it colored and accounted, but exclude from remaining
                    matchedPieceSet.Add(p);
                    continue;
                }
                list.Add(p);
            }
            remainingByType[kv.Key] = list;
        }

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

            // Pre-assigned to locked piece?
            if (enableStatefulMatching)
            {
                int detIndex = node.detIndex;
                if (preAssignedDetections.TryGetValue(detIndex, out var lockedPiece))
                {
                    detToPiece[nodeIdx] = lockedPiece;
                    assignedDetectionIndices.Add(nodeIdx);
                    matchedPieceSet.Add(lockedPiece);
                    var det = latestDetections[detIndex];
                    // Use planar-projected absolute distance for logs/results
                    Vector3 planeN_abs = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 vAbs = Vector3.ProjectOnPlane(GetPieceCenterWorld(lockedPiece) - det.worldPosition, planeN_abs);
                    float absDist = vAbs.magnitude;
                    float absAng = ComputeAngleError(det.shapeType, det.worldRotation, lockedPiece);
                    lastMatchResults.Add(new MatchResult
                    {
                        shapeType = det.shapeType,
                        arucoId = det.arucoId,
                        matchedPieceTransform = lockedPiece,
                        worldDistanceMeters = absDist,
                        detectionWorldPosition = det.worldPosition,
                        detectionWorldRotation = det.worldRotation, // ★ 추가: 감지된 도형의 회전 정보 저장
                        angleErrorDegrees = absAng
                    });
                    continue;
                }
            }

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

            // Multi-detection assignment for duplicate types: choose configuration that maximizes number of matches, then minimal total cost, with stickiness bias
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
                                // If restricting relations to diagram edges, ensure the nearest matched neighbor forms an edge
                                if (restrictRelationsToGraphEdges)
                                {
                                    // Check existence of at least one matched neighbor that is an edge to p
                                    bool hasEdge = false;
                                    foreach (int nn in dg.nodes[di].neighbors)
                                    {
                                        if (!detToPiece.TryGetValue(nn, out var np)) continue;
                                        if (!diagramGraph.indexByTransform.TryGetValue(p, out int ip)) continue;
                                        if (!diagramGraph.indexByTransform.TryGetValue(np, out int inb)) continue;
                                        if (IsDiagramEdge(ip, inb)) { hasEdge = true; break; }
                                    }
                                    if (!hasEdge) continue;
                                }
                                if (!TryComputeRelationalCostForCandidate(di, p, dg, detToPiece, out float d, out float a, out float usedScale, out int cnt))
                                    continue;
                                // Unified REL-style gating: 2-detection → (angle OR ORI), otherwise → (dist AND (angle OR ORI))
                                float absOriDegGate = 0f;
                                bool passOriGate = true;
                                if (useAbsoluteOrientation)
                                {
                                    var detGate = latestDetections[node.detIndex];
                                    absOriDegGate = ComputeAngleError(node.type, detGate.worldRotation, p);
                                    passOriGate = (absOriDegGate <= absoluteOrientationToleranceDeg);
                                }
                                bool passDist = angleOnlyForTwoDetections ? true : (d <= _dynamicGraphDistanceTolerance);
                                bool passAng = (a <= graphAngleToleranceDeg);
                                bool passes = angleOnlyForTwoDetections ? (passAng && passOriGate) : (passDist && passAng && passOriGate);
                                if (passes)
                                {
                                    // Replace distance component with selected model
                                    float distanceCost2 = angleOnlyForTwoDetections ? 0f : d;
                                    if (useFirstPassRelativeScale)
                                    {
                                        if (!angleOnlyForTwoDetections)
                                        {
                                            // Convert to first-pass relative cost using nearest neighbor vectors
                                            int nearestIdx2 = -1; float nearestDet2 = float.PositiveInfinity; Transform nearestPiece2 = null;
                                            foreach (int nIdx in dg.nodes[di].neighbors)
                                            {
                                                if (!detToPiece.TryGetValue(nIdx, out var np)) continue;
                                                float dnn = Vector3.Distance(dg.nodes[nIdx].pos, dg.nodes[di].pos);
                                                if (dnn < nearestDet2) { nearestDet2 = dnn; nearestIdx2 = nIdx; nearestPiece2 = np; }
                                            }
                                            if (nearestIdx2 >= 0 && nearestPiece2 != null)
                                            {
                                                Vector3 planeN2 = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                                Vector3 vDetP2 = Vector3.ProjectOnPlane(dg.nodes[nearestIdx2].pos - dg.nodes[di].pos, planeN2);
                                                Vector3 vDiagP2 = Vector3.ProjectOnPlane(GetPieceCenterWorld(nearestPiece2) - GetPieceCenterWorld(p), planeN2);
                                                float dAct2 = vDetP2.magnitude;
                                                float dExp2 = vDiagP2.magnitude;
                                                float relErr2, distCostFP2;
                                                CheckDistanceWithFirstPassScale(dAct2, dExp2, out relErr2, out distCostFP2);
                                                distanceCost2 = distCostFP2;
                                            }
                                        }
                                    }
                                    else if (useNormalizedRelationDistance)
                                    {
                                        if (!angleOnlyForTwoDetections)
                                        {
                                            int nearestIdx2 = -1; float nearestDet2 = float.PositiveInfinity; Transform nearestPiece2 = null;
                                            foreach (int nIdx in dg.nodes[di].neighbors)
                                            {
                                                if (!detToPiece.TryGetValue(nIdx, out var np)) continue;
                                                float dnn = Vector3.Distance(dg.nodes[nIdx].pos, dg.nodes[di].pos);
                                                if (dnn < nearestDet2) { nearestDet2 = dnn; nearestIdx2 = nIdx; nearestPiece2 = np; }
                                            }
                                            float baseline;
                                            if (useSubsetUnifiedBaseline)
                                            {
                                                // Improvement B: avoid subset-derived baselines when cache is invalid; fall back to global maxima
                                                if (cachedBaselineValid)
                                                {
                                                    baseline = Mathf.Max(1e-4f, cachedBaselineDetRef, cachedBaselineExpRef);
                                                }
                                                else
                                                {
                                                    float baseDet = GetDetectionMaxPairDistance();
                                                    float baseExp = GetDiagramMaxEdgeLength();
                                                    baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                                                }
                                            }
                                            else
                                            {
                                                float baseDet = GetDetectionMaxPairDistance();
                                                float baseExp = GetDiagramMaxEdgeLength();
                                                baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                                            }
                                            if (baseline > 1e-4f)
                                            {
                                                Vector3 planeN2 = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                                Vector3 vDetP2 = Vector3.ProjectOnPlane(dg.nodes[nearestIdx2].pos - dg.nodes[di].pos, planeN2);
                                                Vector3 vDiagP2 = Vector3.ProjectOnPlane(GetPieceCenterWorld(nearestPiece2) - GetPieceCenterWorld(p), planeN2);
                                                float rDet = vDetP2.magnitude / baseline;
                                                float rDiag = vDiagP2.magnitude / baseline;
                                                distanceCost2 = Mathf.Abs(rDet - rDiag);
                                            }
                                        }
                                    }
                                    float cost = distanceCost2 + rotationWeightMetersPerDeg * a;
                                    if (useAbsoluteDistanceInCost)
                                    {
                                        var detx = latestDetections[node.detIndex];
                                        Vector3 planeN_abs = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                        Vector3 vAbs = Vector3.ProjectOnPlane(GetPieceCenterWorld(p) - detx.worldPosition, planeN_abs);
                                        float absDistX = vAbs.magnitude;
                                        cost += absoluteDistanceWeightMeters * absDistX;
                                    }
                                    // Add scale penalty only when not using first-pass global scale
                                    if (!useFirstPassRelativeScale)
                                    {
                                        float scalePenalty = Mathf.Abs(usedScale - 1f) * 0.1f; // weight = 0.1 meters per unit scale deviation
                                        cost += scalePenalty;
                                    }

                                    // Optional absolute orientation cost (do not hard-reject here; gating handled above)
                                    bool oriPassLocal = !useAbsoluteOrientation;
                                    if (useAbsoluteOrientation)
                                    {
                                        var det = latestDetections[node.detIndex];
                                        // Use unified orientation error function to respect angleOffset and symmetry consistently
                                        float absOriDeg = ComputeAngleError(node.type, det.worldRotation, p);
                                        if (absOriDeg <= absoluteOrientationToleranceDeg) oriPassLocal = true;
                                        else if (debugLogOrientationRejections)
                                        {
                                            Debug.Log($"[TangramMatcher] ORI REJECT det({node.type}:{latestDetections[node.detIndex].arucoId}) -> diag({p.name}) | absOri={absOriDeg:F1}° > tol={absoluteOrientationToleranceDeg:F1}°");
                                        }
                                        if (debugLogOrientationDiff)
                                        {
                                            Debug.Log($"[TangramMatcher] ORI diff det({node.type}:{latestDetections[node.detIndex].arucoId}) -> diag({p.name}) | absOri={absOriDeg:F1}°");
                                        }
                                        cost += absoluteOrientationWeightMetersPerDeg * absOriDeg;
                                    }
                                    // If relation angle AND ORI passes, prefer nearest diagram piece by absolute planar distance
                                    bool anglePassLocal = (a <= graphAngleToleranceDeg);
                                    if (anglePassLocal && oriPassLocal)
                                    {
                                        var detx2 = latestDetections[node.detIndex];
                                        Vector3 planeN_abs2 = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                        Vector3 vAbs2 = Vector3.ProjectOnPlane(GetPieceCenterWorld(p) - detx2.worldPosition, planeN_abs2);
                                        float absDistPref = vAbs2.magnitude;
                                        cost = absDistPref;
                                    }
                                    if (preferPreviousAssignment)
                                    {
                                        var key = (dg.nodes[di].type, latestDetections[node.detIndex].arucoId);
                                        if (lastAssignedPieceByKey.TryGetValue(key, out var prev) && prev == p)
                                        {
                                            cost = Mathf.Max(0f, cost - previousAssignmentBiasMeters);
                                        }
                                        else if (lastAssignedPieceByKey.TryGetValue(key, out var prevPiece) && prevPiece != null)
                                        {
                                            if (preserveLockUnlessExtreme && lockedByPiece.TryGetValue(prevPiece, out var lsPrev))
                                            {
                                                float extremeDist = Vector3.Distance(latestDetections[node.detIndex].worldPosition, lsPrev.lastPosition);
                                                if (extremeDist <= extremeReassignDistanceMeters)
                                                {
                                                    cost += cooldownPenaltyMeters;
                                                }
                                            }
                                        }
                                    }
                                    pairs.Add((di, p, cost, d, a));
                                }
                            }
                        }
                        // Assignment: Hungarian (optional) else greedy
                        var usedDet = new HashSet<int>();
                        var usedPiece = new HashSet<Transform>();
                        if (useHungarianForTypeMatching)
                        {
                            // map detIndices and pool to compact indices
                            var detMap = new Dictionary<int, int>();
                            for (int k = 0; k < detIndices.Count; k++) detMap[detIndices[k]] = k;
                            var pieceList = new List<Transform>(pool);
                            var pieceMap = new Dictionary<Transform, int>();
                            for (int k = 0; k < pieceList.Count; k++) pieceMap[pieceList[k]] = k;
                            // build cost matrix with large cost for invalid pairs
                            int R = detIndices.Count, C = pieceList.Count;
                            float[,] costMat = new float[R, C];
                            float BIG = 1e6f;
                            for (int r = 0; r < R; r++) for (int c = 0; c < C; c++) costMat[r, c] = BIG;
                            foreach (var pr in pairs)
                            {
                                int r = detMap[pr.di]; int c = pieceMap[pr.piece];
                                costMat[r, c] = pr.cost;
                            }
                            int[] assign = HungarianSolve(costMat);
                            for (int r = 0; r < assign.Length; r++)
                            {
                                int c = assign[r];
                                if (c < 0 || c >= C) continue;
                                if (costMat[r, c] >= BIG * 0.5f) continue;
                                usedDet.Add(detIndices[r]);
                                usedPiece.Add(pieceList[c]);
                            }
                        }
                        else
                        {
                            pairs.Sort((x, y) => x.cost.CompareTo(y.cost));
                            foreach (var pr in pairs)
                            {
                                if (usedDet.Contains(pr.di) || usedPiece.Contains(pr.piece)) continue;
                                usedDet.Add(pr.di); usedPiece.Add(pr.piece);
                            }
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
                                    detectionWorldRotation = det.worldRotation, // ★ 추가: 감지된 도형의 회전 정보 저장
                                    angleErrorDegrees = pr.a
                                });
                    linkSegments.Add((det.worldPosition, GetPieceCenterWorld(piece), pr.d, piece, det.shapeType, det.arucoId));
                                lastAssignedPieceByKey[(det.shapeType, det.arucoId)] = piece;
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
                // Relational cost using the BEST (lowest-cost) relation among matched neighbors that are connected in the diagram graph
                var nearMissList = new List<(Transform p, float avgDistDiff, float avgAngDiff, float cost)>();

                if (matchOnFirstPassingRelation)
                {
                    foreach (var p in filteredCandidates)
                    {
                        if (TryComputeRelationalCostForCandidate(nodeIdx, p, dg, detToPiece, out float d, out float a, out float usedScale, out int cnt))
                        {
                            bool passDist = d <= _dynamicGraphDistanceTolerance;
                            bool passAng = a <= graphAngleToleranceDeg;
                            bool passOri = !useAbsoluteOrientation || ComputeAngleError(node.type, latestDetections[node.detIndex].worldRotation, p) <= absoluteOrientationToleranceDeg;
                            bool pass = angleOnlyForTwoDetections ? (passAng && passOri) : (passDist && passAng && passOri);

                            if (pass)
                            {
                                chosen = p;
                                chosenCost = 0f;
                                chosenAvgDistDiff = d;
                                chosenAvgAngDiff = a;
                                goto CandidateChosen;
                            }
                            else
                            {
                                nearMissList.Add((p, d, a, d + a * rotationWeightMetersPerDeg));
                            }
                        }
                    }
                CandidateChosen:;
                }
                else
                {
                    // Legacy best-fit logic can be implemented here if needed.
                }

                // If no candidate was chosen, record the reason for the best near-miss.
                if (chosen == null && nearMissList.Count > 0)
                {
                    nearMissList.Sort((a, b) => a.cost.CompareTo(b.cost));
                    var bestFail = nearMissList[0];
                    string failureReason = "";

                    if (bestFail.avgDistDiff > _dynamicGraphDistanceTolerance)
                    {
                        failureReason = $"vs '{bestFail.p.name}': Relational distance error ({bestFail.avgDistDiff:F3}m) > tolerance ({_dynamicGraphDistanceTolerance:F3}m).";
                    }
                    else if (bestFail.avgAngDiff > graphAngleToleranceDeg)
                    {
                        failureReason = $"vs '{bestFail.p.name}': Relational angle error ({bestFail.avgAngDiff:F1}°) > tolerance ({graphAngleToleranceDeg:F1}°).";
                    }
                    else if (useAbsoluteOrientation)
                    {
                        var det = latestDetections[node.detIndex];
                        float absOriDeg = ComputeAngleError(node.type, det.worldRotation, bestFail.p);
                        if (absOriDeg > absoluteOrientationToleranceDeg)
                        {
                            failureReason = $"vs '{bestFail.p.name}': Absolute orientation error ({absOriDeg:F1}°) > tolerance ({absoluteOrientationToleranceDeg:F1}°).";
                        }
                    }

                    if (!string.IsNullOrEmpty(failureReason))
                    {
                        if (preAssignedDetections.TryGetValue(node.detIndex, out var lockedPieceForThisDet) && lockedPieceForThisDet != null)
                        {
                            _detailedFailureReasons[lockedPieceForThisDet.name] = failureReason;
                        }
                    }
                }
                if (debugLogUnmatchedCandidates && nearMissList.Count > 0)
                {
                    nearMissList.Sort((a, b) => a.cost.CompareTo(b.cost));
                    int take = Mathf.Min(debugMaxUnmatchedCandidatesPerDetection, nearMissList.Count);
                    for (int ii = 0; ii < take; ii++)
                    {
                        var nm = nearMissList[ii];
                        Debug.Log($"[TangramMatcher] near-miss for det({node.type}) -> diag({nm.p.name}) avg diff(d)={nm.avgDistDiff:F3}m avg diff(deg)={nm.avgAngDiff:F1} deg | tol(d)<={_dynamicGraphDistanceTolerance:F3} tol(deg)<={graphAngleToleranceDeg:F1}");
                    }
                }

                // Record detailed failure reason if no choice was made but there were near misses
                if (chosen == null && nearMissList.Count > 0)
                {
                    // Most promising failed candidate
                    var bestFail = nearMissList[0];
                    string failureReason = string.Empty;
                    if (bestFail.avgDistDiff > _dynamicGraphDistanceTolerance)
                    {
                        failureReason = $"Relational check failed. Distance error ({bestFail.avgDistDiff:F3}m) exceeded tolerance ({_dynamicGraphDistanceTolerance:F3}m).";
                    }
                    else if (bestFail.avgAngDiff > graphAngleToleranceDeg)
                    {
                        failureReason = $"Relational check failed. Angle error ({bestFail.avgAngDiff:F1}°) exceeded tolerance ({graphAngleToleranceDeg:F1}°).";
                    }
                    else
                    {
                        var detLocal = latestDetections[node.detIndex];
                        float absOriDeg = ComputeAngleError(node.type, detLocal.worldRotation, bestFail.p);
                        if (useAbsoluteOrientation && absOriDeg > absoluteOrientationToleranceDeg)
                        {
                            failureReason = $"Relational check failed. Orientation error ({absOriDeg:F1}°) exceeded tolerance ({absoluteOrientationToleranceDeg:F1}°).";
                        }
                        else
                        {
                            failureReason = $"Relational check failed for an unknown reason (cost: {bestFail.cost:F3}).";
                        }
                    }
                    if (preAssignedDetections.TryGetValue(node.detIndex, out var lockedPieceForThisDet) && lockedPieceForThisDet != null)
                    {
                        _detailedFailureReasons[lockedPieceForThisDet.name] = failureReason;
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
                // Report planar-projected distance/angle to reduce confusion in logs
                Vector3 planeN_abs2 = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                Vector3 vAbs2 = Vector3.ProjectOnPlane(GetPieceCenterWorld(chosen) - det.worldPosition, planeN_abs2);
                float absDist = vAbs2.magnitude;
                float absAng = ComputeAngleError(det.shapeType, det.worldRotation, chosen);
                lastMatchResults.Add(new MatchResult
                {
                    shapeType = det.shapeType,
                    arucoId = det.arucoId,
                    matchedPieceTransform = chosen,
                    worldDistanceMeters = absDist,
                    detectionWorldPosition = det.worldPosition,
                    detectionWorldRotation = det.worldRotation, // ★ 추가: 감지된 도형의 회전 정보 저장
                    angleErrorDegrees = absAng
                });
                linkSegments.Add((det.worldPosition, chosen.position, chosenAvgDistDiff, chosen, det.shapeType, det.arucoId));
                lastAssignedPieceByKey[(det.shapeType, det.arucoId)] = chosen;

            // Detailed relational debug per matched neighbor (only diagram edges if restricted)
                foreach (int nIdx in node.neighbors)
                {
                    if (!detToPiece.TryGetValue(nIdx, out var neighborPiece))
                        continue;
                if (restrictRelationsToGraphEdges)
                {
                    if (!diagramGraph.indexByTransform.TryGetValue(chosen, out int di)) continue;
                    if (!diagramGraph.indexByTransform.TryGetValue(neighborPiece, out int dj)) continue;
                    if (!IsDiagramEdge(di, dj)) continue;
                }
                    // ★ 변경점: 새로운 일관된 상대 각도 계산 함수를 사용하여 관계 검증의 정확성을 향상시킵니다.
                    // Use the new, consistent relative angle calculation functions for relation validation accuracy
                    
                    // For detection positions, we need to find the detection's rotation
                    Quaternion detRot = Quaternion.identity;
                    Quaternion neighDetRot = Quaternion.identity;
                    foreach (var r in lastMatchResults)
                    {
                        if (r.matchedPieceTransform == chosen)
                        {
                            detRot = r.detectionWorldRotation;
                        }
                        if (r.matchedPieceTransform == neighborPiece)
                        {
                            neighDetRot = r.detectionWorldRotation;
                        }
                    }
                    
                    // Calculate expected angle from diagram using the new function
                    float expectedAngle = ComputeRelativeAngleDegForDiagram(chosen, neighborPiece, 
                        $"RelationCheck_{chosen.name}_to_{neighborPiece.name}");
                    
                    // Calculate actual angle from detection using the new function
                    float actualAngle = ComputeRelativeAngleDeg(node.pos, detRot, dg.nodes[nIdx].pos, 
                        $"RelationCheck_{chosen.name}_detection");
                    
                    // Calculate angle difference using the new, consistent method
                    float angDiff = Mathf.Abs(Mathf.DeltaAngle(expectedAngle, actualAngle));
                    
                    // Debug logging for the new angle calculation method in relation checking
                    if (debugLogRelativeAngles)
                    {
                        Debug.Log($"[RelationCheck] {chosen.name} vs {neighborPiece.name}: " +
                                  $"Expected={expectedAngle:F2}°, Actual={actualAngle:F2}°, " +
                                  $"Difference={angDiff:F2}°, Tolerance={relationMaxAngleDiffDeg:F2}°");
                    }
                    
                    // For distance calculation, we still need the old vector approach
                    Vector3 vDet = node.pos - dg.nodes[nIdx].pos;
                    Vector3 vDiag = GetPieceCenterWorld(chosen) - GetPieceCenterWorld(neighborPiece);
                    // Project to diagram plane for stable angle
                    Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 vDetP = Vector3.ProjectOnPlane(vDet, planeN);
                    Vector3 vDiagP = Vector3.ProjectOnPlane(vDiag, planeN);
                    float distDiff = Mathf.Abs(vDetP.magnitude - vDiagP.magnitude);
                if (distDiff <= relationMaxDistDiff && angDiff <= relationMaxAngleDiffDeg)
                {
                    Debug.Log($"[TangramMatcher] MATCH relation det({node.type})-det({dg.nodes[nIdx].type}) vs diag({chosen.name})-diag({neighborPiece.name}) diff(d)={distDiff:F3}m diff(deg)={angDiff:F1} deg");
                }
                else if (verboseRelationDebug)
                {
                    Debug.Log($"[TangramMatcher] CAND relation (REJECT) det({node.type})-det({dg.nodes[nIdx].type}) vs diag({chosen.name})-diag({neighborPiece.name}) diff(d)={distDiff:F3}m diff(deg)={angDiff:F1} deg");
                }
                }

                if (!seeded) seeded = true;
            }
        }

        if (enableStatefulMatching)
        {
            // Update locked confidences
            var seenPieces = new HashSet<Transform>();
            foreach (var r in lastMatchResults) seenPieces.Add(r.matchedPieceTransform);
            // Decay and update (optionally sticky)
            foreach (var kv in lockedByPiece)
            {
                var ls = kv.Value;
                if (seenPieces.Contains(ls.piece))
                {
                    bool relationPass = true;
                    if (requireRelationPassForLock)
                    {
                        // Check relations to matched neighbors along diagram edges
                        relationPass = EvaluateRelationPassForPiece(ls.piece, relationLockRequireAllNeighbors, out string reason);
                        if (debugLogLockRelationDecision)
                        {
                            Debug.Log("[TangramMatcher] LOCK-REL " + ls.piece.name + " pass=" + (relationPass ? 1 : 0) + " all=" + (relationLockRequireAllNeighbors ? 1 : 0) + " " + reason);}
                    }
                    if (relationPass)
                    {
                        if (immediateLockOnRelationPass)
                        {
                            ls.confidence = 1.0f;
                            ls.isLocked = true;
                        }
                        else
                        {
                            ls.confidence = Mathf.Clamp01(ls.confidence + confidenceGainPerHit);
                        }
                    }
                    var last = ls.lastPosition;
                    var current = GetPieceCenterWorld(ls.piece);
                    ls.lastVelocity = (current - last) / Mathf.Max(Time.deltaTime, 1e-3f);
                    ls.lastPosition = current;
                    ls.lastSeenFrame = frameCounter;
                    if (ls.confidence >= lockConfidenceThreshold) ls.isLocked = true;
                }
                else
                {
                    if (debugLockFailures)
                    {
                        // Not matched this frame while locked: distinguish causes
                        if (_gatingFailuresThisFrame.Contains(ls.piece.name))
                        {
                            // Already logged as gating failure; avoid duplicate
                        }
                        else
                        {
                            // Check if any detection of this type existed
                            bool detectionExists = false;
                            for (int i = 0; i < latestDetections.Count; i++)
                            {
                                if (latestDetections[i].isCorner) continue;
                                if (latestDetections[i].shapeType == ls.type) { detectionExists = true; break; }
                            }
                            if (!detectionExists)
                            {
                                Debug.LogWarning($"[TangramMatcher] MATCH FAILED for locked piece '{ls.piece.name}'. Cause: No detection of this type was found in the current frame.");
                            }
                            else
                            {
                                if (suppressFlickerWarnings && _matchedPiecesLastFrame.Contains(ls.piece))
                                {
                                    // 직전 프레임에 매칭됐던 조각이면 경고를 건너뜁니다 (Flicker 억제).
                                }
                                else
                                {
                                    string reason = "A detection existed but failed relational checks (incorrect position relative to neighbors).";
                                    if (!_detailedFailureReasons.TryGetValue(ls.piece.name, out var detailedReason))
                                    {
                                        // Fallback: derive a detailed reason on-the-fly using nearest detection and matched neighbors
                                        // 1) Find nearest detection of the same type (planar distance)
                                        int detIdx = -1; float bestAbs = float.PositiveInfinity; Vector3 nearestDetPos = Vector3.zero; Quaternion nearestDetRot = Quaternion.identity;
                                        Vector3 centerLs = GetPieceCenterWorld(ls.piece);
                                        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                        for (int i = 0; i < latestDetections.Count; i++)
                                        {
                                            var d = latestDetections[i];
                                            if (d.isCorner || d.shapeType != ls.type) continue;
                                            Vector3 vAbs = Vector3.ProjectOnPlane(d.worldPosition - centerLs, planeN);
                                            float m = vAbs.magnitude;
                                            if (m < bestAbs) { bestAbs = m; detIdx = i; nearestDetPos = d.worldPosition; nearestDetRot = d.worldRotation; }
                                        }
                                        if (detIdx >= 0)
                                        {
                                            // 2) Compare relations against matched diagram neighbors
                                            float bestLocalCost = float.PositiveInfinity;
                                            float bestLocalDist = 0f;
                                            float bestLocalAng = 0f;
                                            Transform bestFailNeighbor = null;

                                            if (diagramGraph != null && diagramGraph.indexByTransform.TryGetValue(ls.piece, out int ipiece))
                                            {
                                                foreach (var e in diagramGraph.nodes[ipiece].edges)
                                                {
                                                    var neighPiece = diagramGraph.nodes[e.toIndex].piece;
                                                    // Find detection matched to neighPiece this frame
                                                    Vector3 neighDetPos = Vector3.zero;
                                                    bool neighMatched = false;
                                                    foreach (var r in lastMatchResults)
                                                    {
                                                        if (r.matchedPieceTransform == neighPiece)
                                                        {
                                                            neighDetPos = r.detectionWorldPosition;
                                                            neighMatched = true; break;
                                                        }
                                                    }
                                                    if (!neighMatched) continue;

                                                    // ★ 변경점: 새로운 일관된 상대 각도 계산 함수를 사용하여 오류 보고의 정확성을 향상시킵니다.
                                                    // Use the new, consistent relative angle calculation functions for error reporting accuracy
                                                    
                                                    // For detection positions, we need to find the detection's rotation
                                                    Quaternion neighDetRot = Quaternion.identity;
                                                    foreach (var r in lastMatchResults)
                                                    {
                                                        if (r.matchedPieceTransform == neighPiece)
                                                        {
                                                            neighDetRot = r.detectionWorldRotation;
                                                            break;
                                                        }
                                                    }
                                                    
                                                    // Calculate expected angle from diagram using the new function
                                                    float expectedAngle = ComputeRelativeAngleDegForDiagram(ls.piece, neighPiece, 
                                                        $"ErrorReport_{ls.piece.name}_to_{neighPiece.name}");
                                                    
                                                    // Calculate actual angle from detection using the new function
                                                    float actualAngle = ComputeRelativeAngleDeg(nearestDetPos, nearestDetRot, neighDetPos, 
                                                        $"ErrorReport_{ls.piece.name}_detection");
                                                    
                                                    // Calculate angle difference using the new, consistent method
                                                    float angDiff = Mathf.Abs(Mathf.DeltaAngle(expectedAngle, actualAngle));
                                                    
                                                    // Debug logging for the new angle calculation method
                                                    if (debugLogRelativeAngles)
                                                    {
                                                        Debug.Log($"[ErrorReport] {ls.piece.name} vs {neighPiece.name}: " +
                                                                  $"Expected={expectedAngle:F2}°, Actual={actualAngle:F2}°, " +
                                                                  $"Difference={angDiff:F2}°, Tolerance={graphAngleToleranceDeg:F2}°");
                                                    }
                                                    
                                                    // For distance calculation, we still need the old vector approach
                                                    Vector3 vDet = neighDetPos - nearestDetPos;
                                                    Vector3 vDiag = GetPieceCenterWorld(neighPiece) - centerLs;
                                                    Vector3 vDetP = Vector3.ProjectOnPlane(vDet, planeN);
                                                    Vector3 vDiagP = Vector3.ProjectOnPlane(vDiag, planeN);
                                                    float scaleFactor = 1f;
                                                    if (useScaleNormalization && vDiagP.magnitude > 1e-4f)
                                                        scaleFactor = vDetP.magnitude / vDiagP.magnitude;
                                                    float distDiff;
                                                    if (useNormalizedRelationDistance)
                                                    {
                                                        float baseline = Vector3.Distance(GetPieceCenterWorld(neighPiece), centerLs);
                                                        if (baseline > 1e-4f)
                                                        {
                                                            float ratioDet = vDetP.magnitude / baseline;
                                                            float ratioDiag = (vDiagP.magnitude * scaleFactor) / baseline;
                                                            distDiff = Mathf.Abs(ratioDet - ratioDiag);
                                                        }
                                                        else distDiff = 0f;
                                                    }
                                                    else
                                                    {
                                                        distDiff = Mathf.Abs(vDetP.magnitude - vDiagP.magnitude * scaleFactor);
                                                    }
                                                    float cost = distDiff + rotationWeightMetersPerDeg * angDiff;
                                                    if (cost < bestLocalCost)
                                                    {
                                                        bestLocalCost = cost; bestLocalDist = distDiff; bestLocalAng = angDiff;
                                                        bestFailNeighbor = neighPiece;
                                                    }
                                                }
                                            }
                                            // Enhanced debugging: Show all condition states instead of just the first failure
                                            if (!float.IsPositiveInfinity(bestLocalCost))
                                            {
                                                string neighborName = bestFailNeighbor != null ? bestFailNeighbor.name : "unknown";
                                                
                                                // Check all condition states for comprehensive debugging
                                                bool passDistance = bestLocalDist <= _dynamicGraphDistanceTolerance;
                                                bool passAngle = bestLocalAng <= graphAngleToleranceDeg;
                                                
                                                // Calculate absolute orientation error for debugging
                                                float absOriDeg = useAbsoluteOrientation ? ComputeAngleError(ls.type, nearestDetRot, ls.piece) : 0f;
                                                bool passOrientation = !useAbsoluteOrientation || absOriDeg <= absoluteOrientationToleranceDeg;
                                                
                                                // Calculate absolute distance for debugging
                                                Vector3 vAbsP = Vector3.ProjectOnPlane(nearestDetPos - centerLs, planeN);
                                                float absDist = vAbsP.magnitude;
                                                bool passAbsoluteDistance = absDist <= relockMaxDistanceMeters;
                                                
                                                // Create comprehensive failure reason with all condition states
                                                detailedReason = $"vs '{neighborName}': " +
                                                               $"RelDist={bestLocalDist:F3}m({(passDistance?"PASS":"FAIL")}) " +
                                                               $"RelAngle={bestLocalAng:F1}°({(passAngle?"PASS":"FAIL")}) " +
                                                               $"AbsOri={absOriDeg:F1}°({(passOrientation?"PASS":"FAIL")}) " +
                                                               $"AbsDist={absDist:F3}m({(passAbsoluteDistance?"PASS":"FAIL")}) " +
                                                               $"Tolerances: d≤{_dynamicGraphDistanceTolerance:F3}m, a≤{graphAngleToleranceDeg:F1}°, " +
                                                               $"o≤{absoluteOrientationToleranceDeg:F1}°, ad≤{relockMaxDistanceMeters:F3}m";
                                            }
                                            else
                                            {
                                                // Fallback case when no relational data is available
                                                Vector3 vAbsP = Vector3.ProjectOnPlane(nearestDetPos - centerLs, planeN);
                                                float absDist = vAbsP.magnitude;
                                                bool passAbsoluteDistance = absDist <= relockMaxDistanceMeters;
                                                
                                                detailedReason = $"No relational data available. " +
                                                               $"AbsDist={absDist:F3}m({(passAbsoluteDistance?"PASS":"FAIL")}) " +
                                                               $"Tolerance: ≤{relockMaxDistanceMeters:F3}m";
                                            }
                                        }
                                        if (!string.IsNullOrEmpty(detailedReason))
                                        {
                                            _detailedFailureReasons[ls.piece.name] = detailedReason;
                                            reason = detailedReason;
                                        }
                                    }
                                    else
                                    {
                                        reason = detailedReason;
                                    }
                                    Debug.LogWarning($"[TangramMatcher] MATCH FAILED for locked piece '{ls.piece.name}'. Cause: {reason}");
                                }
                            }
                        }
                    }
                    if (!stickyLocks)
                    {
                        ls.confidence = Mathf.Clamp01(ls.confidence - confidenceDecayPerFrame);
                        if (ls.confidence <= unlockConfidenceThreshold) ls.isLocked = false;
                    }
                }
            }
            // Initialize states for newly matched pieces
            foreach (var r in lastMatchResults)
            {
                if (!lockedByPiece.TryGetValue(r.matchedPieceTransform, out var ls))
                {
                    ls = new LockedState
                    {
                        piece = r.matchedPieceTransform,
                        type = r.shapeType,
                        confidence = confidenceGainPerHit,
                        lastPosition = r.matchedPieceTransform.position,
                        lastVelocity = Vector3.zero,
                        lastSeenFrame = frameCounter,
                        isLocked = confidenceGainPerHit >= lockConfidenceThreshold
                    };
                    // Apply relation-based immediate locking policy for new matches
                    if (requireRelationPassForLock)
                    {
                        bool pass = EvaluateRelationPassForPiece(ls.piece, relationLockRequireAllNeighbors, out string reasonNew);
                        if (debugLogLockRelationDecision)
                        {
                            Debug.Log($"[TangramMatcher] LOCK-REL(new) {ls.piece.name} pass={(pass?1:0)} all={(relationLockRequireAllNeighbors?1:0)} {reasonNew}");
                        }
                        if (pass)
                        {
                            if (immediateLockOnRelationPass)
                            {
                                ls.confidence = 1.0f;
                                ls.isLocked = true;
                            }
                            else
                            {
                                ls.confidence = Mathf.Clamp01(ls.confidence + confidenceGainPerHit);
                                if (ls.confidence >= lockConfidenceThreshold) ls.isLocked = true;
                            }
                        }
                    }
                    lockedByPiece[r.matchedPieceTransform] = ls;
                }
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
            if (d.isCorner) continue; // exclude plane corner markers from detection graph
            dg.nodes.Add(new DetectionGraph.DNode
            {
                detIndex = i,
                type = d.shapeType,
                pos = d.projectedWorldPosition.sqrMagnitude > 0f ? d.projectedWorldPosition : d.worldPosition,
                rot = d.worldRotation
            });
        }
        // Build edges (normalized by max detection pairwise distance; no size-based normalization)
        for (int i = 0; i < dg.nodes.Count; i++)
        {
            var ni = dg.nodes[i];
            // Precompute max detection pairwise distance once per node loop
            float maxPair = GetDetectionMaxPairDistance();
            for (int j = 0; j < dg.nodes.Count; j++)
            {
                if (i == j) continue;
                var nj = dg.nodes[j];
                float dist = Vector3.Distance(ni.pos, nj.pos);
                float norm = maxPair > 1e-4f ? dist / maxPair : float.PositiveInfinity;
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

    // ★ REPLACE ★: Proper Hungarian algorithm (O(n^3)), rectangular cost allowed.
    private static int[] HungarianSolve(float[,] cost)
    {
        int nRows = cost.GetLength(0);
        int nCols = cost.GetLength(1);
        int n = Math.Max(nRows, nCols);

        // Build square matrix
        float[,] a = new float[n, n];
        for (int r = 0; r < n; r++)
            for (int c = 0; c < n; c++)
                a[r, c] = (r < nRows && c < nCols) ? cost[r, c] : 1e6f;

        // Row reduction
        for (int r = 0; r < n; r++)
        {
            float min = float.PositiveInfinity;
            for (int c = 0; c < n; c++) if (a[r, c] < min) min = a[r, c];
            if (!float.IsInfinity(min))
                for (int c = 0; c < n; c++) a[r, c] -= min;
        }
        // Column reduction
        for (int c = 0; c < n; c++)
        {
            float min = float.PositiveInfinity;
            for (int r = 0; r < n; r++) if (a[r, c] < min) min = a[r, c];
            if (!float.IsInfinity(min))
                for (int r = 0; r < n; r++) a[r, c] -= min;
        }

        // Marks
        int[] rowCover = new int[n];
        int[] colCover = new int[n];
        int[,] star = new int[n, n];
        int[,] prime = new int[n, n];

        // Star zeros greedily
        for (int r = 0; r < n; r++)
        {
            for (int c = 0; c < n; c++)
            {
                if (Mathf.Approximately(a[r, c], 0f) && rowCover[r] == 0 && colCover[c] == 0)
                {
                    star[r, c] = 1;
                    rowCover[r] = 1;
                    colCover[c] = 1;
                }
            }
        }
        Array.Clear(rowCover, 0, n);
        Array.Clear(colCover, 0, n);

        // Cover columns of starred zeros
        for (int c = 0; c < n; c++)
            for (int r = 0; r < n; r++)
                if (star[r, c] == 1) { colCover[c] = 1; break; }

        while (true)
        {
            int coveredCols = 0;
            for (int c = 0; c < n; c++) if (colCover[c] == 1) coveredCols++;
            if (coveredCols >= n) break; // optimal

            // Step: find a noncovered zero, prime it. If no star in its row, augment.
            bool found = true;
            int zRow = -1, zCol = -1;
            while (found)
            {
                found = false;
                for (int r = 0; r < n && !found; r++)
                {
                    if (rowCover[r] == 1) continue;
                    for (int c = 0; c < n; c++)
                    {
                        if (colCover[c] == 0 && Mathf.Approximately(a[r, c], 0f) && prime[r, c] == 0)
                        {
                            prime[r, c] = 1;
                            int starCol = -1;
                            for (int cc = 0; cc < n; cc++) if (star[r, cc] == 1) { starCol = cc; break; }
                            if (starCol >= 0)
                            {
                                rowCover[r] = 1;
                                colCover[starCol] = 0;
                            }
                            else
                            {
                                zRow = r; zCol = c;
                                found = false; // break both
                                goto AUGMENT;
                            }
                            found = true;
                            break;
                        }
                    }
                }

                // No uncovered zero found → adjust matrix
                float min = float.PositiveInfinity;
                for (int r = 0; r < n; r++)
                    if (rowCover[r] == 0)
                        for (int c = 0; c < n; c++)
                            if (colCover[c] == 0 && a[r, c] < min) min = a[r, c];

                for (int r = 0; r < n; r++)
                {
                    for (int c = 0; c < n; c++)
                    {
                        if (rowCover[r] == 1) a[r, c] += min;
                        if (colCover[c] == 0) a[r, c] -= min;
                    }
                }
            }

AUGMENT:
            // Build alternating path from (zRow,zCol)
            var path = new List<Vector2Int>();
            path.Add(new Vector2Int(zRow, zCol));
            while (true)
            {
                int r = path[path.Count - 1].x;
                int c = path[path.Count - 1].y;

                int starRow = -1;
                for (int rr = 0; rr < n; rr++) if (star[rr, c] == 1) { starRow = rr; break; }
                if (starRow < 0) break;
                path.Add(new Vector2Int(starRow, c));

                int primeCol = -1;
                for (int cc = 0; cc < n; cc++) if (prime[starRow, cc] == 1) { primeCol = cc; break; }
                path.Add(new Vector2Int(starRow, primeCol));
            }

            // Star ↔ prime toggle along the path
            foreach (var pos in path)
            {
                int r = pos.x, c = pos.y;
                if (star[r, c] == 1) star[r, c] = 0;
                else if (prime[r, c] == 1) { star[r, c] = 1; prime[r, c] = 0; }
            }
            // Clear covers and primes
            Array.Clear(rowCover, 0, n);
            Array.Clear(colCover, 0, n);
            Array.Clear(prime, 0, prime.Length);

            // Cover columns with stars again
            for (int c = 0; c < n; c++)
                for (int r = 0; r < n; r++)
                    if (star[r, c] == 1) { colCover[c] = 1; break; }
        }

        // Build assignment for original rect
        int[] assign = new int[nRows];
        for (int r = 0; r < nRows; r++)
        {
            int cAssigned = -1;
            for (int c = 0; c < nCols; c++)
                if (star[r, c] == 1) { cAssigned = c; break; }
            assign[r] = cAssigned;
        }
        return assign;
    }

    private bool TryComputeRelationalCostForCandidate(int nodeIdx, Transform candidatePiece, DetectionGraph dg, Dictionary<int, Transform> detToPiece,
        out float avgDistDiff, out float avgAngDiff, out float usedScale, out int usedCount)
    {
        // Compute the BEST (lowest cost) relation among all matched neighbors that are connected in the diagram graph
        avgDistDiff = 0f; avgAngDiff = 0f; usedScale = 1f; usedCount = 0;
        var center = dg.nodes[nodeIdx];
        float bestCost = float.PositiveInfinity;
        float bestDist = 0f, bestAng = 0f, bestScale = 1f;

        for (int nn = 0; nn < center.neighbors.Count; nn++)
        {
            int nIdx = center.neighbors[nn];
            if (!detToPiece.TryGetValue(nIdx, out var neighborPiece)) continue;

            if (restrictRelationsToGraphEdges)
            {
                if (!diagramGraph.indexByTransform.TryGetValue(candidatePiece, out int ip)) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(neighborPiece, out int inb)) continue;
                if (!IsDiagramEdge(ip, inb)) continue;
            }

            Vector3 vDet = dg.nodes[nIdx].pos - center.pos;
            Vector3 vDiag = GetPieceCenterWorld(neighborPiece) - GetPieceCenterWorld(candidatePiece);
            if (usePlanarProjectionForRelations)
            {
                Vector3 nrm = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
                vDet = Vector3.ProjectOnPlane(vDet, nrm);
                vDiag = Vector3.ProjectOnPlane(vDiag, nrm);
            }

            float localScale = 1f;
            if (useScaleNormalization && vDiag.magnitude > 1e-4f)
                localScale = vDet.magnitude / vDiag.magnitude;

            // Distance difference per selected model
            float distDiff;
            if (useFirstPassRelativeScale)
            {
                // Use planar magnitudes directly; global first-pass scale handles expected scaling
                Vector3 vDetP = vDet;
                Vector3 vDiagP = vDiag;
                float dAct = vDetP.magnitude;
                float dExp = vDiagP.magnitude;
                float relErrFP, distCostFP;
                CheckDistanceWithFirstPassScale(dAct, dExp, out relErrFP, out distCostFP);
                distDiff = distCostFP; // cost beyond tolerance only
                // In this mode, do not penalize pair-local scale deviation separately
                localScale = 1f;
            }
            else if (useNormalizedRelationDistance)
            {
                float ratioDet, ratioDiag;
                if (useSubsetUnifiedBaseline)
                {
                    // Improvement B: if cached baseline is invalid this frame, avoid subset-derived refs (unstable) and use global maxima
                    if (cachedBaselineValid)
                    {
                        ratioDet = vDet.magnitude / Mathf.Max(1e-4f, cachedBaselineDetRef);
                        ratioDiag = (vDiag.magnitude * localScale) / Mathf.Max(1e-4f, cachedBaselineExpRef);
                    }
                    else
                    {
                        float baseDet = GetDetectionMaxPairDistance();
                        float baseExp = GetDiagramMaxEdgeLength();
                        float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                        ratioDet = vDet.magnitude / baseline;
                        ratioDiag = (vDiag.magnitude * localScale) / baseline;
                    }
                }
                else
                {
                    float baseline = Vector3.Distance(GetPieceCenterWorld(neighborPiece), GetPieceCenterWorld(candidatePiece));
                    ratioDet = vDet.magnitude / Mathf.Max(1e-4f, baseline);
                    ratioDiag = (vDiag.magnitude * localScale) / Mathf.Max(1e-4f, baseline);
                }
                distDiff = Mathf.Abs(ratioDet - ratioDiag);
            }
            else
            {
                distDiff = Mathf.Abs(vDet.magnitude - vDiag.magnitude * localScale);
            }

            // Use new angle calculation method for consistency with other logging
            float expectedAngle = ComputeRelativeAngleDegForDiagram(candidatePiece, neighborPiece, "RelationalCost");
            float actualAngle = ComputeRelativeAngleDeg(center.pos, center.rot, dg.nodes[nIdx].pos, "RelationalCost");
            float angDiff = Mathf.Abs(NormalizeAngle180(actualAngle - expectedAngle));
            float cost = distDiff + rotationWeightMetersPerDeg * angDiff;

            if (useAbsoluteDistanceInCost)
            {
                var detAbs = latestDetections[center.detIndex];
                Vector3 planeN_abs = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                Vector3 vAbs = Vector3.ProjectOnPlane(GetPieceCenterWorld(candidatePiece) - detAbs.worldPosition, planeN_abs);
                float absDist0 = vAbs.magnitude;
                cost += absoluteDistanceWeightMeters * absDist0;
            }

            if (cost < bestCost)
            {
                bestCost = cost;
                bestDist = distDiff;
                bestAng = angDiff;
                bestScale = localScale;
                usedCount = 1;
            }
        }

        if (float.IsPositiveInfinity(bestCost)) return false;
        avgDistDiff = bestDist; avgAngDiff = bestAng; usedScale = bestScale; return true;
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
        Vector3 planeNormal = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 dirProj = Vector3.ProjectOnPlane(dir, planeNormal);
        if (dirProj.sqrMagnitude < 1e-6f) return 0f;
        dirProj.Normalize();
        // Compute angle relative to fixed X axis on the plane for stability
        Vector3 refForward = Vector3.ProjectOnPlane(Vector3.right, planeNormal).normalized;
        if (refForward.sqrMagnitude < 1e-6f) refForward = Vector3.right;
        float ang = Vector3.SignedAngle(refForward, dirProj, planeNormal);
        ang = NormalizeAngle180(ang);
        // Apply connection-angle offset if enabled
        if (applyOffsetToConnectionAngles)
            ang = NormalizeAngle180(ang + angleOffset);
        return ang;
    }

    /// <summary>
    /// [새로 추가] 다이어그램 조각 전용 상대 각도 계산 함수.
    /// 기준 조각(fromPiece)의 'center' -> 'direction' 벡터를 0도 기준선으로 사용합니다.
    /// </summary>
    /// <param name="fromPiece">기준 조각의 Transform</param>
    /// <param name="toPiece">목표 조각의 Transform</param>
    /// <param name="debugContext">디버깅용 컨텍스트 정보</param>
    /// <returns>상대 각도 (degrees, -180 ~ 180)</returns>
    /// <summary>
    /// Computes the relative angle (in degrees) from the "fromPiece" to the "toPiece" on the diagram plane.
    /// The angle is measured from the reference direction of the "fromPiece" (its local +X, or custom direction)
    /// to the vector pointing from "fromPiece" to "toPiece", both projected onto the diagram plane.
    /// </summary>
    /// <param name="fromPiece">Transform of the reference piece</param>
    /// <param name="toPiece">Transform of the target piece</param>
    /// <param name="debugContext">Optional debug context for detailed logging</param>
    /// <returns>Relative angle in degrees (-180 to 180)</returns>
    private float ComputeRelativeAngleDegForDiagram(Transform fromPiece, Transform toPiece, string debugContext = "")
    {
        // Define the normal vector of the plane used for angle calculation.
        Vector3 planeNormal = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);

        // 1. Get the reference direction vector for the "fromPiece" (this is the new 0-degree baseline).
        Vector3 fromOrientationVec = GetPieceDirectionVector(fromPiece);
        Vector3 fromOrientationProj = Vector3.ProjectOnPlane(fromOrientationVec, planeNormal).normalized;

        if (fromOrientationProj.sqrMagnitude < 1e-6f)
        {
            // Debug: Could not compute a valid reference direction vector.
            if (debugLogRelativeAngles && !string.IsNullOrEmpty(debugContext))
            {
                Debug.LogWarning($"[RelativeAngleDiagram] {debugContext}: Could not compute reference direction vector - returning 0 degrees.");
            }
            return 0f;
        }

        // 2. Compute the connection vector from "fromPiece" to "toPiece", projected onto the plane.
        Vector3 fromPos = GetPieceCenterWorld(fromPiece);
        Vector3 toPos = GetPieceCenterWorld(toPiece);
        Vector3 connectionVec = toPos - fromPos;
        Vector3 connectionVecProj = Vector3.ProjectOnPlane(connectionVec, planeNormal);

        if (connectionVecProj.sqrMagnitude < 1e-6f)
        {
            // Debug: Connection vector is too short to define an angle.
            if (debugLogRelativeAngles && !string.IsNullOrEmpty(debugContext))
            {
                Debug.LogWarning($"[RelativeAngleDiagram] {debugContext}: Connection vector is too short - returning 0 degrees.");
            }
            return 0f;
        }
        connectionVecProj.Normalize();

        // 3. Calculate the signed angle from the reference direction to the connection vector, on the plane.
        float relativeAngle = Vector3.SignedAngle(fromOrientationProj, connectionVecProj, planeNormal);
        float normalizedAngle = NormalizeAngle180(relativeAngle);

        // Debug: Output detailed information if enabled.
        if (debugLogRelativeAngles && !string.IsNullOrEmpty(debugContext))
        {
            Debug.Log($"[RelativeAngleDiagram] {debugContext}: " +
                      $"fromPiece={fromPiece.name}, toPiece={toPiece.name}, " +
                      $"referenceDir=({fromOrientationProj.x:F4},{fromOrientationProj.y:F4},{fromOrientationProj.z:F4}), " +
                      $"connectionDir=({connectionVecProj.x:F4},{connectionVecProj.y:F4},{connectionVecProj.z:F4}), " +
                      $"angle={normalizedAngle:F2}°");
        }

        return normalizedAngle;
    }

    /// <summary>
    /// Computes relative angle between two pieces using planar projection.
    /// This method projects both positions onto the diagram plane and calculates
    /// the angle relative to the reference piece's local coordinate system.
    /// </summary>
    /// <param name="fromPos">World position of the reference piece</param>
    /// <param name="fromRotation">World rotation of the reference piece</param>
    /// <param name="toPos">World position of the target piece</param>
    /// <param name="debugContext">Optional debug context for logging</param>
    /// <returns>Relative angle in degrees</returns>
    private float ComputeRelativeAngleDeg(Vector3 fromPos, Quaternion fromRotation, Vector3 toPos, string debugContext = "")
    {
        // Define the diagram plane
        Vector3 planeNormal = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 planeOrigin = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;

        // Project both positions onto the diagram plane
        Vector3 fromPosProjected = fromPos - Vector3.Dot(fromPos - planeOrigin, planeNormal) * planeNormal;
        Vector3 toPosProjected = toPos - Vector3.Dot(toPos - planeOrigin, planeNormal) * planeNormal;

        // Calculate direction vector in the plane
        Vector3 directionInPlane = toPosProjected - fromPosProjected;
        
        if (directionInPlane.sqrMagnitude < 1e-6f)
        {
            if (debugLogRelativeAngles && !string.IsNullOrEmpty(debugContext))
            {
                Debug.Log($"[RelativeAngle] {debugContext}: Zero distance, returning 0°");
            }
            return 0f;
        }

        directionInPlane.Normalize();

        // Get the reference piece's local +X direction projected onto the plane
        Vector3 referenceDirection = fromRotation * Vector3.right;
        Vector3 referenceDirProjected = Vector3.ProjectOnPlane(referenceDirection, planeNormal);
        
        if (referenceDirProjected.sqrMagnitude < 1e-6f)
        {
            // Fallback to world right if reference direction is parallel to plane normal
            referenceDirProjected = Vector3.ProjectOnPlane(Vector3.right, planeNormal);
        }
        
        referenceDirProjected.Normalize();

        // Calculate the relative angle
        float relativeAngle = Vector3.SignedAngle(referenceDirProjected, directionInPlane, planeNormal);
        relativeAngle = NormalizeAngle180(relativeAngle);

        // Apply connection-angle offset if enabled
        if (applyOffsetToConnectionAngles)
            relativeAngle = NormalizeAngle180(relativeAngle + angleOffset);

        // Debug logging
        if (debugLogRelativeAngles && !string.IsNullOrEmpty(debugContext))
        {
            Vector3 fromEuler = fromRotation.eulerAngles;
            Debug.Log($"[RelativeAngle] {debugContext}:");
            Debug.Log($"  From: ({fromPos.x:F3}, {fromPos.y:F3}, {fromPos.z:F3}) rot=({fromEuler.x:F1}°, {fromEuler.y:F1}°, {fromEuler.z:F1}°)");
            Debug.Log($"  To: ({toPos.x:F3}, {toPos.y:F3}, {toPos.z:F3})");
            Debug.Log($"  Projected From: ({fromPosProjected.x:F3}, {fromPosProjected.y:F3}, {fromPosProjected.z:F3})");
            Debug.Log($"  Projected To: ({toPosProjected.x:F3}, {toPosProjected.y:F3}, {toPosProjected.z:F3})");
            Debug.Log($"  Direction in plane: ({directionInPlane.x:F3}, {directionInPlane.y:F3}, {directionInPlane.z:F3})");
            Debug.Log($"  Reference direction: ({referenceDirProjected.x:F3}, {referenceDirProjected.y:F3}, {referenceDirProjected.z:F3})");
            Debug.Log($"  Relative angle: {relativeAngle:F2}° (offset applied: {applyOffsetToConnectionAngles})");
        }

        return relativeAngle;
    }

    private Vector3 GetPieceCenterWorld(Transform piece)
    {
        if (piece == null) return Vector3.zero;
        var mf = piece.GetComponent<MeshFilter>();
        // Prefer an explicit child named "center" if present
        var centerChild = piece.Find("center");
        if (centerChild != null)
            return centerChild.position;
        if (mf != null && mf.sharedMesh != null)
        {
            var bounds = mf.sharedMesh.bounds; // local-space bounds
            Vector3 localCenter = bounds.center;
            return piece.TransformPoint(localCenter);
        }
        // Fallback to transform.position
        return piece.position;
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

    // First-Pass Relative Scale helpers
    private void TrySetFirstPassScale(float dActual, float dExpected)
    {
        if (!useFirstPassRelativeScale) return;
        if (firstPassScaleValid) return;
        if (dExpected <= 1e-4f) return;

        firstPassScale = Mathf.Max(1e-4f, dActual / dExpected);
        firstPassScaleValid = true;
        firstPassScaleFrame = frameCounter;
    }

    private bool CheckDistanceWithFirstPassScale(float dActual, float dExpected, out float relErr, out float distCost)
    {
        // If scale is not available yet, temporarily allow distance to pass
        if (!useFirstPassRelativeScale || !firstPassScaleValid)
        {
            relErr = 0f;
            distCost = 0f;
            return true;
        }

        float expected = Mathf.Max(1e-4f, firstPassScale * dExpected);
        relErr = Mathf.Abs(dActual - expected) / expected;
        distCost = Mathf.Max(0f, relErr - firstPassScaleTolerance);
        return relErr <= firstPassScaleTolerance;
    }

    // Compute relation angle using cached planar coordinates, rounding to a specified number of decimals before forming the direction.
    private float ComputePlanarAngleFromPlaneCoords(Vector2 fromCoord, Vector2 toCoord, int decimals = 4)
    {
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 axisU = Vector3.Cross(planeN, Vector3.up);
        if (axisU.sqrMagnitude < 1e-6f) axisU = Vector3.Cross(planeN, Vector3.right);
        axisU.Normalize();
        Vector3 axisV = Vector3.Cross(planeN, axisU).normalized;
        float scale = 1f;
        for (int i = 0; i < decimals; i++) scale *= 10f;
        float dx = Mathf.Round((toCoord.x - fromCoord.x) * scale) / scale;
        float dy = Mathf.Round((toCoord.y - fromCoord.y) * scale) / scale;
        Vector3 dir = axisU * dx + axisV * dy;
        Vector3 dirProj = Vector3.ProjectOnPlane(dir, planeN);
        if (dirProj.sqrMagnitude < 1e-6f) return 0f;
        dirProj.Normalize();
        Vector3 refForward = Vector3.ProjectOnPlane(Vector3.right, planeN);
        if (refForward.sqrMagnitude < 1e-6f) refForward = Vector3.right;
        refForward.Normalize();
        float ang = Vector3.SignedAngle(refForward, dirProj, planeN);
        ang = NormalizeAngle180(ang);
        // Apply connection-angle offset if enabled
        if (applyOffsetToConnectionAngles)
            ang = NormalizeAngle180(ang + angleOffset);
        return ang;
    }

    /// <summary>
    /// Computes connection line angle between two 2D coordinates on the diagram plane.
    /// This is used for calculating expected angles in diagram graph construction.
    /// </summary>
    /// <param name="fromCoord">2D coordinate of the starting point</param>
    /// <param name="toCoord">2D coordinate of the ending point</param>
    /// <returns>Connection angle in degrees</returns>
    private float ComputeConnectionLineAngleFromCoords(Vector2 fromCoord, Vector2 toCoord)
    {
        Vector2 direction = toCoord - fromCoord;
        if (direction.sqrMagnitude < 1e-6f) return 0f;
        
        // Calculate angle relative to positive X axis (right direction)
        float angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
        angle = NormalizeAngle180(angle);
        
        // Apply connection-angle offset if enabled
        if (applyOffsetToConnectionAngles)
            angle = NormalizeAngle180(angle + angleOffset);
            
        return angle;
    }

    /// <summary>
    /// Computes connection line angle between two 3D world positions.
    /// This is a fallback method when 2D coordinates are not available.
    /// </summary>
    /// <param name="fromPos">3D world position of the starting point</param>
    /// <param name="toPos">3D world position of the ending point</param>
    /// <returns>Connection angle in degrees</returns>
    private float ComputeConnectionLineAngle(Vector3 fromPos, Vector3 toPos)
    {
        // This is essentially the same as ComputePlanarAngleDeg but with a different name for clarity
        return ComputePlanarAngleDeg(fromPos, toPos);
    }

    /// <summary>
    /// Projects a 3D world position onto the 2D diagram plane coordinate system.
    /// This ensures consistent 2D coordinate calculations for distance and angle measurements.
    /// </summary>
    /// <param name="worldPos">3D world position to project</param>
    /// <returns>2D coordinate on the diagram plane</returns>
    private Vector2 ProjectTo2DCornerPlane(Vector3 worldPos)
    {
        // Define the diagram plane
        Vector3 planeNormal = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 planeOrigin = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;

        // Create orthonormal basis for the plane
        Vector3 axisU = Vector3.Cross(planeNormal, Vector3.up);
        if (axisU.sqrMagnitude < 1e-6f) axisU = Vector3.Cross(planeNormal, Vector3.right);
        axisU.Normalize();
        Vector3 axisV = Vector3.Cross(planeNormal, axisU).normalized;

        // Project the world position onto the plane
        Vector3 projectedPos = worldPos - Vector3.Dot(worldPos - planeOrigin, planeNormal) * planeNormal;
        
        // Convert to 2D coordinates using the plane basis
        Vector3 relativePos = projectedPos - planeOrigin;
        float x = Vector3.Dot(relativePos, axisU);
        float y = Vector3.Dot(relativePos, axisV);

        return new Vector2(x, y);
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

    private Vector3 KalmanPredict(LockedState ls, float dt)
    {
        // Simple per-axis constant-velocity Kalman
        if (!ls.kalmanInitialized)
        {
            ls.kPos = ls.lastPosition;
            ls.kVel = ls.lastVelocity;
            ls.kCovX = new Vector4(1, 0, 0, 1);
            ls.kCovY = new Vector4(1, 0, 0, 1);
            ls.kCovZ = new Vector4(1, 0, 0, 1);
            ls.kalmanInitialized = true;
        }
        float q = kalmanProcessNoise;
        // Predict per axis: x
        PredictAxis(ref ls.kPos.x, ref ls.kVel.x, ref ls.kCovX, dt, q);
        PredictAxis(ref ls.kPos.y, ref ls.kVel.y, ref ls.kCovY, dt, q);
        PredictAxis(ref ls.kPos.z, ref ls.kVel.z, ref ls.kCovZ, dt, q);
        return ls.kPos;
    }

    private void KalmanUpdate(LockedState ls, Vector3 meas)
    {
        float r = kalmanMeasurementNoise;
        UpdateAxis(ref ls.kPos.x, ref ls.kVel.x, ref ls.kCovX, meas.x, r);
        UpdateAxis(ref ls.kPos.y, ref ls.kVel.y, ref ls.kCovY, meas.y, r);
        UpdateAxis(ref ls.kPos.z, ref ls.kVel.z, ref ls.kCovZ, meas.z, r);
        ls.lastPosition = ls.kPos;
        ls.lastVelocity = ls.kVel;
    }

    // ★ ADD ★: Mahalanobis distance^2 from predicted position to measurement z.
    // Uses per-axis position covariance p11 from Kalman (ls.kCov* .x).
    private float MahalanobisDistanceSquared(LockedState ls, Vector3 z, bool planar)
    {
        // Predict (do not mutate) — we already call real Predict/Update elsewhere.
        Vector3 pred = useKalmanPrediction
            ? KalmanPredict(ls, Time.deltaTime)
            : (ls.lastPosition + ls.lastVelocity * Time.deltaTime);

        if (gatingPlanar || planar)
        {
            Vector3 nrm = tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up;
            Vector3 zp = Vector3.ProjectOnPlane(z, nrm);
            Vector3 pp = Vector3.ProjectOnPlane(pred, nrm);
            float dx = zp.x - pp.x;
            float dz = zp.z - pp.z;

            // position variance ~ p11 of each axis filter
            float varX = Mathf.Max(ls.kCovX.x, 1e-6f);
            float varZ = Mathf.Max(ls.kCovZ.x, 1e-6f);

            return (dx * dx) / varX + (dz * dz) / varZ;
        }
        else
        {
            float dx = z.x - pred.x;
            float dy = z.y - pred.y;
            float dz = z.z - pred.z;

            float varX = Mathf.Max(ls.kCovX.x, 1e-6f);
            float varY = Mathf.Max(ls.kCovY.x, 1e-6f);
            float varZ = Mathf.Max(ls.kCovZ.x, 1e-6f);

            return (dx * dx) / varX + (dy * dy) / varY + (dz * dz) / varZ;
        }
    }

    private void PredictAxis(ref float pos, ref float vel, ref Vector4 cov, float dt, float q)
    {
        // State transition: [1 dt; 0 1]
        pos = pos + vel * dt;
        // Covariance prediction: P = F P F^T + Q
        float p11 = cov.x, p12 = cov.y, p21 = cov.z, p22 = cov.w;
        float f11 = 1f, f12 = dt, f21 = 0f, f22 = 1f;
        float n11 = f11 * p11 + f12 * p21;
        float n12 = f11 * p12 + f12 * p22;
        float n21 = f21 * p11 + f22 * p21;
        float n22 = f21 * p12 + f22 * p22;
        // P = N * F^T + Q
        float p11p = n11 * f11 + n12 * f12 + q;
        float p12p = n11 * f21 + n12 * f22;
        float p21p = n21 * f11 + n22 * f12;
        float p22p = n21 * f21 + n22 * f22 + q;
        cov = new Vector4(p11p, p12p, p21p, p22p);
    }

    private void UpdateAxis(ref float pos, ref float vel, ref Vector4 cov, float z, float r)
    {
        // Measurement H = [1 0], innovation y = z - pos
        float p11 = cov.x, p12 = cov.y, p21 = cov.z, p22 = cov.w;
        float s = p11 + r;
        float k1 = p11 / s;
        float k2 = p21 / s;
        float y = z - pos;
        pos = pos + k1 * y;
        vel = vel + k2 * y;
        // Covariance update: (I - K H) P
        float p11n = (1 - k1) * p11;
        float p12n = (1 - k1) * p12;
        float p21n = -k2 * p11 + p21;
        float p22n = -k2 * p12 + p22;
        cov = new Vector4(p11n, p12n, p21n, p22n);
    }

    private void DebugLogDiagramGraph()
    {
        if (diagramGraph == null)
        {
            Debug.Log("[TangramMatcher] Diagram graph is null");
            return;
        }
        
        Debug.Log($"[TangramMatcher] === Diagram Graph (relative angle based) ===");
        Debug.Log($"노드 수: {diagramGraph.nodes.Count}");
        
        for (int i = 0; i < diagramGraph.nodes.Count; i++)
        {
            var n = diagramGraph.nodes[i];
            Vector3 centerPos = GetPieceCenterWorld(n.piece);
            int pieceId = GetArucoIdForDiagramPiece(n.piece);
            Vector3 pieceEuler = n.piece.transform.rotation.eulerAngles;
            
            Debug.Log($"[{i}] {n.piece.name} (ArUco ID: {pieceId})");
            Debug.Log($"    Position: ({centerPos.x:F3}, {centerPos.y:F3}, {centerPos.z:F3})");
            Debug.Log($"    Rotation: ({pieceEuler.x:F1}°, {pieceEuler.y:F1}°, {pieceEuler.z:F1}°)");
            Debug.Log($"    Size: {n.sizeMeters:F3}m");
            Debug.Log($"    Number of connected edges: {n.edges.Count}");
            for (int e = 0; e < n.edges.Count; e++)
            {
                var ed = n.edges[e];
                var to = diagramGraph.nodes[ed.toIndex].piece;
                Vector3 toCenterPos = GetPieceCenterWorld(to);
                int toId = GetArucoIdForDiagramPiece(to);
                
                // 기존 방식과 비교를 위한 각도 계산
                float legacyAngle = ComputePlanarAngleDeg(centerPos, toCenterPos);
                float angleDiff = Mathf.Abs(NormalizeAngle180(ed.expectedAngleDeg - legacyAngle));
                
                Debug.Log($"    -> {to.name} (ArUco ID: {toId})");
                Debug.Log($"       Distance: {ed.expectedDistanceMeters:F3}m");
                Debug.Log($"       Relative angle: {ed.expectedAngleDeg:F2}° (legacy: {legacyAngle:F2}°, diff: {angleDiff:F2}°)");
                Debug.Log($"       Normalized distance: {ed.normalizedDistance:F3}");
                Debug.Log($"       Target position: ({toCenterPos.x:F3}, {toCenterPos.y:F3}, {toCenterPos.z:F3})");
            }
            
            if (n.edges.Count == 0)
            {
                Debug.Log($"    (No connected edges)");
            }
        }
        
        Debug.Log($"=== Diagram Graph complete ===");
    }

    /// <summary>
    /// 감지된 도형들 간의 상대 각도 관계를 요약해서 로그로 출력합니다.
    /// </summary>
    private void DebugLogDetectionRelationsSummary()
    {
        // If detailed relative angle debugging is not enabled, or there are not enough detections, exit early.
        if (!debugLogRelativeAngles || latestDetections == null || latestDetections.Count < 2)
            return;

        Debug.Log($"[TangramMatcher] === Summary of Relative Angles Between Detected Shapes ===");
        Debug.Log($"Number of detected shapes: {latestDetections.Count}");

        // Filter out corner detections, as we only want to analyze non-corner shapes.
        var nonCornerDetections = new List<DetectedShape>();
        foreach (var det in latestDetections)
        {
            if (!det.isCorner) nonCornerDetections.Add(det);
        }

        Debug.Log($"Number of non-corner shapes: {nonCornerDetections.Count}");

        for (int i = 0; i < nonCornerDetections.Count; i++)
        {
            var detA = nonCornerDetections[i];
            Vector3 eulerA = detA.worldRotation.eulerAngles;

            Debug.Log($"[{i}] {detA.shapeType}:{detA.arucoId}");
            Debug.Log($"    Position: ({detA.worldPosition.x:F3}, {detA.worldPosition.y:F3}, {detA.worldPosition.z:F3})");
            Debug.Log($"    Rotation (Euler angles): ({eulerA.x:F1}°, {eulerA.y:F1}°, {eulerA.z:F1}°)");
            Debug.Log($"    Plane coordinates: ({detA.planeCoord.x:F3}, {detA.planeCoord.y:F3})");
            Debug.Log($"    Plane angle: {detA.planeAngleDeg:F2}°");

            // Log relationships to all subsequent non-corner shapes (to avoid duplicate pairs)
            for (int j = i + 1; j < nonCornerDetections.Count; j++)
            {
                var detB = nonCornerDetections[j];
                // Always order by arucoId for consistency in debug output
                var fromDet = (detA.arucoId <= detB.arucoId) ? detA : detB;
                var toDet = (detA.arucoId <= detB.arucoId) ? detB : detA;

                float relativeAngle = ComputeRelativeAngleDeg(
                    fromDet.worldPosition,
                    fromDet.worldRotation,
                    toDet.worldPosition,
                    $"SUMMARY {fromDet.shapeType}:{fromDet.arucoId} → {toDet.shapeType}:{toDet.arucoId}"
                );

                float distance = Vector3.Distance(detA.worldPosition, detB.worldPosition);

                Debug.Log($"    -> {detB.shapeType}:{detB.arucoId}: Distance={distance:F3}m, Relative angle={relativeAngle:F2}°");
            }
        }

        Debug.Log($"=== Summary of detected shape relationships complete ===");
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
        if (showMissingCornersUI && requirePlaneCorners && !planeCornersReady)
        {
        var cam = uiCamera != null ? uiCamera : Camera.main;
        if (cam == null) cam = Camera.current;
        if (cam == null && Camera.allCamerasCount > 0) cam = Camera.allCameras[0];
            if (cam != null)
            {
                if (cachedLabelStyle == null)
                {
                    cachedLabelStyle = new GUIStyle(GUI.skin.label)
                    {
                        fontSize = uiFontSize,
                        normal = { textColor = Color.yellow }
                    };
                }
                GUI.Label(new Rect(20f, 20f, 600f, 24f), "Corner markers not detected – matching paused", cachedLabelStyle);
            }
        }
        if (!showErrorLabelsUI && !showDetectionEulerUI)
            return;
        var cam2 = uiCamera != null ? uiCamera : Camera.main;
        if (cam2 == null) cam2 = Camera.current;
        if (cam2 == null && Camera.allCamerasCount > 0) cam2 = Camera.allCameras[0];
        if (cam2 == null) return;
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

        if (showErrorLabelsUI)
        {
            foreach (var res in lastMatchResults)
            {
                Vector3 world = GetPieceCenterWorld(res.matchedPieceTransform);
                Vector3 sp = cam2.WorldToScreenPoint(world);
                if (sp.z < 0f) continue; // behind camera
                float x = sp.x;
                float y = Screen.height - sp.y;
                string text = $"{res.shapeType}: {res.worldDistanceMeters:F2}m, {res.angleErrorDegrees:F0}°";
                GUI.Label(new Rect(x + 6f, y - 18f, 260f, 22f), text, cachedLabelStyle);
            }
        }

        if (showDetectionEulerUI)
        {
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var d = latestDetections[i];
                if (d.isCorner) continue;
                Vector3 sp = cam2.WorldToScreenPoint(d.worldPosition);
                if (sp.z < 0f) continue;
                float x = sp.x;
                float y = Screen.height - sp.y;
                
                // Use cached planar metrics
                float projX = d.planeCoord.x;
                float projY = d.planeCoord.y;
                float absAngle = d.planeAngleDeg;
                string text = $"ID{d.arucoId} | Plane:({projX:F2}, {projY:F2}) | Angle:{absAngle:F1}°";
                GUI.Label(new Rect(x + 6f, y + 4f, 400f, 22f), text, cachedLabelStyle);
            }
        }
    }

    private static bool HasRenderableMesh(GameObject go)
    {
        return go != null && go.GetComponent<MeshFilter>() != null && go.GetComponent<MeshRenderer>() != null;
    }

    private LineRenderer GetOrCreateOrientationArrow(int arucoId)
    {
        if (!orientationArrowById.TryGetValue(arucoId, out var lr) || lr == null)
        {
            var go = new GameObject($"MarkerArrow_{arucoId}");
            go.transform.SetParent(transform, false);
            lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.numCapVertices = 0;
            lr.numCornerVertices = 0;
            lr.positionCount = 2;
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = detectionOrientationArrowColor;
            lr.endColor = detectionOrientationArrowColor;
            lr.startWidth = detectionOrientationArrowWidth;
            lr.endWidth = detectionOrientationArrowWidth;
            orientationArrowById[arucoId] = lr;
        }
        // Keep style up to date from inspector
        lr.startColor = detectionOrientationArrowColor;
        lr.endColor = detectionOrientationArrowColor;
        lr.startWidth = detectionOrientationArrowWidth;
        lr.endWidth = detectionOrientationArrowWidth;
        return lr;
    }

    private void UpdateDetectionOrientationArrows()
    {
        // Disable all if toggle off
        if (!drawDetectionOrientationArrows)
        {
            foreach (var kv in orientationArrowById)
            {
                if (kv.Value != null) kv.Value.enabled = false;
            }
            return;
        }
        // Determine plane normal to project onto
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        var alive = new HashSet<int>();
        for (int i = 0; i < latestDetections.Count; i++)
        {
            var d = latestDetections[i];
            if (d.isCorner) continue;
            alive.Add(d.arucoId);
            var lr = GetOrCreateOrientationArrow(d.arucoId);
            Vector3 pos = d.worldPosition;
            // Use marker's local +X direction projected onto the plane
            Vector3 dir = d.worldRotation * Vector3.right;
            Vector3 dirProj = Vector3.ProjectOnPlane(dir, planeN);
            if (dirProj.sqrMagnitude < 1e-8f)
            {
                // Fallback to +Z if +X is nearly parallel to normal
                dirProj = Vector3.ProjectOnPlane(d.worldRotation * Vector3.forward, planeN);
            }
            dirProj = dirProj.normalized * detectionOrientationArrowLength;
            lr.SetPosition(0, pos);
            lr.SetPosition(1, pos + dirProj);
            lr.enabled = true;
        }
        // Disable arrows for ids not alive this frame
        var toDisable = new List<int>();
        foreach (var kv in orientationArrowById)
        {
            if (!alive.Contains(kv.Key) && kv.Value != null)
            {
                kv.Value.enabled = false;
                toDisable.Add(kv.Key);
            }
        }
        // Optionally keep instances for reuse; we only disable.
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
            // 0,1: Large Triangle
            case 0: shapeType = TangramShapeType.LargeTriangle; return true;
            case 1: shapeType = TangramShapeType.LargeTriangle; return true;
            // 2: Medium Triangle
            case 2: shapeType = TangramShapeType.MediumTriangle; return true;
            // 3,4: Small Triangle
            case 3: shapeType = TangramShapeType.SmallTriangle; return true;
            case 4: shapeType = TangramShapeType.SmallTriangle; return true;
            // 5: Square
            case 5: shapeType = TangramShapeType.Square; return true;
            // 6,7: Parallelogram
            case 6: shapeType = TangramShapeType.Parallelogram; return true;
            case 7: shapeType = TangramShapeType.Parallelogram; return true;
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

        // Use the same plane normal policy as elsewhere
        Vector3 planeNormal = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);

        // Compute detection direction: by convention use local +X of the ArUco marker
        Vector3 detDir = detectionWorldRotation * Vector3.right;
        Vector3 detProj = Vector3.ProjectOnPlane(detDir, planeNormal).normalized;

        // Compute piece direction from 'center' -> 'direction' child if present; fallback to piece forward
        Vector3 pieceDir = GetPieceDirectionVector(piece);
        Vector3 pieceProj = Vector3.ProjectOnPlane(pieceDir, planeNormal).normalized;

        if (detProj.sqrMagnitude < 1e-6f || pieceProj.sqrMagnitude < 1e-6f)
            return 0f;

        float rawAngle = Vector3.SignedAngle(pieceProj, detProj, planeNormal);
        // Normalize later after applying offset
        // Apply orientation offset if enabled
        if (applyOffsetToOrientations)
        {
            rawAngle = rawAngle + angleOffset;
        }
        rawAngle = NormalizeAngle180(rawAngle);

        // Handle symmetry per type (modulo)
        float symmetry = GetSymmetryModuloDegrees(type);
        if (symmetry > 0f)
        {
            rawAngle = rawAngle % symmetry;
            rawAngle = Mathf.Min(rawAngle, symmetry - rawAngle);
        }

        // Mirror-insensitive for parallelogram only when absolute orientation is not enforced
        if (!useAbsoluteOrientation && type == TangramShapeType.Parallelogram)
        {
            rawAngle = Mathf.Min(rawAngle, Mathf.Abs(180f - rawAngle));
        }

        return Mathf.Abs(rawAngle);
    }

    /// <summary>
    /// Returns a world-space direction vector for a diagram piece.
    /// Prefers child objects named 'center' and 'direction', using (direction.position - center.position).
    /// Falls back to the piece's forward vector if children are missing or degenerate.
    /// </summary>
    private Vector3 GetPieceDirectionVector(Transform piece)
    {
        if (piece == null) return Vector3.forward;
        Transform center = piece.Find("center");
        Transform dir = piece.Find("direction");
        if (center != null && dir != null)
        {
            Vector3 v = dir.position - center.position;
            if (v.sqrMagnitude > 1e-8f)
                return v.normalized;
        }
        // Fallback: use the transform's forward
        return piece.forward;
    }

    private static float NormalizeAngle180(float deg)
    {
        while (deg > 180f) deg -= 360f;
        while (deg < -180f) deg += 360f;
        return deg;
    }

    /// <summary>


    /// <summary>
    /// Computes a consistent relation angle between two detections using the 'low ArUco ID -> high ArUco ID' rule.
    /// Uses cached 2D plane coordinates for stability.
    /// </summary>
    private float ComputeConsistentRelationAngle(DetectedShape shapeA, DetectedShape shapeB)
    {
        var fromShape = (shapeA.arucoId <= shapeB.arucoId) ? shapeA : shapeB;
        var toShape   = (shapeA.arucoId <= shapeB.arucoId) ? shapeB : shapeA;
        return ComputeConnectionLineAngleFromCoords(fromShape.planeCoord, toShape.planeCoord);
    }

    /// <summary>
    /// Stabilizes corner positions using exponential smoothing to reduce jitter
    /// </summary>
    private void StabilizeCornerPositions()
    {
        currentFrame++;
        
        if (!enableCornerStabilization)
        {
            // If stabilization is disabled, just copy raw positions
            stabilizedCornerPositions.Clear();
            foreach (var det in latestDetections)
            {
                if (det.isCorner && det.arucoId >= 10 && det.arucoId <= 13)
                {
                    stabilizedCornerPositions[det.arucoId] = det.worldPosition;
                }
            }
            return;
        }
        
        // Update raw corner positions
        var currentRawPositions = new Dictionary<int, Vector3>();
        foreach (var det in latestDetections)
        {
            if (det.isCorner && det.arucoId >= 10 && det.arucoId <= 13)
            {
                currentRawPositions[det.arucoId] = det.worldPosition;
                rawCornerPositions[det.arucoId] = det.worldPosition;
                cornerSeenFrames[det.arucoId] = currentFrame;
            }
        }
        
        // Apply exponential smoothing to corner positions
        foreach (var kv in currentRawPositions)
        {
            int cornerId = kv.Key;
            Vector3 rawPos = kv.Value;
            
            if (stabilizedCornerPositions.ContainsKey(cornerId))
            {
                // Smooth with previous stabilized position
                Vector3 prevStabilized = stabilizedCornerPositions[cornerId];
                float distance = Vector3.Distance(rawPos, prevStabilized);
                
                // Use stronger smoothing for small movements, less for large movements
                float adaptiveSmoothingFactor = cornerSmoothingFactor;
                if (distance > cornerMovementThreshold * 2f)
                {
                    // Reduce smoothing for larger movements to maintain responsiveness
                    adaptiveSmoothingFactor *= 0.5f;
                }
                
                stabilizedCornerPositions[cornerId] = Vector3.Lerp(rawPos, prevStabilized, adaptiveSmoothingFactor);
            }
            else
            {
                // First time seeing this corner, no smoothing
                stabilizedCornerPositions[cornerId] = rawPos;
            }
        }
        
        // Remove corners that haven't been seen recently
        var toRemove = new List<int>();
        foreach (var kv in cornerSeenFrames)
        {
            if (currentFrame - kv.Value > 10) // Remove after 10 frames of absence
            {
                toRemove.Add(kv.Key);
            }
        }
        foreach (int cornerId in toRemove)
        {
            stabilizedCornerPositions.Remove(cornerId);
            rawCornerPositions.Remove(cornerId);
            cornerSeenFrames.Remove(cornerId);
        }
    }
    
    /// <summary>
    /// Checks if a new plane configuration is significantly different from the current one
    /// </summary>
    private bool IsPlaneConfigurationStable(CornerPlaneSystem newPlane)
    {
        if (!currentCornerPlane.isValid || !newPlane.isValid)
            return false;
        
        // Check if corner IDs are the same
        bool sameCorners = true;
        for (int i = 0; i < 3; i++)
        {
            bool foundMatch = false;
            for (int j = 0; j < 3; j++)
            {
                if (currentCornerPlane.cornerIds[i] == newPlane.cornerIds[j])
                {
                    foundMatch = true;
                    break;
                }
            }
            if (!foundMatch)
            {
                sameCorners = false;
                break;
            }
        }
        
        if (!sameCorners)
            return false; // Different corner combination
        
        // Check angular deviation
        float angleDiff = Vector3.Angle(currentCornerPlane.planeNormal, newPlane.planeNormal);
        if (angleDiff > maxPlaneAngleDeviation)
            return false; // Too much angular change
        
        // Check position stability
        float originDist = Vector3.Distance(currentCornerPlane.origin, newPlane.origin);
        if (originDist > cornerMovementThreshold * 3f)
            return false; // Origin moved too much
        
        return true;
    }
    
    /// <summary>
    /// Builds a 2D plane coordinate system using 3 corner markers (ArUco IDs 10,11,12,13) with stabilization.
    /// Creates a stable reference frame for consistent 2D projection of all 3D points.
    /// </summary>
    private bool BuildCornerPlaneSystem()
    {
        // First, stabilize corner positions
        StabilizeCornerPositions();
        
        // Find corner markers with IDs 10, 11, 12, 13 using stabilized positions
        var availableCorners = new List<(int id, Vector3 pos)>();
        foreach (var kv in stabilizedCornerPositions)
        {
            availableCorners.Add((kv.Key, kv.Value));
        }
        
        if (availableCorners.Count < 3)
        {
            if (debugDetectionRelations)
            {
                string cornerIdStr = "";
                for (int c = 0; c < availableCorners.Count; c++)
                {
                    if (c > 0) cornerIdStr += ",";
                    cornerIdStr += availableCorners[c].id.ToString();
                }
                Debug.Log($"[TangramMatcher] CORNER_PLANE insufficient corners: found {availableCorners.Count} [{cornerIdStr}], need at least 3");
            }
            
            // Keep using previous plane if available
            if (currentCornerPlane.isValid)
            {
                currentCornerPlane.frameCount = 0; // Reset frame count due to insufficient corners
                return true;
            }
            currentCornerPlane.isValid = false;
            return false;
        }
        
        // Find the combination of 3 corners that forms the largest triangle (most stable)
        float bestArea = 0f;
        Vector3 bestP1 = Vector3.zero, bestP2 = Vector3.zero, bestP3 = Vector3.zero;
        int[] bestIds = new int[3];
        
        for (int i = 0; i < availableCorners.Count - 2; i++)
        {
            for (int j = i + 1; j < availableCorners.Count - 1; j++)
            {
                for (int k = j + 1; k < availableCorners.Count; k++)
                {
                    Vector3 p1 = availableCorners[i].pos;
                    Vector3 p2 = availableCorners[j].pos;
                    Vector3 p3 = availableCorners[k].pos;
                    
                    // Calculate triangle area to find the largest triangle (most stable plane)
                    Vector3 v1 = p2 - p1;
                    Vector3 v2 = p3 - p1;
                    float area = Vector3.Cross(v1, v2).magnitude * 0.5f;
                    
                    if (area > bestArea)
                    {
                        bestArea = area;
                        bestP1 = p1;
                        bestP2 = p2;
                        bestP3 = p3;
                        bestIds[0] = availableCorners[i].id;
                        bestIds[1] = availableCorners[j].id;
                        bestIds[2] = availableCorners[k].id;
                    }
                }
            }
        }
        
        // Check if we found a valid plane (non-degenerate triangle)
        if (bestArea < 1e-6f)
        {
            if (debugDetectionRelations)
            {
                Debug.Log("[TangramMatcher] CORNER_PLANE degenerate triangle: corners are collinear");
            }
            
            // Keep using previous plane if available
            if (currentCornerPlane.isValid)
            {
                currentCornerPlane.frameCount = 0; // Reset frame count due to degenerate triangle
                return true;
            }
            currentCornerPlane.isValid = false;
            return false;
        }
        
        // Create candidate new plane
        var candidatePlane = new CornerPlaneSystem();
        candidatePlane.origin = bestP1;
        candidatePlane.cornerIds[0] = bestIds[0];
        candidatePlane.cornerIds[1] = bestIds[1];
        candidatePlane.cornerIds[2] = bestIds[2];
        
        // Calculate plane normal
        Vector3 edge1 = bestP2 - bestP1;
        Vector3 edge2 = bestP3 - bestP1;
        candidatePlane.planeNormal = Vector3.Cross(edge1, edge2).normalized;
        
        // Create orthonormal coordinate system
        candidatePlane.uAxis = edge1.normalized;  // U axis along first edge
        candidatePlane.vAxis = Vector3.Cross(candidatePlane.planeNormal, candidatePlane.uAxis).normalized;
        candidatePlane.isValid = true;
        candidatePlane.stability = bestArea; // Use area as stability metric
        
        // Check if we should update the current plane
        bool shouldUpdate = false;
        
        if (!currentCornerPlane.isValid)
        {
            // No current plane, use the new one
            shouldUpdate = true;
        }
        else if (IsPlaneConfigurationStable(candidatePlane))
        {
            // Configuration is stable, increment frame count and update gradually
            currentCornerPlane.frameCount++;
            if (currentCornerPlane.frameCount >= minStableFrames)
            {
                // Smoothly update current plane towards candidate
                float updateRate = 0.3f; // Gentle update rate
                currentCornerPlane.origin = Vector3.Lerp(currentCornerPlane.origin, candidatePlane.origin, updateRate);
                currentCornerPlane.planeNormal = Vector3.Slerp(currentCornerPlane.planeNormal, candidatePlane.planeNormal, updateRate);
                currentCornerPlane.uAxis = Vector3.Slerp(currentCornerPlane.uAxis, candidatePlane.uAxis, updateRate);
                currentCornerPlane.vAxis = Vector3.Slerp(currentCornerPlane.vAxis, candidatePlane.vAxis, updateRate);
                currentCornerPlane.stability = Mathf.Lerp(currentCornerPlane.stability, candidatePlane.stability, updateRate);
            }
        }
        else
        {
            // Configuration changed significantly
            if (candidatePlane.stability > currentCornerPlane.stability * 1.2f)
            {
                // New configuration is much more stable, switch to it
                shouldUpdate = true;
            }
            else
            {
                // Reset frame count but keep current plane
                currentCornerPlane.frameCount = 0;
            }
        }
        
        if (shouldUpdate)
        {
            previousCornerPlane = currentCornerPlane.Clone();
            currentCornerPlane = candidatePlane;
            currentCornerPlane.frameCount = 1;
            
            if (debugDetectionRelations)
            {
                Debug.Log($"[TangramMatcher] CORNER_PLANE updated using corners [{bestIds[0]},{bestIds[1]},{bestIds[2]}] | " +
                    $"origin=({currentCornerPlane.origin.x:F4},{currentCornerPlane.origin.y:F4},{currentCornerPlane.origin.z:F4}) | " +
                    $"normal=({currentCornerPlane.planeNormal.x:F4},{currentCornerPlane.planeNormal.y:F4},{currentCornerPlane.planeNormal.z:F4}) | " +
                    $"area={bestArea:F4} | stability={currentCornerPlane.stability:F4}");
            }
        }
        
        return currentCornerPlane.isValid;
    }



    /// <summary>
    /// Debug function that logs all connection angles between detected shapes for comprehensive debugging.
    /// This provides a complete overview of all detected shape relationships and their connection angles.
    /// </summary>
    private void DebugLogAllConnectionAngles()
    {
        if (!debugDetectionRelations) return;
        
        Debug.Log("[TangramMatcher] === All Detection Connection Angles ===");
        
        for (int i = 0; i < latestDetections.Count; i++)
        {
            if (latestDetections[i].isCorner) continue;
            
            for (int j = i + 1; j < latestDetections.Count; j++)
            {
                if (latestDetections[j].isCorner) continue;
                
                var A = latestDetections[i];
                var B = latestDetections[j];
                
                // ★ 변경점: 새로운 상대 각도 계산 방식을 사용합니다.
                var fromShape = (A.arucoId <= B.arucoId) ? A : B;
                var toShape   = (A.arucoId <= B.arucoId) ? B : A;
                
                string allConnectionsDebugContext = debugLogRelativeAngles ? 
                    $"ALL_CONNECTIONS {fromShape.shapeType}:{fromShape.arucoId} → {toShape.shapeType}:{toShape.arucoId}" : "";
                
                float connectionAngle = ComputeRelativeAngleDeg(
                    fromShape.worldPosition,
                    fromShape.worldRotation,
                    toShape.worldPosition,
                    allConnectionsDebugContext
                );
                float distance = Vector2.Distance(A.planeCoord, B.planeCoord);
                
                Debug.Log($"[TangramMatcher] CONNECTION det({A.shapeType}:{A.arucoId}) -> det({B.shapeType}:{B.arucoId}) | " +
                    $"angle={connectionAngle:F4}° distance={distance:F3} | " +
                    $"A2D=({A.planeCoord.x:F4},{A.planeCoord.y:F4}) B2D=({B.planeCoord.x:F4},{B.planeCoord.y:F4})");
            }
        }
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
                return 0f; // Triangles have no rotational symmetry (orientation is unique)
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
            float actualDist = Vector3.Distance(GetPieceCenterWorld(node.piece), GetPieceCenterWorld(neigh));
            // Use consistent low-ID->high-ID for diagnostic angle
            int idA_dbg = GetArucoIdForDiagramPiece(node.piece);
            int idB_dbg = GetArucoIdForDiagramPiece(neigh);
            Vector3 from_dbg = (idA_dbg >= 0 && idB_dbg >= 0 && idA_dbg > idB_dbg) ? GetPieceCenterWorld(neigh) : GetPieceCenterWorld(node.piece);
            Vector3 to_dbg   = (idA_dbg >= 0 && idB_dbg >= 0 && idA_dbg > idB_dbg) ? GetPieceCenterWorld(node.piece) : GetPieceCenterWorld(neigh);
            float actualAng = ComputePlanarAngleDeg(from_dbg, to_dbg);
            float dDiff = Mathf.Abs(actualDist - e.expectedDistanceMeters);
            float aDiff = Mathf.Abs(NormalizeAngle180(actualAng - e.expectedAngleDeg));
            parts.Add($"-> '{neigh.name}' diff(d)={dDiff:F2}m diff(deg)={aDiff:F0}°");
        }
        return string.Join(", ", parts);
    }

    private void UpdatePlaneCornersFromDetections()
    {
        planeCornersReady = false;
        // Prefer inferring TL/TR/BR/BL from camera coordinates of detected corner markers
        var cornerDetections = new List<DetectedShape>();
        foreach (var d in latestDetections)
        {
            if (d.isCorner) cornerDetections.Add(d);
        }
        if (cornerDetections.Count >= 4)
        {
            if (TryInferPlaneCornersFromCamera(cornerDetections, out var TL, out var TR, out var BR, out var BL))
            {
                planeTLWorld = TL; planeTRWorld = TR; planeBRWorld = BR; planeBLWorld = BL;
                planeCornersReady = true; planeCornersFrame = frameCounter; return;
            }
        }
        // Fallback to ID-based mapping when inference is not possible
        bool hasTL = false, hasTR = false, hasBR = false, hasBL = false;
        foreach (var d in latestDetections)
        {
            if (d.arucoId == planeCornerIdTL) { hasTL = true; planeTLWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdTR) { hasTR = true; planeTRWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdBR) { hasBR = true; planeBRWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdBL) { hasBL = true; planeBLWorld = d.worldPosition; }
        }
        planeCornersReady = hasTL && hasTR && hasBR && hasBL;
        if (planeCornersReady) planeCornersFrame = frameCounter;
    }

    private List<DetectedShape> FuseBufferedDetections()
    {
        // Median fuse positions over buffer window per (type, id)
        var posX = new Dictionary<(TangramShapeType type, int id), List<float>>();
        var posY = new Dictionary<(TangramShapeType type, int id), List<float>>();
        var posZ = new Dictionary<(TangramShapeType type, int id), List<float>>();
        var latestSample = new Dictionary<(TangramShapeType type, int id), DetectedShape>();
        var latestFrame = new Dictionary<(TangramShapeType type, int id), int>();
        foreach (var tuple in recentDetectionsBuffer)
        {
            int f = tuple.frame;
            foreach (var d in tuple.items)
            {
                var key = (d.shapeType, d.arucoId);
                if (!posX.TryGetValue(key, out var lx)) { lx = new List<float>(); posX[key] = lx; }
                if (!posY.TryGetValue(key, out var ly)) { ly = new List<float>(); posY[key] = ly; }
                if (!posZ.TryGetValue(key, out var lz)) { lz = new List<float>(); posZ[key] = lz; }
                lx.Add(d.worldPosition.x);
                ly.Add(d.worldPosition.y);
                lz.Add(d.worldPosition.z);
                if (!latestFrame.TryGetValue(key, out var lf) || f >= lf)
                {
                    latestFrame[key] = f;
                    latestSample[key] = d; // keep latest rotation/confidence
                }
            }
        }
        float MedianOf(List<float> vals)
        {
            vals.Sort();
            int n = vals.Count;
            if (n == 0) return 0f;
            if ((n & 1) == 1) return vals[n / 2];
            return 0.5f * (vals[n / 2 - 1] + vals[n / 2]);
        }
        var fused = new List<DetectedShape>();
        foreach (var kv in latestSample)
        {
            var key = kv.Key;
            var baseDet = kv.Value;
            var lx = posX[key]; var ly = posY[key]; var lz = posZ[key];
            Vector3 med = new Vector3(MedianOf(lx), MedianOf(ly), MedianOf(lz));
            baseDet.worldPosition = med;
            fused.Add(baseDet);
        }
        return fused;
    }

    private void UpdatePlaneCornersFromList(List<DetectedShape> list)
    {
        planeCornersReady = false;
        // Prefer camera-based inference from provided list
        var cornerDetections = new List<DetectedShape>();
        foreach (var d in list) if (d.isCorner) cornerDetections.Add(d);
        if (cornerDetections.Count >= 4)
        {
            if (TryInferPlaneCornersFromCamera(cornerDetections, out var TL, out var TR, out var BR, out var BL))
            {
                planeTLWorld = TL; planeTRWorld = TR; planeBRWorld = BR; planeBLWorld = BL;
                planeCornersReady = true; planeCornersFrame = frameCounter; return;
            }
        }
        // Fallback: IDs within the provided list
        bool hasTL = false, hasTR = false, hasBR = false, hasBL = false;
        foreach (var d in list)
        {
            if (d.arucoId == planeCornerIdTL) { hasTL = true; planeTLWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdTR) { hasTR = true; planeTRWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdBR) { hasBR = true; planeBRWorld = d.worldPosition; }
            else if (d.arucoId == planeCornerIdBL) { hasBL = true; planeBLWorld = d.worldPosition; }
        }
        // Fallback: also look into the recentCornerBuffer if any missing
        if (!(hasTL && hasTR && hasBR && hasBL) && recentCornerBuffer.Count > 0)
        {
            var merged = new Dictionary<int, List<float>>(); // xTL,yTL,zTL,xTR,... (we'll do simple last value)
            foreach (var tuple in recentCornerBuffer)
            {
                foreach (var kv in tuple.corners)
                {
                    if (!merged.TryGetValue(kv.Key, out var listVals)) { listVals = new List<float>(); merged[kv.Key] = listVals; }
                    listVals.Add(kv.Value.x); listVals.Add(kv.Value.y); listVals.Add(kv.Value.z);
                }
            }
            Vector3 LastOf(List<float> vals)
            {
                if (vals == null || vals.Count < 3) return Vector3.zero;
                int n = vals.Count;
                return new Vector3(vals[n-3], vals[n-2], vals[n-1]);
            }
            if (!hasTL && merged.TryGetValue(planeCornerIdTL, out var tlv)) { planeTLWorld = LastOf(tlv); hasTL = tlv.Count >= 3; }
            if (!hasTR && merged.TryGetValue(planeCornerIdTR, out var trv)) { planeTRWorld = LastOf(trv); hasTR = trv.Count >= 3; }
            if (!hasBR && merged.TryGetValue(planeCornerIdBR, out var brv)) { planeBRWorld = LastOf(brv); hasBR = brv.Count >= 3; }
            if (!hasBL && merged.TryGetValue(planeCornerIdBL, out var blv)) { planeBLWorld = LastOf(blv); hasBL = blv.Count >= 3; }
        }
        planeCornersReady = hasTL && hasTR && hasBR && hasBL;
        if (planeCornersReady) planeCornersFrame = frameCounter;
    }

    // Infer TL/TR/BR/BL from camera-view coordinates of detected corner markers
    private bool TryInferPlaneCornersFromCamera(List<DetectedShape> cornerDetections, out Vector3 TL, out Vector3 TR, out Vector3 BR, out Vector3 BL)
    {
        TL = TR = BR = BL = Vector3.zero;
        try
        {
            if (cornerDetections == null || cornerDetections.Count < 4) return false;
            // Build camera-local coordinates
            Matrix4x4 camInv;
            if (arUcoCameraTransform != null)
                camInv = arUcoCameraTransform.worldToLocalMatrix;
            else
                camInv = Matrix4x4.identity; // fallback: world space used as camera space

            var pts = new List<(Vector3 world, Vector3 cam)>();
            for (int i = 0; i < cornerDetections.Count; i++)
            {
                var w = cornerDetections[i].worldPosition;
                var c = camInv.MultiplyPoint3x4(w);
                pts.Add((w, c));
            }
            // Keep four most stable by z-depth (nearest to camera) if more than 4 present
            if (pts.Count > 4)
            {
                pts.Sort((a, b) => a.cam.z.CompareTo(b.cam.z)); // smaller z is closer in camera local
                pts = pts.GetRange(0, 4);
            }
            // Split into left/right by x
            pts.Sort((a, b) => a.cam.x.CompareTo(b.cam.x));
            var leftTwo = new List<(Vector3 world, Vector3 cam)> { pts[0], pts[1] };
            var rightTwo = new List<(Vector3 world, Vector3 cam)> { pts[2], pts[3] };
            // Within each side, top = max y, bottom = min y
            leftTwo.Sort((a, b) => b.cam.y.CompareTo(a.cam.y)); // desc by y
            rightTwo.Sort((a, b) => b.cam.y.CompareTo(a.cam.y));
            TL = leftTwo[0].world;
            BL = leftTwo[leftTwo.Count - 1].world;
            TR = rightTwo[0].world;
            BR = rightTwo[rightTwo.Count - 1].world;
            return true;
        }
        catch { return false; }
    }

    private void ComputeDiagramPlane(out Vector3 ex, out Vector3 ez, out Vector3 originTL, out float widthDiag, out float heightDiag)
    {
        // Plane normal from marker plane
        Vector3 n = Vector3.Normalize(Vector3.Cross(planeTRWorld - planeTLWorld, planeBLWorld - planeTLWorld));
        // Diagram local axes from tangramDiagramRoot or world axes projected on n
        Vector3 baseRight = tangramDiagramRoot != null ? tangramDiagramRoot.right : Vector3.right;
        ex = Vector3.ProjectOnPlane(baseRight, n).normalized;
        if (ex.sqrMagnitude < 1e-6f) ex = Vector3.right;
        ez = Vector3.Cross(n, ex).normalized;
        // Compute diagram bounding box in (ex,ez) coordinates
        float minX = float.PositiveInfinity, maxX = float.NegativeInfinity;
        float minZ = float.PositiveInfinity, maxZ = float.NegativeInfinity;
        Vector3 refPoint = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
        foreach (var kv in diagramPiecesByType)
        {
            foreach (var p in kv.Value)
            {
                Vector3 c = GetPieceCenterWorld(p);
                Vector3 d = c - refPoint;
                float sx = Vector3.Dot(d, ex);
                float sz = Vector3.Dot(d, ez);
                if (sx < minX) minX = sx;
                if (sx > maxX) maxX = sx;
                if (sz < minZ) minZ = sz;
                if (sz > maxZ) maxZ = sz;
            }
        }
        widthDiag = Mathf.Max(1e-6f, maxX - minX);
        heightDiag = Mathf.Max(1e-6f, maxZ - minZ);
        originTL = refPoint + ex * minX + ez * maxZ;
    }

    private void ApplyMarkerPlaneSimilarityMapping()
    {
        // Marker plane basis
        Vector3 n = Vector3.Normalize(Vector3.Cross(planeTRWorld - planeTLWorld, planeBLWorld - planeTLWorld));
        Vector3 u = (planeTRWorld - planeTLWorld); float uLen = u.magnitude; u = uLen > 1e-6f ? u / uLen : Vector3.right;
        Vector3 v = (planeBLWorld - planeTLWorld); float vLen = v.magnitude; v = vLen > 1e-6f ? v / vLen : Vector3.up;
        Matrix4x4 R = new Matrix4x4();
        R.SetColumn(0, new Vector4(u.x, u.y, u.z, 0f));
        R.SetColumn(1, new Vector4(v.x, v.y, v.z, 0f));
        R.SetColumn(2, new Vector4(n.x, n.y, n.z, 0f));
        R.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
        Matrix4x4 RT = R.transpose;

        // Target diagram basis and bbox
        ComputeDiagramPlane(out Vector3 ex, out Vector3 ez, out Vector3 originTL, out float widthDiag, out float heightDiag);
        float markerW = Mathf.Max(1e-6f, (planeTRWorld - planeTLWorld).magnitude);
        float markerH = Mathf.Max(1e-6f, (planeBLWorld - planeTLWorld).magnitude);
        float sx = widthDiag / markerW;
        float sz = heightDiag / markerH;
        float s = 0.5f * (sx + sz); // uniform scale

        for (int i = 0; i < latestDetections.Count; i++)
        {
            var d = latestDetections[i];
            Vector3 p = d.worldPosition;
            Vector3 pProj = p - Vector3.Dot(p - planeTLWorld, n) * n;
            Vector3 local = RT.MultiplyVector(pProj - planeTLWorld);
            Vector3 mappedWorld = originTL + ex * (local.x * s) + ez * (local.y * s);
            d.worldPosition = mappedWorld;
            latestDetections[i] = d;
        }
    }

    private void ReportAllDiagramEdgeDiffs(MatchResult res, int limitPerNode)
    {
        if (diagramGraph == null || res.matchedPieceTransform == null) return;
        if (!diagramGraph.indexByTransform.TryGetValue(res.matchedPieceTransform, out int idx)) return;
        var node = diagramGraph.nodes[idx];
        int reported = 0;
        for (int k = 0; k < node.edges.Count; k++)
        {
            if (limitPerNode > 0 && reported >= limitPerNode) break;
            var e = node.edges[k];
            var neigh = diagramGraph.nodes[e.toIndex].piece;
            float actualDist = Vector3.Distance(GetPieceCenterWorld(node.piece), GetPieceCenterWorld(neigh));
            int idA_dbg2 = GetArucoIdForDiagramPiece(node.piece);
            int idB_dbg2 = GetArucoIdForDiagramPiece(neigh);
            Vector3 from_dbg2 = (idA_dbg2 >= 0 && idB_dbg2 >= 0 && idA_dbg2 > idB_dbg2) ? GetPieceCenterWorld(neigh) : GetPieceCenterWorld(node.piece);
            Vector3 to_dbg2   = (idA_dbg2 >= 0 && idB_dbg2 >= 0 && idA_dbg2 > idB_dbg2) ? GetPieceCenterWorld(node.piece) : GetPieceCenterWorld(neigh);
            float actualAng = ComputePlanarAngleDeg(from_dbg2, to_dbg2);
            float dDiff = Mathf.Abs(actualDist - e.expectedDistanceMeters);
            float aDiff = Mathf.Abs(NormalizeAngle180(actualAng - e.expectedAngleDeg));
            Debug.Log($"[TangramMatcher] edge report '{node.piece.name}' -> '{neigh.name}' diff(d)={dDiff:F3}m diff(deg)={aDiff:F1} expected(d)={e.expectedDistanceMeters:F3}m expAng={e.expectedAngleDeg:F1}");
            if (drawRelationEdgeLines)
            {
                Debug.DrawLine(GetPieceCenterWorld(node.piece), GetPieceCenterWorld(neigh), Color.cyan, 0f, false);
            }
            reported++;
        }
    }

    private void ReportDetectionRelations(MatchResult res, int limitPerNode, bool drawLines)
    {
        // Find the detection node corresponding to this match
        if (latestDetections == null || diagramGraph == null) return;
        // Locate detection index by arucoId and type
        int detIndex = -1;
        for (int i = 0; i < latestDetections.Count; i++)
        {
            var d = latestDetections[i];
            if (d.arucoId == res.arucoId && d.shapeType == res.shapeType)
            {
                detIndex = i; break;
            }
        }
        if (detIndex < 0) return;
        // Build neighborhood for logging based on DIAGRAM EDGES only (exclude non-edge relations)
        var centerPos = latestDetections[detIndex].worldPosition;
        var neighbors = new List<int>();
        Transform pAnchor = res.matchedPieceTransform;
        bool usedDiagramEdgesNeighborhood = false;
        if (diagramGraph != null && pAnchor != null && diagramGraph.indexByTransform.TryGetValue(pAnchor, out int idxAnchor))
        {
            usedDiagramEdgesNeighborhood = true;
            for (int j = 0; j < latestDetections.Count; j++)
            {
                if (j == detIndex) continue;
                if (latestDetections[j].isCorner) continue;
                // Only include detections whose matched piece is connected to the anchor by a diagram edge
                var dj = latestDetections[j];
                Transform pBEdge = FindAssignedPieceFor(dj.shapeType, dj.arucoId);
                if (pBEdge == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(pBEdge, out int idxBEdge)) continue;
                if (IsDiagramEdge(idxAnchor, idxBEdge)) neighbors.Add(j);
            }
        }
        else
        {
            // Fallback: if no diagram context, use proximity-based neighborhood
            float maxPair = GetDetectionMaxPairDistance();
            for (int j = 0; j < latestDetections.Count; j++)
            {
                if (j == detIndex) continue;
                if (latestDetections[j].isCorner) continue;
                float dist = Vector3.Distance(centerPos, latestDetections[j].worldPosition);
                float norm = maxPair > 1e-4f ? dist / maxPair : float.PositiveInfinity;
                if (norm <= detectionAdjacencyMaxNormalizedDistance) neighbors.Add(j);
            }
        }
        // Optionally limit (but do not limit when using diagram-edge neighborhood to keep all edges visible)
        if (!usedDiagramEdgesNeighborhood)
        {
            if (debugDetectionRelationsPerNode > 0 && neighbors.Count > debugDetectionRelationsPerNode)
            {
                neighbors.Sort((a, b) => Vector3.Distance(centerPos, latestDetections[a].worldPosition)
                    .CompareTo(Vector3.Distance(centerPos, latestDetections[b].worldPosition)));
                neighbors.RemoveRange(debugDetectionRelationsPerNode, neighbors.Count - debugDetectionRelationsPerNode);
            }
        }
        // Report
        // Track which diagram-neighbor pieces got logged with actual detections
        var loggedNeighborPieces = new HashSet<Transform>();
        foreach (int j in neighbors)
        {
            var A = latestDetections[detIndex];
            var B = latestDetections[j];
            if (A.isCorner || B.isCorner) continue; // safety
            // Use 2D distance on corner-projected plane
            float dActual = Vector2.Distance(A.planeCoord, B.planeCoord);
            
            // ArUco ID를 기준으로 기준(from)과 목표(to) 도형을 일관되게 결정합니다.
            var fromDet = (A.arucoId <= B.arucoId) ? A : B;
            var toDet   = (A.arucoId <= B.arucoId) ? B : A;
                        
            // 디버깅 컨텍스트 생성
            string detectionDebugContext = debugLogDetectionAngles ? 
                $"DETECTION {fromDet.shapeType}(ID:{fromDet.arucoId}) → {toDet.shapeType}(ID:{toDet.arucoId})" : "";

            // ★ 변경점: 감지된 도형들의 '실제' 관계 각도를 새로운 상대 각도 방식으로 계산합니다.
            float actualConnectionAngle = ComputeRelativeAngleDeg(
                fromDet.worldPosition,
                fromDet.worldRotation,
                toDet.worldPosition,
                detectionDebugContext
            );
            Transform pA = FindAssignedPieceFor(A.shapeType, A.arucoId);
            Transform pB = FindAssignedPieceFor(B.shapeType, B.arucoId);
            if (pA != null && pB != null)
            {
                loggedNeighborPieces.Add(pB);
                // Optional: filter to only diagram-graph edges
                if (restrictRelationsToDiagramEdges && diagramGraph != null)
                {
                    if (diagramGraph.indexByTransform.TryGetValue(pA, out int idxA) && diagramGraph.indexByTransform.TryGetValue(pB, out int idxB))
                    {
                        if (!IsDiagramEdge(idxA, idxB))
                        {
                            // Not an expected relation; skip logging this pair
                            continue;
                        }
                    }
                }
                // Special handling: angle-only logging if exactly 2 non-corner detections
                int nonCornerCount = 0;
                for (int ii = 0; ii < latestDetections.Count; ii++) if (!latestDetections[ii].isCorner) nonCornerCount++;
                bool angleOnlyForTwoDetections = (nonCornerCount == 2);

                // Calculate expected distance using 2D projection of diagram pieces
                Vector2 pACoord = ProjectTo2DCornerPlane(GetPieceCenterWorld(pA));
                Vector2 pBCoord = ProjectTo2DCornerPlane(GetPieceCenterWorld(pB));
                float dExp = Vector2.Distance(pACoord, pBCoord);
                
                // 감지된 도형과 매칭되는 정답 조각들을 가져옵니다.
                var fromPiece = (A.arucoId <= B.arucoId) ? pA : pB;
                var toPiece   = (A.arucoId <= B.arucoId) ? pB : pA;
                
                // 디버깅 컨텍스트 생성
                string diagramDebugContext = debugLogDetectionAngles ? 
                    $"EXPECTED {fromPiece.name} → {toPiece.name}" : "";

                // ★ 변경점: 정답 조각들의 '기대' 관계 각도를 새로운 상대 각도 방식으로 계산합니다.
                // Use the same method as diagram building for consistency
                float expectedConnectionAngle = ComputeRelativeAngleDegForDiagram(
                    fromPiece,
                    toPiece,
                    diagramDebugContext
                );

                // 각도 비교 결과 로그
                if (debugLogAngleComparison)
                {
                    float angleDifference = Mathf.Abs(NormalizeAngle180(actualConnectionAngle - expectedConnectionAngle));
                    // Debug output in English for angle comparison between detected shapes and diagram pieces
                    Debug.Log($"[AngleComparison] {fromDet.shapeType}:{fromDet.arucoId} → {toDet.shapeType}:{toDet.arucoId}\n" +
                              $"  Actual angle: {actualConnectionAngle:F2}° (between detected shapes)\n" +
                              $"  Expected angle: {expectedConnectionAngle:F2}° (between diagram pieces)\n" +
                              $"  Angle difference: {angleDifference:F2}° (tolerance: {relationMaxAngleDiffDeg:F1}°)\n" +
                              $"  Result: {(angleDifference <= relationMaxAngleDiffDeg ? "PASS" : "FAIL")}");
                }
                // Apply first-pass direction offset if available
                if (useFirstPassDirectionOffset && firstPassDirOffsetValid)
                    expectedConnectionAngle = NormalizeAngle180(expectedConnectionAngle + firstPassDirOffsetDeg);
                float connectionAngleDiff = Mathf.Abs(NormalizeAngle180(actualConnectionAngle - expectedConnectionAngle));
                // Orientation error per-shape (detection vs matched piece)
                float oriA = (pA != null) ? ComputeAngleError(A.shapeType, A.worldRotation, pA) : 0f;
                float oriB = (pB != null) ? ComputeAngleError(B.shapeType, B.worldRotation, pB) : 0f;
                string diagAName = pA != null ? pA.name : "null";
                string diagBName = pB != null ? pB.name : "null";
                if (angleOnlyForTwoDetections)
                {
                    bool passConnectionAngle = connectionAngleDiff <= relationMaxAngleDiffDeg;
                    bool passOri = !useAbsoluteOrientation || (oriA <= absoluteOrientationToleranceDeg && oriB <= absoluteOrientationToleranceDeg);
                    // Distance is always true, pass if both connection angle AND orientation passes
                    bool pass = passConnectionAngle && passOri;
                    if (useFirstPassRelativeScale && pass && !firstPassScaleValid)
                    {
                        TrySetFirstPassScale(dActual, dExp);
                    }
                    // Lock first-pass direction offset when requested and not yet set: set offset so expected+offset equals actual
                    if (useFirstPassDirectionOffset && pass && !firstPassDirOffsetValid)
                    {
                        // Use the same angle calculation method as diagram building for consistency
                        // Instead of ComputeConnectionLineAngleFromCoords (legacy method), use the new method
                        float baseExp = (A.arucoId <= B.arucoId)
                            ? ComputeRelativeAngleDegForDiagram(pA, pB, "DirectionOffsetCalc")
                            : ComputeRelativeAngleDegForDiagram(pB, pA, "DirectionOffsetCalc");
                        firstPassDirOffsetDeg = NormalizeAngle180(actualConnectionAngle - baseExp);
                        firstPassDirOffsetValid = true;
                        firstPassDirOffsetFrame = frameCounter;
                    }
                    relLogQueue.Add($"[TangramMatcher] REL det({A.shapeType}:{A.arucoId})->det({B.shapeType}:{B.arucoId}) | diag({diagAName})->diag({diagBName}) | (distance ignored: 2-detection) | connectionAngle={actualConnectionAngle:F1}° exp={expectedConnectionAngle:F1}° Δangle={connectionAngleDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | ORI_A={oriA:F1}° ORI_B={oriB:F1}° (tol {absoluteOrientationToleranceDeg:F1}) | A2D=({A.planeCoord.x:F2},{A.planeCoord.y:F2}) B2D=({B.planeCoord.x:F2},{B.planeCoord.y:F2}) | {(pass ? "PASS" : "FAIL")}");
                }
                else
                {
                    float relErr = 0f, distCost = 0f;
                    bool passDist;
                    if (useFirstPassRelativeScale)
                    {
                        passDist = CheckDistanceWithFirstPassScale(dActual, dExp, out relErr, out distCost);
                    }
                    else if (useNormalizedRelationDistance)
                    {
                        float rAct, rExp;
                        if (useSubsetUnifiedBaseline && cachedBaselineValid)
                        {
                            rAct = dActual / Mathf.Max(1e-4f, cachedBaselineDetRef);
                            rExp = dExp / Mathf.Max(1e-4f, cachedBaselineExpRef);
                        }
                        else if (useSubsetUnifiedBaseline)
                        {
                            float baseDet = GetDetectionMaxPairDistance();
                            float baseExp = GetDiagramMaxEdgeLength();
                            float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                            rAct = dActual / baseline;
                            rExp = dExp / baseline;
                        }
                        else
                        {
                            float baseDet = GetDetectionMaxPairDistance();
                            float baseExp = GetDiagramMaxEdgeLength();
                            float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                            rAct = dActual / baseline;
                            rExp = dExp / baseline;
                        }
                        float dDiff = Mathf.Abs(rAct - rExp);
                        passDist = dDiff <= relationMaxDistDiff;
                    }
                    else
                    {
                        float dDiff = Mathf.Abs(dActual - dExp);
                        passDist = dDiff <= relationMaxDistDiff;
                    }

                    bool passConnectionAngle = (ignoreConnectionAngleUntilOffsetLocked && useFirstPassDirectionOffset && !firstPassDirOffsetValid)
                        ? true
                        : (connectionAngleDiff <= relationMaxAngleDiffDeg);
                    bool passOri = !useAbsoluteOrientation || (oriA <= absoluteOrientationToleranceDeg && oriB <= absoluteOrientationToleranceDeg);
                    // First relation: require dist AND angle AND ori to all pass
                    bool pass = passDist && passConnectionAngle && passOri;

                    // Dynamic tolerance update on successful relation
                    if (pass)
                    {
                        float newTolerance = dActual * 1.25f;
                        if (newTolerance > _dynamicGraphDistanceTolerance)
                            _dynamicGraphDistanceTolerance = newTolerance;
                    }

                    if (useFirstPassRelativeScale)
                    {
                        if (pass && !firstPassScaleValid)
                        {
                            TrySetFirstPassScale(dActual, dExp);
                        }
                        // If angle gating was ignored and overall pass achieved, lock direction offset now
                        if (useFirstPassDirectionOffset && !firstPassDirOffsetValid && pass)
                        {
                            // Use the same angle calculation method as diagram building for consistency
                            float baseExp2 = (A.arucoId <= B.arucoId)
                                ? ComputeRelativeAngleDegForDiagram(pA, pB, "DirectionOffsetCalc2")
                                : ComputeRelativeAngleDegForDiagram(pB, pA, "DirectionOffsetCalc2");
                            firstPassDirOffsetDeg = NormalizeAngle180(actualConnectionAngle - baseExp2);
                            firstPassDirOffsetValid = true;
                            firstPassDirOffsetFrame = frameCounter;
                        }
                        relLogQueue.Add($"[TangramMatcher] REL det({A.shapeType}:{A.arucoId})->det({B.shapeType}:{B.arucoId}) | diag({diagAName})->diag({diagBName}) | relErr={(firstPassScaleValid ? relErr : float.NaN):F3} tol±{(firstPassScaleTolerance * 100f):F0}% s={(firstPassScaleValid ? firstPassScale : float.NaN):F3} | Dist={(passDist?"PASS":"FAIL")} Angle={(passConnectionAngle?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={actualConnectionAngle:F1}° exp={expectedConnectionAngle:F1}° Δangle={connectionAngleDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | ORI_A={oriA:F1}° ORI_B={oriB:F1}° (tol {absoluteOrientationToleranceDeg:F1}) | A2D=({A.planeCoord.x:F2},{A.planeCoord.y:F2}) B2D=({B.planeCoord.x:F2},{B.planeCoord.y:F2}) | {(pass ? "PASS" : "FAIL")}");
                    }
                    else if (useNormalizedRelationDistance)
                    {
                        float baseDet = GetDetectionMaxPairDistance();
                        float baseExp = GetDiagramMaxEdgeLength();
                        float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                        float rAct = dActual / baseline;
                        float rExp = dExp / baseline;
                        float dDiff = Mathf.Abs(rAct - rExp);
                        relLogQueue.Add($"[TangramMatcher] REL det({A.shapeType}:{A.arucoId})->det({B.shapeType}:{B.arucoId}) | diag({diagAName})->diag({diagBName}) | r={rAct:F3} expR={rExp:F3} Δr={dDiff:F3} (tol {relationMaxDistDiff:F3}) | Dist={(passDist?"PASS":"FAIL")} Angle={(passConnectionAngle?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={actualConnectionAngle:F1}° exp={expectedConnectionAngle:F1}° Δangle={connectionAngleDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | ORI_A={oriA:F1}° ORI_B={oriB:F1}° (tol {absoluteOrientationToleranceDeg:F1}) | A2D=({A.planeCoord.x:F2},{A.planeCoord.y:F2}) B2D=({B.planeCoord.x:F2},{B.planeCoord.y:F2}) | {(pass ? "PASS" : "FAIL")}");
                    }
                    else
                    {
                        float dDiff = Mathf.Abs(dActual - dExp);
                        relLogQueue.Add($"[TangramMatcher] REL det({A.shapeType}:{A.arucoId})->det({B.shapeType}:{B.arucoId}) | diag({diagAName})->diag({diagBName}) | d={dActual:F3}m exp={dExp:F3}m Δd={dDiff:F3} (tol {relationMaxDistDiff:F3}) | Dist={(passDist?"PASS":"FAIL")} Angle={(passConnectionAngle?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={actualConnectionAngle:F1}° exp={expectedConnectionAngle:F1}° Δangle={connectionAngleDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | ORI_A={oriA:F1}° ORI_B={oriB:F1}° (tol {absoluteOrientationToleranceDeg:F1}) | A2D=({A.planeCoord.x:F2},{A.planeCoord.y:F2}) B2D=({B.planeCoord.x:F2},{B.planeCoord.y:F2}) | {(pass ? "PASS" : "FAIL")}");
                    }
                }
            }
            else
            {
                if (!suppressNoExpectedRelationLogs)
                {
                    // Even if no expected relation, report ORI per shape for debugging
                    float oriA = (pA != null) ? ComputeAngleError(A.shapeType, A.worldRotation, pA) : 0f;
                    float oriB = (pB != null) ? ComputeAngleError(B.shapeType, B.worldRotation, pB) : 0f;
                    string diagAName = pA != null ? pA.name : "none";
                    string diagBName = pB != null ? pB.name : "none";
                    relLogQueue.Add($"[TangramMatcher] REL det({A.shapeType}:{A.arucoId})->det({B.shapeType}:{B.arucoId}) | diag({diagAName})->diag({diagBName}) | d={dActual:F3}m | connectionAngle={actualConnectionAngle:F1}° | ORI_A={oriA:F1}° ORI_B={oriB:F1}° | A2D=({A.planeCoord.x:F2},{A.planeCoord.y:F2}) B2D=({B.planeCoord.x:F2},{B.planeCoord.y:F2}) | (no expected)");
                }
            }
            if (drawLines)
            {
                Debug.DrawLine(A.worldPosition, B.worldPosition, Color.yellow, 0f, false);
            }
        }

        // Also log diagram-edge neighbors that are missing an actual detection (to avoid gaps)
        if (diagramGraph != null && res.matchedPieceTransform != null && diagramGraph.indexByTransform.TryGetValue(res.matchedPieceTransform, out int idxC))
        {
            var centerPiece = res.matchedPieceTransform;
            for (int e = 0; e < diagramGraph.nodes[idxC].edges.Count; e++)
            {
                var ed = diagramGraph.nodes[idxC].edges[e];
                var neighPiece = diagramGraph.nodes[ed.toIndex].piece;
                if (loggedNeighborPieces.Contains(neighPiece)) continue; // already logged with actual detection

                // Try proxy: use any matched detection of the neighbor's type, even if it was matched to a different diagram piece
                bool proxied = false;
                if (allowBestFitEdgeForRelation)
                {
                    // Determine neighbor piece type
                    if (TryGetPieceType(neighPiece, out TangramShapeType neighType))
                    {
                        // Find best-fit matched detection of this type
                        MatchResult? best = null;
                        float bestCost = float.PositiveInfinity;
                        // Determine frame mode (angle-only for two detections)
                        int nonCornerCount = 0; for (int ii = 0; ii < latestDetections.Count; ii++) if (!latestDetections[ii].isCorner) nonCornerCount++;
                        bool angleOnlyForTwoDetections = (nonCornerCount == 2);
                        // Expected geometry for this edge
                        float dExpEdge = ed.expectedDistanceMeters;
                        // Calculate expected angle using 2D projected coordinates
                        Vector2 centerCoord = ProjectTo2DCornerPlane(GetPieceCenterWorld(centerPiece));
                        Vector2 neighCoord = ProjectTo2DCornerPlane(GetPieceCenterWorld(neighPiece));
                        // Enforce low-ID -> high-ID for expected proxy angle using diagram piece IDs
                        int idCenter = GetArucoIdForDiagramPiece(centerPiece);
                        int idNeigh = GetArucoIdForDiagramPiece(neighPiece);
                        float aExpEdge = (idCenter <= idNeigh)
                            ? ComputeConnectionLineAngleFromCoords(centerCoord, neighCoord)
                            : ComputeConnectionLineAngleFromCoords(neighCoord, centerCoord);
                        // Baseline for normalized distances
                        float baseDet = GetDetectionMaxPairDistance();
                        float baseExp = GetDiagramMaxEdgeLength();
                        float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                        // Find anchor detection position
                        Vector3 planeN2 = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                        Vector3 anchorDetPos = res.detectionWorldPosition;
                        foreach (var mr in lastMatchResults)
                        {
                            if (mr.shapeType != neighType) continue;
                            // Skip if this is the same detection (same ArUco ID) - prevents self-comparison
                            if (mr.arucoId == res.arucoId) continue;
                            // Use 2D distance for proxy relation using corner-projected coordinates
                            var detA2_dist = FindDetection(res.shapeType, res.arucoId);
                            var detB2_dist = FindDetection(mr.shapeType, mr.arucoId);
                            float dAct = (detA2_dist.HasValue && detB2_dist.HasValue) ? 
                                Vector2.Distance(detA2_dist.Value.planeCoord, detB2_dist.Value.planeCoord) :
                                Vector3.Distance(anchorDetPos, mr.detectionWorldPosition); // fallback to 3D if no plane coords
                            // Use 4-decimal planar coords when available to compute relation angle
                            float aAct;
                            var detA2 = FindDetection(res.shapeType, res.arucoId);
                            var detB2 = FindDetection(mr.shapeType, mr.arucoId);
                            if (detA2.HasValue && detB2.HasValue)
                                aAct = (detA2.Value.arucoId <= detB2.Value.arucoId)
                                    ? ComputeConnectionLineAngleFromCoords(detA2.Value.planeCoord, detB2.Value.planeCoord)
                                    : ComputeConnectionLineAngleFromCoords(detB2.Value.planeCoord, detA2.Value.planeCoord);
                            else
                                aAct = ComputeConnectionLineAngle(anchorDetPos, mr.detectionWorldPosition);
                            float dDiff = 0f;
                            if (!angleOnlyForTwoDetections)
                            {
                                if (useFirstPassRelativeScale)
                                {
                                    float relErrP, distCostP;
                                    CheckDistanceWithFirstPassScale(dAct, dExpEdge, out relErrP, out distCostP);
                                    dDiff = distCostP;
                                }
                                else if (useNormalizedRelationDistance)
                                {
                                    float rAct = dAct / baseline;
                                    float rExp = dExpEdge / baseline;
                                    dDiff = Mathf.Abs(rAct - rExp);
                                }
                                else
                                {
                                    dDiff = Mathf.Abs(dAct - dExpEdge);
                                }
                            }
                            float aDiff = Mathf.Abs(NormalizeAngle180(aAct - aExpEdge));
                            float cost = (angleOnlyForTwoDetections ? 0f : dDiff) + rotationWeightMetersPerDeg * aDiff;
                            if (useAbsoluteOrientation)
                            {
                                // include ORI soft cost
                                var detB = FindDetection(mr.shapeType, mr.arucoId);
                                if (detB.HasValue)
                                {
                                    float oriB = ComputeAngleError(mr.shapeType, detB.Value.worldRotation, mr.matchedPieceTransform);
                                    cost += absoluteOrientationWeightMetersPerDeg * oriB;
                                }
                            }
                            if (cost < bestCost)
                            {
                                bestCost = cost;
                                best = mr;
                            }
                        }
                        if (best.HasValue)
                        {
                            // Emit proxy REL line
                            var mr = best.Value;
                            // Use 2D distance for final proxy relation logging as well
                            var detA3_final = FindDetection(res.shapeType, res.arucoId);
                            var detB3_final = FindDetection(mr.shapeType, mr.arucoId);
                            float dAct = (detA3_final.HasValue && detB3_final.HasValue) ? 
                                Vector2.Distance(detA3_final.Value.planeCoord, detB3_final.Value.planeCoord) :
                                Vector3.Distance(anchorDetPos, mr.detectionWorldPosition); // fallback to 3D if no plane coords
                            float aAct;
                            var detA3 = FindDetection(res.shapeType, res.arucoId);
                            var detB3 = FindDetection(mr.shapeType, mr.arucoId);
                            if (detA3.HasValue && detB3.HasValue)
                                aAct = (detA3.Value.arucoId <= detB3.Value.arucoId)
                                    ? ComputeConnectionLineAngleFromCoords(detA3.Value.planeCoord, detB3.Value.planeCoord)
                                    : ComputeConnectionLineAngleFromCoords(detB3.Value.planeCoord, detA3.Value.planeCoord);
                            else
                                aAct = ComputeConnectionLineAngle(anchorDetPos, mr.detectionWorldPosition);
                            float dDiff; float rAct=0f, rExp=0f;
                            bool angleOnly = (nonCornerCount == 2);
                            if (useFirstPassRelativeScale)
                            {
                                float relErrPx, distCostPx;
                                CheckDistanceWithFirstPassScale(dAct, dExpEdge, out relErrPx, out distCostPx);
                                dDiff = distCostPx;
                            }
                            else if (useNormalizedRelationDistance)
                            {
                                rAct = dAct / baseline;
                                rExp = dExpEdge / baseline;
                                dDiff = Mathf.Abs(rAct - rExp);
                            }
                            else
                            {
                                dDiff = Mathf.Abs(dAct - dExpEdge);
                            }
                            float aDiff = Mathf.Abs(NormalizeAngle180(aAct - aExpEdge));
                            bool passDist = angleOnly ? true : (dDiff <= relationMaxDistDiff); // Distance must pass first (except angle-only mode)
                            bool passAng = aDiff <= relationMaxAngleDiffDeg;
                            bool passOri = true;
                            if (useAbsoluteOrientation)
                            {
                                var detA = FindDetection(res.shapeType, res.arucoId);
                                var detB = FindDetection(mr.shapeType, mr.arucoId);
                                float oriA = detA.HasValue ? ComputeAngleError(res.shapeType, detA.Value.worldRotation, centerPiece) : 0f;
                                float oriB = detB.HasValue ? ComputeAngleError(mr.shapeType, detB.Value.worldRotation, mr.matchedPieceTransform) : 0f;
                                passOri = (oriA <= absoluteOrientationToleranceDeg && oriB <= absoluteOrientationToleranceDeg);
                            }
                            string diagAName = centerPiece != null ? centerPiece.name : "null";
                            string diagBName = neighPiece != null ? neighPiece.name : "null";
                            // Compute planar coordinates for logging
                            float a2dx = 0f, a2dy = 0f, b2dx = 0f, b2dy = 0f;
                            {
                                var dA = FindDetection(res.shapeType, res.arucoId);
                                if (dA.HasValue)
                                {
                                    a2dx = dA.Value.planeCoord.x; a2dy = dA.Value.planeCoord.y;
                                }
                                else
                                {
                                    Vector3 pn = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                    Vector3 po = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
                                    Vector3 u = Vector3.Cross(pn, Vector3.up); if (u.sqrMagnitude < 1e-6f) u = Vector3.Cross(pn, Vector3.right); u.Normalize();
                                    Vector3 vaxis = Vector3.Cross(pn, u).normalized;
                                    Vector3 proj = res.detectionWorldPosition - Vector3.Dot(res.detectionWorldPosition - po, pn) * pn;
                                    Vector3 rel = proj - po; a2dx = Vector3.Dot(rel, u); a2dy = Vector3.Dot(rel, vaxis);
                                }
                                var dB = FindDetection(mr.shapeType, mr.arucoId);
                                if (dB.HasValue)
                                {
                                    b2dx = dB.Value.planeCoord.x; b2dy = dB.Value.planeCoord.y;
                                }
                                else
                                {
                                    Vector3 pn = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                                    Vector3 po = tangramDiagramRoot != null ? tangramDiagramRoot.position : Vector3.zero;
                                    Vector3 u = Vector3.Cross(pn, Vector3.up); if (u.sqrMagnitude < 1e-6f) u = Vector3.Cross(pn, Vector3.right); u.Normalize();
                                    Vector3 vaxis = Vector3.Cross(pn, u).normalized;
                                    Vector3 proj = mr.detectionWorldPosition - Vector3.Dot(mr.detectionWorldPosition - po, pn) * pn;
                                    Vector3 rel = proj - po; b2dx = Vector3.Dot(rel, u); b2dy = Vector3.Dot(rel, vaxis);
                                }
                            }
                            // Pass if distance passes AND connection angle AND orientation passes
                            bool proxyPass = passDist && passAng && passOri;
                            if (proxyPass)
                            {
                                float newToleranceP = dAct * 1.25f;
                                if (newToleranceP > _dynamicGraphDistanceTolerance)
                                    _dynamicGraphDistanceTolerance = newToleranceP;
                            }
                            if (useFirstPassRelativeScale)
                            {
                                // Compute ORI for logging regardless of gating flag
                                float logOriA = 0f, logOriB = 0f;
                                var detAL = FindDetection(res.shapeType, res.arucoId);
                                var detBL = FindDetection(mr.shapeType, mr.arucoId);
                                if (detAL.HasValue) logOriA = ComputeAngleError(res.shapeType, detAL.Value.worldRotation, centerPiece);
                                if (detBL.HasValue) logOriB = ComputeAngleError(mr.shapeType, detBL.Value.worldRotation, mr.matchedPieceTransform);
                                relLogQueue.Add($"[TangramMatcher] REL det({res.shapeType}:{res.arucoId})->det({mr.shapeType}:{mr.arucoId}) | diag({diagAName})->diag({diagBName}) (proxy) | relErr={(firstPassScaleValid ? (Mathf.Abs(dAct - Mathf.Max(1e-4f, firstPassScale * dExpEdge)) / Mathf.Max(1e-4f, firstPassScale * dExpEdge)) : float.NaN):F3} tol±{firstPassScaleTolerance * 100f:F0}% s={(firstPassScaleValid ? firstPassScale : float.NaN):F3} | Dist={(passDist?"PASS":"FAIL")} Angle={(passAng?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={aAct:F1}° exp={aExpEdge:F1}° Δangle={aDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | ORI_A={logOriA:F1}° ORI_B={logOriB:F1}° (tol {absoluteOrientationToleranceDeg:F1}) | A2D=({a2dx:F2},{a2dy:F2}) B2D=({b2dx:F2},{b2dy:F2}) | {(proxyPass ? "PASS" : "FAIL")}");
                            }
                            else if (useNormalizedRelationDistance)
                            {
                                relLogQueue.Add($"[TangramMatcher] REL det({res.shapeType}:{res.arucoId})->det({mr.shapeType}:{mr.arucoId}) | diag({diagAName})->diag({diagBName}) (proxy) | r={rAct:F3} expR={rExp:F3} Δr={(Mathf.Abs(rAct-rExp)):F3} (tol {relationMaxDistDiff:F3}) | Dist={(passDist?"PASS":"FAIL")} Angle={(passAng?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={aAct:F1}° exp={aExpEdge:F1}° Δangle={aDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | A2D=({a2dx:F2},{a2dy:F2}) B2D=({b2dx:F2},{b2dy:F2}) | {(proxyPass ? "PASS" : "FAIL")}");
                            }
                            else
                            {
                                relLogQueue.Add($"[TangramMatcher] REL det({res.shapeType}:{res.arucoId})->det({mr.shapeType}:{mr.arucoId}) | diag({diagAName})->diag({diagBName}) (proxy) | d={dAct:F3}m exp={dExpEdge:F3}m Δd={dDiff:F3} (tol {relationMaxDistDiff:F3}) | Dist={(passDist?"PASS":"FAIL")} Angle={(passAng?"PASS":"FAIL")} Ori={(passOri?"PASS":"FAIL")} | connectionAngle={aAct:F1}° exp={aExpEdge:F1}° Δangle={aDiff:F1}° (tol {relationMaxAngleDiffDeg:F1}) | A2D=({a2dx:F2},{a2dy:F2}) B2D=({b2dx:F2},{b2dy:F2}) | {(proxyPass ? "PASS" : "FAIL")}");
                            }
                            proxied = true;
                        }
                    }
                }
                if (!proxied)
                {
                    // Check if this is 2-detection mode before logging missing neighbor
                    int nonCornerCount = 0;
                    for (int ii = 0; ii < latestDetections.Count; ii++) if (!latestDetections[ii].isCorner) nonCornerCount++;
                    bool angleOnlyForTwoDetections = (nonCornerCount == 2);
                    
                    if (!angleOnlyForTwoDetections)
                    {
                        // Build an informational line using expected values only
                        // Calculate expected values using 2D projection for missing neighbor
                        Vector2 centerCoord_missing = ProjectTo2DCornerPlane(GetPieceCenterWorld(centerPiece));
                        Vector2 neighCoord_missing = ProjectTo2DCornerPlane(GetPieceCenterWorld(neighPiece));
                        float dExp = Vector2.Distance(centerCoord_missing, neighCoord_missing);
                        float aExp = (GetArucoIdForDiagramPiece(centerPiece) <= GetArucoIdForDiagramPiece(neighPiece))
                            ? ComputeConnectionLineAngleFromCoords(centerCoord_missing, neighCoord_missing)
                            : ComputeConnectionLineAngleFromCoords(neighCoord_missing, centerCoord_missing);
                        string diagAName = centerPiece != null ? centerPiece.name : "null";
                        string diagBName = neighPiece != null ? neighPiece.name : "null";
                        relLogQueue.Add($"[TangramMatcher] REL diag({diagAName})->diag({diagBName}) | (neighbor missing) | exp d={dExp:F3}m exp deg={aExp:F1}");
                    }
                }
            }
        }
    }

    // Evaluate whether the given piece has at least one (or all) passing relations to matched neighbors along diagram edges this frame
    private bool EvaluateRelationPassForPiece(Transform piece, bool requireAll, out string reason)
    {
        reason = "";
        try
        {
            if (diagramGraph == null || piece == null) { reason = "no-diagram"; return false; }
            if (!diagramGraph.indexByTransform.TryGetValue(piece, out int idx)) { reason = "no-index"; return false; }
            // Find this piece's matched detection result
            MatchResult? mr = null;
            for (int i = 0; i < lastMatchResults.Count; i++) if (lastMatchResults[i].matchedPieceTransform == piece) { mr = lastMatchResults[i]; break; }
            if (!mr.HasValue) { reason = "no-match"; return false; }
            var res = mr.Value;
            // Gather neighbors by diagram edges and check logs directly by recomputing tolerances
            bool anyPass = false;
            bool allPass = true;
            Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
            // Special handling: angle-only logging if exactly 2 non-corner detections
            int nonCornerCount = 0;
            for (int ii = 0; ii < latestDetections.Count; ii++) if (!latestDetections[ii].isCorner) nonCornerCount++;
            bool angleOnlyForTwoDetections = (nonCornerCount == 2);
            
            // New logic: track total neighbors and pass count for detailed evaluation
            int totalNeighbors = 0;
            int passCount = 0;
            var perNeighborLines = new List<string>();
            
            for (int e = 0; e < diagramGraph.nodes[idx].edges.Count; e++)
            {
                var ed = diagramGraph.nodes[idx].edges[e];
                var neighPiece = diagramGraph.nodes[ed.toIndex].piece;
                // Find match result for neighbor
                MatchResult? nb = null;
                for (int i = 0; i < lastMatchResults.Count; i++) if (lastMatchResults[i].matchedPieceTransform == neighPiece) { nb = lastMatchResults[i]; break; }
                if (!nb.HasValue)
                {
                    allPass = false;
                    if (allowCrossMatchedNeighborForRelationPass)
                    {
                        // Try proxy: find any matched detection whose piece is an edge neighbor of 'piece'
                        MatchResult? proxy = null;
                        for (int k = 0; k < lastMatchResults.Count; k++)
                        {
                            var cand = lastMatchResults[k];
                            if (cand.matchedPieceTransform == null) continue;
                            if (!diagramGraph.indexByTransform.TryGetValue(cand.matchedPieceTransform, out int idxCand)) continue;
                            if (!IsDiagramEdge(idx, idxCand)) continue;
                            proxy = cand; break;
                        }
                        if (proxy.HasValue)
                        {
                            var nbv = proxy.Value;
                            float dActP = Vector3.ProjectOnPlane(nbv.detectionWorldPosition - res.detectionWorldPosition, planeN).magnitude;
                            float dExpP = GetDiagramExpectedDistanceBetweenPieces(piece, nbv.matchedPieceTransform);
                            float aActP = ComputePlanarAngleDeg(res.detectionWorldPosition, nbv.detectionWorldPosition);
                            float aExpP = ComputePlanarAngleDeg(GetPieceCenterWorld(piece), GetPieceCenterWorld(nbv.matchedPieceTransform));
                            float dDiffP = 0f; bool passDistP = true;
                            if (!angleOnlyForTwoDetections)
                            {
                                if (useFirstPassRelativeScale)
                                {
                                    float relErrP2, distCostP2;
                                    CheckDistanceWithFirstPassScale(dActP, dExpP, out relErrP2, out distCostP2);
                                    dDiffP = distCostP2;
                                    passDistP = distCostP2 <= 0f;
                                }
                                else if (useNormalizedRelationDistance)
                                {
                                    float baseDet = GetDetectionMaxPairDistance();
                                    float baseExp = GetDiagramMaxEdgeLength();
                                    float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                                    float rAct = dActP / baseline;
                                    float rExp = dExpP / baseline;
                                    dDiffP = Mathf.Abs(rAct - rExp);
                                    passDistP = dDiffP <= relationMaxDistDiff;
                                }
                                else
                                {
                                    dDiffP = Mathf.Abs(dActP - dExpP);
                                    passDistP = dDiffP <= relationMaxDistDiff;
                                }
                            }
                            float aDiffP = Mathf.Abs(NormalizeAngle180(aActP - aExpP));
                            bool passAngP = (aDiffP <= relationMaxAngleDiffDeg);
                            bool passOriP = true;
                            if (useAbsoluteOrientation)
                            {
                                var detA = FindDetection(res.shapeType, res.arucoId);
                                var detB = FindDetection(nbv.shapeType, nbv.arucoId);
                                float oriA = detA.HasValue ? ComputeAngleError(res.shapeType, detA.Value.worldRotation, piece) : 0f;
                                float oriB = detB.HasValue ? ComputeAngleError(nbv.shapeType, detB.Value.worldRotation, nbv.matchedPieceTransform) : 0f;
                                passOriP = (oriA <= absoluteOrientationToleranceDeg && oriB <= absoluteOrientationToleranceDeg);
                            }
                            bool passP = (angleOnlyForTwoDetections ? passAngP && passOriP : passDistP && passAngP && passOriP);
                            anyPass |= passP;
                            
                            // Track for detailed logging
                            totalNeighbors++;
                            if (passP) passCount++;
                            
                            // Add detailed neighbor result to list
                            string proxyNeighborName = nbv.matchedPieceTransform != null ? nbv.matchedPieceTransform.name : "proxy";
                            float proxyAbsOri = 0f;
                            if (useAbsoluteOrientation)
                            {
                                // For proxy, we need to find the actual detection objects
                                var proxyDetA = FindDetection(res.shapeType, res.arucoId);
                                var proxyDetB = FindDetection(nbv.shapeType, nbv.arucoId);
                                float proxyOriA = proxyDetA.HasValue ? ComputeAngleError(res.shapeType, proxyDetA.Value.worldRotation, piece) : 0f;
                                float proxyOriB = proxyDetB.HasValue ? ComputeAngleError(nbv.shapeType, proxyDetB.Value.worldRotation, nbv.matchedPieceTransform) : 0f;
                                proxyAbsOri = Mathf.Max(proxyOriA, proxyOriB);
                            }
                            perNeighborLines.Add($"{proxyNeighborName}: RelDist={dDiffP:F3}m({(passDistP?"PASS":"FAIL")}) " +
                                               $"RelAngle={aDiffP:F1}°({(passAngP?"PASS":"FAIL")}) " +
                                               $"AbsOri={proxyAbsOri:F1}°({(passOriP?"PASS":"FAIL")}) " +
                                               $"AbsDist={dActP:F3}m");
                        }
                    }
                    continue;
                }
                // Compute actual planar distance/angle
                float dAct = Vector3.ProjectOnPlane(nb.Value.detectionWorldPosition - res.detectionWorldPosition, planeN).magnitude;
                float dExp = ed.expectedDistanceMeters;
                float aAct;
                var detA4 = FindDetection(res.shapeType, res.arucoId);
                var detB4 = FindDetection(nb.Value.shapeType, nb.Value.arucoId);
                if (detA4.HasValue && detB4.HasValue)
                    aAct = (detA4.Value.arucoId <= detB4.Value.arucoId)
                        ? ComputePlanarAngleFromPlaneCoords(detA4.Value.planeCoord, detB4.Value.planeCoord, 4)
                        : ComputePlanarAngleFromPlaneCoords(detB4.Value.planeCoord, detA4.Value.planeCoord, 4);
                else
                    aAct = ComputePlanarAngleDeg(res.detectionWorldPosition, nb.Value.detectionWorldPosition);
                float aExp = ed.expectedAngleDeg;
                float dDiff = 0f;
                bool passDist = true;
                if (!angleOnlyForTwoDetections)
                {
                    if (useFirstPassRelativeScale)
                    {
                        float relErr2, distCost2;
                        passDist = CheckDistanceWithFirstPassScale(dAct, dExp, out relErr2, out distCost2);
                        dDiff = distCost2;
                    }
                    else if (useNormalizedRelationDistance)
                    {
                        float baseDet = GetDetectionMaxPairDistance();
                        float baseExp = GetDiagramMaxEdgeLength();
                        float baseline = Mathf.Max(1e-4f, baseDet, baseExp);
                        float rAct = dAct / baseline;
                        float rExp = dExp / baseline;
                        dDiff = Mathf.Abs(rAct - rExp);
                        passDist = dDiff <= relationMaxDistDiff;
                    }
                    else
                    {
                        dDiff = Mathf.Abs(dAct - dExp);
                        passDist = dDiff <= relationMaxDistDiff;
                    }
                }
                float aDiff = Mathf.Abs(NormalizeAngle180(aAct - aExp));
                bool passAng = (aDiff <= relationMaxAngleDiffDeg);
                bool passOri = true;
                if (useAbsoluteOrientation)
                {
                    var detA = FindDetection(res.shapeType, res.arucoId);
                    var detB = FindDetection(nb.Value.shapeType, nb.Value.arucoId);
                    float oriA = detA.HasValue ? ComputeAngleError(res.shapeType, detA.Value.worldRotation, piece) : 0f;
                    float oriB = detB.HasValue ? ComputeAngleError(nb.Value.shapeType, detB.Value.worldRotation, neighPiece) : 0f;
                    passOri = (oriA <= absoluteOrientationToleranceDeg && oriB <= absoluteOrientationToleranceDeg);
                }
                // Pass if distance passes AND connection angle AND orientation passes
                bool pass = (angleOnlyForTwoDetections ? (passAng && passOri) : (passDist && passAng && passOri));
                anyPass |= pass;
                allPass &= pass;
                
                // Track for detailed logging
                totalNeighbors++;
                if (pass) passCount++;
                
                // Add detailed neighbor result to list
                string neighborName = neighPiece != null ? neighPiece.name : "unknown";
                float absDist = dAct;
                float absOri = 0f;
                if (useAbsoluteOrientation)
                {
                    var detA = FindDetection(res.shapeType, res.arucoId);
                    var detB = FindDetection(nb.Value.shapeType, nb.Value.arucoId);
                    float oriA = detA.HasValue ? ComputeAngleError(res.shapeType, detA.Value.worldRotation, piece) : 0f;
                    float oriB = detB.HasValue ? ComputeAngleError(nb.Value.shapeType, detB.Value.worldRotation, neighPiece) : 0f;
                    absOri = Mathf.Max(oriA, oriB);
                }
                perNeighborLines.Add($"{neighborName}: RelDist={dDiff:F3}m({(passDist?"PASS":"FAIL")}) " +
                                   $"RelAngle={aDiff:F1}°({(passAng?"PASS":"FAIL")}) " +
                                   $"AbsOri={absOri:F1}°({(passOri?"PASS":"FAIL")}) " +
                                   $"AbsDist={absDist:F3}m");
            }
            
            // New evaluation logic: minimum neighbor count requirement instead of requireAll
            bool ok;
            if (totalNeighbors == 0)
            {
                ok = false;
                reason = "no-neighbor-relations";
            }
            else if (requireAll)
            {
                // Legacy requireAll logic for backward compatibility
                ok = allPass;
                reason = allPass ? "all-pass" : "some-fail";
            }
            else
            {
                // New logic: minimum neighbor count requirement
                ok = (passCount >= this.minNeighborsForLocked);
                reason = ok ? $"neighbors {passCount}/{totalNeighbors} pass" 
                           : $"neighbors {passCount}/{totalNeighbors} pass (need ≥{this.minNeighborsForLocked})";
            }
            
            // Enhanced logging: show all neighbor results
            if (perNeighborLines.Count > 0)
            {
                string pieceName = piece != null ? piece.name : "unknown";
                string statusText = ok ? "REL PASS" : "MATCH FAILED";
                string logMessage = $"[TangramMatcher] {statusText} '{pieceName}' {reason}\n" + string.Join(" | ", perNeighborLines);
                
                if (ok)
                {
                    Debug.Log(logMessage);
                }
                else
                {
                    Debug.LogWarning(logMessage);
                }
            }
            
            return ok;
        }
        catch (Exception ex)
        {
            reason = ex.Message;
            return false;
        }
    }

    private Transform FindAssignedPieceFor(TangramShapeType type, int arucoId)
    {
        // 1) Prefer current-frame match result
        foreach (var r in lastMatchResults)
        {
            if (r.shapeType == type && r.arucoId == arucoId)
                return r.matchedPieceTransform;
        }
        // 2) Optionally fall back to last assignment if recent enough
        if (!useExpectedFallbackFromLastAssignment) return null;
        if (lastAssignedPieceByKey.TryGetValue((type, arucoId), out var prev) && prev != null)
        {
            // If we track a locked state, ensure it was seen recently to avoid stale expectations
            if (lockedByPiece.TryGetValue(prev, out var ls))
            {
                if (frameCounter - ls.lastSeenFrame <= assignmentCooldownFrames)
                    return prev;
                return null;
            }
            // If not tracked as locked, still allow as a best-effort fallback
            return prev;
        }
        return null;
    }

    private float GetDiagramMaxEdgeLength()
    {
        if (diagramGraph == null || diagramGraph.nodes == null || diagramGraph.nodes.Count == 0)
            return 1f;
        float maxLen = 0.0001f;
        for (int i = 0; i < diagramGraph.nodes.Count; i++)
        {
            var ni = diagramGraph.nodes[i];
            for (int e = 0; e < ni.edges.Count; e++)
            {
                if (ni.edges[e].expectedDistanceMeters > maxLen)
                    maxLen = ni.edges[e].expectedDistanceMeters;
            }
        }
        return maxLen;
    }

    private float GetDetectionMaxPairDistance()
    {
        if (latestDetections == null || latestDetections.Count == 0) return 1f;
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        float maxLen = 0.0001f;
        for (int i = 0; i < latestDetections.Count; i++)
        {
            if (latestDetections[i].isCorner) continue;
            for (int j = i + 1; j < latestDetections.Count; j++)
            {
                if (latestDetections[j].isCorner) continue;
                Vector3 v = Vector3.ProjectOnPlane(latestDetections[j].worldPosition - latestDetections[i].worldPosition, planeN);
                float d = v.magnitude;
                if (d > maxLen) maxLen = d;
            }
        }
        return maxLen;
    }

    private DetectedShape? FindDetection(TangramShapeType type, int arucoId)
    {
        for (int i = 0; i < latestDetections.Count; i++)
        {
            var d = latestDetections[i];
            if (d.shapeType == type && d.arucoId == arucoId)
                return d;
        }
        return null;
    }

    private float GetCurrentMatchedSubsetMaxDiagramEdgeExpected(Dictionary<int, Transform> detToPiece)
    {
        if (detToPiece == null || detToPiece.Count < 2) return 1f;
        var pieces = new List<Transform>(detToPiece.Values);
        float maxLen = 0.0001f;
        for (int i = 0; i < pieces.Count; i++)
        {
            var pi = pieces[i]; if (pi == null) continue;
            for (int j = i + 1; j < pieces.Count; j++)
            {
                var pj = pieces[j]; if (pj == null) continue;
                float d = GetDiagramExpectedDistanceBetweenPieces(pi, pj);
                if (d > maxLen) maxLen = d;
            }
        }
        return maxLen;
    }

    private float GetCurrentMatchedSubsetMaxDetectionEdgeDistance(DetectionGraph dg, Dictionary<int, Transform> detToPiece)
    {
        if (detToPiece == null || detToPiece.Count < 2 || dg == null || dg.nodes == null) return 1f;
        var keys = new List<int>(detToPiece.Keys);
        float maxLen = 0.0001f;
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        for (int i = 0; i < keys.Count; i++)
        {
            int ki = keys[i]; if (!detToPiece.TryGetValue(ki, out var pi) || pi == null) continue;
            for (int j = i + 1; j < keys.Count; j++)
            {
                int kj = keys[j]; if (!detToPiece.TryGetValue(kj, out var pj) || pj == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(pi, out int ip)) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(pj, out int jp)) continue;
                if (!IsDiagramEdge(ip, jp)) continue;
                if (ki < 0 || ki >= dg.nodes.Count || kj < 0 || kj >= dg.nodes.Count) continue;
                Vector3 v = Vector3.ProjectOnPlane(dg.nodes[kj].pos - dg.nodes[ki].pos, planeN);
                float d = v.magnitude;
                if (d > maxLen) maxLen = d;
            }
        }
        return maxLen;
    }

    private float GetMatchedSubsetMaxDetectionEdgeDistanceThisFrame()
    {
        if (lastMatchResults == null || lastMatchResults.Count < 2) return 1f;
        float maxLen = 0.0001f;
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        for (int i = 0; i < lastMatchResults.Count; i++)
        {
            var ai = lastMatchResults[i]; if (ai.matchedPieceTransform == null) continue;
            for (int j = i + 1; j < lastMatchResults.Count; j++)
            {
                var aj = lastMatchResults[j]; if (aj.matchedPieceTransform == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(ai.matchedPieceTransform, out int ip)) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(aj.matchedPieceTransform, out int jp)) continue;
                if (!IsDiagramEdge(ip, jp)) continue;
                Vector3 v = Vector3.ProjectOnPlane(aj.detectionWorldPosition - ai.detectionWorldPosition, planeN);
                float d = v.magnitude;
                if (d > maxLen) maxLen = d;
            }
        }
        return maxLen;
    }

    private float GetDiagramExpectedDistanceBetweenPieces(Transform a, Transform b)
    {
        if (diagramGraph == null || a == null || b == null) return Vector3.Distance(GetPieceCenterWorld(a), GetPieceCenterWorld(b));
        if (!diagramGraph.indexByTransform.TryGetValue(a, out int ia)) return Vector3.Distance(GetPieceCenterWorld(a), GetPieceCenterWorld(b));
        if (!diagramGraph.indexByTransform.TryGetValue(b, out int ib)) return Vector3.Distance(GetPieceCenterWorld(a), GetPieceCenterWorld(b));
        // search edges in either direction
        var na = diagramGraph.nodes[ia];
        for (int e = 0; e < na.edges.Count; e++) if (na.edges[e].toIndex == ib) return na.edges[e].expectedDistanceMeters;
        var nb = diagramGraph.nodes[ib];
        for (int e = 0; e < nb.edges.Count; e++) if (nb.edges[e].toIndex == ia) return nb.edges[e].expectedDistanceMeters;
        // fallback to actual center distance if not directly connected by an edge
        return Vector3.Distance(GetPieceCenterWorld(a), GetPieceCenterWorld(b));
    }

    private bool TryGetSubsetReferenceBaselines(DetectionGraph dg, Dictionary<int, Transform> detToPiece, out float detBaseline, out float expBaseline)
    {
        detBaseline = 0f; expBaseline = 0f;
        if (dg == null || detToPiece == null || detToPiece.Count < 2 || diagramGraph == null)
            return false;
        // Invert mapping: piece -> detIndex
        var pieceToDet = new Dictionary<Transform, int>();
        foreach (var kv in detToPiece)
        {
            if (kv.Value != null && !pieceToDet.ContainsKey(kv.Value)) pieceToDet[kv.Value] = kv.Key;
        }
        // Iterate over piece pairs present in mapping; choose the pair with max expected edge distance
        float bestExp = -1f;
        Transform bestA = null, bestB = null;
        var pieces = new List<Transform>(pieceToDet.Keys);
        for (int i = 0; i < pieces.Count; i++)
        {
            var a = pieces[i]; if (a == null) continue;
            if (!diagramGraph.indexByTransform.TryGetValue(a, out int ia)) continue;
            for (int j = i + 1; j < pieces.Count; j++)
            {
                var b = pieces[j]; if (b == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(b, out int ib)) continue;
                float exp = -1f;
                var na = diagramGraph.nodes[ia];
                for (int e = 0; e < na.edges.Count; e++) if (na.edges[e].toIndex == ib) { exp = na.edges[e].expectedDistanceMeters; break; }
                if (exp < 0f)
                {
                    var nb = diagramGraph.nodes[ib];
                    for (int e = 0; e < nb.edges.Count; e++) if (nb.edges[e].toIndex == ia) { exp = nb.edges[e].expectedDistanceMeters; break; }
                }
                if (exp > bestExp)
                {
                    bestExp = exp; bestA = a; bestB = b;
                }
            }
        }
        if (bestA == null || bestB == null || bestExp <= 0f) return false;
        expBaseline = bestExp;
        // Detection baseline: distance between corresponding detection nodes
        if (!pieceToDet.TryGetValue(bestA, out int da) || !pieceToDet.TryGetValue(bestB, out int db)) return false;
        if (da < 0 || db < 0 || da >= dg.nodes.Count || db >= dg.nodes.Count) return false;
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 v = Vector3.ProjectOnPlane(dg.nodes[db].pos - dg.nodes[da].pos, planeN);
        detBaseline = v.magnitude;
        return detBaseline > 1e-4f && expBaseline > 1e-4f;
    }

    private bool TryGetReferenceBaselinesFromLastResults(out float detBaseline, out float expBaseline)
    {
        detBaseline = 0f; expBaseline = 0f;
        if (lastMatchResults == null || lastMatchResults.Count < 2 || diagramGraph == null) return false;
        float bestExp = -1f;
        MatchResult ra = default, rb = default;
        for (int i = 0; i < lastMatchResults.Count; i++)
        {
            var a = lastMatchResults[i]; if (a.matchedPieceTransform == null) continue;
            if (!diagramGraph.indexByTransform.TryGetValue(a.matchedPieceTransform, out int ia)) continue;
            for (int j = i + 1; j < lastMatchResults.Count; j++)
            {
                var b = lastMatchResults[j]; if (b.matchedPieceTransform == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(b.matchedPieceTransform, out int ib)) continue;
                float exp = -1f;
                var na = diagramGraph.nodes[ia];
                for (int e = 0; e < na.edges.Count; e++) if (na.edges[e].toIndex == ib) { exp = na.edges[e].expectedDistanceMeters; break; }
                if (exp < 0f)
                {
                    var nb = diagramGraph.nodes[ib];
                    for (int e = 0; e < nb.edges.Count; e++) if (nb.edges[e].toIndex == ia) { exp = nb.edges[e].expectedDistanceMeters; break; }
                }
                if (exp > bestExp)
                {
                    bestExp = exp; ra = a; rb = b;
                }
            }
        }
        if (bestExp <= 0f || ra.matchedPieceTransform == null || rb.matchedPieceTransform == null) return false;
        expBaseline = bestExp;
        Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
        Vector3 v = Vector3.ProjectOnPlane(rb.detectionWorldPosition - ra.detectionWorldPosition, planeN);
        detBaseline = v.magnitude;
        return detBaseline > 1e-4f && expBaseline > 1e-4f;
    }

    private float GetMatchedSubsetMaxDiagramDistanceThisFrame()
    {
        if (lastMatchResults == null || lastMatchResults.Count < 2 || diagramGraph == null) return 1f;
        float maxLen = 0.0001f;
        for (int i = 0; i < lastMatchResults.Count; i++)
        {
            var a = lastMatchResults[i]; if (a.matchedPieceTransform == null) continue;
            if (!diagramGraph.indexByTransform.TryGetValue(a.matchedPieceTransform, out int ia)) continue;
            for (int j = i + 1; j < lastMatchResults.Count; j++)
            {
                var b = lastMatchResults[j]; if (b.matchedPieceTransform == null) continue;
                if (!diagramGraph.indexByTransform.TryGetValue(b.matchedPieceTransform, out int ib)) continue;
                // use expected distance if edge exists, else skip (only diagram relations)
                var na = diagramGraph.nodes[ia];
                for (int e = 0; e < na.edges.Count; e++)
                {
                    if (na.edges[e].toIndex == ib)
                    {
                        if (na.edges[e].expectedDistanceMeters > maxLen)
                            maxLen = na.edges[e].expectedDistanceMeters;
                        break;
                    }
                }
            }
        }
        return maxLen;
    }

    private void UpdateCachedBaselinesFromLastResults()
    {
        if (!useNormalizedRelationDistance || !useSubsetUnifiedBaseline) return;
        if (cachedBaselineFrame == frameCounter && cachedBaselineValid) return;
        float detRef, expRef;
        if (TryGetReferenceBaselinesFromLastResults(out detRef, out expRef))
        {
            cachedBaselineDetRef = detRef;
            cachedBaselineExpRef = expRef;
            cachedBaselineFrame = frameCounter;
            cachedBaselineValid = true;
        }
        // else: keep previous cached values
    }

    private bool TryGetPieceType(Transform piece, out TangramShapeType type)
    {
        foreach (var kv in diagramPiecesByType)
        {
            foreach (var p in kv.Value)
            {
                if (p == piece) { type = kv.Key; return true; }
            }
        }
        type = default; return false;
    }

    // Reset cached normalization baselines if a new non-corner detection appeared this frame
    private void InvalidateBaselinesIfNewDetectionsPresent()
    {
        try
        {
            var current = new HashSet<string>();
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var d = latestDetections[i];
                if (d.isCorner) continue;
                current.Add($"{d.shapeType}:{d.arucoId}");
            }
            bool hasNew = false;
            foreach (var key in current)
            {
                if (!previousDetectionKeySet.Contains(key)) { hasNew = true; break; }
            }
            // update snapshot for next frame
            previousDetectionKeySet.Clear();
            foreach (var k in current) previousDetectionKeySet.Add(k);

            if (hasNew)
            {
                cachedBaselineValid = false;
                cachedBaselineFrame = -1;
                // Reset first-pass scale when new detections appear
                firstPassScaleValid = false;
                firstPassScaleFrame = -1;
            }
        }
        catch { /* defensive: do nothing on exceptions */ }
    }

    /// <summary>
    /// Compute matches and apply debug coloring.
    /// </summary>
    private void RunMatchingAndHighlight()
    {
        lastMatchResults.Clear();

        // 상대 각도 디버깅: 감지된 도형들의 관계 요약 출력
        if (debugLogRelativeAngles)
        {
            DebugLogDetectionRelationsSummary();
        }

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

        // Orientation-only PRE-ASSIGNMENT: if a shape type has exactly one detection but multiple diagram pieces
        // of that type exist, assign it solely by absolute orientation BEFORE graph matching to avoid relation bias.
        if (forceOrientationForSingleDetection && useAbsoluteOrientation)
        {
            var detectionsByTypePre = new Dictionary<TangramShapeType, List<int>>();
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var d = latestDetections[i];
                if (d.isCorner) continue;
                if (!detectionsByTypePre.TryGetValue(d.shapeType, out var list))
                {
                    list = new List<int>();
                    detectionsByTypePre[d.shapeType] = list;
                }
                list.Add(i);
            }
            foreach (var kv in detectionsByTypePre)
            {
                var type = kv.Key;
                var list = kv.Value;
                if (list.Count != 1) continue;
                if (!availablePieces.TryGetValue(type, out var candidates) || candidates == null) continue;
                // Consider all currently available (unmatched/unlocked) pieces of this type
                var remaining = new List<Transform>(candidates);
                if (remaining.Count <= 1) continue;

                int detIndex = list[0];
                var det = latestDetections[detIndex];
                float bestAng = float.PositiveInfinity;
                Transform bestPiece = null;
                foreach (var piece in remaining)
                {
                    if (piece == null) continue;
                    float a = ComputeAngleError(type, det.worldRotation, piece);
                    if (a < bestAng)
                    {
                        bestAng = a;
                        bestPiece = piece;
                    }
                }
                if (bestPiece != null)
                {
                    // Use preAssignedDetections so downstream matching treats it as fixed
                    preAssignedDetections[detIndex] = bestPiece;
                    if (debugLogOrientationDiff)
                    {
                        Debug.Log($"[TangramMatcher] ORI preassign (single det) {type}:{det.arucoId} -> diag({bestPiece.name}) | absOri={bestAng:F1}°");
                    }
                }
            }
        }

        // Absolute greedy matching removed – use graph-only matching
        RunGraphOnlyMatching(availablePieces, matchedPieceSet, linkSegments);

        // Orientation-only assignment fallback:
        // If a shape type has exactly 1 detection and multiple diagram pieces remain for that type,
        // assign it to the piece with the smallest absolute orientation difference (ignoring relations),
        // but only if the piece is not already matched in this frame.
        if ((forceOrientationForSingleDetection ) && useAbsoluteOrientation)
        {
            var detectionsByType = new Dictionary<TangramShapeType, List<int>>();
            for (int i = 0; i < latestDetections.Count; i++)
            {
                var d = latestDetections[i];
                if (d.isCorner) continue;
                if (!detectionsByType.TryGetValue(d.shapeType, out var list))
                {
                    list = new List<int>();
                    detectionsByType[d.shapeType] = list;
                }
                list.Add(i);
            }
            foreach (var kv in detectionsByType)
            {
                var type = kv.Key;
                var list = kv.Value;
                if (list.Count != 1) continue;
                // Collect remaining unmatched diagram pieces of this type
                if (!diagramPiecesByType.TryGetValue(type, out var allPieces)) continue;
                var remaining = new List<Transform>();
                foreach (var p in allPieces) if (!matchedPieceSet.Contains(p)) remaining.Add(p);
                if (remaining.Count <= 1) continue;

                int detIndex = list[0];
                var det = latestDetections[detIndex];
                Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                Vector3 detDir = Vector3.ProjectOnPlane(det.worldRotation * Vector3.right, planeN).normalized;
                if (detDir.sqrMagnitude < 1e-6f) continue;

                float bestAng = float.PositiveInfinity;
                Transform bestPiece = null;
                if (debugLogOrientationSweep)
                {
                    Debug.Log($"[TangramMatcher] ORI sweep for det({type}:{det.arucoId}) vs {remaining.Count} candidates");
                }
                foreach (var piece in remaining)
                {
                    Vector3 pieceDir = Vector3.ProjectOnPlane(GetPieceDirectionVector(piece), planeN).normalized;
                    if (pieceDir.sqrMagnitude < 1e-6f) continue;
                    float a = Mathf.Abs(Vector3.SignedAngle(pieceDir, detDir, planeN));
                    a = NormalizeAngle180(a);
                    float mod = GetSymmetryModuloDegrees(type);
                    if (mod > 0f)
                    {
                        a = a % mod;
                        a = Mathf.Min(a, mod - a);
                    }
                    if (debugLogOrientationSweep)
                    {
                        Debug.Log($"[TangramMatcher]   -> diag({piece.name}) absOri={a:F1}°");
                    }
                    if (a < bestAng)
                    {
                        bestAng = a;
                        bestPiece = piece;
                    }
                }
                if (bestPiece != null)
                {
                    matchedPieceSet.Add(bestPiece);
                    Vector3 planeN_abs = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    Vector3 vAbs = Vector3.ProjectOnPlane(GetPieceCenterWorld(bestPiece) - det.worldPosition, planeN_abs);
                    float absDist = vAbs.magnitude;
                    lastMatchResults.Add(new MatchResult
                    {
                        shapeType = type,
                        arucoId = det.arucoId,
                        matchedPieceTransform = bestPiece,
                        worldDistanceMeters = absDist,
                        detectionWorldPosition = det.worldPosition,
                        detectionWorldRotation = det.worldRotation, // ★ 추가: 감지된 도형의 회전 정보 저장
                        angleErrorDegrees = bestAng
                    });
                    if (debugLogOrientationDiff)
                    {
                        Debug.Log($"[TangramMatcher] ORI assign (single det) {type}:{det.arucoId} -> diag({bestPiece.name}) | absOri={bestAng:F1}°");
                    }
                }
            }
        }

        // Error stats per type
        ComputeAndStoreErrorStats();

        ApplyDebugColors(matchedPieceSet);

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

        // Update cached baselines ONCE per frame after matching
        UpdateCachedBaselinesFromLastResults();

        // 현재 프레임의 매칭 결과를 다음 프레임의 경고 억제에 사용하기 위해 저장
        _matchedPiecesLastFrame.Clear();
        foreach (var result in lastMatchResults)
        {
            _matchedPiecesLastFrame.Add(result.matchedPieceTransform);
        }

        // Debug log in English for clarity
        foreach (var result in lastMatchResults)
        {
            string graphInfo = BuildGraphDebugInfo(result);
            
            // 상대 각도 정보 추가
            string relativeAngleInfo = "";
            if (debugLogRelativeAngles)
            {
                var detection = FindDetection(result.shapeType, result.arucoId);
                if (detection.HasValue)
                {
                    // 이 감지된 도형의 회전을 기준으로 한 방향 벡터
                    Vector3 detectionOrientation = detection.Value.worldRotation * Vector3.right;
                    Vector3 pieceOrientation = result.matchedPieceTransform.rotation * Vector3.right;
                    Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                    
                    Vector3 detOrientProj = Vector3.ProjectOnPlane(detectionOrientation, planeN).normalized;
                    Vector3 pieceOrientProj = Vector3.ProjectOnPlane(pieceOrientation, planeN).normalized;
                    
                    if (detOrientProj.sqrMagnitude > 1e-6f && pieceOrientProj.sqrMagnitude > 1e-6f)
                    {
                        float orientationDiff = Vector3.SignedAngle(pieceOrientProj, detOrientProj, planeN);
                        relativeAngleInfo = $" | 방향차이={NormalizeAngle180(orientationDiff):F1}°";
                    }
                }
            }
            
            Debug.Log($"[TangramMatcher] Matched {result.shapeType} (ArUco {result.arucoId}) -> Piece '{result.matchedPieceTransform.name}', distance = {result.worldDistanceMeters:F3} m, angle = {result.angleErrorDegrees:F1} deg{relativeAngleInfo}. {graphInfo}");
            if (debugLogOrientationPerDetection && useAbsoluteOrientation)
            {
                // Report absolute orientation difference (independent of relations) for this detection vs its assigned piece
                Vector3 planeN = useXYPlane ? Vector3.forward : (tangramDiagramRoot != null ? tangramDiagramRoot.up : Vector3.up);
                var det = FindDetection(result.shapeType, result.arucoId);
                if (det.HasValue)
                {
                    Vector3 detDir = Vector3.ProjectOnPlane(det.Value.worldRotation * Vector3.right, planeN).normalized;
                    if (detDir.sqrMagnitude > 1e-6f)
                    {
                        // Matched piece ORI: use the precomputed angle (includes offsets/symmetry)
                        float a = result.angleErrorDegrees;
                        Debug.Log($"[TangramMatcher] ORI report {result.shapeType}:{result.arucoId} -> diag({result.matchedPieceTransform.name}) | absOri={a:F1}°");
                        // Also report per-candidate ORI for same-type pieces when multiple exist
                        if (diagramPiecesByType.TryGetValue(result.shapeType, out var pieces) && pieces != null)
                        {
                            int multi = 0; foreach (var p in pieces) if (p != null) multi++;
                            if (multi >= 2)
                            {
                                foreach (var p in pieces)
                                {
                                    if (p == null) continue;
                                    // Use the unified function so angleOffset and symmetry are applied consistently
                                    float ang = ComputeAngleError(result.shapeType, det.Value.worldRotation, p);
                                    Debug.Log($"[TangramMatcher] ORI candidate {result.shapeType}:{result.arucoId} vs diag({p.name}) | absOri={ang:F1}°");
                                }
                            }
                        }
                    }
                }
            }
            if (debugAllDiagramEdges)
            {
                ReportAllDiagramEdgeDiffs(result, debugEdgesReportPerNode);
            }
            if (debugDetectionRelations)
            {
                ReportDetectionRelations(result, debugDetectionRelationsPerNode, drawDetectionRelationLines);
            }
        }

        if (debugLogPerTypeErrorStats)
        {
            foreach (var kv in lastTypeErrorStats)
            {
                var s = kv.Value;
                Debug.Log($"[TangramMatcher] Error stats for {kv.Key} -> count={s.count}, min={s.min:F3}m, max={s.max:F3}m, avg={s.avg:F3}m");
            }
        }

        // Log all connection angles for comprehensive debugging (optional)
        if (debugDetectionRelations)
        {
            DebugLogAllConnectionAngles();
        }

        // Flush REL logs at end of frame
        if (relLogQueue.Count > 0)
        {
            // One-line header summarizing current angle offset settings for REL logs
            Debug.Log($"[TangramMatcher] REL settings | angleOffset={angleOffset:F1}° conn={(applyOffsetToConnectionAngles ? 1 : 0)} ori={(applyOffsetToOrientations ? 1 : 0)}");
            foreach (var line in relLogQueue) Debug.Log(line);
            relLogQueue.Clear();
        }

        // Debug: current lock states
        if (debugLogLockedStates && enableStatefulMatching)
        {
            foreach (var kv in lockedByPiece)
            {
                var ls = kv.Value;
                string matched = "no";
                foreach (var r in lastMatchResults) { if (r.matchedPieceTransform == ls.piece) { matched = $"det({r.shapeType}:{r.arucoId})"; break; } }
                Debug.Log($"[TangramMatcher] LOCK {ls.piece.name} type={ls.type} locked={(ls.isLocked ? 1:0)} conf={ls.confidence:F2} lastSeen={ls.lastSeenFrame} matched={matched}");
            }
        }

        // Update orientation arrows for markers
        UpdateDetectionOrientationArrows();
    }

    private void ApplyDebugColors(HashSet<Transform> matchedPieces)
    {
        // First, update unmatched pieces
        foreach (var kv in diagramPiecesByType)
        {
            foreach (Transform piece in kv.Value)
            {
                bool isMatchedOrLocked = matchedPieces.Contains(piece) || (enableStatefulMatching && lockedByPiece.TryGetValue(piece, out var ls0) && ls0.isLocked);
                if (isMatchedOrLocked) continue;
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
        var colored = new HashSet<Transform>();
        foreach (Transform piece in matchedPieces)
        {
            colored.Add(piece);
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

        // Also color locked pieces not already colored this frame
        if (enableStatefulMatching)
        {
            foreach (var kv in lockedByPiece)
            {
                var piece = kv.Key;
                var ls = kv.Value;
                if (!ls.isLocked) continue;
                if (colored.Contains(piece)) continue;
                if (useNeutralColorMode && matchedRestoresOriginalColor)
                {
                    RestoreRendererOriginalColor(piece.gameObject);
                }
                else
                {
                    SetRendererColor(piece.gameObject, matchedColor);
                }
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
}


