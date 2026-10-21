using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Game loop on top of TangramMatcher: Intro → Ready (scatter + countdown) → Playing (timer, hints) → Completed.
///
/// A piece counts as placed only while the matcher has it *locked* (relation-verified against a
/// neighbor), so single-frame guesses never advance progress. The matcher itself is not modified
/// by the game; it is only queried each frame and reset between rounds.
/// </summary>
public class TangramGameManager : MonoBehaviour
{
    public enum GameState { Intro, Ready, Playing, Completed }
    public enum PieceStatus { Empty, Detected, Placed }

    public class PieceInfo
    {
        public Transform transform;
        public TangramMatcher.TangramShapeType type;
        public string displayName;
        public Color color;
        public PieceStatus status;
        internal float graceStart = -1f; // when the matcher dropped the lock while we still count it
    }

    public struct RoundResult
    {
        public string puzzleName;
        public float seconds;
        public int hintsUsed;
        public int stars;
        public float bestSeconds;
        public bool isNewBest;
    }

    [Header("References")]
    public TangramMatcher matcher;
    [Tooltip("Answer layouts, each a root with the 7 pieces as children. Empty = use matcher.tangramDiagramRoot only.")]
    public List<Transform> puzzleRoots = new List<Transform>();
    [Tooltip("Display name per puzzle (same order as puzzleRoots). Falls back to \"퍼즐 N\".")]
    public List<string> puzzleDisplayNames = new List<string>();
    [Tooltip("Camera that renders the answer diagram. Used to place hint highlights. Falls back to Camera.main.")]
    public Camera diagramCamera;

    [Header("Rules")]
    [Tooltip("All pieces must stay placed this long before the puzzle counts as solved (filters flicker).")]
    public float completionHoldSeconds = 1.0f;
    [Tooltip("If no detections arrive for this long, the board is treated as out of view and the timer pauses.")]
    public float boardLostTimeoutSeconds = 0.5f;
    [Tooltip("Countdown before the timer starts, once the board is visible and scattered.")]
    public float readyCountdownSeconds = 3f;
    [Tooltip("The countdown only runs while at most this many pieces are already placed. Prevents an instant win when retrying with a solved board.")]
    public int maxPlacedPiecesToStart = 2;
    public float hintCooldownSeconds = 3f;
    public float hintDurationSeconds = 3f;
    [Tooltip("Star rating target: solving within pieces × this = 3 stars, within 2× = 2 stars, slower = 1 star. Each hint then costs one star.")]
    public float secondsPerPieceTarget = 10f;
    [Tooltip("With the front camera the player's hand often covers pieces that are already placed. A placed piece keeps counting this long after the matcher drops its lock. Completion still requires every piece to be actually locked.")]
    public float lostGraceSeconds = 1.5f;

    [Header("Device")]
    [Tooltip("The player watches the screen without touching it while arranging pieces, so keep the display on during Ready/Playing.")]
    public bool keepScreenAwakeWhilePlaying = true;
    [Tooltip("The HUD and camera-feed rotation are laid out for portrait; stop auto-rotation when the device tilts on its stand.")]
    public bool lockPortrait = true;

    [Header("Debug Output")]
    [Tooltip("The matcher/tracker emit many Debug.Log calls per frame, which is slow on device. Player builds keep only errors.")]
    public bool muteVerboseLogsInBuilds = true;
    [Tooltip("Hide the matcher's IMGUI debug labels (Euler/error text) in player builds.")]
    public bool hideDebugLabelsInBuilds = true;

    public event Action<GameState> StateChanged;
    public event Action<PieceInfo> PiecePlaced;
    public event Action<PieceInfo> PieceLost;
    public event Action<PieceInfo, string> HintShown;
    public event Action<int> CountdownTick; // seconds remaining; 0 = go
    public event Action<RoundResult> RoundCompleted;
    public event Action PuzzleLoaded;

    public GameState State { get; private set; } = GameState.Intro;
    public int CurrentPuzzleIndex { get; private set; }
    public int PuzzleCount => puzzleRoots.Count;
    public float ElapsedSeconds { get; private set; }
    public int HintsUsed { get; private set; }
    public bool BoardVisible { get; private set; }
    /// <summary>Placed pieces as shown to the player, including ones briefly hidden (see lostGraceSeconds).</summary>
    public int PlacedCount { get; private set; }
    /// <summary>Pieces the matcher has locked right now. Used for completion and the ready check.</summary>
    public int VerifiedCount { get; private set; }
    public int TotalCount => pieces.Count;
    public IReadOnlyList<PieceInfo> Pieces => pieces;
    public PieceInfo HintPiece { get; private set; }
    public float HintCooldownRemaining => Mathf.Max(0f, nextHintTime - Time.time);
    public RoundResult LastResult { get; private set; }
    public bool ReadyBlockedByPlacedPieces { get; private set; }

    private readonly List<PieceInfo> pieces = new List<PieceInfo>();
    private readonly List<PieceInfo> newlyPlaced = new List<PieceInfo>();
    private readonly List<PieceInfo> newlyLost = new List<PieceInfo>();
    private float completionHoldTimer;
    private float readyTimer;
    private int lastCountdownValue;
    private float nextHintTime;
    private float hintUntil;

    private const string BestKeyPrefix = "TangramBest_";
    private const string LastPuzzleKey = "TangramLastPuzzle";

    void Awake()
    {
        if (!Application.isEditor && muteVerboseLogsInBuilds)
        {
            // Filtering happens before the log reaches logcat/console, which is the expensive part on device.
            Debug.unityLogger.filterLogType = LogType.Error;
        }
        if (lockPortrait) Screen.orientation = ScreenOrientation.Portrait;
    }

    void OnDisable()
    {
        if (keepScreenAwakeWhilePlaying) Screen.sleepTimeout = SleepTimeout.SystemSetting;
    }

    void Start()
    {
        if (matcher == null) matcher = FindFirstObjectByType<TangramMatcher>();
        if (matcher == null)
        {
            Debug.LogError("TangramGameManager: no TangramMatcher found in scene.");
            enabled = false;
            return;
        }

        // The HUD shows its own (localized) status, so the matcher's IMGUI hint would overlap it.
        matcher.showMissingCornersUI = false;
        if (!Application.isEditor && hideDebugLabelsInBuilds)
        {
            matcher.showDetectionEulerUI = false;
            matcher.showErrorLabelsUI = false;
        }

        puzzleRoots.RemoveAll(r => r == null);
        if (puzzleRoots.Count == 0 && matcher.tangramDiagramRoot != null) puzzleRoots.Add(matcher.tangramDiagramRoot);

        if (GetComponent<TangramGameAudio>() == null) gameObject.AddComponent<TangramGameAudio>();
        if (GetComponent<TangramGameHUD>() == null) gameObject.AddComponent<TangramGameHUD>();

        LoadPuzzle(PlayerPrefs.GetInt(LastPuzzleKey, 0));
    }

    void Update()
    {
        if (matcher == null) return;

        bool fresh = Time.time - matcher.LastDetectionsTime <= boardLostTimeoutSeconds;
        BoardVisible = fresh && (!matcher.requirePlaneCorners || matcher.PlaneCornersReady);

        // Matcher state is frozen while the board is out of view, so only re-read it when visible.
        if (BoardVisible) RefreshPieceStatuses();

        switch (State)
        {
            case GameState.Ready: UpdateReady(); break;
            case GameState.Playing: UpdatePlaying(); break;
        }

        if (HintPiece != null && (Time.time >= hintUntil || HintPiece.status == PieceStatus.Placed))
        {
            HintPiece = null;
        }
    }

    // ---------- Public actions (wired to HUD buttons) ----------

    public void LoadPuzzle(int index)
    {
        if (puzzleRoots.Count == 0) return;
        CurrentPuzzleIndex = ((index % puzzleRoots.Count) + puzzleRoots.Count) % puzzleRoots.Count;
        PlayerPrefs.SetInt(LastPuzzleKey, CurrentPuzzleIndex);

        if (puzzleRoots.Count > 1)
        {
            for (int i = 0; i < puzzleRoots.Count; i++)
                puzzleRoots[i].gameObject.SetActive(i == CurrentPuzzleIndex);
        }
        matcher.SetDiagramRoot(puzzleRoots[CurrentPuzzleIndex]);
        BuildPieceList();
        HintPiece = null;
        SetState(GameState.Intro);
        PuzzleLoaded?.Invoke();
    }

    public void NextPuzzle() => LoadPuzzle(CurrentPuzzleIndex + 1);
    public void PreviousPuzzle() => LoadPuzzle(CurrentPuzzleIndex - 1);

    /// <summary>Begin (or restart) a round on the current puzzle.</summary>
    public void StartRound()
    {
        matcher.ResetMatchingState();
        foreach (var p in pieces) { p.status = PieceStatus.Empty; p.graceStart = -1f; }
        PlacedCount = 0;
        VerifiedCount = 0;
        ElapsedSeconds = 0f;
        HintsUsed = 0;
        HintPiece = null;
        nextHintTime = 0f;
        completionHoldTimer = 0f;
        readyTimer = 0f;
        lastCountdownValue = -1;
        SetState(GameState.Ready);
    }

    public void BackToIntro()
    {
        HintPiece = null;
        SetState(GameState.Intro);
    }

    public void RequestHint()
    {
        if (State != GameState.Playing || HintCooldownRemaining > 0f) return;
        var piece = ChooseHintPiece(out string message);
        if (piece == null) return;
        HintsUsed++;
        HintPiece = piece;
        hintUntil = Time.time + hintDurationSeconds;
        nextHintTime = Time.time + hintCooldownSeconds;
        HintShown?.Invoke(piece, message);
    }

    public string CurrentPuzzleName
    {
        get
        {
            int i = CurrentPuzzleIndex;
            if (i < puzzleDisplayNames.Count && !string.IsNullOrEmpty(puzzleDisplayNames[i])) return puzzleDisplayNames[i];
            return $"퍼즐 {i + 1}";
        }
    }

    public bool TryGetBest(out float seconds, out int stars)
    {
        string key = BestKeyPrefix + PuzzleKey();
        seconds = PlayerPrefs.GetFloat(key, -1f);
        stars = PlayerPrefs.GetInt(key + "_stars", 0);
        return seconds >= 0f;
    }

    public Camera GetDiagramCamera() => diagramCamera != null ? diagramCamera : Camera.main;

    // ---------- State updates ----------

    private void UpdateReady()
    {
        ReadyBlockedByPlacedPieces = BoardVisible && VerifiedCount > maxPlacedPiecesToStart;
        if (!BoardVisible || ReadyBlockedByPlacedPieces)
        {
            readyTimer = 0f;
            if (lastCountdownValue != -1) { lastCountdownValue = -1; CountdownTick?.Invoke(-1); }
            return;
        }

        readyTimer += Time.deltaTime;
        int remaining = Mathf.CeilToInt(readyCountdownSeconds - readyTimer);
        if (remaining != lastCountdownValue && remaining > 0)
        {
            lastCountdownValue = remaining;
            CountdownTick?.Invoke(remaining);
        }
        if (readyTimer >= readyCountdownSeconds)
        {
            CountdownTick?.Invoke(0);
            SetState(GameState.Playing);
        }
    }

    private void UpdatePlaying()
    {
        if (!BoardVisible)
        {
            completionHoldTimer = 0f; // timer pauses too: it only advances below
            return;
        }

        ElapsedSeconds += Time.deltaTime;

        if (TotalCount > 0 && VerifiedCount == TotalCount)
        {
            completionHoldTimer += Time.deltaTime;
            if (completionHoldTimer >= completionHoldSeconds) CompleteRound();
        }
        else
        {
            completionHoldTimer = 0f;
        }
    }

    private void CompleteRound()
    {
        string key = BestKeyPrefix + PuzzleKey();
        float best = PlayerPrefs.GetFloat(key, -1f);
        int stars = CalculateStars(ElapsedSeconds, HintsUsed, TotalCount);
        bool isNewBest = best < 0f || ElapsedSeconds < best;
        if (isNewBest)
        {
            best = ElapsedSeconds;
            PlayerPrefs.SetFloat(key, best);
        }
        if (stars > PlayerPrefs.GetInt(key + "_stars", 0)) PlayerPrefs.SetInt(key + "_stars", stars);
        PlayerPrefs.Save();

        LastResult = new RoundResult
        {
            puzzleName = CurrentPuzzleName,
            seconds = ElapsedSeconds,
            hintsUsed = HintsUsed,
            stars = stars,
            bestSeconds = best,
            isNewBest = isNewBest
        };
        HintPiece = null;
        SetState(GameState.Completed);
        RoundCompleted?.Invoke(LastResult);
    }

    /// <summary>
    /// Star rating (0–3) for a solved round.
    /// </summary>
    /// <param name="seconds">Solve time with board-out-of-view time excluded.</param>
    /// <param name="hintsUsed">Number of hints requested this round.</param>
    /// <param name="pieceCount">Pieces in the puzzle (7 for a standard set), useful for scaling time targets.</param>
    private int CalculateStars(float seconds, int hintsUsed, int pieceCount)
    {
        // Target time scales with the puzzle: 10 s per piece (70 s for a standard 7-piece set).
        float target = Mathf.Max(1, pieceCount) * secondsPerPieceTarget;
        int stars = seconds <= target ? 3 : seconds <= target * 2f ? 2 : 1;
        // Each hint costs one star.
        return Mathf.Clamp(stars - hintsUsed, 0, 3);
    }

    private void SetState(GameState s)
    {
        State = s;
        if (keepScreenAwakeWhilePlaying)
        {
            // Intro/result need a tap anyway, so let the OS timeout apply there to save battery.
            Screen.sleepTimeout = s == GameState.Ready || s == GameState.Playing ? SleepTimeout.NeverSleep : SleepTimeout.SystemSetting;
        }
        StateChanged?.Invoke(s);
    }

    // ---------- Pieces ----------

    private void BuildPieceList()
    {
        pieces.Clear();
        foreach (var t in matcher.GetDiagramPieces())
        {
            if (!TangramMatcher.TryGetShapeTypeFromPieceName(t.name, out var type)) continue;
            pieces.Add(new PieceInfo
            {
                transform = t,
                type = type,
                displayName = ShapeDisplayName(type),
                color = ReadPieceColor(t),
                status = PieceStatus.Empty
            });
        }
        // Largest first so the HUD chip row reads big → small.
        pieces.Sort((a, b) => ShapeSizeRank(a.type) != ShapeSizeRank(b.type)
            ? ShapeSizeRank(a.type).CompareTo(ShapeSizeRank(b.type))
            : string.CompareOrdinal(a.transform.name, b.transform.name));
        PlacedCount = 0;
    }

    private void RefreshPieceStatuses()
    {
        int placed = 0, verified = 0;
        newlyPlaced.Clear();
        newlyLost.Clear();
        foreach (var p in pieces)
        {
            var prev = p.status;
            bool locked = matcher.IsPieceLocked(p.transform);
            bool inGrace = false;
            if (!locked && prev == PieceStatus.Placed && State == GameState.Playing)
            {
                // A hand reaching over the board hides markers of placed pieces; don't report them lost right away.
                if (p.graceStart < 0f) p.graceStart = Time.time;
                inGrace = Time.time - p.graceStart < lostGraceSeconds;
            }
            if (!inGrace) p.graceStart = -1f;

            p.status = locked || inGrace ? PieceStatus.Placed
                : matcher.IsPieceMatchedThisFrame(p.transform) ? PieceStatus.Detected
                : PieceStatus.Empty;
            if (locked) verified++;
            if (p.status == PieceStatus.Placed) placed++;
            if (p.status == PieceStatus.Placed && prev != PieceStatus.Placed) newlyPlaced.Add(p);
            else if (prev == PieceStatus.Placed && p.status != PieceStatus.Placed) newlyLost.Add(p);
        }
        // Commit the counts before notifying so listeners see the up-to-date progress.
        PlacedCount = placed;
        VerifiedCount = verified;
        if (State != GameState.Playing) return;
        foreach (var p in newlyPlaced) PiecePlaced?.Invoke(p);
        foreach (var p in newlyLost) PieceLost?.Invoke(p);
    }

    /// <summary>
    /// A piece only locks once a diagram neighbor is verified next to it, so the most useful hint is an
    /// unplaced piece adjacent to one that is already placed. With nothing placed yet, suggest the
    /// largest piece as an anchor and remind the player that it needs a neighbor to register.
    /// </summary>
    private PieceInfo ChooseHintPiece(out string message)
    {
        message = null;
        PieceInfo best = null;
        foreach (var p in pieces)
        {
            if (p.status == PieceStatus.Placed) continue;
            bool touchesPlaced = false;
            foreach (var n in matcher.GetDiagramNeighbors(p.transform))
            {
                if (matcher.IsPieceLocked(n)) { touchesPlaced = true; break; }
            }
            if (touchesPlaced && (best == null || ShapeSizeRank(p.type) < ShapeSizeRank(best.type))) best = p;
        }
        if (best != null)
        {
            message = $"{WithObjectParticle(best.displayName)} 표시된 자리에 놓아 보세요";
            return best;
        }

        foreach (var p in pieces)
        {
            if (p.status == PieceStatus.Placed) continue;
            if (best == null || ShapeSizeRank(p.type) < ShapeSizeRank(best.type)) best = p;
        }
        if (best != null)
        {
            message = PlacedCount == 0
                ? $"{best.displayName}부터 시작해 보세요. 옆 조각과 함께 놓아야 인식돼요"
                : $"{WithObjectParticle(best.displayName)} 표시된 자리에 놓아 보세요";
        }
        return best;
    }

    /// <summary>Appends 을/를 depending on whether the last Hangul syllable has a final consonant.</summary>
    private static string WithObjectParticle(string word)
    {
        if (string.IsNullOrEmpty(word)) return word;
        char c = word[word.Length - 1];
        bool hasFinal = c >= 0xAC00 && c <= 0xD7A3 && (c - 0xAC00) % 28 != 0;
        return word + (hasFinal ? "을" : "를");
    }

    private string PuzzleKey() => CurrentPuzzleIndex < puzzleRoots.Count ? puzzleRoots[CurrentPuzzleIndex].name : "default";

    public static string ShapeDisplayName(TangramMatcher.TangramShapeType type)
    {
        switch (type)
        {
            case TangramMatcher.TangramShapeType.LargeTriangle: return "큰 삼각형";
            case TangramMatcher.TangramShapeType.MediumTriangle: return "중간 삼각형";
            case TangramMatcher.TangramShapeType.SmallTriangle: return "작은 삼각형";
            case TangramMatcher.TangramShapeType.Square: return "정사각형";
            case TangramMatcher.TangramShapeType.Parallelogram: return "평행사변형";
            default: return type.ToString();
        }
    }

    private static int ShapeSizeRank(TangramMatcher.TangramShapeType type)
    {
        switch (type)
        {
            case TangramMatcher.TangramShapeType.LargeTriangle: return 0;
            case TangramMatcher.TangramShapeType.MediumTriangle: return 1;
            case TangramMatcher.TangramShapeType.Square: return 2;
            case TangramMatcher.TangramShapeType.Parallelogram: return 3;
            default: return 4;
        }
    }

    private static Color ReadPieceColor(Transform t)
    {
        var r = t.GetComponent<Renderer>();
        var m = r != null ? r.sharedMaterial : null;
        if (m == null) return Color.white;
        if (m.HasProperty("_BaseColor")) return m.GetColor("_BaseColor");
        if (m.HasProperty("_Color")) return m.GetColor("_Color");
        return Color.white;
    }
}
