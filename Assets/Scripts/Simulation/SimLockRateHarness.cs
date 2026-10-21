using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ObjdetectModule;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// Repeatable lock-rate test on the virtual mirror camera (real ArUco detection → real TangramMatcher).
///
/// Each condition runs N seeded trials. A trial completes the board with sticker and placement noise and
/// succeeds when every piece is locked in its correct slot within <see cref="settleSeconds"/> and stays that
/// way for the last <see cref="holdSeconds"/>. Congruent pieces (the two large / small triangles) may take
/// either slot; "correct" means the slot the physical piece actually occupies.
///
/// Usage: add next to SimulatedTangramCamera and enter Play mode. Results go to the console (and on screen).
/// Trials are seeded, so two runs see the same sticker/placement noise and can be compared directly.
/// </summary>
public class SimLockRateHarness : MonoBehaviour
{
    public enum Protocol
    {
        [Tooltip("Board already solved, matcher reset: does a finished board lock?")]
        SolvedBoard,
        [Tooltip("Matcher reset on a scattered board, then all pieces placed: also counts false locks while scattered.")]
        FromScatter,
        [Tooltip("Pieces placed one at a time like a player would (unplaced pieces stay scattered on the board).")]
        OneByOne,
    }

    [Serializable]
    public class Condition
    {
        public string name = "Typical+noise";
        public Protocol protocol = Protocol.SolvedBoard;
        public CameraDegradationPreset preset = CameraDegradationPreset.Typical;
        public float stickerErrorMm = 1f;
        public float stickerErrorDeg = 2f;
        public float placementNoiseMm = 1.5f;
        public float placementNoiseDeg = 2f;
        [Min(1)] public int trials = 10;

        public Condition() { }
        public Condition(string name, Protocol protocol, CameraDegradationPreset preset, bool noise, int trials)
        {
            this.name = name; this.protocol = protocol; this.preset = preset; this.trials = trials;
            if (!noise) stickerErrorMm = stickerErrorDeg = placementNoiseMm = placementNoiseDeg = 0f;
        }
    }

    public struct TrialResult
    {
        public bool success;
        public float timeToLock;  // seconds from board completion to the first all-locked frame; NaN if never
        public int lockedAtEnd;
        public int wrongAtEnd;    // locked pieces matched to a marker that sits in a different slot
        public int falseLocks;    // FromScatter/OneByOne: most pieces locked while still scattered
        public string detail;
    }

    public SimulatedTangramCamera sim;
    public TangramMatcher matcher;
    public List<Condition> conditions = new List<Condition>
    {
        new Condition("Clean, no noise", Protocol.SolvedBoard, CameraDegradationPreset.Clean, false, 10),
        new Condition("Typical+noise", Protocol.SolvedBoard, CameraDegradationPreset.Typical, true, 20),
        new Condition("Typical+noise from scatter", Protocol.FromScatter, CameraDegradationPreset.Typical, true, 10),
        new Condition("Harsh+noise", Protocol.SolvedBoard, CameraDegradationPreset.Harsh, true, 10),
        new Condition("Typical+noise one by one", Protocol.OneByOne, CameraDegradationPreset.Typical, true, 10),
    };

    [Header("Run")]
    public bool runOnStart = true;
    public bool quitWhenDone = false;
    public int baseSeed = 12345;
    [Tooltip("Seconds after the board is complete within which every piece must lock.")]
    public float settleSeconds = 4f;
    [Tooltip("Every piece must stay locked (and correctly assigned) for this long at the end of the settle window.")]
    public float holdSeconds = 1f;
    [Tooltip("OneByOne: seconds between placing pieces.")]
    public float stepSeconds = 1.5f;
    [Tooltip("The matcher logs thousands of lines per second; mute everything below Error while trials run.")]
    public bool muteVerboseLogs = true;
    [Tooltip("Print one line per trial (which pieces locked, to which marker).")]
    public bool logTrials = true;

    public bool IsRunning { get; private set; }
    public string Summary { get; private set; } = "";
    private string progress = "";

    IEnumerator Start()
    {
        if (sim == null) sim = GetComponent<SimulatedTangramCamera>();
        if (sim == null) sim = FindFirstObjectByType<SimulatedTangramCamera>();
        if (matcher == null) matcher = FindFirstObjectByType<TangramMatcher>();
        if (!runOnStart) yield break;
        yield return RunAll(null);
        if (quitWhenDone) Application.Quit();
    }

    /// <summary>Runs every condition and returns per-condition results. <paramref name="label"/> prefixes the report lines.</summary>
    public IEnumerator RunAll(string label, List<(Condition c, List<TrialResult> r)> collect = null)
    {
        float t0 = Time.time;
        while ((sim == null || !sim.IsReady) && Time.time - t0 < 15f) yield return null;
        if (sim == null || !sim.IsReady || matcher == null) { Report("SimLockRate: simulator or matcher not available."); yield break; }

        IsRunning = true;
        var prevFilter = Debug.unityLogger.filterLogType;
        if (muteVerboseLogs) Debug.unityLogger.filterLogType = LogType.Error;
        var savedSetup = JsonUtility.ToJson(sim.setup);
        var savedPreset = sim.preset;
        string prefix = string.IsNullOrEmpty(label) ? "" : $"[{label}] ";
        var summary = new StringBuilder($"{prefix}SimLockRate summary (settle {settleSeconds:0.#}s, hold {holdSeconds:0.#}s)\n");

        for (int ci = 0; ci < conditions.Count; ci++)
        {
            var c = conditions[ci];
            sim.preset = c.preset;
            sim.setup.stickerErrorMm = c.stickerErrorMm; sim.setup.stickerErrorDeg = c.stickerErrorDeg;
            sim.setup.placementNoiseMm = c.placementNoiseMm; sim.setup.placementNoiseDeg = c.placementNoiseDeg;
            var results = new List<TrialResult>();
            offsetSum = Vector2.zero; offsetCount = 0;
            for (int k = 0; k < c.trials; k++)
            {
                progress = $"{prefix}{c.name}: trial {k + 1}/{c.trials}";
                Random.InitState(baseSeed + ci * 1000 + k);
                var r = new TrialResult();
                yield return RunTrial(c, res => r = res);
                results.Add(r);
                if (logTrials)
                    Report($"{prefix}{c.name} #{k + 1}: {(r.success ? "LOCKED" : "FAILED")} t={(float.IsNaN(r.timeToLock) ? "-" : r.timeToLock.ToString("0.00") + "s")} " +
                           $"locked={r.lockedAtEnd}/{PieceCount} wrong={r.wrongAtEnd}{(c.protocol != Protocol.SolvedBoard ? $" falseLocks={r.falseLocks}" : "")} | {r.detail}");
            }
            collect?.Add((c, results));
            summary.AppendLine(prefix + FormatSummary(c, results) + OffsetReport());
        }

        JsonUtility.FromJsonOverwrite(savedSetup, sim.setup);
        sim.preset = savedPreset;
        Summary = summary.ToString();
        progress = "";
        Report(Summary);
        Debug.unityLogger.filterLogType = prevFilter;
        IsRunning = false;
    }

    public static string FormatSummary(Condition c, List<TrialResult> results)
    {
        int ok = 0, wrong = 0, falseLocks = 0, falseTrials = 0; float lockedSum = 0f; var times = new List<float>();
        foreach (var r in results)
        {
            if (r.success) ok++;
            lockedSum += r.lockedAtEnd;
            if (r.wrongAtEnd > 0) wrong++;
            falseLocks = Mathf.Max(falseLocks, r.falseLocks);
            if (r.falseLocks > 0) falseTrials++;
            if (!float.IsNaN(r.timeToLock)) times.Add(r.timeToLock);
        }
        times.Sort();
        string median = times.Count > 0 ? times[times.Count / 2].ToString("0.00") + "s" : "-";
        return $"{c.name,-28} lock rate {ok}/{results.Count} ({100f * ok / Mathf.Max(1, results.Count):0}%) | mean locked {lockedSum / Mathf.Max(1, results.Count):0.0}/7 | " +
               $"trials with a wrong-slot lock {wrong} | median time-to-lock {median}" +
               (c.protocol != Protocol.SolvedBoard ? $" | false locks while scattered in {falseTrials} trials (max {falseLocks})" : "");
    }

    private int PieceCount => matcher.GetDiagramPieces().Count;

    private IEnumerator RunTrial(Condition c, Action<TrialResult> done)
    {
        var world = sim.World;
        var r = new TrialResult { timeToLock = float.NaN };
        world.RestickMarkers(sim.setup);
        world.Scatter(sim.setup);
        yield return new WaitForSeconds(1.0f); // tracker (10 frames) and matcher (4 frames) buffers forget the old layout

        if (c.protocol == Protocol.SolvedBoard)
        {
            world.Solve(matcher, sim.setup, true);
            yield return new WaitForSeconds(0.5f);
            matcher.ResetMatchingState();
        }
        else
        {
            matcher.ResetMatchingState();
            float until = Time.time + 1.0f;
            while (Time.time < until) { r.falseLocks = Mathf.Max(r.falseLocks, CountLocked()); yield return null; }
            if (c.protocol == Protocol.FromScatter)
            {
                world.Solve(matcher, sim.setup, true);
            }
            else
            {
                for (int i = 0; i < world.Pieces.Count; i++)
                {
                    world.Solve(matcher, sim.setup, false);
                    yield return new WaitForSeconds(stepSeconds);
                }
            }
        }

        var trueSlot = ComputeTrueSlots();
        float start = Time.time, holdFrom = start + settleSeconds - holdSeconds;
        bool held = true;
        while (Time.time - start < settleSeconds)
        {
            bool boardOk = Evaluate(trueSlot, out _, out _);
            if (boardOk && float.IsNaN(r.timeToLock)) r.timeToLock = Time.time - start;
            if (Time.time >= holdFrom && !boardOk) held = false;
            yield return null;
        }
        bool okNow = Evaluate(trueSlot, out r.lockedAtEnd, out r.wrongAtEnd);
        AccumulateOrientationOffset(trueSlot);
        r.success = held && okNow;
        r.detail = Describe(trueSlot) + (r.success ? "" : " | " + DetectedIds());
        done(r);
    }

    /// <summary>Marker id → answer piece whose slot the physical piece occupies (nearest same-type target).</summary>
    private Dictionary<int, Transform> ComputeTrueSlots()
    {
        var targets = SimTangramWorld.ComputeTargets(matcher, sim.setup.setSizeMeters);
        var map = new Dictionary<int, Transform>();
        foreach (var p in sim.World.Pieces)
        {
            float best = float.MaxValue;
            foreach (var t in targets)
            {
                if (t.type != p.type) continue;
                float d = (t.pos - p.pos).sqrMagnitude;
                if (d < best) { best = d; map[p.markerId] = t.piece; }
            }
        }
        return map;
    }

    // ---------- angleOffset check ----------
    // The matcher's absolute-orientation test adds angleOffset to (marker +X vs piece center→direction). For every matched
    // triangle (no rotational symmetry) the offset that would zero that error is known from the true slot; their circular
    // mean is the angleOffset this camera geometry needs.
    private Vector2 offsetSum;
    private int offsetCount;

    private void AccumulateOrientationOffset(Dictionary<int, Transform> trueSlot)
    {
        Vector3 n = matcher.useXYPlane ? Vector3.forward : (matcher.tangramDiagramRoot != null ? matcher.tangramDiagramRoot.up : Vector3.up);
        foreach (var m in matcher.LastMatchResults)
        {
            if (m.shapeType != TangramMatcher.TangramShapeType.LargeTriangle && m.shapeType != TangramMatcher.TangramShapeType.MediumTriangle &&
                m.shapeType != TangramMatcher.TangramShapeType.SmallTriangle) continue;
            if (!trueSlot.TryGetValue(m.arucoId, out var piece) || piece == null) continue;
            Transform c = piece.Find("center"), d = piece.Find("direction");
            Vector3 pieceDir = c != null && d != null ? d.position - c.position : piece.forward;
            Vector3 a = Vector3.ProjectOnPlane(pieceDir, n), b = Vector3.ProjectOnPlane(m.detectionWorldRotation * Vector3.right, n);
            if (a.sqrMagnitude < 1e-8f || b.sqrMagnitude < 1e-8f) continue;
            float needed = -Vector3.SignedAngle(a, b, n) * Mathf.Deg2Rad;
            offsetSum += new Vector2(Mathf.Cos(needed), Mathf.Sin(needed));
            offsetCount++;
        }
    }

    private string OffsetReport()
    {
        if (offsetCount == 0) return "";
        float mean = Mathf.Repeat(Mathf.Atan2(offsetSum.y, offsetSum.x) * Mathf.Rad2Deg, 360f);
        float spread = Mathf.Sqrt(-2f * Mathf.Log(Mathf.Clamp(offsetSum.magnitude / offsetCount, 1e-6f, 1f))) * Mathf.Rad2Deg; // circular std
        float mismatch = Mathf.Abs(Mathf.DeltaAngle(mean, matcher.angleOffset));
        return $" | angleOffset that zeroes orientation error: {mean:0}° ±{spread:0.0}° (n={offsetCount}, current {matcher.angleOffset:0}°)" +
               (mismatch > 20f ? $" ← MISMATCH {mismatch:0}°: the simulated camera does not match the matcher's calibration (check sensorRotationDeg)" : "");
    }

    private int CountLocked()
    {
        int n = 0;
        foreach (var p in matcher.GetDiagramPieces()) if (matcher.IsPieceLocked(p)) n++;
        return n;
    }

    private bool Evaluate(Dictionary<int, Transform> trueSlot, out int locked, out int wrong)
    {
        var pieces = matcher.GetDiagramPieces();
        locked = 0; wrong = 0;
        foreach (var p in pieces) if (matcher.IsPieceLocked(p)) locked++;
        foreach (var m in matcher.LastMatchResults)
            if (matcher.IsPieceLocked(m.matchedPieceTransform) && trueSlot.TryGetValue(m.arucoId, out var slot) && slot != m.matchedPieceTransform)
                wrong++;
        return pieces.Count > 0 && locked == pieces.Count && wrong == 0;
    }

    /// <summary>Per piece: L = locked, m = matched only, - = neither; then the marker matched to it (✗ = wrong slot).</summary>
    private string Describe(Dictionary<int, Transform> trueSlot)
    {
        var sb = new StringBuilder();
        foreach (var p in matcher.GetDiagramPieces())
        {
            int id = -1;
            foreach (var m in matcher.LastMatchResults) if (m.matchedPieceTransform == p) { id = m.arucoId; break; }
            bool locked = matcher.IsPieceLocked(p);
            sb.Append(p.name).Append('=').Append(locked ? "L" : id >= 0 ? "m" : "-");
            if (id >= 0) sb.Append(id).Append(trueSlot.TryGetValue(id, out var s) && s != p ? "✗" : "");
            sb.Append(' ');
        }
        return sb.ToString().TrimEnd();
    }

    private ArucoDetector detector;

    /// <summary>Markers an ArUco detector finds in the latest frame (same dictionary and default parameters as the tracker).</summary>
    private string DetectedIds()
    {
        if (detector == null) detector = new ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50), new DetectorParameters());
        using (var img = new Mat())
        using (var ids = new Mat())
        {
            if (!sim.CopyLatestFrame(img)) return "detected n/a";
            if (sim.ActiveDegradation.mirrorFlip) Core.flip(img, img, 1);
            var corners = new List<Mat>();
            detector.detectMarkers(img, corners, ids);
            var found = new List<int>();
            for (int i = 0; i < (int)ids.total(); i++) found.Add((int)ids.get(i, 0)[0]);
            foreach (var c in corners) c.Dispose();
            found.Sort();
            return $"detected [{string.Join(",", found)}]";
        }
    }

    private static void Report(string line)
    {
        // Print even while verbose logs are muted.
        var prev = Debug.unityLogger.filterLogType;
        Debug.unityLogger.filterLogType = LogType.Log;
        Debug.Log(line);
        Debug.unityLogger.filterLogType = prev;
    }

    void OnGUI()
    {
        string text = IsRunning ? progress : Summary;
        if (string.IsNullOrEmpty(text)) return;
        GUI.Label(new UnityEngine.Rect(10, 10, Screen.width - 20, 200), text);
    }
}
