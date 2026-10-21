using System.Collections.Generic;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityUtils;
using UnityEngine;

/// <summary>
/// Virtual stand-in for "front camera + clip-on mirror looking down at the table".
///
/// A planar mirror is optically identical to a camera placed at the mirror image of the real one, plus a
/// left-right reversal. So this renders the virtual table from that equivalent viewpoint, rotates it like a
/// portrait phone's landscape sensor, and runs <see cref="CameraDegradation"/> (mirror flip, ghosting, lens
/// distortion, blur, lighting, noise, JPEG). ArUcoMobileTracker pulls frames from here instead of WebCamTexture,
/// so detection, matching and the game run unchanged.
///
/// Editor-only by default; on device the real camera is always used.
/// </summary>
[DefaultExecutionOrder(-50)] // produce the frame before ArUcoMobileTracker.Update consumes it
public class SimulatedTangramCamera : MonoBehaviour
{
    [Header("Activation")]
    [Tooltip("Replace the device camera with this virtual mirror camera when playing in the Editor.")]
    public bool simulateInEditor = true;
    [Tooltip("Also simulate in player builds (automated tests only). Keep off for real devices.")]
    public bool allowInPlayerBuilds = false;
    public TangramMatcher matcher;

    public SimSetup setup = new SimSetup();

    [Header("Mirror camera (equivalent viewpoint)")]
    public Vector2Int resolution = new Vector2Int(1920, 1080);
    [Tooltip("Field of view across the long image axis. Phone front cameras are typically 65–80°.")]
    public float longAxisFovDeg = 70f;
    [Tooltip("Height of the equivalent (mirrored) viewpoint above the table. 0.40 m frames a 24 cm board with ~120 px margin at 70° FOV.")]
    public float cameraHeightMeters = 0.40f;
    [Tooltip("Horizontal offset of the viewpoint from the board center toward the device. Larger = more oblique view.")]
    public float cameraBackMeters = 0.10f;
    [Tooltip("Sensor image rotation relative to the player's view. A portrait phone's landscape sensor runs along the table depth, so 90 or 270.\n" +
             "270 reproduces the device: TangramMatcher.angleOffset (tuned on device to 280) then fits within ~10°. With 90 every triangle looks rotated " +
             "by 180° to the matcher (it needs angleOffset ≈ 90), so congruent pieces swap slots. SimLockRateHarness reports the fitted offset.")]
    public float sensorRotationDeg = 270f;
    public float frameRate = 30f;

    [Header("Image degradation")]
    public CameraDegradationPreset preset = CameraDegradationPreset.Typical;
    [Tooltip("Use the values below instead of the preset.")]
    public bool useCustomDegradation = false;
    public CameraDegradationSettings custom = new CameraDegradationSettings();

    [Header("Tracker calibration")]
    [Tooltip("Off: the tracker keeps its own hard-coded camera matrix and marker length, as on device today.\nOn: it receives this camera's true intrinsics, distortion and marker size.")]
    public bool provideTrueCalibration = false;

    public bool ShouldSimulate => isActiveAndEnabled && (Application.isEditor ? simulateInEditor : allowInPlayerBuilds);
    public SimTangramWorld World { get; private set; }
    public bool IsReady => World != null && frame != null;
    public float FocalPx => resolution.x * 0.5f / Mathf.Tan(longAxisFovDeg * 0.5f * Mathf.Deg2Rad);
    public CameraDegradationSettings ActiveDegradation => useCustomDegradation ? custom : presetSettings;
    public int FramesProduced { get; private set; }

    private CameraDegradationSettings presetSettings;
    private CameraDegradationPreset builtPreset;
    private CameraDegradation degradation;
    private Camera cam;
    private RenderTexture msaaTarget, resolved;
    private Texture2D readback;
    private Mat frame;
    private bool frameReady;
    private float nextFrameTime;

    void Start()
    {
        if (!ShouldSimulate) return;
        if (matcher == null) matcher = FindFirstObjectByType<TangramMatcher>();
        if (matcher == null) { Debug.LogError("SimulatedTangramCamera: no TangramMatcher in scene."); return; }

        // Far below the game scene so neither camera sees the other's objects.
        var worldRoot = new GameObject("TangramSimWorld").transform;
        worldRoot.position = new Vector3(0f, -1000f, 0f);
        World = new SimTangramWorld();
        foreach (var w in World.Build(worldRoot, matcher, setup)) Debug.LogWarning("[Sim] " + w);

        presetSettings = CameraDegradationSettings.FromPreset(preset);
        builtPreset = preset;
        degradation = new CameraDegradation();

        msaaTarget = new RenderTexture(resolution.x, resolution.y, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 4, name = "SimMirrorCamera_MSAA" };
        resolved = new RenderTexture(resolution.x, resolution.y, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { name = "SimMirrorCamera" };
        readback = new Texture2D(resolution.x, resolution.y, TextureFormat.RGBA32, false);
        frame = new Mat(resolution.y, resolution.x, CvType.CV_8UC3);

        var camGo = new GameObject("SimMirrorCamera");
        camGo.transform.SetParent(World.Root, false);
        cam = camGo.AddComponent<Camera>();
        cam.targetTexture = msaaTarget;
        cam.nearClipPlane = 0.02f;
        cam.farClipPlane = 3f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.2f, 0.18f, 0.16f);
        UpdateCameraPose();
        foreach (var w in CheckFraming()) Debug.LogWarning("[Sim] " + w);

        if (GetComponent<SimulatedTableControls>() == null) gameObject.AddComponent<SimulatedTableControls>();
        Debug.Log($"[Sim] Virtual mirror camera active: {resolution.x}x{resolution.y}, {longAxisFovDeg}° FOV, set {setup.setSizeMeters * 100f:0.#}cm, marker {setup.markerSizeMeters * 100f:0.#}cm, preset {preset}.");
    }

    void Update()
    {
        if (!IsReady) return;
        if (preset != builtPreset) { presetSettings = CameraDegradationSettings.FromPreset(preset); builtPreset = preset; }
        UpdateCameraPose(); // live-tweakable from the Inspector

        if (Time.time < nextFrameTime) return;
        nextFrameTime = Mathf.Max(nextFrameTime + 1f / Mathf.Max(1f, frameRate), Time.time - 0.1f);
        if (degradation.ShouldDrop(ActiveDegradation)) return;

        // Read last frame's render. Blit resolves MSAA so edges are anti-aliased like a real sensor.
        Graphics.Blit(msaaTarget, resolved);
        var prev = RenderTexture.active;
        RenderTexture.active = resolved;
        readback.ReadPixels(new UnityEngine.Rect(0, 0, resolution.x, resolution.y), 0, 0, false);
        RenderTexture.active = prev;
        Utils.texture2DToMat(readback, frame); // flips to top-left origin, RGBA → RGB

        degradation.Apply(frame, ActiveDegradation, FocalPx, Time.time);
        frameReady = true;
        FramesProduced++;
    }

    /// <summary>Copies the newest frame into <paramref name="dst"/> (reallocated as needed). False if no new frame.</summary>
    public bool TryGetFrame(Mat dst)
    {
        if (!frameReady || dst == null) return false;
        frameReady = false;
        frame.copyTo(dst);
        return true;
    }

    /// <summary>Copies the most recent frame without consuming it (for evaluation tools).</summary>
    public bool CopyLatestFrame(Mat dst)
    {
        if (!IsReady || FramesProduced == 0 || dst == null) return false;
        frame.copyTo(dst);
        return true;
    }

    /// <summary>
    /// True pinhole intrinsics of the frame the tracker sees (after it un-flips the mirror image),
    /// plus OpenCV distortion (k1, k2, p1, p2) and the physical marker edge length.
    /// </summary>
    public void GetCalibration(out double fx, out double fy, out double cx, out double cy, out double[] dist, out float markerLength)
    {
        fx = fy = FocalPx;
        cx = resolution.x * 0.5;
        cy = resolution.y * 0.5;
        var d = ActiveDegradation;
        dist = new double[] { d.k1, d.k2, 0, 0 };
        markerLength = setup.markerSizeMeters;
    }

    public void CyclePreset() => preset = (CameraDegradationPreset)(((int)preset + 1) % 3);

    private void UpdateCameraPose()
    {
        if (cam == null) return;
        Vector3 pos = new Vector3(0f, cameraHeightMeters, cameraBackMeters);
        Vector3 fwd = (-pos).normalized; // look at the board center
        var look = Quaternion.LookRotation(fwd, Vector3.forward); // image up = away from the player
        cam.transform.localPosition = pos;
        cam.transform.localRotation = Quaternion.AngleAxis(sensorRotationDeg, fwd) * look;

        // Render wider than the output so barrel distortion can pull pixels in from outside the frame.
        var d = ActiveDegradation;
        float renderFocal = FocalPx * CameraDegradation.RenderFocalScale(d.k1, d.k2, resolution.x, resolution.y, FocalPx);
        cam.fieldOfView = 2f * Mathf.Atan(resolution.y * 0.5f / renderFocal) * Mathf.Rad2Deg;
    }

    /// <summary>Warns when board corner markers fall outside (or right at the edge of) the distorted output frame.</summary>
    public List<string> CheckFraming()
    {
        var issues = new List<string>();
        if (cam == null || World == null) return issues;
        var d = ActiveDegradation;
        float scale = CameraDegradation.RenderFocalScale(d.k1, d.k2, resolution.x, resolution.y, FocalPx);
        float fr = FocalPx * scale, cx = resolution.x * 0.5f, cy = resolution.y * 0.5f;
        float h = World.BoardSize * 0.5f;
        float margin = 0.03f * resolution.y;
        foreach (var (name, u, v) in new[] { ("TL", -h, h), ("TR", h, h), ("BR", h, -h), ("BL", -h, -h) })
        {
            Vector3 vp = cam.WorldToViewportPoint(World.Root.TransformPoint(new Vector3(u, 0f, v)));
            float xu = (vp.x * resolution.x - cx) / fr, yu = (vp.y * resolution.y - cy) / fr;
            float r2 = xu * xu + yu * yu, k = 1f + d.k1 * r2 + d.k2 * r2 * r2;
            float px = FocalPx * xu * k + cx, py = FocalPx * yu * k + cy;
            if (vp.z <= 0f || px < margin || py < margin || px > resolution.x - margin || py > resolution.y - margin)
                issues.Add($"Board corner {name} is outside or at the edge of the camera frame ({px:0},{py:0}). Raise cameraHeightMeters, widen longAxisFovDeg or shrink the board.");
        }
        return issues;
    }

    void OnDestroy()
    {
        degradation?.Dispose();
        frame?.Dispose();
        if (msaaTarget != null) msaaTarget.Release();
        if (resolved != null) resolved.Release();
    }
}
