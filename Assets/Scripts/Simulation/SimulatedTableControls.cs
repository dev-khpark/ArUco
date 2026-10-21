using UnityEngine;
using UnityEngine.UI;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Editor test controls for the virtual table: a top-down panel where pieces can be dragged and rotated,
/// plus shortcuts to scatter/solve, re-stick markers, show a hand and switch image quality.
/// Added automatically by <see cref="SimulatedTangramCamera"/>.
/// </summary>
public class SimulatedTableControls : MonoBehaviour
{
    public bool showPanel = true;
    [Tooltip("Show a hand over the piece being dragged (occlusion + shadow, like a real player).")]
    public bool handWhileDragging = true;
    public float rotateStepDeg = 15f;

    private SimulatedTangramCamera sim;
    private Camera topCam;
    private RenderTexture topTarget;
    private GameObject panelRoot;
    private RectTransform panelRect;
    private Text statusText;
    private SimTangramWorld.Piece dragging;
    private Vector2 grabOffset;
    private bool hoverHand;

    void Start()
    {
        sim = GetComponent<SimulatedTangramCamera>();
        if (sim == null || !sim.IsReady) { enabled = false; return; }

        topTarget = new RenderTexture(640, 640, 24, RenderTextureFormat.ARGB32) { antiAliasing = 2, name = "SimTopView" };
        var go = new GameObject("SimTopViewCamera");
        go.transform.SetParent(sim.World.Root, false);
        go.transform.localPosition = new Vector3(0f, 0.5f, 0f);
        go.transform.localRotation = Quaternion.Euler(90f, 0f, 0f); // straight down, +v up in the panel
        topCam = go.AddComponent<Camera>();
        topCam.orthographic = true;
        topCam.orthographicSize = sim.World.BoardSize * 0.5f * 1.25f;
        topCam.nearClipPlane = 0.05f;
        topCam.farClipPlane = 1f;
        topCam.clearFlags = CameraClearFlags.SolidColor;
        topCam.backgroundColor = new Color(0.15f, 0.15f, 0.15f);
        topCam.targetTexture = topTarget;

        BuildPanel();
    }

    void Update()
    {
        if (sim == null || !sim.IsReady) return;
        HandleKeys();
        HandleMouse();
        if (statusText != null)
        {
            statusText.text = $"SIM · 화질 {sim.preset}{(sim.useCustomDegradation ? "(사용자)" : "")} · 보정 {(sim.provideTrueCalibration ? "ON" : "OFF")} · 프레임 {sim.FramesProduced}\n" +
                              "드래그 이동 · 휠/Q/E 회전\n[1] 흩기 [2] 정답 [3] 한 조각 [R] 스티커 재부착\n[H] 손 [P] 화질 [C] 보정 [Tab] 패널";
        }
    }

    private void HandleKeys()
    {
        if (Pressed(KeyCode.Tab)) { showPanel = !showPanel; panelRoot.SetActive(showPanel); }
        if (Pressed(KeyCode.Alpha1)) sim.World.Scatter(sim.setup);
        if (Pressed(KeyCode.Alpha2)) sim.World.Solve(sim.matcher, sim.setup, true);
        if (Pressed(KeyCode.Alpha3)) sim.World.Solve(sim.matcher, sim.setup, false);
        if (Pressed(KeyCode.R)) sim.World.RestickMarkers(sim.setup);
        if (Pressed(KeyCode.H)) { hoverHand = !hoverHand; sim.World.Hand.gameObject.SetActive(hoverHand); }
        if (Pressed(KeyCode.P)) sim.CyclePreset();
        if (Pressed(KeyCode.C)) sim.provideTrueCalibration = !sim.provideTrueCalibration;
    }

    private void HandleMouse()
    {
        bool overPanel = TryGetBoardPoint(out Vector2 p);
        if (overPanel && hoverHand) sim.World.PlaceHand(p, 0.06f);

        if (MouseDown() && overPanel)
        {
            dragging = sim.World.PickPiece(p);
            if (dragging != null)
            {
                grabOffset = dragging.pos - p;
                dragging.solved = false;
                if (handWhileDragging) sim.World.Hand.gameObject.SetActive(true);
            }
        }

        var target = dragging ?? (overPanel ? sim.World.PickPiece(p) : null);
        float rot = Scroll() * 7.5f;
        if (Pressed(KeyCode.Q)) rot += rotateStepDeg;
        if (Pressed(KeyCode.E)) rot -= rotateStepDeg;
        if (target != null && Mathf.Abs(rot) > 0.01f) { sim.World.SetPose(target, target.pos, target.angleDeg + rot); target.solved = false; }

        if (dragging != null)
        {
            if (overPanel) sim.World.SetPose(dragging, p + grabOffset, dragging.angleDeg);
            if (handWhileDragging) sim.World.PlaceHand(dragging.pos, 0.045f); // fingers just above the piece
            if (MouseUp())
            {
                dragging = null;
                if (!hoverHand) sim.World.Hand.gameObject.SetActive(false);
            }
        }
    }

    /// <summary>Mouse position over the panel → point on the board (meters, board coordinates).</summary>
    private bool TryGetBoardPoint(out Vector2 boardPoint)
    {
        boardPoint = default;
        if (!showPanel || panelRect == null) return false;
        Vector2 mouse = MousePosition();
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(panelRect, mouse, null, out Vector2 local)) return false;
        Rect r = panelRect.rect;
        Vector2 vp = new Vector2((local.x - r.xMin) / r.width, (local.y - r.yMin) / r.height);
        if (vp.x < 0f || vp.x > 1f || vp.y < 0f || vp.y > 1f) return false;
        Ray ray = topCam.ViewportPointToRay(vp);
        var plane = new Plane(sim.World.Root.up, sim.World.Root.position);
        if (!plane.Raycast(ray, out float t)) return false;
        Vector3 l = sim.World.Root.InverseTransformPoint(ray.GetPoint(t));
        boardPoint = new Vector2(l.x, l.z);
        return true;
    }

    private void BuildPanel()
    {
        var canvasGo = new GameObject("SimControlsCanvas", typeof(Canvas), typeof(CanvasScaler));
        canvasGo.transform.SetParent(transform, false);
        var canvas = canvasGo.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 200;
        var scaler = canvasGo.GetComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1080f, 1920f);
        scaler.matchWidthOrHeight = 0.5f;

        panelRoot = new GameObject("Panel", typeof(RectTransform), typeof(Image));
        panelRoot.transform.SetParent(canvasGo.transform, false);
        var bg = panelRoot.GetComponent<Image>();
        bg.color = new Color(0f, 0f, 0f, 0.7f);
        bg.raycastTarget = false;
        var rootRect = (RectTransform)panelRoot.transform;
        rootRect.anchorMin = rootRect.anchorMax = rootRect.pivot = new Vector2(1f, 0.5f);
        rootRect.anchoredPosition = new Vector2(-16f, -120f);
        rootRect.sizeDelta = new Vector2(400f, 570f);

        var view = new GameObject("TopView", typeof(RectTransform), typeof(RawImage));
        view.transform.SetParent(panelRoot.transform, false);
        view.GetComponent<RawImage>().texture = topTarget;
        view.GetComponent<RawImage>().raycastTarget = false;
        panelRect = (RectTransform)view.transform;
        panelRect.anchorMin = panelRect.anchorMax = panelRect.pivot = new Vector2(0.5f, 1f);
        panelRect.anchoredPosition = new Vector2(0f, -10f);
        panelRect.sizeDelta = new Vector2(380f, 380f);

        var textGo = new GameObject("Status", typeof(RectTransform), typeof(Text));
        textGo.transform.SetParent(panelRoot.transform, false);
        statusText = textGo.GetComponent<Text>();
        try { statusText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf"); } catch { }
        statusText.fontSize = 18;
        statusText.color = Color.white;
        statusText.raycastTarget = false;
        var tr = (RectTransform)textGo.transform;
        tr.anchorMin = new Vector2(0f, 0f); tr.anchorMax = new Vector2(1f, 0f); tr.pivot = new Vector2(0.5f, 0f);
        tr.anchoredPosition = new Vector2(0f, 8f);
        tr.sizeDelta = new Vector2(-20f, 165f);

        panelRoot.SetActive(showPanel);
    }

    void OnDestroy()
    {
        if (topTarget != null) topTarget.Release();
    }

    // ---------- Input (works with either input backend) ----------

#if ENABLE_INPUT_SYSTEM
    private static Key ToKey(KeyCode k)
    {
        switch (k)
        {
            case KeyCode.Tab: return Key.Tab;
            case KeyCode.Alpha1: return Key.Digit1;
            case KeyCode.Alpha2: return Key.Digit2;
            case KeyCode.Alpha3: return Key.Digit3;
            case KeyCode.R: return Key.R;
            case KeyCode.H: return Key.H;
            case KeyCode.P: return Key.P;
            case KeyCode.C: return Key.C;
            case KeyCode.Q: return Key.Q;
            case KeyCode.E: return Key.E;
            default: return Key.None;
        }
    }
    private static bool Pressed(KeyCode k) => Keyboard.current != null && Keyboard.current[ToKey(k)].wasPressedThisFrame;
    private static Vector2 MousePosition() => Mouse.current != null ? Mouse.current.position.ReadValue() : Vector2.zero;
    private static bool MouseDown() => Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame;
    private static bool MouseUp() => Mouse.current == null || Mouse.current.leftButton.wasReleasedThisFrame;
    private static float Scroll() => Mouse.current != null ? Mathf.Sign(Mouse.current.scroll.ReadValue().y) * (Mathf.Abs(Mouse.current.scroll.ReadValue().y) > 0.01f ? 1f : 0f) : 0f;
#else
    private static bool Pressed(KeyCode k) => Input.GetKeyDown(k);
    private static Vector2 MousePosition() => Input.mousePosition;
    private static bool MouseDown() => Input.GetMouseButtonDown(0);
    private static bool MouseUp() => Input.GetMouseButtonUp(0);
    private static float Scroll() => Mathf.Sign(Input.mouseScrollDelta.y) * (Mathf.Abs(Input.mouseScrollDelta.y) > 0.01f ? 1f : 0f);
#endif
}
