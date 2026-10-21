using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

/// <summary>
/// Runtime-built HUD for TangramGameManager: top bar (puzzle, progress, timer, hint/restart), piece chips,
/// status banner, toasts, countdown, intro/result cards, hint ring over the answer diagram and confetti.
///
/// Everything (canvas, sprites, panels) is created in code so the scene needs no UI setup.
/// Legacy uGUI Text is used on purpose: its dynamic font falls back to OS fonts, which renders Korean
/// without bundling a CJK font asset (TextMeshPro would need one).
/// </summary>
public class TangramGameHUD : MonoBehaviour
{
    [Header("Style")]
    public Color panelColor = new Color(0.07f, 0.08f, 0.11f, 0.8f);
    public Color accentColor = new Color(1f, 0.77f, 0.2f, 1f);
    public Color buttonColor = new Color(0.24f, 0.52f, 0.96f, 1f);
    public Color warningColor = new Color(0.93f, 0.42f, 0.2f, 0.92f);
    public Color emptyChipColor = new Color(1f, 1f, 1f, 0.25f);
    public int sortingOrder = 50;

    private const float TopBarHeight = 130f;

    private TangramGameManager game;
    private TangramGameAudio audioFx;
    private TangramGameAudio Fx => audioFx != null ? audioFx : (audioFx = GetComponent<TangramGameAudio>());

    private Font font;
    private Sprite roundSprite, circleSprite, ringSprite, starSprite, checkSprite;
    private readonly Dictionary<TangramMatcher.TangramShapeType, Sprite> shapeSprites = new Dictionary<TangramMatcher.TangramShapeType, Sprite>();

    private Canvas canvas;
    private RectTransform canvasRect, safeArea, effectsLayer, confettiLayer;
    private Rect lastSafeArea;
    private Vector2 lastCanvasSize;

    // Top bar
    private GameObject topBar;
    private Text titleText, progressText, timerText, hintButtonText;
    private Button hintButton;
    private RectTransform progressFill;
    private float shownProgress;

    // Chips
    private class Chip
    {
        public TangramGameManager.PieceInfo piece;
        public RectTransform root;
        public Image background, icon, badge;
        public float popTime = -10f;
    }
    private RectTransform chipRow;
    private readonly List<Chip> chips = new List<Chip>();

    // Banner / toast / countdown
    private GameObject banner;
    private Image bannerBg;
    private Text bannerText;
    private string hintMessage;
    private Text toastText;
    private float toastTime = -10f;
    private Text countdownText;
    private float countdownTime = -10f;

    // Cards
    private GameObject introOverlay, resultOverlay;
    private RectTransform introCard, resultCard;
    private Text introPuzzleText, introBestText, resultTimeText, resultHintsText, resultBestText, resultPuzzleText;
    private Image[] introStars, resultStars;
    private Button prevPuzzleButton, nextPuzzleButtonIntro, resultNextButton, resultRetryButton;
    private float resultShownTime;
    private int resultStarCount;

    // Effects
    private Image hintRing, hintGlow;
    private class Burst { public RectTransform rt; public Image img; public float age; }
    private readonly List<Burst> bursts = new List<Burst>();
    private class Confetto { public RectTransform rt; public Image img; public Vector2 vel; public float rotVel, age, life, sway; }
    private readonly List<Confetto> confetti = new List<Confetto>();

    void Awake()
    {
        game = GetComponent<TangramGameManager>();
        if (game == null) game = FindFirstObjectByType<TangramGameManager>();
        if (game == null) { enabled = false; return; }

        LoadFont();
        BuildSprites();
        BuildCanvas();
        BuildTopBar();
        BuildBannerAndToast();
        BuildIntro();
        BuildResult();

        game.PuzzleLoaded += OnPuzzleLoaded;
        game.StateChanged += OnStateChanged;
        game.PiecePlaced += OnPiecePlaced;
        game.PieceLost += OnPieceLost;
        game.HintShown += OnHintShown;
        game.CountdownTick += OnCountdownTick;
        game.RoundCompleted += OnRoundCompleted;
    }

    void OnDestroy()
    {
        if (game == null) return;
        game.PuzzleLoaded -= OnPuzzleLoaded;
        game.StateChanged -= OnStateChanged;
        game.PiecePlaced -= OnPiecePlaced;
        game.PieceLost -= OnPieceLost;
        game.HintShown -= OnHintShown;
        game.CountdownTick -= OnCountdownTick;
        game.RoundCompleted -= OnRoundCompleted;
    }

    void Update()
    {
        ApplySafeAreaIfChanged();
        UpdateTopBar();
        UpdateChips();
        UpdateBanner();
        UpdateHintRing();
        UpdateToastAndCountdown();
        UpdateResultStars();
        UpdateBursts();
        UpdateConfetti();
    }

    // ---------- Event handlers ----------

    private void OnPuzzleLoaded()
    {
        RebuildChips();
        RefreshIntro();
    }

    private void OnStateChanged(TangramGameManager.GameState s)
    {
        introOverlay.SetActive(s == TangramGameManager.GameState.Intro);
        resultOverlay.SetActive(s == TangramGameManager.GameState.Completed);
        topBar.SetActive(s != TangramGameManager.GameState.Intro);
        chipRow.gameObject.SetActive(s != TangramGameManager.GameState.Intro);
        if (s == TangramGameManager.GameState.Intro) RefreshIntro();
        if (s != TangramGameManager.GameState.Playing) hintMessage = null;
        if (s == TangramGameManager.GameState.Ready) shownProgress = 0f;
    }

    private void OnPiecePlaced(TangramGameManager.PieceInfo p)
    {
        foreach (var c in chips) if (c.piece == p) c.popTime = Time.time;
        SpawnBurst(p);
        ShowToast($"{p.displayName} 성공!  {game.PlacedCount} / {game.TotalCount}");
        Fx?.PlayPlaced(game.PlacedCount, game.TotalCount);
    }

    private void OnPieceLost(TangramGameManager.PieceInfo p)
    {
        Fx?.PlayLost();
    }

    private void OnHintShown(TangramGameManager.PieceInfo p, string message)
    {
        hintMessage = "힌트 · " + message;
        foreach (var c in chips) if (c.piece == p) c.popTime = Time.time;
        Fx?.PlayHint();
    }

    private void OnCountdownTick(int remaining)
    {
        if (remaining < 0) { countdownTime = -10f; countdownText.gameObject.SetActive(false); return; }
        countdownText.text = remaining > 0 ? remaining.ToString() : "시작!";
        countdownText.gameObject.SetActive(true);
        countdownTime = Time.time;
        if (remaining > 0) Fx?.PlayCountdown(); else Fx?.PlayGo();
    }

    private void OnRoundCompleted(TangramGameManager.RoundResult r)
    {
        resultPuzzleText.text = r.puzzleName;
        resultTimeText.text = "기록  " + FormatTime(r.seconds, true);
        resultHintsText.text = r.hintsUsed == 0 ? "힌트 없이 완성!" : $"힌트 {r.hintsUsed}회 사용";
        resultBestText.text = r.isNewBest ? "신기록!" : "최고 기록  " + FormatTime(r.bestSeconds, true);
        resultBestText.color = r.isNewBest ? accentColor : new Color(1f, 1f, 1f, 0.75f);
        bool multi = game.PuzzleCount > 1;
        resultNextButton.gameObject.SetActive(multi);
        LayoutResultButtons(multi);
        resultStarCount = r.stars;
        resultShownTime = Time.time;
        foreach (var s in resultStars) s.rectTransform.localScale = Vector3.zero;
        SpawnConfetti();
        Fx?.PlayComplete();
    }

    // ---------- Per-frame updates ----------

    private void UpdateTopBar()
    {
        if (!topBar.activeSelf) return;
        titleText.text = game.CurrentPuzzleName;
        progressText.text = $"{game.PlacedCount} / {game.TotalCount} 조각";
        bool counting = game.State == TangramGameManager.GameState.Playing;
        timerText.text = FormatTime(game.ElapsedSeconds, false);
        timerText.color = counting && !game.BoardVisible ? new Color(1f, 1f, 1f, 0.45f) : Color.white;

        float cd = game.HintCooldownRemaining;
        hintButton.interactable = counting && cd <= 0f;
        hintButtonText.text = cd > 0f && counting ? $"힌트 {Mathf.CeilToInt(cd)}" : "힌트";

        float target = game.TotalCount > 0 ? game.PlacedCount / (float)game.TotalCount : 0f;
        shownProgress = Mathf.MoveTowards(shownProgress, target, Time.deltaTime * 2f);
        progressFill.anchorMax = new Vector2(Mathf.Max(0.0001f, shownProgress), 1f);
    }

    private void UpdateChips()
    {
        if (!chipRow.gameObject.activeSelf) return;
        foreach (var c in chips)
        {
            var status = c.piece.status;
            Color iconColor = status == TangramGameManager.PieceStatus.Placed ? c.piece.color : emptyChipColor;
            Color bg = panelColor;
            if (status == TangramGameManager.PieceStatus.Detected)
            {
                // "Seen but not verified yet": gentle pulse in the accent color.
                float pulse = 0.5f + 0.5f * Mathf.Sin(Time.time * 6f);
                bg = Color.Lerp(panelColor, accentColor * new Color(1f, 1f, 1f, 0.6f), 0.35f + 0.35f * pulse);
                iconColor = Color.Lerp(emptyChipColor, c.piece.color, 0.45f);
            }
            if (game.HintPiece == c.piece)
            {
                float pulse = 0.5f + 0.5f * Mathf.Sin(Time.time * 8f);
                bg = Color.Lerp(panelColor, accentColor, 0.4f + 0.4f * pulse);
            }
            c.background.color = bg;
            c.icon.color = iconColor;
            c.badge.gameObject.SetActive(status == TangramGameManager.PieceStatus.Placed);

            float t = (Time.time - c.popTime) / 0.35f;
            float s = t >= 0f && t < 1f ? 1f + 0.25f * Mathf.Sin(t * Mathf.PI) : 1f;
            c.root.localScale = new Vector3(s, s, 1f);
        }
    }

    private void UpdateBanner()
    {
        string msg = null;
        Color bg = panelColor;
        switch (game.State)
        {
            case TangramGameManager.GameState.Ready:
                if (!game.BoardVisible) { msg = "보드 네 모서리의 마커가 모두 화면에 보이게 비춰 주세요"; bg = warningColor; }
                else if (game.ReadyBlockedByPlacedPieces) { msg = "조각을 보드 위에 흩어 놓아 주세요"; bg = warningColor; }
                else msg = "준비! 곧 시작합니다";
                break;
            case TangramGameManager.GameState.Playing:
                if (!game.BoardVisible) { msg = "보드가 보이지 않아요 · 타이머 일시정지"; bg = warningColor; }
                else if (game.HintPiece != null && hintMessage != null) { msg = hintMessage; bg = new Color(accentColor.r * 0.55f, accentColor.g * 0.45f, 0.05f, 0.92f); }
                else if (game.PlacedCount == 0) msg = "화면 아래 그림과 같은 모양을 만들어 보세요";
                break;
        }
        banner.SetActive(msg != null);
        if (msg != null)
        {
            bannerText.text = msg;
            bannerBg.color = bg;
        }
    }

    private void UpdateHintRing()
    {
        var piece = game.HintPiece;
        Vector2 center = Vector2.zero;
        float size = 0f;
        bool show = piece != null && game.State == TangramGameManager.GameState.Playing
            && TryGetPieceScreenRect(piece.transform, out center, out size);
        hintRing.gameObject.SetActive(show);
        hintGlow.gameObject.SetActive(show);
        if (!show) return;

        float pulse = 0.5f + 0.5f * Mathf.Sin(Time.time * 6f);
        float s = size * (1.15f + 0.15f * pulse);
        foreach (var img in new[] { hintRing, hintGlow })
        {
            img.rectTransform.anchoredPosition = center;
            img.rectTransform.sizeDelta = new Vector2(s, s);
        }
        hintRing.color = new Color(accentColor.r, accentColor.g, accentColor.b, 0.6f + 0.4f * pulse);
        hintGlow.color = new Color(accentColor.r, accentColor.g, accentColor.b, 0.18f + 0.12f * pulse);
    }

    private void UpdateToastAndCountdown()
    {
        float tt = Time.time - toastTime;
        bool toastOn = tt < 1.6f;
        toastText.gameObject.SetActive(toastOn);
        if (toastOn)
        {
            float a = tt < 1.1f ? 1f : 1f - (tt - 1.1f) / 0.5f;
            float s = tt < 0.15f ? Mathf.Lerp(0.7f, 1.05f, tt / 0.15f) : Mathf.Lerp(1.05f, 1f, Mathf.Clamp01((tt - 0.15f) / 0.15f));
            toastText.color = new Color(1f, 1f, 1f, a);
            toastText.rectTransform.localScale = new Vector3(s, s, 1f);
            toastText.rectTransform.anchoredPosition = new Vector2(0f, 40f * tt);
        }

        if (!countdownText.gameObject.activeSelf) return;
        float ct = Time.time - countdownTime;
        if (ct > 0.9f) { countdownText.gameObject.SetActive(false); return; }
        float cs = Mathf.Lerp(1.4f, 1f, Mathf.Clamp01(ct / 0.25f));
        countdownText.rectTransform.localScale = new Vector3(cs, cs, 1f);
        countdownText.color = new Color(accentColor.r, accentColor.g, accentColor.b, ct < 0.6f ? 1f : 1f - (ct - 0.6f) / 0.3f);
    }

    private void UpdateResultStars()
    {
        if (!resultOverlay.activeSelf) return;
        float t = Time.time - resultShownTime;
        for (int i = 0; i < resultStars.Length; i++)
        {
            bool earned = i < resultStarCount;
            resultStars[i].color = earned ? accentColor : new Color(1f, 1f, 1f, 0.18f);
            float local = (t - 0.35f - i * 0.22f) / 0.3f; // staggered pop-in
            float s = local <= 0f ? 0f : local >= 1f ? 1f : (local < 0.7f ? Mathf.Lerp(0f, 1.25f, local / 0.7f) : Mathf.Lerp(1.25f, 1f, (local - 0.7f) / 0.3f));
            resultStars[i].rectTransform.localScale = new Vector3(s, s, 1f);
        }
    }

    private void UpdateBursts()
    {
        for (int i = bursts.Count - 1; i >= 0; i--)
        {
            var b = bursts[i];
            b.age += Time.deltaTime;
            float t = b.age / 0.6f;
            if (t >= 1f) { Destroy(b.rt.gameObject); bursts.RemoveAt(i); continue; }
            float s = Mathf.Lerp(0.6f, 1.7f, 1f - (1f - t) * (1f - t));
            b.rt.localScale = new Vector3(s, s, 1f);
            var c = b.img.color; c.a = 1f - t; b.img.color = c;
        }
    }

    private void UpdateConfetti()
    {
        if (confetti.Count == 0) return;
        float halfH = canvasRect.rect.height * 0.5f;
        for (int i = confetti.Count - 1; i >= 0; i--)
        {
            var c = confetti[i];
            c.age += Time.deltaTime;
            if (c.age >= c.life || c.rt.anchoredPosition.y < -halfH - 60f) { Destroy(c.rt.gameObject); confetti.RemoveAt(i); continue; }
            c.vel.y -= 900f * Time.deltaTime;
            c.vel *= 1f - 0.6f * Time.deltaTime; // air drag keeps the fall floaty
            Vector2 sway = new Vector2(Mathf.Sin(c.age * 5f + c.sway) * 60f, 0f);
            c.rt.anchoredPosition += (c.vel + sway) * Time.deltaTime;
            c.rt.localRotation = Quaternion.Euler(0f, 0f, c.rt.localEulerAngles.z + c.rotVel * Time.deltaTime);
            float fade = Mathf.Clamp01((c.life - c.age) / 0.6f);
            var col = c.img.color; col.a = fade; c.img.color = col;
        }
    }

    // ---------- Effects ----------

    private void ShowToast(string text)
    {
        toastText.text = text;
        toastTime = Time.time;
    }

    private void SpawnBurst(TangramGameManager.PieceInfo p)
    {
        if (!TryGetPieceScreenRect(p.transform, out Vector2 center, out float size)) return;
        var img = NewImage("Burst", effectsLayer, ringSprite, p.color);
        img.rectTransform.anchoredPosition = center;
        img.rectTransform.sizeDelta = new Vector2(size, size);
        bursts.Add(new Burst { rt = img.rectTransform, img = img });
    }

    private void SpawnConfetti()
    {
        var palette = new List<Color> { accentColor, Color.white };
        foreach (var p in game.Pieces) palette.Add(p.color);
        Vector2 half = canvasRect.rect.size * 0.5f;
        for (int i = 0; i < 90; i++)
        {
            var img = NewImage("Confetti", confettiLayer, roundSprite, palette[UnityEngine.Random.Range(0, palette.Count)]);
            img.type = Image.Type.Simple;
            var rt = img.rectTransform;
            rt.sizeDelta = new Vector2(UnityEngine.Random.Range(14f, 26f), UnityEngine.Random.Range(8f, 14f));
            rt.anchoredPosition = new Vector2(UnityEngine.Random.Range(-half.x, half.x), half.y + UnityEngine.Random.Range(10f, 300f));
            rt.localRotation = Quaternion.Euler(0f, 0f, UnityEngine.Random.Range(0f, 360f));
            confetti.Add(new Confetto
            {
                rt = rt,
                img = img,
                vel = new Vector2(UnityEngine.Random.Range(-160f, 160f), UnityEngine.Random.Range(-450f, -100f)),
                rotVel = UnityEngine.Random.Range(-360f, 360f),
                life = UnityEngine.Random.Range(2.8f, 4f),
                sway = UnityEngine.Random.Range(0f, Mathf.PI * 2f)
            });
        }
    }

    /// <summary>Piece center and on-screen size, in canvas units (canvas center = origin).</summary>
    private bool TryGetPieceScreenRect(Transform piece, out Vector2 center, out float size)
    {
        center = Vector2.zero; size = 0f;
        var cam = game.GetDiagramCamera();
        if (cam == null || piece == null) return false;

        Vector3 sp = cam.WorldToScreenPoint(game.matcher.GetPieceCenter(piece));
        if (sp.z <= 0f) return false;

        var r = piece.GetComponent<Renderer>();
        float px = 80f;
        if (r != null)
        {
            var b = r.bounds;
            Vector2 min = new Vector2(float.MaxValue, float.MaxValue), max = new Vector2(float.MinValue, float.MinValue);
            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = b.center + Vector3.Scale(b.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                Vector3 c = cam.WorldToScreenPoint(corner);
                min = Vector2.Min(min, c); max = Vector2.Max(max, c);
            }
            px = Mathf.Max(max.x - min.x, max.y - min.y);
        }

        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, sp, null, out center)) return false;
        size = Mathf.Clamp(px / Mathf.Max(0.01f, canvas.scaleFactor), 60f, 600f);
        return true;
    }

    // ---------- Building ----------

    private void BuildCanvas()
    {
        var go = new GameObject("TangramHUD", typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
        go.transform.SetParent(transform, false);
        canvas = go.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = sortingOrder;
        var scaler = go.GetComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1080f, 1920f);
        scaler.matchWidthOrHeight = 0.5f;
        canvasRect = go.GetComponent<RectTransform>();

        effectsLayer = Stretch(NewRect("Effects", canvasRect));
        safeArea = Stretch(NewRect("SafeArea", canvasRect));
        confettiLayer = Stretch(NewRect("Confetti", canvasRect));

        hintGlow = NewImage("HintGlow", effectsLayer, circleSprite, accentColor);
        hintRing = NewImage("HintRing", effectsLayer, ringSprite, accentColor);
        hintGlow.gameObject.SetActive(false);
        hintRing.gameObject.SetActive(false);
    }

    private void BuildTopBar()
    {
        var bar = NewImage("TopBar", safeArea, roundSprite, panelColor, true);
        topBar = bar.gameObject;
        Place(bar.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -20f), new Vector2(-40f, TopBarHeight));

        titleText = NewText("Title", bar.transform, "", 34, TextAnchor.LowerLeft, FontStyle.Bold);
        Place(titleText.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0f, 0f), new Vector2(28f, 0f), new Vector2(330f, 50f));
        progressText = NewText("Progress", bar.transform, "", 28, TextAnchor.UpperLeft);
        progressText.color = new Color(1f, 1f, 1f, 0.8f);
        Place(progressText.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f), new Vector2(0f, 1f), new Vector2(28f, -2f), new Vector2(330f, 44f));

        timerText = NewText("Timer", bar.transform, "00:00", 58, TextAnchor.MiddleCenter, FontStyle.Bold);
        Place(timerText.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(-20f, 0f), new Vector2(240f, 90f));

        var restart = NewButton("Restart", bar.transform, "다시", new Color(1f, 1f, 1f, 0.16f), () => { Fx?.PlayTap(); game.StartRound(); }, out _);
        Place((RectTransform)restart.transform, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(-18f, 0f), new Vector2(124f, 84f));
        hintButton = NewButton("Hint", bar.transform, "힌트", buttonColor, () => { Fx?.PlayTap(); game.RequestHint(); }, out hintButtonText);
        Place((RectTransform)hintButton.transform, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(-154f, 0f), new Vector2(150f, 84f));

        var track = NewImage("ProgressTrack", bar.transform, roundSprite, new Color(0f, 0f, 0f, 0.45f), true);
        Place(track.rectTransform, new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(0.5f, 1f), new Vector2(0f, -10f), new Vector2(0f, 14f));
        var fill = NewImage("ProgressFill", track.transform, roundSprite, accentColor, true);
        progressFill = fill.rectTransform;
        progressFill.anchorMin = Vector2.zero; progressFill.anchorMax = new Vector2(0.0001f, 1f);
        progressFill.offsetMin = Vector2.zero; progressFill.offsetMax = Vector2.zero;

        chipRow = NewRect("Chips", safeArea);
        Place(chipRow, new Vector2(0f, 1f), new Vector2(1f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -20f - TopBarHeight - 36f), new Vector2(-40f, 120f));
        var layout = chipRow.gameObject.AddComponent<HorizontalLayoutGroup>();
        layout.childAlignment = TextAnchor.MiddleCenter;
        layout.spacing = 12f;
        layout.childControlWidth = false; layout.childControlHeight = false;
        layout.childForceExpandWidth = false; layout.childForceExpandHeight = false;
    }

    private void RebuildChips()
    {
        foreach (var c in chips) Destroy(c.root.gameObject);
        chips.Clear();
        foreach (var p in game.Pieces)
        {
            var bg = NewImage("Chip_" + p.transform.name, chipRow, roundSprite, panelColor, true);
            bg.rectTransform.sizeDelta = new Vector2(112f, 112f);
            shapeSprites.TryGetValue(p.type, out var sprite);
            var icon = NewImage("Icon", bg.transform, sprite, emptyChipColor);
            float k = ShapeIconScale(p.type);
            Place(icon.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(92f * k, 92f * k));
            var badge = NewImage("Badge", bg.transform, circleSprite, new Color(0.2f, 0.78f, 0.4f, 1f));
            Place(badge.rectTransform, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(0.5f, 0.5f), new Vector2(-14f, -14f), new Vector2(40f, 40f));
            var check = NewImage("Check", badge.transform, checkSprite, Color.white);
            Place(check.rectTransform, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);
            badge.gameObject.SetActive(false);
            chips.Add(new Chip { piece = p, root = bg.rectTransform, background = bg, icon = icon, badge = badge });
        }
    }

    private void BuildBannerAndToast()
    {
        bannerBg = NewImage("Banner", safeArea, roundSprite, panelColor, true);
        banner = bannerBg.gameObject;
        Place(bannerBg.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -20f - TopBarHeight - 172f), new Vector2(960f, 84f));
        bannerText = NewText("Text", banner.transform, "", 30, TextAnchor.MiddleCenter, FontStyle.Bold);
        Place(bannerText.rectTransform, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(-40f, 0f));
        banner.SetActive(false);

        toastText = NewText("Toast", safeArea, "", 52, TextAnchor.MiddleCenter, FontStyle.Bold);
        toastText.gameObject.AddComponent<Outline>().effectColor = new Color(0f, 0f, 0f, 0.6f);
        Place(toastText.rectTransform, new Vector2(0.5f, 0.6f), new Vector2(0.5f, 0.6f), new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(1000f, 120f));
        toastText.gameObject.SetActive(false);

        countdownText = NewText("Countdown", safeArea, "", 240, TextAnchor.MiddleCenter, FontStyle.Bold);
        var o = countdownText.gameObject.AddComponent<Outline>();
        o.effectColor = new Color(0f, 0f, 0f, 0.7f); o.effectDistance = new Vector2(4f, -4f);
        Place(countdownText.rectTransform, new Vector2(0.5f, 0.55f), new Vector2(0.5f, 0.55f), new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(800f, 360f));
        countdownText.gameObject.SetActive(false);
    }

    private void BuildIntro()
    {
        introOverlay = NewOverlay("Intro", out introCard, new Vector2(920f, 1120f));
        var card = introCard;

        var title = NewText("Title", card, "탱그램 퍼즐", 84, TextAnchor.MiddleCenter, FontStyle.Bold);
        Place(title.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -50f), new Vector2(860f, 110f));

        introPuzzleText = NewText("Puzzle", card, "", 44, TextAnchor.MiddleCenter, FontStyle.Bold);
        introPuzzleText.color = accentColor;
        Place(introPuzzleText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -185f), new Vector2(520f, 70f));
        prevPuzzleButton = NewButton("Prev", card, "이전", new Color(1f, 1f, 1f, 0.16f), () => { Fx?.PlayTap(); game.PreviousPuzzle(); }, out _);
        Place((RectTransform)prevPuzzleButton.transform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(-340f, -180f), new Vector2(140f, 80f));
        nextPuzzleButtonIntro = NewButton("Next", card, "다음", new Color(1f, 1f, 1f, 0.16f), () => { Fx?.PlayTap(); game.NextPuzzle(); }, out _);
        Place((RectTransform)nextPuzzleButtonIntro.transform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(340f, -180f), new Vector2(140f, 80f));

        introStars = NewStarRow(card, new Vector2(0f, -300f), 56f, 14f);
        introBestText = NewText("Best", card, "", 32, TextAnchor.MiddleCenter);
        introBestText.color = new Color(1f, 1f, 1f, 0.8f);
        Place(introBestText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -350f), new Vector2(800f, 50f));

        var howTo = NewText("HowTo", card,
            "1.  보드 네 모서리의 마커가 모두 화면에 보이게 해 주세요\n\n" +
            "2.  조각을 보드 위에 흩어 놓고 시작하세요\n\n" +
            "3.  화면 아래 그림과 같은 모양을 만들면\n     맞춘 조각이 하나씩 색으로 채워져요\n\n" +
            "4.  조각은 이웃 조각과 함께 놓여야 인식돼요.\n     막히면 힌트를 눌러 보세요",
            30, TextAnchor.UpperLeft);
        howTo.color = new Color(1f, 1f, 1f, 0.9f);
        howTo.lineSpacing = 1.1f;
        Place(howTo.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -440f), new Vector2(780f, 460f));

        var start = NewButton("Start", card, "시작하기", accentColor, () => { Fx?.PlayTap(); game.StartRound(); }, out var startText);
        startText.fontSize = 50;
        startText.color = new Color(0.12f, 0.1f, 0.05f, 1f);
        Place((RectTransform)start.transform, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0f, 60f), new Vector2(620f, 140f));
    }

    private void RefreshIntro()
    {
        if (introPuzzleText == null) return;
        bool multi = game.PuzzleCount > 1;
        introPuzzleText.text = multi ? $"{game.CurrentPuzzleName}  ({game.CurrentPuzzleIndex + 1}/{game.PuzzleCount})" : game.CurrentPuzzleName;
        prevPuzzleButton.gameObject.SetActive(multi);
        nextPuzzleButtonIntro.gameObject.SetActive(multi);

        bool hasBest = game.TryGetBest(out float best, out int stars);
        introBestText.text = hasBest ? "최고 기록  " + FormatTime(best, true) : "아직 기록이 없어요. 첫 도전!";
        for (int i = 0; i < introStars.Length; i++)
        {
            introStars[i].gameObject.SetActive(hasBest);
            introStars[i].color = i < stars ? accentColor : new Color(1f, 1f, 1f, 0.18f);
        }
    }

    private void BuildResult()
    {
        resultOverlay = NewOverlay("Result", out resultCard, new Vector2(920f, 980f));
        var card = resultCard;

        var title = NewText("Title", card, "완성!", 120, TextAnchor.MiddleCenter, FontStyle.Bold);
        title.color = accentColor;
        title.gameObject.AddComponent<Outline>().effectColor = new Color(0f, 0f, 0f, 0.5f);
        Place(title.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -40f), new Vector2(860f, 160f));

        resultPuzzleText = NewText("Puzzle", card, "", 36, TextAnchor.MiddleCenter);
        resultPuzzleText.color = new Color(1f, 1f, 1f, 0.75f);
        Place(resultPuzzleText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -200f), new Vector2(800f, 56f));

        resultStars = NewStarRow(card, new Vector2(0f, -290f), 150f, 24f);

        resultTimeText = NewText("Time", card, "", 52, TextAnchor.MiddleCenter, FontStyle.Bold);
        Place(resultTimeText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -470f), new Vector2(800f, 76f));
        resultHintsText = NewText("Hints", card, "", 34, TextAnchor.MiddleCenter);
        resultHintsText.color = new Color(1f, 1f, 1f, 0.85f);
        Place(resultHintsText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -556f), new Vector2(800f, 56f));
        resultBestText = NewText("Best", card, "", 40, TextAnchor.MiddleCenter, FontStyle.Bold);
        Place(resultBestText.rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0f, -626f), new Vector2(800f, 60f));

        resultRetryButton = NewButton("Retry", card, "다시 하기", new Color(1f, 1f, 1f, 0.16f), () => { Fx?.PlayTap(); game.StartRound(); }, out _);
        resultNextButton = NewButton("Next", card, "다음 퍼즐", accentColor, () => { Fx?.PlayTap(); game.NextPuzzle(); }, out var nextText);
        nextText.color = new Color(0.12f, 0.1f, 0.05f, 1f);
        LayoutResultButtons(true);
    }

    private void LayoutResultButtons(bool twoButtons)
    {
        var retry = (RectTransform)resultRetryButton.transform;
        var next = (RectTransform)resultNextButton.transform;
        if (twoButtons)
        {
            Place(retry, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(-205f, 60f), new Vector2(380f, 130f));
            Place(next, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(205f, 60f), new Vector2(380f, 130f));
        }
        else
        {
            Place(retry, new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0.5f, 0f), new Vector2(0f, 60f), new Vector2(560f, 130f));
        }
    }

    // ---------- Layout ----------

    private void ApplySafeAreaIfChanged()
    {
        var sa = Screen.safeArea;
        var screen = new Vector2(Screen.width, Screen.height);
        Vector2 canvasSize = canvasRect.rect.size;
        if (sa == lastSafeArea && canvasSize == lastCanvasSize) return;
        lastSafeArea = sa; lastCanvasSize = canvasSize;
        if (screen.x <= 0f || screen.y <= 0f) return;

        safeArea.anchorMin = new Vector2(sa.xMin / screen.x, sa.yMin / screen.y);
        safeArea.anchorMax = new Vector2(sa.xMax / screen.x, sa.yMax / screen.y);
        safeArea.offsetMin = safeArea.offsetMax = Vector2.zero;

        // Cards are authored for portrait; shrink them to fit short (landscape) screens.
        Vector2 avail = safeArea.rect.size;
        foreach (var card in new[] { introCard, resultCard })
        {
            Vector2 size = card.sizeDelta;
            float k = Mathf.Min(1f, avail.x * 0.95f / size.x, avail.y * 0.92f / size.y);
            card.localScale = new Vector3(k, k, 1f);
        }
    }

    // ---------- UI factory helpers ----------

    private GameObject NewOverlay(string name, out RectTransform card, Vector2 cardSize)
    {
        var dim = NewImage(name, canvasRect, null, new Color(0f, 0f, 0f, 0.55f));
        dim.raycastTarget = true; // block touches to the HUD underneath
        Stretch(dim.rectTransform);
        dim.transform.SetSiblingIndex(confettiLayer.GetSiblingIndex()); // above HUD, below confetti
        var c = NewImage("Card", dim.transform, roundSprite, new Color(0.08f, 0.09f, 0.13f, 0.94f), true);
        Place(c.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0f, 40f), cardSize);
        card = c.rectTransform;
        dim.gameObject.SetActive(false);
        return dim.gameObject;
    }

    private Image[] NewStarRow(RectTransform parent, Vector2 topCenter, float size, float spacing)
    {
        var stars = new Image[3];
        for (int i = 0; i < 3; i++)
        {
            stars[i] = NewImage("Star" + i, parent, starSprite, accentColor);
            float x = (i - 1) * (size + spacing);
            Place(stars[i].rectTransform, new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(x, topCenter.y), new Vector2(size, size));
        }
        return stars;
    }

    private static RectTransform NewRect(string name, Transform parent)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.layer = 5; // UI
        go.transform.SetParent(parent, false);
        return (RectTransform)go.transform;
    }

    private static RectTransform Stretch(RectTransform rt)
    {
        rt.anchorMin = Vector2.zero; rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero; rt.offsetMax = Vector2.zero;
        return rt;
    }

    private static void Place(RectTransform rt, Vector2 anchorMin, Vector2 anchorMax, Vector2 pivot, Vector2 pos, Vector2 size)
    {
        rt.anchorMin = anchorMin; rt.anchorMax = anchorMax; rt.pivot = pivot;
        rt.anchoredPosition = pos; rt.sizeDelta = size;
    }

    private Image NewImage(string name, Transform parent, Sprite sprite, Color color, bool sliced = false)
    {
        var rt = NewRect(name, parent);
        var img = rt.gameObject.AddComponent<Image>();
        img.sprite = sprite;
        img.color = color;
        img.raycastTarget = false;
        if (sliced) img.type = Image.Type.Sliced;
        return img;
    }

    private Text NewText(string name, Transform parent, string text, int size, TextAnchor anchor, FontStyle style = FontStyle.Normal)
    {
        var rt = NewRect(name, parent);
        var t = rt.gameObject.AddComponent<Text>();
        t.font = font;
        t.text = text;
        t.fontSize = size;
        t.fontStyle = style;
        t.alignment = anchor;
        t.color = Color.white;
        t.horizontalOverflow = HorizontalWrapMode.Wrap;
        t.verticalOverflow = VerticalWrapMode.Overflow;
        t.raycastTarget = false;
        var sh = rt.gameObject.AddComponent<Shadow>();
        sh.effectColor = new Color(0f, 0f, 0f, 0.45f);
        sh.effectDistance = new Vector2(2f, -2f);
        return t;
    }

    private Button NewButton(string name, Transform parent, string label, Color color, UnityAction onClick, out Text labelText)
    {
        var img = NewImage(name, parent, roundSprite, color, true);
        img.raycastTarget = true;
        var b = img.gameObject.AddComponent<Button>();
        b.targetGraphic = img;
        var colors = b.colors;
        colors.disabledColor = new Color(0.6f, 0.6f, 0.6f, 0.45f);
        b.colors = colors;
        b.onClick.AddListener(onClick);
        labelText = NewText("Label", img.transform, label, 34, TextAnchor.MiddleCenter, FontStyle.Bold);
        Stretch(labelText.rectTransform);
        return b;
    }

    private static string FormatTime(float seconds, bool withTenths)
    {
        if (seconds < 0f) seconds = 0f;
        int m = (int)(seconds / 60f);
        float s = seconds - m * 60f;
        return withTenths ? $"{m:00}:{s:00.0}" : $"{m:00}:{(int)s:00}";
    }

    private static float ShapeIconScale(TangramMatcher.TangramShapeType type)
    {
        // Rough area cue: a large triangle is twice a medium one, which is twice a small one.
        switch (type)
        {
            case TangramMatcher.TangramShapeType.LargeTriangle: return 1f;
            case TangramMatcher.TangramShapeType.MediumTriangle: return 0.78f;
            case TangramMatcher.TangramShapeType.SmallTriangle: return 0.58f;
            case TangramMatcher.TangramShapeType.Square: return 0.72f;
            default: return 0.86f;
        }
    }

    // ---------- Procedural sprites ----------

    private void LoadFont()
    {
        try { font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf"); } catch { font = null; }
        if (font == null)
        {
            try { font = Resources.GetBuiltinResource<Font>("Arial.ttf"); } catch { font = null; }
        }
    }

    private void BuildSprites()
    {
        // Signed-distance shapes give anti-aliased edges without any texture assets.
        roundSprite = MakeSprite("Round", 64, (x, y) => Sdf(RoundRectDistance(x, y, 32f, 32f, 32f, 32f, 22f)), new Vector4(26, 26, 26, 26));
        circleSprite = MakeSprite("Circle", 128, (x, y) => Sdf(Vector2.Distance(new Vector2(x, y), new Vector2(64f, 64f)) - 62f));
        ringSprite = MakeSprite("Ring", 256, (x, y) => Sdf(Mathf.Abs(Vector2.Distance(new Vector2(x, y), new Vector2(128f, 128f)) - 116f) - 9f));
        checkSprite = MakeSprite("Check", 64, (x, y) =>
        {
            var p = new Vector2(x, y);
            float d = Mathf.Min(SegmentDistance(p, new Vector2(16f, 33f), new Vector2(27f, 21f)), SegmentDistance(p, new Vector2(27f, 21f), new Vector2(48f, 44f)));
            return Sdf(d - 4.5f);
        });

        var star = new Vector2[10];
        for (int i = 0; i < 10; i++)
        {
            float a = Mathf.PI / 2f + i * Mathf.PI / 5f;
            float r = (i % 2 == 0) ? 0.48f : 0.2f;
            star[i] = new Vector2(0.5f + r * Mathf.Cos(a), 0.5f + r * Mathf.Sin(a));
        }
        starSprite = PolygonSprite("Star", 128, star);

        var tri = PolygonSprite("Triangle", 128, new[] { new Vector2(0.04f, 0.27f), new Vector2(0.96f, 0.27f), new Vector2(0.5f, 0.73f) });
        shapeSprites[TangramMatcher.TangramShapeType.LargeTriangle] = tri;
        shapeSprites[TangramMatcher.TangramShapeType.MediumTriangle] = tri;
        shapeSprites[TangramMatcher.TangramShapeType.SmallTriangle] = tri;
        shapeSprites[TangramMatcher.TangramShapeType.Square] = PolygonSprite("Square", 128, new[] { new Vector2(0.5f, 0.06f), new Vector2(0.94f, 0.5f), new Vector2(0.5f, 0.94f), new Vector2(0.06f, 0.5f) });
        shapeSprites[TangramMatcher.TangramShapeType.Parallelogram] = PolygonSprite("Parallelogram", 128, new[] { new Vector2(0.03f, 0.3f), new Vector2(0.64f, 0.3f), new Vector2(0.97f, 0.7f), new Vector2(0.36f, 0.7f) });
    }

    private static float Sdf(float distance) => Mathf.Clamp01(0.5f - distance);

    private static float RoundRectDistance(float x, float y, float cx, float cy, float hw, float hh, float r)
    {
        float qx = Mathf.Abs(x - cx) - (hw - r);
        float qy = Mathf.Abs(y - cy) - (hh - r);
        float outside = new Vector2(Mathf.Max(qx, 0f), Mathf.Max(qy, 0f)).magnitude;
        return outside + Mathf.Min(Mathf.Max(qx, qy), 0f) - r;
    }

    private static float SegmentDistance(Vector2 p, Vector2 a, Vector2 b)
    {
        Vector2 ab = b - a;
        float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
        return Vector2.Distance(p, a + ab * t);
    }

    /// <summary>Polygon in normalized [0,1]² coordinates, 4×4 supersampled for anti-aliasing.</summary>
    private static Sprite PolygonSprite(string name, int size, Vector2[] poly)
    {
        return MakeSprite(name, size, (x, y) =>
        {
            int hits = 0;
            for (int sy = 0; sy < 4; sy++)
                for (int sx = 0; sx < 4; sx++)
                {
                    var p = new Vector2((x - 0.5f + (sx + 0.5f) / 4f) / size, (y - 0.5f + (sy + 0.5f) / 4f) / size);
                    if (PointInPolygon(p, poly)) hits++;
                }
            return hits / 16f;
        });
    }

    private static bool PointInPolygon(Vector2 p, Vector2[] poly)
    {
        bool inside = false;
        for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
        {
            if ((poly[i].y > p.y) != (poly[j].y > p.y) &&
                p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)
                inside = !inside;
        }
        return inside;
    }

    /// <summary>White texture whose alpha comes from <paramref name="alpha"/>(pixelCenterX, pixelCenterY).</summary>
    private static Sprite MakeSprite(string name, int size, Func<float, float, float> alpha, Vector4 border = default)
    {
        var tex = new Texture2D(size, size, TextureFormat.RGBA32, false) { name = name, wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear };
        var px = new Color32[size * size];
        for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
                px[y * size + x] = new Color32(255, 255, 255, (byte)Mathf.RoundToInt(255f * alpha(x + 0.5f, y + 0.5f)));
        tex.SetPixels32(px);
        tex.Apply(false, true);
        return Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 100f, 0, SpriteMeshType.FullRect, border);
    }
}
