using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ObjdetectModule;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Rect = UnityEngine.Rect;

/// <summary>
/// Exports a printable, true-scale (1:1) PDF for putting the ArUco stickers on the physical pieces and making the board.
///
/// Geometry comes from <see cref="SimTangramWorld.ReadDiagramShapes"/>, the same diagram → sticker-frame reading the
/// simulator uses, so a piece made with this guide is what TangramMatcher.ComputeAngleError expects: sticker centered
/// on the answer piece's 'center' child, marker +X along 'center'→'direction' (+ SimSetup.stickerRotationOffsetDeg).
/// Every piece is drawn with its +X pointing right, so a correctly placed marker reads upright with the page text.
///
/// Pages: piece labels (outline, sticker square, +X arrow, ID), loose marker stickers, and the board with corner
/// markers 10..13 (split over several sheets when it is larger than the paper). The PDF is vector, in millimeters:
/// markers are filled cell by cell from Objdetect.generateImageMarker, and every page carries a 50 mm bar to check the
/// print scale. Files go to &lt;project&gt;/StickerGuides (outside Assets, so Unity does not import them).
/// </summary>
public class StickerGuideExporter : EditorWindow
{
    public enum Paper { A4, Letter, A3 }

    [SerializeField] private SimSetup setup = new SimSetup();
    [SerializeField] private string setSizesCm = "15";
    [SerializeField] private float boardToSetRatio = 1.6f;
    [SerializeField] private Paper paper = Paper.A4;
    [SerializeField] private string outputFolder = DefaultFolder;

    private const string DefaultFolder = "StickerGuides";

    // ---------- Menu / window ----------

    [MenuItem("Tools/Tangram/Sticker Guide...")]
    private static void Open()
    {
        var w = GetWindow<StickerGuideExporter>("Sticker Guide");
        w.LoadSceneSetup();
    }

    [MenuItem("Tools/Tangram/Export Sticker Guide (Scene Setup)")]
    private static void ExportSceneSetup()
    {
        var matcher = FindMatcher();
        if (matcher == null) return;
        var warnings = new List<string>();
        string path = Export(matcher, SceneSetup(), Paper.A4, OutputDir(DefaultFolder), warnings);
        Report(new List<string> { path }, warnings);
    }

    private void LoadSceneSetup()
    {
        setup = SceneSetup();
        setSizesCm = (setup.setSizeMeters * 100f).ToString("0.#", CultureInfo.InvariantCulture);
        boardToSetRatio = setup.boardSizeMeters / setup.setSizeMeters;
    }

    private void OnGUI()
    {
        EditorGUILayout.HelpBox("Printable 1:1 PDF: piece labels/stencils, marker stickers and the board with corner markers 10–13. " +
            "Piece shapes and sticker frames come from the open scene's TangramMatcher diagram, as in the simulator.", MessageType.None);
        if (GUILayout.Button("Load sizes from the scene's simulator")) LoadSceneSetup();
        EditorGUILayout.Space();
        setSizesCm = EditorGUILayout.TextField(new GUIContent("Set sizes (cm)", "Side of the square the seven pieces form. Comma-separated: one PDF per size."), setSizesCm);
        setup.markerSizeMeters = EditorGUILayout.FloatField(new GUIContent("Marker (mm)", "Black marker square. Corner and piece markers share it, and it should equal ArUcoMobileTracker.markerLength."), setup.markerSizeMeters * 1000f) / 1000f;
        setup.stickerQuietZoneRatio = EditorGUILayout.Slider(new GUIContent("White border", "Quiet zone per side, as a fraction of the marker size."), setup.stickerQuietZoneRatio, 0f, 0.5f);
        boardToSetRatio = EditorGUILayout.FloatField(new GUIContent("Board / set size", "Board side as a multiple of the set size (simulator: 24 cm / 15 cm = 1.6)."), boardToSetRatio);
        setup.stickerRotationOffsetDeg = EditorGUILayout.FloatField(new GUIContent("Sticker rotation (°)", "Marker +X relative to the piece's center→direction axis. Keep 0 unless the matcher's angleOffset convention changes."), setup.stickerRotationOffsetDeg);
        paper = (Paper)EditorGUILayout.EnumPopup("Paper", paper);
        outputFolder = EditorGUILayout.TextField(new GUIContent("Output folder", "Relative to the project folder."), outputFolder);
        EditorGUILayout.Space();
        if (GUILayout.Button("Export PDF", GUILayout.Height(28))) ExportFromWindow();
    }

    private void ExportFromWindow()
    {
        var matcher = FindMatcher();
        if (matcher == null) return;
        var warnings = new List<string>();
        var paths = new List<string>();
        foreach (float cm in ParseList(setSizesCm))
            paths.Add(Export(matcher, ForSetSize(setup, cm / 100f, boardToSetRatio), paper, OutputDir(outputFolder), warnings));
        Report(paths, warnings);
    }

    /// <summary>
    /// Batch entry point, e.g.
    /// Unity -batchmode -projectPath . -executeMethod StickerGuideExporter.ExportFromCommandLine -guideSetsCm 15,30
    /// Options: -guideScene, -guideSetsCm, -guideMarkerMm, -guideQuietZone, -guideBoardRatio, -guidePaper A4|Letter|A3, -guideOut.
    /// Unset options use the scene simulator's SimSetup.
    /// </summary>
    public static void ExportFromCommandLine()
    {
        int code = 0;
        try
        {
            var args = Environment.GetCommandLineArgs();
            string Arg(string name, string fallback) { int i = Array.IndexOf(args, name); return i >= 0 && i + 1 < args.Length ? args[i + 1] : fallback; }

            EditorSceneManager.OpenScene(Arg("-guideScene", "Assets/Scenes/SampleScene.unity"));
            var matcher = FindMatcher();
            if (matcher == null) throw new InvalidOperationException("No TangramMatcher in the scene.");
            var baseSetup = SceneSetup();
            baseSetup.markerSizeMeters = ParseFloat(Arg("-guideMarkerMm", null), baseSetup.markerSizeMeters * 1000f) / 1000f;
            baseSetup.stickerQuietZoneRatio = ParseFloat(Arg("-guideQuietZone", null), baseSetup.stickerQuietZoneRatio);
            float ratio = ParseFloat(Arg("-guideBoardRatio", null), baseSetup.boardSizeMeters / baseSetup.setSizeMeters);
            var paper = (Paper)Enum.Parse(typeof(Paper), Arg("-guidePaper", "A4"), true);
            string outDir = OutputDir(Arg("-guideOut", DefaultFolder));

            var warnings = new List<string>();
            var paths = new List<string>();
            foreach (float cm in ParseList(Arg("-guideSetsCm", (baseSetup.setSizeMeters * 100f).ToString(CultureInfo.InvariantCulture))))
                paths.Add(Export(matcher, ForSetSize(baseSetup, cm / 100f, ratio), paper, outDir, warnings));
            Report(paths, warnings);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            code = 1;
        }
        EditorApplication.Exit(code);
    }

    private static TangramMatcher FindMatcher()
    {
        var matcher = FindFirstObjectByType<TangramMatcher>(FindObjectsInactive.Include);
        if (matcher == null && !Application.isBatchMode)
            EditorUtility.DisplayDialog("Sticker Guide", "Open the game scene first: no TangramMatcher found.", "OK");
        return matcher;
    }

    private static SimSetup SceneSetup()
    {
        var sim = FindFirstObjectByType<SimulatedTangramCamera>(FindObjectsInactive.Include);
        return JsonUtility.FromJson<SimSetup>(JsonUtility.ToJson(sim != null ? sim.setup : new SimSetup()));
    }

    private static SimSetup ForSetSize(SimSetup baseSetup, float setMeters, float boardRatio)
    {
        var s = JsonUtility.FromJson<SimSetup>(JsonUtility.ToJson(baseSetup));
        s.setSizeMeters = setMeters;
        s.boardSizeMeters = setMeters * boardRatio;
        return s;
    }

    private static string OutputDir(string folder) =>
        Path.IsPathRooted(folder) ? folder : Path.Combine(Directory.GetParent(Application.dataPath).FullName, folder);

    private static List<float> ParseList(string csv)
    {
        var list = new List<float>();
        foreach (var part in csv.Split(new[] { ',', ';', ' ' }, StringSplitOptions.RemoveEmptyEntries))
            if (float.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out float v) && v > 0f) list.Add(v);
        return list;
    }

    private static float ParseFloat(string s, float fallback) =>
        s != null && float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : fallback;

    private static void Report(List<string> paths, List<string> warnings)
    {
        foreach (var p in paths) Debug.Log("[StickerGuide] Wrote " + p);
        foreach (var w in warnings) Debug.LogWarning("[StickerGuide] " + w);
        if (Application.isBatchMode || paths.Count == 0) return;
        string msg = $"Wrote {paths.Count} PDF(s) to\n{Path.GetDirectoryName(paths[0])}\n\nPrint at 100 % (Actual size) and check the 50 mm bar.";
        if (warnings.Count > 0) msg += $"\n\n{warnings.Count} warning(s), see the Console:\n• " + string.Join("\n• ", warnings);
        EditorUtility.DisplayDialog("Sticker Guide", msg, "OK");
        EditorUtility.RevealInFinder(paths[0]);
    }

    // ---------- Export ----------

    private const float Margin = 12f, Gap = 6f, Overlap = 12f, FooterH = 11f, LineH = 3.6f; // mm
    private static readonly Color Ink = Color.black, Faint = new Color(0.6f, 0.6f, 0.6f), Dim = new Color(0.3f, 0.3f, 0.3f), Warn = new Color(0.8f, 0.1f, 0.1f);

    /// <summary>Something to place on paper, drawn in its own millimeter coordinates.</summary>
    private sealed class Item
    {
        public string name;
        public Rect bounds;
        public Action<Pdf> draw;
        public List<(string what, Rect rect)> keepWhole = new List<(string, Rect)>(); // markers a sheet seam must not cut
    }

    private sealed class Page
    {
        public string title;
        public List<string> intro = new List<string>();
        public Action<Pdf> body;
    }

    private sealed class Guide
    {
        public SimSetup setup;
        public float setMm, markerMm, stickerHalf, boardMm;
        public string summary;
        public Dictionary dictionary;
        private readonly Dictionary<int, bool[,]> cells = new Dictionary<int, bool[,]>();

        /// <summary>6×6 cells (4×4 bits + 1-cell black border), row 0 at the top, true = black.</summary>
        public bool[,] Cells(int id)
        {
            if (cells.TryGetValue(id, out var c)) return c;
            const int n = 6, px = 10;
            c = new bool[n, n];
            using (var img = new Mat())
            {
                Objdetect.generateImageMarker(dictionary, id, n * px, img, 1);
                for (int r = 0; r < n; r++)
                    for (int k = 0; k < n; k++) c[r, k] = img.get(r * px + px / 2, k * px + px / 2)[0] < 128;
            }
            return cells[id] = c;
        }
    }

    /// <summary>Writes one PDF for <paramref name="setup"/> and returns its path. Problems go to <paramref name="warnings"/>.</summary>
    public static string Export(TangramMatcher matcher, SimSetup setup, Paper paper, string outDir, List<string> warnings)
    {
        Transform root = matcher.tangramDiagramRoot != null ? matcher.tangramDiagramRoot : matcher.transform;
        var children = new List<Transform>();
        foreach (Transform c in root) children.Add(c);
        var shapes = SimTangramWorld.ReadDiagramShapes(matcher, children, setup.setSizeMeters, out float scale);
        if (shapes.Count == 0) throw new InvalidOperationException($"No tangram pieces under '{root.name}'.");
        if (shapes.Count != 7) warnings.Add($"'{root.name}' has {shapes.Count} pieces instead of 7, so the piece sizes (scaled from their total area) are off.");
        var ids = SimTangramWorld.AssignMarkerIds(shapes);

        var g = new Guide
        {
            setup = setup,
            setMm = setup.setSizeMeters * 1000f,
            markerMm = setup.markerSizeMeters * 1000f,
            stickerHalf = setup.markerSizeMeters * (0.5f + setup.stickerQuietZoneRatio) * 1000f,
            boardMm = setup.boardSizeMeters * 1000f,
            dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50),
        };
        string tag = $"set {g.setMm:0.#} mm, marker {g.markerMm:0.#} mm";
        g.summary = $"Set {g.setMm:0.#} mm square · marker {g.markerMm:0.#} mm (DICT_4X4_50) · white border {setup.stickerQuietZoneRatio * 100f:0.#} % " +
                    $"(sticker {g.stickerHalf * 2f:0.#} mm) · board {g.boardMm:0.#} mm · diagram '{root.name}' · {DateTime.Now:yyyy-MM-dd}";

        Vector2 size = PaperMm(paper);
        var pages = new List<Page>();
        try
        {
            var order = new List<int>();
            for (int i = 0; i < shapes.Count; i++) order.Add(i);
            order.Sort((a, b) => ids[a].CompareTo(ids[b]));

            // Piece labels
            var pieceItems = new List<Item>();
            foreach (int i in order) pieceItems.Add(PieceItem(g, shapes[i], ids[i], warnings, tag));
            AddSection(pages, size, pieceItems, "Tangram sticker guide · piece labels", PieceIntro(size), warnings, tag);

            // Loose stickers
            var stickerItems = new List<Item>();
            foreach (int i in order) stickerItems.Add(StickerItem(g, ids[i], $"ID {ids[i]} · {DisplayName(shapes[i].type)}"));
            foreach (var (id, _) in SimTangramWorld.CornerLayout(setup)) stickerItems.Add(StickerItem(g, id, $"ID {id} · board {CornerName(id)}"));
            AddSection(pages, size, stickerItems, "Marker stickers", warnings: warnings, tag: tag, intro: new List<string>
            {
                "Cut along the corner marks and keep the white border: the camera needs it around the black square. " +
                "The arrow under each sticker points to its right-hand side (+X). Print this page twice for spares.",
            });

            // Board
            AddBoard(pages, size, g, warnings, tag);
            CheckBoardClearance(g, shapes, scale, warnings, tag);
            CheckSameTypeFrames(shapes, ids, warnings, tag);
            LogNotes(g, shapes, ids, tag);

            var pdf = new Pdf(size.x, size.y);
            for (int p = 0; p < pages.Count; p++)
            {
                pdf.NewPage();
                DrawHeader(pdf, pages[p], g.summary);
                pages[p].body?.Invoke(pdf);
                DrawFooter(pdf, p + 1, pages.Count);
            }
            Directory.CreateDirectory(outDir);
            string path = Path.Combine(outDir, $"StickerGuide_set{g.setMm:0.#}mm_marker{g.markerMm:0.#}mm_{paper}.pdf");
            File.WriteAllBytes(path, pdf.Build($"Tangram sticker guide ({tag})"));
            return path;
        }
        finally
        {
            g.dictionary.Dispose();
        }
    }

    private static List<string> PieceIntro(Vector2 paperMm) => new List<string>
    {
        "1. Print at 100 % (\"Actual size\", not \"Fit to page\"). The bar at the bottom of every page must measure exactly 50 mm.",
        "2. Labels: print this on full-sheet sticker paper, cut along the solid outline and stick each label on the top face of its piece, edges flush.",
        "    Stencils instead: print on plain paper, cut out the outline and the dashed square, lay the stencil on the piece, edges flush, " +
        "and press the marker sticker into the window, upright like this page's text (arrow = the marker's right-hand side).",
        "3. Keep each marker exactly as drawn: never rotate or mirror it. The game reads each piece's angle from its marker's +X.",
    };

    private static void AddSection(List<Page> pages, Vector2 paperMm, List<Item> items, string title, List<string> intro, List<string> warnings, string tag)
    {
        // Each item keeps its page-relative position; oversized items get pages of their own, split into sheets.
        var page = new Page { title = title, intro = intro };
        var placed = new List<(Item, Vector2)>();
        Rect area = ContentArea(paperMm, HeaderHeight(page, paperMm));
        float x = area.xMin, y = area.yMax, rowH = 0f;
        void Flush()
        {
            if (placed.Count == 0) return;
            var snapshot = placed.ToArray();
            page.body = pdf => { foreach (var (item, at) in snapshot) DrawAt(pdf, item, at); };
            pages.Add(page);
            placed.Clear();
            page = new Page { title = title + " (continued)" };
            area = ContentArea(paperMm, HeaderHeight(page, paperMm));
            x = area.xMin; y = area.yMax; rowH = 0f;
        }
        foreach (var item in items)
        {
            Vector2 s = item.bounds.size;
            if (s.x > area.width || s.y > area.height)
            {
                Flush();
                AddTiled(pages, paperMm, item, title, page.intro, warnings, tag);
                page = new Page { title = title + " (continued)" };
                area = ContentArea(paperMm, HeaderHeight(page, paperMm));
                continue;
            }
            if (x > area.xMin && x + s.x > area.xMax) { y -= rowH + Gap; x = area.xMin; rowH = 0f; }
            if (y - s.y < area.yMin) Flush();
            placed.Add((item, new Vector2(x - item.bounds.xMin, y - s.y - item.bounds.yMin)));
            x += s.x + Gap;
            rowH = Mathf.Max(rowH, s.y);
        }
        Flush();
    }

    /// <summary>
    /// Splits an item larger than the paper into overlapping sheets. Shared content repeats in the overlap and carries
    /// + marks; each sheet after the first in a row/column has a dashed trim line on its left/top edge.
    /// </summary>
    private static void AddTiled(List<Page> pages, Vector2 paperMm, Item item, string title, List<string> intro, List<string> warnings, string tag)
    {
        var probe = new Page { title = title, intro = intro };
        Rect area = ContentArea(paperMm, HeaderHeight(probe, paperMm));
        Rect b = item.bounds;
        int cols = b.width <= area.width ? 1 : Mathf.CeilToInt((b.width - Overlap) / (area.width - Overlap));
        int rows = b.height <= area.height ? 1 : Mathf.CeilToInt((b.height - Overlap) / (area.height - Overlap));
        float stepX = cols == 1 ? b.width : (b.width - Overlap) / cols, stepY = rows == 1 ? b.height : (b.height - Overlap) / rows;
        float tileW = cols == 1 ? b.width : stepX + Overlap, tileH = rows == 1 ? b.height : stepY + Overlap;

        var seamsX = new List<float>();
        var seamsY = new List<float>();
        for (int c = 1; c < cols; c++) seamsX.Add(b.xMin + c * stepX + Overlap * 0.5f);
        for (int r = 1; r < rows; r++) seamsY.Add(b.yMax - r * stepY - Overlap * 0.5f);
        var strips = new List<Rect>();
        foreach (float sx in seamsX) strips.Add(new Rect(sx - Overlap * 0.5f, b.yMin, Overlap, b.height));
        foreach (float sy in seamsY) strips.Add(new Rect(b.xMin, sy - Overlap * 0.5f, b.width, Overlap));
        foreach (var (what, rect) in item.keepWhole)
            if (strips.Exists(st => st.Overlaps(rect)))
                warnings.Add($"{item.name} ({tag}): a sheet seam runs through {what}, so it would be taped together. Print on larger paper, " +
                             "or use the loose marker sticker with a stencil.");

        int n = rows * cols;
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                var tile = new Rect(b.xMin + c * stepX, b.yMax - r * stepY - tileH, tileW, tileH);
                int k = r * cols + c + 1;
                string where = n == 1 ? "" : rows == 1 ? (cols == 2 ? (c == 0 ? " (left)" : " (right)") : $" (column {c + 1})")
                    : cols == 1 ? (rows == 2 ? (r == 0 ? " (top)" : " (bottom)") : $" (row {r + 1})") : $" (row {r + 1}, column {c + 1})";
                var page = new Page { title = n == 1 ? title : $"{title} · sheet {k} of {n}{where}", intro = k == 1 ? intro : new List<string>() };
                Rect a = ContentArea(paperMm, HeaderHeight(page, paperMm));
                Vector2 at = new Vector2(a.center.x - tile.center.x, a.center.y - tile.center.y);
                bool trimLeft = c > 0, trimTop = r > 0;
                page.body = pdf =>
                {
                    pdf.Save();
                    pdf.Translate(at);
                    pdf.Save();
                    pdf.Clip(tile);
                    item.draw(pdf);
                    pdf.Stroke(Ink); pdf.Width(0.2f);
                    foreach (float sx in seamsX) for (int m = 1; m <= 3; m++) Cross(pdf, new Vector2(sx, b.yMin + b.height * m / 4f), 4f);
                    foreach (float sy in seamsY) for (int m = 1; m <= 3; m++) Cross(pdf, new Vector2(b.xMin + b.width * m / 4f, sy), 4f);
                    pdf.Restore();
                    pdf.Stroke(Dim); pdf.Width(0.25f); pdf.Dash(2f, 1.5f);
                    if (trimLeft) pdf.Line(new Vector2(tile.xMin, tile.yMin), new Vector2(tile.xMin, tile.yMax));
                    if (trimTop) pdf.Line(new Vector2(tile.xMin, tile.yMax), new Vector2(tile.xMax, tile.yMax));
                    pdf.NoDash();
                    pdf.Fill(Dim);
                    if (trimLeft) pdf.Text("trim", new Vector2(tile.xMin + 1f, tile.yMin + 1.5f), 6f);
                    if (trimTop) pdf.Text("trim", new Vector2(tile.xMax - 1f, tile.yMax - 3f), 6f, align: 1f);
                    pdf.Restore();
                };
                pages.Add(page);
            }
        }
    }

    private static void DrawAt(Pdf pdf, Item item, Vector2 at)
    {
        pdf.Save();
        pdf.Translate(at);
        item.draw(pdf);
        pdf.Restore();
    }

    // ---------- Piece labels ----------

    private static Item PieceItem(Guide g, SimTangramWorld.ShapeInfo s, int id, List<string> warnings, string tag)
    {
        var outline = new Vector2[s.outline.Length];
        for (int k = 0; k < outline.Length; k++) outline[k] = s.outline[k] * 1000f;
        float rot = g.setup.stickerRotationOffsetDeg, sh = g.stickerHalf;
        var sticker = SquareCorners(sh, rot);

        float arrowStart = sh + 1.2f, arrowLen = Mathf.Clamp(sh * 0.6f, 6f, 12f);
        Vector2 a0 = SimTangramWorld.Rotate(new Vector2(arrowStart, 0f), rot), a1 = SimTangramWorld.Rotate(new Vector2(arrowStart + arrowLen, 0f), rot);

        var captions = new List<(string text, Color color)> { ($"ID {id} · {DisplayName(s.type)}", Ink) };
        string name = $"{DisplayName(s.type)} (ID {id}, {tag})";
        if (!SimTangramWorld.SquareFits(s.outline, sh / 1000f, rot))
        {
            bool markerFits = SimTangramWorld.SquareFits(s.outline, g.markerMm * 0.5f / 1000f, rot);
            float maxHalf = LargestFittingHalf(s.outline, rot) * 1000f;
            float maxMarker = 2f * maxHalf / (1f + 2f * g.setup.stickerQuietZoneRatio);
            float minSet = g.setMm * sh / Mathf.Max(maxHalf, 1e-3f);
            if (markerFits)
            {
                captions.Add(("White border overhangs the edge (marker itself fits).", Warn));
                warnings.Add($"{name}: the sticker's white border overhangs the piece; the black marker fits. Light-colored pieces still give the " +
                             $"camera a border. For a full border use a marker ≤ {maxMarker:0.#} mm or a set ≥ {minSet:0} mm.");
            }
            else
            {
                captions.Add(("Marker overhangs the piece: it will be cut off.", Warn));
                warnings.Add($"{name}: the black marker itself overhangs the piece and would be cut off. Use a marker ≤ {maxMarker:0.#} mm or a set ≥ {minSet:0} mm.");
            }
            captions.Add(($"Fits: marker ≤ {maxMarker:0.#} mm, or set ≥ {minSet:0} mm", Warn));
        }

        Rect shape = Bounds(outline);
        Rect bounds = Union(Union(shape, Bounds(sticker)), Bounds(new[] { a0, a1 }));
        float captionTop = bounds.yMin - 1.5f;
        bounds.yMin -= 1.5f + captions.Count * LineH;
        foreach (var (text, _) in captions) bounds = Union(bounds, CenteredTextBox(text, new Vector2(shape.center.x, captionTop), 7.5f));

        // A small ID inside the outline survives cutting and fixes which way is up on a stencil.
        string inner = $"ID {id}";
        var avoid = new[] { Bounds(sticker), Inflate(Bounds(new[] { a0, a1 }), 2.2f) };
        bool hasInner = TryPlaceInside(outline, avoid, new Vector2(Pdf.TextWidth(inner, 7f), 2.4f), out Vector2 innerAt);

        Color tint = Color.Lerp(Color.white, s.color, 0.4f);
        return new Item
        {
            name = $"{DisplayName(s.type)} (ID {id})",
            keepWhole = { ("its marker", Bounds(sticker)) },
            bounds = bounds,
            draw = pdf =>
            {
                pdf.Fill(tint);
                pdf.Poly(outline, fill: true, stroke: false);
                DrawSticker(pdf, g, id, Vector2.zero, rot, edgeLine: true);
                pdf.Stroke(Ink); pdf.Width(0.3f);
                pdf.Poly(outline, fill: false, stroke: true); // over the sticker, so an overhang shows
                DrawArrow(pdf, a0, a1, Dim);
                pdf.Fill(Dim);
                Vector2 dir = (a1 - a0).normalized;
                pdf.Text("+X", (a0 + a1) * 0.5f + new Vector2(-dir.y, dir.x) * 1.2f - new Vector2(0f, 0.6f), 5.5f, align: 0.5f);
                if (hasInner) { pdf.Fill(Ink); pdf.Text(inner, innerAt - new Vector2(0f, 0.85f), 7f, align: 0.5f); }
                float y = captionTop;
                foreach (var (text, color) in captions)
                {
                    y -= LineH;
                    pdf.Fill(color);
                    pdf.Text(text, new Vector2(shape.center.x, y + 0.8f), 7.5f, align: 0.5f);
                }
            },
        };
    }

    /// <summary>Largest sticker half-size that SquareFits at this rotation (sticker centered on the 'center' child).</summary>
    private static float LargestFittingHalf(Vector2[] outline, float rot)
    {
        float lo = 0f, hi = 1f;
        for (int i = 0; i < 40; i++) { float mid = (lo + hi) * 0.5f; if (SimTangramWorld.SquareFits(outline, mid, rot)) lo = mid; else hi = mid; }
        return lo;
    }

    private static bool TryPlaceInside(Vector2[] poly, Rect[] avoid, Vector2 size, out Vector2 best)
    {
        best = Vector2.zero;
        float bestScore = -1f;
        Rect bb = Bounds(poly);
        for (float x = bb.xMin; x <= bb.xMax; x += 0.5f)
        {
            for (float y = bb.yMin; y <= bb.yMax; y += 0.5f)
            {
                var box = new Rect(x - size.x * 0.5f, y - size.y * 0.5f, size.x, size.y);
                var padded = Inflate(box, 1f);
                bool ok = true;
                foreach (var c in Corners(padded)) if (!SimTangramWorld.PointInConvex(c, poly)) { ok = false; break; }
                if (!ok) continue;
                foreach (var r in avoid) if (r.Overlaps(padded)) { ok = false; break; }
                if (!ok) continue;
                // Prefer the roomiest spot: largest distance from the box to the outline.
                float score = float.MaxValue;
                foreach (var c in Corners(box))
                    for (int i = 0; i < poly.Length; i++) score = Mathf.Min(score, DistanceToSegment(c, poly[i], poly[(i + 1) % poly.Length]));
                if (score > bestScore) { bestScore = score; best = new Vector2(x, y); }
            }
        }
        return bestScore >= 0f;
    }

    // ---------- Stickers ----------

    private static Item StickerItem(Guide g, int id, string caption)
    {
        float sh = g.stickerHalf, mark = sh + 4f;
        var bounds = new Rect(-mark, -mark - 2f * LineH - 1f, 2f * mark, 2f * mark + 2f * LineH + 1f);
        bounds = Union(bounds, CenteredTextBox(caption, new Vector2(0f, -mark - LineH), 7.5f));
        return new Item
        {
            bounds = bounds,
            draw = pdf =>
            {
                DrawSticker(pdf, g, id, Vector2.zero, 0f, edgeLine: false);
                pdf.Stroke(Ink); pdf.Width(0.2f);
                foreach (var c in SquareCorners(sh, 0f))
                {
                    Vector2 sx = new Vector2(Mathf.Sign(c.x), 0f), sy = new Vector2(0f, Mathf.Sign(c.y));
                    pdf.Line(c + sx * 1f, c + sx * 4f);
                    pdf.Line(c + sy * 1f, c + sy * 4f);
                }
                pdf.Fill(Ink);
                pdf.Text(caption, new Vector2(0f, -mark - LineH + 0.8f), 7.5f, align: 0.5f);
                float y = -mark - 2f * LineH + 1.6f;
                DrawArrow(pdf, new Vector2(-5f, y), new Vector2(5f, y), Dim);
                pdf.Fill(Dim);
                pdf.Text("+X", new Vector2(6.2f, y - 0.8f), 6f);
            },
        };
    }

    private static void DrawSticker(Pdf pdf, Guide g, int id, Vector2 center, float deg, bool edgeLine)
    {
        float sh = g.stickerHalf, m = g.markerMm, cell = m / 6f;
        pdf.Save();
        pdf.Translate(center, deg);
        pdf.Fill(Color.white);
        pdf.Box(new Rect(-sh, -sh, 2f * sh, 2f * sh), fill: true, stroke: false);
        if (edgeLine)
        {
            pdf.Stroke(Faint); pdf.Width(0.15f); pdf.Dash(1f, 0.8f);
            pdf.Box(new Rect(-sh, -sh, 2f * sh, 2f * sh), fill: false, stroke: true);
            pdf.NoDash();
        }
        // One path for all black cells: a single fill has no seams between neighboring cells.
        var cells = g.Cells(id);
        var rects = new List<Rect>();
        for (int r = 0; r < 6; r++)
            for (int k = 0; k < 6; k++)
                if (cells[r, k]) rects.Add(new Rect(-m * 0.5f + k * cell, m * 0.5f - (r + 1) * cell, cell, cell));
        pdf.Fill(Ink);
        pdf.FillRects(rects);
        pdf.Restore();
    }

    // ---------- Board ----------

    private static void AddBoard(List<Page> pages, Vector2 paperMm, Guide g, List<string> warnings, string tag)
    {
        float w = g.boardMm, half = w * 0.5f, sh = g.stickerHalf;
        var corners = SimTangramWorld.CornerLayout(g.setup);
        // One header line at most, so a 24 cm board still takes only two A4 sheets; the rest is printed on the board.
        var intro = new List<string> { "Trim each sheet along its dashed edge, overlap the sheets so the + marks line up, and tape them together on the back." };
        Rect area = ContentArea(paperMm, HeaderHeight(new Page { intro = intro }, paperMm));
        if (w + 2f <= area.width && w + 2f <= area.height) intro.Clear();
        // Text stays inside the left or right half: two A4 sheets meet in the middle.
        var notes = Wrap("Sit at PLAYER SIDE. As you look at the board, marker 10 is top-left, 11 top-right, 12 bottom-right and 13 bottom-left. " +
                         "Keep them uncovered while playing.", w * 0.5f - 2f * sh - 12f, 7f, hanging: false);
        var item = new Item
        {
            bounds = new Rect(-1f, -1f, w + 2f, w + 2f), // room for the outline's stroke width
            draw = pdf =>
            {
                pdf.Stroke(Ink); pdf.Width(0.3f);
                pdf.Box(new Rect(0f, 0f, w, w), fill: false, stroke: true);
                pdf.Stroke(Faint); pdf.Width(0.2f);
                Cross(pdf, new Vector2(half, half), 6f);
                foreach (var (id, pos) in corners)
                {
                    Vector2 c = new Vector2(half, half) + pos * 1000f;
                    DrawSticker(pdf, g, id, c, 0f, edgeLine: false);
                    float ly = pos.y > 0f ? c.y - sh - 4.5f : c.y + sh + 2f;
                    pdf.Fill(Dim);
                    pdf.Text($"{id} · {CornerName(id)}", new Vector2(c.x, ly), 8f, align: 0.5f);
                }
                pdf.Fill(Dim);
                foreach (float x in new[] { w * 0.25f, w * 0.75f })
                {
                    pdf.Text("DEVICE SIDE (phone + mirror)", new Vector2(x, w - 7f), 8f, align: 0.5f);
                    pdf.Text("PLAYER SIDE (sit here)", new Vector2(x, 4.5f), 8f, align: 0.5f);
                }
                for (int i = 0; i < notes.Count; i++) pdf.Text(notes[i], new Vector2(w * 0.25f, half + 6f + (notes.Count - i) * LineH), 7f, align: 0.5f);
                pdf.Text($"Board {w:0.#} mm · corner markers {g.markerMm:0.#} mm", new Vector2(w * 0.75f, half + 6f + LineH), 7f, align: 0.5f);
            },
        };

        foreach (var (id, pos) in corners)
        {
            Vector2 c = new Vector2(half, half) + pos * 1000f;
            item.keepWhole.Add(($"corner marker {id}", new Rect(c.x - sh, c.y - sh, 2f * sh, 2f * sh)));
        }
        item.name = "Board";
        AddTiled(pages, paperMm, item, "Board with corner markers", intro, warnings, tag);
    }

    /// <summary>
    /// Warns when the solved answer, centered on the board as the simulator places it (SimTangramWorld.ComputeTargets),
    /// covers a corner sticker: the tracker would lose the board plane right when the puzzle is solved.
    /// </summary>
    private static void CheckBoardClearance(Guide g, List<SimTangramWorld.ShapeInfo> shapes, float scale, List<string> warnings, string tag)
    {
        Vector2 min = new Vector2(float.MaxValue, float.MaxValue), max = new Vector2(float.MinValue, float.MinValue);
        var solved = new List<Vector2[]>();
        foreach (var s in shapes)
        {
            var poly = new Vector2[s.outline.Length];
            for (int k = 0; k < poly.Length; k++)
            {
                poly[k] = s.center * scale + SimTangramWorld.Rotate(s.outline[k], s.angleDeg);
                min = Vector2.Min(min, poly[k]); max = Vector2.Max(max, poly[k]);
            }
            solved.Add(poly);
        }
        Vector2 mid = (min + max) * 0.5f;
        foreach (var poly in solved) for (int k = 0; k < poly.Length; k++) poly[k] -= mid;

        List<int> Covered(SimSetup s)
        {
            var hit = new List<int>();
            float half = s.markerSizeMeters * (0.5f + s.stickerQuietZoneRatio);
            foreach (var (id, pos) in SimTangramWorld.CornerLayout(s))
            {
                var square = SquareCorners(half, 0f);
                for (int k = 0; k < 4; k++) square[k] += pos;
                foreach (var poly in solved) if (ConvexOverlap(poly, square)) { hit.Add(id); break; }
            }
            return hit;
        }
        var covered = Covered(g.setup);
        if (covered.Count == 0) return;
        var bigger = JsonUtility.FromJson<SimSetup>(JsonUtility.ToJson(g.setup));
        while (Covered(bigger).Count > 0 && bigger.boardSizeMeters < g.setup.boardSizeMeters * 4f) bigger.boardSizeMeters += 0.005f;
        Vector2 ext = (max - min) * 1000f;
        warnings.Add($"Board ({tag}): solved and centered on the board as the simulator places it, the answer ({ext.x:0} × {ext.y:0} mm) covers corner " +
                     $"marker(s) {string.Join(", ", covered)}, so the board can be lost right when the puzzle is solved. " +
                     $"A {bigger.boardSizeMeters * 1000f:0} mm board keeps all four clear.");
    }

    /// <summary>
    /// Same-type pieces are interchangeable on the table and the matcher may assign a detection to either slot, so if
    /// their 'center'/'direction' children sit differently on the shape, one sticker is off for the matcher whenever they swap.
    /// </summary>
    private static void CheckSameTypeFrames(List<SimTangramWorld.ShapeInfo> shapes, List<int> ids, List<string> warnings, string tag)
    {
        for (int i = 0; i < shapes.Count; i++)
            for (int j = i + 1; j < shapes.Count; j++)
            {
                if (shapes[i].type != shapes[j].type) continue;
                if (!FrameDifference(Simplify(shapes[i].outline), Simplify(shapes[j].outline), out float deg, out float meters)) continue;
                if (Mathf.Abs(deg) > 1f || meters > 0.001f)
                    warnings.Add($"{DisplayName(shapes[i].type)}s ID {ids[i]} and ID {ids[j]} ({tag}): their sticker frames in the diagram differ by " +
                                 $"{Mathf.Abs(deg):0.0}° and {meters * 1000f:0.0} mm ({shapes[i].name} vs {shapes[j].name}). The pieces are interchangeable, " +
                                 "so the matcher sees that error whenever they swap places. Give both diagram pieces the same 'center'/'direction' placement.");
            }
    }

    /// <summary>
    /// Rigid motion taking outline a onto outline b (the same shape, each in its own sticker frame): the rotation is how
    /// far a's sticker is turned in b's frame (smallest over the shape's symmetries), the offset is where its center lands.
    /// </summary>
    private static bool FrameDifference(Vector2[] a, Vector2[] b, out float deg, out float offset)
    {
        deg = 0f; offset = 0f;
        int n = a.Length;
        if (n < 3 || n != b.Length) return false;
        Vector2 ca = Centroid(a), cb = Centroid(b);
        bool found = false;
        for (int k = 0; k < n; k++)
        {
            float ang = Mathf.DeltaAngle(Angle(a[1] - a[0]), Angle(b[(k + 1) % n] - b[k]));
            Vector2 t = cb - SimTangramWorld.Rotate(ca, ang);
            float err = 0f;
            for (int i = 0; i < n; i++) err = Mathf.Max(err, Vector2.Distance(SimTangramWorld.Rotate(a[i], ang) + t, b[(i + k) % n]));
            if (err > 0.002f || (found && Mathf.Abs(ang) >= Mathf.Abs(deg))) continue;
            found = true; deg = ang; offset = t.magnitude;
        }
        return found;
    }

    /// <summary>Console notes that are not errors but worth knowing when making a set.</summary>
    private static void LogNotes(Guide g, List<SimTangramWorld.ShapeInfo> shapes, List<int> ids, string tag)
    {
        var sb = new StringBuilder($"[StickerGuide] {tag}: sticker frames from the diagram (+X = center→direction):");
        for (int i = 0; i < shapes.Count; i++)
        {
            var s = shapes[i];
            var poly = Simplify(s.outline);
            Vector2 centroid = Centroid(poly) * 1000f;
            // How far +X is from the nearest 45° step of the piece's edges: a 'direction' child that is meant to follow an
            // edge or a diagonal should give ~0°. The guide follows it either way, since the matcher does.
            float off = 180f;
            for (int k = 0; k < poly.Length; k++)
                off = Mathf.Min(off, Mathf.Abs(Mathf.Repeat(Angle(poly[(k + 1) % poly.Length] - poly[k]) + 22.5f, 45f) - 22.5f));
            sb.Append($"\n  ID {ids[i]} {s.name}: center is {centroid.magnitude:0.0} mm from the centroid; +X is {off:0.0}° off the nearest 45° edge step");
        }
        Debug.Log(sb.ToString());

        var tracker = FindFirstObjectByType<ArUcoMobileTracker>(FindObjectsInactive.Include);
        if (tracker != null && Mathf.Abs(tracker.markerLength * 1000f - g.markerMm) > 0.05f)
            Debug.Log($"[StickerGuide] Note: ArUcoMobileTracker.markerLength is {tracker.markerLength * 1000f:0.#} mm but this guide prints {g.markerMm:0.#} mm markers. " +
                      "Pose distances scale with markerLength, so set it to the printed size once the physical size is chosen.");
    }

    // ---------- Page furniture ----------

    private static float HeaderHeight(Page page, Vector2 paperMm)
    {
        int lines = 0;
        foreach (var p in page.intro) lines += Wrap(p, paperMm.x - 2f * Margin, 7.5f).Count;
        return 11f + (lines > 0 ? lines * LineH + 2f : 0f);
    }

    private static Rect ContentArea(Vector2 paperMm, float headerH) =>
        new Rect(Margin, Margin + FooterH, paperMm.x - 2f * Margin, paperMm.y - 2f * Margin - FooterH - headerH);

    private static void DrawHeader(Pdf pdf, Page page, string summary)
    {
        float top = pdf.H - Margin;
        pdf.Fill(Ink);
        pdf.Text(page.title, new Vector2(Margin, top - 4f), 12f, bold: true);
        pdf.Fill(Dim);
        pdf.Text(summary, new Vector2(Margin, top - 8.5f), 7f);
        float y = top - 11f;
        pdf.Fill(Ink);
        foreach (var p in page.intro)
            foreach (var line in Wrap(p, pdf.W - 2f * Margin, 7.5f)) { y -= LineH; pdf.Text(line, new Vector2(Margin, y + 0.8f), 7.5f); }
    }

    private static void DrawFooter(Pdf pdf, int page, int pages)
    {
        float x = Margin, y = Margin + 1f;
        // 50 mm scale bar in 10 mm steps.
        for (int i = 0; i < 5; i++)
        {
            pdf.Fill(i % 2 == 0 ? Ink : Color.white);
            pdf.Box(new Rect(x + i * 10f, y, 10f, 2.5f), fill: true, stroke: false);
        }
        pdf.Stroke(Ink); pdf.Width(0.15f);
        pdf.Box(new Rect(x, y, 50f, 2.5f), fill: false, stroke: true);
        pdf.Fill(Ink);
        pdf.Text("0", new Vector2(x, y + 3.6f), 6f, align: 0.5f);
        pdf.Text("50 mm", new Vector2(x + 50f, y + 3.6f), 6f, align: 0.5f);
        pdf.Text("Print at 100 % (Actual size): this bar must measure exactly 50 mm.", new Vector2(x + 54f, y + 0.4f), 7f);
        pdf.Text($"{page} / {pages}", new Vector2(pdf.W - Margin, y + 0.4f), 7f, align: 1f);
    }

    // ---------- Helpers ----------

    private static Vector2 PaperMm(Paper p) => p == Paper.Letter ? new Vector2(215.9f, 279.4f) : p == Paper.A3 ? new Vector2(297f, 420f) : new Vector2(210f, 297f);

    private static string DisplayName(TangramMatcher.TangramShapeType t)
    {
        switch (t)
        {
            case TangramMatcher.TangramShapeType.LargeTriangle: return "large triangle";
            case TangramMatcher.TangramShapeType.MediumTriangle: return "medium triangle";
            case TangramMatcher.TangramShapeType.SmallTriangle: return "small triangle";
            case TangramMatcher.TangramShapeType.Square: return "square";
            case TangramMatcher.TangramShapeType.Parallelogram: return "parallelogram";
            default: return t.ToString();
        }
    }

    private static string CornerName(int id) => id == 10 ? "TL" : id == 11 ? "TR" : id == 12 ? "BR" : id == 13 ? "BL" : "?";

    private static void DrawArrow(Pdf pdf, Vector2 from, Vector2 to, Color color)
    {
        Vector2 d = (to - from).normalized, n = new Vector2(-d.y, d.x);
        const float head = 2.2f;
        pdf.Stroke(color); pdf.Width(0.35f);
        pdf.Line(from, to - d * head * 0.8f);
        pdf.Fill(color);
        pdf.Poly(new[] { to, to - d * head + n * head * 0.45f, to - d * head - n * head * 0.45f }, fill: true, stroke: false);
    }

    private static void Cross(Pdf pdf, Vector2 c, float size)
    {
        pdf.Line(c - new Vector2(size * 0.5f, 0f), c + new Vector2(size * 0.5f, 0f));
        pdf.Line(c - new Vector2(0f, size * 0.5f), c + new Vector2(0f, size * 0.5f));
    }

    private static Vector2[] SquareCorners(float half, float deg)
    {
        var c = new[] { new Vector2(-half, -half), new Vector2(half, -half), new Vector2(half, half), new Vector2(-half, half) };
        for (int i = 0; i < 4; i++) c[i] = SimTangramWorld.Rotate(c[i], deg);
        return c;
    }

    private static Vector2[] Corners(Rect r) => new[] { new Vector2(r.xMin, r.yMin), new Vector2(r.xMax, r.yMin), new Vector2(r.xMax, r.yMax), new Vector2(r.xMin, r.yMax) };

    private static Rect Bounds(IEnumerable<Vector2> pts)
    {
        Vector2 min = new Vector2(float.MaxValue, float.MaxValue), max = new Vector2(float.MinValue, float.MinValue);
        foreach (var p in pts) { min = Vector2.Min(min, p); max = Vector2.Max(max, p); }
        return Rect.MinMaxRect(min.x, min.y, max.x, max.y);
    }

    private static Rect Union(Rect a, Rect b) => Rect.MinMaxRect(Mathf.Min(a.xMin, b.xMin), Mathf.Min(a.yMin, b.yMin), Mathf.Max(a.xMax, b.xMax), Mathf.Max(a.yMax, b.yMax));

    private static Rect Inflate(Rect r, float d) => Rect.MinMaxRect(r.xMin - d, r.yMin - d, r.xMax + d, r.yMax + d);

    private static Rect CenteredTextBox(string text, Vector2 topCenter, float pt)
    {
        float w = Pdf.TextWidth(text, pt);
        return new Rect(topCenter.x - w * 0.5f, topCenter.y - LineH, w, LineH);
    }

    private static float DistanceToSegment(Vector2 p, Vector2 a, Vector2 b)
    {
        Vector2 ab = b - a;
        float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-12f));
        return Vector2.Distance(p, a + ab * t);
    }

    private static Vector2 Centroid(Vector2[] poly)
    {
        float a = 0f; Vector2 c = Vector2.zero;
        for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
        {
            float cross = poly[j].x * poly[i].y - poly[i].x * poly[j].y;
            a += cross;
            c += (poly[j] + poly[i]) * cross;
        }
        return Mathf.Abs(a) > 1e-12f ? c / (3f * a) : Vector2.zero;
    }

    private static float Angle(Vector2 v) => Mathf.Atan2(v.y, v.x) * Mathf.Rad2Deg;

    /// <summary>Drops hull vertices within 1 mm of the previous one or on a straight line (mesh seams leave a few).</summary>
    private static Vector2[] Simplify(Vector2[] poly)
    {
        var pts = new List<Vector2>();
        foreach (var p in poly) if (pts.Count == 0 || Vector2.Distance(p, pts[pts.Count - 1]) > 0.001f) pts.Add(p);
        if (pts.Count > 1 && Vector2.Distance(pts[0], pts[pts.Count - 1]) <= 0.001f) pts.RemoveAt(pts.Count - 1);
        for (int i = 0; i < pts.Count && pts.Count > 3;)
        {
            Vector2 prev = pts[(i + pts.Count - 1) % pts.Count], next = pts[(i + 1) % pts.Count];
            if (Mathf.Abs(Vector2.SignedAngle(pts[i] - prev, next - pts[i])) < 1f) pts.RemoveAt(i); else i++;
        }
        return pts.ToArray();
    }

    /// <summary>Separating-axis test for two convex polygons.</summary>
    private static bool ConvexOverlap(Vector2[] a, Vector2[] b)
    {
        foreach (var poly in new[] { a, b })
            for (int i = 0; i < poly.Length; i++)
            {
                Vector2 e = poly[(i + 1) % poly.Length] - poly[i], axis = new Vector2(-e.y, e.x);
                float aMin = float.MaxValue, aMax = float.MinValue, bMin = float.MaxValue, bMax = float.MinValue;
                foreach (var p in a) { float d = Vector2.Dot(p, axis); aMin = Mathf.Min(aMin, d); aMax = Mathf.Max(aMax, d); }
                foreach (var p in b) { float d = Vector2.Dot(p, axis); bMin = Mathf.Min(bMin, d); bMax = Mathf.Max(bMax, d); }
                if (aMax <= bMin || bMax <= aMin) return false;
            }
        return true;
    }

    private static List<string> Wrap(string text, float widthMm, float pt, bool hanging = true)
    {
        var lines = new List<string>();
        string indent = text.StartsWith("    ") ? "    " : "", next = hanging ? indent + "    " : "";
        var line = new StringBuilder();
        foreach (var word in text.Trim().Split(' '))
        {
            string candidate = line.Length == 0 ? indent + word : line + " " + word;
            if (line.Length > 0 && Pdf.TextWidth(candidate, pt) > widthMm) { lines.Add(line.ToString()); line.Clear().Append(next + word); }
            else line.Clear().Append(candidate);
        }
        if (line.Length > 0) lines.Add(line.ToString());
        return lines;
    }

    // ---------- Minimal PDF writer ----------

    /// <summary>
    /// Vector PDF 1.4 with the standard Helvetica fonts (not embedded). User space is millimeters from the page's
    /// bottom-left corner, so geometry is written at true scale. /PrintScaling /None asks viewers to print at 100 %.
    /// </summary>
    private sealed class Pdf
    {
        public readonly float W, H;
        private const float PtPerMm = 72f / 25.4f;
        private readonly List<string> pages = new List<string>();
        private StringBuilder s;

        public Pdf(float widthMm, float heightMm) { W = widthMm; H = heightMm; }

        public void NewPage()
        {
            if (s != null) pages.Add(s.ToString());
            s = new StringBuilder();
            s.Append(F(PtPerMm)).Append(" 0 0 ").Append(F(PtPerMm)).Append(" 0 0 cm\n1 J 1 j\n");
        }

        public void Save() => s.Append("q\n");
        public void Restore() => s.Append("Q\n");
        public void Translate(Vector2 t, float deg = 0f)
        {
            float r = deg * Mathf.Deg2Rad, c = Mathf.Cos(r), n = Mathf.Sin(r);
            s.Append($"{F(c)} {F(n)} {F(-n)} {F(c)} {F(t.x)} {F(t.y)} cm\n");
        }
        public void Clip(Rect r) => s.Append($"{F(r.x)} {F(r.y)} {F(r.width)} {F(r.height)} re W n\n");
        public void Fill(Color c) => s.Append($"{F(c.r)} {F(c.g)} {F(c.b)} rg\n");
        public void Stroke(Color c) => s.Append($"{F(c.r)} {F(c.g)} {F(c.b)} RG\n");
        public void Width(float mm) => s.Append($"{F(mm)} w\n");
        public void Dash(float on, float off) => s.Append($"[{F(on)} {F(off)}] 0 d\n");
        public void NoDash() => s.Append("[] 0 d\n");

        public void Line(Vector2 a, Vector2 b) => s.Append($"{F(a.x)} {F(a.y)} m {F(b.x)} {F(b.y)} l S\n");

        public void Box(Rect r, bool fill, bool stroke) => s.Append($"{F(r.x)} {F(r.y)} {F(r.width)} {F(r.height)} re {Paint(fill, stroke)}\n");

        public void FillRects(List<Rect> rects)
        {
            if (rects.Count == 0) return;
            foreach (var r in rects) s.Append($"{F(r.x)} {F(r.y)} {F(r.width)} {F(r.height)} re\n");
            s.Append("f\n");
        }

        public void Poly(IList<Vector2> pts, bool fill, bool stroke)
        {
            for (int i = 0; i < pts.Count; i++) s.Append($"{F(pts[i].x)} {F(pts[i].y)} {(i == 0 ? "m" : "l")}\n");
            s.Append("h ").Append(Paint(fill, stroke)).Append('\n');
        }

        /// <summary>align: 0 = left, 0.5 = centered, 1 = right (regular weight only; bold is measured as regular).</summary>
        public void Text(string text, Vector2 at, float pt, bool bold = false, float align = 0f)
        {
            float x = at.x - TextWidth(text, pt) * align;
            s.Append($"BT /{(bold ? "F2" : "F1")} {F(pt / PtPerMm)} Tf {F(x)} {F(at.y)} Td ({Escape(text)}) Tj ET\n");
        }

        public static float TextWidth(string text, float pt)
        {
            float units = 0f;
            foreach (char ch in text) units += CharWidth(ch);
            return units / 1000f * pt / PtPerMm;
        }

        public byte[] Build(string title)
        {
            if (s != null) { pages.Add(s.ToString()); s = null; }
            var objs = new List<string>
            {
                "<< /Type /Catalog /Pages 2 0 R /ViewerPreferences << /PrintScaling /None >> >>",
                null, // pages, filled below
                "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>",
                "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>",
                $"<< /Title ({Escape(title)}) /Producer (StickerGuideExporter) >>",
            };
            var kids = new StringBuilder();
            string box = $"[0 0 {F(W * PtPerMm)} {F(H * PtPerMm)}]";
            foreach (var content in pages)
            {
                kids.Append($"{objs.Count + 1} 0 R ");
                objs.Add($"<< /Type /Page /Parent 2 0 R /MediaBox {box} /Resources << /Font << /F1 3 0 R /F2 4 0 R >> >> /Contents {objs.Count + 2} 0 R >>");
                objs.Add($"<< /Length {content.Length} >>\nstream\n{content}endstream");
            }
            objs[1] = $"<< /Type /Pages /Kids [{kids}] /Count {pages.Count} >>";

            // Everything is ASCII (text is escaped), so string offsets are byte offsets.
            var file = new StringBuilder("%PDF-1.4\n");
            var offsets = new List<int>();
            for (int i = 0; i < objs.Count; i++) { offsets.Add(file.Length); file.Append($"{i + 1} 0 obj\n{objs[i]}\nendobj\n"); }
            int xref = file.Length;
            file.Append($"xref\n0 {objs.Count + 1}\n0000000000 65535 f \n");
            foreach (int o in offsets) file.Append(o.ToString("D10")).Append(" 00000 n \n");
            file.Append($"trailer\n<< /Size {objs.Count + 1} /Root 1 0 R /Info 5 0 R >>\nstartxref\n{xref}\n%%EOF\n");
            return Encoding.ASCII.GetBytes(file.ToString());
        }

        private static string Paint(bool fill, bool stroke) => fill && stroke ? "B" : fill ? "f" : stroke ? "S" : "n";

        private static string F(float v) => Math.Abs(v) < 5e-5f ? "0" : v.ToString("0.####", CultureInfo.InvariantCulture);

        // Helvetica advance widths (1/1000 em) for ASCII 32..126, from the standard AFM.
        private static readonly short[] Widths =
        {
            278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278,
            556, 556, 556, 556, 556, 556, 556, 556, 556, 556, 278, 278, 584, 584, 584, 556,
            1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833, 722, 778,
            667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556,
            333, 556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556,
            556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334, 260, 334, 584,
        };

        // The few non-ASCII characters the guide uses, as WinAnsi codes. ≤ and ≥ are written as <= and >=.
        private static readonly Dictionary<char, (int code, short width)> Extra = new Dictionary<char, (int, short)>
        {
            { '°', (0xB0, 400) }, { '×', (0xD7, 584) }, { '·', (0xB7, 278) }, { '–', (0x96, 556) },
        };

        private static float CharWidth(char ch)
        {
            if (ch >= 32 && ch <= 126) return Widths[ch - 32];
            if (ch == '≤' || ch == '≥') return 584f + 584f; // written as "<=" / ">="
            return Extra.TryGetValue(ch, out var e) ? e.width : 556f;
        }

        private static string Escape(string text)
        {
            var sb = new StringBuilder();
            foreach (char ch in text)
            {
                if (ch == '(' || ch == ')' || ch == '\\') sb.Append('\\').Append(ch);
                else if (ch >= 32 && ch <= 126) sb.Append(ch);
                else if (ch == '≤') sb.Append("<=");
                else if (ch == '≥') sb.Append(">=");
                else if (Extra.TryGetValue(ch, out var e)) sb.Append('\\').Append(Convert.ToString(e.code, 8));
                else sb.Append('?');
            }
            return sb.ToString();
        }
    }
}
