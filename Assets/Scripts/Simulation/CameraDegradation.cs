using System;
using System.Collections.Generic;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgcodecsModule;
using OpenCVForUnity.ImgprocModule;
using UnityEngine;

public enum CameraDegradationPreset { Clean, Typical, Harsh }

/// <summary>
/// Image-quality knobs for a phone front camera looking at the table through a clip-on mirror.
/// Values are in output-image pixels / 8-bit levels unless noted.
/// </summary>
[Serializable]
public class CameraDegradationSettings
{
    [Header("Mirror")]
    [Tooltip("A planar mirror reverses the image. The tracker un-flips it when useFrontCamera is on, exactly as on device.")]
    public bool mirrorFlip = true;
    [Tooltip("Second-surface reflection of a glass/acrylic mirror: a faint shifted copy of the image.")]
    [Range(0f, 0.5f)] public float ghostStrength = 0.1f;
    public Vector2 ghostOffsetPx = new Vector2(0f, 3f);

    [Header("Lens")]
    [Tooltip("Radial distortion k1 (negative = barrel). Wide front cameras are typically -0.05 .. -0.25.")]
    public float k1 = -0.12f;
    public float k2 = 0.02f;
    [Tooltip("Defocus blur sigma. Fixed-focus front cameras are slightly soft at 20–35 cm.")]
    public float defocusSigmaPx = 1.0f;
    [Tooltip("Brightness loss at the image corners (lens vignetting).")]
    [Range(0f, 0.8f)] public float vignetting = 0.3f;

    [Header("Lighting")]
    public float exposure = 1.0f;
    [Tooltip("Uneven room light: brightness drops by this fraction across the frame.")]
    [Range(0f, 0.8f)] public float lightGradient = 0.25f;
    public float lightGradientAngleDeg = 30f;
    [Tooltip("Indoor white balance tint (multiplied per channel).")]
    public Color whiteBalance = new Color(1f, 0.96f, 0.86f, 1f);
    [Tooltip("Auto-exposure wobble amplitude (fraction of brightness).")]
    [Range(0f, 0.2f)] public float exposureFlicker = 0.03f;

    [Header("Sensor / Encoding")]
    [Tooltip("Gaussian sensor noise in 8-bit levels.")]
    public float noiseSigma = 5f;
    [Range(10, 100)] public int jpegQuality = 75;
    [Tooltip("Chance a frame never arrives (camera/thermal hiccups).")]
    [Range(0f, 0.5f)] public float frameDropProbability = 0.03f;

    public static CameraDegradationSettings FromPreset(CameraDegradationPreset preset)
    {
        var s = new CameraDegradationSettings();
        switch (preset)
        {
            case CameraDegradationPreset.Clean:
                s.ghostStrength = 0f; s.k1 = 0f; s.k2 = 0f; s.defocusSigmaPx = 0f; s.vignetting = 0f;
                s.lightGradient = 0f; s.whiteBalance = Color.white; s.exposureFlicker = 0f;
                s.noiseSigma = 0f; s.jpegQuality = 100; s.frameDropProbability = 0f;
                break;
            case CameraDegradationPreset.Harsh:
                s.ghostStrength = 0.22f; s.ghostOffsetPx = new Vector2(1f, 5f);
                s.k1 = -0.22f; s.k2 = 0.05f; s.defocusSigmaPx = 2.0f; s.vignetting = 0.5f;
                s.exposure = 0.75f; s.lightGradient = 0.5f; s.whiteBalance = new Color(1f, 0.9f, 0.72f, 1f);
                s.exposureFlicker = 0.08f; s.noiseSigma = 12f; s.jpegQuality = 45; s.frameDropProbability = 0.1f;
                break;
        }
        return s;
    }
}

/// <summary>
/// Turns an ideal pinhole render into something closer to a real front-camera frame.
/// Order follows the light path: mirror (flip, ghost) → lens (distortion, defocus, vignetting)
/// → scene light (gradient, white balance, exposure) → sensor noise → JPEG.
///
/// Distortion uses remap tables. The ideal render must be taken with focal length
/// <see cref="RenderFocalScale"/> × the output focal length so barrel distortion has pixels to pull in.
/// </summary>
public sealed class CameraDegradation : IDisposable
{
    private Mat mapX, mapY, gainMap, scratch, scratchF, noise;
    private int mapW, mapH;
    private float mapK1 = float.NaN, mapK2 = float.NaN, mapFocal;
    private float gainVig = -1f, gainGrad = -1f, gainAngle;
    private Color gainWb;
    private readonly System.Random rng = new System.Random(1234);

    /// <summary>Ideal render focal / output focal (≤ 1) so the distorted output has no empty borders.</summary>
    public static float RenderFocalScale(float k1, float k2, int width, int height, float focalPx)
    {
        if (Mathf.Approximately(k1, 0f) && Mathf.Approximately(k2, 0f)) return 1f;
        float xd = width * 0.5f / focalPx, yd = height * 0.5f / focalPx;
        Vector2 u = Undistort(xd, yd, k1, k2);
        float overscan = Mathf.Max(1f, Mathf.Max(Mathf.Abs(u.x) / xd, Mathf.Abs(u.y) / yd)) * 1.02f;
        return 1f / overscan;
    }

    /// <summary>Inverts r_d = r_u (1 + k1 r_u² + k2 r_u⁴) by fixed-point iteration (converges for moderate k).</summary>
    public static Vector2 Undistort(float xd, float yd, float k1, float k2)
    {
        float x = xd, y = yd;
        for (int i = 0; i < 8; i++)
        {
            float r2 = x * x + y * y;
            float k = 1f + k1 * r2 + k2 * r2 * r2;
            x = xd / k; y = yd / k;
        }
        return new Vector2(x, y);
    }

    /// <summary>Decide before capturing whether this frame never arrives (camera/thermal hiccups).</summary>
    public bool ShouldDrop(CameraDegradationSettings s) => s.frameDropProbability > 0f && rng.NextDouble() < s.frameDropProbability;

    /// <summary>Degrades <paramref name="rgb"/> (CV_8UC3) in place.</summary>
    public void Apply(Mat rgb, CameraDegradationSettings s, float focalPx, float time)
    {
        int w = rgb.cols(), h = rgb.rows();
        if (scratch == null) scratch = new Mat();

        if (s.mirrorFlip) Core.flip(rgb, rgb, 1);

        if (s.ghostStrength > 0.001f)
        {
            using (var m = new Mat(2, 3, CvType.CV_64F))
            {
                m.put(0, 0, 1, 0, s.ghostOffsetPx.x, 0, 1, s.ghostOffsetPx.y);
                Imgproc.warpAffine(rgb, scratch, m, rgb.size(), Imgproc.INTER_LINEAR, Core.BORDER_REPLICATE);
            }
            Core.addWeighted(rgb, 1.0 - s.ghostStrength, scratch, s.ghostStrength, 0, rgb);
        }

        float renderScale = RenderFocalScale(s.k1, s.k2, w, h, focalPx);
        if (!Mathf.Approximately(renderScale, 1f))
        {
            EnsureDistortionMaps(w, h, s.k1, s.k2, focalPx, renderScale);
            Imgproc.remap(rgb, scratch, mapX, mapY, Imgproc.INTER_LINEAR, Core.BORDER_REPLICATE);
            scratch.copyTo(rgb);
        }

        if (s.defocusSigmaPx > 0.05f)
            Imgproc.GaussianBlur(rgb, rgb, new Size(0, 0), s.defocusSigmaPx);

        // Light + sensor in float so gains and noise do not band or clip early.
        if (scratchF == null) scratchF = new Mat();
        rgb.convertTo(scratchF, CvType.CV_32FC3, 1.0 / 255.0);
        EnsureGainMap(w, h, s.vignetting, s.lightGradient, s.lightGradientAngleDeg, s.whiteBalance);
        Core.multiply(scratchF, gainMap, scratchF);
        float flicker = 1f + s.exposureFlicker * Mathf.Sin(time * 1.7f) * Mathf.Sin(time * 0.43f + 1f);
        double e = s.exposure * flicker;
        Core.multiply(scratchF, new Scalar(e, e, e), scratchF);
        if (s.noiseSigma > 0.01f)
        {
            if (noise == null || noise.cols() != w || noise.rows() != h) { noise?.Dispose(); noise = new Mat(h, w, CvType.CV_32FC3); }
            Core.randn(noise, 0, s.noiseSigma / 255.0);
            Core.add(scratchF, noise, scratchF);
        }
        scratchF.convertTo(rgb, CvType.CV_8UC3, 255.0); // saturates to 0..255

        if (s.jpegQuality < 100)
        {
            using (var buf = new MatOfByte())
            using (var prm = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, s.jpegQuality))
            {
                Imgcodecs.imencode(".jpg", rgb, buf, prm);
                using (var decoded = Imgcodecs.imdecode(buf, Imgcodecs.IMREAD_COLOR)) decoded.copyTo(rgb);
            }
        }
    }

    private void EnsureDistortionMaps(int w, int h, float k1, float k2, float focalPx, float renderScale)
    {
        if (mapX != null && mapW == w && mapH == h && mapK1 == k1 && mapK2 == k2 && mapFocal == focalPx) return;
        mapX?.Dispose(); mapY?.Dispose();
        mapX = new Mat(h, w, CvType.CV_32FC1);
        mapY = new Mat(h, w, CvType.CV_32FC1);
        float cx = w * 0.5f, cy = h * 0.5f, fr = focalPx * renderScale;
        var rowX = new float[w];
        var rowY = new float[w];
        for (int v = 0; v < h; v++)
        {
            for (int u = 0; u < w; u++)
            {
                // For each output (distorted) pixel, sample the ideal render where that ray landed.
                Vector2 p = Undistort((u + 0.5f - cx) / focalPx, (v + 0.5f - cy) / focalPx, k1, k2);
                rowX[u] = fr * p.x + cx - 0.5f;
                rowY[u] = fr * p.y + cy - 0.5f;
            }
            mapX.put(v, 0, rowX);
            mapY.put(v, 0, rowY);
        }
        mapW = w; mapH = h; mapK1 = k1; mapK2 = k2; mapFocal = focalPx;
    }

    private void EnsureGainMap(int w, int h, float vignetting, float gradient, float angleDeg, Color wb)
    {
        if (gainMap != null && gainMap.cols() == w && gainMap.rows() == h && gainVig == vignetting
            && gainGrad == gradient && gainAngle == angleDeg && gainWb == wb) return;
        gainMap?.Dispose();
        var single = new Mat(h, w, CvType.CV_32FC1);
        float cx = w * 0.5f, cy = h * 0.5f, rMax2 = cx * cx + cy * cy;
        Vector2 dir = new Vector2(Mathf.Cos(angleDeg * Mathf.Deg2Rad), Mathf.Sin(angleDeg * Mathf.Deg2Rad));
        float halfDiag = Mathf.Sqrt(rMax2);
        var row = new float[w];
        for (int v = 0; v < h; v++)
        {
            for (int u = 0; u < w; u++)
            {
                float dx = u - cx, dy = v - cy;
                float vig = 1f - vignetting * (dx * dx + dy * dy) / rMax2;
                float t = 0.5f + 0.5f * (dx * dir.x + dy * dir.y) / halfDiag; // 0..1 across the frame
                row[u] = vig * (1f - gradient * t);
            }
            single.put(v, 0, row);
        }
        var channels = new List<Mat> { new Mat(), new Mat(), new Mat() };
        single.convertTo(channels[0], CvType.CV_32FC1, wb.r);
        single.convertTo(channels[1], CvType.CV_32FC1, wb.g);
        single.convertTo(channels[2], CvType.CV_32FC1, wb.b);
        gainMap = new Mat();
        Core.merge(channels, gainMap);
        foreach (var c in channels) c.Dispose();
        single.Dispose();
        gainVig = vignetting; gainGrad = gradient; gainAngle = angleDeg; gainWb = wb;
    }

    public void Dispose()
    {
        mapX?.Dispose(); mapY?.Dispose(); gainMap?.Dispose(); scratch?.Dispose(); scratchF?.Dispose(); noise?.Dispose();
        mapX = mapY = gainMap = scratch = scratchF = noise = null;
    }
}
