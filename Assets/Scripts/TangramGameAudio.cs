using UnityEngine;

/// <summary>
/// Procedural sound effects for the tangram game, so no audio assets are needed.
/// Clips are synthesized once in Awake (sine + soft 2nd harmonic with exponential decay).
/// </summary>
public class TangramGameAudio : MonoBehaviour
{
    public bool soundEnabled = true;
    [Tooltip("Vibrate on puzzle completion (Android/iOS only).")]
    public bool hapticsEnabled = true;
    [Range(0f, 1f)] public float volume = 0.6f;

    private const int SampleRate = 44100;

    private AudioSource source;
    private AudioClip tapClip, placeClip, lostClip, hintClip, countdownClip, goClip, completeClip;

    void Awake()
    {
        source = GetComponent<AudioSource>();
        if (source == null) source = gameObject.AddComponent<AudioSource>();
        source.playOnAwake = false;
        source.spatialBlend = 0f;

        tapClip = Synthesize("tap", new[] { 660f }, 0.04f, 30f);
        placeClip = Synthesize("place", new[] { 784f, 1175f }, 0.07f, 9f);
        lostClip = Synthesize("lost", new[] { 392f, 330f }, 0.08f, 12f);
        hintClip = Synthesize("hint", new[] { 988f, 1319f, 988f }, 0.06f, 14f);
        countdownClip = Synthesize("count", new[] { 523f }, 0.05f, 16f);
        goClip = Synthesize("go", new[] { 1047f }, 0.08f, 8f);
        // C major arpeggio up to the octave
        completeClip = Synthesize("complete", new[] { 523f, 659f, 784f, 1047f, 1319f }, 0.09f, 4f);
    }

    public void PlayTap() => Play(tapClip, 1f);
    public void PlayHint() => Play(hintClip, 1f);
    public void PlayLost() => Play(lostClip, 1f);
    public void PlayCountdown() => Play(countdownClip, 1f);
    public void PlayGo() => Play(goClip, 1f);

    /// <summary>Pitch rises with progress so the player can hear how close they are.</summary>
    public void PlayPlaced(int placedCount, int total)
    {
        float t = total > 1 ? Mathf.Clamp01((placedCount - 1f) / (total - 1f)) : 1f;
        Play(placeClip, Mathf.Lerp(1f, 1.5f, t));
    }

    public void PlayComplete()
    {
        Play(completeClip, 1f);
        if (hapticsEnabled)
        {
#if UNITY_ANDROID || UNITY_IOS
            Handheld.Vibrate();
#endif
        }
    }

    private void Play(AudioClip clip, float pitch)
    {
        if (!soundEnabled || clip == null || source == null) return;
        source.pitch = pitch;
        source.PlayOneShot(clip, volume);
    }

    /// <summary>Notes play back-to-back (noteSeconds apart) and each rings out with exp(-decay*t).</summary>
    private static AudioClip Synthesize(string name, float[] notes, float noteSeconds, float decay)
    {
        int step = Mathf.CeilToInt(SampleRate * noteSeconds);
        int ring = Mathf.CeilToInt(SampleRate * Mathf.Min(1.2f, 5f / decay));
        int length = step * (notes.Length - 1) + ring;
        var data = new float[length];

        for (int n = 0; n < notes.Length; n++)
        {
            int start = n * step;
            float f = notes[n];
            for (int i = 0; start + i < length; i++)
            {
                float t = i / (float)SampleRate;
                float env = Mathf.Min(1f, t / 0.004f) * Mathf.Exp(-decay * t); // 4ms attack avoids clicks
                float s = Mathf.Sin(2f * Mathf.PI * f * t) + 0.3f * Mathf.Sin(4f * Mathf.PI * f * t);
                data[start + i] += 0.35f * env * s;
            }
        }

        var clip = AudioClip.Create(name, length, 1, SampleRate, false);
        clip.SetData(data, 0);
        return clip;
    }
}
