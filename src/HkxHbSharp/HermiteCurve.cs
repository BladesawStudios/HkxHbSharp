namespace HkxHbSharp;

/// <summary>Cubic Hermite spline through keys, clamped to the first and last value outside their range.</summary>
public static class HermiteCurve
{
    public static float Evaluate(IReadOnlyList<HermiteKey> keys, float t)
    {
        if (keys.Count == 0) return 0f;
        if (keys.Count == 1) return keys[0].Value;

        int last = keys.Count - 1;
        if (t <= keys[0].Time) return keys[0].Value;
        if (t >= keys[last].Time) return keys[last].Value;

        // The interval [i, i + 1] holding t.
        int low = 0, high = last;
        while (low <= high)
        {
            int mid = (low + high) >> 1;
            if (keys[mid].Time <= t)
            {
                if (mid + 1 < keys.Count && keys[mid + 1].Time > t) { low = mid; break; }
                low = mid + 1;
            }
            else high = mid - 1;
        }

        int i = Math.Clamp(low, 0, last - 1);
        HermiteKey k0 = keys[i], k1 = keys[i + 1];

        float h = k1.Time - k0.Time;
        if (h < 1e-7f) return k0.Value;

        float s = (t - k0.Time) / h;
        float s2 = s * s, s3 = s2 * s;

        return (2f * s3 - 3f * s2 + 1f) * k0.Value
             + (s3 - 2f * s2 + s) * (h * k0.OutSlope)
             + (-2f * s3 + 3f * s2) * k1.Value
             + (s3 - s2) * (h * k1.InSlope);
    }
}
