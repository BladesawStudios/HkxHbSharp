using System.Numerics;

namespace HkxHbSharp;

/// <summary>
/// A helper-bone rig as authored in a <c>.bphhb</c>: driver bones watch the pose of a few skeleton bones, curves turn
/// what they see into numbers, and pose-driven bones turn those numbers back into the transforms of the secondary bones
/// (shoulder pads, skirt anchors, twist and corrective joints) that the animation itself leaves alone.
/// </summary>
public sealed class HelperBoneData
{
    /// <summary>Bone names, in the file's own numbering. Every <c>BoneId</c> below indexes this list.</summary>
    public List<string> Bones { get; } = [];
    public List<DriverBone> DriverBones { get; } = [];
    public List<ConnectionCurve> ConnectionCurves { get; } = [];
    public List<Output> Outputs { get; } = [];
    public List<DrivenBone> DrivenBones { get; } = [];
    public List<PoseDriven> PoseDrivens { get; } = [];
}

/// <summary>Watches one bone against a base bone. Swing-twist about <see cref="AimAxis"/> gives roll, horizontal and vertical bend.</summary>
public sealed class DriverBone
{
    public int BoneId { get; set; }
    public int BaseBoneId { get; set; }
    public Vector3 BaseTranslate { get; set; }
    public Quaternion BaseRotate { get; set; } = Quaternion.Identity;
    public Vector3 AimAxis { get; set; } = Vector3.UnitX;
    public Vector3 UpAxis { get; set; } = Vector3.UnitY;
}

/// <summary>Which motion of a driver bone a curve reads.</summary>
public enum DriverAttribute
{
    /// <summary>Twist about the aim axis.</summary>
    Roll = 0,
    /// <summary>Swing around the up axis.</summary>
    BendH = 1,
    /// <summary>Swing around the side axis, aim x up.</summary>
    BendV = 2,
}

/// <summary>A cubic Hermite curve from one driver bone's angle to a value.</summary>
public sealed class ConnectionCurve
{
    public int DriverBoneId { get; set; }
    public DriverAttribute Attr { get; set; }
    public List<HermiteKey> Keys { get; } = [];
}

/// <summary>One Hermite key as authored: time, value, in slope, out slope.</summary>
public readonly record struct HermiteKey(float Time, float Value, float InSlope, float OutSlope);

/// <summary>The sum of up to eight curves, so one channel can answer to several drivers.</summary>
public sealed class Output
{
    public List<int> ConnectionCurveIds { get; } = [];
}

/// <summary>Says which pose-driven transform a skeleton bone takes its rotation and translation from.</summary>
public sealed class DrivenBone
{
    public int BoneId { get; set; }
    public int TranslateDrivenType { get; set; } = -1;
    public int TranslateDrivenId { get; set; } = -1;
    public int RotateDrivenType { get; set; } = -1;
    public int RotateDrivenId { get; set; } = -1;
}

/// <summary>A channel is either a fixed number or the value of an <see cref="Output"/>.</summary>
public readonly record struct ChannelBinding(bool IsBound, int OutputId, float Constant)
{
    public static ChannelBinding FromConstant(float value) => new(false, 0, value);
    public static ChannelBinding FromOutput(int id) => new(true, id, 0f);
}

/// <summary>A transform built relative to a base bone from bend/roll rotations and translations.</summary>
public sealed class PoseDriven
{
    public int BaseBoneId { get; set; }
    public Vector3 BaseTranslate { get; set; }
    public Quaternion BaseRotate { get; set; } = Quaternion.Identity;
    public Vector3 AimAxis { get; set; } = Vector3.UnitX;
    public Vector3 UpAxis { get; set; } = Vector3.UnitY;

    public ChannelBinding Roll { get; set; }
    public ChannelBinding BendH { get; set; }
    public ChannelBinding BendV { get; set; }
    public ChannelBinding TranslateX { get; set; }
    public ChannelBinding TranslateY { get; set; }
    public ChannelBinding TranslateZ { get; set; }
}
