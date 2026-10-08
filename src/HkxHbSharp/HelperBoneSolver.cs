using System.Numerics;

namespace HkxHbSharp;

/// <summary>
/// Evaluates a helper-bone rig against a pose: reads each driver bone's swing and twist against its base, runs them
/// through the curves, sums the curves into outputs, builds the pose-driven transforms from those, and writes them over
/// the driven bones.
/// </summary>
/// <remarks>
/// Ported from Marrow's solver and, like it, not checked against the game: the swing-twist convention and the order the
/// bend and roll rotations combine in are inferred from the file's shape. So is how a driven bone takes what it is given:
/// as an addition to the animation's own movement of it, which for a bone the animation leaves alone is the same as taking
/// the pose outright. Matrices are row-vector, <c>System.Numerics</c>'s way, so a child's world is local * parent.
/// </remarks>
public sealed class HelperBoneSolver
{
    private readonly float[] _roll;
    private readonly float[] _bendH;
    private readonly float[] _bendV;
    private readonly float[] _curves;
    private readonly float[] _outputs;
    private readonly bool[] _poseValid;

    // What each pose adds to its authored rest: the turn the bend and roll channels make, and the shift the translation
    // channels make. A bone the animation moves keeps that movement and takes these on top.
    private readonly Quaternion[] _poseTurn;
    private readonly Vector3[] _poseShift;

    // Which bones of the pose were supplied; empty means all of them. Set by <see cref="Evaluate"/>.
    private bool[] _available = [];

    public HelperBoneData Data { get; }

    public HelperBoneSolver(HelperBoneData data)
    {
        Data = data ?? throw new ArgumentNullException(nameof(data));
        _roll = new float[data.DriverBones.Count];
        _bendH = new float[data.DriverBones.Count];
        _bendV = new float[data.DriverBones.Count];
        _curves = new float[data.ConnectionCurves.Count];
        _outputs = new float[data.Outputs.Count];
        _poseValid = new bool[data.PoseDrivens.Count];
        _poseTurn = new Quaternion[data.PoseDrivens.Count];
        _poseShift = new Vector3[data.PoseDrivens.Count];
    }

    private bool Has(int bone) => bone >= 0 && (_available.Length == 0 ? true : bone < _available.Length && _available[bone]);

    /// <summary>
    /// Solves the rig in place. <paramref name="transforms"/> holds the bones' world matrices numbered as
    /// <see cref="HelperBoneData.Bones"/> numbers them; the driven ones are overwritten.
    /// </summary>
    /// <param name="available">
    /// Which of those bones were really supplied. A rig can name bones its host does not have - an armour's helper bones
    /// read the body it is worn on (<c>Link:Waist</c>), which the armour alone lacks. A driver on a missing bone reads as
    /// at rest, and a pose anchored to one drives nothing, so the bones stay where the animation has them rather than
    /// being posed from a matrix that was never there. Null or empty means every bone is there.
    /// </param>
    public void Solve(Matrix4x4[] transforms, bool[]? available = null)
    {
        Evaluate(transforms, available);

        Matrix4x4[] animated = (Matrix4x4[])transforms.Clone();
        foreach (DrivenBone driven in Data.DrivenBones) Drive(driven, animated, transforms);
    }

    /// <summary>
    /// Reads the drivers from a pose and works out what every pose-driven transform adds, without touching any bone. The
    /// driven bones are then taken one at a time with <see cref="Drive"/>, which lets a host with more bones than this
    /// rig names carry the rest of its skeleton along between them.
    /// </summary>
    public void Evaluate(Matrix4x4[] transforms, bool[]? available = null)
    {
        ArgumentNullException.ThrowIfNull(transforms);
        _available = available ?? [];

        for (int d = 0; d < Data.DriverBones.Count; d++) ReadDriver(d, transforms);

        for (int c = 0; c < Data.ConnectionCurves.Count; c++)
        {
            ConnectionCurve curve = Data.ConnectionCurves[c];
            float input = 0f;
            if (curve.DriverBoneId >= 0 && curve.DriverBoneId < Data.DriverBones.Count)
                input = curve.Attr switch
                {
                    DriverAttribute.Roll => _roll[curve.DriverBoneId],
                    DriverAttribute.BendH => _bendH[curve.DriverBoneId],
                    DriverAttribute.BendV => _bendV[curve.DriverBoneId],
                    _ => 0f,
                };
            _curves[c] = HermiteCurve.Evaluate(curve.Keys, input);
        }

        for (int o = 0; o < Data.Outputs.Count; o++)
        {
            float sum = 0f;
            foreach (int id in Data.Outputs[o].ConnectionCurveIds)
                if (id >= 0 && id < _curves.Length) sum += _curves[id];
            _outputs[o] = sum;
        }

        for (int p = 0; p < Data.PoseDrivens.Count; p++) BuildPose(p, Data.PoseDrivens[p], out _poseValid[p]);
    }

    /// <summary>A driver's roll, horizontal bend and vertical bend, from its rotation relative to its base bone.</summary>
    private void ReadDriver(int d, Matrix4x4[] transforms)
    {
        DriverBone driver = Data.DriverBones[d];
        _roll[d] = _bendH[d] = _bendV[d] = 0f;

        if (driver.BoneId >= transforms.Length || driver.BaseBoneId >= transforms.Length
            || !Has(driver.BoneId) || !Has(driver.BaseBoneId)) return;
        if (!Matrix4x4.Invert(transforms[driver.BaseBoneId], out Matrix4x4 baseInverse)) return;

        Matrix4x4.Decompose(transforms[driver.BoneId] * baseInverse, out _, out Quaternion relative, out _);
        Quaternion delta = Quaternion.Normalize(Quaternion.Inverse(driver.BaseRotate) * Quaternion.Normalize(relative));

        Vector3 aim = Vector3.Normalize(driver.AimAxis);
        Vector3 up = Vector3.Normalize(driver.UpAxis);
        Vector3 side = Vector3.Normalize(Vector3.Cross(aim, up));

        // Twist is the part of the rotation about the aim axis; swing is what is left.
        Vector3 vector = new(delta.X, delta.Y, delta.Z);
        Quaternion twist = new(Vector3.Dot(vector, aim) * aim, delta.W);
        twist = twist.LengthSquared() > 1e-8f ? Quaternion.Normalize(twist) : Quaternion.Identity;

        float roll = 2f * MathF.Atan2(Vector3.Dot(new Vector3(twist.X, twist.Y, twist.Z), aim), twist.W);
        if (roll > MathF.PI) roll -= 2f * MathF.PI;
        if (roll < -MathF.PI) roll += 2f * MathF.PI;
        _roll[d] = roll;

        Quaternion swing = Quaternion.Normalize(delta * Quaternion.Inverse(twist));
        Vector3 swung = Vector3.Transform(aim, swing);
        _bendH[d] = MathF.Atan2(Vector3.Dot(swung, side), Vector3.Dot(swung, aim));
        _bendV[d] = MathF.Atan2(-Vector3.Dot(swung, up), Vector3.Dot(swung, aim));
    }

    private void BuildPose(int index, PoseDriven pose, out bool valid)
    {
        _poseTurn[index] = Quaternion.Identity;
        _poseShift[index] = Vector3.Zero;

        // A pose anchored to a bone that is not there cannot be placed; one with no base bone is placed at the origin.
        valid = pose.BaseBoneId < 0 || Has(pose.BaseBoneId);
        if (!valid) return;

        Vector3 aim = Vector3.Normalize(pose.AimAxis);
        Vector3 up = Vector3.Normalize(pose.UpAxis);
        Vector3 side = Vector3.Normalize(Vector3.Cross(aim, up));

        _poseTurn[index] = Quaternion.CreateFromAxisAngle(up, Channel(pose.BendH))
                         * Quaternion.CreateFromAxisAngle(side, Channel(pose.BendV))
                         * Quaternion.CreateFromAxisAngle(aim, Channel(pose.Roll));
        _poseShift[index] = new Vector3(Channel(pose.TranslateX), Channel(pose.TranslateY), Channel(pose.TranslateZ));
    }

    /// <summary>
    /// Poses one driven bone in <paramref name="current"/>: from where the animation has it against its pose's base bone,
    /// carried to where that base bone is now, with the rotation and translation its poses add put on top.
    /// </summary>
    /// <param name="animated">The pose as the animation left it, before any bone was driven.</param>
    /// <param name="current">The pose as it stands, with the bones driven so far and what hangs from them already moved.</param>
    /// <remarks>
    /// For a bone the animation leaves at its rest - hair, a horn, a twist joint - the result is the pose's rest and what the
    /// channels add, as it would be had the pose replaced the bone. A bone the animation does move, such as a head or a
    /// spine the rig also drives, keeps that movement: replacing it with the rest left everything that hung from it behind
    /// while the rest of the body went on.
    /// </remarks>
    public void Drive(DrivenBone driven, Matrix4x4[] animated, Matrix4x4[] current)
    {
        if (driven.BoneId < 0 || driven.BoneId >= current.Length || !Has(driven.BoneId)) return;

        bool rotates = driven.RotateDrivenType == 0 && driven.RotateDrivenId >= 0 && driven.RotateDrivenId < _poseValid.Length && _poseValid[driven.RotateDrivenId];
        bool translates = driven.TranslateDrivenType == 0 && driven.TranslateDrivenId >= 0 && driven.TranslateDrivenId < _poseValid.Length && _poseValid[driven.TranslateDrivenId];
        if (!rotates && !translates) return;

        // Where the animation has the bone against the base bone, carried onto the base bone as it stands now.
        int seed = Data.PoseDrivens[rotates ? driven.RotateDrivenId : driven.TranslateDrivenId].BaseBoneId;
        Matrix4x4 animatedBase = Base(animated, seed), currentBase = Base(current, seed);
        if (!Matrix4x4.Invert(animatedBase, out Matrix4x4 animatedInverse)) return;
        current[driven.BoneId] = animated[driven.BoneId] * animatedInverse * currentBase;

        if (rotates) Add(driven.BoneId, driven.RotateDrivenId, current, turn: true);
        if (translates) Add(driven.BoneId, driven.TranslateDrivenId, current, turn: false);
    }

    private static Matrix4x4 Base(Matrix4x4[] transforms, int bone) =>
        bone >= 0 && bone < transforms.Length ? transforms[bone] : Matrix4x4.Identity;

    /// <summary>Puts a pose's turn or shift on a bone's transform, in the frame of the pose's base bone.</summary>
    private void Add(int bone, int poseId, Matrix4x4[] current, bool turn)
    {
        Matrix4x4 baseWorld = Base(current, Data.PoseDrivens[poseId].BaseBoneId);
        if (!Matrix4x4.Invert(baseWorld, out Matrix4x4 baseInverse)) return;

        Matrix4x4 local = current[bone] * baseInverse;
        if (!Matrix4x4.Decompose(local, out Vector3 scale, out Quaternion rotation, out Vector3 translation)) return;

        if (turn) rotation = Quaternion.Normalize(rotation * _poseTurn[poseId]);
        else translation += _poseShift[poseId];

        current[bone] = Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation)
                      * Matrix4x4.CreateTranslation(translation) * baseWorld;
    }

    private float Channel(ChannelBinding binding)
        => binding.IsBound ? (binding.OutputId >= 0 && binding.OutputId < _outputs.Length ? _outputs[binding.OutputId] : 0f) : binding.Constant;
}
