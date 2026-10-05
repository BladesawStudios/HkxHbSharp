using System.Numerics;

namespace HkxHbSharp;

/// <summary>
/// Evaluates a helper-bone rig against a pose: reads each driver bone's swing and twist against its base, runs them
/// through the curves, sums the curves into outputs, builds the pose-driven transforms from those, and writes them over
/// the driven bones.
/// </summary>
/// <remarks>
/// Ported from Marrow's solver and, like it, not checked against the game: the swing-twist convention, the order the
/// bend and roll rotations combine in, and whether translations add to or replace the authored one are all inferred
/// from the file's shape. Matrices are row-vector, <c>System.Numerics</c>'s way, so a child's world is local * parent.
/// </remarks>
public sealed class HelperBoneSolver
{
    private readonly float[] _roll;
    private readonly float[] _bendH;
    private readonly float[] _bendV;
    private readonly float[] _curves;
    private readonly float[] _outputs;
    private readonly Matrix4x4[] _poses;

    public HelperBoneData Data { get; }

    public HelperBoneSolver(HelperBoneData data)
    {
        Data = data ?? throw new ArgumentNullException(nameof(data));
        _roll = new float[data.DriverBones.Count];
        _bendH = new float[data.DriverBones.Count];
        _bendV = new float[data.DriverBones.Count];
        _curves = new float[data.ConnectionCurves.Count];
        _outputs = new float[data.Outputs.Count];
        _poses = new Matrix4x4[data.PoseDrivens.Count];
    }

    /// <summary>
    /// Solves the rig in place. <paramref name="transforms"/> holds the bones' world matrices numbered as
    /// <see cref="HelperBoneData.Bones"/> numbers them; the driven ones are overwritten.
    /// </summary>
    public void Solve(Matrix4x4[] transforms)
    {
        ArgumentNullException.ThrowIfNull(transforms);

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

        for (int p = 0; p < Data.PoseDrivens.Count; p++) _poses[p] = BuildPose(Data.PoseDrivens[p], transforms);

        foreach (DrivenBone driven in Data.DrivenBones) Drive(driven, transforms);
    }

    /// <summary>A driver's roll, horizontal bend and vertical bend, from its rotation relative to its base bone.</summary>
    private void ReadDriver(int d, Matrix4x4[] transforms)
    {
        DriverBone driver = Data.DriverBones[d];
        _roll[d] = _bendH[d] = _bendV[d] = 0f;

        if (driver.BoneId < 0 || driver.BoneId >= transforms.Length
            || driver.BaseBoneId < 0 || driver.BaseBoneId >= transforms.Length) return;
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

    private Matrix4x4 BuildPose(PoseDriven pose, Matrix4x4[] transforms)
    {
        Vector3 aim = Vector3.Normalize(pose.AimAxis);
        Vector3 up = Vector3.Normalize(pose.UpAxis);
        Vector3 side = Vector3.Normalize(Vector3.Cross(aim, up));

        Quaternion delta = Quaternion.CreateFromAxisAngle(up, Channel(pose.BendH))
                         * Quaternion.CreateFromAxisAngle(side, Channel(pose.BendV))
                         * Quaternion.CreateFromAxisAngle(aim, Channel(pose.Roll));
        Quaternion rotation = Quaternion.Normalize(pose.BaseRotate * delta);
        Vector3 translation = pose.BaseTranslate + new Vector3(Channel(pose.TranslateX), Channel(pose.TranslateY), Channel(pose.TranslateZ));

        Matrix4x4 local = Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation);
        Matrix4x4 parent = pose.BaseBoneId >= 0 && pose.BaseBoneId < transforms.Length ? transforms[pose.BaseBoneId] : Matrix4x4.Identity;
        return local * parent;
    }

    private void Drive(DrivenBone driven, Matrix4x4[] transforms)
    {
        if (driven.BoneId < 0 || driven.BoneId >= transforms.Length) return;

        bool rotates = driven.RotateDrivenType == 0 && driven.RotateDrivenId >= 0 && driven.RotateDrivenId < _poses.Length;
        bool translates = driven.TranslateDrivenType == 0 && driven.TranslateDrivenId >= 0 && driven.TranslateDrivenId < _poses.Length;
        if (!rotates && !translates) return;

        // One pose giving both: the bone takes it whole, scale and all.
        if (rotates && translates && driven.RotateDrivenId == driven.TranslateDrivenId)
        {
            transforms[driven.BoneId] = _poses[driven.RotateDrivenId];
            return;
        }

        // Otherwise only the channels named are replaced; the bone keeps its own scale and the rest.
        Matrix4x4.Decompose(transforms[driven.BoneId], out Vector3 scale, out Quaternion rotation, out Vector3 translation);
        if (rotates) Matrix4x4.Decompose(_poses[driven.RotateDrivenId], out _, out rotation, out _);
        if (translates) translation = _poses[driven.TranslateDrivenId].Translation;
        transforms[driven.BoneId] = Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation);
    }

    private float Channel(ChannelBinding binding)
        => binding.IsBound ? (binding.OutputId >= 0 && binding.OutputId < _outputs.Length ? _outputs[binding.OutputId] : 0f) : binding.Constant;
}
