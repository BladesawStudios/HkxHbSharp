using System.Numerics;
using AampSharp;

namespace HkxHbSharp;

/// <summary>Reads a <c>.bphhb</c> - an AAMP parameter archive - into <see cref="HelperBoneData"/>.</summary>
public static class BphhbReader
{
    // The lists sit under this list in the files seen, but some have them straight under the root.
    private const uint ContainerList = 0x5b362b9b;
    private const uint BoneNameHash = 0x5e237e06;

    public static HelperBoneData Read(ReadOnlySpan<byte> bytes) => Read(ParameterIO.FromBinary(bytes));

    public static HelperBoneData FromFile(string path) => Read(File.ReadAllBytes(path));

    public static HelperBoneData Read(ParameterIO io)
    {
        ParameterList root = io.Root;
        ParameterList container = root.List(ContainerList) ?? root;
        ParameterList? Find(string name) => container.List(name) ?? root.List(name);

        HelperBoneData data = new();

        if (Find("bone_list") is { } boneList)
            foreach (ParameterObject obj in Objects(boneList))
            {
                Parameter? name = obj["name"] ?? obj[BoneNameHash] ?? obj.Parameters.FirstOrDefault().Value;
                if (name is { IsString: true } && name.Text is { Length: > 0 } text) data.Bones.Add(text);
            }

        if (Find("driver_bone_list") is { } drivers)
            foreach (ParameterObject obj in Objects(drivers))
                data.DriverBones.Add(new DriverBone
                {
                    BoneId = Int(obj, "bone_id"),
                    BaseBoneId = Int(obj, "base_bone_id"),
                    BaseTranslate = Vec3(obj, "base_translate", Vector3.Zero),
                    BaseRotate = Quat(obj, "base_rotate"),
                    AimAxis = Vec3(obj, "aim_axis", Vector3.UnitX),
                    UpAxis = Vec3(obj, "up_axis", Vector3.UnitY),
                });

        if (Find("connection_curve_list") is { } curves)
            foreach (ParameterObject obj in Objects(curves))
            {
                ConnectionCurve curve = new() { DriverBoneId = Int(obj, "driver_bone_id"), Attr = (DriverAttribute)Int(obj, "attr") };
                int count = Int(obj, "key_num");
                for (int k = 0; k < count; k++)
                    if (obj[$"key_{k}"] is { } key && Four(key) is { } v)
                        curve.Keys.Add(new HermiteKey(v.X, v.Y, v.Z, v.W));
                data.ConnectionCurves.Add(curve);
            }

        if (Find("output_list") is { } outputs)
            foreach (ParameterObject obj in Objects(outputs))
            {
                Output output = new();
                for (int c = 0; c < 8; c++)
                    if (obj[$"connection_{c}_id"] is { } id && Number(id) is { } n) output.ConnectionCurveIds.Add((int)n);
                data.Outputs.Add(output);
            }

        if (Find("driven_bone_list") is { } driven)
            foreach (ParameterObject obj in Objects(driven))
                data.DrivenBones.Add(new DrivenBone
                {
                    BoneId = Int(obj, "bone_id"),
                    TranslateDrivenType = Int(obj, "translate_driven_type", -1),
                    TranslateDrivenId = Int(obj, "translate_driven_id", -1),
                    RotateDrivenType = Int(obj, "rotate_driven_type", -1),
                    RotateDrivenId = Int(obj, "rotate_driven_id", -1),
                });

        if (Find("pose_driven_list") is { } poses)
            foreach (ParameterObject obj in Objects(poses))
                data.PoseDrivens.Add(new PoseDriven
                {
                    BaseBoneId = Int(obj, "base_bone_id"),
                    BaseTranslate = Vec3(obj, "base_translate", Vector3.Zero),
                    BaseRotate = Quat(obj, "base_rotate"),
                    AimAxis = Vec3(obj, "aim_axis", Vector3.UnitX),
                    UpAxis = Vec3(obj, "up_axis", Vector3.UnitY),
                    Roll = Binding(obj, "roll"),
                    BendH = Binding(obj, "bendH"),
                    BendV = Binding(obj, "bendV"),
                    TranslateX = Binding(obj, "translateX"),
                    TranslateY = Binding(obj, "translateY"),
                    TranslateZ = Binding(obj, "translateZ"),
                });

        return data;
    }

    private static IEnumerable<ParameterObject> Objects(ParameterList list) => list.Objects.Select(o => o.Value);

    /// <summary>A channel names an output as <c>{name}_id</c>, or carries its value as <c>{name}</c>.</summary>
    private static ChannelBinding Binding(ParameterObject obj, string name)
    {
        if (obj[$"{name}_id"] is { } id && Number(id) is { } n) return ChannelBinding.FromOutput((int)n);
        if (obj[name] is { } value && Number(value) is { } f) return ChannelBinding.FromConstant(f);
        return ChannelBinding.FromConstant(0f);
    }

    // The archive's integers arrive as Int or U32 depending on the writer, so read either.
    private static float? Number(Parameter p) => p.Type switch
    {
        ParameterType.Int => p.AsInt(),
        ParameterType.U32 => p.AsUInt(),
        ParameterType.F32 => p.AsFloat(),
        ParameterType.Bool => p.AsBool() ? 1 : 0,
        _ => null,
    };

    private static int Int(ParameterObject obj, string name, int fallback = 0)
        => obj[name] is { } p && Number(p) is { } n ? (int)n : fallback;

    private static Vector3 Vec3(ParameterObject obj, string name, Vector3 fallback)
        => obj[name] is { Type: ParameterType.Vec3 } p ? p.AsVector3() : fallback;

    // A rotation is stored as a Vec4 or a Quat depending on the file.
    private static Vector4? Four(Parameter p) => p.Type switch
    {
        ParameterType.Vec4 => p.AsVector4(),
        ParameterType.Quat => new Vector4(p.AsQuaternion().X, p.AsQuaternion().Y, p.AsQuaternion().Z, p.AsQuaternion().W),
        _ => null,
    };

    private static Quaternion Quat(ParameterObject obj, string name)
        => obj[name] is { } p && Four(p) is { } v ? new Quaternion(v.X, v.Y, v.Z, v.W) : Quaternion.Identity;
}
