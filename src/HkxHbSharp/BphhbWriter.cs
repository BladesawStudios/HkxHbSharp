using System.Numerics;
using AampSharp;

namespace HkxHbSharp;

/// <summary>Writes <see cref="HelperBoneData"/> as a <c>.bphhb</c>: the parameter archive <see cref="BphhbReader"/> reads, laid out as the game's own files are.</summary>
/// <remarks>
/// The lists sit under one container list, after an object that says how many of each there are. A channel of a pose-driven bone is written as <c>{name}_id</c> where an output drives it and as the bare
/// <c>{name}</c> where it is a fixed number, which is how the reader tells them apart. The position, rotate and aim lists the header counts are empty in every file seen and are written as such.
/// </remarks>
public static class BphhbWriter
{
    private const uint Header = 0x3a1f6f57, Container = 0x5b362b9b, BoneName = 0x5e237e06, HeaderScale = 0x1936f0df;

    public static ParameterIO Write(HelperBoneData data)
    {
        ParameterIO io = new();
        ParameterObject header = new();
        header.Set(HeaderScale, Parameter.FromFloat(0.1f));
        header.Set("bone_num", Parameter.FromInt(data.Bones.Count));
        header.Set("connection_curve_num", Parameter.FromInt(data.ConnectionCurves.Count));
        header.Set("output_num", Parameter.FromInt(data.Outputs.Count));
        header.Set("driver_bone_num", Parameter.FromInt(data.DriverBones.Count));
        header.Set("driven_bone_num", Parameter.FromInt(data.DrivenBones.Count));
        header.Set("pose_driven_num", Parameter.FromInt(data.PoseDrivens.Count));
        header.Set("position_driven_num", Parameter.FromInt(0));
        header.Set("rotate_driven_num", Parameter.FromInt(0));
        header.Set("aim_driven_num", Parameter.FromInt(0));
        io.Root.Objects.Add(new(Header, header));

        ParameterList container = new();
        container.Lists.Add(new(Crc32.Hash("bone_list"), Bones(data)));
        container.Lists.Add(new(Crc32.Hash("connection_curve_list"), List("connection_curve", data.ConnectionCurves, Curve)));
        container.Lists.Add(new(Crc32.Hash("output_list"), List("output", data.Outputs, Output)));
        container.Lists.Add(new(Crc32.Hash("driver_bone_list"), List("driver_bone", data.DriverBones, Driver)));
        container.Lists.Add(new(Crc32.Hash("driven_bone_list"), List("driven_bone", data.DrivenBones, Driven)));
        container.Lists.Add(new(Crc32.Hash("pose_driven_list"), List("pose_driven", data.PoseDrivens, Pose)));
        container.Lists.Add(new(Crc32.Hash("position_driven_list"), new ParameterList()));
        container.Lists.Add(new(Crc32.Hash("rotate_driven_list"), new ParameterList()));
        container.Lists.Add(new(Crc32.Hash("aim_driven_list"), new ParameterList()));
        io.Root.Lists.Add(new(Container, container));
        return io;
    }

    public static byte[] ToBytes(HelperBoneData data) => Write(data).ToBinary();

    private static ParameterList Bones(HelperBoneData data) => List("bone", data.Bones, (name, obj) =>
        obj.Set(BoneName, Parameter.FromString(ParameterType.StringRef, name)));

    private static ParameterList List<T>(string prefix, IReadOnlyList<T> items, Action<T, ParameterObject> fill)
    {
        ParameterList list = new();
        for (int i = 0; i < items.Count; i++)
        {
            ParameterObject obj = new();
            fill(items[i], obj);
            list.Objects.Add(new(Crc32.Hash($"{prefix}_{i}"), obj));
        }
        return list;
    }

    private static void Driver(DriverBone d, ParameterObject obj)
    {
        obj.Set("bone_id", Parameter.FromInt(d.BoneId));
        obj.Set("base_translate", Parameter.FromVector3(d.BaseTranslate));
        obj.Set("base_rotate", Parameter.FromQuaternion(d.BaseRotate));
        obj.Set("aim_axis", Parameter.FromVector3(d.AimAxis));
        obj.Set("up_axis", Parameter.FromVector3(d.UpAxis));
        obj.Set("base_bone_id", Parameter.FromInt(d.BaseBoneId));
    }

    private static void Curve(ConnectionCurve c, ParameterObject obj)
    {
        obj.Set("driver_bone_id", Parameter.FromInt(c.DriverBoneId));
        obj.Set("attr", Parameter.FromInt((int)c.Attr));
        obj.Set("key_num", Parameter.FromInt(c.Keys.Count));
        for (int k = 0; k < c.Keys.Count; k++)
            obj.Set($"key_{k}", Parameter.FromVector4(new Vector4(c.Keys[k].Time, c.Keys[k].Value, c.Keys[k].InSlope, c.Keys[k].OutSlope)));
    }

    private static void Output(Output o, ParameterObject obj)
    {
        for (int i = 0; i < o.ConnectionCurveIds.Count; i++) obj.Set($"connection_{i}_id", Parameter.FromInt(o.ConnectionCurveIds[i]));
    }

    private static void Driven(DrivenBone d, ParameterObject obj)
    {
        obj.Set("bone_id", Parameter.FromInt(d.BoneId));
        if (d.TranslateDrivenId >= 0)
        {
            obj.Set("translate_driven_type", Parameter.FromInt(d.TranslateDrivenType));
            obj.Set("translate_driven_id", Parameter.FromInt(d.TranslateDrivenId));
        }
        if (d.RotateDrivenId >= 0)
        {
            obj.Set("rotate_driven_type", Parameter.FromInt(d.RotateDrivenType));
            obj.Set("rotate_driven_id", Parameter.FromInt(d.RotateDrivenId));
        }
    }

    private static void Pose(PoseDriven p, ParameterObject obj)
    {
        obj.Set("base_translate", Parameter.FromVector3(p.BaseTranslate));
        obj.Set("base_rotate", Parameter.FromQuaternion(p.BaseRotate));
        obj.Set("aim_axis", Parameter.FromVector3(p.AimAxis));
        obj.Set("up_axis", Parameter.FromVector3(p.UpAxis));
        obj.Set("base_bone_id", Parameter.FromInt(p.BaseBoneId));
        Channel(obj, "roll", p.Roll);
        Channel(obj, "bendH", p.BendH);
        Channel(obj, "bendV", p.BendV);
        Channel(obj, "translateX", p.TranslateX);
        Channel(obj, "translateY", p.TranslateY);
        Channel(obj, "translateZ", p.TranslateZ);
    }

    private static void Channel(ParameterObject obj, string name, ChannelBinding channel)
    {
        if (channel.IsBound) obj.Set($"{name}_id", Parameter.FromInt(channel.OutputId));
        else obj.Set(name, Parameter.FromFloat(channel.Constant));
    }
}
