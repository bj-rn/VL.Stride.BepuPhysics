using System.ComponentModel;
using System.Globalization;
using System.Runtime.CompilerServices;
using Stride.Core.Mathematics;

namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Makes [DefaultValue(typeof(Vector3), "1.0, 1.0, 1.0")] work: DefaultValueAttribute resolves
/// values via TypeDescriptor, and Stride's math types have no working converter in a vvvv
/// session. (Stride.Core.Design's converters pass standalone tests but fail inside vvvv —
/// verified live — so we register our own minimal invariant-culture converters.)
/// Accepts both Stride's "X:1 Y:1 Z:1" format and comma-separated "1, 1, 1".
/// The TypeDescriptor.Refresh calls defeat descriptor caching: vvvv queries some of these
/// descriptors during startup and the cached converter-less descriptor would otherwise win
/// (observed for Quaternion).
/// </summary>
internal static class StrideTypeConverters
{
    private static bool _registered;

    [ModuleInitializer]
    internal static void Register()
    {
        if (_registered)
            return;
        _registered = true;

        TypeDescriptor.AddAttributes(typeof(Vector2), new TypeConverterAttribute(typeof(Vector2Converter)));
        TypeDescriptor.AddAttributes(typeof(Vector3), new TypeConverterAttribute(typeof(Vector3Converter)));
        TypeDescriptor.AddAttributes(typeof(Vector4), new TypeConverterAttribute(typeof(Vector4Converter)));
        TypeDescriptor.AddAttributes(typeof(Quaternion), new TypeConverterAttribute(typeof(QuaternionConverter)));

        TypeDescriptor.Refresh(typeof(Vector2));
        TypeDescriptor.Refresh(typeof(Vector3));
        TypeDescriptor.Refresh(typeof(Vector4));
        TypeDescriptor.Refresh(typeof(Quaternion));
    }

    private static float[] ParseComponents(string text, int expected)
    {
        // "1, 1, 1" (comma-separated) or Stride's ToString format "X:1 Y:1 Z:1" (space-separated).
        var parts = text.Contains(',')
            ? text.Split(',')
            : text.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != expected)
            throw new FormatException($"Expected {expected} components, got {parts.Length} in '{text}'.");

        var result = new float[expected];
        for (int i = 0; i < expected; i++)
        {
            var token = parts[i].Trim();
            var colon = token.IndexOf(':');
            if (colon >= 0)
                token = token[(colon + 1)..]; // strip "X:" style prefixes
            result[i] = float.Parse(token.TrimEnd('f', 'F'), CultureInfo.InvariantCulture);
        }
        return result;
    }

    private abstract class ComponentsConverter : TypeConverter
    {
        public override bool CanConvertFrom(ITypeDescriptorContext? context, Type sourceType)
            => sourceType == typeof(string) || base.CanConvertFrom(context, sourceType);

        public override object? ConvertFrom(ITypeDescriptorContext? context, CultureInfo? culture, object value)
            => value is string text ? FromComponents(text) : base.ConvertFrom(context, culture, value);

        protected abstract object FromComponents(string text);
    }

    private sealed class Vector2Converter : ComponentsConverter
    {
        protected override object FromComponents(string text)
        {
            var c = ParseComponents(text, 2);
            return new Vector2(c[0], c[1]);
        }
    }

    private sealed class Vector3Converter : ComponentsConverter
    {
        protected override object FromComponents(string text)
        {
            var c = ParseComponents(text, 3);
            return new Vector3(c[0], c[1], c[2]);
        }
    }

    private sealed class Vector4Converter : ComponentsConverter
    {
        protected override object FromComponents(string text)
        {
            var c = ParseComponents(text, 4);
            return new Vector4(c[0], c[1], c[2], c[3]);
        }
    }

    private sealed class QuaternionConverter : ComponentsConverter
    {
        protected override object FromComponents(string text)
        {
            var c = ParseComponents(text, 4);
            return new Quaternion(c[0], c[1], c[2], c[3]);
        }
    }
}
