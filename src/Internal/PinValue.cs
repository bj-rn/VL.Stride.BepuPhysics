namespace VL.Stride.BepuPhysics.Internal;

/// <summary>
/// Change detection against the last pin value instead of the component state, so values
/// written to the component from elsewhere (setter nodes operating on query/contact results)
/// are not reverted while the pin is unchanged. The pin wins again once its value changes.
/// The first call always reports a change so pin defaults initialize the component.
/// </summary>
internal struct PinValue<T>
{
    private T? _last;
    private bool _hasValue;

    public bool Changed(T value)
    {
        if (_hasValue && EqualityComparer<T>.Default.Equals(_last!, value))
            return false;
        _last = value;
        _hasValue = true;
        return true;
    }
}
