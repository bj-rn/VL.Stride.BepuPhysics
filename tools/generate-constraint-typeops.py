"""Generates src/Constraints/ConstraintTypeOperations.cs from the constraint node definitions.

Parses the seven *Constraints.cs node files and emits a GetXSettings / SetXSettings
operation pair for every constraint node, mirroring its Update pins: same types, same
defaults (including the [DefaultValue] axis attributes), same doc texts. The pin to
component property mapping is taken from the PinValue diff lines in the node bodies,
so the generated operations follow the nodes automatically.

Run after changing constraint node pins, then rebuild:
    python tools/generate-constraint-typeops.py
    dotnet build src/VL.Stride.BepuPhysics.csproj -c Release

Excluded pins: body references (read only by design, constraints are rewired via their
nodes), enabled (covered by SetConstraintEnabled) and reapplyInputs (a node concern).
"""
import re
import glob
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src" / "Constraints"
OUT = SRC / "ConstraintTypeOperations.cs"

EXCLUDED = {"bodyA", "bodyB", "bodyC", "bodyD", "body", "enabled", "reapplyInputs"}

def parse_classes(text):
    """Yield (node name, component type, pins, pin docs, pin to property map) per node class."""
    classes = []
    for m in re.finditer(
        r'\[ProcessNode\(Name = "(\w+)"\)\]\s*\npublic class \w+\n\{(.*?)\n\}', text, re.DOTALL):
        node_name, body = m.group(1), m.group(2)
        comp = re.search(r"private readonly SConstraints\.(\w+) _c = new\(\);", body).group(1)
        docs = dict(re.findall(r'/// <param name="(\w+)">(.*?)</param>', body))
        sig = re.search(r"public SConstraints\.\w+ Update\((.*?)\)\s*\n    \{", body, re.DOTALL).group(1)
        params = []
        for line in sig.splitlines():
            raw = line.strip().rstrip(",)").strip()
            # out parameters (attached) are node outputs, not settings
            if not raw or raw.startswith("out "):
                continue
            attr = None
            am = re.match(r"\[(.*?)\]\s*(.*)", raw)
            if am:
                attr, raw = am.group(1), am.group(2)
            dm = re.match(r"([\w\.\?<>]+)\s+(\w+)(?:\s*=\s*(.+))?$", raw)
            if not dm:
                continue
            ptype, pname, pdefault = dm.group(1), dm.group(2), dm.group(3)
            if pname in EXCLUDED:
                continue
            params.append((ptype, pname, pdefault, attr))
        # if (_pin.Changed(pin) | reapplyInputs) _c.Property = pin;  ->  pin: Property
        props = dict(re.findall(r"if \(_(\w+)\.Changed\(\w+\) \| reapplyInputs\) _c\.(\w+) = \w+;", body))
        classes.append((node_name, comp, params, docs, props))
    return classes

def vec3_expr(attr):
    """[DefaultValue("0.0, 1.0, 0.0")] as a C# construction expression for getter fallbacks."""
    vals = re.search(r'DefaultValue\("([^"]+)"\)', attr).group(1)
    comps = [v.strip() for v in vals.split(",")]
    return f"new Vector3({comps[0]}f, {comps[1]}f, {comps[2]}f)"

def getter_fallback(ptype, pdefault, attr):
    """Expression a getter outputs while the constraint input is null."""
    if ptype == "Quaternion":
        # quaternion pins have no C# default, VL supplies identity, so the getter does too
        return "Quaternion.Identity"
    if attr and "DefaultValue" in attr and ptype == "Vector3":
        return vec3_expr(attr)
    if pdefault:
        return pdefault
    return "default"

DEFAULT_VALUE_RE = re.compile(r'(DefaultValue\("[^"]+"\))')

def setter_param(ptype, pname, pdefault, attr):
    """One setter parameter line, keeping the node's default (attribute or inline)."""
    prefix = ""
    if attr and "DefaultValue" in attr and ptype == "Vector3":
        prefix = "[" + DEFAULT_VALUE_RE.search(attr).group(1) + "] "
    default = f" = {pdefault}" if pdefault else ""
    if ptype == "Quaternion":
        # no expressible C# default, VL supplies identity
        default = ""
    return f"        {prefix}{ptype} {pname}{default}"

sections = []
all_text = ""
for path in sorted(glob.glob(str(SRC / "*Constraints.cs"))):
    all_text += open(path, encoding="utf-8").read() + "\n"

for node_name, comp, params, docs, props in parse_classes(all_text):
    a_or_an = "an" if node_name[0] in "AEIOU" else "a"

    g = []
    g.append(f"    /// <summary>Reads all settings of {a_or_an} {node_name} constraint, mirroring the {node_name} node's inputs.</summary>")
    g.append(f'    /// <param name="constraint">The {node_name} constraint to read. Outputs the defaults while null. Use CastAs ({comp}) to narrow a ConstraintComponentBase.</param>')
    for ptype, pname, pdefault, attr in params:
        g.append(f'    /// <param name="{pname}">{docs[pname]}</param>')
    g.append(f"    public static void {node_name}Settings(SConstraints.{comp}? constraint,")
    outs = [f"        out {ptype} {pname}" for ptype, pname, pdefault, attr in params]
    g.append(",\n".join(outs) + ")")
    g.append("    {")
    for ptype, pname, pdefault, attr in params:
        g.append(f"        {pname} = constraint?.{props[pname]} ?? {getter_fallback(ptype, pdefault, attr)};")
    g.append("    }")

    s = []
    s.append("    /// <summary>")
    s.append(f"    /// Writes all settings of {a_or_an} {node_name} constraint while Apply is true, mirroring the {node_name} node's inputs.")
    s.append("    /// A written property is overwritten again once the owning constraint node's pin value changes.")
    s.append("    /// </summary>")
    s.append(f'    /// <param name="constraint">The {node_name} constraint to write to. Use CastAs ({comp}) to narrow a ConstraintComponentBase.</param>')
    for ptype, pname, pdefault, attr in params:
        s.append(f'    /// <param name="{pname}">{docs[pname]}</param>')
    s.append('    /// <param name="apply">Writes all settings each frame while true. Connect a Bang for a one-shot write.</param>')
    s.append('    [return: Pin(Name = "Output")]')
    s.append(f"    public static SConstraints.{comp}? Set{node_name}Settings(SConstraints.{comp}? constraint,")
    ins = [setter_param(ptype, pname, pdefault, attr) for ptype, pname, pdefault, attr in params]
    ins.append("        bool apply = false")
    s.append(",\n".join(ins) + ")")
    s.append("    {")
    s.append("        if (apply && constraint is not null)")
    s.append("        {")
    for ptype, pname, pdefault, attr in params:
        s.append(f"            constraint.{props[pname]} = {pname};")
    s.append("        }")
    s.append("        return constraint;")
    s.append("    }")

    sections.append("\n".join(g) + "\n\n" + "\n".join(s))

header = '''using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using SConstraints = global::Stride.BepuPhysics.Constraints;

namespace VL.Stride.BepuPhysics.Constraints;

// Type specific read and write access to constraints, mirroring each constraint node's
// inputs. Part of the Operations class (see ConstraintOperations.cs for the summary).
// Use CastAs to narrow a ConstraintComponentBase from a GetConstraints node, routed by
// ConstraintInfo's Kind. Body references stay read only and Enabled is covered by
// SetConstraintEnabled.
// Generated by tools/generate-constraint-typeops.py from the constraint node definitions.
// Do not edit by hand, rerun the script when node pins change.
public static partial class Operations
{
'''

out = header + "\n\n".join(sections) + "\n}\n"
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(out)
op_count = out.count("public static") - 1  # minus the class declaration itself
print(f"written {OUT}: {op_count} operations")
