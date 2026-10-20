"""Extract a machine-readable specification from Simulink ``.mdl``/``.slx`` files.

An R2021b ``.mdl`` file is a "MathWorks OPC Text Package": the same XML
parts an ``.slx`` zip contains, concatenated as text and separated by
``__MWOPC_PART_BEGIN__ /<part>`` markers (binary parts are base64 and tagged
``BASE64``). This script reads either container directly (no MATLAB/Simulink
needed) and writes, per model:

* ``docs/spec/<model>.json``: every block with its full path, type, library
  source, explicitly stored parameters, named ports, the signal lines
  (with branches flattened), Stateflow/MATLAB Function charts with their
  instance paths and script hashes, solver settings, model arguments,
  every URI-like communication endpoint and every non-inherited sample time.
* ``docs/spec/<model>.md``: a human-readable digest of the same data.
* ``docs/spec/<model>/charts/<chart>.m``: the verbatim script of every
  MATLAB Function block, so the reference code can be diffed and executed
  in GNU Octave.

Parameters that are not stored in the block XML take the Simulink default
for that block type, which the archive records in ``simulink/bddefaults.xml``.
The JSON keeps the stored value and the default separately (``params`` vs
``defaults_applied``) so a reader can tell what the model states explicitly
and what is implied. Library blocks (QUARC ``Reference`` blocks) have no
defaults in the archive; any parameter absent from their XML is unknown here.

Usage::

    python tools/mdl_spec.py                 # the Phase 10 mission server + vehicle
    python tools/mdl_spec.py --check         # fail if committed outputs are stale
    python tools/mdl_spec.py path/to/model.mdl --out docs/spec
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Purely graphical parameters. They never affect behaviour, so they are
# dropped to keep the spec readable.
COSMETIC = {
    "Position", "ZOrder", "FontName", "FontSize", "FontWeight", "FontAngle",
    "BackgroundColor", "ForegroundColor", "ShowName", "HideAutomaticName",
    "BlockMirror", "BlockRotation", "NamePlacement", "NameLocation",
    "DropShadow", "IconDisplay", "Location", "Open", "ScrollbarOffset",
    "ZoomFactor", "ReportName", "SIDHighWatermark", "SIDPrevWatermark",
    "ContentPreviewEnabled", "RequestExecContextInheritance",
    "ScopeSpecificationString", "AttributesFormatString", "LibraryVersion",
    "RTWMemSecFuncInitTerm", "RTWMemSecFuncExecute", "RTWMemSecDataConstants",
    "RTWMemSecDataInternal", "RTWMemSecDataParameters", "MultithreadedSim",
    "IconShape", "DisplayOption", "VectorParamsAs1DForOutWhenUnconnected",
    "SystemRect", "SetExecutionDomain", "ExecutionDomainType",
    "SimulinkSubDomain", "Floating", "HasFrameUpgradeWarning",
}

# Defaults we fill in from bddefaults.xml, limited to parameters that change
# behaviour for the block types used in these models.
BEHAVIOURAL_DEFAULTS = {
    "Constant": ["Value", "SampleTime"],
    "Switch": ["Criteria", "Threshold"],
    "ManualSwitch": ["CurrentSetting"],
    "Gain": ["Gain"],
    "Saturate": ["UpperLimit", "LowerLimit"],
    "Bias": ["Bias"],
    "Sum": ["Inputs"],
    "Product": ["Inputs", "Multiplication"],
    "Logic": ["Operator", "Inputs"],
    "Mux": ["Inputs"],
    "Demux": ["Outputs"],
    "RateTransition": ["OutPortSampleTime", "InitialCondition"],
    "Memory": ["InitialCondition"],
    "UnitDelay": ["InitialCondition", "SampleTime"],
    "DeadZone": ["LowerValue", "UpperValue"],
    "RateLimiter": ["RisingSlewLimit", "FallingSlewLimit", "InitialCondition"],
    "Math": ["Operator"],
    "Inport": ["Port"],
    "Outport": ["Port", "InitialOutput"],
}

URI_RE = re.compile(r"\b(?:tcpip|udp|shmem|pipe|serial|file)://[^\s'\"]+")


DEFAULT_MODELS = (
    "simulink/Mission_Server_QDrone_YOLO_Autonomous_Land.mdl",
    "simulink/QDrone_YOLO_Direct_Stream_Commander.mdl",
)

PART_RE = re.compile(r"^__MWOPC_PART_BEGIN__ /(\S+)( BASE64)?\s*$", re.M)


class OpcText:
    """Read-only, zipfile-like view of a MathWorks OPC text package (.mdl)."""

    def __init__(self, path: Path):
        text = path.read_bytes().decode("utf-8")
        if "__MWOPC_PACKAGE_BEGIN__" not in text:
            raise ValueError(f"{path} is not an OPC text package (legacy .mdl is unsupported)")
        marks = list(PART_RE.finditer(text))
        self.parts: dict[str, bytes] = {}
        for i, m in enumerate(marks):
            end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
            body = text[m.end():end].lstrip("\r\n")
            if m.group(2):
                self.parts[m.group(1)] = base64.b64decode("".join(body.split()))
            else:
                self.parts[m.group(1)] = body.rstrip("\r\n").encode("utf-8")

    def read(self, name: str) -> bytes:
        return self.parts[name]

    def namelist(self) -> list[str]:
        return list(self.parts)


def open_container(path: Path):
    return zipfile.ZipFile(path) if path.suffix.lower() == ".slx" else OpcText(path)


def _p(elem: ET.Element) -> dict[str, str]:
    return {p.get("Name"): (p.text or "") for p in elem.findall("P")}


def _load_defaults(zf) -> dict[str, dict[str, str]]:
    out: dict[str, dict[str, str]] = {}
    try:
        tree = ET.fromstring(zf.read("simulink/bddefaults.xml"))
    except KeyError:
        return out
    for blk in tree.iter("Block"):
        bt = blk.get("BlockType")
        if bt and bt not in out:
            out[bt] = _p(blk)
    return out


class Model:
    def __init__(self, path: Path):
        self.path = path
        self.zf = open_container(path)
        self.defaults = _load_defaults(self.zf)
        self.blocks: list[dict] = []
        self.lines: list[dict] = []
        self._sid_path: dict[str, str] = {}

    # ----------------------------------------------------------- systems
    def _system(self, ref: str) -> ET.Element:
        return ET.fromstring(self.zf.read(f"simulink/systems/{ref}.xml"))

    def walk(self) -> None:
        self._walk_system("system_root", "")

    def _walk_system(self, ref: str, prefix: str) -> None:
        sysel = self._system(ref)
        local: dict[str, dict] = {}
        for blk in sysel.findall("Block"):
            name = blk.get("Name", "").replace("\n", " ")
            path = f"{prefix}/{name}" if prefix else name
            sid = blk.get("SID", "")
            bt = blk.get("BlockType", "")
            raw = _p(blk)
            raw.update(_p(blk.find("InstanceData")) if blk.find("InstanceData") is not None else {})
            params = {k: v for k, v in sorted(raw.items()) if k not in COSMETIC}
            applied = {}
            for key in BEHAVIOURAL_DEFAULTS.get(bt, []):
                if key not in params and key in self.defaults.get(bt, {}):
                    applied[key] = self.defaults[bt][key]
            ports = []
            for port in blk.findall("Port"):
                pp = _p(port)
                ports.append({k: pp[k] for k in ("PortNumber", "Name", "TestPoint") if k in pp})
            rec = {
                "path": path,
                "sid": sid,
                "type": bt,
                "params": params,
                "defaults_applied": applied,
                "named_ports": ports,
            }
            self.blocks.append(rec)
            local[sid] = rec
            self._sid_path[sid] = path
            sref = blk.find("System")
            if sref is not None and sref.get("Ref"):
                self._walk_system(sref.get("Ref"), path)
        for line in sysel.findall("Line"):
            lp = _p(line)
            src = lp.get("Src", "")
            for dst, name in self._dests(line, lp.get("Name", "")):
                self.lines.append({"system": prefix or "<root>", "src": src, "dst": dst, "name": name})

    def _dests(self, elem: ET.Element, name: str):
        p = _p(elem)
        if "Dst" in p:
            yield p["Dst"], name
        for br in elem.findall("Branch"):
            yield from self._dests(br, name)

    def resolve(self, endpoint: str) -> str:
        sid, _, port = endpoint.partition("#")
        return f"{self._sid_path.get(sid, sid)}#{port}"

    # ------------------------------------------------------------ charts
    def charts(self) -> list[dict]:
        out = []
        try:
            machine = ET.fromstring(self.zf.read("simulink/stateflow/machine.xml"))
        except KeyError:
            return out
        inst = {}
        for i in machine.iter("instance"):
            ip = _p(i)
            inst[ip.get("chart")] = ip.get("name", "").replace("//", "/").replace("\n", " ")
        for ch in machine.iter("chart"):
            ref = ch.get("Ref")
            if not ref:
                continue
            tree = ET.fromstring(self.zf.read(f"simulink/stateflow/{ref}.xml"))
            cid = tree.find("chart").get("id") if tree.find("chart") is not None else ref.split("_")[-1]
            scripts = [e.text for e in tree.iter() if e.attrib.get("Name") == "script"]
            data = []
            for d in tree.iter("data"):
                dp = _p(d)
                data.append({"name": d.get("name"), "scope": dp.get("scope"), "dataType": dp.get("dataType")})
            sig = re.search(r"function\s+(.*?)\s*=\s*(\w+)\s*\((.*?)\)", scripts[0] or "") if scripts else None
            out.append({
                "member": f"simulink/stateflow/{ref}.xml",
                "chart_id": cid,
                "instance_path": inst.get(cid, ""),
                "function": sig.group(2) if sig else None,
                "signature": sig.group(0) if sig else None,
                "data": data,
                "script": scripts[0] if scripts else None,
                "script_sha256": hashlib.sha256((scripts[0] or "").encode()).hexdigest() if scripts else None,
                "has_persistent_state": bool(scripts and re.search(r"^\s*persistent\b", scripts[0], re.M)),
            })
        return out

    # ------------------------------------------------------------ config
    def config(self) -> dict:
        out = {}
        try:
            info = ET.fromstring(self.zf.read("simulink/configSetInfo.xml"))
            act = [c for c in info.iter("ConfigSet") if c.get("Active") == "true"]
            active = ({"part": act[0].get("PartName").lstrip("/"), "name": act[0].text} if act else None)
        except KeyError:
            active = None
        names = sorted(n for n in self.zf.namelist() if re.match(r"simulink/configSet\d+\.xml", n))
        sets = {}
        for n in names:
            root = ET.fromstring(self.zf.read(n))
            wanted = {}
            for p in root.iter("P"):
                k = p.get("Name")
                if k in ("FixedStep", "StopTime", "SolverType", "Solver", "SolverName", "SystemTargetFile",
                         "ExtModeMexArgs", "StartTime", "EnableMultiTasking", "SampleTimeConstraint"):
                    wanted.setdefault(k, p.text)
            name = None
            for o in root.iter("Object"):
                op = _p(o)
                if "Name" in op and o.get("ClassName", "").endswith("ConfigSet"):
                    name = op["Name"]
                    break
            sets[n] = {"name": name, **wanted}
        out["config_sets"] = sets
        out["active_config_set"] = active
        return out


def model_arguments(cfg: dict) -> dict[str, str]:
    args = {}
    for s in cfg["config_sets"].values():
        mex = s.get("ExtModeMexArgs") or ""
        for m in re.finditer(r"-(URI_\w+)\s+(\S+?)'?(?=\s|$)", mex):
            args[m.group(1)] = m.group(2).rstrip("'")
    return args


def build_spec(path: Path) -> dict:
    m = Model(path)
    m.walk()
    cfg = m.config()
    margs = model_arguments(cfg)
    lines = [{"system": ln["system"], "src": m.resolve(ln["src"]), "dst": m.resolve(ln["dst"]),
              **({"name": ln["name"].replace("\n", " ")} if ln["name"] else {})} for ln in m.lines]
    endpoints, sample_times, constants = [], [], []
    for b in m.blocks:
        merged = {**b["defaults_applied"], **b["params"]}
        for k, v in b["params"].items():
            for uri in URI_RE.findall(v):
                ep = {"block": b["path"], "param": k, "uri": uri, "used": True}
                src = merged.get("uri_source")
                if k == "uri" and src == "External input port":
                    ep.update(used=False, note="uri_source = External input port; the dialog URI is ignored")
                elif k == "uri" and src == "Use external mode communications":
                    ep.update(used=False, note="uri_source = Use external mode communications; the dialog URI "
                                               "is ignored (connection rides on external mode)")
                endpoints.append(ep)
        if merged.get("argument_name"):
            endpoints.append({"block": b["path"], "param": "argument_name", "model_argument": merged["argument_name"],
                              "uri": margs.get(merged["argument_name"]), "used": True,
                              "source": "ExtModeMexArgs"})
        for k in ("sample_time", "SampleTime", "OutPortSampleTime"):
            v = merged.get(k)
            if v is not None and v not in ("-1", "inf"):
                sample_times.append({"block": b["path"], "param": k, "value": v})
        if b["type"] == "Constant":
            constants.append({"block": b["path"], "value": merged.get("Value"),
                              "explicit": "Value" in b["params"]})
    return {
        "model": path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "solver": cfg,
        "model_arguments": margs,
        "endpoints": endpoints,
        "sample_times": sample_times,
        "constants": constants,
        "charts": m.charts(),
        "blocks": m.blocks,
        "lines": lines,
    }


def to_markdown(spec: dict) -> str:
    out = [f"# Spec digest: `{Path(spec['model']).name}`", "",
           "Generated by `tools/mdl_spec.py` from the model file. Do not edit by hand.", "",
           f"- SHA-256: `{spec['sha256']}`"]
    for name, s in spec["solver"]["config_sets"].items():
        out.append(f"- {name} ({s.get('name')}): FixedStep={s.get('FixedStep')}, StopTime={s.get('StopTime')}, "
                   f"target={s.get('SystemTargetFile')}")
    if spec["model_arguments"]:
        out.append(f"- Model arguments (ExtModeMexArgs): {spec['model_arguments']}")
    out += ["", "## Communication endpoints", "",
            "| Block | Parameter | URI | Used by the block |", "|---|---|---|---|"]
    for e in spec["endpoints"]:
        used = "yes" if e["used"] else "no: " + e.get("note", "")
        out.append(f"| {e['block']} | {e['param']} | `{e.get('uri')}` | {used} |")
    out += ["", "## Stream / device blocks", ""]
    for b in spec["blocks"]:
        if b["type"] == "Reference" and any(s in b["params"].get("SourceBlock", "") for s in
                                            ("Stream", "OptiTrack", "Game Controller", "Host Initialize")):
            keep = {k: v for k, v in b["params"].items() if k not in ("SourceProductName", "SourceType")}
            out.append(f"- **{b['path']}**: `{json.dumps(keep, ensure_ascii=False)}`")
    out += ["", "## MATLAB Function charts", "",
            "| Chart | Instance | Signature | Persistent state |", "|---|---|---|---|"]
    for c in spec["charts"]:
        out.append(f"| {c['member']} | {c['instance_path']} | `{c['signature']}` | {c['has_persistent_state']} |")
    out += ["", "## Constants", "", "| Block | Value | Stored explicitly |", "|---|---|---|"]
    for c in spec["constants"]:
        out.append(f"| {c['block']} | `{c['value']}` | {'yes' if c['explicit'] else 'no (bddefaults)'} |")
    out += ["", "## Sample times", "", "| Block | Parameter | Value |", "|---|---|---|"]
    for s in spec["sample_times"]:
        out.append(f"| {s['block']} | {s['param']} | `{s['value']}` |")
    out += ["", "## Signal lines", "", "| System | Source | Destination | Name |", "|---|---|---|---|"]
    for ln in spec["lines"]:
        out.append(f"| {ln['system']} | {ln['src']} | {ln['dst']} | {ln.get('name', '')} |")
    return "\n".join(out).replace("\r", "") + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("models", nargs="*", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "spec")
    ap.add_argument("--check", action="store_true", help="exit 1 if committed outputs differ")
    args = ap.parse_args(argv)
    models = args.models or [ROOT / m for m in DEFAULT_MODELS]
    stale = []
    for path in models:
        spec = build_spec(path.resolve())
        outputs = {}
        for c in spec["charts"]:
            script = c.pop("script")
            if script is not None:
                label = re.sub(r"[^A-Za-z0-9]+", "_", c["instance_path"].split("/")[-1]).strip("_")
                fname = f"{path.stem}/charts/{Path(c['member']).stem}_{label}.m"
                c["script_file"] = fname
                outputs[fname] = script.replace("\r\n", "\n").rstrip("\n") + "\n"
        js = json.dumps(spec, indent=1, ensure_ascii=False, sort_keys=False) + "\n"
        md = to_markdown(spec)
        outputs[f"{path.stem}.json"] = js
        outputs[f"{path.stem}.md"] = md
        for rel, text in outputs.items():
            target = args.out / rel
            if args.check:
                if not target.exists() or target.read_text(encoding="utf-8") != text:
                    stale.append(target)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(text, encoding="utf-8")
    if args.check and stale:
        print("stale spec outputs:", *map(str, stale), sep="\n  ")
        return 1
    print(f"{'checked' if args.check else 'wrote'} specs for {len(models)} model(s) in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
