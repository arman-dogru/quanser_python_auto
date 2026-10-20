"""Check byte-for-byte source preservation and embedded script extraction."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / "docs/source_manifest.json").read_text())
for item in manifest["copied_files"]:
    path = root / item["repository_path"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"], path
for item in manifest["extracted_scripts"]:
    with zipfile.ZipFile(root / item["model"]) as archive:
        tree = ET.fromstring(archive.read(item["member"]))
    scripts = [e.text for e in tree.iter() if e.attrib.get("Name") == "script"]
    assert (root / item["path"]).read_text(encoding="utf-8") in scripts, item["path"]
print(f"Verified {len(manifest['copied_files'])} source files and {len(manifest['extracted_scripts'])} embedded functions.")
