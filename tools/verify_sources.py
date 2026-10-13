"""Offline verification of preserved third-party copies and parameter inputs."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
checked = 0
for name in ['manifest.json', 'mesh_manifest.json']:
    manifest = json.loads((root / 'sources' / name).read_text())
    for item in manifest:
        path = root / item['path']
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != item['sha256']:
            raise SystemExit(f"Source hash mismatch: {item['path']}")
        checked += 1
print(f'PASS: {checked} preserved source records')
