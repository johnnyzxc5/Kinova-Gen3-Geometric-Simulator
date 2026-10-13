"""Optional development-time retrieval of official Kinova visual STL meshes."""
from pathlib import Path
import hashlib
import json
import urllib.request
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
data = ET.parse(root / 'sources/kinova_gen3_6dof_novision.urdf').getroot()
revision = json.loads((root / 'sources/parameters.json').read_text())['urdf_revision']
manifest = []
for link in data.findall('link'):
    mesh = link.find('visual/geometry/mesh')
    if mesh is None:
        continue
    relative = mesh.get('filename')[len('package://'):]
    url = f'https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/{revision}/' + relative
    path = root / 'sources/meshes' / Path(relative).name
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        with urllib.request.urlopen(url, timeout=30) as response:
            path.write_bytes(response.read())
    manifest.append(dict(link=link.get('name'), path=path.relative_to(root).as_posix(),
                         url=url, revision=revision, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    print(link.get('name'), path.stat().st_size, flush=True)
(root / 'sources/mesh_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
