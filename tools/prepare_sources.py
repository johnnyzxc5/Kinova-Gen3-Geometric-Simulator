"""Rebuild the pinned parameter file and vendor the author's existing kernels.

This is a development utility. MATLAB users do not need Python or network access.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--shape-source', type=Path, help='Optional SHAPE repository for re-vendoring kernels')
parser.add_argument('--flatness-source', type=Path, help='Optional Flatness-MPC benchmark_v5 directory')
args = parser.parse_args()
SHAPE, FLATNESS = args.shape_source, args.flatness_source
COMMIT = "5f66bbd1be84cb699020621dc7c8f3df01394960"
BASE = f"https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/{COMMIT}/"
URDF_PATH = "kortex_description/arms/gen3/6dof/urdf/GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf"
manifest_path = ROOT / 'sources/manifest.json'
manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else []


def record(target, source, **extra):
    manifest[:] = [item for item in manifest if item['path'] != target.relative_to(ROOT).as_posix()]
    manifest.append(dict(path=target.relative_to(ROOT).as_posix(), source=source,
                         sha256=hashlib.sha256(target.read_bytes()).hexdigest(), **extra))


def copy(source, target, **extra):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    record(target, str(source), **extra)


urdf = ROOT / "sources" / "kinova_gen3_6dof_novision.urdf"
if not urdf.exists():
    with urllib.request.urlopen(BASE + URDF_PATH, timeout=30) as response:
        urdf.write_bytes(response.read())
record(urdf, BASE + URDF_PATH, revision=COMMIT)
record(ROOT / "sources" / "kinova_license.txt", BASE + "LICENSE", revision=COMMIT)
xml = ET.parse(urdf).getroot()
links = {e.get("name"): e for e in xml.findall("link")}


def vector(element, attribute):
    return [float(v) for v in element.get(attribute).split()]


joints = []
for joint in xml.findall("joint"):
    origin = joint.find("origin")
    entry = dict(name=joint.get("name"), type=joint.get("type"),
                 parent=joint.find("parent").get("link"),
                 child=joint.find("child").get("link"),
                 xyz=vector(origin, "xyz"), rpy=vector(origin, "rpy"),
                 axis=vector(joint.find("axis"), "xyz"))
    if joint.get("type") != "fixed":
        limit = joint.find("limit")
        inertial = links[entry["child"]].find("inertial")
        tensor = inertial.find("inertia")
        entry.update(mass=float(inertial.find("mass").get("value")),
                     com=vector(inertial.find("origin"), "xyz"),
                     inertia_rpy=vector(inertial.find("origin"), "rpy"),
                     inertia=[float(tensor.get(a)) for a in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]],
                     effort=float(limit.get("effort")), velocity=float(limit.get("velocity")),
                     lower=float(limit.get("lower", "-inf")), upper=float(limit.get("upper", "inf")))
        # JSON has no portable infinities; null identifies continuous axes.
        if entry["type"] == "continuous":
            entry["lower"] = entry["upper"] = None
    joints.append(entry)
params = dict(schema="kinova-gen3-6dof-v1", robot=xml.get("name"),
              urdf_revision=COMMIT, joints=joints,
              actuator_proxy=dict(source="Cholet 2025, Tables A.1/B.1; commercial substitutes, not identified Kinova actuators",
                                  motor_inertia=[9.3e-6]*3+[5.6e-6]*3,
                                  reducer_output_inertia=[7.9e-6]*3+[3.3e-6]*3,
                                  reduction=[100]*6),
              friction=dict(source="Muraccioli et al., RSS 2025, Sec. VII-A, eq. (12)",
                            viscous=[2.0]*3+[1.36]*3, coulomb=[2.15]*3+[1.6]*3))
(ROOT / "sources" / "parameters.json").write_text(json.dumps(params, indent=2, allow_nan=False)+"\n")
record(ROOT / "sources" / "parameters.json", "Derived from the bundled URDF and cited parameter tables")

mr_files = ["ad", "Adjoint", "AxisAng3", "InverseDynamics", "MassMatrix", "MatrixExp3", "MatrixExp6",
            "MatrixLog3", "MatrixLog6", "NearZero", "TransInv", "VecTose3", "VecToso3", "so3ToVec",
            "se3ToVec", "TransToRp", "JacobianSpace", "JacobianBody"]
if SHAPE:
    for name in mr_files:
        copy(SHAPE / "Code_Image" / "MATLAB" / "mr" / (name+".m"),
             ROOT / "third_party" / "modern_robotics" / (name+".m"),
             license="MIT, NxRLab; preserved local SHAPE copy")
    copy(SHAPE / "third_party" / "ModernRobotics" / "LICENSE", ROOT / "third_party" / "modern_robotics" / "LICENSE")
    for name in ["rnea_shared_kinematics_MR", "InverseDynamicsSharedKinematics_MR", "MassMatrixZeroStateSharedKinematics_MR"]:
        copy(SHAPE / "Code_Image" / (name+".m"), ROOT / "third_party" / "shape_kernels" / (name+".m"),
             note="Copied byte-for-byte; validated on corrected relative transforms in this repository")
if FLATNESS:
    for name in ["phase", "phase_plan"]:
        copy(FLATNESS / "+ftv5" / (name+".m"), ROOT / "third_party" / "flatness" / "+ftv5" / (name+".m"))

(ROOT / "sources" / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
print(f"Prepared {len(manifest)} source records at {ROOT}")
