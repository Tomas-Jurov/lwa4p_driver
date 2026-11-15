"""
Convert the generated URDF to USD once and cache the result.

The cache key covers the URDF, its meshes (the digest from make_isaac_urdf), the import
settings and the importer version, so a change in Tomas's model triggers a new import.
"""

import hashlib
import json
from pathlib import Path

IMPORT_FORMAT_VERSION = 'isaac-6.2-urdf-import-v1'
MANIFEST_NAME = 'asset.json'


def import_robot(robot_manifest: dict, import_config: dict, cache_dir: Path) -> Path:
    """Return the path of the robot USD, importing the URDF if it is not cached."""
    asset_dir = cache_dir / _cache_key(robot_manifest, import_config)
    asset_manifest = asset_dir / MANIFEST_NAME
    if asset_manifest.exists():
        usd_path = Path(json.loads(asset_manifest.read_text())['usd_path'])
        if usd_path.exists():
            return usd_path

    asset_dir.mkdir(parents=True, exist_ok=True)
    usd_path = _run_importer(robot_manifest['urdf'], import_config, asset_dir)
    asset_manifest.write_text(json.dumps({'usd_path': str(usd_path)}, indent=2) + '\n')
    return usd_path


def _cache_key(robot_manifest: dict, import_config: dict) -> str:
    digest = hashlib.sha256()
    digest.update(robot_manifest['digest'].encode())
    digest.update(json.dumps(import_config, sort_keys=True).encode())
    digest.update(IMPORT_FORMAT_VERSION.encode())
    return digest.hexdigest()[:16]


def _run_importer(urdf_path: str, import_config: dict, asset_dir: Path) -> Path:
    import omni.kit.app

    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
        'isaacsim.asset.importer.urdf', True)
    from isaacsim.asset.importer.urdf import URDFImporter
    from isaacsim.asset.importer.urdf import URDFImporterConfig

    # Drives are tuned on the live stage (simulation/robot_setup.py), so changing a
    # gain does not need a new import. Here they only need to be position drives.
    settings = URDFImporterConfig(
        urdf_path=urdf_path,
        usd_path=str(asset_dir),
        fix_base=True,
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_type=import_config['collision_type'],
        allow_self_collision=True,
        joint_drive_type='force',
        joint_target_type='position',
    )
    importer = URDFImporter()
    importer.config = settings
    usd_path = Path(importer.import_urdf()).resolve()
    if not usd_path.exists():
        raise RuntimeError(f'the URDF importer did not write {usd_path}')
    return usd_path
