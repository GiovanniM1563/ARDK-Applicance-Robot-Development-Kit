"""
Map Catalog - Named map storage with metadata.
Maps are stored in: {map_dir}/{name}/{name}.yaml, {name}.pgm, metadata.json
"""

import os
import re
import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Optional


def validate_name(name: str) -> bool:
    """Check if name is valid (alphanumeric + underscore, 1-50 chars)."""
    return bool(re.match(r'^[a-zA-Z0-9_]{1,50}$', name))


def get_map_dir(name: str, base_dir: str) -> Path:
    """Get the directory path for a named map."""
    return Path(base_dir) / name


def get_map_path(name: str, base_dir: str) -> Optional[str]:
    """Get the yaml path for a named map, or None if not found."""
    map_dir = get_map_dir(name, base_dir)
    yaml_path = map_dir / f"{name}.yaml"
    if yaml_path.exists():
        return str(yaml_path)
    return None


def list_maps(base_dir: str) -> list:
    """List all maps in the catalog with metadata."""
    base = Path(base_dir)
    if not base.exists():
        return []
    
    maps = []
    for entry in base.iterdir():
        if entry.is_dir():
            metadata_path = entry / "metadata.json"
            if metadata_path.exists():
                try:
                    with open(metadata_path) as f:
                        meta = json.load(f)
                    maps.append(meta)
                except:
                    # Fallback if metadata is corrupt
                    maps.append({"name": entry.name, "error": "metadata corrupt"})
            else:
                # Map exists but no metadata
                yaml_path = entry / f"{entry.name}.yaml"
                if yaml_path.exists():
                    maps.append({
                        "name": entry.name,
                        "created": None,
                        "size_bytes": yaml_path.stat().st_size
                    })
    return maps


def save_map_to_catalog(name: str, source_prefix: str, base_dir: str, overwrite: bool = False) -> dict:
    """
    Move a saved map into the catalog with metadata.
    source_prefix: path prefix used when saving (e.g., /tmp/my_map -> my_map.yaml, my_map.pgm)
    Returns metadata dict on success, raises on error.
    """
    if not validate_name(name):
        raise ValueError(f"Invalid map name: '{name}'. Use alphanumeric and underscore only.")
    
    map_dir = get_map_dir(name, base_dir)
    
    if map_dir.exists() and not overwrite:
        raise FileExistsError(f"Map '{name}' already exists. Set overwrite=true to replace.")
    
    # Source files
    src_yaml = f"{source_prefix}.yaml"
    src_pgm = f"{source_prefix}.pgm"
    
    if not os.path.exists(src_yaml):
        raise FileNotFoundError(f"Source map not found: {src_yaml}")
    
    # Create destination
    map_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy files
    dst_yaml = map_dir / f"{name}.yaml"
    dst_pgm = map_dir / f"{name}.pgm"
    
    shutil.copy2(src_yaml, dst_yaml)
    if os.path.exists(src_pgm):
        shutil.copy2(src_pgm, dst_pgm)
    
    # Update yaml to point to correct pgm filename
    _fix_yaml_image_path(dst_yaml, f"{name}.pgm")
    
    # Parse yaml for resolution/origin
    resolution, origin = _parse_yaml_metadata(dst_yaml)
    
    # Create metadata
    metadata = {
        "name": name,
        "created": datetime.now().isoformat(),
        "yaml_path": str(dst_yaml),
        "resolution": resolution,
        "origin": origin,
        "size_bytes": dst_yaml.stat().st_size
    }
    
    with open(map_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    
    return metadata


def _fix_yaml_image_path(yaml_path: Path, new_image: str):
    """Update the image field in the yaml to point to the new filename."""
    try:
        with open(yaml_path) as f:
            content = f.read()
        # Replace image: line
        content = re.sub(r'image:\s*\S+', f'image: {new_image}', content)
        with open(yaml_path, 'w') as f:
            f.write(content)
    except:
        pass  # Best effort


def _parse_yaml_metadata(yaml_path: Path) -> tuple:
    """Extract resolution and origin from yaml."""
    resolution = None
    origin = None
    try:
        with open(yaml_path) as f:
            for line in f:
                if line.startswith('resolution:'):
                    resolution = float(line.split(':')[1].strip())
                elif line.startswith('origin:'):
                    # origin: [x, y, z]
                    match = re.search(r'\[(.*)\]', line)
                    if match:
                        origin = [float(x.strip()) for x in match.group(1).split(',')]
    except:
        pass
    return resolution, origin
