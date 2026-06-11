from __future__ import annotations

import json
from pathlib import Path

import numpy as np


TYPE_MAP = {
    "char": "i1",
    "uchar": "u1",
    "short": "<i2",
    "ushort": "<u2",
    "int": "<i4",
    "uint": "<u4",
    "float": "<f4",
    "double": "<f8",
}


def parse_ply(path: Path) -> np.ndarray:
    with path.open("rb") as handle:
        offset = 0
        vertex_count = None
        data_format = ""
        props: list[tuple[str, str]] = []
        in_vertex = False
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Incomplete PLY header: {path}")
            offset += len(line)
            text = line.decode("ascii", errors="replace").strip()
            parts = text.split()
            if text == "end_header":
                break
            if len(parts) >= 2 and parts[0] == "format":
                data_format = parts[1]
            elif len(parts) >= 3 and parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    vertex_count = int(parts[2])
            elif in_vertex and len(parts) >= 3 and parts[0] == "property":
                props.append((parts[1], parts[2]))

    if data_format != "binary_little_endian":
        raise ValueError(f"Unsupported PLY format: {data_format}")
    if vertex_count is None:
        raise ValueError(f"No vertex element: {path}")

    dtype = np.dtype([(name, TYPE_MAP[prop_type]) for prop_type, name in props])
    data = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(vertex_count,))
    return np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)


def voxel_sample(points: np.ndarray, voxel_size: float, max_points: int) -> np.ndarray:
    voxels = np.floor(points / voxel_size).astype(np.int64)
    keys = voxels[:, 0] * 73856093 ^ voxels[:, 1] * 19349663 ^ voxels[:, 2] * 83492791
    _, indices = np.unique(keys, return_index=True)
    indices.sort()
    sampled = points[indices]
    if len(sampled) > max_points:
        step = int(np.ceil(len(sampled) / max_points))
        sampled = sampled[::step][:max_points]
    return sampled


def surface_anchor(points: np.ndarray, sampled: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centroid = points.mean(axis=0)
    distances = np.linalg.norm(sampled - centroid, axis=1)
    anchor = sampled[int(np.argmin(distances))]

    local_distances = np.linalg.norm(sampled - anchor, axis=1)
    nearest_count = min(3000, len(sampled))
    nearest = sampled[np.argpartition(local_distances, nearest_count - 1)[:nearest_count]]
    centered = nearest - nearest.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    normal /= max(np.linalg.norm(normal), 1e-8)
    return anchor, normal


def main() -> None:
    clouds = [Path("data/1.ply"), Path("data/2.ply")]
    poses = []
    occluders = []

    for index, path in enumerate(clouds, start=1):
        print(f"Reading {path}")
        points = parse_ply(path)
        sampled = voxel_sample(points, voxel_size=0.035, max_points=20000)
        anchor, normal = surface_anchor(points, sampled)
        stride = max(1, len(sampled) // 12000)
        occluders.extend(sampled[::stride].tolist())
        poses.append(
            {
                "id": f"pose-{index}",
                "cloud": index,
                "label": str(index),
                "name": f"位姿 {index:02d}",
                "position": [round(float(v), 6) for v in anchor],
                "normal": [round(float(v), 6) for v in normal],
            }
        )
        print(f"  anchor: {anchor}, normal: {normal}, sampled: {len(sampled)}")

    output = {
        "poses": poses,
        "occlusionPoints": [[round(float(v), 5) for v in point] for point in occluders],
    }
    output_path = Path("app") / "pose_assets.json"
    output_path.write_text(json.dumps(output, ensure_ascii=False), encoding="utf-8")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
