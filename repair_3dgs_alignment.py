from pathlib import Path
import math
import shutil

import numpy as np


SOURCE_GS = Path("data/MIC_3DGS0.ply")
ALIGNED_CC = Path("data/MIC_3DGS.ply")
OUTPUT_GS = Path("data/MIC_3DGS_aligned_preserved.ply")


TYPE_MAP = {
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "uchar": "u1",
    "uint8": "u1",
    "char": "i1",
    "short": "<i2",
    "ushort": "<u2",
    "int": "<i4",
    "uint": "<u4",
}


def parse_ply(path):
    data = path.read_bytes()
    marker = b"end_header\n"
    header_end = data.find(marker)
    if header_end < 0:
        raise ValueError(f"{path} has no PLY end_header marker")
    header_end += len(marker)
    header = data[:header_end].decode("ascii", errors="replace")

    props = []
    vertex_count = None
    in_vertex = False
    for line in header.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "element" and parts[1] == "vertex":
            vertex_count = int(parts[2])
            in_vertex = True
        elif len(parts) >= 2 and parts[0] == "element":
            in_vertex = False
        elif in_vertex and len(parts) >= 3 and parts[0] == "property":
            props.append((parts[1], parts[2]))

    if vertex_count is None:
        raise ValueError(f"{path} has no vertex element")

    dtype = np.dtype([(name, TYPE_MAP[prop_type]) for prop_type, name in props])
    vertices = np.frombuffer(data, dtype=dtype, count=vertex_count, offset=header_end)
    return data, header_end, header, props, vertices


def xyz(vertices):
    return np.column_stack([vertices["x"], vertices["y"], vertices["z"]]).astype(np.float64)


def estimate_similarity(source_xyz, target_xyz):
    source_mean = source_xyz.mean(axis=0)
    target_mean = target_xyz.mean(axis=0)
    source_centered = source_xyz - source_mean
    target_centered = target_xyz - target_mean

    covariance = (target_centered.T @ source_centered) / len(source_xyz)
    u, singular_values, vt = np.linalg.svd(covariance)
    d = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        d[-1, -1] = -1

    rotation = u @ d @ vt
    variance = (source_centered * source_centered).sum() / len(source_xyz)
    scale = (singular_values * np.diag(d)).sum() / variance
    translation = target_mean - scale * (rotation @ source_mean)
    return scale, rotation, translation


def rotation_matrix_to_quaternion(rotation):
    trace = np.trace(rotation)
    if trace > 0:
        root = math.sqrt(trace + 1.0) * 2.0
        return np.array([
            0.25 * root,
            (rotation[2, 1] - rotation[1, 2]) / root,
            (rotation[0, 2] - rotation[2, 0]) / root,
            (rotation[1, 0] - rotation[0, 1]) / root,
        ], dtype=np.float64)

    axis = int(np.argmax(np.diag(rotation)))
    if axis == 0:
        root = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return np.array([
            (rotation[2, 1] - rotation[1, 2]) / root,
            0.25 * root,
            (rotation[0, 1] + rotation[1, 0]) / root,
            (rotation[0, 2] + rotation[2, 0]) / root,
        ], dtype=np.float64)
    if axis == 1:
        root = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return np.array([
            (rotation[0, 2] - rotation[2, 0]) / root,
            (rotation[0, 1] + rotation[1, 0]) / root,
            0.25 * root,
            (rotation[1, 2] + rotation[2, 1]) / root,
        ], dtype=np.float64)

    root = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return np.array([
        (rotation[1, 0] - rotation[0, 1]) / root,
        (rotation[0, 2] + rotation[2, 0]) / root,
        (rotation[1, 2] + rotation[2, 1]) / root,
        0.25 * root,
    ], dtype=np.float64)


def multiply_quaternions(left, right):
    lw, lx, ly, lz = left.T
    rw, rx, ry, rz = right.T
    return np.column_stack([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ])


def main():
    source_data, source_header_end, source_header, source_props, source_vertices = parse_ply(SOURCE_GS)
    _, _, _, _, aligned_vertices = parse_ply(ALIGNED_CC)

    source_points = xyz(source_vertices)
    aligned_points = xyz(aligned_vertices)
    scale, rotation, translation = estimate_similarity(source_points, aligned_points)

    transformed = scale * (rotation @ source_points.T).T + translation
    residual = np.linalg.norm(transformed - aligned_points, axis=1)
    print(f"scale: {scale:.12f}")
    print("rotation:")
    print(rotation)
    print(f"translation: {translation}")
    print(f"rmse: {np.sqrt(np.mean(residual * residual)):.12g}")
    print(f"max error: {residual.max():.12g}")

    output = np.array(source_vertices, copy=True)
    output["x"] = transformed[:, 0].astype(np.float32)
    output["y"] = transformed[:, 1].astype(np.float32)
    output["z"] = transformed[:, 2].astype(np.float32)

    log_scale = math.log(scale)
    for name in ("scale_0", "scale_1", "scale_2"):
        if name in output.dtype.names:
            output[name] = (output[name].astype(np.float64) + log_scale).astype(np.float32)

    if all(name in output.dtype.names for name in ("rot_0", "rot_1", "rot_2", "rot_3")):
        transform_q = rotation_matrix_to_quaternion(rotation)
        source_q = np.column_stack([output[f"rot_{i}"] for i in range(4)]).astype(np.float64)
        transformed_q = multiply_quaternions(
            np.broadcast_to(transform_q, source_q.shape),
            source_q,
        )
        transformed_q /= np.linalg.norm(transformed_q, axis=1, keepdims=True)
        for i in range(4):
            output[f"rot_{i}"] = transformed_q[:, i].astype(np.float32)

    OUTPUT_GS.write_bytes(source_header.encode("ascii") + output.tobytes())
    print(f"wrote: {OUTPUT_GS}")


if __name__ == "__main__":
    main()
