#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import queue
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d


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


@dataclass
class CloudEntry:
    cloud_id: str
    path: Path
    marker_position: np.ndarray
    center: np.ndarray


@dataclass
class LoadResult:
    token: int
    entry: CloudEntry
    points: np.ndarray | None
    colors: np.ndarray | None
    error: str | None


def parse_ply_header(path: Path) -> tuple[int, list[tuple[str, str]], int]:
    with path.open("rb") as handle:
        offset = 0
        vertex_count = 0
        properties: list[tuple[str, str]] = []
        data_format = ""
        in_vertex = False

        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"PLY header is incomplete: {path}")
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
                properties.append((parts[1], parts[2]))

    if data_format != "binary_little_endian":
        raise ValueError(f"Only binary_little_endian PLY is supported: {path}")
    if vertex_count <= 0:
        raise ValueError(f"PLY has no vertices: {path}")
    return vertex_count, properties, offset


def ply_dtype(properties: list[tuple[str, str]]) -> np.dtype:
    return np.dtype([(name, TYPE_MAP[prop_type]) for prop_type, name in properties])


def load_ply_numpy(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    vertex_count, properties, data_offset = parse_ply_header(path)
    dtype = ply_dtype(properties)
    names = [name for _, name in properties]
    if not all(name in names for name in ("x", "y", "z")):
        raise ValueError(f"PLY lacks x/y/z: {path}")

    data = np.memmap(path, dtype=dtype, mode="r", offset=data_offset, shape=(vertex_count,))
    points = np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)

    colors = None
    if all(name in names for name in ("red", "green", "blue")):
        colors = np.column_stack([data["red"], data["green"], data["blue"]]).astype(np.float64) / 255.0
    return points, colors


def voxel_downsample_numpy(
    points: np.ndarray,
    colors: np.ndarray | None,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray | None]:
    if voxel_size <= 0 or len(points) == 0:
        return points, colors

    voxels = np.floor(points / voxel_size).astype(np.int64)
    keys = voxels[:, 0] * 73856093 ^ voxels[:, 1] * 19349663 ^ voxels[:, 2] * 83492791
    _, indices = np.unique(keys, return_index=True)
    indices.sort()
    return points[indices], colors[indices] if colors is not None else None


def numpy_to_cloud(points: np.ndarray, colors: np.ndarray | None) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def make_camera_markers(entries: list[CloudEntry], selected_index: int, radius: float) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh()
    for index, entry in enumerate(entries):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=16)
        sphere.translate(entry.marker_position)
        sphere.paint_uniform_color([1.0, 0.58, 0.12] if index == selected_index else [0.1, 0.75, 1.0])
        mesh += sphere
    mesh.compute_vertex_normals()
    return mesh


def make_labels(entries: list[CloudEntry], selected_index: int) -> o3d.geometry.TriangleMesh:
    # Open3D's legacy visualizer has no true text overlay, so labels are simple vertical ticks.
    mesh = o3d.geometry.TriangleMesh()
    for index, entry in enumerate(entries):
        tick = o3d.geometry.TriangleMesh.create_cylinder(radius=0.012, height=0.22, resolution=10)
        tick.translate(entry.marker_position + np.array([0.0, 0.0, 0.13]))
        tick.paint_uniform_color([1.0, 0.74, 0.18] if index == selected_index else [0.25, 0.85, 1.0])
        mesh += tick
    mesh.compute_vertex_normals()
    return mesh


def pick_camera_index(
    vis: o3d.visualization.Visualizer,
    mouse_x: float,
    mouse_y: float,
    entries: list[CloudEntry],
    pick_radius_px: float = 42.0,
) -> int | None:
    params = vis.get_view_control().convert_to_pinhole_camera_parameters()
    extrinsic = params.extrinsic
    intrinsic = params.intrinsic.intrinsic_matrix
    best_index = None
    best_distance_sq = pick_radius_px * pick_radius_px

    for index, entry in enumerate(entries):
        point = np.array([*entry.marker_position, 1.0], dtype=np.float64)
        camera_point = extrinsic @ point
        if camera_point[2] <= 0:
            continue
        screen_u = intrinsic[0, 0] * camera_point[0] / camera_point[2] + intrinsic[0, 2]
        screen_v = intrinsic[1, 1] * camera_point[1] / camera_point[2] + intrinsic[1, 2]
        distance_sq = (screen_u - mouse_x) ** 2 + (screen_v - mouse_y) ** 2
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_index = index
    return best_index


def setup_scene_view(vis: o3d.visualization.Visualizer, points: np.ndarray) -> None:
    if len(points) == 0:
        return
    bounds_min = points.min(axis=0)
    bounds_max = points.max(axis=0)
    center = (bounds_min + bounds_max) * 0.5
    extent = float(np.linalg.norm(bounds_max - bounds_min))
    view = vis.get_view_control()
    view.set_lookat(center)
    view.set_front([0.4, -0.8, -0.45])
    view.set_up([0.0, 0.0, 1.0])
    view.set_zoom(0.75 if extent < 10 else 0.45)


def setup_focus_view(vis: o3d.visualization.Visualizer, points: np.ndarray) -> None:
    if len(points) == 0:
        return
    bounds_min = points.min(axis=0)
    bounds_max = points.max(axis=0)
    center = (bounds_min + bounds_max) * 0.5
    view = vis.get_view_control()
    view.set_lookat(center)
    view.set_front([0.35, -0.78, -0.52])
    view.set_up([0.0, 0.0, 1.0])
    view.set_zoom(0.8)


def read_pose_config(path: Path | None) -> dict[str, np.ndarray]:
    if path is None or not path.is_file():
        return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    poses: dict[str, np.ndarray] = {}
    for key, value in raw.items():
        arr = np.asarray(value, dtype=np.float64)
        if arr.shape == (4, 4):
            poses[str(key)] = arr[:3, 3]
        elif arr.shape == (3,):
            poses[str(key)] = arr
        else:
            raise ValueError(f"Pose for {key} must be [x,y,z] or 4x4 matrix")
    return poses


class PoseCloudViewer:
    def __init__(
        self,
        entries: list[CloudEntry],
        overview_points: np.ndarray,
        overview_colors: np.ndarray | None,
        selected_voxel: float,
        marker_radius: float,
    ) -> None:
        self.entries = entries
        self.overview_points = overview_points
        self.overview_colors = overview_colors
        self.selected_voxel = selected_voxel
        self.marker_radius = marker_radius

        self.selected_index = 0
        self._load_token = 0
        self._load_results: queue.Queue[LoadResult] = queue.Queue()
        self._cloud_cache: dict[str, tuple[np.ndarray, np.ndarray | None]] = {}
        self._mouse_x = 0.0
        self._mouse_y = 0.0
        self._mouse_down_x = 0.0
        self._mouse_down_y = 0.0
        self._drag_active = False
        self._drag_button = -1
        self._drag_mods = 0
        self._drag_last_x = 0.0
        self._drag_last_y = 0.0
        self._pending_index: int | None = None
        self._selected_added = False

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Open3D Pose Cloud Viewer", width=1280, height=900)
        self.vis.register_animation_callback(self._on_animation)
        self._register_mouse()
        self._register_keys()

        self.overview_cloud = numpy_to_cloud(overview_points, overview_colors)
        self.selected_cloud = o3d.geometry.PointCloud()
        self.markers = make_camera_markers(entries, self.selected_index, marker_radius)
        self.marker_ticks = make_labels(entries, self.selected_index)

        render = self.vis.get_render_option()
        render.background_color = np.array([0.06, 0.065, 0.075])
        render.point_size = 1.5

        self.vis.add_geometry(self.overview_cloud)
        self.vis.add_geometry(self.markers, reset_bounding_box=False)
        self.vis.add_geometry(self.marker_ticks, reset_bounding_box=False)
        setup_scene_view(self.vis, overview_points)
        self._select_index(0, focus=True)

    def _register_keys(self) -> None:
        self.vis.register_key_callback(ord("A"), lambda _vis: self._request(max(0, self.selected_index - 1)))
        self.vis.register_key_callback(ord("D"), lambda _vis: self._request(min(len(self.entries) - 1, self.selected_index + 1)))
        self.vis.register_key_callback(263, lambda _vis: self._request(max(0, self.selected_index - 1)))
        self.vis.register_key_callback(262, lambda _vis: self._request(min(len(self.entries) - 1, self.selected_index + 1)))
        self.vis.register_key_callback(ord("R"), self._reset_view)
        self.vis.register_key_callback(ord("F"), self._focus_selected)
        for index in range(min(9, len(self.entries))):
            self.vis.register_key_callback(ord(str(index + 1)), self._make_key_select(index))

    def _make_key_select(self, index: int):
        def callback(_vis: o3d.visualization.Visualizer) -> bool:
            self._request(index)
            return False

        return callback

    def _register_mouse(self) -> None:
        click_threshold_sq = 64.0

        def on_move(vis: o3d.visualization.Visualizer, x: float, y: float) -> None:
            if self._drag_active:
                dx = x - self._drag_last_x
                dy = y - self._drag_last_y
                view = vis.get_view_control()
                if self._drag_button == 0:
                    if self._drag_mods & (1 << 0):
                        view.translate(dx, dy, self._drag_last_x, self._drag_last_y)
                    else:
                        view.rotate(dx, dy, self._drag_last_x, self._drag_last_y)
                elif self._drag_button in (1, 2):
                    view.translate(dx, dy, self._drag_last_x, self._drag_last_y)
            self._mouse_x = x
            self._mouse_y = y
            self._drag_last_x = x
            self._drag_last_y = y

        def on_button(vis: o3d.visualization.Visualizer, button: int, action: int, mods: int) -> bool:
            if action == 1:
                self._drag_active = True
                self._drag_button = button
                self._drag_mods = mods
                self._mouse_down_x = self._mouse_x
                self._mouse_down_y = self._mouse_y
                self._drag_last_x = self._mouse_x
                self._drag_last_y = self._mouse_y
                return False

            if action == 0:
                dx = self._mouse_x - self._mouse_down_x
                dy = self._mouse_y - self._mouse_down_y
                if button == 0 and mods == 0 and dx * dx + dy * dy <= click_threshold_sq:
                    picked = pick_camera_index(vis, self._mouse_x, self._mouse_y, self.entries)
                    if picked is not None:
                        self._pending_index = picked
                self._drag_active = False
                self._drag_button = -1
                self._drag_mods = 0
            return False

        def on_scroll(vis: o3d.visualization.Visualizer, _x: float, y: float) -> bool:
            vis.get_view_control().scale(y * 0.18)
            return False

        self.vis.register_mouse_move_callback(on_move)
        self.vis.register_mouse_button_callback(on_button)
        self.vis.register_mouse_scroll_callback(on_scroll)

    def _request(self, index: int) -> bool:
        self._pending_index = index
        return False

    def _reset_view(self, _vis: o3d.visualization.Visualizer) -> bool:
        setup_scene_view(self.vis, self.overview_points)
        print("Reset to merged overview")
        return False

    def _focus_selected(self, _vis: o3d.visualization.Visualizer) -> bool:
        if len(self.selected_cloud.points) > 0:
            setup_focus_view(self.vis, np.asarray(self.selected_cloud.points))
        return False

    def _on_animation(self, _vis: o3d.visualization.Visualizer) -> bool:
        if self._pending_index is not None:
            index = self._pending_index
            self._pending_index = None
            self._select_index(index, focus=False)

        while True:
            try:
                result = self._load_results.get_nowait()
            except queue.Empty:
                break
            if result.token == self._load_token:
                self._apply_result(result)
        return False

    def _refresh_markers(self) -> None:
        self.vis.remove_geometry(self.markers, reset_bounding_box=False)
        self.vis.remove_geometry(self.marker_ticks, reset_bounding_box=False)
        self.markers = make_camera_markers(self.entries, self.selected_index, self.marker_radius)
        self.marker_ticks = make_labels(self.entries, self.selected_index)
        self.vis.add_geometry(self.markers, reset_bounding_box=False)
        self.vis.add_geometry(self.marker_ticks, reset_bounding_box=False)

    def _select_index(self, index: int, focus: bool) -> None:
        if index < 0 or index >= len(self.entries):
            return
        self.selected_index = index
        entry = self.entries[index]
        self._refresh_markers()
        print(f"Loading high-detail cloud: {entry.path.name}")
        self._load_token += 1
        token = self._load_token

        def worker() -> None:
            try:
                if entry.cloud_id in self._cloud_cache:
                    points, colors = self._cloud_cache[entry.cloud_id]
                else:
                    points, colors = load_ply_numpy(entry.path)
                    points, colors = voxel_downsample_numpy(points, colors, self.selected_voxel)
                    self._cloud_cache[entry.cloud_id] = (points, colors)
                self._load_results.put(LoadResult(token, entry, points, colors, None))
            except Exception as exc:  # noqa: BLE001
                self._load_results.put(LoadResult(token, entry, None, None, str(exc)))

        threading.Thread(target=worker, daemon=True).start()
        if focus:
            setup_focus_view(self.vis, np.asarray([entry.center]))

    def _apply_result(self, result: LoadResult) -> None:
        if result.error:
            print(f"Load failed: {result.error}")
            return
        assert result.points is not None
        self.selected_cloud.points = o3d.utility.Vector3dVector(result.points)
        if result.colors is not None:
            self.selected_cloud.colors = o3d.utility.Vector3dVector(result.colors)
        else:
            self.selected_cloud.colors = o3d.utility.Vector3dVector()
        if not self.selected_cloud.has_colors():
            self.selected_cloud.paint_uniform_color([1.0, 0.88, 0.42])

        if self._selected_added:
            self.vis.update_geometry(self.selected_cloud)
        else:
            self.vis.add_geometry(self.selected_cloud, reset_bounding_box=False)
            self._selected_added = True
        setup_focus_view(self.vis, result.points)
        print(f"Current: {result.entry.cloud_id}, displayed points: {len(result.points):,}")

    def run(self) -> None:
        print("Controls: left drag rotate | wheel zoom | Shift+left drag pan | click marker to load cloud")
        print("Keys: A/D or arrows switch | 1-9 select | R overview | F focus selected")
        self.vis.run()
        self.vis.destroy_window()


def discover_clouds(data_dir: Path, names: list[str]) -> list[Path]:
    if names:
        paths = [data_dir / name for name in names]
    else:
        paths = sorted(data_dir.glob("*.ply"), key=lambda p: p.stem)
    return [path for path in paths if path.is_file()]


def build_entries(
    cloud_paths: list[Path],
    pose_config: dict[str, np.ndarray],
    overview_voxel: float,
) -> tuple[list[CloudEntry], np.ndarray, np.ndarray | None]:
    entries: list[CloudEntry] = []
    overview_points_list: list[np.ndarray] = []
    overview_colors_list: list[np.ndarray] = []

    for path in cloud_paths:
        print(f"Preparing overview: {path.name}")
        points, colors = load_ply_numpy(path)
        center = points.mean(axis=0)
        overview_points, overview_colors = voxel_downsample_numpy(points, colors, overview_voxel)
        dim_color = overview_colors * 0.42 if overview_colors is not None else None
        if dim_color is None:
            dim_color = np.tile(np.array([[0.42, 0.46, 0.52]], dtype=np.float64), (len(overview_points), 1))

        marker_position = pose_config.get(path.stem)
        if marker_position is None:
            marker_position = pose_config.get(path.name)
        if marker_position is None:
            marker_position = center

        entries.append(CloudEntry(path.stem, path, marker_position, center))
        overview_points_list.append(overview_points)
        overview_colors_list.append(dim_color)
        print(f"  overview points: {len(overview_points):,}, marker: {marker_position}")

    return entries, np.vstack(overview_points_list), np.vstack(overview_colors_list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open3D viewer for aligned clouds with clickable pose markers.")
    parser.add_argument("--data-dir", type=Path, default=Path("data"), help="Folder containing PLY files.")
    parser.add_argument("--cloud", action="append", default=[], help="PLY filename to include. Repeatable.")
    parser.add_argument("--poses", type=Path, default=None, help="Optional JSON: cloud stem/name -> [x,y,z] or 4x4 pose.")
    parser.add_argument("--overview-voxel", type=float, default=0.02, help="Voxel size for merged overview cloud.")
    parser.add_argument("--selected-voxel", type=float, default=0.0, help="Voxel size for selected cloud; 0 keeps full detail.")
    parser.add_argument("--marker-radius", type=float, default=0.08, help="Camera marker sphere radius.")
    parser.add_argument("--dry-run", action="store_true", help="Prepare data and print stats without opening the viewer.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_dir = args.data_dir.resolve()
    cloud_paths = discover_clouds(data_dir, args.cloud or ["1.ply", "2.ply"])
    if not cloud_paths:
        print(f"No PLY files found in {data_dir}")
        return 1
    pose_config = read_pose_config(args.poses)
    entries, overview_points, overview_colors = build_entries(cloud_paths, pose_config, args.overview_voxel)
    print(f"Merged overview points: {len(overview_points):,}")
    if args.dry_run:
        for index, entry in enumerate(entries, start=1):
            print(f"{index}. {entry.path.name}: marker={entry.marker_position}, center={entry.center}")
        return 0
    app = PoseCloudViewer(entries, overview_points, overview_colors, args.selected_voxel, args.marker_radius)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
