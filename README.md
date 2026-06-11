# 3DGS + Point Cloud Detail Viewer

这个项目用于用一个高斯泼溅模型作为导航视图，并在用户点击对应位置后，在主视图中加载已经对齐好的高清点云块。

## 文件说明

- `app/index.html`：浏览器界面入口
- `app/main.js`：高斯导航、点云加载、位姿标记逻辑
- `app/styles.css`：界面样式
- `app/pose_assets.json`：从点云表面生成的位姿锚点和遮挡采样点
- `data/1.ply` / `data/2.ply`：已经对齐好的高清点云块（不入库，手动放入）
- `data/MIC_3DGS_aligned_preserved.ply`：保留 3DGS 属性并按点云坐标对齐后的高斯泼溅模型（不入库）
- `build_pose_assets.py`：从点云生成 `app/pose_assets.json`
- `repair_3dgs_alignment.py`：从 CloudCompare 对齐结果反推变换，修复生成可加载的对齐 3DGS
- `open3d_pose_cloud_viewer.py`：Open3D 版本的点云/位姿查看器实验脚本

## 运行 Web 查看器

在项目根目录启动本地服务器：

```powershell
python -m http.server 8000 --bind 127.0.0.1
```

然后打开：

```text
http://127.0.0.1:8000/app/
```

注意：前端通过 CDN 加载 Three.js 和 GaussianSplats3D，首次打开需要联网。

## 重新生成位姿锚点

如果更新了 `data/1.ply`、`data/2.ply` 或新增点云块，运行：

```powershell
python .\build_pose_assets.py
```

它会重新生成 `app/pose_assets.json`。

## 数据文件说明

`.ply` 点云 / 3DGS 文件体积较大（单文件常超过 100 MB），**不纳入 Git 版本管理**。

项目提供了 `data/` 占位目录，clone 后将以下文件放入 **`data/`** 目录即可：

| 文件名 | 说明 |
|---|---|
| `data/1.ply` | 对齐好的高清点云块 1 |
| `data/2.ply` | 对齐好的高清点云块 2 |
| `data/MIC_3DGS_aligned_preserved.ply` | 保留 3DGS 属性的对齐高斯泼溅模型（Web 查看器直接加载） |
| `data/MIC_3DGS0.ply` | 原始高斯泼溅模型（`repair_3dgs_alignment.py` 输入） |
| `data/MIC_3DGS.ply` | CloudCompare 对齐结果（`repair_3dgs_alignment.py` 输入） |

> 如有新的点云块，同样放入 `data/` 目录，并重新运行 `build_pose_assets.py` 生成位姿锚点。

## Clone 后恢复数据

```powershell
# clone 后将点云数据文件手动复制到 data/ 目录
# 然后根据需要重新生成位姿锚点
python .\build_pose_assets.py
```

