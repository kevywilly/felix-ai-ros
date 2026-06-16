# Models

The YOLO weights (`yolo11n.pt`, stock COCO) live here and are shipped to the
package share dir via `setup.py` `data_files`.

- **`yolo11n.pt`** — commit the small (~5 MB) PyTorch weights here. If absent,
  Ultralytics fetches `yolo11n.pt` on first use (the `detector` node and the
  `build-engine` script both pass the weights path/name straight to `YOLO(...)`,
  which auto-downloads a bare model name).
- **`*.engine`** — the TensorRT FP16 engine is **not** stored here. It is
  device/TensorRT-version specific, so it is built on-device by
  `ros2 run felix_perception build-engine` into a writable cache
  (`~/.cache/felix/` by default, or `$FELIX_ENGINE_DIR`) and is gitignored.
  Re-run `build-engine --force` after a JetPack/TensorRT upgrade.
