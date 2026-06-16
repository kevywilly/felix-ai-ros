"""Build a TensorRT FP16 engine from the committed YOLO .pt, on-device.

A TensorRT engine is serialized against the exact GPU + TRT + CUDA tuple, so it
is NOT portable and must be built where it runs. Run once after install (and
again with --force after a JetPack/TensorRT upgrade):

    ros2 run felix_perception build-engine
    ros2 run felix_perception build-engine --weights yolo11n.pt --force

Writes <stem>.engine into the cache dir ($FELIX_ENGINE_DIR or ~/.cache/felix),
which the detector loads. The engine is gitignored, never committed. INT8 is
deliberately avoided (broken on TRT 10.x with Ultralytics); FP16 only.

This mirrors felix_base's `calibrate` console-script setup-task idiom: a
deliberate one-shot with visible logging, not a surprise build during launch.
"""
import argparse
import os
import shutil

from felix_perception.geometry import engine_cache_path


def _resolve_weights(weights: str) -> str:
    """A bare model name resolves to the shipped copy in the package share dir
    if present; otherwise it is handed to Ultralytics, which downloads it."""
    if os.path.sep in weights or os.path.exists(weights):
        return weights
    try:
        from ament_index_python.packages import get_package_share_directory
        shared = os.path.join(
            get_package_share_directory("felix_perception"), "models", weights)
        if os.path.exists(shared):
            return shared
    except Exception:
        pass
    return weights


def main():
    ap = argparse.ArgumentParser(
        description="Build the felix_perception TensorRT FP16 engine on-device.")
    ap.add_argument("--weights", default="yolo11n.pt",
                    help="weights name or path (default yolo11n.pt)")
    ap.add_argument("--out", default=None,
                    help="engine output dir (default $FELIX_ENGINE_DIR or "
                         "~/.cache/felix)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--force", action="store_true",
                    help="rebuild even if the engine already exists")
    args = ap.parse_args()

    out_path = engine_cache_path(args.weights, args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    if os.path.exists(out_path) and not args.force:
        print(f"[build-engine] {out_path} already exists; --force to rebuild.")
        return

    # Log the toolchain so a later deserialize failure is diagnosable.
    try:
        import tensorrt
        print(f"[build-engine] tensorrt {tensorrt.__version__}")
    except Exception as exc:  # pragma: no cover - device-only
        print(f"[build-engine] WARNING: tensorrt import failed: {exc}")

    from ultralytics import YOLO

    weights = _resolve_weights(args.weights)
    print(f"[build-engine] exporting {weights} -> FP16 engine "
          f"(imgsz={args.imgsz}, batch=1, device=0)...")
    model = YOLO(weights)
    produced = str(model.export(
        format="engine", half=True, imgsz=args.imgsz, batch=1, device=0))

    # Ultralytics writes the engine next to the weights; move it to the cache.
    if os.path.abspath(produced) != os.path.abspath(out_path):
        shutil.move(produced, out_path)
    print(f"[build-engine] wrote {out_path}")


if __name__ == "__main__":
    main()
