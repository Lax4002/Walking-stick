"""
export_model.py — Convert a YOLO .pt model to ONNX + save class names.

▸ Run this on your WINDOWS machine (not the Pi).
▸ It produces two files that you copy to the Pi:
    1. yolo11_obstacles_best.onnx   — the model
    2. class_names.txt              — one class name per line

Usage:
    python export_model.py
    python export_model.py --model yolo11_obstacles_best.pt --size 320
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

DEFAULT_ONNX_SIZE = 320


def export_pt_to_onnx(model_source: str | Path, imgsz: int = DEFAULT_ONNX_SIZE, output_dir: Path | None = None) -> Path:
    """Convert a YOLO .pt model or remote alias into an ONNX file."""
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "ultralytics is required only when exporting a .pt model; "
            "use an existing .onnx model or install it with "
            "'pip install ultralytics'"
        ) from exc

    model_source = str(model_source)
    source_path = Path(model_source)
    if output_dir is None:
        output_dir = Path.cwd()
    output_dir.mkdir(parents=True, exist_ok=True)

    old_cwd = Path.cwd()
    os.chdir(output_dir)
    try:
        print(f"[INFO] Loading model from {model_source} …")
        model = YOLO(model_source)
        class_names = model.names

        names_file = source_path.with_name("class_names.txt") if source_path.exists() and source_path.suffix.lower() == ".pt" else output_dir / "class_names.txt"
        with open(names_file, "w") as f:
            for idx in sorted(class_names.keys()):
                f.write(class_names[idx] + "\n")
        print(f"[INFO] Saved {len(class_names)} class names → {names_file}")
        print(f"       Classes: {', '.join(class_names.values())}")

        print(f"[INFO] Exporting to ONNX (imgsz={imgsz}) …")
        onnx_path = Path(model.export(format="onnx", imgsz=imgsz))
    finally:
        os.chdir(old_cwd)

    print(f"\n[DONE] Export complete!")
    print(f"       ONNX model : {onnx_path}")
    print(f"       Class names: {names_file}")
    return onnx_path


def main() -> None:
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "ultralytics is required to run export_model.py; "
            "install it with 'pip install ultralytics'"
        ) from exc

    parser = argparse.ArgumentParser(description="Export YOLO .pt → .onnx for Raspberry Pi")
    parser.add_argument("--model", type=str, default="yolo11_obstacles_best.pt", help="Path to .pt model")
    parser.add_argument("--size", type=int, default=320, help="Input image size for export (default: 320)")
    args = parser.parse_args()

    pt_path = Path(args.model)
    if not pt_path.exists():
        raise FileNotFoundError(f"Model not found: {pt_path.resolve()}")

    # ── Load model ──────────────────────────────
    print(f"[INFO] Loading {pt_path.name} …")
    model = YOLO(str(pt_path))

    # ── Save class names ────────────────────────
    names_file = pt_path.with_name("class_names.txt")
    class_names = model.names  # dict {0: "name", 1: "name", …}
    with open(names_file, "w") as f:
        for idx in sorted(class_names.keys()):
            f.write(class_names[idx] + "\n")
    print(f"[INFO] Saved {len(class_names)} class names → {names_file.name}")
    print(f"       Classes: {', '.join(class_names.values())}")

    # ── Export to ONNX ──────────────────────────
    print(f"[INFO] Exporting to ONNX (imgsz={args.size}) …")
    onnx_path = model.export(format="onnx", imgsz=args.size)
    print(f"\n[DONE] Export complete!")
    print(f"       ONNX model : {onnx_path}")
    print(f"       Class names: {names_file}")
    print(f"\n       Copy BOTH files to your Raspberry Pi:")
    print(f"         scp {Path(onnx_path).name} {names_file.name} pi@<PI_IP>:~/walking_stick/")


if __name__ == "__main__":
    main()
