"""Re-analyse saved RoboMaster target camera samples without robot connection.

By default reads the latest target_raw_*.png from the camera sample folder.
Pass a path explicitly if needed. Saves a matching retuned debug PNG.
"""

import argparse
from pathlib import Path

import cv2

from classwork8.config import Classwork8Config
from classwork8.target_detection import TargetDetector


def main():
    parser = argparse.ArgumentParser(
        description="Offline target-image regression check (no RoboMaster needed)"
    )
    parser.add_argument("image", nargs="?", help="saved target_raw_*.png")
    args = parser.parse_args()

    config = Classwork8Config()
    if args.image:
        image_path = Path(args.image)
    else:
        folder = Path(config.output_dir) / "target_camera_samples"
        candidates = sorted(folder.glob("target_raw_*.png"))
        if not candidates:
            parser.error(
                "No target_raw_*.png found in {}. Supply an image path.".format(
                    folder
                )
            )
        image_path = candidates[-1]

    if not image_path.is_file():
        parser.error("Image not found: {}".format(image_path))

    frame = cv2.imread(str(image_path))
    if frame is None:
        parser.error("OpenCV could not read image: {}".format(image_path))

    detector = TargetDetector(config)
    detections, debug = detector.detect(frame)

    h, w = frame.shape[:2]
    top = int(h * config.target_roi_top_ratio)
    bottom = int(h * config.target_roi_bottom_ratio)
    print("Source: {}".format(image_path))
    print("Image: {}x{}".format(w, h))
    print("ROI: y={}..{} (top {:.2f}, bottom {:.2f})".format(
        top,
        bottom,
        config.target_roi_top_ratio,
        config.target_roi_bottom_ratio,
    ))
    print("DETECTIONS: {}".format(len(detections)))

    for index, item in enumerate(detections, 1):
        print(
            "#{:02d} {:<7} {:<9} confidence={:.2f} "
            "bbox={} aspect={:.3f} circularity={:.3f}".format(
                index,
                item.color.upper(),
                item.shape.upper(),
                item.confidence,
                item.bbox,
                item.aspect_ratio,
                item.circularity,
            )
        )

    output_path = image_path.with_name(
        "target_retuned_{}".format(image_path.name)
    )
    if not cv2.imwrite(str(output_path), debug):
        raise RuntimeError("Failed to save {}".format(output_path))

    print("Annotated result: {}".format(output_path))


if __name__ == "__main__":
    main()
