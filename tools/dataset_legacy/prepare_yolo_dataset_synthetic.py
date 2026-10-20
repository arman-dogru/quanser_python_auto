import os
import csv
import random
import shutil
from pathlib import Path
from PIL import Image


# ============================================================
# CONFIG
# ============================================================

BASE_DIR = r"E:\berkay\Quanser\images"

SYN_POS_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "positive")
SYN_NEG_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "negative")
SYN_METADATA = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "positive_metadata.csv")

YOLO_DIR = os.path.join(BASE_DIR, "dataset_yolo")

CLASS_ID = 0
CLASS_NAME = "landing_pad"

TRAIN_RATIO = 0.8
RANDOM_SEED = 42

CLEAR_OLD_DATASET = True

random.seed(RANDOM_SEED)


# ============================================================
# HELPERS
# ============================================================

def ensure_dirs():
    if CLEAR_OLD_DATASET and os.path.isdir(YOLO_DIR):
        shutil.rmtree(YOLO_DIR)

    for split in ["train", "val"]:
        Path(os.path.join(YOLO_DIR, "images", split)).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(YOLO_DIR, "labels", split)).mkdir(parents=True, exist_ok=True)


def yolo_bbox_from_xyxy(x1, y1, x2, y2, img_w, img_h):
    """
    Convert pixel bbox [x1,y1,x2,y2] to YOLO normalized:
    class x_center y_center width height
    """
    x1 = max(0, min(float(x1), img_w))
    y1 = max(0, min(float(y1), img_h))
    x2 = max(0, min(float(x2), img_w))
    y2 = max(0, min(float(y2), img_h))

    bbox_w = x2 - x1
    bbox_h = y2 - y1

    x_center = x1 + bbox_w / 2
    y_center = y1 + bbox_h / 2

    return (
        x_center / img_w,
        y_center / img_h,
        bbox_w / img_w,
        bbox_h / img_h
    )


def copy_image(src_path, dst_path):
    Path(os.path.dirname(dst_path)).mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_path, dst_path)


def write_label(label_path, lines):
    Path(os.path.dirname(label_path)).mkdir(parents=True, exist_ok=True)
    with open(label_path, "w", encoding="utf-8") as f:
        for line in lines:
            f.write(line + "\n")


def read_synthetic_metadata():
    records = []

    with open(SYN_METADATA, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            records.append(row)

    return records


def split_items(items):
    random.shuffle(items)

    n_train = int(len(items) * TRAIN_RATIO)

    train_items = items[:n_train]
    val_items = items[n_train:]

    return train_items, val_items


# ============================================================
# PREPARE DATA
# ============================================================

def prepare_synthetic_positives():
    records = read_synthetic_metadata()

    items = []

    for row in records:
        filename = row["filename"]

        src_img = os.path.join(SYN_POS_DIR, filename)

        if not os.path.isfile(src_img):
            print(f"Missing image: {src_img}")
            continue

        items.append({
            "type": "positive",
            "src_img": src_img,
            "filename": filename,
            "bbox_x1": float(row["bbox_x1"]),
            "bbox_y1": float(row["bbox_y1"]),
            "bbox_x2": float(row["bbox_x2"]),
            "bbox_y2": float(row["bbox_y2"]),
        })

    return items


def prepare_synthetic_negatives():
    image_exts = [".jpg", ".jpeg", ".png", ".bmp", ".webp"]

    items = []

    for p in Path(SYN_NEG_DIR).iterdir():
        if p.suffix.lower() in image_exts:
            items.append({
                "type": "negative",
                "src_img": str(p),
                "filename": p.name
            })

    return items


def save_items(items, split):
    for idx, item in enumerate(items):
        src_img = item["src_img"]

        # Prefix filenames to avoid name collisions
        new_filename = f"{item['type']}_{idx:05d}_{item['filename']}"
        dst_img = os.path.join(YOLO_DIR, "images", split, new_filename)

        copy_image(src_img, dst_img)

        label_filename = Path(new_filename).with_suffix(".txt").name
        label_path = os.path.join(YOLO_DIR, "labels", split, label_filename)

        if item["type"] == "positive":
            with Image.open(src_img) as im:
                img_w, img_h = im.size

            xc, yc, bw, bh = yolo_bbox_from_xyxy(
                item["bbox_x1"],
                item["bbox_y1"],
                item["bbox_x2"],
                item["bbox_y2"],
                img_w,
                img_h
            )

            label_line = f"{CLASS_ID} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"
            write_label(label_path, [label_line])

        else:
            # Negative image: empty label file
            write_label(label_path, [])


def write_data_yaml():
    yaml_path = os.path.join(YOLO_DIR, "data.yaml")

    content = f"""path: {YOLO_DIR.replace(os.sep, '/')}
train: images/train
val: images/val

names:
  0: {CLASS_NAME}
"""

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"Wrote: {yaml_path}")


def main():
    ensure_dirs()

    positives = prepare_synthetic_positives()
    negatives = prepare_synthetic_negatives()

    all_items = positives + negatives

    print(f"Synthetic positives found: {len(positives)}")
    print(f"Synthetic negatives found: {len(negatives)}")

    train_items, val_items = split_items(all_items)

    save_items(train_items, "train")
    save_items(val_items, "val")

    write_data_yaml()

    print("\nYOLO synthetic dataset prepared.")
    print(f"Train images: {len(train_items)}")
    print(f"Val images: {len(val_items)}")
    print(f"Dataset path: {YOLO_DIR}")


if __name__ == "__main__":
    main()