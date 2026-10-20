import os
import random
import shutil
from pathlib import Path

# ============================================================
# CONFIG
# ============================================================

BASE_DIR = r"E:\berkay\Quanser\images"

SYN_POS_IMG_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "positive")
SYN_POS_LABEL_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "positive_labels")

SYN_NEG_IMG_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "negative")
SYN_NEG_LABEL_DIR = os.path.join(BASE_DIR, "synthetic_pad", "generated_dataset", "negative_labels")

PHONE_NEG_IMG_DIR = os.path.join(BASE_DIR, "phone_photos", "phone_negative")

YOLO_DIR = os.path.join(BASE_DIR, "dataset_yolo_v2")

CLASS_NAME = "landing_pad"

TRAIN_RATIO = 0.8
RANDOM_SEED = 42
CLEAR_OLD_DATASET = True

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

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


def get_image_files(folder):
    folder = Path(folder)
    if not folder.exists():
        print(f"WARNING: folder does not exist: {folder}")
        return []

    return [p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS]


def copy_pair(image_path, label_path, split, prefix):
    new_image_name = f"{prefix}_{image_path.name}"
    new_label_name = Path(new_image_name).with_suffix(".txt").name

    dst_img = os.path.join(YOLO_DIR, "images", split, new_image_name)
    dst_lbl = os.path.join(YOLO_DIR, "labels", split, new_label_name)

    shutil.copy2(image_path, dst_img)

    if label_path is not None and os.path.isfile(label_path):
        shutil.copy2(label_path, dst_lbl)
    else:
        # Empty label for negative image
        Path(dst_lbl).write_text("", encoding="utf-8")


def split_items(items):
    random.shuffle(items)
    n_train = int(len(items) * TRAIN_RATIO)
    return items[:n_train], items[n_train:]


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

    print(f"Wrote data.yaml: {yaml_path}")


# ============================================================
# MAIN
# ============================================================

def main():
    ensure_dirs()

    items = []

    # Synthetic positives
    for img in get_image_files(SYN_POS_IMG_DIR):
        label = os.path.join(SYN_POS_LABEL_DIR, img.with_suffix(".txt").name)
        items.append({
            "image": str(img),
            "label": label,
            "prefix": "synpos"
        })

    # Synthetic negatives
    for img in get_image_files(SYN_NEG_IMG_DIR):
        label = os.path.join(SYN_NEG_LABEL_DIR, img.with_suffix(".txt").name)
        items.append({
            "image": str(img),
            "label": label,
            "prefix": "synneg"
        })

    # Phone negatives with empty labels
    for img in get_image_files(PHONE_NEG_IMG_DIR):
        items.append({
            "image": str(img),
            "label": None,
            "prefix": "phoneneg"
        })

    print(f"Total items collected: {len(items)}")

    train_items, val_items = split_items(items)

    for item in train_items:
        copy_pair(Path(item["image"]), item["label"], "train", item["prefix"])

    for item in val_items:
        copy_pair(Path(item["image"]), item["label"], "val", item["prefix"])

    write_data_yaml()

    print("\nYOLO v2 dataset prepared.")
    print(f"Train images: {len(train_items)}")
    print(f"Val images: {len(val_items)}")
    print(f"Dataset path: {YOLO_DIR}")


if __name__ == "__main__":
    main()