import os
import random
import shutil
from pathlib import Path

# ============================================================
# CONFIG
# ============================================================

BASE_DIR = r"E:\berkay\Quanser\images"

# Folder where you unzipped Roboflow export with 49 positive labeled images
ROBOFLOW_DIR = os.path.join(BASE_DIR, "roboflow_positive_49")

# Local phone negatives
PHONE_NEGATIVE_DIR = os.path.join(BASE_DIR, "phone_photos", "phone_negative")

# Final merged YOLO dataset
OUTPUT_DIR = os.path.join(BASE_DIR, "dataset_yolo_v3_local")

CLASS_NAME = "landing_pad"

TRAIN_RATIO = 0.80
VAL_RATIO = 0.20

CLEAR_OLD_OUTPUT = True

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

random.seed(42)


# ============================================================
# HELPERS
# ============================================================

def reset_output():
    if CLEAR_OLD_OUTPUT and os.path.isdir(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)

    for split in ["train", "val"]:
        Path(os.path.join(OUTPUT_DIR, "images", split)).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(OUTPUT_DIR, "labels", split)).mkdir(parents=True, exist_ok=True)


def get_images(folder):
    folder = Path(folder)
    if not folder.exists():
        return []
    return [p for p in folder.rglob("*") if p.suffix.lower() in IMAGE_EXTS]


def find_label_for_image(image_path):
    """
    Finds matching YOLO .txt label for a Roboflow image.
    Works for common Roboflow structures.
    """
    image_path = Path(image_path)

    # Case 1: train/images/img.jpg -> train/labels/img.txt
    parts = list(image_path.parts)

    if "images" in parts:
        label_parts = parts.copy()
        idx = label_parts.index("images")
        label_parts[idx] = "labels"
        label_path = Path(*label_parts).with_suffix(".txt")
        if label_path.exists():
            return label_path

    # Case 2: nearby labels folder
    possible = list(Path(ROBOFLOW_DIR).rglob(image_path.with_suffix(".txt").name))
    if possible:
        return possible[0]

    return None


def copy_item(image_path, label_path, split, prefix):
    image_path = Path(image_path)

    new_img_name = f"{prefix}_{image_path.name}"
    new_lbl_name = Path(new_img_name).with_suffix(".txt").name

    dst_img = Path(OUTPUT_DIR) / "images" / split / new_img_name
    dst_lbl = Path(OUTPUT_DIR) / "labels" / split / new_lbl_name

    shutil.copy2(image_path, dst_img)

    if label_path is not None and Path(label_path).exists():
        shutil.copy2(label_path, dst_lbl)
    else:
        # Empty label for negative image
        dst_lbl.write_text("", encoding="utf-8")


def write_data_yaml():
    yaml_path = Path(OUTPUT_DIR) / "data.yaml"

    content = f"""path: {OUTPUT_DIR.replace(os.sep, '/')}
train: images/train
val: images/val

names:
  0: {CLASS_NAME}
"""

    yaml_path.write_text(content, encoding="utf-8")
    print(f"Wrote {yaml_path}")


# ============================================================
# MAIN
# ============================================================

def main():
    reset_output()

    items = []

    # ------------------------------------------------------------
    # 1. Add Roboflow labeled positives
    # ------------------------------------------------------------
    roboflow_images = get_images(ROBOFLOW_DIR)

    for img in roboflow_images:
        label = find_label_for_image(img)

        if label is None:
            print(f"WARNING: no label found for Roboflow image: {img}")
            continue

        # Skip empty labels from Roboflow if any
        if label.read_text(encoding="utf-8").strip() == "":
            print(f"WARNING: empty positive label skipped: {label}")
            continue

        items.append({
            "image": img,
            "label": label,
            "prefix": "rfpos"
        })

    # ------------------------------------------------------------
    # 2. Add phone negatives with empty labels
    # ------------------------------------------------------------
    negative_images = get_images(PHONE_NEGATIVE_DIR)

    for img in negative_images:
        items.append({
            "image": img,
            "label": None,
            "prefix": "phoneneg"
        })

    print(f"Roboflow positive labeled images: {sum(1 for i in items if i['prefix'] == 'rfpos')}")
    print(f"Phone negative images: {sum(1 for i in items if i['prefix'] == 'phoneneg')}")
    print(f"Total images: {len(items)}")

    # ------------------------------------------------------------
    # 3. Split train / val
    # ------------------------------------------------------------
    random.shuffle(items)

    n_train = int(len(items) * TRAIN_RATIO)

    train_items = items[:n_train]
    val_items = items[n_train:]

    for item in train_items:
        copy_item(item["image"], item["label"], "train", item["prefix"])

    for item in val_items:
        copy_item(item["image"], item["label"], "val", item["prefix"])

    write_data_yaml()

    print("\nMerged YOLO dataset created.")
    print(f"Train images: {len(train_items)}")
    print(f"Val images: {len(val_items)}")
    print(f"Output: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()