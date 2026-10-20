from pathlib import Path

# Change these if your folders are different
PHONE_POSITIVE_IMAGES = Path(r"E:\berkay\Quanser\images\unseen_test\positive")
PHONE_NEGATIVE_IMAGES = Path(r"E:\berkay\Quanser\images\unseen_test\negative")

PHONE_POSITIVE_LABELS = Path(r"runs\detect\unseen_positive_conf50\labels")
PHONE_NEGATIVE_LABELS = Path(r"runs\detect\unseen_negative_conf50\labels")

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def image_files(folder):
    return [p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS]


def has_detection(label_folder, image_path):
    label_file = label_folder / (image_path.stem + ".txt")

    if not label_file.exists():
        return False

    text = label_file.read_text(encoding="utf-8").strip()
    return len(text) > 0


def evaluate_presence(image_folder, label_folder, expected_positive=True):
    images = image_files(image_folder)

    detected = 0
    not_detected = 0

    detected_files = []
    not_detected_files = []

    for img in images:
        if has_detection(label_folder, img):
            detected += 1
            detected_files.append(img.name)
        else:
            not_detected += 1
            not_detected_files.append(img.name)

    total = len(images)

    print("\nFolder:", image_folder)
    print("Total images:", total)
    print("Detected:", detected)
    print("Not detected:", not_detected)

    if expected_positive:
        print("Correct positives:", detected)
        print("Missed positives:", not_detected)
        accuracy = detected / total * 100 if total > 0 else 0
        print(f"Positive detection rate: {accuracy:.2f}%")
    else:
        print("False positives:", detected)
        print("Correct negatives:", not_detected)
        false_positive_rate = detected / total * 100 if total > 0 else 0
        print(f"False positive rate: {false_positive_rate:.2f}%")

    return detected_files, not_detected_files


print("YOLO PHONE POSITIVE RESULTS")
pos_detected, pos_missed = evaluate_presence(
    PHONE_POSITIVE_IMAGES,
    PHONE_POSITIVE_LABELS,
    expected_positive=True
)

print("\nYOLO PHONE NEGATIVE RESULTS")
neg_false, neg_correct = evaluate_presence(
    PHONE_NEGATIVE_IMAGES,
    PHONE_NEGATIVE_LABELS,
    expected_positive=False
)

print("\nMissed positive images:")
for name in pos_missed[:20]:
    print("  ", name)

print("\nFalse positive negative images:")
for name in neg_false[:20]:
    print("  ", name)