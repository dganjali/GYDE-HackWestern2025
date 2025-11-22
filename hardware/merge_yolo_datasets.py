import os
import shutil
from pathlib import Path

# List your dataset folders here (relative to script location)
DATASET_DIRS = [
    'Dataset 1 - hjhn j - 9884',
    'Dataset 2 - senior2 - 4565',
    'Dataset 3 - DrowningDetection - 9532',
    'Dataset 4 - Pool safety - 8370',
]

SPLITS = ['train', 'valid', 'test']  # 'valid' is used in your structure instead of 'val'
IMG_EXTS = {'.jpg', '.jpeg', '.png'}
LABEL_EXT = '.txt'

MASTER_DIR = Path('master_dataset')


def make_master_dirs():
    for split in SPLITS:
        (MASTER_DIR / split / 'images').mkdir(parents=True, exist_ok=True)
        (MASTER_DIR / split / 'labels').mkdir(parents=True, exist_ok=True)


def copy_files():
    stats = {}
    for dataset in DATASET_DIRS:
        dataset_dir = Path(dataset)
        dataset_id = dataset.replace(' ', '_')  # Use folder name as unique prefix
        stats[dataset] = {}
        for split in SPLITS:
            img_src = dataset_dir / split / 'images'
            lbl_src = dataset_dir / split / 'labels'
            img_dst = MASTER_DIR / split / 'images'
            lbl_dst = MASTER_DIR / split / 'labels'
            img_count = 0
            lbl_count = 0
            # Copy images
            if img_src.exists():
                for f in img_src.iterdir():
                    if f.suffix.lower() in IMG_EXTS:
                        new_name = f"{dataset_id}__{f.name}"
                        shutil.copy2(f, img_dst / new_name)
                        img_count += 1
            # Copy labels
            if lbl_src.exists():
                for f in lbl_src.iterdir():
                    if f.suffix.lower() == LABEL_EXT:
                        new_name = f"{dataset_id}__{f.name}"
                        shutil.copy2(f, lbl_dst / new_name)
                        lbl_count += 1
            stats[dataset][split] = {'images': img_count, 'labels': lbl_count}
    return stats


def print_stats(stats):
    print("\nCopy Summary:")
    for dataset, splits in stats.items():
        print(f"\nDataset: {dataset}")
        for split, counts in splits.items():
            print(f"  {split}: {counts['images']} images, {counts['labels']} labels copied")


def main():
    make_master_dirs()
    stats = copy_files()
    print_stats(stats)

if __name__ == "__main__":
    main() 
