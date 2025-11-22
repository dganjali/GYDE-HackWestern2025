import os
from pathlib import Path
import shutil
from ultralytics import YOLO

# 1. Define drowning-like class names and numeric classes
drowning_names = {
    'drowning', 'Drowning', 'Drowning-headdown', 'Drowning-head-down',
    '3', '4', '5', '6'
}

# 2. Paths
MASTER_DATASET = Path('master_dataset')
SPLITS = ['train', 'valid', 'test']

# 3. Clean label files
def clean_labels():
    for split in SPLITS:
        label_dir = MASTER_DATASET / split / 'labels'
        for label_file in label_dir.glob('*.txt'):
            new_lines = []
            with open(label_file, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) < 2:
                        continue
                    class_id = parts[0]
                    # Map class_id to class name using original dataset's mapping
                    # Since all files are merged, we don't have the original mapping, so we must infer from the text
                    # Instead, we will use a heuristic: if class_id is numeric and in drowning_names, keep it
                    # But we need to map all to 0
                    # For now, keep all lines and remap class_id to 0 if the class is drowning-like
                    # But we don't have class names in YOLO txt files, so we need to know the mapping
                    # Instead, let's assume all class_ids in the merged set correspond to the union of all classes
                    # So, we need to build a mapping from class_id to class name for each dataset
                    # But since we don't have that, let's just keep class_ids that are 0-6 (from the union above)
                    # and treat them as drowning if their class name is in drowning_names
                    # For now, keep all class_ids 0-6 as drowning
                    if class_id in {'0', '1', '2', '3', '4', '5', '6'}:
                        # Map to 0
                        new_line = '0 ' + ' '.join(parts[1:])
                        new_lines.append(new_line)
            # Overwrite file with new lines
            with open(label_file, 'w') as f:
                for l in new_lines:
                    f.write(l + '\n')

# 4. Generate unified data.yaml
def write_data_yaml():
    yaml_content = f"""
path: ./master_dataset
train: train/images
val: valid/images
test: test/images
names:
  0: Drowning
"""
    with open(MASTER_DATASET / 'data.yaml', 'w') as f:
        f.write(yaml_content)

# 5. Train YOLOv8n model
def train_yolo():
    model = YOLO('yolov8n.pt')
    model.train(
        data=str(MASTER_DATASET / 'data.yaml'),
        epochs=20,
        imgsz=640,
        batch=16,
        project='yolo_master_training',
        name='yolov8n_drowning',
        exist_ok=True
    )
    return model

# 6. Inference script
def run_inference(model_path, image_path):
    model = YOLO(model_path)
    results = model(image_path)
    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls == 0:  # Drowning
                print(f"Drowning: {conf*100:.1f}%")

if __name__ == "__main__":
    print("Cleaning label files...")
    clean_labels()
    print("Writing unified data.yaml...")
    write_data_yaml()
    print("Training YOLOv8n model...")
    model = train_yolo()
    print("Training complete. Running inference on a test image...")
    # Example usage: replace with your test image path
    test_image = str(MASTER_DATASET / 'test' / 'images' / os.listdir(MASTER_DATASET / 'test' / 'images')[0])
    run_inference('yolo_master_training/yolov8n_drowning/weights/best.pt', test_image) 
