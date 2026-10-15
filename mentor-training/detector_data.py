"""Portable RGB/box dataset. Simulator metadata is never an input to the model."""
import hashlib
import json
import math
import random
from pathlib import Path
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision.transforms import functional as F

CLASSES = ('yellow', 'blue', 'red')

def read_boxes(path, width, height):
    boxes, labels = [], []
    for line in Path(path).read_text().splitlines():
        fields = list(map(float, line.split()))
        if len(fields) != 5 or not all(math.isfinite(x) for x in fields):
            raise ValueError(f'Malformed label: {path}')
        cls, cx, cy, bw, bh = fields
        if cls not in (0, 1, 2) or min(bw, bh) <= 0:
            raise ValueError(f'Invalid class/box: {path}')
        box = [(cx-bw/2)*width, (cy-bh/2)*height,
               (cx+bw/2)*width, (cy+bh/2)*height]
        if not (-.001 <= box[0] < box[2] <= width+.001 and
                -.001 <= box[1] < box[3] <= height+.001):
            raise ValueError(f'Box outside crop: {path}')
        boxes.append([max(0., box[0]), max(0., box[1]),
                      min(float(width), box[2]), min(float(height), box[3])])
        # Text format uses 0..2; torchvision reserves 0 for background.
        labels.append(int(cls)+1)
    return (torch.tensor(boxes, dtype=torch.float32).reshape(-1, 4),
            torch.tensor(labels, dtype=torch.int64))

def flip_boxes(boxes, width):
    out = boxes.clone()
    out[:, 0] = width - boxes[:, 2]
    out[:, 2] = width - boxes[:, 0]
    return out

def crop_to_full(boxes, record):
    """Restore detector boxes to full-frame pixels for camera backprojection."""
    out = boxes.clone()
    left, top = record['crop_rect_px'][:2]
    out[:, [0, 2]] += left
    out[:, [1, 3]] += top
    return out

class CubeDataset(Dataset):
    def __init__(self, root, split, augment=False, verify_hashes=False):
        self.root = Path(root)
        manifest = json.loads((self.root/'manifest.json').read_text())
        if manifest['classes'] != list(CLASSES):
            raise ValueError('Class mapping differs from this training recipe')
        self.records = [r for r in manifest['records'] if r['split'] == split]
        if not self.records:
            raise ValueError(f'Empty split: {split}')
        if augment and split != 'train':
            raise ValueError('Augmentation is only allowed for training')
        self.augment = augment
        for r in self.records:
            for key, hash_key in (('image_path','crop_image_sha256'), ('label_path','label_sha256')):
                path = self.root/r[key]
                if not path.is_file(): raise FileNotFoundError(path)
                if verify_hashes and hashlib.sha256(path.read_bytes()).hexdigest() != r[hash_key]:
                    raise ValueError(f'Checksum mismatch: {path}')

    def __len__(self): return len(self.records)

    def __getitem__(self, index):
        r = self.records[index]
        with Image.open(self.root/r['image_path']) as source:
            image = source.convert('RGB')
        w, h = image.size
        if [w,h] != r['crop_resolution']: raise ValueError('Image dimensions changed')
        boxes, labels = read_boxes(self.root/r['label_path'], w, h)
        if len(labels) != r['cube_count']: raise ValueError('Annotation count changed')
        if self.augment:
            if random.random() < .5:
                image = F.hflip(image); boxes = flip_boxes(boxes, w)
            # Hue is deliberately unchanged: object colour is the class label.
            image = F.adjust_brightness(image, random.uniform(.9, 1.1))
            image = F.adjust_contrast(image, random.uniform(.9, 1.1))
            image = F.adjust_saturation(image, random.uniform(.9, 1.1))
        target = dict(boxes=boxes, labels=labels,
                      image_id=torch.tensor(index, dtype=torch.int64),
                      area=(boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:,1]),
                      iscrowd=torch.zeros(len(labels), dtype=torch.int64))
        return F.to_tensor(image), target

def collate(batch): return tuple(zip(*batch))
