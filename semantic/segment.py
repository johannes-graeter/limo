#!/usr/bin/env python3
"""Semantic segmentation of camera images for LIMO.

Runs SegFormer (fine-tuned on Cityscapes) over every image in the input directory and writes one
8-bit label image per input to the output directory (same file name). Pixel values are cityscapes
*label ids* (road=7, vegetation=21, sky=23, car=26, ...), which is what limo_with_kitti's
--semantic option expects.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

MODEL = "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"

# The model predicts cityscapes train ids (0..18); the pipeline works with label ids.
TRAIN_ID_TO_LABEL_ID = np.array(
    [7, 8, 11, 12, 13, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33], dtype=np.uint8
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="directory with input images (*.png, *.jpg)")
    parser.add_argument("output_dir", type=Path, help="directory for the label images")
    args = parser.parse_args()

    images = sorted(p for p in args.input_dir.iterdir() if p.suffix.lower() in {".png", ".jpg"})
    if not images:
        print(f"No images found in {args.input_dir}", file=sys.stderr)
        return 1
    args.output_dir.mkdir(parents=True, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    # Infer at native resolution: the default processor squashes images to 1024x1024, which
    # destroys thin structures in wide-aspect images like KITTI's (1226x370). SegFormer is fully
    # convolutional and handles arbitrary sizes.
    processor = SegformerImageProcessor.from_pretrained(MODEL, do_resize=False)
    model = SegformerForSemanticSegmentation.from_pretrained(MODEL).to(device).eval()
    print(f"Segmenting {len(images)} images on {device} with {MODEL} (native resolution)")

    start = time.time()
    for index, path in enumerate(images):
        image = Image.open(path).convert("RGB")
        inputs = processor(images=image, return_tensors="pt").to(device)
        with torch.no_grad():
            logits = model(**inputs).logits
        logits = torch.nn.functional.interpolate(
            logits, size=image.size[::-1], mode="bilinear", align_corners=False
        )
        train_ids = logits.argmax(dim=1)[0].cpu().numpy().astype(np.uint8)
        labels = TRAIN_ID_TO_LABEL_ID[train_ids]

        Image.fromarray(labels, mode="L").save(args.output_dir / (path.stem + ".png"))
        if (index + 1) % 25 == 0 or index + 1 == len(images):
            rate = (index + 1) / (time.time() - start)
            print(f"{index + 1}/{len(images)} ({rate:.2f} img/s)", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
