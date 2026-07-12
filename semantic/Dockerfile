# Semantic segmentation service for LIMO: SegFormer-B0 fine-tuned on Cityscapes, CPU inference.
# Build:  docker build -t limo-semantic semantic/
# Run:    docker run --rm -v /path/to/image_2:/in:ro -v /path/to/semantic_2:/out limo-semantic /in /out
FROM python:3.11-slim

RUN pip install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && pip install --no-cache-dir transformers pillow numpy

# Bake the model into the image so runs are reproducible and work offline.
RUN python -c "from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor; \
    m = 'nvidia/segformer-b0-finetuned-cityscapes-1024-1024'; \
    SegformerImageProcessor.from_pretrained(m); SegformerForSemanticSegmentation.from_pretrained(m)"

COPY segment.py /app/segment.py

# The model is baked into the image; never touch the network at runtime.
ENV HF_HUB_OFFLINE=1

ENTRYPOINT ["python", "/app/segment.py"]
