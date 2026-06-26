"""ObjectDetector — YOLO-детектор (слим-порт логики из project_1).

Перенесён из project_1 `src/detector.py` как ЛОГИКА (решение Р3 / Ф3-4): дрону нужна
только детекция одной цели за кадр, поэтому из оригинала убраны трекинг/ReID/counter и
поддержка backends .onnx/.engine — оставлен путь .pt (CPU/CUDA), которого хватает для MVP.
Дивергенция от project_1 ожидаема. Расширять (half/onnx) — при необходимости в Фазе 4.

Формат детекции (как в project_1): ((x1, y1, x2, y2), class_name, confidence).
Вход detect() — кадр OpenCV в BGR (см. docs/phase3_setup.md §6 про RGB/BGR).
"""

from pathlib import Path
from typing import List, Tuple

import numpy as np
import torch
from ultralytics import YOLO

# ((x1, y1, x2, y2), class_name, confidence)
Detection = Tuple[Tuple[int, int, int, int], str, float]


class ObjectDetector:
    """YOLO-детектор поверх одного кадра (без трекинга)."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
        device: str = "cuda",
    ) -> None:
        """Args:
        model_path: путь/имя весов YOLO (.pt). Если файла нет, ultralytics
            попытается скачать по имени (напр. "yolov8n.pt").
        confidence_threshold: минимальная уверенность детекции.
        device: "cuda" или "cpu". При запросе cuda без GPU — откат на cpu.
        """
        self.model_path = str(model_path)
        self.confidence_threshold = confidence_threshold
        self.device = self._resolve_device(device)

        # task="detect" объявляем явно (детекция-онли) — глушит варнинг угадывания
        # задачи. .to(device) нужен только нативному .pt (мы используем именно его).
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

    def detect(self, frame_bgr: np.ndarray) -> List[Detection]:
        """Детектировать объекты в кадре (OpenCV BGR).

        Returns:
            [((x1, y1, x2, y2), class_name, confidence), ...]
        """
        results = self.model(
            frame_bgr,
            conf=self.confidence_threshold,
            verbose=False,
            device=self.device,
        )

        detections: List[Detection] = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                detections.append(((x1, y1, x2, y2), class_name, confidence))

        return detections

    @staticmethod
    def _resolve_device(device: str) -> str:
        """Откат на CPU, если запрошена CUDA, но GPU недоступен."""
        if device == "cuda" and not torch.cuda.is_available():
            print("CUDA requested but unavailable. Falling back to CPU.")
            return "cpu"
        return device

    @staticmethod
    def _normalize_model_path(model_path: str) -> str:
        """Добавить суффикс .pt, если имя без расширения (как в project_1)."""
        p = Path(model_path)
        return model_path if p.suffix else f"{model_path}.pt"
