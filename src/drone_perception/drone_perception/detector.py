"""ObjectDetector — YOLO-детектор/трекер (слим-порт логики из project_1).

Перенесён из project_1 `src/detector.py` как ЛОГИКА (решение Р3 / Ф3-4): оставлен путь
.pt (CPU/CUDA), убраны backends .onnx/.engine — их хватает для проекта. Дивергенция от
project_1 ожидаема.

Фаза 4 (Ф4-1): добавлен режим ТРЕКИНГА — `model.track(persist=True)` со встроенным в
ultralytics ByteTrack. Даёт стабильный `track_id` между кадрами «из коробки», без новых
зависимостей. Режим переключается флагом `tracking`: False → как в Фазе 3 (detect-only,
id=None), True → ByteTrack. persist=True сохраняет состояние трекера между вызовами, поэтому
кадры ОБЯЗАНЫ идти последовательно от одного источника (так и есть — один detector_node).

Формат детекции (Фаза 4): ((x1, y1, x2, y2), class_name, confidence, track_id).
track_id — int от трекера, либо None (detect-only, либо трек ещё не подтверждён ByteTrack).
Вход detect()/track() — кадр OpenCV в BGR (см. docs/phase3_setup.md §6 про RGB/BGR).
"""

from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import torch
from ultralytics import YOLO

# ((x1, y1, x2, y2), class_name, confidence, track_id|None)
Detection = Tuple[Tuple[int, int, int, int], str, float, Optional[int]]


class ObjectDetector:
    """YOLO-детектор с опциональным ByteTrack-трекингом."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
        device: str = "cuda",
        tracking: bool = False,
        tracker: str = "bytetrack.yaml",
    ) -> None:
        """Args:
        model_path: путь/имя весов YOLO (.pt). Если файла нет, ultralytics
            попытается скачать по имени (напр. "yolov8n.pt").
        confidence_threshold: минимальная уверенность детекции.
        device: "cuda" или "cpu". При запросе cuda без GPU — откат на cpu.
        tracking: включить ByteTrack (track_id между кадрами). False = detect-only (Фаза 3).
        tracker: конфиг трекера ultralytics ("bytetrack.yaml" | "botsort.yaml").
        """
        self.model_path = str(model_path)
        self.confidence_threshold = confidence_threshold
        self.device = self._resolve_device(device)
        self.tracking = tracking
        self.tracker = tracker

        # task="detect" объявляем явно (детекция-онли) — глушит варнинг угадывания
        # задачи. .to(device) нужен только нативному .pt (мы используем именно его).
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

    def detect(self, frame_bgr: np.ndarray) -> List[Detection]:
        """Детектировать (и, если tracking=True, трекать) объекты в кадре (OpenCV BGR).

        Returns:
            [((x1, y1, x2, y2), class_name, confidence, track_id|None), ...]
        """
        if self.tracking:
            # persist=True — держим состояние трекера между кадрами (одна последовательность).
            results = self.model.track(
                frame_bgr,
                conf=self.confidence_threshold,
                tracker=self.tracker,
                persist=True,
                verbose=False,
                device=self.device,
            )
        else:
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
                # box.id есть только в режиме трекинга и только для подтверждённых треков.
                track_id = int(box.id[0]) if box.id is not None else None
                detections.append(((x1, y1, x2, y2), class_name, confidence, track_id))

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
