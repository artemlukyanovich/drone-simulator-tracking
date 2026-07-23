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
        logger=None,
        cpu_fallback: bool = False,
    ) -> None:
        """Args:
        model_path: путь/имя весов YOLO (.pt). Если файла нет, ultralytics
            попытается скачать по имени (напр. "yolov8n.pt").
        confidence_threshold: минимальная уверенность детекции.
        device: "cuda" или "cpu". При запросе cuda без GPU — откат на cpu.
        tracking: включить ByteTrack (track_id между кадрами). False = detect-only (Фаза 3).
        tracker: конфиг трекера ultralytics ("bytetrack.yaml" | "botsort.yaml").
        logger: rclpy-логгер ноды (см. reid/embedder.py) — сообщения об откате на CPU
            должны попадать в ROS-лог, а не в stdout. None → print (юнит-тесты вне ноды).
        cpu_fallback: при нехватке VRAM на cuda молча откатиться на cpu вместо падения ноды.
            По умолчанию False: CPU-инференс YOLOv8n в 9-10× медленнее cuda (замерено
            2026-07-23) — тихая деградация даёт рваное видео и портит демо незаметно для
            оператора. Явное падение ноды заметно сразу и указывает на реальную причину
            (нехватка VRAM, обычно из-за одновременной записи через OBS). True — включить
            обратно, если важнее устойчивость (не-демо прогоны, отладка).
        """
        self.model_path = str(model_path)
        self.confidence_threshold = confidence_threshold
        self.tracking = tracking
        self.tracker = tracker
        self._log = logger
        self._cpu_fallback = cpu_fallback
        self.device = self._resolve_device(device)

        # task="detect" объявляем явно (детекция-онли) — глушит варнинг угадывания
        # задачи. .to(device) нужен только нативному .pt (мы используем именно его).
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

    def _say(self, msg, warn=False):
        if self._log is None:
            print(msg)
            return
        # warning и info — РАЗНЫМИ строками намеренно, см. reid/embedder.py._say: rclpy
        # привязывает уровень к месту вызова и падает при смене уровня с одной строки.
        if warn:
            self._log.warning(msg)
        else:
            self._log.info(msg)

    def detect(self, frame_bgr: np.ndarray) -> List[Detection]:
        """Детектировать (и, если tracking=True, трекать) объекты в кадре (OpenCV BGR).

        При нехватке видеопамяти на CUDA (см. тот же отказ 2026-07-19, что чинили в
        reid/embedder.py — YOLO делит GPU с рендером Gazebo и ReID) НЕ падаем: освобождаем
        занятое, пересобираем модель на CPU и повторяем этот же кадр. Частично загруженная
        (упавшая на fuse()) модель может быть в неопределённом состоянии, поэтому это именно
        пересборка YOLO(), а не повторный .to() на старом объекте.

        Returns:
            [((x1, y1, x2, y2), class_name, confidence, track_id|None), ...]
        """
        if not self._cpu_fallback:
            return self._run(frame_bgr)
        try:
            return self._run(frame_bgr)
        except Exception as exc:                              # noqa: BLE001
            if self.device == "cpu":
                raise
            self._say(f"YOLO на {self.device} не поднялась ({type(exc).__name__}), "
                      f"пробую на cpu. Причина: {exc}", warn=True)
            self._free_cuda()
            self.device = "cpu"
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            self._say(f"YOLO готова: {self.model_path} на cpu")
            return self._run(frame_bgr)

    def _run(self, frame_bgr: np.ndarray) -> List[Detection]:
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

    def _resolve_device(self, device: str) -> str:
        """Откат на CPU, если запрошена CUDA, но GPU недоступен."""
        if device == "cuda" and not torch.cuda.is_available():
            self._say("CUDA запрошена, но недоступна — откатываюсь на cpu.", warn=True)
            return "cpu"
        return device

    @staticmethod
    def _free_cuda():
        """Вернуть видеопамять, занятую неудавшейся загрузкой (см. reid/embedder.py)."""
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:                                      # noqa: BLE001
            pass

    @staticmethod
    def _normalize_model_path(model_path: str) -> str:
        """Добавить суффикс .pt, если имя без расширения (как в project_1)."""
        p = Path(model_path)
        return model_path if p.suffix else f"{model_path}.pt"
