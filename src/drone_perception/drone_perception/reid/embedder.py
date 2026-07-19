"""Эмбеддинги кропов через OpenCLIP (Фаза 4, Ф4-13).

Порт `project_1/src/embedder.py`. Отличия от оригинала — по месту применения, не по сути:
  * `crop()` (кроппер project_1) свёрнут сюда как `crop_bbox` — нам нужен один кроп на
    кандидата, а не словарь по всем трекам, и никакого сохранения кропов на диск;
  * добавлен `available` — нода должна уметь подняться и работать БЕЗ ReID (детектор Фазы 3
    остаётся рабочим), если `open-clip-torch` не установлен или веса не скачались;
  * логгер вместо print — сообщения должны попадать в ROS-лог, а не в stdout ноды.

Модель грузится один раз в конструкторе (~350 МБ весов ViT-B-32 из кэша Torch Hub, при
первом запуске качаются). Векторы L2-нормированы → косинус == скалярное произведение.
"""

from typing import List, Optional

import cv2
import numpy as np


def crop_bbox(frame: np.ndarray, bbox, padding: int = 0) -> np.ndarray:
    """Вырезать bbox из кадра с полем `padding`, обрезая по границам кадра.

    Возвращает BGR-кроп либо пустой массив, если bbox вырожден. Поле нужно, чтобы в кроп
    попал контур человека целиком: bbox YOLO часто режет по краю одежды, а эмбеддинг
    чувствителен к обрезанию силуэта.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = (int(v) for v in bbox)
    x1 = max(0, x1 - padding)
    y1 = max(0, y1 - padding)
    x2 = min(w, x2 + padding)
    y2 = min(h, y2 + padding)
    if x2 <= x1 or y2 <= y1:
        return np.empty((0, 0, 3), dtype=np.uint8)
    return frame[y1:y2, x1:x2].copy()


class ObjectEmbedder:
    """OpenCLIP-энкодер кропов в векторы признаков."""

    def __init__(self, model_name='ViT-B-32', pretrained='laion2b_s34b_b79k',
                 device='cuda', normalize=True, logger=None):
        self.model_name = model_name
        self.pretrained = pretrained
        self.normalize = normalize
        self._log = logger
        self._model = None
        self._preprocess = None
        self.device = self._resolve_device(device)
        self.available = self._load_model()

    def _say(self, msg, warn=False):
        if self._log is None:
            return
        (self._log.warning if warn else self._log.info)(msg)

    def _resolve_device(self, device):
        try:
            import torch
        except ImportError:
            return 'cpu'
        if device == 'cuda' and not torch.cuda.is_available():
            self._say('ReID: запрошена cuda, но она недоступна — откатываюсь на cpu.', warn=True)
            return 'cpu'
        return device

    def _load_model(self) -> bool:
        """Загрузить модель. False = ReID недоступен (нода продолжит работать без него)."""
        try:
            import open_clip
        except ImportError:
            self._say('ReID ВЫКЛЮЧЕН: нет open-clip-torch (pip install -r requirements.txt). '
                      'Детектор работает, но перезахват цели после потери невозможен.', warn=True)
            return False
        try:
            self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                self.model_name, pretrained=self.pretrained, device=self.device)
            self._model.eval()
        except Exception as exc:                                  # noqa: BLE001
            self._say(f'ReID ВЫКЛЮЧЕН: не удалось загрузить {self.model_name}/{self.pretrained}: '
                      f'{exc}', warn=True)
            return False
        self._say(f'ReID готов: {self.model_name}/{self.pretrained} на {self.device}')
        return True

    def embed(self, crop: np.ndarray) -> Optional[np.ndarray]:
        """Эмбеддинг одного BGR-кропа. None — если ReID недоступен или кроп пуст."""
        result = self.embed_batch([crop])
        return None if result is None or len(result) == 0 else result[0]

    def embed_batch(self, crops: List[np.ndarray]) -> Optional[np.ndarray]:
        """Эмбеддинги списка BGR-кропов → (N, dim). None — если ReID недоступен.

        Батчем, а не в цикле: при перезахвате кандидатов несколько, и один прогон модели
        заметно дешевле нескольких — это часть бюджета производительности (Ф4-15, риск R4).
        """
        if not self.available:
            return None
        usable = [c for c in crops if c is not None and c.size > 0]
        if not usable:
            return np.empty((0,), dtype=np.float32)

        import torch
        inputs = [self._preprocess(self._to_pil(c)) for c in usable]
        batch = torch.stack(inputs).to(self.device)
        with torch.no_grad():
            features = self._model.encode_image(batch)
        embeddings = features.cpu().numpy().astype(np.float32)

        if self.normalize:
            norms = np.linalg.norm(embeddings, axis=1, keepdims=True)
            norms = np.where(norms > 0, norms, 1.0)
            embeddings = embeddings / norms
        return embeddings

    @staticmethod
    def _to_pil(crop: np.ndarray):
        from PIL import Image
        return Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
