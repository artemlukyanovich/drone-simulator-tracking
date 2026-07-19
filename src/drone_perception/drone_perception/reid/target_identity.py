"""Память облика ЗАЛОЧЕННОЙ цели и гейт перезахвата (Фаза 4, M2 / Ф4-11, Ф4-13).

Роль в системе: ByteTrack отвечает за ассоциацию кадр-к-кадру и выдаёт ВРЕМЕННЫЙ `track_id`,
который умирает вместе с треком. Этот класс держит ПОСТОЯННУЮ идентичность — «как выглядит
наш человек» — и отвечает на единственный вопрос: «этот кандидат — он?».

Почему не порт `ObjectMemory` из project_1 «как есть». Там реестр МНОГИХ объектов с
жизненным циклом `add/update/expire_old`, и объект деактивируется через `max_missing_frames`.
Нам нужно ровно обратное (Ф4-11): цель одна, и она **никогда не протухает** — дрон обязан
искать своего человека сколько угодно долго, а не «забыть» его по таймауту и взять другого.
Поэтому перенесена ЛОГИКА (роллинг-буфер эмбеддингов + способы агрегации + порог косинуса),
а реестр и экспирация — нет. Это ровно тот случай дивергенции от project_1, который решение
Р3 (`project_plan.md` §3) называет здоровым.

Скважность (Ф4-15): класс НЕ считает эмбеддинги сам — ему их передают. Решение «когда
тратить GPU» принимает нода, потому что бюджет делится с YOLO и рендером Gazebo (риск R4).
"""

from typing import List, Optional

import numpy as np

from drone_perception.reid.similarity import cosine_similarity_batch


class TargetIdentity:
    """Эталонные эмбеддинги цели + проверка кандидата на «это он».

    Порог `similarity_threshold` — единственная ручка строгости. Выше порог = строже гейт =
    реже ложный перезахват на чужого, но и реже возврат к своему (дрон дольше ищет). По
    Ф4-11 ошибка «взял чужого» дороже ошибки «не нашёл», поэтому порог держим высоким.
    """

    def __init__(self, similarity_threshold=0.90, max_embeddings=5,
                 aggregation='mean', ema_alpha=0.3, recent_n=3, weighted_decay=0.7):
        self.similarity_threshold = float(similarity_threshold)
        self.max_embeddings = int(max_embeddings)
        self.aggregation = aggregation
        self.ema_alpha = float(ema_alpha)
        self.recent_n = int(recent_n)
        self.weighted_decay = float(weighted_decay)

        self._embeddings: List[np.ndarray] = []   # роллинг-буфер обликов цели
        self._ema: Optional[np.ndarray] = None    # инкрементальное состояние для aggregation='ema'
        self.locked = False                        # цель захвачена (эталон есть)

    # --- Жизненный цикл эталона ---
    def lock(self, embedding: np.ndarray) -> None:
        """Захватить цель: этот облик становится эталоном. Сбрасывает прошлый."""
        self._embeddings = [embedding.copy()]
        self._ema = embedding.copy()
        self.locked = True

    def reinforce(self, embedding: np.ndarray) -> None:
        """Дополнить эталон свежим обликом цели (пока трекер её уверенно ведёт).

        Нужно, потому что человек поворачивается: облик со спины ≠ облик анфас. Без
        подпитки эталон устаревает и после разворота цели гейт перестал бы узнавать своего.
        Буфер ограничен `max_embeddings` — храним разные ракурсы, а не тысячу похожих кадров.
        """
        if not self.locked:
            return
        self._embeddings.append(embedding.copy())
        if len(self._embeddings) > self.max_embeddings:
            self._embeddings.pop(0)
        self._ema = (embedding.copy() if self._ema is None
                     else (self.ema_alpha * embedding
                           + (1.0 - self.ema_alpha) * self._ema).astype(np.float32))

    # --- Сверка кандидатов ---
    def reference(self) -> Optional[np.ndarray]:
        """Представительный эталон согласно выбранному способу агрегации."""
        if not self._embeddings:
            return None
        if self.aggregation == 'ema' and self._ema is not None:
            return self._ema
        if self.aggregation == 'recent':
            return np.mean(self._embeddings[-self.recent_n:], axis=0).astype(np.float32)
        if self.aggregation == 'weighted':
            n = len(self._embeddings)
            w = np.array([self.weighted_decay ** (n - 1 - i) for i in range(n)], dtype=np.float32)
            w /= w.sum()
            return np.average(self._embeddings, axis=0, weights=w).astype(np.float32)
        return np.mean(self._embeddings, axis=0).astype(np.float32)   # 'mean' по умолчанию

    def scores(self, embeddings: np.ndarray) -> np.ndarray:
        """Сходство каждого кандидата (N, dim) с эталоном → (N,)."""
        ref = self.reference()
        if ref is None or embeddings is None or len(embeddings) == 0:
            return np.zeros(0 if embeddings is None else len(embeddings), dtype=np.float32)
        return cosine_similarity_batch(ref, embeddings)

    def match(self, embeddings: np.ndarray):
        """Выбрать кандидата, прошедшего гейт. → (индекс|None, лучший балл).

        `None` означает «своего среди кандидатов нет» — и по Ф4-11 это НОРМАЛЬНЫЙ исход:
        нода публикует detected=false, дрон продолжает искать. Подмена цели ближайшим
        похожим здесь принципиально невозможна: без прохождения порога кандидат не
        возвращается вообще.
        """
        s = self.scores(embeddings)
        if len(s) == 0:
            return None, 0.0
        best = int(np.argmax(s))
        best_score = float(s[best])
        return (best, best_score) if best_score >= self.similarity_threshold else (None, best_score)
