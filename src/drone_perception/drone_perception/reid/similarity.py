"""Косинусное сходство эмбеддингов (Фаза 4, Ф4-13).

Порт `project_1/src/similarity.py` — перенесён почти без изменений: математика от домена
не зависит, а расхождение с лабораторией допустимо и ожидаемо (решение Р3). Убран
`find_best_match` (он искал лучшего в галерее из МНОГИХ объектов) — у нас цель ровно одна,
и сравнение с ней живёт в `TargetIdentity`.
"""

import numpy as np


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Косинусное сходство двух 1D-векторов; 0.0, если любой из них нулевой.

    Диапазон [-1..1], больше = похожее. Для L2-нормированных векторов (наш случай,
    `ObjectEmbedder(normalize=True)`) равно скалярному произведению.
    """
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a == 0.0 or norm_b == 0.0:
        return 0.0
    return float(np.dot(a, b) / (norm_a * norm_b))


def cosine_similarity_batch(query: np.ndarray, gallery: np.ndarray) -> np.ndarray:
    """Сходство одного вектора со всеми строками галереи (N, dim) → массив (N,)."""
    query_norm = np.linalg.norm(query)
    if query_norm == 0.0 or len(gallery) == 0:
        return np.zeros(len(gallery), dtype=np.float32)

    gallery_norms = np.linalg.norm(gallery, axis=1)
    gallery_norms = np.where(gallery_norms > 0, gallery_norms, 1.0)
    return (gallery.dot(query) / (gallery_norms * query_norm)).astype(np.float32)
