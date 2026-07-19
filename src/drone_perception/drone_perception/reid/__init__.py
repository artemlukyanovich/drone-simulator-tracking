"""ReID-подсистема перцепции (Фаза 4, M2 / Ф4-13).

Порт стека ре-идентификации из project_1 (`real-time-object-counter/src/`) в ROS2-ноду
согласно решению Р3 (`docs/project_plan.md` §3): переносим ЛОГИКУ, не пакет.

Зачем нужен отдельный слой поверх трекера: `track_id` от ByteTrack — ВРЕМЕННЫЙ, он умирает
вместе с треком при окклюзии и возвращается уже другим числом. Сопровождение конкретного
человека (Ф4-11) — задача идентичности, а не ассоциации, поэтому нужна «память облика»:
эмбеддинг цели, с которым сверяется каждый кандидат при перезахвате.
"""

from drone_perception.reid.embedder import ObjectEmbedder
from drone_perception.reid.similarity import cosine_similarity, cosine_similarity_batch
from drone_perception.reid.target_identity import TargetIdentity

__all__ = ['ObjectEmbedder', 'TargetIdentity',
           'cosine_similarity', 'cosine_similarity_batch']
