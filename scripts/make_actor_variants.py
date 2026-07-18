#!/usr/bin/env python3
"""Генератор вендорных обликов ходячих actor'ов (Фаза 4, Ф4-12).

Берёт базовый меш Fuel `Mingfei/actor/walk.dae` и раскладывает его перекрашенные копии в
`src/drone_simulator/models/actor_<key>/` — по одному model-каталогу на человека. Палитра и
ростовки — в `configs/simulator/actor_variants.yaml` (§7 CLAUDE.md: числа не в коде).

Меш НЕ текстурирован: цвета лежат прямо в COLLADA как <diffuse> именованных материалов
(`sweater-green-effect`, `jeans-blue-effect`), поэтому перекраска — точечная замена одного
тега, без графических редакторов и без похода в Fuel.

РЕЗУЛЬТАТ КОММИТИТСЯ. Скрипт нужен для воспроизводимости и повторной перекраски, но сцена
не должна зависеть от того, прогрет ли у кого-то кэш Fuel (иначе ловушка порядка запуска).

Запуск (из корня проекта, venv не обязателен — только стандартная библиотека + yaml):
    python3 scripts/make_actor_variants.py
    python3 scripts/make_actor_variants.py --base /путь/к/walk.dae   # если кэш в другом месте
"""

import argparse
import re
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
CONFIG = REPO / 'configs' / 'simulator' / 'actor_variants.yaml'
MODELS = REPO / 'src' / 'drone_simulator' / 'models'
FUEL_BASE = (Path.home() / '.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/1/meshes/walk.dae')

# Материал COLLADA → ключ цвета в конфиге. Имена заданы автором меша и стабильны.
MATERIALS = {'sweater-green': 'sweater', 'jeans-blue': 'jeans'}

MODEL_CONFIG = """<?xml version="1.0"?>
<!-- СГЕНЕРИРОВАНО scripts/make_actor_variants.py — руками не править.
     Вендорный облик actor'а для worlds/follow_crowd.sdf (Фаза 4, Ф4-12): копия меша
     Mingfei/actor с перекрашенными материалами свитера/джинсов. Нужен, чтобы люди в
     сцене отличались друг от друга и ре-идентификация (Ф4-13) имела сигнал (риск R7).
     Палитра/ростовки — configs/simulator/actor_variants.yaml. -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Mingfei (базовый меш, Gazebo Fuel)</name>
  </author>
  <description>{description}</description>
</model>
"""

# Actor'у нужен только меш (он подключается через <skin>/<animation> в мире), но Gazebo
# резолвит model:// лишь для каталога с model.config; минимальный model.sdf держим рядом,
# чтобы каталог был валидной моделью.
MODEL_SDF = """<?xml version="1.0" ?>
<!-- СГЕНЕРИРОВАНО scripts/make_actor_variants.py — руками не править.
     Заглушка: реально используется только meshes/walk.dae, который мир подключает как
     <skin>/<animation> у <actor>. Модель нужна, чтобы резолвился model://{name}/... -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link"/>
  </model>
</sdf>
"""


def recolor(dae_text, palette):
    """Заменить <diffuse> у именованных материалов. Возвращает (текст, число замен)."""
    out, done = dae_text, 0
    for material, key in MATERIALS.items():
        r, g, b = palette[key]
        # Ищем ИМЕННО внутри нужного <effect …>: у файла 6 блоков <diffuse>, и глобальная
        # замена перекрасила бы заодно кожу и глаза.
        start = out.index(f'<effect id="{material}-effect"')
        end = out.index('</effect>', start)
        block = out[start:end]
        block_new, n = re.subn(
            r'(<color sid="diffuse">)[^<]*(</color>)',
            rf'\g<1>{r} {g} {b} 1\g<2>', block, count=1)
        if n != 1:
            raise RuntimeError(f'не нашёл <diffuse> в эффекте {material}')
        out = out[:start] + block_new + out[end:]
        done += n
    return out, done


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base', type=Path, default=FUEL_BASE,
                    help=f'базовый walk.dae (по умолчанию кэш Fuel: {FUEL_BASE})')
    args = ap.parse_args()

    if not args.base.is_file():
        sys.exit(f'ОШИБКА: базовый меш не найден: {args.base}\n'
                 'Он появляется в кэше Fuel после первого запуска мира с actor\'ом '
                 '(например follow_target.sdf), либо укажи путь через --base.')

    cfg = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    base_text = args.base.read_text(encoding='utf-8')

    for key, spec in cfg['actors'].items():
        name = f'actor_{key}'
        target_dir = MODELS / name
        (target_dir / 'meshes').mkdir(parents=True, exist_ok=True)

        dae, n = recolor(base_text, spec)
        (target_dir / 'meshes' / 'walk.dae').write_text(dae, encoding='utf-8')
        (target_dir / 'model.config').write_text(
            MODEL_CONFIG.format(name=name, description=spec['description']), encoding='utf-8')
        (target_dir / 'model.sdf').write_text(MODEL_SDF.format(name=name), encoding='utf-8')

        print(f'{name:16s} материалов перекрашено: {n}  scale={spec["scale"]}  '
              f'свитер={spec["sweater"]} джинсы={spec["jeans"]}')

    print(f'\nГотово: {MODELS}\nДальше — мир follow_crowd.sdf ссылается на '
          'model://actor_<key>/meshes/walk.dae')


if __name__ == '__main__':
    main()
