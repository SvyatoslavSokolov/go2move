"""
Пакет go2move - инструменты для управления роботом Unitree Go2 в сложной местности.

Основные компоненты:
- main: Основной скрипт симуляции и логика управления.
- controller: Класс для обработки ввода с клавиатуры и GUI.
"""

from . import main
from . import controller

from .main import *  # Импорт всех классов/функций из main
from .controller import *    # Импорт всех функций/классов из controller

__all__ = [
    'main',
    'controller'
]