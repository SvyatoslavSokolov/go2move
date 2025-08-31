"""
go2move - инструменты для управления роботом Unitree Go2.

Основные компоненты:
- main: Основной скрипт симуляции и логика управления.
- controller: Класс для обработки ввода с клавиатуры и GUI.
- add_types: Дополнительные типы данных
"""

from . import main
from . import controller
from . import add_types

from .add_types import * 
from .main import * 
from .controller import *

__all__ = [
    'main',
    'controller'
    'add_types'
]