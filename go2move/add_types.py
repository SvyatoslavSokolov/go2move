import yaml
import os

class ControllerConfig:
    """Класс для работы с конфигурационным файлом."""
    
    _instance = None
    _config = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ControllerConfig, cls).__new__(cls)
            cls._load_config()
        return cls._instance
    
    @classmethod
    def _load_config(cls):
        """Загружает конфигурацию из YAML файла."""
        config_path = os.path.join(os.path.dirname(__file__), 'configs/base_controller.yaml')
        try:
            with open(config_path, 'r') as f:
                cls._config = yaml.safe_load(f)
        except FileNotFoundError:
            # Если файл не найден, используем значения по умолчанию
            cls._config = {
                'control': {
                    'initial_speed': 0.5,
                    'initial_turn': 1.0,
                    'timeout_threshold': 0.2,
                    'check_interval': 0.1,
                    'move_bindings': {
                        'i': [1, 0, 0, 0],
                        'o': [1, 0, 0, -1],
                        'j': [0, 0, 0, 1],
                        'l': [0, 0, 0, -1],
                        'u': [1, 0, 0, 1],
                        ',': [-1, 0, 0, 0],
                        '.': [-1, 0, 0, 1],
                        'm': [-1, 0, 0, -1],
                        'O': [1, -1, 0, 0],
                        'I': [1, 0, 0, 0],
                        'J': [0, 1, 0, 0],
                        'L': [0, -1, 0, 0],
                        'U': [1, 1, 0, 0],
                        '<': [-1, 0, 0, 0],
                        '>': [-1, -1, 0, 0],
                        'M': [-1, 1, 0, 0],
                        't': [0, 0, 1, 0],
                        'b': [0, 0, -1, 0]
                    },
                    'speed_bindings': {
                        'q': [1.1, 1.1],
                        'z': [0.9, 0.9],
                        'w': [1.1, 1],
                        'x': [0.9, 1],
                        'e': [1, 1.1],
                        'c': [1, 0.9]
                    }
                }
            }
    
    @classmethod
    def get(cls, key, default=None):
        """Получает значение из конфигурации по ключу."""
        keys = key.split('.')
        value = cls._config
        for k in keys:
            value = value.get(k, {})
            if value == {}:
                return default
        return value
    
    @classmethod
    def set(cls, key, value):
        """Устанавливает значение в конфигурации по ключу."""
        keys = key.split('.')
        config = cls._config
        for k in keys[:-1]:
            config = config.setdefault(k, {})
        config[keys[-1]] = value
        cls._save_config()
    
    @classmethod
    def _save_config(cls):
        """Сохраняет конфигурацию в YAML файл."""
        config_path = os.path.join(os.path.dirname(__file__), 'configs/base_controller.yaml')
        with open(config_path, 'w') as f:
            yaml.safe_dump(cls._config, f, default_flow_style=False)