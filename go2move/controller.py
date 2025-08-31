import carb
import queue
import torch
import omni
import omni.ui as ui
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from add_types import ControllerConfig


class KeyboardController:
    """Класс для управления роботом через клавиатуру."""

    def __init__(self, command_queue: queue.Queue, device: torch.device):
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        self.command_queue = command_queue
        self.device = device
        self.is_running = True

        # Загружаем параметры из конфигурации
        self.config = ControllerConfig()
        self.speed = self.config.get('control.initial_speed', 0.5)
        self.turn = self.config.get('control.initial_turn', 1.0)
        self.moveBindings = self.config.get('control.move_bindings', {})
        self.speedBindings = self.config.get('control.speed_bindings', {})
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0

    def shutdown(self):
        self.is_running = False
        if self._sub_keyboard:
            self._input.unsubscribe_from_keyboard_events(self._sub_keyboard)
            self._sub_keyboard = None

    def _on_keyboard_event(self, event):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            key = event.input.name
            # Обработка специальных символов
            if key == 'COMMA':
                key = ','
            elif key == 'PERIOD':
                key = '.'
            elif key == 'MINUS':
                key = '<'
            elif key == 'GREATER':
                key = '>'
            else:
                key = key.upper() if event.modifiers & carb.input.KEYBOARD_MODIFIER_FLAG_SHIFT else key.lower()

            self._process_key(key)

    def _process_key(self, key):
        # Обработка движений
        if key in self.moveBindings:
            self.x = self.moveBindings[key][0]
            self.y = self.moveBindings[key][1]
            self.z = self.moveBindings[key][2]
            self.th = self.moveBindings[key][3]
        # Обработка изменения скорости
        elif key in self.speedBindings:
            self.speed = self.speed * self.speedBindings[key][0]
            self.turn = self.turn * self.speedBindings[key][1]
            # Сохраняем новые значения в конфигурации
            self.config.set('control.initial_speed', self.speed)
            self.config.set('control.initial_turn', self.turn)
        # Остановка при отпускании клавиш
        else:
            self.x = 0
            self.y = 0
            self.z = 0
            self.th = 0

        self._update_commands()

    def _update_commands(self):
        # Создаем команду в формате Twist
        lin_vel_x = self.x * self.speed
        lin_vel_y = self.y * self.speed
        lin_vel_z = self.z * self.speed
        ang_vel_z = self.th * self.turn

        # Формируем тензор команды
        new_commands = torch.tensor([lin_vel_x, lin_vel_y, ang_vel_z, 0.0], device=self.device)

        # Отправляем в очередь
        try:
            self.command_queue.get_nowait()
        except queue.Empty:
            pass
        self.command_queue.put_nowait(new_commands)


class KeyboardControllerGUI:
    """Класс для управления роботом через GUI."""

    def __init__(self, controller: KeyboardController):
        self.controller = controller
        self.config = ControllerConfig()
        self.window = ui.Window(
            "Robot Control",
            width=300,
            height=400,
            dockPreference=ui.DockPreference.RIGHT_TOP,
        )
        # Инициализируем словарь для кнопок
        self.key_buttons = {}
        self.build_ui()

    def build_ui(self):
        """Создает графический интерфейс"""
        with self.window.frame:
            with ui.VStack(spacing=10, alignment=ui.Alignment.CENTER):
                ui.Spacer(height=10)
                
                # Заголовок окна
                ui.Label("Robot Control", alignment=ui.Alignment.CENTER, 
                        style={"font_size": 18, "font_weight": "bold", "color": ui.color(200, 200, 200)})
                ui.Spacer(height=10)
                
                # Отображение текущих скоростей
                with ui.HStack(spacing=10):
                    with ui.VStack(spacing=2):
                        ui.Label("Speed:", style={"font_size": 12})
                        self.speed_label = ui.Label(f"{self.controller.speed:.1f}", 
                                                  style={"font_size": 14, "font_weight": "bold", "color": ui.color(100, 200, 100)})
                    with ui.VStack(spacing=2):
                        ui.Label("Turn:", style={"font_size": 12})
                        self.turn_label = ui.Label(f"{self.controller.turn:.1f}", 
                                                 style={"font_size": 14, "font_weight": "bold", "color": ui.color(100, 200, 100)})
                
                ui.Spacer(height=15)
                
                with ui.VStack(spacing=5, alignment=ui.Alignment.CENTER):
                    # Верхний ряд (U, I, O)
                    with ui.HStack(spacing=5):
                        ui.Spacer()
                        self.key_buttons["u"] = ui.Button("Fwd-Left", tooltip="Forward-Left (U)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["u"].set_clicked_fn(lambda: self._on_button_click("u"))
                        
                        self.key_buttons["i"] = ui.Button("Forward", tooltip="Forward (I)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["i"].set_clicked_fn(lambda: self._on_button_click("i"))
                        
                        self.key_buttons["o"] = ui.Button("Fwd-Right", tooltip="Forward-Right (O)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["o"].set_clicked_fn(lambda: self._on_button_click("o"))
                        ui.Spacer()
                    
                    # Средний ряд (J, K, L)
                    with ui.HStack(spacing=5):
                        ui.Spacer()
                        self.key_buttons["j"] = ui.Button("Left", tooltip="Left (J)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["j"].set_clicked_fn(lambda: self._on_button_click("j"))
                        
                        # Центральная кнопка - стоп
                        self.key_buttons["stop"] = ui.Button("Stop", tooltip="Stop", 
                                                           width=80, height=40,
                                                           style={"background_color": 0x884444ff, "border_radius": 5})
                        self.key_buttons["stop"].set_clicked_fn(lambda: self._on_stop())
                        
                        self.key_buttons["l"] = ui.Button("Right", tooltip="Right (L)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["l"].set_clicked_fn(lambda: self._on_button_click("l"))
                        ui.Spacer()
                    
                    # Нижний ряд (M, <, >)
                    with ui.HStack(spacing=5):
                        ui.Spacer()
                        self.key_buttons["m"] = ui.Button("Back-Left", tooltip="Backward-Left (M)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["m"].set_clicked_fn(lambda: self._on_button_click("m"))
                        
                        self.key_buttons[","] = ui.Button("Back", tooltip="Backward (,)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons[","].set_clicked_fn(lambda: self._on_button_click(","))
                        
                        self.key_buttons["."] = ui.Button("Back-Right", tooltip="Backward-Right (.)", 
                                                        width=80, height=40,
                                                        style={"background_color": 0x444444ff, "border_radius": 5})
                        self.key_buttons["."].set_clicked_fn(lambda: self._on_button_click("."))
                        ui.Spacer()
                
                ui.Spacer(height=15)
                
                # Кнопки вертикального движения
                with ui.HStack(spacing=10, alignment=ui.Alignment.CENTER):
                    ui.Spacer()
                    self.key_buttons["t"] = ui.Button("Up (T)", tooltip="Move Up (T)", 
                                                    width=100, height=35,
                                                    style={"background_color": 0x444466ff, "border_radius": 5})
                    self.key_buttons["t"].set_clicked_fn(lambda: self._on_button_click("t"))
                    
                    self.key_buttons["b"] = ui.Button("Down (B)", tooltip="Move Down (B)", 
                                                    width=100, height=35,
                                                    style={"background_color": 0x444466ff, "border_radius": 5})
                    self.key_buttons["b"].set_clicked_fn(lambda: self._on_button_click("b"))
                    ui.Spacer()
                
                ui.Spacer(height=15)
                
                # Панель управления скоростями
                with ui.VStack(spacing=5):
                    ui.Label("Speed Controls", style={"font_size": 14, "font_weight": "bold"})
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Speed+ (Q)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("q"),
                                 style={"background_color": 0x446644ff, "border_radius": 4})
                        
                        ui.Button("Speed- (Z)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("z"),
                                 style={"background_color": 0x664444ff, "border_radius": 4})
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Lin+ (W)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("w"),
                                 style={"background_color": 0x446644ff, "border_radius": 4})
                        
                        ui.Button("Lin- (X)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("x"),
                                 style={"background_color": 0x664444ff, "border_radius": 4})
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Ang+ (E)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("e"),
                                 style={"background_color": 0x446644ff, "border_radius": 4})
                        
                        ui.Button("Ang- (C)", width=90, height=30,
                                 clicked_fn=lambda: self._on_speed_button_click("c"),
                                 style={"background_color": 0x664444ff, "border_radius": 4})
                
                ui.Spacer(height=10)

    def _on_button_click(self, key):
        """Обработчик нажатия кнопки движения."""
        self.controller._process_key(key)
        self._update_labels()

    def _on_speed_button_click(self, key):
        """Обработчик нажатия кнопки изменения скорости."""
        self.controller._process_key(key)
        self._update_labels()

    def _on_stop(self):
        """Обработчик нажатия кнопки остановки."""
        self.controller.x = 0
        self.controller.y = 0
        self.controller.z = 0
        self.controller.th = 0
        self.controller._update_commands()
        self._update_labels()

    def _update_labels(self):
        """Обновляет отображение текущих скоростей."""
        self.speed_label.text = f"{self.controller.speed:.1f}"
        self.turn_label.text = f"{self.controller.turn:.1f}"


class ROS2KeyboardBridge(Node):
    """Мост для подключения к ROS2."""

    def __init__(self, command_queue: queue.Queue, device: torch.device):
        super().__init__('keyboard_controller_bridge')
        self.command_queue = command_queue
        self.device = device
        
        # Загружаем параметры из конфигурации
        self.config = ControllerConfig()
        self.timeout_threshold = self.config.get('control.timeout_threshold', 0.2)
        self.check_interval = self.config.get('control.check_interval', 0.1)
        
        self.last_msg_time = time.time()
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
    def twist_callback(self, msg: Twist):
        # Обновляем время последнего сообщения
        self.last_msg_time = time.time()
        
        # Преобразование сообщения Twist в тензор
        new_commands = torch.tensor([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            0.0
        ], device=self.device)

        # Отправка в очередь
        try:
            self.command_queue.get_nowait()
        except queue.Empty:
            pass
        self.command_queue.put_nowait(new_commands)