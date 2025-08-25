import carb
import queue
import torch
import omni
import omni.ui as ui


class KeyboardController:
    """Класс для управления роботом через клавиатуру."""

    def __init__(self, command_queue: queue.Queue, device: torch.device):
        """Инициализирует контроллер клавиатуры.

        Args:
            command_queue: Очередь команд для отправки роботу.
            device: (CPU или GPU).
        """
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        self.command_queue = command_queue
        self.device = device
        self.is_running = True

        # Скорость линейного движения робота.
        self.linear_speed = 1.0
        # Скорость углового движения робота.
        self.angular_speed = 0.8
        # Скорость бокового движения робота.
        self.strafe_speed = 0.5

        # Словарь для хранения состояний клавиш.
        self.key_states = {
            "UP": False,
            "DOWN": False,
            "LEFT": False,
            "RIGHT": False,
            "A": False,
            "D": False,
            "Q": False,
            "E": False,
            "ESCAPE": False,
            "C": False,
        }

    def shutdown(self):
        """Останавливает контроллер клавиатуры и отписывается от событий."""
        self.is_running = False
        if self._sub_keyboard:
            self._input.unsubscribe_from_keyboard_events(self._sub_keyboard)
            self._sub_keyboard = None
        logger.info("Keyboard controller shutting down.")

    def _on_keyboard_event(self, event):
        """Обрабатывает событие клавиатуры и обновляет состояния клавиш."""
        key_name = event.input.name
        if key_name in self.key_states:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.key_states[key_name] = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                self.key_states[key_name] = False

        # Обрабатываем нажатие клавиши ESCAPE и C отдельно, чтобы гарантировать их срабатывание.
        if key_name == "ESCAPE" and event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self.key_states["ESCAPE"] = True
        if key_name == "C" and event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self.key_states["C"] = True

        self._update_commands()

    def set_key_state(self, key_name: str, state: bool):
        """Программно устанавливает состояние клавиши (True/False).

        Args:
            key_name: Название клавиши.
            state: Состояние клавиши (True - нажата, False - отпущена).
        """
        if key_name in self.key_states:
            self.key_states[key_name] = state
            self._update_commands()

    def _update_commands(self):
        """Формирует команды на основе состояний клавиш и помещает их в очередь."""
        lin_vel_x = 0.0
        lin_vel_y = 0.0
        ang_vel_z = 0.0

        if self.key_states["UP"]:
            lin_vel_x = self.linear_speed
        if self.key_states["DOWN"]:
            lin_vel_x = -self.linear_speed * 0.5  # Уменьшаем скорость при движении назад
        if self.key_states["A"]:
            lin_vel_y = self.strafe_speed
        if self.key_states["D"]:
            lin_vel_y = -self.strafe_speed

        if self.key_states["Q"]:
            ang_vel_z = self.angular_speed
            lin_vel_x = 0  # Останавливаем движение вперед при повороте вбок
        if self.key_states["E"]:
            ang_vel_z = -self.angular_speed
            lin_vel_x = 0  # Останавливаем движение вперед при повороте вбок

        # Если не нажаты Q или E, обрабатываем клавиши LEFT и RIGHT для поворота.
        if not self.key_states["Q"] and not self.key_states["E"]:
            if self.key_states["LEFT"]:
                ang_vel_z = self.angular_speed
            if self.key_states["RIGHT"]:
                ang_vel_z = -self.angular_speed

        # Создаем тензор с новыми командами.
        new_commands = torch.tensor([lin_vel_x, lin_vel_y, ang_vel_z, 0.0], device=self.device)

        try:
            # Получаем текущие команды из очереди без блокировки.
            current_commands = self.command_queue.get_nowait()
            # Если новые команды отличаются от текущих, помещаем их в очередь.
            if not torch.equal(current_commands, new_commands):
                self.command_queue.put_nowait(new_commands)
            else:
                # Иначе помещаем текущие команды обратно в очередь, чтобы избежать лишних операций.
                self.command_queue.put_nowait(current_commands)
        except queue.Empty:
            # Если очередь пуста, просто помещаем новые команды в нее.
            self.command_queue.put_nowait(new_commands)


class KeyboardControllerGUI:
    """Класс для управления роботом через GUI."""

    def __init__(self, controller: KeyboardController):
        """Инициализирует графический интерфейс контроллера клавиатуры.

        Args:
            controller: Контроллер клавиатуры.
        """
        self.controller = controller
        # Создаем окно для управления роботом.
        self.window = ui.Window(
            "Robot Control",
            width=200,
            height=200,
            dockPreference=ui.DockPreference.RIGHT_TOP,
        )
        # Словарь для хранения кнопок GUI.
        self.key_buttons = {}

        # Переменные для отслеживания состояния нажатых кнопок GUI.
        self.pressed_buttons = set()

        self.build_ui()

        # Подписываемся на события мыши для обработки отпускания кнопки.
        self._input = carb.input.acquire_input_interface()
        self._mouse = omni.appwindow.get_default_app_window().get_mouse()
        self._mouse_event_sub = self._input.subscribe_to_mouse_events(
            self._mouse, self._on_mouse_event
        )

    def _on_mouse_event(self, event):
        """Обрабатывает события мыши."""
        # Проверяем, если кнопка мыши была отпущена.
        if event.type == carb.input.MouseEventType.BUTTON_RELEASE:
            # Сбрасываем все нажатые кнопки.
            for key_name in list(self.pressed_buttons):
                self.controller.set_key_state(key_name, False)
                self.pressed_buttons.remove(key_name)
            # Обновляем визуализацию кнопок.
            self.update_buttons()

    def _on_button_press(self, key_name: str):
        """Обработчик нажатия кнопки GUI."""
        self.controller.set_key_state(key_name, True)
        self.pressed_buttons.add(key_name)
        self.update_buttons()

    def build_ui(self):
        """Создает графический интерфейс."""
        with self.window.frame:
            with ui.VStack(spacing=5, alignment=ui.Alignment.CENTER):
                ui.Spacer(height=10)

                # Заголовок окна.
                ui.Label("Robot Control", alignment=ui.Alignment.CENTER, style={"font_size": 16, "font_weight": "bold"})
                ui.Spacer(height=10)

                # Кнопки движения.
                with ui.HStack(spacing=5):
                    ui.Spacer()
                    self.key_buttons["UP"] = ui.Button("UP", tooltip="Move Forward", style={"background_color": 0x666666ff})
                    self.key_buttons["UP"].set_clicked_fn(lambda: self._on_button_press("UP"))
                    ui.Spacer()

                with ui.HStack(spacing=5):
                    self.key_buttons["LEFT"] = ui.Button("LEFT", tooltip="Turn Left", style={"background_color": 0x666666ff})
                    self.key_buttons["LEFT"].set_clicked_fn(lambda: self._on_button_press("LEFT"))

                    self.key_buttons["DOWN"] = ui.Button("DOWN", tooltip="Move Backward", style={"background_color": 0x666666ff})
                    self.key_buttons["DOWN"].set_clicked_fn(lambda: self._on_button_press("DOWN"))

                    self.key_buttons["RIGHT"] = ui.Button("RIGHT", tooltip="Turn Right", style={"background_color": 0x666666ff})
                    self.key_buttons["RIGHT"].set_clicked_fn(lambda: self._on_button_press("RIGHT"))

                ui.Spacer(height=10)

                # Дополнительные кнопки.
                with ui.HStack(spacing=5):
                    ui.Spacer()
                    self.key_buttons["Q"] = ui.Button("Q", tooltip="Strafe Left", style={"background_color": 0x666666ff})
                    self.key_buttons["Q"].set_clicked_fn(lambda: self._on_button_press("Q"))

                    self.key_buttons["E"] = ui.Button("E", tooltip="Strafe Right", style={"background_color": 0x666666ff})
                    self.key_buttons["E"].set_clicked_fn(lambda: self._on_button_press("E"))
                    ui.Spacer()

                ui.Spacer(height=10)

                # Кнопки для специальных действий.
                with ui.HStack(spacing=10):
                    ui.Button("Camera (C)", tooltip="Toggle Camera", clicked_fn=lambda: self.controller.set_key_state("C", True))
                    ui.Button("Deselect (Esc)", tooltip="Deselect Robot", clicked_fn=lambda: self.controller.set_key_state("ESCAPE", True))

    def update_buttons(self):
        """Обновляет цвет кнопок в зависимости от состояния клавиш."""
        for key_name, is_pressed in self.controller.key_states.items():
            if key_name in self.key_buttons:
                if is_pressed:
                    self.key_buttons[key_name].style["background_color"] = 0x88ff88ff  # Зеленый цвет при нажатии
                else:
                    self.key_buttons[key_name].style["background_color"] = 0x666666ff  # Серый цвет при отпускании