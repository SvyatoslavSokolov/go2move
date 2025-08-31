import argparse
import os
import sys
import threading
import queue
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
import scripts.reinforcement_learning.rsl_rl.cli_args as cli_args  # isort: skip

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates an interactive demo with the H1 "
                "rough terrain environment."
)
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import carb
import omni
from isaacsim.core.utils.stage import get_current_stage
from omni.kit.viewport.utility import get_viewport_from_window_name
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf
from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import quat_apply
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

from isaaclab_tasks.manager_based.locomotion.velocity.config.h1.rough_env_cfg import \
    H1RoughEnvCfg_PLAY
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import \
    UnitreeGo2RoughEnvCfg_PLAY

TASK = "Isaac-Velocity-Rough-Unitree-Go2-v0"
RL_LIBRARY = "rsl_rl"

import omni.ui as ui
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import your updated controller classes
from controller import KeyboardController, KeyboardControllerGUI, ROS2KeyboardBridge


class UnitreeGo2RoughDemo:
    """
    Класс для управления роботом Unitree Go2.
    """

    def __init__(self, command_queue: queue.Queue):
        """
        Инициализирует демонстрацию.

        Args:
            command_queue: Очередь команд от клавиатуры.
        """
        agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(TASK, args_cli)
        checkpoint = get_published_pretrained_checkpoint(RL_LIBRARY, TASK)
        env_config = UnitreeGo2RoughEnvCfg_PLAY()
        env_config.scene.num_envs = 1
        env_config.episode_length_s = 1000000
        env_config.curriculum = None
        env_config.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        env_config.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        # env_config.commands.base_velocity.ranges.heading = (-1.0, 1.0)

        self.environment = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg=env_config))
        self.device = self.environment.unwrapped.device
        ppo_runner = OnPolicyRunner(self.environment, agent_cfg.to_dict(), log_dir=None,
                                    device=self.device)
        ppo_runner.load(checkpoint)
        self.policy = ppo_runner.get_inference_policy(device=self.device)

        self.create_camera()

        self.commands = torch.zeros(env_config.scene.num_envs, 4, device=self.device)
        self.commands[:, 0:3] = self.environment.unwrapped.command_manager.get_command(
            "base_velocity").clone()

        self._selected_object_prim = omni.usd.get_context().get_selection()
        self._selected_object_id = None
        self._previous_selected_object_id = None
        self._camera_local_transform = torch.tensor([-2.5, 0.0, 0.8], device=self.device)

        self.command_queue = command_queue

    def create_camera(self):
        """Создает камеру для обзора."""
        stage = get_current_stage()
        self.viewport = get_viewport_from_window_name("Viewport")
        self.camera_path = "/World/Camera"
        self.perspective_path = "/OmniverseKit_Persp"
        camera_prim = stage.DefinePrim(self.camera_path, "Camera")
        camera_prim.GetAttribute("focalLength").Set(8.5)
        coi_prop = camera_prim.GetProperty("omni:kit:centerOfInterest")
        if not coi_prop or not coi_prop.IsValid():
            camera_prim.CreateAttribute(
                "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
            ).Set(Gf.Vec3d(0, 0, -10))
        self.viewport.set_active_camera(self.perspective_path)

    def update_selected_object(self):
        """Обновляет информацию о выбранном объекте."""
        self._previous_selected_object_id = self._selected_object_id
        selected_prim_paths = self._selected_object_prim.get_selected_prim_paths()

        if len(selected_prim_paths) == 0:
            self._selected_object_id = None
            self.viewport.set_active_camera(self.perspective_path)
        elif len(selected_prim_paths) > 1:
            print("Multiple prims are selected. Please only select one!")
        else:
            prim_splitted_path = selected_prim_paths[0].split("/")
            if len(prim_splitted_path) >= 4 and prim_splitted_path[3][0:4] == "env_":
                self._selected_object_id = int(prim_splitted_path[3][4:])
                if self._previous_selected_object_id != self._selected_object_id:
                    self.viewport.set_active_camera(self.camera_path)
                    print(f"Selected robot: {self._selected_object_id}")
                self._update_camera()
            else:
                print("The selected prim was not a robot")

        if self._previous_selected_object_id is not None and \
           self._previous_selected_object_id != self._selected_object_id:
            self.commands[self._previous_selected_object_id] = torch.zeros(4, device=self.device)
            print(f"Reset commands for robot: {self._previous_selected_object_id}")

    def _update_camera(self):
        """Обновляет положение камеры в соответствии с выбранным роботом."""
        if self._selected_object_id is not None:
            base_pos = self.environment.unwrapped.scene["robot"].data.root_pos_w[
                self._selected_object_id, :]
            base_quat = self.environment.unwrapped.scene["robot"].data.root_quat_w[
                self._selected_object_id, :]
            camera_pos = quat_apply(base_quat, self._camera_local_transform) + base_pos
            camera_state = ViewportCameraState(self.camera_path, self.viewport)
            eye = Gf.Vec3d(camera_pos[0].item(), camera_pos[1].item(), camera_pos[2].item())
            target = Gf.Vec3d(base_pos[0].item(), base_pos[1].item(), base_pos[2].item() + 0.6)
            camera_state.set_position_world(eye, True)
            camera_state.set_target_world(target, True)


def keyboard_thread_func(command_queue: queue.Queue, device: torch.device):
    """
    Функция, которая выполняется в отдельном потоке для обработки ввода с клавиатуры
    и создания GUI.
    """
    controller = KeyboardController(command_queue, device)
    gui = KeyboardControllerGUI(controller)
    
    # Инициализация ROS 2 моста
    rclpy.init()
    ros_bridge = ROS2KeyboardBridge(command_queue, device)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_bridge)

    while simulation_app.is_running() and controller.is_running:
        # Обновляем GUI в каждом цикле
        gui._update_labels()
        executor.spin_once(timeout_sec=0.01)
        time.sleep(0.01)
    
    # Завершение работы
    controller.shutdown()
    ros_bridge.destroy_node()
    rclpy.shutdown()


def main():
    """Основная функция, выполняющая симуляцию."""
    command_queue = queue.Queue(maxsize=1)
    demo_go2 = UnitreeGo2RoughDemo(command_queue)
    obs, _ = demo_go2.environment.reset()

    keyboard_thread = threading.Thread(
        target=keyboard_thread_func, 
        args=(command_queue, demo_go2.device)
    )
    keyboard_thread.daemon = True
    keyboard_thread.start()

    while simulation_app.is_running():
        demo_go2.update_selected_object()

        with torch.inference_mode():
            try:
                new_commands = command_queue.get_nowait()
                if demo_go2._selected_object_id is not None:
                    demo_go2.commands[demo_go2._selected_object_id] = new_commands
            except queue.Empty:
                pass

            if demo_go2._selected_object_id is not None:
                obs[:, 9:13] = demo_go2.commands

            action = demo_go2.policy(obs)
            obs, _, _, _ = demo_go2.environment.step(action)


if __name__ == "__main__":
    main()
    simulation_app.close()