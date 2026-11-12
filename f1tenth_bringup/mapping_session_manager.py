#!/usr/bin/env python3

import glob
import os
import re
import shutil
import signal
import subprocess
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String


class MappingSessionManager(Node):
    def __init__(self) -> None:
        super().__init__('mapping_session_manager')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('save_button_index', 3)
        self.declare_parameter('reset_button_index', 0)
        self.declare_parameter(
            'maps_directory',
            '/home/nvidia/f1tenth_ws/src/f1tenth_bringup/maps',
        )
        self.declare_parameter('map_basename', 'racetrack')
        self.declare_parameter(
            'slam_params_file',
            '/home/nvidia/f1tenth_ws/src/f1tenth_bringup/config/slam_toolbox.yaml',
        )
        self.declare_parameter('slam_executable', 'async_slam_toolbox_node')
        self.declare_parameter('slam_node_name', 'slam_toolbox')
        self.declare_parameter('slam_log_level', 'warn')
        self.declare_parameter('slam_startup_timeout_sec', 10.0)
        self.declare_parameter('slam_map_frame', 'map')
        self.declare_parameter('slam_base_frame', 'base_link')
        self.declare_parameter('slam_odom_frame', 'odom')
        self.declare_parameter('slam_transform_publish_period', 0.02)
        self.declare_parameter('slam_use_sim_time', False)

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.save_button_index = int(self.get_parameter('save_button_index').value)
        self.reset_button_index = int(self.get_parameter('reset_button_index').value)
        self.maps_directory = Path(str(self.get_parameter('maps_directory').value))
        self.map_basename = str(self.get_parameter('map_basename').value)
        self.slam_params_file = str(self.get_parameter('slam_params_file').value)
        self.slam_executable = str(self.get_parameter('slam_executable').value)
        self.slam_node_name = str(self.get_parameter('slam_node_name').value)
        self.slam_log_level = str(self.get_parameter('slam_log_level').value)
        self.slam_startup_timeout_sec = max(
            1.0, float(self.get_parameter('slam_startup_timeout_sec').value)
        )
        self.slam_map_frame = str(self.get_parameter('slam_map_frame').value)
        self.slam_base_frame = str(self.get_parameter('slam_base_frame').value)
        self.slam_odom_frame = str(self.get_parameter('slam_odom_frame').value)
        self.slam_transform_publish_period = float(
            self.get_parameter('slam_transform_publish_period').value
        )
        self.slam_use_sim_time = bool(self.get_parameter('slam_use_sim_time').value)

        self.last_save_pressed = False
        self.last_reset_pressed = False
        self.save_in_progress = False
        self._slam_process: Optional[subprocess.Popen] = None
        self._shutdown_requested = False

        self.maps_directory.mkdir(parents=True, exist_ok=True)
        self.save_map_client = self.create_client(SaveMap, 'slam_toolbox/save_map')
        self.create_subscription(Joy, self.joy_topic, self.handle_joy, 20)
        self.monitor_timer = self.create_timer(1.0, self.monitor_slam_process)

        self.start_slam_process()

        self.get_logger().info(
            'Mapping session manager ready. Save button=%d, reset button=%d, '
            'target map=%s'
            % (
                self.save_button_index,
                self.reset_button_index,
                str(self.maps_directory / self.map_basename),
            )
        )

    def handle_joy(self, msg: Joy) -> None:
        save_pressed = self._button_pressed(msg, self.save_button_index)
        if save_pressed and not self.last_save_pressed:
            self.request_save()
        self.last_save_pressed = save_pressed

        reset_pressed = self._button_pressed(msg, self.reset_button_index)
        if reset_pressed and not self.last_reset_pressed:
            self.reset_mapping_session()
        self.last_reset_pressed = reset_pressed

    def request_save(self) -> None:
        if self.save_in_progress:
            self.get_logger().warn('Map save already in progress; ignoring duplicate button press.')
            return

        if not self.save_map_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                'slam_toolbox/save_map is not available yet; cannot save racetrack right now.'
            )
            return

        self.save_in_progress = True
        temp_base = self.maps_directory / (self.map_basename + '_tmp')
        self._remove_matching_files(temp_base)

        request = SaveMap.Request()
        request.name = String(data=str(temp_base))
        future = self.save_map_client.call_async(request)
        future.add_done_callback(self._handle_save_response)
        self.get_logger().info('Saving current map to %s' % str(temp_base))

    def _handle_save_response(self, future) -> None:
        self.save_in_progress = False
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error('Map save service call failed: %s' % str(exc))
            return

        temp_base = self.maps_directory / (self.map_basename + '_tmp')
        temp_files = self._matching_files(temp_base)
        if not temp_files:
            self.get_logger().error(
                'slam_toolbox/save_map returned, but no map files were created at %s'
                % str(temp_base)
            )
            return

        final_base = self.maps_directory / self.map_basename
        final_paths = []
        for temp_path in temp_files:
            suffix = ''.join(temp_path.suffixes)
            final_path = final_base.with_name(final_base.name + suffix)
            if final_path.exists():
                final_path.unlink()
            final_paths.append(final_path)

        self._remove_existing_map_files(exempt_paths=temp_files)

        for temp_path, final_path in zip(temp_files, final_paths):
            shutil.move(str(temp_path), str(final_path))

        self._rewrite_yaml_image_reference(final_base)
        self._remove_nonfinal_map_files(final_paths)
        saved_assets = ', '.join(path.name for path in final_paths)
        self.get_logger().info('Saved racetrack map assets: %s' % saved_assets)

    def reset_mapping_session(self) -> None:
        if self.save_in_progress:
            self.get_logger().warn('Reset requested during map save; ignoring reset button press.')
            return

        self.get_logger().info('Resetting mapping session: deleting saved maps and restarting SLAM.')
        self.stop_slam_process()
        self._remove_existing_map_files(exempt_paths=[])
        self.start_slam_process()

    def monitor_slam_process(self) -> None:
        if self._shutdown_requested or self._slam_process is None:
            return

        return_code = self._slam_process.poll()
        if return_code is None:
            return

        self.get_logger().warn(
            'slam_toolbox exited with code %d; restarting it automatically.' % return_code
        )
        self._slam_process = None
        self.start_slam_process()

    def start_slam_process(self) -> None:
        if self._slam_process is not None and self._slam_process.poll() is None:
            return

        command = [
            'ros2',
            'run',
            'slam_toolbox',
            self.slam_executable,
            '--ros-args',
            '--params-file',
            self.slam_params_file,
            '-r',
            '__node:=%s' % self.slam_node_name,
            '-p',
            'map_frame:=%s' % self.slam_map_frame,
            '-p',
            'base_frame:=%s' % self.slam_base_frame,
            '-p',
            'odom_frame:=%s' % self.slam_odom_frame,
            '-p',
            'transform_publish_period:=%s' % str(self.slam_transform_publish_period),
            '-p',
            'use_sim_time:=%s' % ('true' if self.slam_use_sim_time else 'false'),
            '--log-level',
            self.slam_log_level,
        ]

        self._slam_process = subprocess.Popen(
            command,
            start_new_session=True,
        )
        self.get_logger().info(
            'Started slam_toolbox process pid=%d with params %s'
            % (self._slam_process.pid, self.slam_params_file)
        )

    def stop_slam_process(self) -> None:
        process = self._slam_process
        if process is None:
            return

        self._slam_process = None
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('slam_toolbox did not stop cleanly; sending SIGKILL.')
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass

    def _remove_existing_map_files(self, exempt_paths: List[Path]) -> None:
        exempt_resolved = {path.resolve() for path in exempt_paths if path.exists()}
        for path in self.maps_directory.iterdir():
            if not path.is_file():
                continue
            if path.name.upper().startswith('README'):
                continue
            if path.resolve() in exempt_resolved:
                continue
            try:
                path.unlink()
            except OSError as exc:
                self.get_logger().warn('Failed to remove map file %s: %s' % (path, str(exc)))

    def _remove_nonfinal_map_files(self, final_paths: List[Path]) -> None:
        keep = {path.resolve() for path in final_paths if path.exists()}
        for path in self.maps_directory.iterdir():
            if not path.is_file():
                continue
            if path.name.upper().startswith('README'):
                continue
            if path.resolve() in keep:
                continue
            try:
                path.unlink()
            except OSError as exc:
                self.get_logger().warn(
                    'Failed to remove extra map file %s after save: %s' % (path, str(exc))
                )

    @staticmethod
    def _matching_files(base_path: Path) -> List[Path]:
        return sorted(Path(path) for path in glob.glob(str(base_path) + '.*'))

    def _remove_matching_files(self, base_path: Path) -> None:
        for path in self._matching_files(base_path):
            try:
                path.unlink()
            except OSError as exc:
                self.get_logger().warn('Failed to remove temporary map file %s: %s' % (path, str(exc)))

    def _rewrite_yaml_image_reference(self, final_base: Path) -> None:
        yaml_path = final_base.with_suffix('.yaml')
        pgm_name = final_base.with_suffix('.pgm').name
        if not yaml_path.exists():
            return

        try:
            content = yaml_path.read_text(encoding='utf-8')
            updated = re.sub(r'^image:\s*.+$', 'image: %s' % pgm_name, content, flags=re.MULTILINE)
            if updated != content:
                yaml_path.write_text(updated, encoding='utf-8')
        except OSError as exc:
            self.get_logger().warn(
                'Failed to rewrite image reference in %s: %s' % (str(yaml_path), str(exc))
            )

    @staticmethod
    def _button_pressed(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and msg.buttons[index] != 0

    def destroy_node(self) -> bool:
        self._shutdown_requested = True
        self.stop_slam_process()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingSessionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
