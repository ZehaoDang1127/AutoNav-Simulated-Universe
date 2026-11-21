"""Scope the pinned harness's legacy AUTO output away from robot controls.

Load the installed launch description without changing the persistent ROS
workspace. Only the harness is remapped; the steady-clock command gate owns
the robot's /autonomous_mode throughout a desktop session.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import SetRemap


def generate_launch_description():
    path = Path(get_package_share_directory("igvc_competition_sim")) / "launch/igvc_competition.launch.py"
    spec = importlib.util.spec_from_file_location("pinned_course_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    harness = module._harness_process

    def supervised_harness(context, *args, **kwargs):
        return [GroupAction([
            SetRemap(src="/autonomous_mode", dst="/igvc_sim/harness_autonomous_mode"),
            *harness(context, *args, **kwargs),
        ])]

    module._harness_process = supervised_harness
    return module.generate_launch_description()
