"""Optional Qt desktop view; drawing consumes the same episode snapshots.

This module is never imported by the headless engine or worker process.
"""
from __future__ import annotations

import json
import threading
from pathlib import Path


def _qt():
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError as exc:
        raise RuntimeError("Qt view requires: pip install -e '.[gui]'") from exc
    return QtCore, QtGui, QtWidgets


def _classes():
    QtCore, QtGui, QtWidgets = _qt()

    class CourseCanvas(QtWidgets.QWidget):
        def __init__(self, scenario):
            super().__init__()
            self.scenario = scenario
            self.frames = []
            self.sensor_points = []
            self.setMinimumSize(700, 470)

        def set_frames(self, frames):
            self.frames = frames
            self.update()

        def set_sensor_points(self, points):
            self.sensor_points = points
            self.update()

        def paintEvent(self, event):
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.fillRect(self.rect(), QtGui.QColor("#12161c"))
            s = self.scenario
            margin = 30
            scale = min((self.width()-2*margin)/s.width_m,
                        (self.height()-2*margin)/s.height_m)
            painter.translate((self.width()-s.width_m*scale)/2,
                              (self.height()+s.height_m*scale)/2)
            painter.scale(scale, -scale)
            painter.setPen(QtGui.QPen(QtGui.QColor("#7c8998"), 0.04))
            painter.setBrush(QtGui.QColor("#28323b"))
            painter.drawRect(QtCore.QRectF(0, 0, s.width_m, s.height_m))
            painter.setPen(QtGui.QPen(QtGui.QColor("#d7a348"), 0.04))
            painter.setBrush(QtGui.QColor("#77623d"))
            for ramp in s.ramps:
                painter.drawRect(QtCore.QRectF(ramp.x0_m, ramp.y0_m,
                                             ramp.x1_m-ramp.x0_m, ramp.y1_m-ramp.y0_m))
            painter.setBrush(QtGui.QColor("#bc5f5f"))
            for obstacle in s.obstacles:
                r = obstacle.radius_m
                painter.drawEllipse(QtCore.QRectF(obstacle.x_m-r, obstacle.y_m-r, 2*r, 2*r))
            painter.setPen(QtGui.QPen(QtGui.QColor("#6bc5a8"), 0.05))
            painter.setBrush(QtCore.Qt.NoBrush)
            for goal in s.goals:
                r = s.arrival_radius_m
                painter.drawEllipse(QtCore.QRectF(goal[0]-r, goal[1]-r, 2*r, 2*r))
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor("#71d9eb"))
            for x, y in self.sensor_points[::4]:
                painter.drawEllipse(QtCore.QRectF(x-0.025, y-0.025, 0.05, 0.05))
            if self.frames:
                path = QtGui.QPainterPath()
                path.moveTo(self.frames[0]["x_m"], self.frames[0]["y_m"])
                for frame in self.frames[1:]:
                    path.lineTo(frame["x_m"], frame["y_m"])
                painter.setPen(QtGui.QPen(QtGui.QColor("#8bbdf5"), 0.06))
                painter.drawPath(path)
                frame = self.frames[-1]
                painter.save()
                painter.translate(frame["x_m"], frame["y_m"])
                painter.rotate(frame["yaw_rad"]*180/3.141592653589793)
                painter.setPen(QtGui.QPen(QtGui.QColor("#eaf2fa"), 0.03))
                painter.setBrush(QtGui.QColor("#dce9f6"))
                painter.drawRect(QtCore.QRectF(-0.1, -0.21, 0.54, 0.42))
                painter.setPen(QtGui.QPen(QtGui.QColor("#ee7959"), 0.06))
                painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(0.6, 0))
                painter.restore()
            painter.end()

    class Bridge(QtCore.QObject):
        frame = QtCore.pyqtSignal(object)
        finished = QtCore.pyqtSignal(object, object)

    class MainWindow(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            from .cli import DEFAULT_ALGORITHM
            from .scenarios import load_scenario
            self.setWindowTitle("AutoNav Universe")
            self.scenario = load_scenario("straight")
            self.canvas = CourseCanvas(self.scenario)
            self.scenario_box = QtWidgets.QComboBox()
            self.scenario_box.addItems(["straight", "turns", "obstacle", "ramp",
                                        "ramp-forbidden", "combined", "gps-turn",
                                        "gps-blackout", "gps-bias", "gps-drift",
                                        "gps-combined",
                                        "planner-straight", "planner-turns",
                                        "planner-obstacle", "planner-ramp",
                                        "planner-ramp-forbidden", "planner-combined"])
            self.scenario_box.currentTextChanged.connect(self.change_scenario)
            self.algorithm_box = QtWidgets.QComboBox()
            self.algorithm_box.setEditable(True)
            self.algorithm_box.addItems([DEFAULT_ALGORITHM,
                "autonav_universe.algorithms:WaypointFollower",
                "autonav_universe.algorithms:CautiousFollower",
                "autonav_universe.algorithms:FixedPipelinePlanner",
                "autonav_universe.algorithms:GPSLidarNavigator",
                "autonav_universe.algorithms:GPSFollower"])
            self.seed_spin = QtWidgets.QSpinBox()
            self.seed_spin.setRange(0, 2**31-1)
            self.seed_spin.setValue(1)
            self.run_button = QtWidgets.QPushButton("Run")
            self.pause_button = QtWidgets.QPushButton("Pause")
            self.reset_button = QtWidgets.QPushButton("Reset")
            self.pause_button.setEnabled(False)
            self.run_button.clicked.connect(self.run_episode)
            self.pause_button.clicked.connect(self.toggle_pause)
            self.reset_button.clicked.connect(self.reset_episode)
            controls = QtWidgets.QHBoxLayout()
            for widget in (self.scenario_box, self.algorithm_box, self.seed_spin,
                           self.run_button, self.pause_button, self.reset_button):
                controls.addWidget(widget)
            self.status = QtWidgets.QLabel("Ready. Course and trail are truth-viewer data.")
            self.sensor_status = QtWidgets.QLabel("Sensors: unavailable until run")
            layout = QtWidgets.QVBoxLayout(self)
            layout.addLayout(controls)
            layout.addWidget(self.canvas)
            layout.addWidget(self.sensor_status)
            layout.addWidget(self.status)
            self.bridge = Bridge()
            self.bridge.frame.connect(self.on_frame)
            self.bridge.finished.connect(self.on_finished)
            self.thread = None
            self.stop_event = threading.Event()
            self.running_event = threading.Event()

        def change_scenario(self, name):
            if self.thread and self.thread.is_alive():
                return
            from .scenarios import load_scenario
            self.scenario = load_scenario(name)
            self.canvas.scenario = self.scenario
            self.canvas.set_frames([])
            self.canvas.set_sensor_points([])
            if self.scenario.track == "gps-only":
                self.algorithm_box.setCurrentText("autonav_universe.algorithms:GPSFollower")
            elif self.scenario.goal_frame == "gps":
                self.algorithm_box.setCurrentText("autonav_universe.algorithms:GPSLidarNavigator")
            elif self.scenario.track == "planner-controller":
                self.algorithm_box.setCurrentText("autonav_universe.algorithms:FixedPipelinePlanner")
            elif self.algorithm_box.currentText().endswith((":GPSFollower", ":FixedPipelinePlanner",
                                                             ":GPSLidarNavigator")):
                self.algorithm_box.setCurrentText("autonav_universe.algorithms:ReferenceNavigator")

        def run_episode(self):
            if self.thread and self.thread.is_alive():
                return
            from .runner import EpisodeRunner
            scenario = self.scenario
            algorithm = self.algorithm_box.currentText().strip()
            seed = self.seed_spin.value()
            self.canvas.set_frames([])
            self.canvas.set_sensor_points([])
            self.stop_event = threading.Event()
            self.running_event = threading.Event()
            self.running_event.set()
            self.run_button.setEnabled(False)
            self.pause_button.setEnabled(True)
            self.status.setText("Running")

            def work():
                try:
                    runner = EpisodeRunner(scenario, algorithm, seed)
                    def frame_callback(current):
                        sensors = []
                        for kind, sample in sorted(current.samples.items()):
                            if kind == "gps" and sample.valid:
                                value = (f"{sample.data.latitude_deg:.6f},"
                                         f"{sample.data.longitude_deg:.6f}")
                            elif kind == "odometry" and sample.valid:
                                value = f"({sample.data[0]:.2f}, {sample.data[1]:.2f}) m"
                            elif kind == "lidar" and sample.valid:
                                value = f"{len(sample.data)} points"
                            else:
                                value = "missing"
                            sensors.append(f"{kind}: {value}")
                        points = []
                        lidar = current.samples.get("lidar")
                        if lidar is not None and lidar.valid:
                            import math
                            c, s = math.cos(current.state.yaw_rad), math.sin(current.state.yaw_rad)
                            points = [(current.state.x_m+c*x-s*y,
                                       current.state.y_m+s*x+c*y)
                                      for x, y, _ in lidar.data]
                        self.bridge.frame.emit((current.truth[-1].copy(), ";  ".join(sensors), points))
                    result = runner.run(frame_callback, self.running_event, self.stop_event, True)
                    self.bridge.finished.emit(result, None)
                except Exception as exc:
                    self.bridge.finished.emit(None, str(exc))
            self.thread = threading.Thread(target=work, daemon=True)
            self.thread.start()

        def on_frame(self, payload):
            frame, sensors, points = payload
            self.canvas.set_frames(self.canvas.frames + [frame])
            self.canvas.set_sensor_points(points)
            self.sensor_status.setText("Sensors: " + sensors)
            self.status.setText(f"t={frame['time_s']:.1f}s  goal={frame['goal_index']+1}")

        def on_finished(self, result, error):
            self.run_button.setEnabled(True)
            self.pause_button.setEnabled(False)
            if error:
                self.status.setText("Error: " + error)
            else:
                self.status.setText(f"{result.outcome}  {result.time_s:.2f}s  "
                                    f"{result.goals_reached}/{result.total_goals} goals")

        def toggle_pause(self):
            if self.running_event.is_set():
                self.running_event.clear()
                self.pause_button.setText("Resume")
            else:
                self.running_event.set()
                self.pause_button.setText("Pause")

        def reset_episode(self):
            self.stop_event.set()
            self.running_event.set()
            self.canvas.set_frames([])
            self.canvas.set_sensor_points([])
            self.status.setText("Reset")

        def closeEvent(self, event):
            self.stop_event.set()
            self.running_event.set()
            if self.thread is not None:
                self.thread.join(timeout=2.5)
            super().closeEvent(event)

    return QtCore, QtGui, QtWidgets, CourseCanvas, MainWindow


def launch_app() -> None:
    QtCore, QtGui, QtWidgets, CourseCanvas, MainWindow = _classes()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()


def show_replay(folder: Path, scenario) -> None:
    QtCore, QtGui, QtWidgets, CourseCanvas, MainWindow = _classes()
    frames = json.loads((folder / "truth_replay.json").read_text(encoding="utf-8"))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtWidgets.QWidget()
    window.setWindowTitle("AutoNav Universe replay — recorded truth")
    layout = QtWidgets.QVBoxLayout(window)
    canvas = CourseCanvas(scenario)
    label = QtWidgets.QLabel("Recorded episode")
    layout.addWidget(canvas)
    layout.addWidget(label)
    position = [0]
    timer = QtCore.QTimer(window)

    def tick():
        if position[0] >= len(frames):
            timer.stop()
            return
        canvas.set_frames(frames[:position[0]+1])
        label.setText(f"Recorded truth at {frames[position[0]]['time_s']:.1f}s")
        position[0] += 1
    timer.timeout.connect(tick)
    timer.start(100)
    window.show()
    app.exec_()
