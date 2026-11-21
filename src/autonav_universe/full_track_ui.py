"""Qt workspace for the exported course and its real ROS/Gazebo session."""
from __future__ import annotations

import base64
import math
from pathlib import Path
import threading

from .full_track import FullTrackSession, load_bundle, read_json, repository_root
from .ui import _qt


def _classes():
    QtCore, QtGui, QtWidgets = _qt()

    class CourseView(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.course, self.live = {}, {}
            self.setMinimumSize(580, 420)

        def paintEvent(self, event):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            p.fillRect(self.rect(), QtGui.QColor("#12212b"))
            if not self.course:
                p.setPen(QtGui.QColor("#b0c4ce"))
                p.drawText(self.rect(), QtCore.Qt.AlignCenter, "Select an exported course bundle to preview the track")
                return
            course = self.course
            lines = course.get("internal_lines", []) + course.get("dashed_lines", [])
            points = [course.get("start", {}), course.get("finish", {})]
            points += course.get("mission_waypoints", []) + course.get("obstacles", [])
            points += [point for line in lines for point in (line["start"], line["end"])]
            xs = [v.get("x_m", 0) for v in points]
            ys = [v.get("y_m", 0) for v in points]
            x0, x1, y0, y1 = min(xs)-2, max(xs)+2, min(ys)-2, max(ys)+2
            scale = min((self.width()-48)/(x1-x0), (self.height()-48)/(y1-y0))
            left, top = (self.width()-(x1-x0)*scale)/2, (self.height()-(y1-y0)*scale)/2

            def point(x, y):
                return QtCore.QPointF(left+(x-x0)*scale, top+(y1-y)*scale)

            def pen(color, width=1):
                p.setPen(QtGui.QPen(QtGui.QColor(color), width))

            pen("#1d3440")
            for x in range(math.ceil(x0), math.ceil(x1), 2):
                p.drawLine(point(x, y0), point(x, y1))
            for y in range(math.ceil(y0), math.ceil(y1), 2):
                p.drawLine(point(x0, y), point(x1, y))
            # The course world has an asphalt ground; distinguish the authored
            # two-lane section in the schematic preview.
            for surface in course.get("diagnostic_road_surfaces", []):
                corners = surface.get("corners_m", [])
                if len(corners) == 4:
                    p.setPen(QtCore.Qt.NoPen)
                    p.setBrush(QtGui.QColor("#45545a"))
                    p.drawPolygon(QtGui.QPolygonF([point(*corner) for corner in corners]))
            p.setBrush(QtCore.Qt.NoBrush)
            for ramp in course.get("ramps", []):
                cx, cy = ramp["center_x_m"], ramp["center_y_m"]
                c, s = math.cos(ramp["yaw_rad"]), math.sin(ramp["yaw_rad"])
                length, width = ramp["run_length_m"]/2, ramp["width_m"]/2
                polygon = QtGui.QPolygonF([point(cx+c*x-s*y, cy+s*x+c*y)
                                          for x, y in ((-length,-width),(length,-width),
                                                       (length,width),(-length,width))])
                pen("#d79a61")
                p.setBrush(QtGui.QColor("#5e4935"))
                p.drawPolygon(polygon)
            p.setBrush(QtCore.Qt.NoBrush)
            for line in course.get("internal_lines", []):
                pen("#e4e8df", max(1, line.get("width_m", 0.0762)*scale))
                a, b = line["start"], line["end"]
                p.drawLine(point(a["x_m"], a["y_m"]), point(b["x_m"], b["y_m"]))
            for line in course.get("dashed_lines", []):
                pen("#e4e8df", max(1, line.get("width_m", 0.0762)*scale))
                a, b = line["start"], line["end"]
                dx, dy = b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]
                length = math.hypot(dx, dy)
                dash, gap = line.get("dash_length_m", .75), line.get("gap_length_m", .45)
                if length <= 0 or dash <= 0 or gap < 0:
                    continue
                offset = 0.0
                while offset < length:
                    end = min(offset + dash, length)
                    p.drawLine(point(a["x_m"] + dx*offset/length, a["y_m"] + dy*offset/length),
                               point(a["x_m"] + dx*end/length, a["y_m"] + dy*end/length))
                    offset += dash + gap
            for obstacle in course.get("obstacles", []):
                pen("#eb9872")
                p.setBrush(QtGui.QColor("#9c5a42"))
                radius = obstacle.get("radius_m", 0.3)*scale
                p.drawEllipse(point(obstacle["x_m"], obstacle["y_m"]), radius, radius)
            score = self.live.get("score") or {}
            reached = score.get("waypoints_reached_count", 0)
            for i, goal in enumerate(course.get("mission_waypoints", [])):
                pen("#68d9aa" if i < reached else "#77a6d8", 1.5)
                p.setBrush(QtGui.QColor("#234c43" if i < reached else "#20374d"))
                center = point(goal["x_m"], goal["y_m"])
                p.drawEllipse(center, 12, 12)
                p.setPen(QtGui.QColor("#ffffff"))
                p.drawText(QtCore.QRectF(center.x()-12, center.y()-12, 24, 24),
                           QtCore.Qt.AlignCenter, str(i+1))

            def path(points, color):
                if not points:
                    return
                drawing = QtGui.QPainterPath(point(*points[0]))
                for item in points[1:]:
                    drawing.lineTo(point(*item))
                pen(color, 2)
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawPath(drawing)

            path(self.live.get("trail", []), "#52d0df")
            frame = self.live.get("plan_frame", "map")
            transform = self.live.get(frame + "_to_world")
            if transform:
                tx, ty, yaw = transform
                c, s = math.cos(yaw), math.sin(yaw)
                path([(tx+c*x-s*y, ty+s*x+c*y) for x, y in self.live.get("plan", [])], "#ecd47b")
            pose = self.live.get("pose")
            start = course.get("start", {})
            x, y, yaw = ((pose["x"], pose["y"], pose["yaw"]) if pose else
                         (start.get("x_m",0), start.get("y_m",0), start.get("yaw_rad",0)))
            c, s = math.cos(yaw), math.sin(yaw)
            robot = course.get("robot", {})
            offset = robot.get("base_link_to_nav_center_m", .225)
            half_length = robot.get("physical_half_length_m", .545)
            half_width = robot.get("physical_half_width_m", .41)
            corners = [(x+c*a-s*b, y+s*a+c*b) for a,b in (
                (offset-half_length, -half_width), (offset+half_length, -half_width),
                (offset+half_length, half_width), (offset-half_length, half_width))]
            pen("#a4f5d0", 1.5)
            p.setBrush(QtGui.QColor("#3c9474"))
            p.drawPolygon(QtGui.QPolygonF([point(*item) for item in corners]))
            p.drawLine(point(x,y), point(x+1.1*c,y+1.1*s))
            pen("#91a9b7")
            p.drawText(16, self.height()-14, "2 m grid   ·   course + pose: evaluator truth   ·   yellow: Nav2 plan")

    class Bridge(QtCore.QObject):
        changed = QtCore.pyqtSignal(object)
        finished = QtCore.pyqtSignal()

    class FullCourseWindow(QtWidgets.QMainWindow):
        def __init__(self, repo=None, bundle=None):
            super().__init__()
            self.repo = Path(repo or repository_root()).resolve()
            self.session, self.worker = None, None
            self.reset_pending, self.closing = False, False
            self.run_dir = None
            self.setWindowTitle("AutoNav Universe · Full course")
            self.resize(1320, 930)
            self.setStyleSheet("""
                QWidget { background: #0c1821; color: #dce7ed; font-family: 'Segoe UI'; font-size: 13px; }
                QGroupBox { border: 1px solid #2b414e; border-radius: 8px; margin-top: 14px; padding: 12px; }
                QGroupBox::title { subcontrol-origin: margin; left: 12px; color: #9db5c3; }
                QPushButton { background: #203541; border: 1px solid #385363; border-radius: 5px; padding: 9px 15px; }
                QPushButton:hover { background: #2d4a59; }
                QPushButton:disabled { color: #607783; border-color: #253a46; }
                QPushButton#run { background: #24705a; border-color: #45ac87; color: white; }
                QPushButton#run:disabled { background: #203541; border-color: #253a46; color: #607783; }
                QLineEdit, QSpinBox, QPlainTextEdit, QTableWidget { background: #12232e; border: 1px solid #2b414e; padding: 4px; }
                QHeaderView::section { background: #1d3340; padding: 6px; border: 0; }
                QTabBar::tab { background: #162a36; padding: 10px 18px; }
                QTabBar::tab:selected { background: #284552; color: #8ce8c3; }
                QProgressBar { border: 1px solid #304a59; border-radius: 4px; text-align: center; }
                QProgressBar::chunk { background: #32886b; }
            """)
            root = QtWidgets.QWidget()
            self.setCentralWidget(root)
            layout = QtWidgets.QVBoxLayout(root)
            layout.setContentsMargins(20, 16, 20, 16)
            heading = QtWidgets.QHBoxLayout()
            title = QtWidgets.QLabel("AutoNav Universe   /   Full course")
            title.setStyleSheet("font-size: 23px; font-weight: 600;")
            self.state_label = QtWidgets.QLabel("Idle")
            self.state_label.setStyleSheet("color: #8ce8c3; font-size: 17px;")
            heading.addWidget(title)
            heading.addStretch()
            heading.addWidget(self.state_label)
            layout.addLayout(heading)
            subtitle = QtWidgets.QLabel("Local Blender course · ROS 2 Humble / Gazebo Fortress · baseline robot stack")
            subtitle.setStyleSheet("color: #90aaba; padding-bottom: 8px;")
            layout.addWidget(subtitle)
            tabs = QtWidgets.QTabWidget()
            layout.addWidget(tabs)
            course_page = QtWidgets.QWidget()
            tabs.addTab(course_page, "Full course")
            page = QtWidgets.QVBoxLayout(course_page)
            controls = QtWidgets.QHBoxLayout()
            self.run_button = QtWidgets.QPushButton("Run course")
            self.run_button.setObjectName("run")
            self.pause_button = QtWidgets.QPushButton("Pause")
            self.stop_button = QtWidgets.QPushButton("Stop")
            self.reset_button = QtWidgets.QPushButton("Reset")
            self.replay_button = QtWidgets.QPushButton("Open replay")
            for button in (self.run_button, self.pause_button, self.stop_button, self.reset_button, self.replay_button):
                controls.addWidget(button)
            controls.addStretch()
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 5)
            self.progress.setValue(0)
            self.progress.setFormat("%v / %m waypoints")
            controls.addWidget(self.progress)
            page.addLayout(controls)
            metrics = QtWidgets.QHBoxLayout()
            self.metrics = {}
            for key, title in (("goals","WAYPOINTS"),("time","SIMULATION TIME"),
                               ("speed","MEASURED SPEED"),("distance","DISTANCE")):
                group = QtWidgets.QGroupBox(title)
                box = QtWidgets.QVBoxLayout(group)
                value = QtWidgets.QLabel("—")
                value.setStyleSheet("font-size: 23px; color: #ecf5fa;")
                box.addWidget(value)
                metrics.addWidget(group)
                self.metrics[key] = value
            page.addLayout(metrics)
            split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            self.canvas = CourseView()
            split.addWidget(self.canvas)
            side = QtWidgets.QWidget()
            side.setMaximumWidth(390)
            side_layout = QtWidgets.QVBoxLayout(side)
            config = QtWidgets.QGroupBox("Session")
            form = QtWidgets.QFormLayout(config)
            self.bundle_box = QtWidgets.QLineEdit(str(bundle or self.repo / "runs/full-track-integrated-lanes"))
            browse = QtWidgets.QPushButton("…")
            browse.setToolTip("Choose an exported course bundle")
            browse.setFixedWidth(34)
            browse.clicked.connect(self.choose_bundle)
            bundle_row = QtWidgets.QWidget()
            bundle_layout = QtWidgets.QHBoxLayout(bundle_row)
            bundle_layout.setContentsMargins(0, 0, 0, 0)
            bundle_layout.addWidget(self.bundle_box, 1)
            bundle_layout.addWidget(browse)
            self.timeout = QtWidgets.QSpinBox()
            self.timeout.setRange(30, 7200)
            self.timeout.setValue(1800)
            self.timeout.setSuffix(" s active wall time")
            form.addRow("Course bundle", bundle_row)
            form.addRow("Time budget", self.timeout)
            self.costmap_version = QtWidgets.QComboBox()
            self.costmap_version.addItem("v2 · lane-aware (requires rebuilt workspace)", "2")
            self.costmap_version.addItem("v1 · previous baseline", "1")
            self.lane_change_cost = QtWidgets.QDoubleSpinBox()
            self.lane_change_cost.setRange(0, 100)
            self.lane_change_cost.setValue(.25)
            self.lane_change_cost.setSuffix(" m / change")
            form.addRow("Cost map", self.costmap_version)
            form.addRow("Lane penalty", self.lane_change_cost)
            self.costmap_version.currentIndexChanged.connect(
                lambda: self.lane_change_cost.setEnabled(self.costmap_version.currentData() == "2"))
            side_layout.addWidget(config)
            self.readiness = QtWidgets.QLabel("Run checks sensors, transforms, Nav2 and the motion gate before dispatch.")
            self.readiness.setWordWrap(True)
            side_layout.addWidget(self.readiness)
            self.sensors = QtWidgets.QTableWidget(0, 2)
            self.sensors.setHorizontalHeaderLabels(["Sensor", "Status"])
            self.sensors.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            self.sensors.verticalHeader().hide()
            self.sensors.verticalHeader().setDefaultSectionSize(24)
            self.sensors.setFixedHeight(180)
            self.sensors.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            side_layout.addWidget(self.sensors)
            self.audit = QtWidgets.QPlainTextEdit()
            self.audit.setReadOnly(True)
            side_layout.addWidget(self.audit)
            split.addWidget(side)
            split.setStretchFactor(0, 1)
            page.addWidget(split, 1)
            details = QtWidgets.QTabWidget()
            details.setMaximumHeight(150)
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setPlaceholderText("Runtime output appears here after the simulator starts.")
            details.addTab(self.log, "Runtime log")
            self.costmap = QtWidgets.QLabel("Estimated costmap unavailable until the ROS topic publishes.")
            self.costmap.setAlignment(QtCore.Qt.AlignCenter)
            details.addTab(self.costmap, "Estimated local costmap")
            self.camera = QtWidgets.QLabel("Camera unavailable until an image is received.")
            self.camera.setAlignment(QtCore.Qt.AlignCenter)
            details.addTab(self.camera, "Camera")
            page.addWidget(details)
            self.message = QtWidgets.QLabel("Ready to preview the selected course.")
            self.message.setWordWrap(True)
            page.addWidget(self.message)
            self.results = QtWidgets.QTableWidget(0, 6)
            self.results.setHorizontalHeaderLabels(["Run", "Cost map", "Outcome", "Waypoints", "Finish", "Stop gate"])
            self.results.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            self.results.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.results.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            self.results.cellDoubleClicked.connect(self.open_result)
            tabs.addTab(self.results, "Saved results · double-click for replay")
            from .ui import _classes as python_classes
            self.python_lab = python_classes()[4]()
            tabs.addTab(self.python_lab, "Python scenario lab")
            from .self_drive_ui import _classes as self_drive_classes
            self.self_drive_lab = self_drive_classes()[1]()
            tabs.addTab(self.self_drive_lab, "Self-drive track")
            self.bridge = Bridge()
            self.bridge.changed.connect(self.on_snapshot)
            self.bridge.finished.connect(self.on_finished)
            self.run_button.clicked.connect(self.start_session)
            self.pause_button.clicked.connect(self.pause_session)
            self.stop_button.clicked.connect(self.stop_session)
            self.reset_button.clicked.connect(self.reset_session)
            self.replay_button.clicked.connect(self.open_replay)
            self.bundle_box.editingFinished.connect(self.preview)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh_log)
            self.timer.start(1500)
            self.preview()
            self.refresh_results()
            self.set_controls(False)

        def set_controls(self, active):
            self.run_button.setEnabled(not active and bool(self.canvas.course))
            acknowledged = (self.session and
                            ((self.session.desired == "running" and self.state_label.text() == "Running") or
                             (self.session.desired == "paused" and self.state_label.text() == "Paused")))
            self.pause_button.setEnabled(bool(active and acknowledged))
            self.stop_button.setEnabled(active)
            self.bundle_box.setEnabled(not active)
            self.timeout.setEnabled(not active)
            self.costmap_version.setEnabled(not active)
            self.lane_change_cost.setEnabled(not active and self.costmap_version.currentData() == "2")
            self.replay_button.setEnabled(bool(self.run_dir and (self.run_dir / "replay.html").is_file()))

        def preview(self):
            try:
                course, manifest = load_bundle(self.repo, Path(self.bundle_box.text()))
                self.canvas.course = course
                gaps = manifest.get("semantic_gaps", [])
                self.audit.setPlainText(("DIAGNOSTIC COURSE\n\n" if gaps else "ASSET CANDIDATE\n\n") +
                    "Source, course and world hashes verified.\n\n" + "\n\n".join(gaps))
                self.message.setText(f"{course['course_id']} · {len(course.get('mission_waypoints', []))} ordered goals · "
                                     f"{len(course.get('obstacles', []))} obstacles · {len(course.get('ramps', []))} ramp(s)")
            except (OSError, ValueError, ImportError) as exc:
                self.canvas.course = {}
                self.audit.setPlainText(str(exc))
                self.message.setText("Course preview unavailable. See the bundle diagnostic.")
            self.canvas.update()
            self.set_controls(bool(self.worker and self.worker.is_alive()))

        def choose_bundle(self):
            if self.worker and self.worker.is_alive():
                return
            folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Exported course bundle", self.bundle_box.text())
            if folder:
                self.bundle_box.setText(folder)
                self.preview()

        def start_session(self):
            if self.worker and self.worker.is_alive():
                return
            self.clear_live_view()
            self.readiness.setText("Waiting for readiness from the new ROS/Gazebo session.")
            self.session = FullTrackSession(self.repo, Path(self.bundle_box.text()), self.timeout.value(),
                                            costmap_version=self.costmap_version.currentData(),
                                            lane_change_cost_m=self.lane_change_cost.value())
            self.run_dir = self.session.run_dir
            self.state_label.setText("Starting")
            self.pause_button.setText("Pause")
            self.set_controls(True)
            session = self.session

            def work():
                try:
                    session.run(self.bridge.changed.emit)
                finally:
                    self.bridge.finished.emit()
            self.worker = threading.Thread(target=work, daemon=True)
            self.worker.start()

        def pause_session(self):
            if self.session:
                paused = self.session.desired == "paused"
                self.session.request("running" if paused else "paused")
                self.pause_button.setEnabled(False)
                self.message.setText("Resuming Gazebo…" if paused else "Inhibiting motion and pausing Gazebo…")

        def stop_session(self):
            if self.session:
                self.session.request("stopped")
                self.pause_button.setEnabled(False)
                self.message.setText("Stopping navigation and saving the result bundle…")

        def reset_session(self):
            if self.worker and self.worker.is_alive():
                self.reset_pending = True
                self.stop_session()
                return
            self.session = None
            self.clear_live_view()
            self.state_label.setText("Idle")
            self.state_label.setStyleSheet("color: #8ce8c3; font-size: 17px;")
            self.pause_button.setText("Pause")
            self.readiness.setText("Next Run starts a fresh simulator, localization, maps, mission and referee.")
            self.preview()

        def clear_live_view(self):
            self.canvas.live = {}
            self.canvas.update()
            self.progress.setValue(0)
            for label in self.metrics.values():
                label.setText("—")
            self.sensors.setRowCount(0)
            self.costmap.setText("Estimated costmap unavailable until the ROS topic publishes.")
            self.camera.setText("Camera unavailable until an image is received.")

        def on_snapshot(self, snapshot):
            state = snapshot.get("state", "Starting")
            self.state_label.setText(state)
            color = "#ed9b8e" if state == "Failed" else "#e2c77c" if state == "Paused" else "#8ce8c3"
            self.state_label.setStyleSheet(f"color: {color}; font-size: 17px;")
            self.message.setText(snapshot.get("message", ""))
            self.pause_button.setText("Resume" if state == "Paused" else "Pause")
            readiness = snapshot.get("readiness", {})
            if readiness:
                self.readiness.setText("Readiness passed · sensors / TF / Nav2 / gate" if readiness.get("ready")
                                       else "Readiness: " + ", ".join(readiness.get("missing", [])))
            telemetry = snapshot.get("telemetry")
            if telemetry:
                self.canvas.live = telemetry
                self.canvas.update()
                score, pose = telemetry.get("score") or {}, telemetry.get("pose") or {}
                count, total = score.get("waypoints_reached_count", 0), score.get("waypoints_total", len(self.canvas.course.get("mission_waypoints", [])))
                self.metrics["goals"].setText(f"{count} / {total}")
                self.metrics["time"].setText(f"{score['score_time_s']:.1f} s" if "score_time_s" in score else "—")
                self.metrics["speed"].setText(f"{pose['speed']:.2f} m/s" if "speed" in pose else "—")
                self.metrics["distance"].setText(f"{score['distance_m']:.1f} m" if "distance_m" in score else "—")
                self.progress.setRange(0, max(1, total))
                self.progress.setValue(count)
                if state == "Running":
                    self.message.setText(f"Navigating waypoint {min(count+1, total)} / {total}" if count < total
                                         else "All waypoints reached · approaching finish")
                health = telemetry.get("sensors", {})
                self.sensors.setRowCount(len(health))
                for row, (name, sample) in enumerate(health.items()):
                    self.sensors.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
                    age = sample.get("age_s")
                    text = "Unavailable" if age is None else ("Paused" if state == "Paused" else
                            (f"Live · {age:.1f}s" if age < 5 else f"Stale · {age:.1f}s"))
                    self.sensors.setItem(row, 1, QtWidgets.QTableWidgetItem(text))
                self.show_media(telemetry)
            elif snapshot.get("connection_lost") and state == "Running":
                self.message.setText("Course telemetry disconnected; waiting for the supervisor.")
            self.set_controls(bool(self.worker and self.worker.is_alive()))

        def show_media(self, telemetry):
            camera = telemetry.get("camera_jpeg")
            if camera:
                pixmap = QtGui.QPixmap()
                if pixmap.loadFromData(base64.b64decode(camera), "JPG"):
                    self.camera.setPixmap(pixmap.scaled(420, 150, QtCore.Qt.KeepAspectRatio))
            grid = telemetry.get("costmap")
            if grid:
                values = grid["data"]
                pixels = bytearray()
                for value in values:
                    pixels.extend((37, 52, 64) if value < 0 else (int(60+value*1.8), int(160-value), int(170-value*1.4)))
                image = QtGui.QImage(bytes(pixels), grid["width"], grid["height"],
                                     grid["width"]*3, QtGui.QImage.Format_RGB888).copy().mirrored(False, True)
                self.costmap.setPixmap(QtGui.QPixmap.fromImage(image).scaled(420,150,QtCore.Qt.KeepAspectRatio))

        def refresh_log(self):
            if not self.run_dir:
                return
            parts = []
            for name in ("mission.log", "stack.log", "container.log"):
                path = self.run_dir / name
                if path.is_file():
                    with path.open("rb") as stream:
                        stream.seek(max(0, path.stat().st_size-7000))
                        parts.append(name + "\n" + stream.read().decode("utf-8", errors="replace"))
            self.log.setPlainText("\n\n".join(parts))
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

        def refresh_results(self):
            paths = list((self.repo / "runs").glob("full-track-*/result.json"))
            paths += [path for path in (self.repo / "runs").glob("full-track-*/desktop_host_result.json")
                      if not (path.parent / "result.json").is_file()]
            paths.sort(key=lambda path: path.stat().st_mtime, reverse=True)
            self.results.setRowCount(len(paths))
            for row, path in enumerate(paths):
                result = read_json(path)
                config = read_json(path.parent / "costmap_config.json")
                version = "v"+str(config["version"]) if config.get("version") else "Unrecorded"
                values = (path.parent.name, version,
                          "Passed" if result.get("mission_passed") else result.get("reason", "Failed"),
                          f"{result.get('waypoints_reached', '—')} / {result.get('waypoints_total', '—')}",
                          str(result.get("finish_reached", "—")), str(result.get("safety_gate_passed", "—")))
                for col, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    item.setData(QtCore.Qt.UserRole, str(path.parent))
                    self.results.setItem(row, col, item)

        def on_finished(self):
            self.worker = None
            for row in range(self.sensors.rowCount()):
                status = self.sensors.item(row, 1)
                if status and status.text() != "Unavailable":
                    status.setText("Session ended")
            self.set_controls(False)
            self.refresh_results()
            self.refresh_log()
            if self.reset_pending:
                self.reset_pending = False
                self.reset_session()
            if self.closing:
                self.close()

        def open_result(self, row, _column):
            self.open_replay(Path(self.results.item(row, 0).data(QtCore.Qt.UserRole)))

        def open_replay(self, folder=None):
            folder = folder if isinstance(folder, Path) else self.run_dir
            if folder and (folder / "replay.html").is_file():
                QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(folder / "replay.html")))

        def closeEvent(self, event):
            if self.worker and self.worker.is_alive():
                self.closing = True
                self.stop_session()
                event.ignore()
                return
            self.python_lab.close()
            self.self_drive_lab.close()
            super().closeEvent(event)

    return QtCore, QtWidgets, CourseView, FullCourseWindow


def launch_full_track_app(repo=None, bundle=None):
    QtCore, QtWidgets, CourseView, Window = _classes()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = Window(repo, bundle)
    window.show()
    app.exec_()
