"""Qt view of the live ROS local costmap and supervised track session."""
from __future__ import annotations

import math
import json
import threading
import time
from pathlib import Path

from .full_track import repository_root
from .ui import _qt


def _classes():
    QtCore, QtGui, QtWidgets = _qt()

    class CostmapCanvas(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.telemetry = {}
            self.setMinimumSize(550, 420)

        def paintEvent(self, event):
            p = QtGui.QPainter(self)
            p.fillRect(self.rect(), QtGui.QColor('#101b23'))
            grid = self.telemetry.get('costmap') or {}
            width, height = grid.get('width', 0), grid.get('height', 0)
            values = grid.get('data', [])
            if not width or not height or len(values) != width*height:
                p.setPen(QtGui.QColor('#c6d8e1'))
                p.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Waiting for ROS local costmap')
                return
            if time.time() - grid.get('received_wall_s', 0) > 3.0:
                p.setPen(QtGui.QColor('#ffb49c'))
                p.drawText(self.rect(), QtCore.Qt.AlignCenter, 'ROS local costmap is stale')
                return
            margin = 20
            scale = min((self.width()-2*margin)/width, (self.height()-2*margin)/height)
            left, top = (self.width()-width*scale)/2, (self.height()-height*scale)/2
            for iy in range(height):
                for ix in range(width):
                    value = values[iy*width+ix]
                    if value < 0:
                        color = QtGui.QColor('#33434b')
                    elif value >= 100:
                        color = QtGui.QColor('#be4053')
                    elif value:
                        fraction = min(1.0, value/100.0)
                        color = QtGui.QColor(int(33+205*fraction), int(46+97*fraction), 43)
                    else:
                        color = QtGui.QColor('#202e35')
                    p.fillRect(QtCore.QRectF(left+ix*scale, top+(height-iy-1)*scale,
                                            scale+0.4, scale+0.4), color)
            origin = grid.get('origin')
            resolution = grid.get('resolution_m', 0)
            if not origin or not resolution:
                return
            ox, oy, angle = origin
            c, s = math.cos(angle), math.sin(angle)
            def point(x, y):
                dx, dy = x-ox, y-oy
                gx, gy = (c*dx+s*dy)/resolution, (-s*dx+c*dy)/resolution
                return QtCore.QPointF(left+gx*scale, top+(height-gy)*scale)
            divider = self.telemetry.get('divider_costmap') or {}
            divider_data = divider.get('data', [])
            dw, dh = divider.get('width', 0), divider.get('height', 0)
            if (divider.get('frame') == grid.get('frame') and divider.get('origin')
                    and dw and dh and len(divider_data) == dw*dh
                    and time.time()-divider.get('received_wall_s', 0) <= 3.0):
                dx0, dy0, da = divider['origin']
                dr = divider.get('resolution_m', 0)
                dc, ds = math.cos(da), math.sin(da)
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(178, 114, 238, 180))
                for iy in range(dh):
                    for ix in range(dw):
                        if divider_data[iy*dw+ix] <= 0:
                            continue
                        corners = [(ix*dr, iy*dr), ((ix+1)*dr, iy*dr),
                                   ((ix+1)*dr, (iy+1)*dr), (ix*dr, (iy+1)*dr)]
                        p.drawPolygon(QtGui.QPolygonF([point(dx0+dc*x-ds*y, dy0+ds*x+dc*y)
                                                       for x, y in corners]))
            if self.telemetry.get('self_drive_plan_frame') == grid.get('frame'):
                path_points = self.telemetry.get('self_drive_plan', [])
                if path_points:
                    path = QtGui.QPainterPath(point(*path_points[0]))
                    for x, y in path_points[1:]:
                        path.lineTo(point(x, y))
                    p.setPen(QtGui.QPen(QtGui.QColor('#77e9a6'), 2))
                    p.drawPath(path)
            pose = grid.get('robot_pose')
            if pose:
                x, y, yaw = pose
                cy, sy = math.cos(yaw), math.sin(yaw)
                # Pinned ROS base_link footprint; x is forward, not centred.
                corners = [(x+cy*a-sy*b, y+sy*a+cy*b) for a,b in
                           ((-.32,-.41),(.77,-.41),(.77,.41),(-.32,.41))]
                p.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 2))
                p.setBrush(QtGui.QColor('#b4dce8'))
                p.drawPolygon(QtGui.QPolygonF([point(*v) for v in corners]))
                p.setPen(QtGui.QPen(QtGui.QColor('#ee775f'), 3))
                p.drawLine(point(x, y), point(x+cy*.77, y+sy*.77))

    class Bridge(QtCore.QObject):
        changed = QtCore.pyqtSignal(object)
        finished = QtCore.pyqtSignal()

    class TrackCanvas(QtWidgets.QWidget):
        """Course and Gazebo truth are display-only; the controller does not read them."""
        def __init__(self):
            super().__init__()
            self.track = {}
            self.telemetry = {}
            self.setMinimumSize(350, 300)

        def paintEvent(self, event):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            p.fillRect(self.rect(), QtGui.QColor('#101b23'))
            if not self.track:
                p.setPen(QtGui.QColor('#c6d8e1'))
                p.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Waiting for generated ROS track')
                return
            margin = 20
            x0, y0, x1, y1 = self.track.get('extent') or (0.0, 0.0, 26.0, 20.0)
            span_x, span_y = x1-x0, y1-y0
            scale = min((self.width()-2*margin)/span_x, (self.height()-2*margin)/span_y)
            left, top = (self.width()-span_x*scale)/2, (self.height()-span_y*scale)/2
            def point(x, y):
                return QtCore.QPointF(left+(x-x0)*scale, top+(y1-y)*scale)
            def line(points, color, width, dashed=False, closed=True):
                if not points:
                    return
                path = QtGui.QPainterPath(point(*points[0]))
                for v in points[1:]:
                    path.lineTo(point(*v))
                if closed:
                    path.closeSubpath()
                pen = QtGui.QPen(QtGui.QColor(color), width)
                if dashed:
                    pen.setStyle(QtCore.Qt.DashLine)
                p.setPen(pen)
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawPath(path)
            line(self.track.get('outer', []), '#e9f0e7', 2)
            line(self.track.get('inner', []), '#e9f0e7', 2)
            line(self.track.get('center', []), '#d5c98e', 1.5, True)
            # Open-polyline layouts (the T-intersection).
            for points in self.track.get('lines', []):
                line(points, '#e9f0e7', 2, closed=False)
            for points in self.track.get('dashed', []):
                line(points, '#d5c98e', 1.5, True, closed=False)
            for points in self.track.get('stop_lines', []):
                line(points, '#f0c36b', 3, closed=False)
            for name, zone in (self.track.get('exits') or {}).items():
                p.setPen(QtGui.QPen(QtGui.QColor('#5f8c6e'), 1, QtCore.Qt.DotLine))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawEllipse(point(zone['x_m'], zone['y_m']), zone['radius_m']*scale, zone['radius_m']*scale)
                p.drawText(point(zone['x_m']-zone['radius_m'], zone['y_m']+zone['radius_m']+.2), name)
            p.setPen(QtGui.QPen(QtGui.QColor('#ec9a9a'), 1))
            p.setBrush(QtGui.QColor('#a84450'))
            for x, y, r in self.track.get('obstacles', []):
                p.drawEllipse(point(x, y), r*scale, r*scale)
            trail = self.telemetry.get('trail') or []
            if len(trail) > 1:
                path = QtGui.QPainterPath(point(*trail[0]))
                for v in trail[1:]:
                    path.lineTo(point(*v))
                p.setPen(QtGui.QPen(QtGui.QColor('#64c7ed'), 2))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawPath(path)
            pose = self.telemetry.get('pose')
            if pose:
                x, y, yaw = pose['x'], pose['y'], pose['yaw']
            else:
                start = self.track['start']
                x, y, yaw = start['x_m'], start['y_m'], start['yaw_rad']
            c, s = math.cos(yaw), math.sin(yaw)
            corners = [(x+c*a-s*b, y+s*a+c*b) for a,b in
                       ((-.32,-.41),(.77,-.41),(.77,.41),(-.32,.41))]
            p.setPen(QtGui.QPen(QtGui.QColor('#fff'), 1))
            p.setBrush(QtGui.QColor('#b4dce8'))
            p.drawPolygon(QtGui.QPolygonF([point(*v) for v in corners]))

    class SelfDrivePage(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.setStyleSheet('''
                QWidget { background: #0c1821; color: #dce7ed; font-family: "Segoe UI"; font-size: 13px; }
                QPushButton { background: #203541; border: 1px solid #385363; border-radius: 5px; padding: 8px 16px; }
                QPushButton:hover { background: #2d4a59; }
                QPushButton:disabled { color: #607783; border-color: #253a46; }
                QComboBox { background: #203541; border: 1px solid #385363; border-radius: 5px; padding: 6px; }
                QComboBox:disabled { color: #607783; }
            ''')
            self.session = None
            self.worker = None
            self.bridge = Bridge()
            self.bridge.changed.connect(self.update_state)
            self.bridge.finished.connect(self.finished)
            self.canvas = CostmapCanvas()
            self.track_canvas = TrackCanvas()
            self.run_button = QtWidgets.QPushButton('Run ROS track')
            self.pause_button = QtWidgets.QPushButton('Pause')
            self.stop_button = QtWidgets.QPushButton('Stop')
            self.lane_change_box = QtWidgets.QComboBox()
            for value in (0.0, 0.1, 0.25, 0.5, 1.0):
                self.lane_change_box.addItem(f'{value:g} m', value)
            self.lane_change_box.setCurrentIndex(2)
            self.lane_change_box.setToolTip('Extra path cost for crossing the broken lane divider. Set before starting a run.')
            self.track_box = QtWidgets.QComboBox()
            self.track_box.addItem('Closed loop', 'loop')
            self.track_box.addItem('T-intersection', 'intersection')
            self.track_box.setToolTip('Track layout. Set before starting a run.')
            self.turn_box = QtWidgets.QComboBox()
            self.turn_box.addItem('Straight (default)', 'straight')
            self.turn_box.addItem('Turn left', 'left')
            self.turn_box.setToolTip('Route at the next junction. Can be changed while running; '
                                     'the robot decides when it leaves the stop line.')
            self.turn_box.currentIndexChanged.connect(self.turn_changed)
            self.run_button.clicked.connect(self.run)
            self.pause_button.clicked.connect(self.pause)
            self.stop_button.clicked.connect(self.stop)
            self.pause_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            self.status = QtWidgets.QLabel('Ready · ROS track is not running')
            self.status.setWordWrap(True)
            layout = QtWidgets.QVBoxLayout(self)
            description = QtWidgets.QLabel('Waypoint-free ROS self-drive · live local costmap · supervised velocity commands')
            description.setWordWrap(True)
            controls = QtWidgets.QHBoxLayout()
            for button in (self.run_button, self.pause_button, self.stop_button):
                controls.addWidget(button)
            controls.addWidget(QtWidgets.QLabel('Broken-line crossing penalty'))
            controls.addWidget(self.lane_change_box)
            controls.addWidget(QtWidgets.QLabel('Track'))
            controls.addWidget(self.track_box)
            controls.addWidget(QtWidgets.QLabel('At junction'))
            controls.addWidget(self.turn_box)
            controls.addStretch()
            layout.addWidget(description)
            layout.addLayout(controls)
            views = QtWidgets.QHBoxLayout()
            for title, canvas in (('Track · Gazebo truth for display', self.track_canvas),
                                  ('Local ROS costmap · controller input', self.canvas)):
                column = QtWidgets.QVBoxLayout()
                column.addWidget(QtWidgets.QLabel(title))
                column.addWidget(canvas, 1)
                views.addLayout(column, 1)
            layout.addLayout(views, 1)
            legend = QtWidgets.QLabel('Red: lethal (100) · amber: nonlethal cost (1–99) · purple: observed crossable paint (divider, stop lines) · grey: unknown · green: local trajectory · white: robot footprint')
            legend.setWordWrap(True)
            layout.addWidget(legend)
            layout.addWidget(self.status)
            self.paused = False

        def run(self):
            if self.worker and self.worker.is_alive():
                return
            from .self_drive_ros import SelfDriveRosSession
            self.session = SelfDriveRosSession(repo=Path(repository_root()),
                                               lane_change_cost_m=self.lane_change_box.currentData(),
                                               track_layout=self.track_box.currentData(),
                                               turn_command=self.turn_box.currentData())
            self.paused = False
            self.run_button.setEnabled(False)
            self.lane_change_box.setEnabled(False)
            self.track_box.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.status.setText('Starting ROS track…')
            self.canvas.telemetry = {}
            self.canvas.update()
            self.track_canvas.telemetry = {}
            self.track_canvas.track = {}
            self.track_canvas.update()
            def work():
                try:
                    self.session.run(self.bridge.changed.emit)
                finally:
                    self.bridge.finished.emit()
            self.worker = threading.Thread(target=work, daemon=True)
            self.worker.start()

        def turn_changed(self, _index):
            if self.session and self.worker and self.worker.is_alive():
                try:
                    self.session.set_turn_command(self.turn_box.currentData())
                except OSError as exc:
                    self.status.setText(f'Turn command not sent: {exc}')

        def pause(self):
            if not self.session:
                return
            self.paused = not self.paused
            self.session.request('paused' if self.paused else 'running')
            self.pause_button.setText('Resume' if self.paused else 'Pause')

        def stop(self):
            if self.session:
                self.session.request('stopped')
                self.stop_button.setEnabled(False)
                self.pause_button.setEnabled(False)
                self.status.setText('Stopping ROS track…')

        def update_state(self, snapshot):
            state = snapshot.get('state', 'Starting')
            message = snapshot.get('message') or state
            self.status.setText(f'{state} · {message}')
            self.pause_button.setEnabled(state in ('Running', 'Paused'))
            telemetry = snapshot.get('telemetry')
            if not self.track_canvas.track and snapshot.get('run_dir'):
                try:
                    path = Path(snapshot['run_dir']) / 'track' / 'track.json'
                    self.track_canvas.track = json.loads(path.read_text(encoding='utf-8'))
                except (OSError, ValueError):
                    pass
            if telemetry:
                self.canvas.telemetry = telemetry
                self.canvas.update()
                self.track_canvas.telemetry = telemetry
                self.track_canvas.update()
                controller = telemetry.get('self_drive_status') or {}
                pose = telemetry.get('pose') or {}
                details = f'{state} · {message}'
                if controller and state not in ('Failed', 'Stopped') and controller.get('state') not in message:
                    details += f' · controller: {controller.get("reason", controller.get("state", "active"))}'
                if 'divider_available' in controller:
                    if controller['divider_available']:
                        details += f' · divider: {controller.get("observed_divider_cells", 0)} observed cells'
                    else:
                        details += ' · divider data unavailable'
                junction = controller.get('junction') or {}
                if junction and (junction.get('state') != 'cruise' or junction.get('route')):
                    details += (f' · junction: {junction.get("reason", junction.get("state"))}'
                                f' · request {junction.get("turn_request")}')
                    if junction.get('route'):
                        details += f' · route {junction["route"]}'
                if isinstance(pose.get('speed'), (int, float)):
                    details += f' · speed {pose["speed"]:.2f} m/s'
                self.status.setText(details)
            elif snapshot.get('connection_lost'):
                self.canvas.telemetry = {}
                self.canvas.update()

        def finished(self):
            self.run_button.setEnabled(True)
            self.lane_change_box.setEnabled(True)
            self.track_box.setEnabled(True)
            self.pause_button.setEnabled(False)
            self.pause_button.setText('Pause')
            self.stop_button.setEnabled(False)

        def closeEvent(self, event):
            self.stop()
            super().closeEvent(event)

    return CostmapCanvas, SelfDrivePage


def launch_self_drive_app():
    import sys
    _, _, QtWidgets = _qt()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    page = _classes()[1]()
    page.setWindowTitle('AutoNav Universe · ROS self-drive track')
    page.resize(1100, 820)
    page.show()
    app.exec_()
