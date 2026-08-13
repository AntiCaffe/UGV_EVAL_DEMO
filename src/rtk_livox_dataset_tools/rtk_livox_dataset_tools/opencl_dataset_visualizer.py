import argparse
import os

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from rtk_livox_dataset_tools.opencl_dataset_exporter import FRAME_DTYPE, POINT_DTYPE, RTK_DTYPE


class Dataset:
    def __init__(self, directory):
        self.directory = directory
        self.frames = np.fromfile(os.path.join(directory, "frames.bin"), dtype=FRAME_DTYPE)
        self.points = np.memmap(os.path.join(directory, "points.bin"), dtype=POINT_DTYPE, mode="r")
        self.rtk = np.fromfile(os.path.join(directory, "rtk_gt.bin"), dtype=RTK_DTYPE)
        if self.frames.size == 0:
            raise RuntimeError("No frames in dataset")
        if self.rtk.size == 0:
            raise RuntimeError("No RTK samples in dataset")

    @property
    def start_time(self):
        return float(self.frames["stamp_sec"][0])

    @property
    def end_time(self):
        return float(self.frames["stamp_sec"][-1])

    def frame_points(self, index):
        frame = self.frames[index]
        start = int(frame["offset"])
        end = start + int(frame["count"])
        return self.points[start:end]

    def frame_indices_in_window(self, now, decay):
        stamps = self.frames["stamp_sec"]
        start = np.searchsorted(stamps, now - decay, side="left")
        end = np.searchsorted(stamps, now, side="right")
        return range(max(0, start), min(len(stamps), end))

    def interpolate_rtk(self, stamp):
        stamps = self.rtk["stamp_sec"]
        if stamp <= stamps[0]:
            row = self.rtk[0]
            return _rtk_state_from_row(row)
        if stamp >= stamps[-1]:
            row = self.rtk[-1]
            return _rtk_state_from_row(row)
        idx = int(np.searchsorted(stamps, stamp, side="right"))
        a = self.rtk[idx - 1]
        b = self.rtk[idx]
        denom = float(b["stamp_sec"] - a["stamp_sec"])
        alpha = 0.0 if denom <= 0.0 else float((stamp - a["stamp_sec"]) / denom)
        pos = np.array([a["px"], a["py"], a["pz"]], dtype=float) * (1.0 - alpha) + np.array([b["px"], b["py"], b["pz"]], dtype=float) * alpha
        vel = np.array([a["vx"], a["vy"], a["vz"]], dtype=float) * (1.0 - alpha) + np.array([b["vx"], b["vy"], b["vz"]], dtype=float) * alpha
        return pos, vel


def _rtk_state_from_row(row):
    return (
        np.array([row["px"], row["py"], row["pz"]], dtype=float),
        np.array([row["vx"], row["vy"], row["vz"]], dtype=float),
    )


class TopDownView(QtWidgets.QWidget):
    def __init__(self, dataset):
        super().__init__()
        self.dataset = dataset
        self.now = dataset.start_time
        self.decay_sec = 1.0
        self.antenna_offset = np.zeros(3, dtype=float)
        self.scale = 22.0
        self.max_render_points = 120000
        self.last_drawn = 0
        self.last_frame_count = 0
        self.setMinimumSize(900, 700)
        self.setAutoFillBackground(True)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(15, 18, 22))
        width = self.width()
        height = self.height()
        cx = width * 0.5
        cy = height * 0.58

        indices = list(self.dataset.frame_indices_in_window(self.now, self.decay_sec))
        total = 0
        for frame_index in indices:
            points = self.dataset.frame_points(frame_index)
            total += int(points.shape[0])
        stride = max(1, int(np.ceil(float(total) / float(self.max_render_points)))) if total else 1

        image = np.zeros((height, width, 4), dtype=np.uint8)
        image[:, :, 0] = 22
        image[:, :, 1] = 18
        image[:, :, 2] = 15
        image[:, :, 3] = 255

        drawn = 0
        for frame_index in indices:
            frame = self.dataset.frames[frame_index]
            age = max(0.0, self.now - float(frame["stamp_sec"]))
            alpha = max(30, int(210 * (1.0 - min(1.0, age / max(self.decay_sec, 1.0e-3)))))
            points = self.dataset.frame_points(frame_index)[::stride]
            xs = (cx + points["x"] * self.scale).astype(np.int32, copy=False)
            ys = (cy - points["y"] * self.scale).astype(np.int32, copy=False)
            mask = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
            if np.any(mask):
                xs = xs[mask]
                ys = ys[mask]
                image[ys, xs, 0] = np.maximum(image[ys, xs, 0], alpha)
                image[ys, xs, 1] = 180
                image[ys, xs, 2] = 120
                drawn += int(xs.size)

        qimage = QtGui.QImage(image.data, width, height, image.strides[0], QtGui.QImage.Format_RGBA8888)
        painter.drawImage(0, 0, qimage)
        self.last_drawn = drawn
        self.last_frame_count = len(indices)

        pos, vel = self.dataset.interpolate_rtk(self.now)
        pos = pos - self.antenna_offset
        speed = float(np.linalg.norm(vel[:2]))
        px = cx + pos[0] * self.scale
        py = cy - pos[1] * self.scale

        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 255, 100), 2))
        painter.setBrush(QtGui.QColor(20, 255, 100, 110))
        painter.drawEllipse(QtCore.QPointF(px, py), 8.0, 8.0)

        arrow_len_px = 70.0
        if speed > 1.0e-3:
            direction = vel[:2] / speed
            ex = px + direction[0] * arrow_len_px
            ey = py - direction[1] * arrow_len_px
        else:
            ex = px
            ey = py
        thickness = min(16.0, 2.0 + speed * 5.0)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 90, 25), thickness, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        painter.drawLine(QtCore.QPointF(px, py), QtCore.QPointF(ex, ey))

        painter.setPen(QtGui.QColor(235, 235, 235))
        painter.drawText(12, 22, "t=%.3f  decay=%.2fs  frames=%d  drawn_points=%d  speed=%.2fm/s" % (self.now - self.dataset.start_time, self.decay_sec, len(indices), drawn, speed))
        painter.drawText(12, 44, "p_antenna_in_lidar=[%.2f, %.2f, %.2f]" % tuple(self.antenna_offset))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, dataset, rate):
        super().__init__()
        self.dataset = dataset
        self.rate = rate
        self.view = TopDownView(dataset)
        self.playing = True
        self.last_tick = QtCore.QElapsedTimer()
        self.last_tick.start()

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.addWidget(self.view, 1)
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        self.play_button = QtWidgets.QPushButton("Pause")
        self.play_button.clicked.connect(self._toggle_play)
        controls.addWidget(self.play_button)

        self.decay_spin = self._spin(0.05, 10.0, self.view.decay_sec, 0.05)
        self.decay_spin.valueChanged.connect(self._set_decay)
        controls.addWidget(QtWidgets.QLabel("Decay"))
        controls.addWidget(self.decay_spin)

        self.x_spin = self._spin(-5.0, 5.0, 0.0, 0.01)
        self.y_spin = self._spin(-5.0, 5.0, 0.0, 0.01)
        self.z_spin = self._spin(-5.0, 5.0, 0.0, 0.01)
        for label, spin in (("p_x", self.x_spin), ("p_y", self.y_spin), ("p_z", self.z_spin)):
            controls.addWidget(QtWidgets.QLabel(label))
            controls.addWidget(spin)
            spin.valueChanged.connect(self._set_offset)

        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.time_slider.setRange(0, 10000)
        self.time_slider.sliderMoved.connect(self._seek)
        layout.addWidget(self.time_slider)

        self.setCentralWidget(central)
        self.setWindowTitle("Livox RTK offline visualizer")

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

    def _spin(self, lo, hi, value, step):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        return spin

    def _toggle_play(self):
        self.playing = not self.playing
        self.play_button.setText("Pause" if self.playing else "Play")
        self.last_tick.restart()

    def _set_decay(self, value):
        self.view.decay_sec = float(value)
        self.view.update()

    def _set_offset(self):
        self.view.antenna_offset = np.array([self.x_spin.value(), self.y_spin.value(), self.z_spin.value()], dtype=float)
        self.view.update()

    def _seek(self, value):
        frac = float(value) / 10000.0
        self.view.now = self.dataset.start_time + (self.dataset.end_time - self.dataset.start_time) * frac
        self.last_tick.restart()
        self.view.update()

    def _tick(self):
        elapsed = self.last_tick.restart() * 1.0e-3
        if self.playing:
            self.view.now += elapsed * self.rate
            if self.view.now > self.dataset.end_time:
                self.view.now = self.dataset.start_time
        frac = (self.view.now - self.dataset.start_time) / max(1.0e-6, self.dataset.end_time - self.dataset.start_time)
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(int(max(0.0, min(1.0, frac)) * 10000))
        self.time_slider.blockSignals(False)
        self.view.update()


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QtWidgets.QApplication([])
    window = MainWindow(Dataset(args.dataset_dir), args.rate)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
