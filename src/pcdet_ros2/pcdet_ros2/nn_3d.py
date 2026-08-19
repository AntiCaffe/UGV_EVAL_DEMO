"""Class-aware 3D ByteTrack implementation for OpenPCDet detections."""

import numpy as np

from .util_3d import ExtendedKalmanFilterXYZWH

try:
    import lap
except ImportError:  # pragma: no cover - exercised on development hosts
    lap = None


def _bev_corners(box):
    """Return counter-clockwise XY corners for a center-format 3D box."""
    x, y, _, dx, dy, _, yaw = box[:7]
    local = np.array([
        [-dx / 2.0, -dy / 2.0],
        [dx / 2.0, -dy / 2.0],
        [dx / 2.0, dy / 2.0],
        [-dx / 2.0, dy / 2.0],
    ], dtype=np.float64)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.array([x, y])


def _polygon_area(polygon):
    if len(polygon) < 3:
        return 0.0
    polygon = np.asarray(polygon)
    return 0.5 * abs(
        np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1))
        - np.dot(polygon[:, 1], np.roll(polygon[:, 0], -1))
    )


def _cross_2d(first, second):
    return first[0] * second[1] - first[1] * second[0]


def _line_intersection(start, end, edge_start, edge_end):
    direction = end - start
    edge = edge_end - edge_start
    denominator = _cross_2d(direction, edge)
    if abs(denominator) < 1e-12:
        return end
    factor = _cross_2d(edge_start - start, edge) / denominator
    return start + factor * direction


def _polygon_clip(subject, clip_polygon):
    """Sutherland-Hodgman intersection for convex CCW polygons."""
    output = list(subject)
    for index in range(len(clip_polygon)):
        edge_start = clip_polygon[index]
        edge_end = clip_polygon[(index + 1) % len(clip_polygon)]
        input_polygon = output
        output = []
        if not input_polygon:
            break
        start = input_polygon[-1]
        for end in input_polygon:
            end_inside = _cross_2d(
                edge_end - edge_start, end - edge_start
            ) >= -1e-9
            start_inside = _cross_2d(
                edge_end - edge_start, start - edge_start
            ) >= -1e-9
            if end_inside:
                if not start_inside:
                    output.append(_line_intersection(start, end, edge_start, edge_end))
                output.append(end)
            elif start_inside:
                output.append(_line_intersection(start, end, edge_start, edge_end))
            start = end
    return output


def compute_iou_3d(a_boxes, b_boxes):
    """Pairwise yaw-aware 3D IoU for center-format boxes."""
    a_boxes = np.asarray(a_boxes, dtype=np.float64).reshape(-1, 7)
    b_boxes = np.asarray(b_boxes, dtype=np.float64).reshape(-1, 7)
    result = np.zeros((len(a_boxes), len(b_boxes)), dtype=np.float32)
    if result.size == 0:
        return result

    a_corners = [_bev_corners(box) for box in a_boxes]
    b_corners = [_bev_corners(box) for box in b_boxes]
    a_bounds = np.array([[c[:, 0].min(), c[:, 1].min(), c[:, 0].max(), c[:, 1].max()] for c in a_corners])
    b_bounds = np.array([[c[:, 0].min(), c[:, 1].min(), c[:, 0].max(), c[:, 1].max()] for c in b_corners])
    a_volume = np.prod(np.maximum(a_boxes[:, 3:6], 0.0), axis=1)
    b_volume = np.prod(np.maximum(b_boxes[:, 3:6], 0.0), axis=1)

    for row, a_box in enumerate(a_boxes):
        a_bottom = a_box[2] - a_box[5] / 2.0
        a_top = a_box[2] + a_box[5] / 2.0
        for column, b_box in enumerate(b_boxes):
            if (
                a_bounds[row, 2] <= b_bounds[column, 0]
                or b_bounds[column, 2] <= a_bounds[row, 0]
                or a_bounds[row, 3] <= b_bounds[column, 1]
                or b_bounds[column, 3] <= a_bounds[row, 1]
            ):
                continue
            height = min(a_top, b_box[2] + b_box[5] / 2.0) - max(
                a_bottom, b_box[2] - b_box[5] / 2.0
            )
            if height <= 0.0:
                continue
            area = _polygon_area(_polygon_clip(a_corners[row], b_corners[column]))
            intersection = area * height
            union = a_volume[row] + b_volume[column] - intersection
            if union > 1e-9:
                result[row, column] = intersection / union
    return result


def iou_distance_3d(a_tracks, b_tracks):
    a_boxes = [track.tlwhdy for track in a_tracks]
    b_boxes = [track.tlwhdy for track in b_tracks]
    return 1.0 - compute_iou_3d(a_boxes, b_boxes)


class State:
    New = 0
    Tracked = 1
    Lost = 2
    Removed = 3


class Track:
    count = 0

    def __init__(self, box, score, object_class):
        box = np.asarray(box, dtype=np.float64)
        if box.size not in (7, 8):
            raise ValueError('A tracking box must contain 7 values and optional index')
        self._tlwhdy = box[:7].copy()
        self.idx = float(box[7]) if box.size == 8 else -1.0
        self.score = float(score)
        self.cls = float(object_class)
        self.kalman_filter = None
        self.mean = None
        self.covariance = None
        self.track_id = None
        self.state = State.New
        self.is_activated = False
        self.frame_id = 0
        self.start_frame = 0
        self.tracklet_len = 0
        self.hit_streak = 0
        self.missed_time = 0.0

    @classmethod
    def next_id(cls):
        cls.count += 1
        return cls.count

    @classmethod
    def reset_id(cls):
        cls.count = 0

    def activate(self, kalman_filter, frame_id, confirmed=False):
        self.kalman_filter = kalman_filter
        self.track_id = self.next_id()
        self.mean, self.covariance = kalman_filter.initiate(self._tlwhdy)
        self.state = State.Tracked
        self.is_activated = bool(confirmed)
        self.frame_id = frame_id
        self.start_frame = frame_id
        self.hit_streak = 1

    def predict(self, lost_velocity_decay=1.0, elapsed_dt=None):
        if self.state == State.Lost:
            self.mean[self.kalman_filter.ndim:] *= lost_velocity_decay
            self.missed_time += (
                self.kalman_filter.dt if elapsed_dt is None else elapsed_dt
            )
        self.mean, self.covariance = self.kalman_filter.predict(
            self.mean, self.covariance
        )

    def update(self, new_track, frame_id):
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, new_track._tlwhdy
        )
        self.state = State.Tracked
        self.is_activated = True
        self.frame_id = frame_id
        self.tracklet_len += 1
        self.hit_streak += 1
        self.missed_time = 0.0
        self.score = new_track.score
        self.cls = new_track.cls
        self.idx = new_track.idx

    def re_activate(self, new_track, frame_id):
        self.update(new_track, frame_id)

    def mark_lost(self, dt):
        self.state = State.Lost
        self.hit_streak = 0
        self.missed_time = max(self.missed_time, dt)

    def mark_removed(self):
        self.state = State.Removed
        self.hit_streak = 0

    @property
    def end_frame(self):
        return self.frame_id

    @property
    def tlwhdy(self):
        if self.mean is None:
            return self._tlwhdy.copy()
        return self.mean[:self.kalman_filter.ndim].copy()

    @property
    def velocity(self):
        if self.mean is None:
            return np.zeros(3, dtype=np.float64)
        return self.kalman_filter.velocity(self.mean)


class BYTETracker:
    """Two-stage, class-aware ByteTrack with EKF-CV box motion."""

    motion_model = 'EKF-CV'

    def __init__(
        self,
        frame_rate=10.0,
        high_score_threshold=0.5,
        class_score_thresholds=None,
        low_score_threshold=0.1,
        match_threshold=0.95,
        second_match_threshold=0.98,
        unconfirmed_match_threshold=0.90,
        max_time_lost=1.0,
        mahalanobis_gate=16.27,
        lost_velocity_decay=0.95,
    ):
        self.tracked_tracks = []
        self.lost_tracks = []
        self.removed_tracks = []
        self.frame_id = 0
        self.default_dt = 1.0 / max(float(frame_rate), 1e-3)
        self.last_timestamp = None
        self.high_score_threshold = float(high_score_threshold)
        self.class_score_thresholds = list(class_score_thresholds or [])
        self.low_score_threshold = float(low_score_threshold)
        self.match_threshold = float(match_threshold)
        self.second_match_threshold = float(second_match_threshold)
        self.unconfirmed_match_threshold = float(unconfirmed_match_threshold)
        self.max_time_lost = float(max_time_lost)
        self.mahalanobis_gate = float(mahalanobis_gate)
        self.lost_velocity_decay = float(lost_velocity_decay)
        self.kalman_filter = ExtendedKalmanFilterXYZWH(self.default_dt)
        Track.reset_id()

    def _compute_dt(self, timestamp):
        elapsed_dt = self.default_dt
        filter_dt = self.default_dt
        if timestamp is not None and np.isfinite(timestamp):
            timestamp = float(timestamp)
            if self.last_timestamp is not None:
                measured_dt = timestamp - self.last_timestamp
                if measured_dt > 0.0:
                    elapsed_dt = measured_dt
                    filter_dt = np.clip(measured_dt, 0.01, 0.5)
            self.last_timestamp = timestamp
        self.kalman_filter.set_dt(filter_dt)
        return elapsed_dt

    def _high_threshold(self, object_class):
        index = int(round(float(object_class))) - 1
        if 0 <= index < len(self.class_score_thresholds):
            return float(self.class_score_thresholds[index])
        return self.high_score_threshold

    def update(self, boxes, scores, object_classes, timestamp=None):
        self.frame_id += 1
        dt = self._compute_dt(timestamp)
        activated, refound, newly_lost, removed = [], [], [], []

        detections = self.init_track(boxes, scores, object_classes)
        detections = [
            detection for detection in detections
            if np.isfinite(detection.score)
            and detection.score >= min(
                self.low_score_threshold,
                self._high_threshold(detection.cls),
            )
            and np.all(np.isfinite(detection._tlwhdy))
            and np.all(detection._tlwhdy[3:6] > 0.0)
        ]
        high_detections = [
            detection for detection in detections
            if detection.score >= self._high_threshold(detection.cls)
        ]
        low_detections = [
            detection for detection in detections
            if detection.score < self._high_threshold(detection.cls)
        ]

        unconfirmed = [track for track in self.tracked_tracks if not track.is_activated]
        tracked = [track for track in self.tracked_tracks if track.is_activated]
        track_pool = self.joint_stracks(tracked, self.lost_tracks)
        self.multi_predict(track_pool, dt)
        self.multi_predict(unconfirmed, dt)

        matches, unmatched_pool, unmatched_high = self.linear_assignment(
            self.get_dists(track_pool, high_detections), self.match_threshold
        )
        for track_index, detection_index in matches:
            track = track_pool[track_index]
            detection = high_detections[detection_index]
            if track.state == State.Tracked:
                track.update(detection, self.frame_id)
                activated.append(track)
            else:
                track.re_activate(detection, self.frame_id)
                refound.append(track)

        remaining_tracked = [
            track_pool[index] for index in unmatched_pool
            if track_pool[index].state == State.Tracked
        ]
        second_matches, unmatched_tracked, _ = self.linear_assignment(
            self.get_dists(remaining_tracked, low_detections),
            self.second_match_threshold,
        )
        for track_index, detection_index in second_matches:
            track = remaining_tracked[track_index]
            track.update(low_detections[detection_index], self.frame_id)
            activated.append(track)
        for track_index in unmatched_tracked:
            track = remaining_tracked[track_index]
            track.mark_lost(dt)
            newly_lost.append(track)

        remaining_high = [high_detections[index] for index in unmatched_high]
        unconfirmed_matches, unmatched_unconfirmed, unmatched_remaining = (
            self.linear_assignment(
                self.get_dists(unconfirmed, remaining_high),
                self.unconfirmed_match_threshold,
            )
        )
        for track_index, detection_index in unconfirmed_matches:
            track = unconfirmed[track_index]
            track.update(remaining_high[detection_index], self.frame_id)
            activated.append(track)
        for track_index in unmatched_unconfirmed:
            track = unconfirmed[track_index]
            track.mark_removed()
            removed.append(track)

        for detection_index in unmatched_remaining:
            track = remaining_high[detection_index]
            track.activate(
                self.kalman_filter,
                self.frame_id,
                confirmed=(self.frame_id == 1),
            )
            activated.append(track)

        for track in self.joint_stracks(self.lost_tracks, newly_lost):
            if track.missed_time > self.max_time_lost:
                track.mark_removed()
                removed.append(track)

        self.tracked_tracks = [
            track for track in self.tracked_tracks if track.state == State.Tracked
        ]
        self.tracked_tracks = self.joint_stracks(self.tracked_tracks, activated)
        self.tracked_tracks = self.joint_stracks(self.tracked_tracks, refound)
        self.lost_tracks = self.sub_stracks(self.lost_tracks, self.tracked_tracks)
        self.lost_tracks.extend(newly_lost)
        self.lost_tracks = self.sub_stracks(self.lost_tracks, removed)
        self.removed_tracks.extend(removed)
        self.tracked_tracks, self.lost_tracks = self.remove_duplicate_stracks(
            self.tracked_tracks, self.lost_tracks
        )
        return self._outputs()

    def _outputs(self):
        outputs = []
        for track in self.tracked_tracks:
            if not track.is_activated or track.state != State.Tracked:
                continue
            x, y, z, dx, dy, dz, yaw = track.tlwhdy
            vx, vy, vz = track.velocity
            outputs.append([
                x - dx / 2.0, y - dy / 2.0, z - dz / 2.0,
                x + dx / 2.0, y + dy / 2.0, z + dz / 2.0,
                track.track_id, track.score, track.cls, track.idx,
                vx, vy, vz, yaw,
            ])
        return np.asarray(outputs, dtype=np.float32).reshape(-1, 14)

    @staticmethod
    def init_track(boxes, scores, object_classes):
        return [
            Track(box, score, object_class)
            for box, score, object_class in zip(boxes, scores, object_classes)
        ]

    def get_dists(self, tracks, detections):
        if not tracks or not detections:
            return np.empty((len(tracks), len(detections)), dtype=np.float32)
        iou_cost = iou_distance_3d(tracks, detections)
        detection_measurements = np.asarray([det.tlwhdy for det in detections])
        score_cost = 1.0 - np.asarray([det.score for det in detections])[None, :]
        cost = np.empty_like(iou_cost)
        for row, track in enumerate(tracks):
            gate_distance = self.kalman_filter.gating_distance(
                track.mean, track.covariance, detection_measurements
            )
            normalized_motion = np.minimum(
                gate_distance / max(self.mahalanobis_gate, 1e-6), 1.0
            )
            cost[row] = (
                0.75 * iou_cost[row]
                + 0.20 * normalized_motion
                + 0.05 * score_cost[0]
            )
            incompatible = np.array([
                int(round(track.cls)) != int(round(det.cls))
                for det in detections
            ])
            cost[row, incompatible | (gate_distance > self.mahalanobis_gate)] = 1e6
        return cost

    def multi_predict(self, tracks, elapsed_dt=None):
        for track in tracks:
            track.predict(self.lost_velocity_decay, elapsed_dt)

    @staticmethod
    def joint_stracks(first, second):
        result = list(first)
        known = {track.track_id for track in first}
        result.extend(track for track in second if track.track_id not in known)
        return result

    @staticmethod
    def sub_stracks(first, second):
        removed_ids = {track.track_id for track in second}
        return [track for track in first if track.track_id not in removed_ids]

    @staticmethod
    def remove_duplicate_stracks(tracked, lost):
        if not tracked or not lost:
            return tracked, lost
        distances = iou_distance_3d(tracked, lost)
        duplicate_tracked, duplicate_lost = set(), set()
        for tracked_index, lost_index in zip(*np.where(distances < 0.15)):
            if int(round(tracked[tracked_index].cls)) != int(round(lost[lost_index].cls)):
                continue
            tracked_age = tracked[tracked_index].frame_id - tracked[tracked_index].start_frame
            lost_age = lost[lost_index].frame_id - lost[lost_index].start_frame
            if tracked_age >= lost_age:
                duplicate_lost.add(lost_index)
            else:
                duplicate_tracked.add(tracked_index)
        return (
            [track for index, track in enumerate(tracked) if index not in duplicate_tracked],
            [track for index, track in enumerate(lost) if index not in duplicate_lost],
        )

    @staticmethod
    def linear_assignment(cost_matrix, threshold):
        rows, columns = cost_matrix.shape
        if cost_matrix.size == 0:
            return (
                np.empty((0, 2), dtype=int),
                np.arange(rows, dtype=int),
                np.arange(columns, dtype=int),
            )
        if lap is not None:
            _, row_assignment, column_assignment = lap.lapjv(
                cost_matrix, extend_cost=True, cost_limit=threshold
            )
            matches = np.array([
                [row, column]
                for row, column in enumerate(row_assignment)
                if column >= 0
            ], dtype=int).reshape(-1, 2)
            return (
                matches,
                np.where(row_assignment < 0)[0],
                np.where(column_assignment < 0)[0],
            )

        from scipy.optimize import linear_sum_assignment
        assigned_rows, assigned_columns = linear_sum_assignment(cost_matrix)
        matches = np.array([
            [row, column]
            for row, column in zip(assigned_rows, assigned_columns)
            if cost_matrix[row, column] <= threshold
        ], dtype=int).reshape(-1, 2)
        matched_rows = set(matches[:, 0]) if len(matches) else set()
        matched_columns = set(matches[:, 1]) if len(matches) else set()
        return (
            matches,
            np.array([row for row in range(rows) if row not in matched_rows]),
            np.array([column for column in range(columns) if column not in matched_columns]),
        )
