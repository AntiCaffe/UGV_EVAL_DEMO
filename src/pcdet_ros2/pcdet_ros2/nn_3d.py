import numpy
from .util_3d import *
import logging


logging.basicConfig(level=logging.DEBUG)

try:
    import lap  # for linear_assignment

    assert lap.__version__  # verify package is not directory
except (ImportError, AssertionError, AttributeError):
    import lap


def compute_iou_3d(a_boxes, b_boxes):
    """
    Compute IoU between 3D bounding boxes
    """
    iou = numpy.zeros((len(a_boxes), len(b_boxes)), dtype=numpy.float32)
    if iou.size == 0:
        return iou
    
    # 메모리 연속성(속도 최적화)을 보장하는 함수 
    a_boxes = numpy.ascontiguousarray(a_boxes, dtype=numpy.float32)
    b_boxes = numpy.ascontiguousarray(b_boxes, dtype=numpy.float32)
    
    # Get the coordinates of 3D bounding boxes
    b1_x1, b1_y1, b1_z1, b1_x2, b1_y2, b1_z2 = a_boxes.T
    b2_x1, b2_y1, b2_z1, b2_x2, b2_y2, b2_z2 = b_boxes.T

    # Intersection volume
    inter_vol = (
        (numpy.minimum(b1_x2[:, None], b2_x2) - numpy.maximum(b1_x1[:, None], b2_x1)).clip(0) *
        (numpy.minimum(b1_y2[:, None], b2_y2) - numpy.maximum(b1_y1[:, None], b2_y1)).clip(0) *
        (numpy.minimum(b1_z2[:, None], b2_z2) - numpy.maximum(b1_z1[:, None], b2_z1)).clip(0)
    )

    # Volume of each box
    box1_vol = abs(b1_x2 - b1_x1) * abs(b1_y2 - b1_y1) * abs(b1_z2 - b1_z1)
    box2_vol = abs(b2_x2 - b2_x1) * abs(b2_y2 - b2_y1) * abs(b2_z2 - b2_z1)

    # IoU calculation
    return inter_vol / (box1_vol[:, None] + box2_vol - inter_vol + 1E-7)


def iou_distance_3d(a_tracks, b_tracks):
    """
    Compute IoU distance between tracks and detections
    """
    a_boxes = [track.tlbr for track in a_tracks]
    b_boxes = [track.tlbr for track in b_tracks]
    
    return 1 - compute_iou_3d(a_boxes, b_boxes)  # cost matrix

def motion_distance(tracks, detections):
    """
    모션 정보를 이용하여 비용 행렬 계산
    """
    cost_matrix = numpy.zeros((len(tracks), len(detections)), dtype=numpy.float32)
    for i, track in enumerate(tracks):
        for j, det in enumerate(detections):
            
            # 속도 벡터의 코사인 유사도 계산
            track_vel = track.mean[6:9]
            det_vel = det.mean[6:9] if det.mean is not None else numpy.zeros(3)
            
            norm_track = numpy.linalg.norm(track_vel)
            norm_det = numpy.linalg.norm(det_vel)
            
            if norm_track > 0 and norm_det > 0:
                cos_sim = numpy.dot(track_vel, det_vel) / (norm_track * norm_det)
                cost = 1 - cos_sim  # 코사인 거리
            else:
                cost = 1.0
            cost_matrix[i, j] = cost
    return cost_matrix

def appearance_distance(tracks, detections):
        cost_matrix = numpy.zeros((len(tracks), len(detections)), dtype=numpy.float32)
        for i, track in enumerate(tracks):
            for j, det in enumerate(detections):
                # 코사인 유사도 등으로 외관 거리 계산
                feature_distance = 1 - numpy.dot(track.feature, det.feature) / (
                    numpy.linalg.norm(track.feature) * numpy.linalg.norm(det.feature) + 1e-6)
                cost_matrix[i, j] = feature_distance
        return cost_matrix

def fuse_score(cost_matrix, detections):
    if cost_matrix.size == 0:
        return cost_matrix
    iou_sim = 1 - cost_matrix
    det_scores = numpy.array([det.score for det in detections])
    det_scores = numpy.expand_dims(det_scores, axis=0).repeat(cost_matrix.shape[0], axis=0)
    fuse_sim = iou_sim * det_scores
    return 1 - fuse_sim  # fuse_cost





class State:
    New = 0
    Tracked = 1
    Lost = 2
    Removed = 3

class Track:
    count = 0

    # 전역으로 공유하는 kalman
    shared_kalman = ExtendedKalmanFilterXYZWH()

    def __init__(self, tlwhdy, score, cls):
        """
        tlwhdy: [x, y, z, w, h, l, yaw, idx] 형태라 가정
        (마지막에 idx까지 붙어있다면, len=8)
        """
        # 기존에 tlbr_to_tlwhd를 썼다면, 여기서는 (x, y, z, w, h, l, yaw) 직접 보관 가능
        self._tlwhdy = numpy.asarray(tlwhdy[:-1], dtype=numpy.float32)  # 7D
        self.idx = tlwhdy[-1]

        self.kalman_filter = None
        self.mean, self.covariance = None, None
        self.is_activated = False
        self.track_id = None

        self.score = score
        self.cls = cls
        self.tracklet_len = 0
        self.history = []
        self.velocity = numpy.zeros(3)  # 이 부분도 yaw 속도까지 추적하려면 4D로 확장 가능
        self.alpha = 0.3

        self.hit_streak = 0

    def predict(self):
        mean_state = self.mean.copy()

        # Lost 상태 시, 속도·가속도 초기화
        if self.state != State.Tracked:
            ndim = self.kalman_filter.ndim  # 7
            mean_state[ndim:2*ndim] = 0  # 속도
            mean_state[2*ndim:] = 0      # 가속도
        self.mean, self.covariance = self.kalman_filter.predict(mean_state, self.covariance)

        

    @staticmethod
    def multi_predict(tracks):
        if len(tracks) == 0:
            return
        multi_mean = numpy.asarray([st.mean.copy() for st in tracks])
        multi_covariance = numpy.asarray([st.covariance for st in tracks])
        ndim = Track.shared_kalman.ndim  # 7

        for i, st in enumerate(tracks):
            if st.state != State.Tracked:
                multi_mean[i][ndim:2*ndim] = 0
                multi_mean[i][2*ndim:] = 0
        means, covs = Track.shared_kalman.multi_predict(multi_mean, multi_covariance)
        for i, (m, c) in enumerate(zip(means, covs)):
            tracks[i].mean = m
            tracks[i].covariance = c


    def activate(self, kalman_filter, frame_id):
        self.kalman_filter = kalman_filter
        self.track_id = self.next_id()

        # EKF 초기화: (x, y, z, w, h, l, yaw)
        self.mean, self.covariance = self.kalman_filter.initiate(self._tlwhdy)

        self.tracklet_len = 0
        self.state = State.Tracked
        self.frame_id = frame_id
        self.start_frame = frame_id
        self.is_activated = True
        self.hit_streak = 1

        

    def re_activate(self, new_track, frame_id, new_id=False):
        """
        재활성화 (기존 트랙 + 새 측정)
        """
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, new_track._tlwhdy
        )
        self.tracklet_len = 0
        self.state = State.Tracked
        self.is_activated = True
        self.frame_id = frame_id
        if new_id:
            self.track_id = self.next_id()
        self.score = new_track.score
        self.cls = new_track.cls
        self.idx = new_track.idx
        self.hit_streak = 1


    def update(self, new_track, frame_id):
        """
        매칭된 detection으로 트랙 업데이트
        """
        self.frame_id = frame_id
        self.tracklet_len += 1

        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, new_track._tlwhdy
        )
        self.state = State.Tracked
        self.is_activated = True

        self.score = new_track.score
        self.cls = new_track.cls
        self.idx = new_track.idx

        self.hit_streak += 1

        # 위치 기록
        self.history.append(self.mean[:self.kalman_filter.ndim])
        if len(self.history) > 5:
            self.history.pop(0)

        # 3D 위치만으로 속도 추정 (yaw 속도를 따로 계산하려면 추가 로직 필요)
        if len(self.history) >= 2:
            pos_prev = self.history[-2][:3]
            pos_curr = self.history[-1][:3]
            delta = pos_curr - pos_prev
            delta_time = self.kalman_filter.dt
            raw_v = delta / delta_time
            self.velocity = self.alpha * raw_v + (1 - self.alpha) * self.velocity

        

    def mark_lost(self):
        self.state = State.Lost
        self.hit_streak = 0

    def mark_removed(self):
        self.state = State.Removed
        self.hit_streak = 0

    def mark_missed(self):
        self.hit_streak = 0

    @property
    def end_frame(self):
        return self.frame_id

    @staticmethod
    def next_id():
        Track.count += 1
        return Track.count

    @staticmethod
    def reset_id():
        Track.count = 0

    @property
    def tlwhdy(self):
        """
        현재 상태 mean으로부터 [x, y, z, w, h, l, yaw] 가져오기
        """
        if self.mean is None:
            return self._tlwhdy.copy()
        return self.mean[:self.kalman_filter.ndim].copy()

    @property
    def tlbr(self):
        """
        x,y,z,w,h,l (yaw 무시) 로부터
        (x1,y1,z1,x2,y2,z2) 형태 반환
        """
        if self.mean is None:
            # 아직 kalman_filter 초기화 전이면 _tlwhdy 사용
            box_7d = self._tlwhdy
        else:
            box_7d = self.tlwhdy  # self.mean[:7] or similar
        x, y, z, w, h, l, yaw = box_7d

        # 여기서는 yaw를 무시하고 축에 평행한 박스를 가정
        x1 = x - w/2
        x2 = x + w/2
        y1 = y - h/2
        y2 = y + h/2
        z1 = z - l/2
        z2 = z + l/2
        return numpy.array([x1, y1, z1, x2, y2, z2], dtype=numpy.float32)
    
    @property
    def state(self):
        return getattr(self, "_state", State.New)

    @state.setter
    def state(self, val):
        self._state = val

    def __repr__(self):
        return f'Track_{self.track_id}_({self.start_frame}-{self.end_frame})'
    


    


class BYTETracker:
    def __init__(self, frame_rate=10):
        """
        frame_rate: 초당 프레임 수
        """
        self.tracked_tracks = []
        self.lost_tracks = []
        self.removed_tracks = []

        self.frame_id = 0
        # 최대 lost 유지 시간(예: 90 프레임 * frame_rate). 필요에 따라 조정.
        self.max_time_lost = int(frame_rate * 1)

        # yaw를 포함한 확장 칼만 필터
        self.kalman_filter = ExtendedKalmanFilterXYZWH()

        # 트랙 ID를 1부터 다시 세도록 초기화
        self.reset_id()


    def update(self, boxes, scores, object_classes):
        """
        매 프레임마다 새로운 검출(바운딩 박스, 점수, 클래스, 검출 인덱스)을 받아 추적을 갱신한다.
        
        boxes: shape (N, 7 or 8)
          - [x, y, z, w, h, l, yaw] (+ idx) 형태
          - 만약 idx(검출 인덱스)를 마지막에 붙였다면 shape (N, 8).
          - 예: [x, y, z, w, h, l, yaw, idx]
          
        scores: shape (N,)
          - 각 검출 bbox에 대한 confidence score

        object_classes: shape (N,)
          - 각 검출 bbox에 대한 클래스 (예: 1.0=Car, 2.0=Pedestrian 등)
        """
        self.frame_id += 1
        activated_tracks = []
        re_find_tracks = []
        lost_tracks = []
        removed_tracks = []

        # ----------------------------------------------------------------------
        # 1) 검출 결과를 Track 객체 리스트로 초기화
        #    boxes에 idx가 없는 경우(7차원)라면, 여기서 별도 concatenation 필요
        detections = self.init_track(boxes, scores, object_classes)

        # 현재 활성화된(이전에 추적 중이었던) 트랙들 중
        # 아직 확실히 활성화되지 않은(unconfirmed) vs 확실히 활성화된(tracked) 구분
        unconfirmed = []
        tracked_stracks = []
        for track in self.tracked_tracks:
            if not track.is_activated:
                unconfirmed.append(track)
            else:
                tracked_stracks.append(track)

        # ----------------------------------------------------------------------
        # 2) 기존 추적 중(tracked) + Lost 상태 트랙을 합쳐서 현재 프레임에서 예측
        track_pool = self.joint_stracks(tracked_stracks, self.lost_tracks)
        self.multi_predict(track_pool)


        # ----------------------------------------------------------------------
        # 3) 첫 번째 연관 (High-score detection과의 매칭) -> influence ! 
        #    cost matrix 계산(예: iou_distance + 모션 거리 + score fuse 등)
        dists = self.get_dists(track_pool, detections)
        matches, u_track, u_detection = self.linear_assignment(dists, thresh=1.2) 

        for t_idx, d_idx in matches:
            track = track_pool[t_idx]
            det = detections[d_idx]
            if track.state == State.Tracked:
                # 이미 Tracked 상태였던 경우 -> update
                track.update(det, self.frame_id)
                activated_tracks.append(track)
            else:
                # Lost였던 경우 -> re_activate
                track.re_activate(det, self.frame_id, new_id=False)
                re_find_tracks.append(track)

        # u_track: 할당되지 못한 track 인덱스
        # u_detection: 할당되지 못한 detection 인덱스

        # ----------------------------------------------------------------------
        # 4) 두 번째 연관 (Low-score detection과의 매칭, 보행자/소형 물체 등)
        #    실제로는 점수 기준 필터링하거나 IoU 임계값을 낮춰 재연결하는 등 가능
        #    여기서는 예시로 iou_distance_3d를 사용한다고 가정
        #    => 필요 시 detection을 따로 분리하여 두 번째 matching 진행
        detections_second = []  # 예: 낮은 점수 검출만 골라내는 로직
        # 여기서는 예시로 [] 처리. 실제론 아래처럼 하거나, 
        # boxes_second = boxes[indices_second], scores_second = scores[indices_second], etc.

        r_tracked_tracks = [track_pool[i] for i in u_track if track_pool[i].state == State.Tracked]
        dists_second = self.get_dists(r_tracked_tracks, detections_second)
        matches_second, u_track_second, u_detection_second = self.linear_assignment(dists_second, thresh=0.5)

        for t_idx, d_idx in matches_second:
            track = r_tracked_tracks[t_idx]
            det = detections_second[d_idx]
            if track.state == State.Tracked:
                track.update(det, self.frame_id)
                activated_tracks.append(track)
            else:
                track.re_activate(det, self.frame_id, new_id=False)
                re_find_tracks.append(track)

        # 두 번째 매칭에서도 연결되지 못한 트랙은 Lost로 전환
        for it in u_track_second:
            track = r_tracked_tracks[it]
            if track.state != State.Lost:
                track.mark_lost()
                lost_tracks.append(track)

        # ----------------------------------------------------------------------
        # 5) unconfirmed(아직 완전히 활성화되지 않은) 트랙과 할당되지 않은 detection 매칭
        u_det_final = [detections[i] for i in u_detection]
        dists_unconfirmed = self.get_dists(unconfirmed, u_det_final)
        matches_unconfirmed, u_unconfirmed, u_detection_final = self.linear_assignment(dists_unconfirmed, thresh=0.7)

        for t_idx, d_idx in matches_unconfirmed:
            unconfirmed[t_idx].update(u_det_final[d_idx], self.frame_id)
            activated_tracks.append(unconfirmed[t_idx])
        for it in u_unconfirmed:
            track = unconfirmed[it]
            track.mark_removed()
            removed_tracks.append(track)


        # ----------------------------------------------------------------------
        # 6) 새로운 트랙(High-score 등) 활성화
        #    u_detection_final = 아직 매칭 안 된 detection들
        for new_i in u_detection_final:
            track = u_det_final[new_i]
            # 특정 클래스별로 점수 임계값 다르게 처리 가능
            if track.cls == 'Pedestrian':
                score_threshold = 0.7
            else:
                score_threshold = 0.6     # 0.6

            if track.score < score_threshold:
                continue
            track.activate(self.kalman_filter, self.frame_id)
            activated_tracks.append(track)

        # ----------------------------------------------------------------------
        # 7) Lost 상태 오래 지속된 트랙은 Removed
        for track in self.lost_tracks:
            if self.frame_id - track.end_frame > self.max_time_lost:
                track.mark_removed()
                removed_tracks.append(track)


        # ----------------------------------------------------------------------
        # hit_streak 등으로 최종 활성화 여부 결정
        for track in activated_tracks:
            if track.hit_streak >= 1:
                track.is_activated = True
            else:
                track.is_activated = False

        for track in re_find_tracks:
            if track.hit_streak >= 1:
                track.is_activated = True
            else:
                track.is_activated = False

        # ----------------------------------------------------------------------
        # tracked_tracks 리스트 갱신
        self.tracked_tracks = [t for t in self.tracked_tracks if t.state == State.Tracked]
        self.tracked_tracks = self.joint_stracks(self.tracked_tracks, activated_tracks)
        self.tracked_tracks = self.joint_stracks(self.tracked_tracks, re_find_tracks)

        self.lost_tracks = self.sub_stracks(self.lost_tracks, self.tracked_tracks)
        self.lost_tracks.extend(lost_tracks)
        self.lost_tracks = self.sub_stracks(self.lost_tracks, removed_tracks)
        self.removed_tracks.extend(removed_tracks)

        # 중복 트랙 제거(필요할 경우 IoU 기반 중복 제거)
        self.tracked_tracks, self.lost_tracks = self.remove_duplicate_stracks(self.tracked_tracks, self.lost_tracks)

        # ----------------------------------------------------------------------
        # 8) 최종 추적 결과 구성: 3D 바운딩 박스를 코너 좌표 + 트랙 ID + 속도 등으로 정리
        outputs = []
        for track in self.tracked_tracks:
            if track.is_activated:
                # track.tlwhdy: [x, y, z, w, h, l, yaw]
                box_7d = track.tlwhdy
                x, y, z, w, h, l, yaw = box_7d

                # 여기서는 코너 좌표(x1,y1,z1,x2,y2,z2)로 변환
                x1 = x - w/2
                y1 = y - h/2
                z1 = z - l/2
                x2 = x + w/2
                y2 = y + h/2
                z2 = z + l/2

                # EKF 상태 벡터에서 속도는 mean[ndim:ndim+3]라 가정(3D vx,vy,vz)
                # vx = track.mean[self.kalman_filter.ndim]
                # vy = track.mean[self.kalman_filter.ndim + 1]
                # vz = track.mean[self.kalman_filter.ndim + 2]

                # self.history를 통해 연속된 위치 이력을 바탕으로 프레임 단위로 직접 계산한 속도를 알파 필터(가중치)로 업데이트한 속도 값(3D vx,vy,vz)
                vx = track.velocity[0]
                vy = track.velocity[1]
                vz = track.velocity[2]

                # 필요하면 yaw 속도(track.mean[self.kalman_filter.ndim + 6] 등)도 사용 가능
                outputs.append([
                    x1, y1, z1, x2, y2, z2,           # 3D 코너
                    track.track_id,                   # 추적 ID
                    track.score,                      # confidence
                    track.cls,                        # 클래스
                    track.idx,                        # detection idx
                    vx, vy, vz,                       # 추정 속도
                    yaw                                # 추가로 yaw을 출력(원한다면)
                ])
        return numpy.asarray(outputs, dtype=numpy.float32)

    @staticmethod
    def init_track(boxes, scores, cls):
        return [Track(box, s, c) for (box, s, c) in zip(boxes, scores, cls)] if len(boxes) else []  # detections

    
    @staticmethod
    def get_dists(tracks, detections):
        """
        tracks와 detections 간의 총 비용 행렬을 계산.
        1) 3D IoU distance
        2) 모션 distance
        3) fuse_score를 이용해 점수 반영
        등 을 합쳐 최종 cost_matrix를 만든다.
        """
        # 3D IoU 거리 계산
        iou_cost = iou_distance_3d(tracks, detections)    # shape (len(tracks), len(detections))

        # 모션 거리 계산
        motion_cost = motion_distance(tracks, detections) # shape 동일

        # 외형 특징 계산
        # appearance_cost = appearance_distance(tracks, detections)

        # 가중합 (예: IoU 0.75, 모션 0.25)
        total_cost = 0.75 * iou_cost + 0.25 * motion_cost
        # total_cost = 0.5 * total_cost + 0.5 * appearance_cost

        # 검출 점수를 활용해 cost를 조정
        # (검출 점수가 높으면 matching에 이점, 점수가 낮으면 불이익)
        total_cost = fuse_score(total_cost, detections)   # fuse_score 내부에서 det.score 등을 사용

        return total_cost

    @staticmethod
    def reset_id():
        Track.reset_id()
    
    @staticmethod
    def multi_predict(tracks):
        """
        여러 Track에 대해 Kalman predict를 일괄 수행
        """
        Track.multi_predict(tracks)

    @staticmethod
    def joint_stracks(tlista, tlistb):
        """
        두 리스트를 합치되, track_id가 중복되지 않도록 병합
        """
        exists = {}
        res = []
        for t in tlista:
            exists[t.track_id] = True
            res.append(t)
        for t in tlistb:
            if not exists.get(t.track_id, False):
                exists[t.track_id] = True
                res.append(t)
        return res

    @staticmethod
    def sub_stracks(tlista, tlistb):
        """
        tlistb에 있는 트랙을 tlista에서 제외
        """
        stracks = {t.track_id: t for t in tlista}
        for t in tlistb:
            tid = t.track_id
            if tid in stracks:
                del stracks[tid]
        return list(stracks.values())

    @staticmethod
    def remove_duplicate_stracks(stracksa, stracksb):
        pdist = iou_distance_3d(stracksa, stracksb)
        pairs = numpy.where(pdist < 0.15)
        dupa, dupb = [], []
        for p, q in zip(*pairs):
            timep = stracksa[p].frame_id - stracksa[p].start_frame
            timeq = stracksb[q].frame_id - stracksb[q].start_frame
            if timep > timeq:
                dupb.append(q)
            else:
                dupa.append(p)
        resa = [t for i, t in enumerate(stracksa) if i not in dupa]
        resb = [t for i, t in enumerate(stracksb) if i not in dupb]
        return resa, resb
    
    @staticmethod
    def linear_assignment(cost_matrix, thresh, use_lap=True):
        """
        tracks-detections 간 비용 행렬(cost_matrix)에 대해
        선형 할당(최적의 match)을 수행한다.

        - cost_matrix: shape (num_tracks, num_detections)
        - thresh: cost_limit(매칭 임계값). 
        lap.lapjv()의 cost_limit 파라미터이자, 
        scipy.optimize.linear_sum_assignment()에서 사용할 때는 cost_matrix[i,x] <= thresh 인 match만 허용.
        - use_lap: True면 lap.lapjv, False면 scipy.optimize.linear_sum_assignment 사용

        return:
        matches: [[track_idx, detection_idx], ...]의 리스트
        unmatched_a: 매칭되지 못한 track 인덱스 배열
        unmatched_b: 매칭되지 못한 detection 인덱스 배열
        """
        # 만약 cost_matrix가 비어있으면 바로 반환
        if cost_matrix.size == 0:
            matches = numpy.empty((0, 2), dtype=int)
            unmatched_a = tuple(range(cost_matrix.shape[0]))
            unmatched_b = tuple(range(cost_matrix.shape[1]))
            return matches, unmatched_a, unmatched_b

        if use_lap:
            # lap.lapjv를 사용하는 경우
            # cost_limit=thresh -> 이 값보다 큰 비용이면 매칭 제외
            _, x, y = lap.lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)

            # x: row(트랙)별 할당된 col(디텍션)의 인덱스(또는 -1)
            matches = []
            for track_idx, detection_idx in enumerate(x):
                if detection_idx >= 0:  # 매칭 성공
                    matches.append([track_idx, detection_idx])

            # unmatched
            unmatched_a = numpy.where(x < 0)[0]  # 매칭 실패한 row(트랙)
            unmatched_b = numpy.where(y < 0)[0]  # 매칭 실패한 col(디텍션)
            return matches, unmatched_a, unmatched_b

        else:
            # scipy.optimize.linear_sum_assignment 이용
            # linear_sum_assignment는 cost_matrix 전체를 대상으로 global min-cost match 수행
            # 반환: row index 배열(y), col index 배열(x)
            from scipy.optimize import linear_sum_assignment
            y, x = linear_sum_assignment(cost_matrix)  # row y, col x

            # 비용이 임계값 이하인 쌍만 선택
            valid_matches = []
            for row_idx, col_idx in zip(y, x):
                if cost_matrix[row_idx, col_idx] <= thresh:
                    valid_matches.append([row_idx, col_idx])

            matches = numpy.asarray(valid_matches, dtype=int)

            # unmatched 계산
            unmatched = numpy.ones(cost_matrix.shape, dtype=bool)
            for row_idx, col_idx in matches:
                unmatched[row_idx, col_idx] = False
            unmatched_a = numpy.where(unmatched.all(axis=1))[0]  # 모든 col에서 True => row 미매칭
            unmatched_b = numpy.where(unmatched.all(axis=0))[0]  # 모든 row에서 True => col 미매칭

            return matches, unmatched_a, unmatched_b
