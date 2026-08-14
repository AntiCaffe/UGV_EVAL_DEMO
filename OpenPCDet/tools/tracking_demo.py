"""Run OpenPCDet inference and 3D ByteTrack on an ordered point-cloud sequence."""

import argparse
import colorsys
import glob
import sys
from pathlib import Path

import numpy as np
import torch

try:
    import open3d
except ImportError:
    open3d = None

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PCDET_ROS2_SOURCE = REPOSITORY_ROOT / 'src' / 'pcdet_ros2'
if str(PCDET_ROS2_SOURCE) not in sys.path:
    sys.path.insert(0, str(PCDET_ROS2_SOURCE))

try:
    from pcdet_ros2.nn_3d import BYTETracker
except ImportError as exc:
    raise ImportError(
        'BYTETracker dependencies are unavailable. Build/source the ROS 2 workspace '
        'and install the Python package "lap" before running this script.'
    ) from exc

if open3d is not None:
    from visual_utils import open3d_vis_utils


class TrackingDataset(DatasetTemplate):
    """Load an individual point cloud or a filename-ordered frame directory."""

    def __init__(self, dataset_cfg, class_names, root_path, logger, ext='.bin'):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=False,
            root_path=root_path,
            logger=logger,
        )
        self.root_path = root_path
        self.ext = ext
        if self.root_path.is_dir():
            files = glob.glob(str(self.root_path / f'*{self.ext}'))
        else:
            files = [str(self.root_path)]
        self.sample_file_list = sorted(files)

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        file_path = self.sample_file_list[index]
        if self.ext == '.bin':
            points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
        elif self.ext == '.npy':
            points = np.load(file_path)
        else:
            raise NotImplementedError(f'Unsupported point-cloud extension: {self.ext}')

        return self.prepare_data({
            'points': points,
            'frame_id': index,
        })


def parse_config():
    parser = argparse.ArgumentParser(
        description='OpenPCDet demo with class-specific 3D ByteTrack'
    )
    parser.add_argument(
        '--cfg_file', type=str, default='cfgs/kitti_models/second.yaml',
        help='OpenPCDet model configuration file'
    )
    parser.add_argument(
        '--data_path', type=str, required=True,
        help='Point-cloud file or directory containing an ordered frame sequence'
    )
    parser.add_argument('--ckpt', type=str, required=True, help='Model checkpoint')
    parser.add_argument(
        '-m', '--mode', choices=('demo', 'infer', 'both'), default='demo',
        help='demo: visualize, infer: save TXT, both: visualize and save TXT'
    )
    parser.add_argument(
        '-o', '--output_dir', '--output_path', dest='output_dir', type=str,
        default='tracking_results',
        help='Directory for per-frame TXT results in infer/both mode'
    )
    parser.add_argument(
        '--ext', type=str, default='.bin', choices=('.bin', '.npy'),
        help='Point-cloud file extension'
    )
    parser.add_argument(
        '--track_classes', type=int, nargs='+', default=[2],
        help='One-based class IDs to track (default: 2, Pedestrian for KITTI)'
    )
    parser.add_argument(
        '--frame_rate', type=int, default=30,
        help='Input sequence frame rate passed to BYTETracker'
    )
    parser.add_argument(
        '--velocity_scale', type=float, default=1.0,
        help='Seconds used to scale velocity arrows (default: 1.0)'
    )
    parser.add_argument(
        '--no_visualization', action='store_true',
        help='Disable visualization (backward-compatible demo option)'
    )
    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg


def make_tracking_input(prediction, class_id):
    """Convert one OpenPCDet prediction into BYTETracker's 8-column input."""
    scores = prediction['pred_scores'].detach().cpu().numpy()
    labels = prediction['pred_labels'].detach().cpu().numpy()
    boxes = prediction['pred_boxes'].detach().cpu().numpy()[:, :7]

    selected = np.flatnonzero(labels == class_id)
    if selected.size == 0:
        return (
            np.empty((0, 8), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
        )

    tracking_boxes = np.column_stack((boxes[selected], selected)).astype(
        np.float32, copy=False
    )
    return (
        tracking_boxes,
        scores[selected].astype(np.float32, copy=False),
        labels[selected].astype(np.float32, copy=False),
    )


def tracking_outputs_to_boxes(outputs):
    """Convert BYTETracker output rows back to OpenPCDet box representation."""
    if outputs.size == 0:
        return np.empty((0, 7), dtype=np.float32)

    mins = outputs[:, 0:3]
    maxes = outputs[:, 3:6]
    centers = (mins + maxes) / 2.0
    dimensions = maxes - mins
    yaws = outputs[:, 13:14]
    boxes = np.concatenate((centers, dimensions, yaws), axis=1)

    # Keep the same visualization-only bottom-z correction used by demo.py.
    boxes[:, 2] += boxes[:, 5] / 2.0
    return boxes.astype(np.float32, copy=False)


def track_color(track_id):
    """Return a stable and visually distinct RGB color for a track ID."""
    hue = (int(track_id) * 0.618033988749895) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.8, 1.0)


def create_velocity_arrow(start, velocity, color, velocity_scale):
    """Create a line arrow showing the direction of a 3D velocity vector."""
    vector = np.asarray(velocity, dtype=np.float64) * velocity_scale
    length = float(np.linalg.norm(vector))
    if length < 1e-3:
        return None

    direction = vector / length
    reference = np.array([0.0, 0.0, 1.0])
    side = np.cross(direction, reference)
    if np.linalg.norm(side) < 1e-6:
        reference = np.array([0.0, 1.0, 0.0])
        side = np.cross(direction, reference)
    side /= np.linalg.norm(side)

    end = np.asarray(start, dtype=np.float64) + vector
    head_length = min(max(length * 0.25, 0.1), 0.5)
    head_width = head_length * 0.5
    head_base = end - direction * head_length
    points = np.vstack((
        start,
        end,
        head_base + side * head_width,
        head_base - side * head_width,
    ))
    lines = np.array([[0, 1], [1, 2], [1, 3]], dtype=np.int32)
    arrow = open3d.geometry.LineSet(
        points=open3d.utility.Vector3dVector(points),
        lines=open3d.utility.Vector2iVector(lines),
    )
    arrow.paint_uniform_color(color)
    return arrow


def add_tracking_labels(visualizer, boxes, track_ids, velocities):
    """Attach ID, position, and velocity text above each tracked box."""
    for box, track_id, velocity in zip(boxes, track_ids, velocities):
        x, y, z = map(float, box[0:3])
        vx, vy, vz = map(float, velocity)
        speed = float(np.linalg.norm(velocity))
        label_position = np.array(
            [x, y, z + float(box[5]) / 2.0 + 0.2], dtype=np.float32
        )
        label = (
            f'ID:{int(track_id)}  pos:({x:.2f}, {y:.2f}, {z:.2f})  '
            f'vel:({vx:.2f}, {vy:.2f}, {vz:.2f})  {speed:.2f} m/s'
        )
        visualizer.add_3d_label(label_position, label)


def draw_tracking_scene(
    points, boxes, track_ids, velocities, frame_index, velocity_scale
):
    """Show tracked boxes, using a stable color for each track ID."""
    if open3d is None:
        raise RuntimeError(
            'Open3D is required for visualization; use --no_visualization '
            'to print tracking results only.'
        )

    axis = open3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=[0, 0, 0]
    )

    point_cloud = open3d.geometry.PointCloud()
    point_cloud.points = open3d.utility.Vector3dVector(points[:, :3])
    point_cloud.colors = open3d.utility.Vector3dVector(
        np.full((points.shape[0], 3), 0.25, dtype=np.float64)
    )
    geometries = [axis, point_cloud]

    for box, track_id, velocity in zip(boxes, track_ids, velocities):
        color = track_color(track_id)
        line_set, _ = open3d_vis_utils.translate_boxes_to_open3d_instance(box)
        line_set.paint_uniform_color(color)
        geometries.append(line_set)

        arrow = create_velocity_arrow(
            start=box[0:3],
            velocity=velocity,
            color=color,
            velocity_scale=velocity_scale,
        )
        if arrow is not None:
            geometries.append(arrow)

    def initialize_labels(visualizer):
        add_tracking_labels(visualizer, boxes, track_ids, velocities)

    open3d.visualization.draw(
        geometry=geometries,
        title=f'OpenPCDet tracking - frame {frame_index}',
        # Open3D 0.13 renders O3DVisualizer labels in black and does not
        # expose their color, so a light background is required for contrast.
        bg_color=(0.95, 0.95, 0.95, 1.0),
        point_size=1,
        on_init=initialize_labels,
    )


def print_tracking_results(logger, frame_index, frame_path, detections, outputs, class_names):
    logger.info(
        f'Frame {frame_index + 1}: {frame_path} | '
        f'detections={detections}, active_tracks={len(outputs)}'
    )
    for result in outputs:
        track_id = int(result[6])
        score = float(result[7])
        class_id = int(result[8])
        vx, vy, vz = result[10:13]
        speed = float(np.linalg.norm([vx, vy, vz]))
        class_name = (
            class_names[class_id - 1]
            if 1 <= class_id <= len(class_names)
            else f'class_{class_id}'
        )
        logger.info(
            f'  ID={track_id} class={class_name} score={score:.3f} '
            f'velocity=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s '
            f'speed={speed:.2f} m/s'
        )


def format_track_row(frame_number, frame_file, result, class_names):
    """Format one BYTETracker output as a tab-separated text row."""
    x1, y1, z1, x2, y2, z2 = result[0:6]
    track_id = int(result[6])
    score = float(result[7])
    class_id = int(result[8])
    detection_index = int(result[9])
    vx, vy, vz = result[10:13]
    yaw = float(result[13])
    speed = float(np.linalg.norm([vx, vy, vz]))
    class_name = (
        class_names[class_id - 1]
        if 1 <= class_id <= len(class_names)
        else f'class_{class_id}'
    )
    values = [
        frame_number, Path(frame_file).name, track_id, class_id, class_name,
        f'{score:.6f}', f'{x1:.6f}', f'{y1:.6f}', f'{z1:.6f}',
        f'{x2:.6f}', f'{y2:.6f}', f'{z2:.6f}', f'{yaw:.6f}',
        f'{vx:.6f}', f'{vy:.6f}', f'{vz:.6f}', f'{speed:.6f}',
        detection_index,
    ]
    return '\t'.join(map(str, values))


def prepare_output_directory(args):
    """Create the optional directory for per-frame tracking results."""
    if args.mode not in ('infer', 'both'):
        return None

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def write_frame_result(
    output_dir, args, frame_number, frame_path, detection_count, outputs,
    class_names
):
    """Write one point-cloud frame and its active tracks to one TXT file."""
    output_path = output_dir / f'{Path(frame_path).stem}.txt'
    with output_path.open('w', encoding='utf-8') as output_file:
        output_file.write('# OpenPCDet ByteTrack inference results\n')
        output_file.write(f'# cfg_file: {args.cfg_file}\n')
        output_file.write(f'# checkpoint: {args.ckpt}\n')
        output_file.write(f'# frame: {frame_number}\n')
        output_file.write(f'# frame_file: {Path(frame_path).name}\n')
        output_file.write(f'# detections: {detection_count}\n')
        output_file.write(f'# active_tracks: {len(outputs)}\n')
        output_file.write(
            '# columns: frame frame_file track_id class_id class_name score '
            'x1 y1 z1 x2 y2 z2 yaw vx vy vz speed detection_index\n'
        )
        for result in outputs:
            output_file.write(
                format_track_row(
                    frame_number, frame_path, result, class_names
                ) + '\n'
            )
    return output_path


def main():
    args, model_cfg = parse_config()
    logger = common_utils.create_logger()
    logger.info(
        f'---------------OpenPCDet 3D tracking ({args.mode})---------------'
    )

    if args.frame_rate <= 0:
        raise ValueError('--frame_rate must be greater than zero')
    if args.velocity_scale <= 0:
        raise ValueError('--velocity_scale must be greater than zero')

    invalid_classes = [
        class_id for class_id in args.track_classes
        if class_id < 1 or class_id > len(model_cfg.CLASS_NAMES)
    ]
    if invalid_classes:
        raise ValueError(
            f'Invalid --track_classes values {invalid_classes}; valid IDs are '
            f'1..{len(model_cfg.CLASS_NAMES)} for {list(model_cfg.CLASS_NAMES)}'
        )

    data_path = Path(args.data_path)
    if not data_path.exists():
        raise FileNotFoundError(f'Point-cloud path does not exist: {data_path}')

    dataset = TrackingDataset(
        dataset_cfg=model_cfg.DATA_CONFIG,
        class_names=model_cfg.CLASS_NAMES,
        root_path=data_path,
        ext=args.ext,
        logger=logger,
    )
    if not dataset.sample_file_list:
        raise FileNotFoundError(f'No {args.ext} files found at {data_path}')
    logger.info(f'Total number of frames: {len(dataset)}')
    if len(dataset) == 1:
        logger.warning(
            'Only one frame was provided. Use a directory of ordered frames '
            'to verify temporal tracking.'
        )

    model = build_network(
        model_cfg=model_cfg.MODEL,
        num_class=len(model_cfg.CLASS_NAMES),
        dataset=dataset,
    )
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda().eval()

    trackers = {
        class_id: BYTETracker(frame_rate=args.frame_rate)
        for class_id in sorted(set(args.track_classes))
    }
    output_dir = prepare_output_directory(args)
    total_track_rows = 0
    visualize = args.mode in ('demo', 'both') and not args.no_visualization

    with torch.no_grad():
        for frame_index, sample in enumerate(dataset):
            batch = dataset.collate_batch([sample])
            load_data_to_gpu(batch)
            predictions, _ = model.forward(batch)
            prediction = predictions[0]

            frame_outputs = []
            tracked_detection_count = 0
            for class_id, tracker in trackers.items():
                boxes, scores, labels = make_tracking_input(prediction, class_id)
                tracked_detection_count += len(boxes)
                outputs = tracker.update(boxes, scores, labels)
                if outputs.size:
                    frame_outputs.append(outputs)

            outputs = (
                np.concatenate(frame_outputs, axis=0)
                if frame_outputs
                else np.empty((0, 14), dtype=np.float32)
            )
            if len(outputs):
                outputs = outputs[np.argsort(outputs[:, 6])]

            frame_path = dataset.sample_file_list[frame_index]
            print_tracking_results(
                logger, frame_index, frame_path, tracked_detection_count,
                outputs, model_cfg.CLASS_NAMES
            )

            if output_dir is not None:
                write_frame_result(
                    output_dir, args, frame_index + 1, frame_path,
                    tracked_detection_count, outputs, model_cfg.CLASS_NAMES
                )
                total_track_rows += len(outputs)

            if visualize:
                tracked_boxes = tracking_outputs_to_boxes(outputs)
                track_ids = outputs[:, 6].astype(np.int64, copy=False)
                velocities = outputs[:, 10:13]
                points = batch['points'][:, 1:].detach().cpu().numpy()
                draw_tracking_scene(
                    points, tracked_boxes, track_ids, velocities,
                    frame_index + 1, args.velocity_scale
                )

    if output_dir is not None:
        logger.info(
            f'Saved {total_track_rows} track rows from {len(dataset)} frames '
            f'as per-frame TXT files in {output_dir.resolve()}'
        )
    logger.info('Tracking done.')


if __name__ == '__main__':
    main()
