from collections import namedtuple
import glob
import os
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

BagMessage = namedtuple("BagMessage", ["topic", "msg", "stamp_sec"])


def _message_stamp_sec(msg, fallback_stamp_ns):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_stamp_ns * 1.0e-9
    sec = getattr(stamp, "sec", 0)
    nanosec = getattr(stamp, "nanosec", 0)
    return float(sec) + float(nanosec) * 1.0e-9


def iter_deserialized_messages(bag_uri, topics=None, storage_id="sqlite3", use_header_stamp=True):
    try:
        import rosbag2_py
    except ImportError as exc:  # pragma: no cover - ROS environment dependent
        yield from _iter_sqlite_messages(
            bag_uri,
            topics=topics,
            storage_id=storage_id,
            use_header_stamp=use_header_stamp,
        )
        return

    selected_topics = set(topics) if topics else None
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    type_by_topic = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
        if selected_topics is None or topic.name in selected_topics
    }
    msg_type_cache = {}

    while reader.has_next():
        topic_name, serialized_data, stamp_ns = reader.read_next()
        if selected_topics is not None and topic_name not in selected_topics:
            continue
        if topic_name not in type_by_topic:
            continue
        if topic_name not in msg_type_cache:
            msg_type_cache[topic_name] = get_message(type_by_topic[topic_name])
        msg = deserialize_message(serialized_data, msg_type_cache[topic_name])
        stamp_sec = _message_stamp_sec(msg, stamp_ns) if use_header_stamp else stamp_ns * 1.0e-9
        yield BagMessage(topic=topic_name, msg=msg, stamp_sec=stamp_sec)


def _sqlite_db_path(bag_uri):
    if os.path.isfile(bag_uri):
        return bag_uri

    metadata_path = os.path.join(bag_uri, "metadata.yaml")
    if os.path.exists(metadata_path):
        with open(metadata_path, "r") as f:
            metadata = yaml.safe_load(f) or {}
        bag_info = metadata.get("rosbag2_bagfile_information", {})
        relative_paths = bag_info.get("relative_file_paths") or []
        for relative_path in relative_paths:
            candidate = os.path.join(bag_uri, relative_path)
            if os.path.exists(candidate):
                return candidate

    candidates = sorted(glob.glob(os.path.join(bag_uri, "*.db3")))
    if candidates:
        return candidates[0]

    raise RuntimeError("No sqlite3 rosbag database found in %s" % bag_uri)


def _iter_sqlite_messages(bag_uri, topics=None, storage_id="sqlite3", use_header_stamp=True):
    if storage_id != "sqlite3":
        raise RuntimeError("rosbag2_py is unavailable; sqlite fallback only supports storage_id=sqlite3")

    selected_topics = set(topics) if topics else None
    db_path = _sqlite_db_path(bag_uri)
    connection = sqlite3.connect(db_path)
    try:
        topic_rows = connection.execute("select id, name, type from topics").fetchall()
        topic_by_id = {
            row[0]: (row[1], row[2])
            for row in topic_rows
            if selected_topics is None or row[1] in selected_topics
        }
        msg_type_cache = {}
        cursor = connection.execute(
            "select topic_id, timestamp, data from messages order by timestamp"
        )
        for topic_id, stamp_ns, data in cursor:
            if topic_id not in topic_by_id:
                continue
            topic_name, topic_type = topic_by_id[topic_id]
            if topic_name not in msg_type_cache:
                msg_type_cache[topic_name] = get_message(topic_type)
            msg = deserialize_message(bytes(data), msg_type_cache[topic_name])
            stamp_sec = _message_stamp_sec(msg, stamp_ns) if use_header_stamp else stamp_ns * 1.0e-9
            yield BagMessage(topic=topic_name, msg=msg, stamp_sec=stamp_sec)
    finally:
        connection.close()
