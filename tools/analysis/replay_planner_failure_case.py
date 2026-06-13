#!/usr/bin/env python3
"""
FastNav planner failure case 离线 RViz 重放工具。

PlannerDebugRecorder 会把一次 MINCO / fine check 失败现场保存到 tools/debug_cases/fail_xxx。
本脚本读取该目录下的 PCD / CSV 文件，并重新发布为 ROS 可视化 topic：
1. cloud_filtered / occupied / inflated 发布为 PointCloud2；
2. frontend / reference / shortcut / sampled 发布为 nav_msgs/Path；
 3. safe corridor 半空间 $n^T x + d \le 0$ 会转换为线框 MarkerArray；
4. searched_nodes 发布为 PointCloud2。
运行时可在终端按键浏览 case：
  s: 下一个 failure case
  w: 上一个 failure case
  r: 重新加载当前 case
  q: 退出

使用方式：
  roscore
  source /home/shukun/Project/FastNav/ros1_ws/devel/setup.bash
  python3 tools/analysis/replay_planner_failure_case.py --case latest

RViz:
  Fixed Frame = odom
  添加本脚本打印出的 /fastnav/debug_case/... topic。
"""

import argparse
import csv
import itertools
import math
import os
import select
import site
import struct
import sys
import termios
import tty

# 避免用户 site-packages 中的 NumPy 2.x 干扰 ROS Noetic 的 Python 环境。
if os.environ.get("FASTNAV_ALLOW_USER_SITE", "0") != "1":
    user_site = site.getusersitepackages()
    sys.path = [p for p in sys.path if p != user_site]

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def list_case_dirs(root_dir):
    if not os.path.isdir(root_dir):
        raise RuntimeError("debug case root does not exist: {}".format(root_dir))

    cases = [
        os.path.join(root_dir, name)
        for name in os.listdir(root_dir)
        if name.startswith("fail_") and os.path.isdir(os.path.join(root_dir, name))
    ]
    if not cases:
        raise RuntimeError("no fail_* case found in {}".format(root_dir))
    cases.sort()
    return cases


def latest_case_dir(root_dir):
    return list_case_dirs(root_dir)[-1]


def resolve_case(case_arg, root_dir):
    if case_arg == "latest":
        return latest_case_dir(root_dir)
    if os.path.isabs(case_arg):
        return case_arg
    candidate = os.path.join(root_dir, case_arg)
    return candidate if os.path.isdir(candidate) else case_arg


def find_case_index(cases, case_dir):
    abs_case = os.path.abspath(case_dir)
    for i, case in enumerate(cases):
        if os.path.abspath(case) == abs_case:
            return i
    return -1


def read_xyz_csv(path):
    points = []
    if not os.path.exists(path):
        return points

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                points.append((float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, ValueError):
                continue
    return points


def read_corridor_csv(path):
    corridors = {}
    if not os.path.exists(path):
        return []

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                cid = int(row["corridor_id"])
                plane = (
                    float(row["nx"]),
                    float(row["ny"]),
                    float(row["nz"]),
                    float(row["d"]),
                )
            except (KeyError, ValueError):
                continue
            corridors.setdefault(cid, []).append(plane)
    return [corridors[k] for k in sorted(corridors.keys())]


def read_pcd_xyz(path):
    if not os.path.exists(path):
        return []

    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                return []
            header_lines.append(line.decode("utf-8", errors="ignore").strip())
            if line.startswith(b"DATA"):
                break

        header = {}
        for line in header_lines:
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]

        fields = header.get("FIELDS", [])
        sizes = [int(x) for x in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
        points_num = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_type = header.get("DATA", [""])[0].lower()

        if fields[:3] != ["x", "y", "z"] or sizes[:3] != [4, 4, 4] or types[:3] != ["F", "F", "F"]:
            raise RuntimeError("only x/y/z float32 PCD is supported: {}".format(path))

        point_step = sum(size * count for size, count in zip(sizes, counts))
        xyz = []

        if data_type == "ascii":
            text = f.read().decode("utf-8", errors="ignore").splitlines()
            for line in text:
                parts = line.split()
                if len(parts) >= 3:
                    xyz.append((float(parts[0]), float(parts[1]), float(parts[2])))
            return xyz

        if data_type != "binary":
            raise RuntimeError("unsupported PCD DATA type: {}".format(data_type))

        raw = f.read(points_num * point_step)
        for i in range(points_num):
            offset = i * point_step
            x, y, z = struct.unpack_from("fff", raw, offset)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xyz.append((x, y, z))
        return xyz


def make_cloud(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, points)


def make_path(points, frame_id):
    msg = Path()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    for x, y, z in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def det3(a):
    return (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )


def solve3(a, b):
    d = det3(a)
    if abs(d) < 1.0e-9:
        return None
    ax = [[b[0], a[0][1], a[0][2]], [b[1], a[1][1], a[1][2]], [b[2], a[2][1], a[2][2]]]
    ay = [[a[0][0], b[0], a[0][2]], [a[1][0], b[1], a[1][2]], [a[2][0], b[2], a[2][2]]]
    az = [[a[0][0], a[0][1], b[0]], [a[1][0], a[1][1], b[1]], [a[2][0], a[2][1], b[2]]]
    return (det3(ax) / d, det3(ay) / d, det3(az) / d)


def corridor_vertices(planes):
    # 每个平面为 $n^T x + d <= 0$。三平面相交得到候选顶点，
    # 再用所有半空间过滤，得到凸多面体顶点。
    vertices = []
    active_sets = []
    eps = 1.0e-5

    for ids in itertools.combinations(range(len(planes)), 3):
        a = []
        b = []
        for idx in ids:
            nx, ny, nz, d = planes[idx]
            a.append([nx, ny, nz])
            b.append(-d)
        p = solve3(a, b)
        if p is None:
            continue

        inside = True
        active = set()
        for j, plane in enumerate(planes):
            nx, ny, nz, d = plane
            value = nx * p[0] + ny * p[1] + nz * p[2] + d
            if value > eps:
                inside = False
                break
            if abs(value) < 2.0e-4:
                active.add(j)
        if not inside:
            continue

        duplicate = False
        for q in vertices:
            if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 < 1.0e-8:
                duplicate = True
                break
        if not duplicate:
            vertices.append(p)
            active_sets.append(active)

    return vertices, active_sets


def make_corridor_markers(corridors, frame_id):
    markers = MarkerArray()

    delete_all = Marker()
    delete_all.action = Marker.DELETEALL
    markers.markers.append(delete_all)

    marker_id = 0
    for cid, planes in enumerate(corridors):
        vertices, active_sets = corridor_vertices(planes)
        if len(vertices) < 2:
            continue

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = "failure_safe_corridor"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.85

        for i in range(len(vertices)):
            for j in range(i + 1, len(vertices)):
                # 两个顶点若共享至少两个激活平面，一般就是多面体的一条边。
                if len(active_sets[i].intersection(active_sets[j])) < 2:
                    continue
                p0 = Point(*vertices[i])
                p1 = Point(*vertices[j])
                marker.points.append(p0)
                marker.points.append(p1)

        if marker.points:
            markers.markers.append(marker)

    return markers


def print_meta(case_dir):
    path = os.path.join(case_dir, "meta.yaml")
    if not os.path.exists(path):
        return
    print("\n========== meta.yaml ==========")
    with open(path, "r") as f:
        for line in f:
            print(line.rstrip())
    print("================================\n")


def load_case_data(case_dir, frame_id):
    print_meta(case_dir)
    data = {
        "cloud_filtered": make_cloud(read_pcd_xyz(os.path.join(case_dir, "cloud_filtered.pcd")), frame_id),
        "occupied": make_cloud(read_pcd_xyz(os.path.join(case_dir, "debug_occupied_cloud.pcd")), frame_id),
        "inflated": make_cloud(read_pcd_xyz(os.path.join(case_dir, "debug_inflated_cloud.pcd")), frame_id),
        "searched": make_cloud(read_xyz_csv(os.path.join(case_dir, "searched_nodes.csv")), frame_id),
        "frontend": make_path(read_xyz_csv(os.path.join(case_dir, "frontend_path.csv")), frame_id),
        "reference": make_path(read_xyz_csv(os.path.join(case_dir, "optimization_reference_path.csv")), frame_id),
        "shortcut": make_path(read_xyz_csv(os.path.join(case_dir, "shortcut_path.csv")), frame_id),
        "sampled": make_path(read_xyz_csv(os.path.join(case_dir, "sampled_path.csv")), frame_id),
        "corridor": make_corridor_markers(read_corridor_csv(os.path.join(case_dir, "safe_corridor.csv")), frame_id),
    }
    print("Loaded case:", case_dir)
    return data


def stamp_data(data, stamp):
    for msg in data.values():
        if hasattr(msg, "header"):
            msg.header.stamp = stamp
        elif isinstance(msg, MarkerArray):
            for marker in msg.markers:
                marker.header.stamp = stamp


def publish_data(pubs, data):
    pubs["cloud_filtered"].publish(data["cloud_filtered"])
    pubs["occupied"].publish(data["occupied"])
    pubs["inflated"].publish(data["inflated"])
    pubs["searched"].publish(data["searched"])
    pubs["frontend"].publish(data["frontend"])
    pubs["reference"].publish(data["reference"])
    pubs["shortcut"].publish(data["shortcut"])
    pubs["sampled"].publish(data["sampled"])
    pubs["corridor"].publish(data["corridor"])


class TerminalKeyReader:
    """非阻塞读取终端单键。不是 tty 时自动禁用按键浏览。"""

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.old_settings = None

    def __enter__(self):
        if self.enabled:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        if not self.enabled:
            return ""
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return ""
        return sys.stdin.read(1)


def main():
    parser = argparse.ArgumentParser(description="Replay FastNav planner failure case to RViz.")
    parser.add_argument("--root", default="/home/shukun/Project/FastNav/tools/debug_cases")
    parser.add_argument("--case", default="latest", help="case dir name, absolute path, or latest")
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args(rospy.myargv()[1:])

    cases = list_case_dirs(args.root)
    case_dir = resolve_case(args.case, args.root)
    if not os.path.isdir(case_dir):
        raise RuntimeError("case directory does not exist: {}".format(case_dir))
    case_index = find_case_index(cases, case_dir)

    rospy.init_node("fastnav_failure_case_replay", anonymous=True)

    pubs = {
        "cloud_filtered": rospy.Publisher("/fastnav/debug_case/cloud_filtered", PointCloud2, queue_size=1, latch=True),
        "occupied": rospy.Publisher("/fastnav/debug_case/occupied_cloud", PointCloud2, queue_size=1, latch=True),
        "inflated": rospy.Publisher("/fastnav/debug_case/inflated_cloud", PointCloud2, queue_size=1, latch=True),
        "searched": rospy.Publisher("/fastnav/debug_case/searched_nodes", PointCloud2, queue_size=1, latch=True),
        "frontend": rospy.Publisher("/fastnav/debug_case/frontend_path", Path, queue_size=1, latch=True),
        "reference": rospy.Publisher("/fastnav/debug_case/reference_path", Path, queue_size=1, latch=True),
        "shortcut": rospy.Publisher("/fastnav/debug_case/shortcut_path", Path, queue_size=1, latch=True),
        "sampled": rospy.Publisher("/fastnav/debug_case/sampled_path", Path, queue_size=1, latch=True),
        "corridor": rospy.Publisher("/fastnav/debug_case/safe_corridor", MarkerArray, queue_size=1, latch=True),
    }

    data = load_case_data(case_dir, args.frame_id)

    print("RViz Fixed Frame:", args.frame_id)
    print("Keyboard: s=next, w=previous, r=reload, q=quit")
    if case_index >= 0:
        print("Case index: {}/{}".format(case_index + 1, len(cases)))
    else:
        print("Current case is outside root list; keyboard next/previous is disabled.")
    print("Topics:")
    for topic in [
        "/fastnav/debug_case/cloud_filtered",
        "/fastnav/debug_case/occupied_cloud",
        "/fastnav/debug_case/inflated_cloud",
        "/fastnav/debug_case/searched_nodes",
        "/fastnav/debug_case/frontend_path",
        "/fastnav/debug_case/reference_path",
        "/fastnav/debug_case/shortcut_path",
        "/fastnav/debug_case/sampled_path",
        "/fastnav/debug_case/safe_corridor",
    ]:
        print("  ", topic)

    def switch_case(new_index):
        nonlocal case_index, case_dir, data
        if new_index < 0 or new_index >= len(cases):
            return
        case_index = new_index
        case_dir = cases[case_index]
        data = load_case_data(case_dir, args.frame_id)
        print("Case index: {}/{}".format(case_index + 1, len(cases)))
        stamp_data(data, rospy.Time.now())
        publish_data(pubs, data)

    rate = rospy.Rate(args.rate)
    with TerminalKeyReader() as key_reader:
        while not rospy.is_shutdown():
            key = key_reader.read_key()
            if key == "q":
                print("Quit.")
                break
            if key == "s":
                if case_index >= 0:
                    switch_case(min(case_index + 1, len(cases) - 1))
                else:
                    print("Current case is outside root list; cannot switch next.")
            elif key == "w":
                if case_index >= 0:
                    switch_case(max(case_index - 1, 0))
                else:
                    print("Current case is outside root list; cannot switch previous.")
            elif key == "r":
                data = load_case_data(case_dir, args.frame_id)
                stamp_data(data, rospy.Time.now())
                publish_data(pubs, data)

            now = rospy.Time.now()
            stamp_data(data, now)
            publish_data(pubs, data)
            rate.sleep()


if __name__ == "__main__":
    main()
