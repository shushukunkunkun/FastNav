#!/usr/bin/env python3
"""
FastNav 实时状态曲线工具。

订阅实测里程计、控制期望值、planner FSM、controller FSM 和 local_target，
在线绘制速度模长、FSM 状态阶跃曲线和状态信息。

本版本增强：
1. 默认不绘制 vx/vy/vz 三轴速度，只绘制速度模长；
2. 可通过 --show-axis-velocity 开启三轴速度图；
3. 新增 FSM 阶跃曲线图：
   - Planner FSM
   - Controller FSM
4. local_target 发生变化时，在速度图上标注：
   - 淡红竖线
   - 红色三角点
   - LT#编号
5. 关闭窗口或 Ctrl+C 时自动保存结果图；
6. 默认保存到本脚本路径下的 result 文件夹。
"""

import argparse
import math
import os
import site
import sys
import threading
import time
from collections import deque

# ROS Noetic 的系统 matplotlib 通常是基于 NumPy 1.x 编译的；
# 如果用户目录里安装了 NumPy 2.x，直接 import matplotlib 可能触发 `_ARRAY_API not found`。
# 因此默认移除 user site，只使用 ROS/系统 Python 包。
# 若确实想使用用户 Python 包，可以运行前设置 FASTNAV_ALLOW_USER_SITE=1。
if os.environ.get("FASTNAV_ALLOW_USER_SITE", "0") != "1":
    user_site = site.getusersitepackages()
    sys.path = [p for p in sys.path if p != user_site]

# 避免 matplotlib 尝试写入不可写的 $HOME/.config/matplotlib。
os.environ.setdefault("MPLCONFIGDIR", "/tmp/fastnav_matplotlib")
os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    from fastnav_msgs.msg import ControlCommand
except Exception:
    # 允许用户只画 odom，不要求一定 source FastNav 工作空间。
    ControlCommand = None

try:
    from fastnav_msgs.msg import PlannerTiming
except Exception:
    PlannerTiming = None


class TimeSeries:
    """保存滑动窗口数据或完整历史数据。"""

    def __init__(self):
        self.t = deque()
        self.values = {}

    def add(self, t, **kwargs):
        self.t.append(t)

        for key, value in kwargs.items():
            self.values.setdefault(key, deque()).append(value)

        # 如果某一时刻没有提供某个字段，用 nan 补齐，保证曲线长度一致。
        for key, data in self.values.items():
            if key not in kwargs:
                data.append(float("nan"))

    def trim(self, min_t):
        while self.t and self.t[0] < min_t:
            self.t.popleft()
            for data in self.values.values():
                if data:
                    data.popleft()

    def get(self, key):
        return list(self.t), list(self.values.get(key, []))

    def nearest_value(self, key, query_t):
        values = self.values.get(key)

        if not self.t or not values:
            return float("nan")

        best_i = 0
        best_dt = abs(self.t[0] - query_t)

        for i, t_i in enumerate(self.t):
            dt = abs(t_i - query_t)
            if dt < best_dt:
                best_i = i
                best_dt = dt

        if best_i >= len(values):
            return float("nan")

        return values[best_i]


class StepSeries:
    """保存 FSM 状态阶跃曲线。"""

    def __init__(self):
        self.t = deque()
        self.codes = deque()
        self.states = deque()

    def add(self, t, code, state):
        # 相同状态重复发布时，不重复记录。
        if self.codes and self.codes[-1] == code:
            return

        # 防止极端情况下时间戳倒退导致 step 曲线显示异常。
        if self.t and t < self.t[-1]:
            t = self.t[-1] + 1.0e-6

        self.t.append(t)
        self.codes.append(code)
        self.states.append(state)

    def get_step_data(self, x_min=None, x_max=None):
        """
        返回用于 ax.step(..., where='post') 的数据。
        如果指定 x_min/x_max，会保留 x_min 之前最近一次状态，
        这样曲线在窗口左边界不会断掉。
        """
        if not self.t:
            return [], []

        times = list(self.t)
        codes = list(self.codes)

        start_i = 0

        if x_min is not None:
            # 找到最后一个 <= x_min 的点，保证窗口左侧状态连续。
            for i, t_i in enumerate(times):
                if t_i <= x_min:
                    start_i = i
                else:
                    break

        times = times[start_i:]
        codes = codes[start_i:]

        if x_min is not None and times:
            times[0] = max(times[0], x_min)

        if x_max is not None and times:
            if times[-1] < x_max:
                times.append(x_max)
                codes.append(codes[-1])

        return times, codes


class LiveStatePlotter:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.save_lock = threading.Lock()
        self.saved_results = False

        self.t0 = None

        # 显示用滑动窗口数据。
        self.odom_series = TimeSeries()
        self.cmd_series = TimeSeries()
        self.timing_series = TimeSeries()

        # 保存结果用完整历史数据。
        self.odom_history = TimeSeries()
        self.cmd_history = TimeSeries()
        self.timing_history = TimeSeries()

        self.last_odom_t = None
        self.last_odom_vel = None

        self.latest_cmd_type = "none"
        self.planner_fsm_state = "no msg"
        self.prev_planner_fsm_state = "no msg"
        self.control_fsm_state = "no msg"
        self.latest_timing_success = "no msg"
        self.latest_timing_reason = ""

        # FSM 阶跃曲线。
        self.state_to_code = {}
        self.code_to_state = {}
        self.planner_fsm_series = StepSeries()
        self.control_fsm_series = StepSeries()

        # local target 事件相关状态。
        self.local_target_count = 0
        self.latest_local_target_pos = None
        self.last_local_target_time = float("nan")
        self.pending_local_target_marks = deque()
        self.local_target_mark_artists = []
        self.pending_clear_local_target_marks = False

        # 保存用 local target 全历史事件。
        self.local_target_events_history = []

        # 动态字体的基准窗口尺寸。
        self.base_fig_width = 14.0
        self.base_fig_height = 9.0
        self.base_dpi = 110.0
        self.current_font_scale = 1.0

        self.odom_sub = rospy.Subscriber(
            args.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=50,
        )

        self.cmd_sub = None
        if not args.no_command and ControlCommand is not None:
            self.cmd_sub = rospy.Subscriber(
                args.cmd_topic,
                ControlCommand,
                self.command_callback,
                queue_size=100,
            )
        elif not args.no_command:
            rospy.logwarn(
                "[FastNav][LivePlot] fastnav_msgs not available, command curve disabled."
            )

        self.planner_fsm_sub = rospy.Subscriber(
            args.planner_fsm_topic,
            String,
            self.planner_fsm_callback,
            queue_size=10,
        )

        self.control_fsm_sub = rospy.Subscriber(
            args.control_fsm_topic,
            String,
            self.control_fsm_callback,
            queue_size=10,
        )

        self.local_target_sub = rospy.Subscriber(
            args.local_target_topic,
            PoseStamped,
            self.local_target_callback,
            queue_size=20,
        )

        self.timing_sub = None
        if not args.no_timing and PlannerTiming is not None:
            self.timing_sub = rospy.Subscriber(
                args.timing_topic,
                PlannerTiming,
                self.timing_callback,
                queue_size=50,
            )
        elif not args.no_timing:
            rospy.logwarn(
                "[FastNav][LivePlot] fastnav_msgs/PlannerTiming not available, timing curve disabled."
            )

        # 默认四块：
        # 1. speed
        # 2. planner timing
        # 3. FSM step
        # 4. status text
        #
        # 如果 --show-axis-velocity，则变为五块：
        # 1. speed
        # 2. vx/vy/vz
        # 3. planner timing
        # 4. FSM step
        # 5. status text
        if self.args.show_axis_velocity:
            self.fig, raw_axes = plt.subplots(
                5,
                1,
                sharex=False,
                figsize=(self.base_fig_width, self.base_fig_height + 3.0),
                dpi=self.base_dpi,
            )
            self.ax_speed = raw_axes[0]
            self.ax_axis_velocity = raw_axes[1]
            self.ax_timing = raw_axes[2]
            self.ax_fsm = raw_axes[3]
            self.ax_status = raw_axes[4]
            self.axes = raw_axes
        else:
            self.fig, raw_axes = plt.subplots(
                4,
                1,
                sharex=False,
                figsize=(self.base_fig_width, self.base_fig_height + 2.0),
                dpi=self.base_dpi,
            )
            self.ax_speed = raw_axes[0]
            self.ax_axis_velocity = None
            self.ax_timing = raw_axes[1]
            self.ax_fsm = raw_axes[2]
            self.ax_status = raw_axes[3]
            self.axes = raw_axes

        self.fig.canvas.manager.set_window_title("FastNav State Monitor")

        self.lines = {}
        self.status_text = None
        self.title_text = None
        self.legends = []

        self.setup_plot()

        # 监听窗口大小变化。窗口变大/变小时，自动调整字体和线宽。
        self.fig.canvas.mpl_connect("resize_event", self.on_resize)

        # 关闭窗口时保存结果。
        self.fig.canvas.mpl_connect("close_event", self.on_close)

    def setup_plot(self):
        # -------------------------
        # 第一张图：速度模长
        # -------------------------
        ax = self.ax_speed

        self.lines["odom_speed"], = ax.plot(
            [],
            [],
            label="odom |v|",
            color="tab:blue",
        )

        self.lines["cmd_speed"], = ax.plot(
            [],
            [],
            "--",
            label="cmd |v|",
            color="tab:orange",
        )

        ax.set_ylabel("speed [m/s]")
        ax.set_xlabel("time [s]")
        ax.grid(True)

        legend = ax.legend(loc="upper right")
        self.legends.append(legend)

        # -------------------------
        # 可选图：三轴速度
        # -------------------------
        if self.ax_axis_velocity is not None:
            ax = self.ax_axis_velocity

            self.lines["odom_vx"], = ax.plot(
                [],
                [],
                label="odom vx",
                color="tab:red",
            )

            self.lines["odom_vy"], = ax.plot(
                [],
                [],
                label="odom vy",
                color="tab:green",
            )

            self.lines["odom_vz"], = ax.plot(
                [],
                [],
                label="odom vz",
                color="tab:purple",
            )

            self.lines["cmd_vx"], = ax.plot(
                [],
                [],
                "--",
                label="cmd vx",
                color="tab:red",
                alpha=0.60,
            )

            self.lines["cmd_vy"], = ax.plot(
                [],
                [],
                "--",
                label="cmd vy",
                color="tab:green",
                alpha=0.60,
            )

            self.lines["cmd_vz"], = ax.plot(
                [],
                [],
                "--",
                label="cmd vz",
                color="tab:purple",
                alpha=0.60,
            )

            ax.set_ylabel("velocity [m/s]")
            ax.set_xlabel("time [s]")
            ax.grid(True)

            legend = ax.legend(loc="upper right", ncol=3)
            self.legends.append(legend)

        # -------------------------
        # Planner timing 图
        # -------------------------
        ax = self.ax_timing

        self.lines["timing_total"], = ax.plot(
            [],
            [],
            label="total",
            color="tab:blue",
        )
        self.lines["timing_astar"], = ax.plot(
            [],
            [],
            label="A*",
            color="tab:green",
        )
        self.lines["timing_corridor"], = ax.plot(
            [],
            [],
            label="corridor",
            color="tab:purple",
        )
        self.lines["timing_minco"], = ax.plot(
            [],
            [],
            label="MINCO",
            color="tab:orange",
        )
        self.lines["timing_check"], = ax.plot(
            [],
            [],
            label="fine check",
            color="tab:red",
        )

        ax.set_ylabel("planner [ms]")
        ax.set_xlabel("time [s]")
        ax.grid(True)

        legend = ax.legend(loc="upper right", ncol=5)
        self.legends.append(legend)

        # -------------------------
        # FSM 阶跃图
        # -------------------------
        ax = self.ax_fsm

        self.lines["planner_fsm"], = ax.step(
            [],
            [],
            where="post",
            label="Planner FSM",
            color="tab:blue",
        )

        self.lines["control_fsm"], = ax.step(
            [],
            [],
            where="post",
            label="Controller FSM",
            color="tab:orange",
            linestyle="--",
        )

        ax.set_ylabel("FSM state")
        ax.set_xlabel("time [s]")
        ax.grid(True)

        legend = ax.legend(loc="upper right")
        self.legends.append(legend)

        # -------------------------
        # 状态文字面板
        # -------------------------
        ax = self.ax_status
        ax.axis("off")

        self.status_text = ax.text(
            0.02,
            0.90,
            "",
            transform=ax.transAxes,
            family="monospace",
            verticalalignment="top",
        )

        self.title_text = self.fig.suptitle("")

        self.apply_dynamic_style()

    def compute_font_scale(self):
        try:
            width_px, height_px = self.fig.canvas.get_width_height()
        except Exception:
            return 1.0

        base_width_px = self.base_fig_width * self.base_dpi
        base_height_px = self.base_fig_height * self.base_dpi

        if width_px <= 0 or height_px <= 0:
            return 1.0

        area_scale = math.sqrt(
            (width_px * height_px) / (base_width_px * base_height_px)
        )

        return max(0.75, min(1.80, area_scale))

    def apply_dynamic_style(self):
        scale = self.compute_font_scale()
        self.current_font_scale = scale

        title_size = 15.0 * scale
        label_size = 14.0 * scale
        tick_size = 11.5 * scale
        legend_size = 11.0 * scale
        status_size = 15.0 * scale

        line_width_main = max(1.4, min(4.0, 2.2 * scale))
        line_width_cmd = max(1.2, min(3.6, 2.0 * scale))
        fsm_line_width = max(1.4, min(4.0, 2.2 * scale))

        local_target_line_width = max(1.0, min(2.8, 1.5 * scale))
        local_target_marker_size = max(6.0, min(13.0, 8.0 * scale))
        local_target_text_size = max(8.0, min(15.0, 10.5 * scale))

        if self.title_text is not None:
            self.title_text.set_fontsize(title_size)
            self.title_text.set_fontweight("bold")

        for ax in self.axes:
            if ax is self.ax_status:
                continue

            ax.xaxis.label.set_fontsize(label_size)
            ax.yaxis.label.set_fontsize(label_size)
            ax.tick_params(axis="both", labelsize=tick_size)

            for tick_label in ax.get_xticklabels() + ax.get_yticklabels():
                tick_label.set_fontsize(tick_size)

        if self.status_text is not None:
            self.status_text.set_fontsize(status_size)

        for legend in self.legends:
            if legend is None:
                continue
            for text in legend.get_texts():
                text.set_fontsize(legend_size)

        for name, line in self.lines.items():
            if name.startswith("cmd_"):
                line.set_linewidth(line_width_cmd)
            elif name in ["planner_fsm", "control_fsm"]:
                line.set_linewidth(fsm_line_width)
            elif name.startswith("timing_"):
                line.set_linewidth(line_width_main)
            else:
                line.set_linewidth(line_width_main)

        # local target 事件标注也随窗口变化自动调整。
        for mark in self.local_target_mark_artists:
            line_speed = mark["line_speed"]
            marker = mark["marker"]
            text = mark["text"]

            line_speed.set_linewidth(local_target_line_width)

            if marker is not None:
                marker.set_markersize(local_target_marker_size)

            if text is not None:
                text.set_fontsize(local_target_text_size)

            if mark.get("line_axis_velocity") is not None:
                mark["line_axis_velocity"].set_linewidth(local_target_line_width)

        try:
            self.fig.tight_layout(rect=[0.03, 0.03, 0.98, 0.93])
        except Exception:
            pass

    def on_resize(self, event):
        self.apply_dynamic_style()
        self.fig.canvas.draw_idle()

    def on_close(self, event):
        self.save_results(reason="close_event")

    def relative_now(self):
        return self.time_from_msg(rospy.Time.now())

    def time_from_msg(self, stamp):
        if stamp is None or stamp.is_zero():
            stamp = rospy.Time.now()

        t_abs = stamp.to_sec()

        if self.t0 is None:
            self.t0 = t_abs

        return t_abs - self.t0

    def get_state_code(self, state):
        if state not in self.state_to_code:
            code = len(self.state_to_code)
            self.state_to_code[state] = code
            self.code_to_state[code] = state

        return self.state_to_code[state]

    def odom_callback(self, msg):
        t = self.time_from_msg(msg.header.stamp)

        v = msg.twist.twist.linear
        p = msg.pose.pose.position

        vel = (v.x, v.y, v.z)

        speed = math.sqrt(
            v.x * v.x +
            v.y * v.y +
            v.z * v.z
        )

        # 由 odom 速度差分估计运动加速度模长。
        acc_norm = float("nan")

        if self.last_odom_t is not None:
            dt = t - self.last_odom_t

            if dt > 1.0e-4 and self.last_odom_vel is not None:
                ax = (vel[0] - self.last_odom_vel[0]) / dt
                ay = (vel[1] - self.last_odom_vel[1]) / dt
                az = (vel[2] - self.last_odom_vel[2]) / dt

                acc_norm = math.sqrt(
                    ax * ax +
                    ay * ay +
                    az * az
                )

        self.last_odom_t = t
        self.last_odom_vel = vel

        with self.lock:
            kwargs = dict(
                speed=speed,
                vx=v.x,
                vy=v.y,
                vz=v.z,
                acc_norm=acc_norm,
                z=p.z,
            )

            self.odom_series.add(t, **kwargs)
            self.odom_history.add(t, **kwargs)

    def command_callback(self, msg):
        t = self.time_from_msg(msg.header.stamp)

        v = msg.velocity
        a = msg.acceleration
        p = msg.position

        speed = math.sqrt(
            v.x * v.x +
            v.y * v.y +
            v.z * v.z
        )

        acc_norm = math.sqrt(
            a.x * a.x +
            a.y * a.y +
            a.z * a.z
        )

        with self.lock:
            self.latest_cmd_type = str(msg.command_type)

            kwargs = dict(
                speed=speed,
                vx=v.x,
                vy=v.y,
                vz=v.z,
                acc_norm=acc_norm,
                z=p.z,
            )

            self.cmd_series.add(t, **kwargs)
            self.cmd_history.add(t, **kwargs)

    def planner_fsm_callback(self, msg):
        t = self.relative_now()

        with self.lock:
            self.prev_planner_fsm_state = self.planner_fsm_state
            self.planner_fsm_state = msg.data

            code = self.get_state_code(self.planner_fsm_state)
            self.planner_fsm_series.add(t, code, self.planner_fsm_state)

            # 一个任务周期定义为两次 WAIT_TARGET 之间；
            # 回到 WAIT_TARGET 时清空本轮 local target 计数和当前窗口标注。
            # 注意：保存用的 local_target_events_history 不会被清空。
            if (
                self.planner_fsm_state == "WAIT_TARGET"
                and self.prev_planner_fsm_state != "WAIT_TARGET"
            ):
                self.local_target_count = 0
                self.latest_local_target_pos = None
                self.last_local_target_time = float("nan")
                self.pending_local_target_marks.clear()
                self.pending_clear_local_target_marks = True

    def control_fsm_callback(self, msg):
        t = self.relative_now()

        with self.lock:
            self.control_fsm_state = msg.data

            code = self.get_state_code(self.control_fsm_state)
            self.control_fsm_series.add(t, code, self.control_fsm_state)

    def is_new_local_target(self, msg):
        p = msg.pose.position
        current_pos = (p.x, p.y, p.z)

        if self.latest_local_target_pos is None:
            self.latest_local_target_pos = current_pos
            return True

        dx = current_pos[0] - self.latest_local_target_pos[0]
        dy = current_pos[1] - self.latest_local_target_pos[1]
        dz = current_pos[2] - self.latest_local_target_pos[2]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.args.local_target_change_threshold:
            return False

        self.latest_local_target_pos = current_pos
        return True

    def local_target_callback(self, msg):
        t = self.time_from_msg(msg.header.stamp)

        with self.lock:
            # WAIT_TARGET 中收到的 latched local target 可能属于上一轮任务，不计入当前周期。
            if self.planner_fsm_state == "WAIT_TARGET":
                return

            if not self.is_new_local_target(msg):
                return

            self.local_target_count += 1
            self.last_local_target_time = t

            speed_at_event = self.odom_history.nearest_value("speed", t)

            if math.isnan(speed_at_event):
                speed_at_event = self.latest_value(self.odom_history, "speed")

            if math.isnan(speed_at_event):
                speed_at_event = 0.0

            event = {
                "t": t,
                "index": self.local_target_count,
                "speed": speed_at_event,
            }

            self.pending_local_target_marks.append(event)
            self.local_target_events_history.append(event)

    def timing_callback(self, msg):
        t = self.time_from_msg(msg.header.stamp)

        with self.lock:
            self.latest_timing_success = str(bool(msg.success))
            self.latest_timing_reason = msg.failure_reason

            kwargs = dict(
                total=msg.total_ms,
                astar=msg.frontend_astar_ms,
                corridor=msg.corridor_ms,
                minco=msg.minco_ms,
                fine_check=msg.fine_check_ms,
                clearance=msg.clearance_used,
                retry=float(msg.minco_retry_count),
            )

            self.timing_series.add(t, **kwargs)
            self.timing_history.add(t, **kwargs)

    def clear_local_target_marks(self):
        for mark in self.local_target_mark_artists:
            for key in ["line_speed", "line_axis_velocity", "marker", "text"]:
                artist = mark.get(key)
                if artist is None:
                    continue
                try:
                    artist.remove()
                except Exception:
                    pass

        self.local_target_mark_artists = []

    def add_local_target_mark(self, t, index, speed_at_event):
        scale = self.current_font_scale

        line_width = max(1.0, min(2.8, 1.5 * scale))
        marker_size = max(6.0, min(13.0, 8.0 * scale))
        text_size = max(8.0, min(15.0, 10.5 * scale))

        line_speed = self.ax_speed.axvline(
            t,
            color="tab:red",
            linestyle=":",
            linewidth=line_width,
            alpha=0.45,
            zorder=1,
        )

        line_axis_velocity = None
        if self.ax_axis_velocity is not None:
            line_axis_velocity = self.ax_axis_velocity.axvline(
                t,
                color="tab:red",
                linestyle=":",
                linewidth=line_width,
                alpha=0.35,
                zorder=1,
            )

        marker, = self.ax_speed.plot(
            [t],
            [speed_at_event],
            marker="^",
            linestyle="None",
            color="tab:red",
            markersize=marker_size,
            zorder=6,
        )

        text = self.ax_speed.annotate(
            "LT#{}".format(index),
            xy=(t, speed_at_event),
            xytext=(0, 12),
            textcoords="offset points",
            ha="center",
            va="bottom",
            color="tab:red",
            fontsize=text_size,
            fontweight="bold",
            zorder=7,
        )

        self.local_target_mark_artists.append(
            {
                "t": t,
                "index": index,
                "line_speed": line_speed,
                "line_axis_velocity": line_axis_velocity,
                "marker": marker,
                "text": text,
            }
        )

    def trim_local_target_marks(self, min_t):
        kept = []

        for mark in self.local_target_mark_artists:
            if mark["t"] >= min_t:
                kept.append(mark)
                continue

            for key in ["line_speed", "line_axis_velocity", "marker", "text"]:
                artist = mark.get(key)
                if artist is None:
                    continue
                try:
                    artist.remove()
                except Exception:
                    pass

        self.local_target_mark_artists = kept

    def update_fsm_axis_ticks(self):
        if not self.code_to_state:
            return

        codes = sorted(self.code_to_state.keys())
        labels = [self.code_to_state[c] for c in codes]

        self.ax_fsm.set_yticks(codes)
        self.ax_fsm.set_yticklabels(labels)

        self.ax_fsm.set_ylim(min(codes) - 0.5, max(codes) + 0.5)

    def update_plot(self):
        pending_marks = []
        should_clear_marks = False

        with self.lock:
            current_t = 0.0

            if self.odom_series.t:
                current_t = max(current_t, self.odom_series.t[-1])

            if self.cmd_series.t:
                current_t = max(current_t, self.cmd_series.t[-1])

            if self.timing_series.t:
                current_t = max(current_t, self.timing_series.t[-1])

            if self.planner_fsm_series.t:
                current_t = max(current_t, self.planner_fsm_series.t[-1])

            if self.control_fsm_series.t:
                current_t = max(current_t, self.control_fsm_series.t[-1])

            min_t = max(0.0, current_t - self.args.window)

            self.odom_series.trim(min_t)
            self.cmd_series.trim(min_t)
            self.timing_series.trim(min_t)

            should_clear_marks = self.pending_clear_local_target_marks
            self.pending_clear_local_target_marks = False

            while self.pending_local_target_marks:
                pending_marks.append(self.pending_local_target_marks.popleft())

            self.set_line("odom_speed", self.odom_series, "speed")
            self.set_line("cmd_speed", self.cmd_series, "speed")

            if self.ax_axis_velocity is not None:
                self.set_line("odom_vx", self.odom_series, "vx")
                self.set_line("odom_vy", self.odom_series, "vy")
                self.set_line("odom_vz", self.odom_series, "vz")
                self.set_line("cmd_vx", self.cmd_series, "vx")
                self.set_line("cmd_vy", self.cmd_series, "vy")
                self.set_line("cmd_vz", self.cmd_series, "vz")

            self.set_line("timing_total", self.timing_series, "total")
            self.set_line("timing_astar", self.timing_series, "astar")
            self.set_line("timing_corridor", self.timing_series, "corridor")
            self.set_line("timing_minco", self.timing_series, "minco")
            self.set_line("timing_check", self.timing_series, "fine_check")

            planner_t, planner_y = self.planner_fsm_series.get_step_data(min_t, current_t)
            control_t, control_y = self.control_fsm_series.get_step_data(min_t, current_t)

            self.lines["planner_fsm"].set_data(planner_t, planner_y)
            self.lines["control_fsm"].set_data(control_t, control_y)

            latest_speed = self.latest_value(self.odom_series, "speed")
            latest_cmd_speed = self.latest_value(self.cmd_series, "speed")
            latest_z = self.latest_value(self.odom_series, "z")
            latest_cmd_z = self.latest_value(self.cmd_series, "z")
            latest_acc = self.latest_value(self.odom_series, "acc_norm")
            latest_cmd_acc = self.latest_value(self.cmd_series, "acc_norm")
            latest_plan_total = self.latest_value(self.timing_series, "total")
            latest_plan_astar = self.latest_value(self.timing_series, "astar")
            latest_plan_minco = self.latest_value(self.timing_series, "minco")
            latest_clearance = self.latest_value(self.timing_series, "clearance")

            self.status_text.set_text(
                "Planner FSM   : {planner}\n"
                "Controller FSM: {control}\n"
                "Command type  : {cmd_type}\n"
                "Planner timing: ok={timing_ok}, total={plan_total:.2f} ms, A*={plan_astar:.2f} ms, MINCO={plan_minco:.2f} ms, clearance={clearance:.3f}\n"
                "Local targets : {local_target_count}\n"
                "Last LT time  : {last_lt_time:.3f} s\n"
                "Latest |v|    : odom={odom_speed:.3f} m/s, cmd={cmd_speed:.3f} m/s\n"
                "Latest |a|    : odom={odom_acc:.3f} m/s^2, cmd={cmd_acc:.3f} m/s^2\n"
                "Latest z      : odom={odom_z:.3f} m,   cmd={cmd_z:.3f} m".format(
                    planner=self.planner_fsm_state,
                    control=self.control_fsm_state,
                    cmd_type=self.latest_cmd_type,
                    timing_ok=self.latest_timing_success,
                    plan_total=latest_plan_total,
                    plan_astar=latest_plan_astar,
                    plan_minco=latest_plan_minco,
                    clearance=latest_clearance,
                    local_target_count=self.local_target_count,
                    last_lt_time=self.last_local_target_time,
                    odom_speed=latest_speed,
                    cmd_speed=latest_cmd_speed,
                    odom_acc=latest_acc,
                    cmd_acc=latest_cmd_acc,
                    odom_z=latest_z,
                    cmd_z=latest_cmd_z,
                )
            )

        x_max = max(self.args.window, current_t)
        x_min = max(0.0, x_max - self.args.window)

        if should_clear_marks:
            self.clear_local_target_marks()

        for mark in pending_marks:
            if mark["t"] < x_min or mark["t"] > x_max:
                continue

            self.add_local_target_mark(
                mark["t"],
                mark["index"],
                mark["speed"],
            )

        self.trim_local_target_marks(x_min)
        self.update_fsm_axis_ticks()
        self.apply_dynamic_style()

        axes_to_limit = [self.ax_speed, self.ax_timing, self.ax_fsm]
        if self.ax_axis_velocity is not None:
            axes_to_limit.append(self.ax_axis_velocity)

        for ax in axes_to_limit:
            ax.set_xlim(x_min, x_max)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        # FSM 图的 y 轴不要被 autoscale 改乱。
        self.update_fsm_axis_ticks()

        if self.title_text is not None:
            self.title_text.set_text(
                "FastNav live monitor: odom={}    cmd={}    latest_cmd_type={}".format(
                    self.args.odom_topic,
                    "disabled" if self.args.no_command else self.args.cmd_topic,
                    self.latest_cmd_type,
                )
            )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def set_line(self, name, series, key):
        t, y = series.get(key)
        self.lines[name].set_data(t, y)

    def latest_value(self, series, key):
        values = series.values.get(key)

        if not values:
            return float("nan")

        return values[-1]

    def get_result_dir(self):
        if self.args.result_dir:
            return os.path.abspath(self.args.result_dir)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(script_dir, "result")

    def save_results(self, reason="shutdown"):
        """
        关闭窗口或 Ctrl+C 时调用。

        默认保存四个图：
        1. 当前监控窗口完整图；
        2. 速度模长单独图；
        3. FSM 阶跃曲线单独图；
        4. planner 各阶段耗时曲线单独图。
        """
        with self.save_lock:
            if self.saved_results:
                return
            self.saved_results = True

        result_dir = self.get_result_dir()
        os.makedirs(result_dir, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")

        monitor_path = os.path.join(result_dir, "fastnav_monitor_{}.png".format(stamp))
        speed_path = os.path.join(result_dir, "fastnav_speed_{}.png".format(stamp))
        fsm_path = os.path.join(result_dir, "fastnav_fsm_{}.png".format(stamp))
        timing_path = os.path.join(result_dir, "fastnav_planner_timing_{}.png".format(stamp))

        rospy.loginfo("[FastNav][LivePlot] saving figures to: %s", result_dir)

        try:
            self.fig.savefig(monitor_path, dpi=150, bbox_inches="tight")
            rospy.loginfo("[FastNav][LivePlot] saved monitor figure: %s", monitor_path)
        except Exception as e:
            rospy.logwarn("[FastNav][LivePlot] failed to save monitor figure: %s", str(e))

        with self.lock:
            odom_t, odom_speed = self.odom_history.get("speed")
            cmd_t, cmd_speed = self.cmd_history.get("speed")

            local_target_events = list(self.local_target_events_history)

            planner_t, planner_y = self.planner_fsm_series.get_step_data()
            control_t, control_y = self.control_fsm_series.get_step_data()

            code_to_state = dict(self.code_to_state)

            timing_t, timing_total = self.timing_history.get("total")
            _, timing_astar = self.timing_history.get("astar")
            _, timing_corridor = self.timing_history.get("corridor")
            _, timing_minco = self.timing_history.get("minco")
            _, timing_check = self.timing_history.get("fine_check")

        self.save_speed_figure(
            speed_path,
            odom_t,
            odom_speed,
            cmd_t,
            cmd_speed,
            local_target_events,
        )

        self.save_fsm_figure(
            fsm_path,
            planner_t,
            planner_y,
            control_t,
            control_y,
            code_to_state,
        )

        self.save_timing_figure(
            timing_path,
            timing_t,
            timing_total,
            timing_astar,
            timing_corridor,
            timing_minco,
            timing_check,
        )

    def save_speed_figure(
        self,
        path,
        odom_t,
        odom_speed,
        cmd_t,
        cmd_speed,
        local_target_events,
    ):
        fig, ax = plt.subplots(figsize=(14, 6), dpi=150)

        if odom_t and odom_speed:
            ax.plot(
                odom_t,
                odom_speed,
                label="odom |v|",
                linewidth=2.2,
                color="tab:blue",
            )

        if cmd_t and cmd_speed:
            ax.plot(
                cmd_t,
                cmd_speed,
                "--",
                label="cmd |v|",
                linewidth=2.0,
                color="tab:orange",
            )

        for event in local_target_events:
            t = event["t"]
            index = event["index"]
            speed = event["speed"]

            ax.axvline(
                t,
                color="tab:red",
                linestyle=":",
                linewidth=1.4,
                alpha=0.40,
            )

            ax.plot(
                [t],
                [speed],
                marker="^",
                linestyle="None",
                color="tab:red",
                markersize=7,
            )

            ax.annotate(
                "LT#{}".format(index),
                xy=(t, speed),
                xytext=(0, 10),
                textcoords="offset points",
                ha="center",
                va="bottom",
                color="tab:red",
                fontsize=10,
                fontweight="bold",
            )

        ax.set_title("FastNav Speed Curve")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("speed [m/s]")
        ax.grid(True)
        ax.legend(loc="upper right")
        fig.tight_layout()

        try:
            fig.savefig(path, dpi=150, bbox_inches="tight")
            rospy.loginfo("[FastNav][LivePlot] saved speed figure: %s", path)
        except Exception as e:
            rospy.logwarn("[FastNav][LivePlot] failed to save speed figure: %s", str(e))
        finally:
            plt.close(fig)

    def save_fsm_figure(
        self,
        path,
        planner_t,
        planner_y,
        control_t,
        control_y,
        code_to_state,
    ):
        fig, ax = plt.subplots(figsize=(14, 6), dpi=150)

        if planner_t and planner_y:
            ax.step(
                planner_t,
                planner_y,
                where="post",
                label="Planner FSM",
                linewidth=2.2,
                color="tab:blue",
            )

        if control_t and control_y:
            ax.step(
                control_t,
                control_y,
                where="post",
                label="Controller FSM",
                linewidth=2.2,
                linestyle="--",
                color="tab:orange",
            )

        if code_to_state:
            codes = sorted(code_to_state.keys())
            labels = [code_to_state[c] for c in codes]
            ax.set_yticks(codes)
            ax.set_yticklabels(labels)
            ax.set_ylim(min(codes) - 0.5, max(codes) + 0.5)

        ax.set_title("FastNav FSM State Step Curves")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("FSM state")
        ax.grid(True)
        ax.legend(loc="upper right")
        fig.tight_layout()

        try:
            fig.savefig(path, dpi=150, bbox_inches="tight")
            rospy.loginfo("[FastNav][LivePlot] saved FSM figure: %s", path)
        except Exception as e:
            rospy.logwarn("[FastNav][LivePlot] failed to save FSM figure: %s", str(e))
        finally:
            plt.close(fig)

    def save_timing_figure(
        self,
        path,
        timing_t,
        timing_total,
        timing_astar,
        timing_corridor,
        timing_minco,
        timing_check,
    ):
        fig, ax = plt.subplots(figsize=(14, 6), dpi=150)

        curves = [
            ("total", timing_total, "tab:blue"),
            ("A*", timing_astar, "tab:green"),
            ("corridor", timing_corridor, "tab:purple"),
            ("MINCO", timing_minco, "tab:orange"),
            ("fine check", timing_check, "tab:red"),
        ]

        for label, values, color in curves:
            if timing_t and values:
                ax.plot(
                    timing_t,
                    values,
                    label=label,
                    linewidth=2.0,
                    color=color,
                )

        ax.set_title("FastNav Planner Timing")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("time [ms]")
        ax.grid(True)
        ax.legend(loc="upper right")
        fig.tight_layout()

        try:
            fig.savefig(path, dpi=150, bbox_inches="tight")
            rospy.loginfo("[FastNav][LivePlot] saved planner timing figure: %s", path)
        except Exception as e:
            rospy.logwarn("[FastNav][LivePlot] failed to save planner timing figure: %s", str(e))
        finally:
            plt.close(fig)

    def spin(self):
        rate = rospy.Rate(self.args.rate)

        while not rospy.is_shutdown() and plt.fignum_exists(self.fig.number):
            self.update_plot()
            rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot FastNav odom, control command and FSM states in real time."
    )

    parser.add_argument(
        "--odom-topic",
        default="/fastnav/state/odom",
        help="Odometry topic, type: nav_msgs/Odometry.",
    )

    parser.add_argument(
        "--cmd-topic",
        default="/fastnav/control_cmd",
        help="Control command topic, type: fastnav_msgs/ControlCommand.",
    )

    parser.add_argument(
        "--planner-fsm-topic",
        default="/fastnav/planner/fsm_state",
        help="Planner FSM state topic, type: std_msgs/String.",
    )

    parser.add_argument(
        "--control-fsm-topic",
        default="/fastnav/control/fsm_state",
        help="Controller FSM state topic, type: std_msgs/String.",
    )

    parser.add_argument(
        "--local-target-topic",
        default="/fastnav/planner/local_target",
        help="Local target update topic, type: geometry_msgs/PoseStamped.",
    )

    parser.add_argument(
        "--timing-topic",
        default="/fastnav/planner/timing",
        help="Planner timing topic, type: fastnav_msgs/PlannerTiming.",
    )

    parser.add_argument(
        "--local-target-change-threshold",
        type=float,
        default=0.05,
        help=(
            "Minimum position change in meters to treat a received local target "
            "as a new local target."
        ),
    )

    parser.add_argument(
        "--window",
        type=float,
        default=20.0,
        help="Visible time window in seconds.",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Plot refresh rate.",
    )

    parser.add_argument(
        "--no-command",
        action="store_true",
        help="Only plot odometry curves.",
    )

    parser.add_argument(
        "--no-timing",
        action="store_true",
        help="Disable planner timing subscription and timing plot updates.",
    )

    parser.add_argument(
        "--show-axis-velocity",
        action="store_true",
        help="Also plot vx/vy/vz curves. Disabled by default.",
    )

    parser.add_argument(
        "--result-dir",
        default="",
        help=(
            "Directory to save result figures. "
            "Default: result folder under this script directory."
        ),
    )

    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()

    rospy.init_node("fastnav_live_state_plot", anonymous=True)

    plotter = LiveStatePlotter(args)

    # Ctrl+C 或 rosnode shutdown 时保存结果。
    rospy.on_shutdown(plotter.save_results)

    rospy.loginfo("[FastNav][LivePlot] odom topic: %s", args.odom_topic)
    rospy.loginfo(
        "[FastNav][LivePlot] command topic: %s",
        "disabled" if args.no_command else args.cmd_topic,
    )
    rospy.loginfo("[FastNav][LivePlot] planner FSM topic: %s", args.planner_fsm_topic)
    rospy.loginfo("[FastNav][LivePlot] control FSM topic: %s", args.control_fsm_topic)
    rospy.loginfo("[FastNav][LivePlot] local target topic: %s", args.local_target_topic)
    rospy.loginfo(
        "[FastNav][LivePlot] planner timing topic: %s",
        "disabled" if args.no_timing else args.timing_topic,
    )
    rospy.loginfo(
        "[FastNav][LivePlot] local target change threshold: %.3f m",
        args.local_target_change_threshold,
    )
    rospy.loginfo("[FastNav][LivePlot] result dir: %s", plotter.get_result_dir())

    plotter.spin()

    # 如果是直接关闭 matplotlib 窗口，确保退出前也保存一次。
    plotter.save_results(reason="spin_exit")


if __name__ == "__main__":
    main()
