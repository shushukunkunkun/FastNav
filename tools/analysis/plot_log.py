#!/usr/bin/env python3
"""FastNav 离线绘图入口。

当前主要的在线曲线工具是 live_plot_state.py。这个文件保留为 tools/analysis
下的统一入口，避免旧脚本路径失效。
"""


def main():
    print("当前请使用实时曲线工具：")
    print("  python3 tools/analysis/live_plot_state.py")
    print("如果要离线复盘，先用 tools/bag/record_sim.sh 录制 bag。")


if __name__ == '__main__':
    main()
