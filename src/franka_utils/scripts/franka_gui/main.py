#!/usr/bin/env python3
"""Entry point for the Franka Panda Control Studio GUI."""

import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from PySide6.QtWidgets import QApplication

from franka_gui import DARK_STYLE, FrankaRosNode, MainWindow


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLE)

    # Signal bridge: ROS node -> Qt via callback
    signals = {"log": None, "joint_state": None, "controller_status": None}

    def log_cb(msg):
        if signals["log"]:
            signals["log"](msg)

    def joint_state_cb(positions):
        if signals["joint_state"]:
            signals["joint_state"](positions)

    def controller_status_cb(data):
        if signals["controller_status"]:
            signals["controller_status"](data)

    signals["log"] = log_cb
    signals["joint_state"] = joint_state_cb
    signals["controller_status"] = controller_status_cb

    node = FrankaRosNode(log_cb)
    node._log = log_cb  # override for internal use

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="ros2-executor")
    spin_thread.start()

    window = MainWindow(
        node=node,
        log_callback=log_cb,
        get_joint_positions_callback=lambda: node.latest_positions
    )
    window.show()

    # Wire up Qt slots
    signals["log"] = window.append_log
    signals["joint_state"] = window.on_joint_state
    signals["controller_status"] = window.on_controller_status

    try:
        exit_code = app.exec()
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()