"""pose_tuner_node — live editor for the verifier's p_start / v_dir.

A small tkinter window with sliders + numeric entries for the virtual-stack
start position (p_start) and growth direction (v_dir). Every change is pushed
to `cup_occupancy_verifier` via its `set_parameters` service, and the verifier
re-reads those parameters on every render tick, so the boundary, the p_start
sphere and the v_dir arrow in RViz update in real time.

Usage:
  ros2 run cup_stacking_verify pose_tuner
  ros2 run cup_stacking_verify pose_tuner --ros-args -p target_node:=cup_occupancy_verifier
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


class PoseTunerNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_tuner_node')
        self.declare_parameter('target_node', 'cup_occupancy_verifier')
        target = str(self.get_parameter('target_node').value).strip('/')

        self._set_cli = self.create_client(
            SetParameters, f'/{target}/set_parameters')
        self._get_cli = self.create_client(
            GetParameters, f'/{target}/get_parameters')
        self.get_logger().info(
            f'pose_tuner ready → /{target}/set_parameters')

    # ── parameter I/O ─────────────────────────────────────────────────────
    def fetch_initial(self, timeout_s: float = 3.0):
        """Best-effort read of current p_start / v_dir. None on failure."""
        if not self._get_cli.wait_for_service(timeout_sec=timeout_s):
            return None
        req = GetParameters.Request(names=['p_start', 'v_dir'])
        future = self._get_cli.call_async(req)
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while not future.done() and self.get_clock().now().nanoseconds < end:
            pass
        if not future.done() or future.result() is None:
            return None
        vals = future.result().values
        if len(vals) < 2:
            return None
        try:
            p = list(vals[0].double_array_value)
            d = list(vals[1].double_array_value)
            if len(p) == 3 and len(d) == 3:
                return p, d
        except Exception:
            return None
        return None

    def apply(self, p_start, v_dir) -> None:
        if not self._set_cli.service_is_ready():
            self._set_cli.wait_for_service(timeout_sec=0.1)
            if not self._set_cli.service_is_ready():
                self.get_logger().warn(
                    'set_parameters not available yet', throttle_duration_sec=2.0)
                return
        req = SetParameters.Request(parameters=[
            Parameter(name='p_start', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for v in p_start])),
            Parameter(name='v_dir', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for v in v_dir])),
        ])
        self._set_cli.call_async(req)


# ── tkinter UI ─────────────────────────────────────────────────────────────

class TunerUI:
    # (label, key, index, min, max, default)
    FIELDS = [
        ('p_start.x', 'p', 0,  0.0, 1.20, 0.50),
        ('p_start.y', 'p', 1, -0.60, 0.60, 0.00),
        ('p_start.z', 'p', 2, -0.20, 0.60, 0.10),
        ('v_dir.x',   'd', 0, -1.00, 1.00, 1.00),
        ('v_dir.y',   'd', 1, -1.00, 1.00, 0.00),
        ('v_dir.z',   'd', 2, -1.00, 1.00, 0.00),
    ]

    def __init__(self, node: PoseTunerNode) -> None:
        self.node = node
        self._apply_job = None

        init = node.fetch_initial()
        if init is not None:
            p0, d0 = init
            node.get_logger().info(f'loaded current p_start={p0} v_dir={d0}')
        else:
            p0, d0 = [0.5, 0.0, 0.1], [1.0, 0.0, 0.0]
            node.get_logger().info('using default p_start/v_dir')

        self.root = tk.Tk()
        self.root.title('p_start / v_dir tuner  [live]')
        self.root.resizable(False, False)

        frm = ttk.Frame(self.root, padding=12)
        frm.grid(row=0, column=0, sticky='nsew')

        self.vars: dict[str, tk.DoubleVar] = {}
        for row, (label, key, idx, lo, hi, _default) in enumerate(self.FIELDS):
            cur = (p0 if key == 'p' else d0)[idx]
            var = tk.DoubleVar(value=round(float(cur), 4))
            self.vars[f'{key}{idx}'] = var

            ttk.Label(frm, text=label, width=10,
                      font=('Courier', 10)).grid(row=row, column=0, sticky='w',
                                                 pady=3)
            scale = ttk.Scale(frm, from_=lo, to=hi, orient=tk.HORIZONTAL,
                              length=240, variable=var,
                              command=lambda *_: self._schedule_apply())
            scale.grid(row=row, column=1, padx=6)
            ent = ttk.Entry(frm, width=9, textvariable=var, justify='right')
            ent.grid(row=row, column=2)
            ent.bind('<Return>', lambda *_: self._schedule_apply())
            if row == 2:  # spacer between p_start and v_dir
                ttk.Separator(frm, orient=tk.HORIZONTAL).grid(
                    row=row, column=0, columnspan=3, sticky='ew',
                    pady=(8, 2), in_=frm)

        btns = ttk.Frame(frm)
        btns.grid(row=len(self.FIELDS), column=0, columnspan=3,
                  pady=(10, 0), sticky='ew')
        ttk.Button(btns, text='Apply now',
                   command=self._apply_now).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text='Reset defaults',
                   command=self._reset).grid(row=0, column=1, padx=4)

        self.status = tk.StringVar(value='live: every change is applied')
        ttk.Label(frm, textvariable=self.status, foreground='#2a7a2a',
                  font=('Helvetica', 9)).grid(
            row=len(self.FIELDS) + 1, column=0, columnspan=3,
            sticky='w', pady=(8, 0))

        self._apply_now()  # push the loaded/initial values once

    # ── apply (debounced) ─────────────────────────────────────────────────
    def _schedule_apply(self) -> None:
        if self._apply_job is not None:
            self.root.after_cancel(self._apply_job)
        self._apply_job = self.root.after(80, self._apply_now)

    def _apply_now(self) -> None:
        self._apply_job = None
        try:
            p = [self.vars['p0'].get(), self.vars['p1'].get(), self.vars['p2'].get()]
            d = [self.vars['d0'].get(), self.vars['d1'].get(), self.vars['d2'].get()]
        except tk.TclError:
            self.status.set('⚠ invalid number')
            return
        self.node.apply(p, d)
        self.status.set(
            f'applied  p=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})  '
            f'd=({d[0]:.2f},{d[1]:.2f},{d[2]:.2f})')

    def _reset(self) -> None:
        for label, key, idx, _lo, _hi, default in self.FIELDS:
            self.vars[f'{key}{idx}'].set(default)
        self._apply_now()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseTunerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    ui = TunerUI(node)
    try:
        ui.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
