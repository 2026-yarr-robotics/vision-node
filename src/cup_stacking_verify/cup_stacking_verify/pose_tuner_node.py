"""pose_tuner_node — live editor for the verifier's cp / degree.

A small tkinter window with sliders + numeric entries for the L1_M position
(`cp` — bottom-row centre cup) and the row direction (`degree`, base +X CCW
around base +Z).  Every change is pushed to `cup_occupancy_verifier` via its
`set_parameters` service, and the verifier re-reads those parameters on every
render tick, so the boundary, the cp sphere and the degree arrow in RViz
update in real time.

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
        """Best-effort read of current cp + degree. None on failure."""
        if not self._get_cli.wait_for_service(timeout_sec=timeout_s):
            return None
        req = GetParameters.Request(names=['cp', 'degree'])
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
            cp = list(vals[0].double_array_value)
            deg = float(vals[1].double_value)
            if len(cp) == 3:
                return cp, deg
        except Exception:
            return None
        return None

    def apply(self, cp, degree: float) -> None:
        if not self._set_cli.service_is_ready():
            self._set_cli.wait_for_service(timeout_sec=0.1)
            if not self._set_cli.service_is_ready():
                self.get_logger().warn(
                    'set_parameters not available yet',
                    throttle_duration_sec=2.0)
                return
        req = SetParameters.Request(parameters=[
            Parameter(name='cp', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for v in cp])),
            Parameter(name='degree', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(degree))),
        ])
        self._set_cli.call_async(req)


# ── tkinter UI ─────────────────────────────────────────────────────────────

class TunerUI:
    # (label, key, min, max, default)  — keys: cx/cy/cz for cp; deg for degree.
    FIELDS = [
        ('cp.x  (m)',     'cx',    0.0,   1.20,  0.50),
        ('cp.y  (m)',     'cy',   -0.60,  0.60,  0.00),
        ('cp.z  (m)',     'cz',   -0.20,  0.60,  0.10),
        ('degree (°)',    'deg',    0.0, 360.0,  0.00),
    ]

    def __init__(self, node: PoseTunerNode) -> None:
        self.node = node
        self._apply_job = None

        init = node.fetch_initial()
        if init is not None:
            cp0, deg0 = init
            node.get_logger().info(f'loaded current cp={cp0} degree={deg0}')
        else:
            cp0, deg0 = [0.5, 0.0, 0.1], 0.0
            node.get_logger().info('using default cp / degree')

        self.root = tk.Tk()
        self.root.title('cp / degree tuner  [live]')
        self.root.resizable(False, False)

        frm = ttk.Frame(self.root, padding=12)
        frm.grid(row=0, column=0, sticky='nsew')

        # Layout:
        #   row 0..2   cp.x / cp.y / cp.z  (position section)
        #   row 3      HR separator with 15 px gap above + below
        #   row 4      degree slider
        #   row 5      buttons
        #   row 6      status
        self.vars: dict[str, tk.DoubleVar] = {}
        initial = {'cx': cp0[0], 'cy': cp0[1], 'cz': cp0[2], 'deg': deg0}
        row = 0
        for label, key, lo, hi, _default in self.FIELDS:
            if key == 'deg':
                # Position → degree boundary: explicit gap-HR-gap row of its
                # OWN, never sharing a row with a slider.
                ttk.Separator(frm, orient=tk.HORIZONTAL).grid(
                    row=row, column=0, columnspan=3, sticky='ew',
                    pady=(15, 15))
                row += 1

            var = tk.DoubleVar(value=round(float(initial[key]), 4))
            self.vars[key] = var

            ttk.Label(frm, text=label, width=12,
                      font=('Courier', 10)).grid(row=row, column=0, sticky='w',
                                                 pady=3)
            scale = ttk.Scale(frm, from_=lo, to=hi, orient=tk.HORIZONTAL,
                              length=260, variable=var,
                              command=lambda *_: self._schedule_apply())
            scale.grid(row=row, column=1, padx=6)
            ent = ttk.Entry(frm, width=9, textvariable=var, justify='right')
            ent.grid(row=row, column=2)
            ent.bind('<Return>', lambda *_: self._schedule_apply())
            row += 1

        btns = ttk.Frame(frm)
        btns.grid(row=row, column=0, columnspan=3,
                  pady=(10, 0), sticky='ew')
        ttk.Button(btns, text='Apply now',
                   command=self._apply_now).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text='Reset defaults',
                   command=self._reset).grid(row=0, column=1, padx=4)
        row += 1

        self.status = tk.StringVar(value='live: every change is applied')
        ttk.Label(frm, textvariable=self.status, foreground='#2a7a2a',
                  font=('Helvetica', 9)).grid(
            row=row, column=0, columnspan=3, sticky='w', pady=(8, 0))

        self._apply_now()  # push the loaded/initial values once

    # ── apply (debounced) ─────────────────────────────────────────────────
    def _schedule_apply(self) -> None:
        if self._apply_job is not None:
            self.root.after_cancel(self._apply_job)
        self._apply_job = self.root.after(80, self._apply_now)

    def _apply_now(self) -> None:
        self._apply_job = None
        try:
            cp = [self.vars['cx'].get(), self.vars['cy'].get(),
                  self.vars['cz'].get()]
            deg = float(self.vars['deg'].get())
        except tk.TclError:
            self.status.set('⚠ invalid number')
            return
        self.node.apply(cp, deg)
        self.status.set(
            f'applied  cp=({cp[0]:.3f},{cp[1]:.3f},{cp[2]:.3f})  '
            f'deg={deg:+.1f}°')

    def _reset(self) -> None:
        for label, key, _lo, _hi, default in self.FIELDS:
            self.vars[key].set(default)
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
