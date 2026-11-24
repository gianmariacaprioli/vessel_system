"""Default parameters for the supported Gazebo surface vessels."""

VESSEL_PROFILES = {
    'vessel_v2': dict(
        mass=55000.0, xU=-2750.0, xUabsU=-1500.0, nR=-60922.0,
        nRabsR=-283000.0, kp_surge=2000.0, kd_surge=5000.0,
        kp_yaw=40000.0, kd_yaw=20000.0, max_speed=5.0,
        use_gps=True, lever_arm=1.0, forward_thrust=30000.0,
        reverse_thrust=20000.0,
    ),
    'nautilus_vessel': dict(
        mass=260.0, xU=-100.0, xUabsU=-55.0, nR=-120.0,
        nRabsR=-140.0, kp_surge=25.0, kd_surge=85.0,
        kp_yaw=95.0, kd_yaw=55.0, max_speed=1.0,
        omega_n=0.45, yaw_drag_feedforward=0.0,
        approach_gain=0.7, waypoint_tolerance=0.25,
        use_gps=False, lever_arm=0.345, forward_thrust=150.0,
        reverse_thrust=150.0,
    ),
}


def declare_model(node):
    model = node.declare_parameter('model', 'vessel_v2').value
    if model not in VESSEL_PROFILES:
        raise ValueError(f'Unknown surface vessel: {model!r}')
    return model, VESSEL_PROFILES[model]
