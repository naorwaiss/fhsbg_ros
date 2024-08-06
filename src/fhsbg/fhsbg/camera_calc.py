def adjust_velocity_ratios(z_filtered):
    if z_filtered < 1.0:
        x_velocity_ratio = 0.25
        y_velocity_ratio = 0.25
        z_velocity_ratio = 0.25
    elif 1.0 <= z_filtered < 2.0:
        x_velocity_ratio = 0.5
        y_velocity_ratio = 0.5
        z_velocity_ratio = 0.5
    else:
        x_velocity_ratio = 1.0
        y_velocity_ratio = 1.0
        z_velocity_ratio = 1.0
    return x_velocity_ratio, y_velocity_ratio, z_velocity_ratio

def calculate_velocity(position, velocity_ratio):
    return position * velocity_ratio
