


def speedControl(ex, ey, dex, dey, ddex, ddey):
    # PID调节
    # ex: 本次误差;dex: 上次的误差; ddex: 上上次的误差
    # 仿真中，Kp=0.6,Kd=0.02效果较好
    # Kp = 0.06
    # Kd = 0.002
    Kp = 1.3
    Kd = 0.04
    Ki = 0.005

    ep_x = ex - dex
    ep_y = ey - dey
    ed_x = ex - 2 * dex + ddex
    ed_y = ey - 2 * dey + ddey
    ei_x = ex
    ei_y = ey

    velocity_x = Kp * ep_x + Kd * ed_x + Ki * ei_x
    velocity_y = Kp * ep_y + Kd * ed_y + Ki * ei_y

    ddex = dex
    ddey = dey
    dex = ex
    dey = ey

    return velocity_x, velocity_y, dex, dey, ddex, ddey
def position_control(offset_px, offset_py):
    P = 0.05

    x = P * offset_px
    y = P * offset_py

    return x, y