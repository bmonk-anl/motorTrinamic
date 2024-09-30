# script for finding ideal values for pulse divisor and ramp divisor given max velocity and
# acceleration needed for application

import math

def vel_int_to_steps(v_int, pulse_div):
    v_steps = (16*(10**6) * v_int) / (2**pulse_div * 2048*32)
    return v_steps

def accel_int_to_steps(a_int, pulse_div, ramp_div):
    accel_steps = ((16*(10**6))**2 * a_int) / (2**(ramp_div+pulse_div+29))
    return accel_steps

def vel_steps_to_int(v_steps, pulse_div):
    v_int = round(0.004096 * (2**pulse_div) * v_steps)
    return vel_int

def get_pulse_div(v_max):
    pulse_div = math.log2(2047/(0.004096 * v_max))
    return 13 if (pulse_div > 13) else math.floor(pulse_div)

def get_ramp_div(a_max, pulse_div):
    ramp_div = math.log2((16e6)**2 * (2047/a_max)) - pulse_div - 29
    return 13 if (ramp_div > 13) else math.floor(ramp_div)

def get_accel_range(pulse_div, ramp_div):
    lower_lim = 2**(ramp_div - pulse_div - 1)
    upper_lim = 2**(ramp_div - pulse_div + 12) - 1
    if upper_lim > 2047:
        upper_lim = 2047
    return lower_lim, upper_lim


if __name__ == '__main__':
    max_needed_vel = float(input("Enter max velocity needed (microsteps/s): "))
    pulse_div = get_pulse_div(max_needed_vel)
    print("ideal pulse divisor: ", pulse_div)

    max_vel = vel_int_to_steps(2047, pulse_div)
    print("max possible velocity with pulse divisor of", pulse_div, ":", max_vel)

    max_needed_accel = float(input("Enter max acceleration needed (microsteps/s^2): "))
    ramp_div = get_ramp_div(max_needed_accel, pulse_div)
    print("ideal ramp divisor: ", ramp_div)

    max_accel = accel_int_to_steps(2047, pulse_div, ramp_div)
    print("max possible accel with pulse divisor of", pulse_div, "and ramp divisor of", ramp_div,
    ":", max_accel)
    
    lower_accel_lim, upper_accel_lim = get_accel_range(3,7)
    lower_accel_lim_steps = accel_int_to_steps(lower_accel_lim, pulse_div, ramp_div) 
    upper_accel_lim_steps = accel_int_to_steps(upper_accel_lim, pulse_div, ramp_div) 
    
    print("lower accel limit: " + str(lower_accel_lim) + " (" + str(lower_accel_lim_steps) + " microsteps/s^2), upper accel limit: " + str(upper_accel_lim) + " (" + str(upper_accel_lim_steps) + " microsteps/s^2)")



