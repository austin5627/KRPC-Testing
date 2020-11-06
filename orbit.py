import math
import time
import krpc


print(f'Start')

conn = krpc.connect(name='Orbit')
vessel = conn.space_center.active_vessel

turn_start_altitude = 250
turn_end_altitude = 50000
target_altitude = 200000

ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_2_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)
stage_2_fuel = conn.add_stream(stage_2_resources.amount, 'LiquidFuel')

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

# Countdown...
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

# Launch
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

turn_angle = 0


while True:

    # Gravity Turn
    if turn_start_altitude < altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        # Turn Every Half a Degree
        if abs(new_turn_angle - turn_angle) > .5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

    if apoapsis() > target_altitude * .9:
        print("Approaching target apoapsis")
        break

# Fine Burn Until Target Apoapsis Is Reached
vessel.control.throttle = .25
while apoapsis() < target_altitude:
    if stage_2_fuel() < .1 and vessel.control.current_stage == 2:
        print("Staging")
        vessel.control.activate_next_stage()

print('Target apoapsis reached')
vessel.control.throttle = 0

# Jettison Ascent Stage
if vessel.control.current_stage == 2:
    print("Staging")
    vessel.control.activate_next_stage()

print('Coasting out of atmosphere')
while altitude() < 70000:
    pass


# circularize

# plan circularization burn using vis-visa equation
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2.0/r) - (1.0/a1)))
v2 = math.sqrt(mu*((2.0/r) - (1.0/a2)))
delta_v = v2 - v1
print(f"Creating manoeuvre node with {delta_v}m/s")

# calculate burn time with rocket equation
F = vessel.available_thrust
Ve = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Ve)
flow_rate = F / Ve
burn_time = (m0 - m1) / flow_rate
print(f"Burn Time = {burn_time}")
node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Orientate ship
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Wait until burn
print('Wait until Burn')
burn_ut = node.ut - (burn_time/2)
lead_time = 3
conn.space_center.warp_to(burn_ut - lead_time)

print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
delta_v_remaining = conn.add_stream(getattr, node, 'remaining_delta_v')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass

print('Executing burn')
vessel.control.throttle = 1.0

while delta_v_remaining() > 5.0:
    pass

print('Fine tuning')
vessel.control.throttle = .05
while delta_v_remaining() > 0.2:
    pass

vessel.control.throttle = 0
node.remove()

print("Orbit Achieved")
