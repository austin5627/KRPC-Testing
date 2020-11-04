import math
import time

import deltaV
import krpc
import numpy


def match_inclination(vessel, target):
    an_ta = vessel.orbit.true_anomaly_at_an(target.orbit)
    node_ut = vessel.orbit.ut_at_true_anomaly(an_ta)
    node_dv = 2 * vessel.orbit.speed * math.sin(vessel.orbit.relative_inclination(target.orbit) / 2)
    node = vessel.control.add_node(node_ut, 0, -node_dv, 0)
    return node


conn = krpc.connect(name='rendezvous')
vessel = conn.space_center.active_vessel
target = conn.space_center.target_vessel

if target is None:
    print("No Target")
    exit(-2)

if vessel.orbit.relative_inclination(target.orbit) > 0.1:
    node = match_inclination(vessel, target)
    deltaV.execute_node(vessel, node, conn)

# Rendezvous

print("Rendezvous")
v_pos = vessel.position(vessel.orbit.body.non_rotating_reference_frame)
t_pos = target.position(target.orbit.body.non_rotating_reference_frame)
v_pos = (v_pos[0], v_pos[2])
t_pos = (t_pos[0], t_pos[2])
unit_v = v_pos / numpy.linalg.norm(v_pos)
unit_t = t_pos / numpy.linalg.norm(t_pos)

dot_product = numpy.dot(unit_v, unit_t)
angle = numpy.arccos(numpy.clip(dot_product, -1.0, 1.0))

orbit = vessel.orbit
period = orbit.period
mu = orbit.body.gravitational_parameter

time_from_target = period * ((math.tau - angle) / math.tau)
new_period = period + time_from_target
new_semi_major_axis = math.pow((new_period / math.tau) ** 2 * mu, (1. / 3.))
new_apoapsis = new_semi_major_axis * 2 - orbit.periapsis

r = vessel.orbit.periapsis
a1 = vessel.orbit.semi_major_axis
a2 = new_semi_major_axis
v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
delta_v = v2 - v1

node_ut = conn.space_center.ut + 30
node = vessel.control.add_node(node_ut, prograde=delta_v)
deltaV.execute_node(vessel, node, conn)

# Match Velocities
print("Match Velocities")
ca_time = orbit.time_of_closest_approach(target.orbit)
conn.space_center.warp_to(ca_time - 10)

print("Point Retrograde")
vessel.auto_pilot.reference_frame = target.orbital_reference_frame
v = vessel.flight(target.orbital_reference_frame).velocity
n_v = (-v[0], -v[1], -v[2])
vessel.auto_pilot.target_direction = n_v
vessel.auto_pilot.engage()
vessel.auto_pilot.wait()

print("Throttle up")
target_sp = conn.add_stream(getattr, vessel.flight(target.orbital_reference_frame), "speed")
vessel.control.throttle = 1
last_sp = target_sp()
while target_sp() <= last_sp:
    last_sp = target_sp()
    print(target_sp())
vessel.control.throttle = 0
