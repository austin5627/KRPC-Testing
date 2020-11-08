import math
import time

import vessel_utilities
import krpc
import numpy


def match_inclination(vessel, target, conn):
    an_ta = vessel.orbit.true_anomaly_at_an(target.orbit)
    node_ut = vessel.orbit.ut_at_true_anomaly(an_ta)
    node_dv = 2 * vessel.orbit.speed * math.sin(vessel.orbit.relative_inclination(target.orbit) / 2)
    node = vessel.control.add_node(node_ut, 0, -node_dv, 0)
    vessel_utilities.execute_node(vessel, node, conn)


def get_close_approach(vessel, target, conn):
    print("Create Close Approach")

    # Get Angle Between target and vessel
    # TODO: More Testing for vessel in front of target
    v_pos = vessel.position(vessel.orbit.body.non_rotating_reference_frame)
    t_pos = target.position(target.orbit.body.non_rotating_reference_frame)
    v_pos = (v_pos[0], v_pos[2])  # Ignore Altitude
    t_pos = (t_pos[0], t_pos[2])
    angle = angle_between(v_pos, t_pos)
    print(angle)

    orbit = vessel.orbit
    period = orbit.period
    mu = orbit.body.gravitational_parameter

    # Find the time between target and vessel
    time_from_target = period * ((math.tau - angle) / math.tau)
    # Change orbit to be time_from_target longer
    new_period = period + time_from_target
    new_semi_major_axis = math.pow((new_period / math.tau) ** 2 * mu, (1. / 3.))

    # delta v required for orbit change
    r = vessel.orbit.periapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = new_semi_major_axis
    v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
    v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
    delta_v = v2 - v1

    # execute orbit change
    node_ut = conn.space_center.ut + 30
    node = vessel.control.add_node(node_ut, prograde=delta_v)
    vessel_utilities.execute_node(vessel, node, conn)


def match_velocities(vessel, target, conn, max_throttle=1):
    print("Match Velocities")

    print("Point Retrograde")
    vessel.auto_pilot.reference_frame = target.orbital_reference_frame
    v = vessel.flight(target.orbital_reference_frame).velocity
    n_v = (-v[0], -v[1], -v[2])
    vessel.auto_pilot.target_direction = n_v
    vessel.auto_pilot.engage()
    vessel.auto_pilot.wait()

    print("Throttle up")
    target_sp = conn.add_stream(getattr, vessel.flight(target.orbital_reference_frame), "speed")
    vessel.control.throttle = max_throttle
    time.sleep(.1)
    last_sp = target_sp()
    while target_sp() <= last_sp:
        last_sp = target_sp()
        time.sleep(0.01)
    vessel.control.throttle = 0


def fine_tune_closest_approach(vessel, target, conn):
    target_position = conn.add_stream(target.position, vessel.orbital_reference_frame)
    target_sp = conn.add_stream(getattr, vessel.flight(target.orbital_reference_frame), "speed")
    sensitivities = [[50, 1, 400], [30, .5, 200], [5, .1, 50]]
    while numpy.linalg.norm(target_position()) > 50:
        vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
        vessel.auto_pilot.target_direction = target_position()
        vessel.auto_pilot.engage()
        vessel.auto_pilot.wait()
        dist = numpy.linalg.norm(target_position())
        if dist > 1000:
            x = 0
        elif dist > 500:
            x = 1
        else:
            x = 2
        vessel.control.throttle = sensitivities[x][1]
        while target_sp() < sensitivities[x][0]:
            pass
        vessel.control.throttle = 0
        vessel.auto_pilot.reference_frame = target.orbital_reference_frame
        v = vessel.flight(target.orbital_reference_frame).velocity
        n_v = (-v[0], -v[1], -v[2])
        vessel.auto_pilot.target_direction = n_v
        vessel.auto_pilot.engage()
        vessel.auto_pilot.wait()
        last_dist = numpy.linalg.norm(target_position())
        time.sleep(0.1)
        while sensitivities[x][2] < numpy.linalg.norm(target_position()) < last_dist:
            last_dist = numpy.linalg.norm(target_position())
            time.sleep(0.1)
        match_velocities(vessel, target, conn, sensitivities[x][1])


def rendezvous():
    conn = krpc.connect(name='rendezvous')
    vessel = conn.space_center.active_vessel
    target = conn.space_center.target_vessel

    if target is None:
        print("No Target")
        exit(-2)

    if vessel.orbit.relative_inclination(target.orbit) > 0.1:
        match_inclination(vessel, target, conn)

    # Rendezvous
    print("Rendezvous")

    get_close_approach(vessel, target, conn)

    ca_time = vessel.orbit.time_of_closest_approach(target.orbit)
    conn.space_center.warp_to(ca_time - 10)

    match_velocities(vessel, target, conn)

    fine_tune_closest_approach(vessel, target, conn)


def angle_between(v1, v2):
    unit_v1 = v1 / numpy.linalg.norm(v1)  # Convert to unit vectors
    unit_v2 = v2 / numpy.linalg.norm(v2)

    dot_product = numpy.dot(unit_v1, unit_v2)
    determinant = unit_v1[0]*unit_v2[1] - unit_v1[1]*unit_v2[0]
    angle = numpy.arctan2(determinant, dot_product)
    return angle
