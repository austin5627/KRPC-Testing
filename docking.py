import krpc
import numpy
import time
import math


def translate(axis, value):
    if axis == 0:
        vessel.control.right = value
    elif axis == 1:
        vessel.control.forward = value
    elif axis == 2:
        vessel.control.up = value


def align_axis(axis):
    print(f'align axis {axis}')
    if abs(dist_to_port()[axis]) > 3:
        while abs(port_velocity()[axis] + math.copysign(1, dist_to_port()[axis])) > velocity_tolerance\
                and abs(dist_to_port()[axis]) > 3:
            thrust = (math.copysign(1, dist_to_port()[axis] * -1) - port_velocity()[axis]) * -1
            if abs(thrust) < min_thrust:
                thrust = math.copysign(min_thrust, thrust)
            translate(axis, thrust)
    translate(axis, 0)

    print('Coast to port')
    while abs(dist_to_port()[axis] - 0.25) > 0.25:
        if 1 < abs(dist_to_port()[axis]) < 5 and abs(port_velocity()[axis]) > .2:
            thrust = port_velocity()[axis]
            if abs(thrust) < min_thrust:
                thrust = math.copysign(min_thrust, thrust)
            translate(axis, thrust)
        elif 0 < abs(dist_to_port()[axis]) < 1 and abs(port_velocity()[axis]) > .1:
            thrust = port_velocity()[axis]
            if abs(thrust) < min_thrust:
                thrust = math.copysign(min_thrust, thrust)
            translate(axis, thrust)
        else:
            translate(axis, 0)
    translate(axis, 0)
    print('Stop')
    stop()
    translate(axis, 0)


def stop():
    print('Stop')
    while abs(port_velocity()[0]) > velocity_tolerance \
            or abs(port_velocity()[1]) > velocity_tolerance \
            or abs(port_velocity()[2]) > velocity_tolerance:
        if abs(port_velocity()[0]) > velocity_tolerance:
            right = port_velocity()[0]
            if abs(right) < min_thrust:
                right = math.copysign(min_thrust, right)
            vessel.control.right = right
        else:
            vessel.control.right = 0
        if abs(port_velocity()[1]) > velocity_tolerance:
            forward = port_velocity()[1]
            if abs(forward) < min_thrust:
                forward = math.copysign(min_thrust, forward)
            vessel.control.forward = forward
        else:
            vessel.control.forward = 0
        if abs(port_velocity()[2]) > velocity_tolerance:
            up = port_velocity()[2]
            if abs(up) < min_thrust:
                up = math.copysign(min_thrust, up)
            vessel.control.up = up
        else:
            vessel.control.up = 0
    vessel.control.right = 0
    vessel.control.forward = 0
    vessel.control.up = 0


conn = krpc.connect(name='rendezvous')
vessel = conn.space_center.active_vessel

target = conn.space_center.target_vessel
if target is None:
    target = conn.space_center.target_docking_port.part.vessel

if numpy.linalg.norm(vessel.position(target.reference_frame)) > 1000:
    print('Too far away from target')

docking_ports = target.parts.docking_ports
for d in docking_ports:
    if d.state == conn.space_center.DockingPortState.ready and d.part.name != 'dockingPortLarge':
        conn.space_center.target_docking_port = d
        target = d
        break

controller = vessel
if vessel.parts.controlling.docking_port is not None:
    controller = vessel.parts.controlling
else:
    vessel.parts.controlling = vessel.parts.docking_ports[0].part
    controller = vessel.parts.controlling

vessel.auto_pilot.reference_frame = target.part.reference_frame
v = target.direction(target.part.reference_frame)
n_v = (v[0], -v[1], v[2])
vessel.auto_pilot.target_direction = n_v
vessel.auto_pilot.target_roll = 90.0
vessel.auto_pilot.engage()
vessel.auto_pilot.wait()

dist_to_port = conn.add_stream(controller.position, target.reference_frame)
port_velocity = conn.add_stream(controller.velocity, target.reference_frame)

conn.drawing.add_direction((1, 0, 0), controller.reference_frame, 1).color = (255, 0, 0)
conn.drawing.add_direction((0, 1, 0), controller.reference_frame, 1).color = (0, 255, 0)
conn.drawing.add_direction((0, 0, 1), controller.reference_frame, 1).color = (0, 0, 255)

velocity_tolerance = 0.005
min_thrust = 0.06

coast_velocity = 0.05

# Explanation of conditions
# abs(num - offset) > tolerance
# returns false when num is within tolerance of offset

# stop
stop()

# align y axis
align_axis(1)
# align x axis
align_axis(0)
# align z axis
align_axis(2)

vessel.control.forward = .2
time.sleep(0.5)
vessel.control.forward = 0

