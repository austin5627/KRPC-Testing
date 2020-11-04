import krpc
import time

conn = krpc.connect(name='rendezvous')
vessel = conn.space_center.active_vessel

target = conn.space_center.target_vessel
if target is None:
    target = conn.space_center.target_docking_port.part.vessel

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
vessel.auto_pilot.target_roll = 0.0
vessel.auto_pilot.engage()
vessel.auto_pilot.wait()


dist_to_port = conn.add_stream(controller.position, target.reference_frame)
port_velocity = conn.add_stream(controller.velocity, target.reference_frame)

# align y axis
if dist_to_port()[1] < 1 and port_velocity()[1] < 1:
    while dist_to_port()[1] < 1 and port_velocity()[1] < 1:
        vessel.control.forward = -1
vessel.control.forward = 0

while dist_to_port()[1] < 1:
    pass

while port_velocity()[1] > 0.1:
    vessel.control.forward = 1
vessel.control.forward = 0

conn.drawing.add_direction((1, 0, 0), target.reference_frame)
conn.drawing.add_direction((1, 0, 0), controller.reference_frame)

if dist_to_port()[2] > .1:
    while dist_to_port()[2] > .1 and port_velocity()[2] > -.1:
        vessel.control.right = 1
        print("1 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
    vessel.control.right = 0

    while dist_to_port()[2] > -.1:
        print("2 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
        pass

    while port_velocity()[2] < 0:
        vessel.control.right = -1
        print("3 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
    vessel.control.right = 0

if dist_to_port()[0] < -.1:
    while dist_to_port()[0] < -.1 and port_velocity()[0] < .1:
        vessel.control.up = 1
        print("1 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
    vessel.control.up = 0

    while dist_to_port()[0] < -.1:
        print("2 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
        pass

    while port_velocity()[0] > 0:
        vessel.control.up = -1
        print("3 - x: %.3f y: %.3f, z: %.3f" % dist_to_port(), " ------ x: %.3f y: %.3f, z: %.3f" % port_velocity())
    vessel.control.up = 0
