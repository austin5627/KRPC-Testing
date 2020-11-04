import math
import time


# get the current mass of all parts in 'stage'
def stage_mass(vessel, stage):
    mass = 0
    for part in vessel.parts.in_decouple_stage(stage - 1):
        mass += part.mass
    return mass


# get the dry mass of all parts in stage
def stage_dry_mass(vessel, stage):
    dry_mass = 0
    for part in vessel.parts.in_decouple_stage(stage - 1):
        if "LiquidFuel" in part.resources.names or "SolidFuel" in part.resources.names:
            dry_mass += part.dry_mass
        else:
            dry_mass += part.mass
    return dry_mass


# get the net isp of all engines in 'engines'
def net_isp(engines):
    thrust = sum(engine.max_thrust for engine in engines)
    fuel_consumption = sum(engine.max_thrust / engine.vacuum_specific_impulse for engine in engines)
    isp = thrust / fuel_consumption
    return isp


# get the net isp of all engines in 'stage'
def stage_isp(vessel, stage):
    # Get all engines that are active in 'stage'
    stage_engines = [engine for engine in vessel.parts.engines if engine.part.stage >= stage > engine.part.decouple_stage]
    if len(stage_engines) == 0:
        return 0
    return net_isp(stage_engines)


def stage_thrust(vessel, stage):
    stage_engines = [engine for engine in vessel.parts.engines if engine.part.stage >= stage > engine.part.decouple_stage]
    if len(stage_engines) == 0:
        return 0
    return sum(e.available_thrust for e in stage_engines)


# calculate the delta V in 'stage'
def stage_delta_v(vessel, stage):
    other_mass = 0
    for i in range(stage):
        other_mass += stage_mass(vessel, i)

    mass0 = other_mass + stage_mass(vessel, stage)
    mass1 = other_mass + stage_dry_mass(vessel, stage)

    isp = stage_isp(vessel, stage)
    v_e = isp * 9.81
    delta_v = math.log(mass0 / mass1) * v_e
    return delta_v


def get_burn_time(vessel, delta_v):
    burn_time = 0
    remaining_dv = delta_v
    stage = vessel.control.current_stage
    while remaining_dv > 0:
        if stage < 0:
            return None
        f = stage_thrust(vessel, stage)
        curr_dv = min(stage_delta_v(vessel, stage), remaining_dv)
        remaining_dv -= curr_dv
        ve = stage_isp(vessel, stage) * 9.82
        if ve == 0:
            stage -= 1
            continue
        m0 = sum(stage_mass(vessel, s) for s in range(stage + 1))
        m1 = m0 / math.exp(curr_dv / ve)
        flow_rate = f / ve
        burn_time += (m0 - m1) / flow_rate
        stage -= 1
    return burn_time


def execute_node(vessel, node, conn):
    dv = node.delta_v
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.wait()
    burn_time = get_burn_time(vessel, dv)
    burn_ut = node.ut - (burn_time / 2)
    lead_time = 3
    conn.space_center.warp_to(burn_ut - lead_time)
    delta_v_remaining = conn.add_stream(getattr, node, 'remaining_delta_v')

    time.sleep(lead_time)

    vessel.control.throttle = 1.0
    while delta_v_remaining() > 5.0:
        pass
    vessel.control.throttle = .05
    while delta_v_remaining() > 0.1:
        pass
    vessel.control.throttle = 0
    node.remove()
    vessel.control.sas = True
