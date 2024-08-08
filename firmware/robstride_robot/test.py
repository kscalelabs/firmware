import can
import robstride.robstride as robstride
import time



with can.Bus(interface="socketcan", channel="can0") as bus:
    rs_client = robstride.Client(bus)

    # First set the run mode to position
    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)

    # Then enable the motor
    rs_client.enable(1)
    
    resp = rs_client.zero_pos(1)
    
    print(resp)

    # Next tell the motor to go to its zero position
    # Many calls including enable and write return the current state of the motor
    # resp = rs_client.write_param(1, 'loc_ref', 0)
    # print('starting position:', resp.angle)
    # resp = rs_client.use_control_mode(1, 5, 2, 5, 15, 0.2)

    # Give is a few seconds to reach that position
    time.sleep(3)

    # Now read the position, this time through the read call.
    # new_angle = rs_client.read_param(1, 'mech_pos')
    # print('ending position:', new_angle)

    # Finally deactivate the motor
    rs_client.disable(1)