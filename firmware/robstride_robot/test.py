import can
import robstride
import time

def degrees_to_radians(degrees):
        return degrees * 3.14159 / 180


with can.Bus(interface="socketcan", channel="can0") as bus:
    rs_client = robstride.Client(bus)

    # First set the run mode to position
    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)

    # Then enable the motor
    rs_client.enable(1)
    
    time.sleep(5)
    
    # for i in range(0, 360):
    #     resp = rs_client.write_param(1, 'loc_ref', degrees_to_radians(i))
    #     time.sleep(0.01)
        
    # rs_client.write_param(1, 'loc_ref', degrees_to_radians(0))
    
    print("Going to zero position")
    
    
    # rs_client.write_param(1, 'mechpos', 0)
    
    # resp = rs_client.zero_pos(1)
    
    # time.sleep(10)
    
    # for i in range(10):
        
    #     resp = rs_client.zero_pos(1)
    #     time.sleep(4)
    
    
    
    # print("This is zero")
    
    
    # time.sleep(5)
    resp = rs_client.zero_pos(1)
    resp = rs_client.write_param(1, 'loc_ref', degrees_to_radians(100))
    time.sleep(5)
    resp = rs_client.write_param(1, 'loc_ref', degrees_to_radians(0))
    time.sleep(5)
    
    # time.sleep(10)
    
    # rs_client.disable(1)
    
    # rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)

    # # Then enable the motor
    # rs_client.enable(1)
    
    # resp = rs_client.write_param(1, 'loc_ref', degrees_to_radians(0))
    
    # time.sleep(10)
    
    
    # rs_client.write_param(1, 'loc_ref', degrees_to_radians(0))
    print("going back to zero")
    # time.sleep(10)
    
    
    # rs_client.disable(1)
    
    # print(resp)
    
    

    # Next tell the motor to go to its zero position
    # Many calls including enable and write return the current state of the motor
    # for i in range(0, 360):
    #     resp = rs_client.write_param(1, 'loc_ref', degrees_to_radians(i))
    #     time.sleep(0.01)
    # print('starting position:', resp.angle)
    # resp = rs_client.use_control_mode(1, 5, 2, 20, 20, 0.2)

    # Give is a few seconds to reach that position
    # time.sleep(5)

    # Now read the position, this time through the read call.
    # new_angle = rs_client.read_param(1, 'mech_pos')
    # print('ending position:', new_angle)

    # Finally deactivate the motor
    # rs_client.disable(1)