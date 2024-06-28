"""Takes in responses from the Q&A return type and interprets them"""
import struct

msgType = 0

""" TODO: Need to add conversions for each message frame into
actual readable units"""

# Defintions 

ErrorMap = {
    0: "No Errors",
    1: "Motor Overheating",
    2: "Motor Overcurrent",
    3: "Motor Voltage too low",
    4: "Motor Encoder Error",
    6: "Motor brake voltage too high",
    7: "DRV Driver Error"
}


QueryMap = {
    0: "Reserved",
    1: "Angle",
    2: "Speed",
    3: "Current",
    4: "Power",
    5: "Accel",
    6: "Current Flux"
}


ConfigMap = {
    0: "Reserved",
    1: "Configuring System Acceleration",
    2: "Configuring Flux Observation Gain",
    3: "Configuring Damping Coefficient"
}

ConfigStatusMap = {
    0: "Failure",
    1: "Success"
}


def getMsgType(msg: bytes):
    for i in range(1,6):
        if msg[0] >> 5 == i:
            return i
    return -1


def MsgType1(msg: bytes):
    """
    Message Type 1
    
    Interprets the message type 1 and returns a list of the results
    
    Args:
        msg: bytes: The message to interpret
        
    Returns:
        dictionary of the message results
    """
    getPosition = lambda data: data * (25.0 / 65536.0) - 12.5
    getSpeed = lambda data: data * (36.0 / 4095.0) - 18.0
    getCurrent = lambda data: data * (140.0 / 4095) - 70.0
    getTemp = lambda data: (data - 50.0) / 2.0
    getMosTemp = lambda data: (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_pos = int.from_bytes(msg[1:3], "big")
    motor_speed = int.from_bytes(msg[3:5], 'big') >> 4
    motor_current = (int.from_bytes(msg[4:6], 'big') & 0xFFF)
    motor_temp = ((msg[6]) - 50) / 2
    motor_MOS_temp = ((msg[7]) - 50) / 2

    return {
        "Message Type" : 1,
        "Error" : ErrorMap[error], 
        "Position" : getPosition(motor_pos), 
        "Speed" : getSpeed(motor_speed),
        "Current" : getCurrent(motor_current),
        "Temperature" : getTemp(motor_temp),
        "MOS" : getMosTemp(motor_MOS_temp)
    }

def MsgType2(msg: bytes):
    """
    Message Type 2

    Interprets the message type 2 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    
    """

    getPosition = lambda data: data
    getCurrent = lambda data: data / 10.0
    getTemp = lambda data: (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_pos = struct.unpack('!f', msg[1:5])[0]
    motor_current = int.from_bytes(msg[5:7])
    motor_temp = msg[7]

    return {
        "Message Type" : 2,
        "Error" : ErrorMap[error], 
        "Position" : getPosition(motor_pos), 
        "Current" : getCurrent(motor_current),
        "Temperature" : getTemp(motor_temp)
    }




def MsgType3(msg: bytes) :
    """
    Message Type 3  
    
    Interprets the message type 3 and returns a list of the results 

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    """
    getSpeed = lambda data: data
    getCurrent = lambda data: data / 10.0
    getTemp = lambda data: (data - 50.0) / 2.0

    error = msg[0] & 0x1F
    motor_speed = struct.unpack('!f', msg[1:5])[0]
    motor_current = int.from_bytes(msg[5:7])
    motor_temp = msg[7]

    return {
        "Message Type" : 3,
        "Error" : ErrorMap[error], 
        "Speed" : getSpeed(motor_speed),
        "Current" : getCurrent(motor_current),
        "Temperature" : getTemp(motor_temp)
    }



def MsgType4(msg: bytes):
    """
    Message Type 4 

    Interprets the message type 4 and returns a list of the results

    Args:
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    
    """
    error = msg[0] & 0x1F
    configuration_code = msg[1]
    configuration_status = msg[2]

    return {
        "Message Type": 4,
        "Error": ErrorMap[error], 
        "Configuration Code": ConfigMap[configuration_code], 
        "Configuration Status": ConfigStatusMap[configuration_status]
    }


def MsgType5(msg: bytes):
    """Message Type 5
    Interprets the message type 5 and returns a list of the results
    Args:   
        msg: bytes: The message to interpret

    Returns:
        dictionary of the message results
    
    """
    error = msg[0] & 0x1F
    query_code = msg[1]
    data = bytes(msg[2:])
    data = struct.unpack('!f',data)[0]

    return {
        "Message Type": 5,
        "Error": ErrorMap[error], 
        "Query Code": QueryMap[query_code],
        "Data": data
    }

MsgMap = {
    1: MsgType1,
    2: MsgType2,
    3: MsgType3,
    4: MsgType4,
    5: MsgType5
}


def valid_message(msg: bytes):
    """Checks if the message is valid by checking the message type

    Args:
        msg: bytes: The message to check

    Returns:
        bool: True if the message is valid, False otherwise
    
    """
    return getMsgType(msg) in MsgMap


def read_result(msg: bytes):
    """Reads the result of the message and returns a list of the results regardless of Message Type

    Args:
        msg: bytes: The message to interpret
        
    Returns:
        dictionary of the message results
    
    """
    msgType = getMsgType(msg)
    if valid_message(msg):
        return MsgMap[msgType](msg)
    return None

if __name__ == "__main__":
    vector = [0xA0, 0x01, 0x39, 0xF7, 0x24, 0x7D]
    print(read_result(vector))
