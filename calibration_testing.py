import socket
import json
import time

def calibration_testing():
    print('Connecting to Robot...')
    robot_ip = '192.168.100.150'
    robot_port = 16001

    # Buffer
    buffer = b''
    seqID = 1  # number to send robot
    recvID = 0  # number to recieve from robot

    # Setup the communication socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sock.connect((robot_ip, robot_port))
    print('Remote Motion Interface Socket Established')

    # Get connection to Robot RMI
    response, buffer, recvID = connect_to_rmi(sock, buffer)
    robot_port = response["PortNumber"]  # Update the port number
    time.sleep(2)


    # Connect to the update port
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.connect((robot_ip, robot_port))

    # Initialize the Robot RMI .3secs is min. May Require additional time.
    # sock.send()
    response, buffer = read_system_status(sock, buffer)
    time.sleep(1)

    response, buffer = reset_rmi(sock, buffer)
    time.sleep(1)

    response, buffer = abort_rmi(sock, buffer)
    time.sleep(1)
    response, buffer, recvID = initialize_rmi(sock, buffer)
    time.sleep(1)

    # Set Speed Override
    message = SetSpdOvd(100)
    sock.send(message)

    # Set Payload
    message, seqID = frc_setpayload(seqID, 3)
    sock.send(message)

    # Set UFrame
    message, seqID = frc_setuframe(seqID, 0)
    sock.send(message)
    time.sleep(.3)

    # Set ToolFrame
    message, seqID = frc_setutool(seqID, 1)
    sock.send(message)
    time.sleep(.3)

    print('Getting Pose')
    pose, buffer = getCoordinates(sock, buffer)
    print("Robot pose:", pose)

    pose, buffer = getCoordinates(sock, buffer)
    save_robot_pose(pose, r"C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\robot_coordinates_calibration.txt")

    image_path = capture_realsense_image(r"C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\camera_calibration_images")

    extract_chessboard_points(
        image_path,
        board_size=(9, 7),
        square_size=20.0,
        output_path=r"C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\chessboard_image_coordinates.txt"
    )

def reset_rmi(sock, buffer):

    print("🔄 Sending FRC_Reset...")
    sock.send(b'{"Command":"FRC_Reset"}\r\n')

    response, buffer, _ = GetResponse(sock, buffer)
    if response.get("Command") != "FRC_Reset" or response.get("ErrorID", 1) != 0:
        raise RuntimeError(f"FRC_Reset failed: {response}")

    print("✅ RMI reset complete.")
    return response, buffer

def send_and_confirm(sock, buffer, message, target_seq):
    sock.send(message)
    while True:
        response, buffer, recvID = GetResponse(sock, buffer)
        if recvID >= target_seq:
            break
    return buffer, recvID

def connect_to_rmi(sock, buffer):

    sock.send(b'{"Communication":"FRC_Connect"}\r\n')

    response, buffer, recvID = GetResponse(sock, buffer)

    if response.get("Communication") != "FRC_Connect" or response.get("ErrorID") != 0:
        raise RuntimeError(f"Failed to connect via RMI: {response}")

    print("✅ Connected via RMI")
    return response, buffer, recvID

def explore_cartesian_directions(sock, buffer, seqID, step=50, count=10):

    current_pose, buffer = getCoordinates(sock, buffer)
    if current_pose is None:
        print("[ERROR] Could not retrieve current pose.")
        return buffer, seqID

    print("Starting pose:", current_pose)

    directions = [
        ("X", 0), ("Y", 1), ("Z", 2),
        ("W", 3), ("P", 4), ("R", 5)
    ]

    for name, idx in directions:
        print(f"Exploring {name} axis...")
        pose = current_pose.copy()
        for i in range(count):
            pose[idx] += step
            print(f"→ Moving to {name} step {i+1}: {pose}")
            current_seq = seqID
            message, seqID = LinearMotion(pose, seqID, termType='"FINE"', termVal=0)
            sock.send(message)

            while True:
                response, buffer, recvID = GetResponse(sock, buffer)
                if recvID >= current_seq:
                    break

            time.sleep(0.2)

    return buffer, seqID


def connect_to_rmi(sock, buffer):

    sock.send(b'{"Communication":"FRC_Connect"}\r\n')

    response, buffer, recvID = GetResponse(sock, buffer)

    if response.get("Communication") != "FRC_Connect" or response.get("ErrorID") != 0:
        raise RuntimeError(f"Failed to connect via RMI: {response}")

    print("Connected via RMI")
    return response, buffer, recvID

#This Defines the LinearMotion Instruction as outlined in RMI Manual Section 2.4.7
def LinearMotion(pos, seqID, spdType="mmSec", spd=2000, termType='"CNT"', termVal=100):
    cmdMsg = '"Instruction":"FRC_LinearMotion",'
    seqMsg = '"SequenceID":' + str(seqID) + ','
    cfgMsg = '"Configuration":{"UtoolNumber":1,"UFrameNumber":0, "Front":1,"Up":1,"Left":0,"Flip":0, "Turn4":0,"Turn5":0,"Turn6":0},'
    posMsg = '"Position":{ "X":' + str(pos[0]) + ',"Y":' + str(pos[1]) + ',"Z":' + str(pos[2]) + ',"W":' + str(pos[3]) + ',"P":' + str(pos[4]) + ',"R":' + str(pos[5]) + '},'
    spdMsg = '"SpeedType":' + spdType + ',' + '"Speed":' + str(spd) + ','
    termMsg = '"TermType":' + termType + ',' + '"TermValue":' + str(termVal)
    message = '{' + cmdMsg + seqMsg + cfgMsg + posMsg + spdMsg + termMsg + '}\r\n'
    print(message)
    message = message.encode('utf-8')
    seqID += 1
    return message, seqID

def read_system_status(sock, buffer):
    sock.send(b'{"Command":"FRC_GetStatus"}\r\n')
    for _ in range(20):  # avoid infinite loop
        response, buffer, _ = GetResponse(sock, buffer)
        if response.get("Command") == "FRC_GetStatus":
            break
    print("System status:", response)
    return response, buffer


def move_linear_pose(sock, buffer, pose, seqID, speed=250, termType="FINE", termVal=0,
                     spdType="mmSec", tool=1, frame=0):

    cmd = {
        "Instruction": "FRC_LinearMotion",
        "SequenceID": seqID,
        "Configuration": {
            "UToolNumber": tool,
            "UFrameNumber": frame,
            "Front": 1, "Up": 1, "Left": 0,
            "Flip": 0, "Turn4": 0, "Turn5": 0, "Turn6": 0
        },
        "Position": {
            "X": pose[0], "Y": pose[1], "Z": pose[2],
            "W": pose[3], "P": pose[4], "R": pose[5]
        },
        "SpeedType": spdType,
        "Speed": speed,
        "TermType": termType,
        "TermValue": termVal
    }

    message = (json.dumps(cmd) + "\r\n").encode('utf-8')
    sock.send(message)

    while True:
        response, buffer, recvID = GetResponse(sock, buffer)
        if recvID >= seqID:
            break

    return seqID + 1


def connect_to_rmi(sock, buffer):

    sock.send(b'{"Communication":"FRC_Connect"}\r\n')

    response, buffer, recvID = GetResponse(sock, buffer)

    if response.get("Communication") != "FRC_Connect" or response.get("ErrorID") != 0:
        raise RuntimeError(f"Failed to connect via RMI: {response}")

    print("Connected via RMI")
    return response, buffer, recvID

def abort_rmi(sock, buffer):

    print("Sending FRC_Abort to cancel all running RMI instructions...")
    sock.send(b'{"Command":"FRC_Abort"}\r\n')

    response, buffer, recvID = GetResponse(sock, buffer)
    if response.get("Command") != "FRC_Abort" or response.get("ErrorID", 1) != 0:
        raise RuntimeError(f"FRC_Abort failed: {response}")
    
    print("RMI program aborted successfully.")
    return response, buffer


def initialize_rmi(sock, buffer):
    
    sock.send(b'{"Command":"FRC_Initialize"}\r\n')

    response, buffer, recvID = GetResponse(sock, buffer)

    if response.get("Command") != "FRC_Initialize" or response.get("ErrorID", 1) != 0:
        raise RuntimeError(f"Failed to initialize robot: {response}")

    print("Robot RMI initialized")
    return response, buffer, recvID


def getCoordinates(sock, buffer, group=1, max_tries=20):

    req = {"Command":"FRC_ReadCartesianPosition","Group":group}
    sock.send((json.dumps(req)+"\r\n").encode('utf-8'))

    for _ in range(max_tries):
        response, buffer, _ = GetResponse(sock, buffer)
        if response.get("Command") == "FRC_ReadCartesianPosition":
            if response.get("ErrorID",1) != 0:
                raise RuntimeError(f"ReadCartesianPosition error: {response}")
            pos = response["Position"]
            return [pos["X"], pos["Y"], pos["Z"],
                    pos["W"], pos["P"], pos["R"]], buffer

    raise RuntimeError("Timed out waiting for FRC_ReadCartesianPosition")

def wait_for_ack(sock, buffer, target_seq, timeout=20.0):

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            resp, buffer, recvID = GetResponse(sock, buffer, timeout=1.0)
        except socket.timeout:
            continue
        if recvID >= target_seq:
            return resp, buffer, recvID
    raise TimeoutError(f"No ACK for seqID={target_seq} after {timeout}s")

def SetSpdOvd(val):
    cmdMsg = '"Command":"FRC_SetOverRide",'
    valMsg = '"Value":' + str(int(val))
    message = '{' + cmdMsg + valMsg + '}\r\n'
    print(message)
    message = message.encode('utf-8')
    return message

def frc_setuframe(seqID, framenum):

    instruction = {
        "Instruction": "FRC_SetUFrame",
        "SequenceID": seqID,
        "FrameNumber": framenum
    }
    instruction_str = str(instruction).replace("'", '"') + "\r\n"
    print(instruction_str)
    seqID += 1
    message = instruction_str.encode('utf-8')
    return message, seqID

def frc_setutool(seqID, toolnum):

    instruction = {
        "Instruction": "FRC_SetUTool",
        "SequenceID": seqID,
        "ToolNumber": toolnum
    }
    instruction_str = str(instruction).replace("'", '"') + "\r\n"
    print(instruction_str)
    seqID += 1
    message = instruction_str.encode('utf-8')
    return message, seqID

def frc_setpayload(seqID, payloadid):

    instruction = {
        "Instruction": "FRC_SetPayLoad",
        "SequenceID": seqID,
        "ScheduleNumber": payloadid
    }
    instruction_str = str(instruction).replace("'", '"') + "\r\n"
    print(instruction_str)
    seqID += 1
    message = instruction_str.encode('utf-8')
    return message, seqID

def frc_call(seqID, program_name: str):

    instruction = {
        "Instruction": "FRC_Call",
        "SequenceID": seqID,
        "ProgramName": program_name
    }
    instruction_str = str(instruction).replace("'", '"') + "\r\n"
    print(instruction_str)
    seqID += 1
    message = instruction_str.encode('utf-8')
    return message, seqID

def frc_abort():
    command = {"Command": "FRC_Abort"}
    message = command.encode('utf-8')
    print(message)
    return command

def Pause(sock):
    sock.send(b'{Command":"FRC_Pause"}\r\n')


def frc_continue():
    cmdMsg = '"Command":"FRC_Continue",'
    message = '{' + cmdMsg + '}\r\n'
    print(message)
    message = message.encode('utf-8')
    return message

def GetResponse(sock, buffer, timeout=10.0):
    delim = b'\r\n'
    if not delim in buffer:
        sock.settimeout(timeout)
        response = sock.recv(1024)
        sock.settimeout(None)
        buffer += response
    i = buffer.find(delim)
    response = buffer[:i]
    i += len(delim)
    buffer = buffer[i:]

    response = response.decode('utf-8')
    print('received message: %s' % response)
    response = json.loads(response)

    recvSeqID = response["SequenceID"] if ('SequenceID' in response) else 0
    return response, buffer, recvSeqID

def save_robot_pose(pose, path):
    with open(path, 'a') as f:
        f.write(','.join([str(p) for p in pose]) + '\n')

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

def capture_realsense_image(save_dir):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    for _ in range(100):  
        frames = pipeline.wait_for_frames()

    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(save_dir, f"calib_image_{timestamp}.png")
    cv2.imwrite(save_path, color_image)
    # cv2.imshow("Captured Image", color_image)
    # cv2.waitKey(0)

    pipeline.stop()
    print(f"Image saved to: {save_path}")
    return save_path

def extract_chessboard_points(image_path, board_size=(9, 7), square_size=20.0, output_path=None):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, board_size)
    if not found:
        raise ValueError("Chessboard not found")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

    if output_path:
        with open(output_path, 'a') as f:
            f.write(';'.join([f"{pt[0][0]:.2f},{pt[0][1]:.2f}" for pt in corners]) + '\n')
    cv2.drawChessboardCorners(image, board_size, corners, found)
    cv2.imshow("Detected Chessboard", image)
    cv2.waitKey(0)
    print(f"Saved {len(corners)} image points to {output_path}")
    return corners


if __name__ == "__main__":
    calibration_testing()
