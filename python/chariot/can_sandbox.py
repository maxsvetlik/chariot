import can
import cantools

import time
import os

db = cantools.database.load_file("odrive-cansimple.dbc")
axes = [0x0, 0x1, 0x2, 0x3]
axes_decoded = [id << 5 for id in axes]
print("\n\rCAN Rx test")
print("Bring up CAN0....")
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
time.sleep(0.1)


def get_axis_info(axis_id):
    while True:
        msg = bus.recv()
        if msg.arbitration_id == (
            (axis_id << 5) | db.get_message_by_name("Heartbeat").frame_id
        ):
            current_state = db.decode_message("Heartbeat", msg.data)["Axis_State"]
            if current_state == 0x1:
                print(f"\nAxis {axis_id} has returned to Idle state.")
                break


def send(msg):
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent!")


def set_axis_closed_loop_mode(axis_id):
    print("\nPutting axis", axis_id, "into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
    data = db.encode_message("Set_Axis_State", {"Axis_Requested_State": 0x08})
    msg = can.Message(
        arbitration_id=0x07 | axis_id << 5, is_extended_id=False, data=data
    )
    send(msg)


def set_axis_closed_loop_mode(axis_id):
    print("\nPutting axis", axis_id, "into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
    data = db.encode_message("Set_Axis_State", {"Axis_Requested_State": 0x08})
    msg = can.Message(
        arbitration_id=0x07 | axis_id << 5, is_extended_id=False, data=data
    )
    send(msg)


try:
    bus = can.interface.Bus(channel="can0", bustype="socketcan")
except OSError:
    print("Cannot find PiCAN board.")
    exit()

print("Ready")

try:
    [get_axis_info(id) for id in axes]
except KeyboardInterrupt:
    # Catch keyboard interrupt
    os.system("sudo /sbin/ip link set can0 down")
    print("\n\rKeyboard interrtupt")
