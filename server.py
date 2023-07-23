import socket as skt
import dance_template
from pymavlink import mavutil

# creates and returns a socket
def createSocket():
    protocol = skt.SOCK_STREAM  # Protocol = TCP
    addressFamily = skt.AF_INET  # Address family = IPv4

    # socket creation
    socket = skt.socket(addressFamily, protocol)

    # Bind information
    serverIp = "192.168.1.185"
    serverPrt = 6777

    # Binding port and IP
    socket.bind((serverIp, serverPrt))

    return socket

# logs request
def requestProcessor(request):
    command, time_str = request.decode().split()
    time = int(time_str)

    commandDict = {
        "forward": [-100, -100, 100, 100],
        "backward": [100, 100, -100, -100],
        "stop": [0, 0, 0, 0]
    }
    vector = commandDict[command]

    # executing request
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    dance_template.arm_rov(mav_connection)
    # Arm the ROV and wait for confirmation
    dance_template.run_motors_timed(mav_connection, seconds=time, motor_settings=vector)

# create socket
socket = createSocket()

# listen for incoming connections
socket.listen()

while True:
    try:
        # accept a connection from the client
        conn, addr = socket.accept()

        # receive data
        request = conn.recv(1024)

        # executes request - (command time)
        requestProcessor(request)

        # close the connection
        conn.close()

    except KeyboardInterrupt:
        print("Server stopped.")
        break

# close the server socket
socket.close()
