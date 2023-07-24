import socket as skt
import dance_template
from pymavlink import mavutil


# creates and returns a socket
def createSocket():
    protocol = skt.SOCK_STREAM  # Protocol = TCP
    addressFamily = skt.AF_INET  # Address family = IPv4

    # socket creation
    socket = skt.socket(addressFamily, protocol)
    socket.setsockopt(skt.SOL_SOCKET, skt.SO_KEEPALIVE, 1)
    # Bind information
    serverIp = "10.29.122.55"
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
        "clockwise": [100, -100, -100, 100],
        "counter_clockwise": [-100, 100, 100, -100],  
        "left": [-100, 100, -100, 100],
        "right": [100, -100, 100, -100],
        "stop": [0, 0, 0, 0]
    }
    if command in commandDict:
        vector = commandDict[command]
    else:
        print("That's not a command.")
        return
    
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    dance_template.arm_rov(mav_connection)
    dance_template.run_motors_timed(mav_connection, seconds = time, motor_settings=vector)
    

if __name__ == "__main__":

    # create socket
    socket = createSocket()

    # listen for incoming connections
    socket.listen()

    try:
        while True:
            # accept a connection from the client
            conn, addr = socket.accept()

            while True:
                # receive data
                request = conn.recv(1024)

                if not request:
                    # If the client closed the connection, break the loop
                    break

                # Add the command to the queue
                requestProcessor(request)

            # Close the connection after processing the requests
            conn.close()

    except KeyboardInterrupt:
        print("Server stopped.")

    # close the server socket
    socket.close()



