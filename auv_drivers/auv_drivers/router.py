import zmq
import pickle
import time
import sys

def router():
    context = zmq.Context()
    socket = context.socket(zmq.ROUTER)
    socket.bind("tcp://*:5555")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    clients = {b'USB', b'ROS', b'GUI'}
    print("Router: Waiting for messages...")

    start_time = time.time_ns()

    try:
        while True:
            message = socket.recv_multipart()
            if len(message) == 4: 
                sender_identity = message[0]
                recipient_identity = message[1] 
                dtype = message[2].decode()
                data = message[3]

                # print(sender_identity.decode(), recipient_identity.decode(), dtype, pickle.loads(data))

                if recipient_identity != b'ALL':
                    socket.send_multipart([recipient_identity, sender_identity, dtype.encode(), data])
                    # print(f"{(time.time_ns() - start_time) / 1000000}ms Router: Forwarded data from {sender_identity.decode()} to {recipient_identity.decode()}")
                else:
                    for client in clients:
                        if client != sender_identity:
                            socket.send_multipart([client, sender_identity, dtype.encode(), data])
                            # print(f"{(time.time_ns() - start_time) / 1000000}ms Router: Forwarded data from {sender_identity.decode()} -> {client.decode()}")
            else:
                print(f"Router: Received incomplete message -> {message}")
                print(len(message))

    except KeyboardInterrupt:
        print("\nCTRL+C received. Cleaning up...")
    finally:
        print("Unbinding socket and closing...")
        socket.close(linger=None)
        context.term() 
        sys.exit(0) 

if __name__ == "__main__":
    router()

# message format:
# Sent message: [Receiver, data type, data]
# Received message: [Sender, data type, data]
#
# to usb: data = [function name, [arguments]]
