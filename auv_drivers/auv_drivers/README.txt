Descriptions of files in auv_drivers

router.py
 Matsya 7 @AUV-IITB
 
 The main router node that connects all tcp nodes. It starts a binding socket at port 5555 accessible on any local network. TCP over zmq is used in python. Technically this router, the USB driver, the ROS codebase, and the GUI could run on 4 different machines if we would like. However, router, driver_node, and USB will be on SBC, while GUI could be anywhere. The sole purpose of this script is to route data coming from different places to their destination. Some may wonder that IPC is faster than TCP, but a latency check shows a less than a 1.5x increase in speed using ipc, and so much optimisation is not needed when our max speeds of communicating are 10Hz for IMU, 10Hz for PS, 10Hz for PWMs, and less than that for every other message. Unbinding is NOT needed for zmq sockets, they are automatically garbage collected, yet they have been added explicity as socket.close() and context.term(). 
 
 Author 
 - Aniket Gupta (Elec '27)


usb.py
 Matsya 7 @AUV-IITB
 
 The new and very improved USB driver (not CAN). CAN has been discontinued in Matsya 7, and we communicate using USB. A lot of thought has gone into making a much more robust and efficient USB driver that can be easily expanded to accomodate extra commands. Effort has been put to maintain modularity and decoupling among the various parts: Only USB driver has pyserial, only the GUI has PyQt, and only driver node has ROS2 imported into it. 
 
 Message framing is used for sending and receiving messages over pyserial. Frame is as follows:
 [START_BYTE][MESSAGE_ID][NUMBER_OF_BYTES_OF_DATA][PAYLOAD * N][XOR_CHECKSUM][STOP_BYTE]
 Message is of variable length and has inbuilt error detection. 
 
 A zmq dealer socket connects to localhost port 5555 and communicats via TCP. Messages are polled sufficiently fast that nothing will be missed from neither from the USB side nor the TCP side. USB is read about once every millisecond, and TCP much faster than that. Pyserial port can be configured dynamically by sending commands mid program, and stopping of usb driver is not needed. The GUI can connect if it is present on the same network and it has replaced cutecom for debugging stack. 
 
 Data sent and received from TCP is also packetised in the following format:
 [RECEIVER][TYPE][DATA] where receiver is self.USB, self.ROS, self.GUI, or self.ALL. Type is only "command" if receiver is usb, otherwise there are several more. Data is necessarily a python list when type is command. The first index of data list is the function to be executed, and if there are arguments to be passed (depends on which function), at index 1 of the data list, give another list containing the arguments. 
 Eg: for sending PWM to an indivudal thruster, the tcp command will be
     send_to_router(self.USB, "command", ["send_thruster_individual_pwm", [thruster_index, pwm]])
 
 This driver is a direct compliment of the one in elec stack, so please DO NOT MODIFY without complete knowledge of what you are doing.
 For driver testing purposes, the debugging interface is the best choice, as messages that were barely readable earlier in cutecom, are now rendered completely unreadable by the new message format. Also, for transmitting floats and integers, the bytes get modified and unpacked later, further adding to the non-readability. The benefits however, do outweigh the shortcomings and the driver is much more robust now.
 
 Author 
 - Aniket Gupta (Elec '27)


node.py
 Matsya 7 @AUV-IITB '25
 
 The main communication point between all things software, and all things outside. 
 This ROS2 node sends information to ROS from the elec stack, like IMU and PS data (now on stack, instead of coming directly to SBC), stack logs, start of mission, servo states of marker dropper, torpedo shooter, and gripper, and also usb connection related things like getting available COM ports. 
 Similarly, it also sends PWMs, kill/unkill, servo actuation commands, power toggles, current state data to the USB driver which relays it to the elec stack. Additionally, there is a GUI which can communicate with the ROS node, by exchanging data like auv pose, pwms given, and setpoints given and also accept commands like localisation reset, home, and in the near future, editing yaml/json constants.
 
 This node contains a dealer socket connected via tcp, using the zmq library to localhost port 5555. Router binds this port. Data can be sent to USB, GUI, or all, conveniently mentioned using self.USB, self.GUI and self.ALL. Every 1 millisecond, this socket is polled for incoming data and a blocking call is made to corresponding callback function based on what data is received. Message structure and format is explained better in USB driver file. To run rclpy and the zmq socket together, multithreading/multiprocessing was determined to be unnecessary, considering the not very overwhelming volume of data. Spinning rclpy once, and polling zmq in a loop is more than sufficient for our application. 
 The serial port must be set in the main loop by sending commands and is not done automatically in the background. 
 
 Authors:
 - Akshika Jain (Software '27)
 - Aniket Gupta (Elec '27)
 