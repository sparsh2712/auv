import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Set up camera
cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Use the appropriate camera
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
cam.setFps(30)

# Create output
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.video.link(xout.input)

# Connect to the device and start the pipeline
with dai.Device(pipeline) as device:
    q = device.getOutputQueue("video", maxSize=8, blocking=False)

    while True:
        frame = q.get()  # Get a frame
        frame = frame.getCvFrame()  # Convert to OpenCV format
        
        # Display the frame
        cv2.imshow("DepthAI Video", frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()

