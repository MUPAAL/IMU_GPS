#!/usr/bin/env python3

import os

import cv2
import depthai as dai

_DEVICE_IP = os.environ.get("DEVICE_IP", None)

# Create pipeline
if _DEVICE_IP:
    device = dai.Device(dai.DeviceInfo(_DEVICE_IP))
else:
    device = dai.Device()
with dai.Pipeline(device) as pipeline:
    outputQueues = {}
    sockets = device.getConnectedCameras()
    for socket in sockets:
        cam = pipeline.create(dai.node.Camera).build(socket)
        outputQueues[str(socket)] = cam.requestFullResolutionOutput().createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        for name in outputQueues.keys():
            queue = outputQueues[name]
            videoIn = queue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            # Visualizing the frame on slower hosts might have overhead
            cv2.imshow(name, videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
