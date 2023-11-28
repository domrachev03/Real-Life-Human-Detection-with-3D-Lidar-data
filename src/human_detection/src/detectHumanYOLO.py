import cv2
import numpy as np
import os

# Load YOLO
net = cv2.dnn.readNet("yolov4.weights", "yolov4.cfg")

# Load the COCO class labels
classes = ['person']

# Load image
image = cv2.imread("left_camera_0135.jpg")
height, width = image.shape[:2]

# Create a blob from the image
blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
net.setInput(blob)

# Get the output layer names
layer_names = net.getUnconnectedOutLayersNames()

# Run YOLO object detection
detections = net.forward(layer_names)

# Minimum confidence score for detection
conf_threshold = 0.95

# Loop through the detections
for detection in detections:
    for obj in detection:
        scores = obj[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > conf_threshold and classes[class_id] == "person":
            # Get the coordinates of the bounding box
            center_x, center_y, box_width, box_height = obj[:4] * np.array([width, height, width, height])
            x = int(center_x - (box_width / 2))
            y = int(center_y - (box_height / 2))
            # Draw the bounding box and label on the image
            cv2.rectangle(image, (x, y), (x + int(box_width), y + int(box_height)), (0, 255, 0), 2)
            label = f"{classes[class_id]}: {confidence:.2f}"
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Show the image with detections
cv2.imwrite('output_image.jpg', image)
cv2.imshow("YOLO Object Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
