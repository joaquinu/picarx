#!/usr/bin/env python3
import argparse
import cv2
import time
import sys
import numpy as np
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
import libcamera
#import utils

# Variables to calculate FPS
counter, fps = 0, 0
start_time = time.time()
fps_avg_frame_count = 10

def set_input_tensor(interpreter, image):
    """Sets the input tensor in the TensorFlow Lite interpreter."""
    input_detail = interpreter.get_input_details()[0]
    tensor_index = input_detail['index']
    input_tensor = interpreter.tensor(tensor_index)()[0]

    # Resize the frame to match the model's expected input shape
    image_resized = cv2.resize(image, (input_detail['shape'][2], input_detail['shape'][1]))

    # Ensure the image has 3 channels (RGB), remove alpha channel if present (RGBA -> RGB)
    if image_resized.shape[-1] == 4:
        image_resized = image_resized[:, :, :3]  # Remove the alpha channel

    # Normalize the image
    image_normalized = image_resized / 255.0
    
    # Insert the normalized image data into the tensor
    input_tensor[:, :] = image_normalized


def visualize(image, results):
    """Draws bounding boxes on the image based on the detected objects."""
    height, width, _ = image.shape
    for obj in results:
        ymin, xmin, ymax, xmax = obj['bounding_box']
        xmin = int(xmin * width)
        xmax = int(xmax * width)
        ymin = int(ymin * height)
        ymax = int(ymax * height)

        # Draw bounding box
        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

        # Add label and confidence score
        label = f"Class ID: {obj['class_id']}, Conf: {obj['score']:.2f}"
        cv2.putText(image, label, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return image


def get_output_tensor(interpreter, index):
    """Returns the output tensor at the given index."""
    output_details = interpreter.get_output_details()[index]
    tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
    return tensor


def detect_objects(interpreter, image, threshold=0.4):
    """Runs object detection on the provided image."""
    set_input_tensor(interpreter, image)
    interpreter.invoke()

    # Get classification results
    boxes = get_output_tensor(interpreter, 0)
    class_ids = get_output_tensor(interpreter, 1)
    scores = get_output_tensor(interpreter, 2)
    count = int(get_output_tensor(interpreter, 3))

    # Collect results with `score` above threshold
    results = []
    for i in range(count):
        if scores[i] >= threshold:
            result = {
                'bounding_box': boxes[i],
                'class_id': int(class_ids[i]),
                'score': scores[i]
            }
            results.append(result)
    
    return results


def run(model: str, width: int, height: int, threshold: float) -> None:
    """Continuously run inference on images from the camera and calculate FPS."""
    # Initialize the TensorFlow Lite model
    interpreter = Interpreter(model)
    interpreter.allocate_tensors()

    # Initialize camera using picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (width, height)})
    picam2.configure(config)
    picam2.start()

    print("Starting video stream. Press 'q' to exit.")

    global counter, fps, start_time

    while True:
        # Capture frame from the camera in RGB format
        frame = picam2.capture_array()

        # Convert the frame from RGB to BGR (for OpenCV display)
        output_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Run object detection (use the original RGB frame)
        results = detect_objects(interpreter, frame, threshold=threshold)

        # Draw the results on the output frame
        output_frame = visualize(output_frame, results)

        # Show FPS
        counter += 1
        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()

        # Add FPS text to the frame
        fps_text = f'FPS: {fps:.2f}'
        cv2.putText(output_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show the result in a window
        cv2.imshow('Object Detection', output_frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    picam2.close()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', required=False, default='efficientdet_lite0.tflite')
    parser.add_argument('--width', help='Width of the camera frame.', type=int, default=640)
    parser.add_argument('--height', help='Height of the camera frame.', type=int, default=480)
    parser.add_argument('--threshold', help='Confidence threshold for detected objects.', type=float, default=0.4)
    
    args = parser.parse_args()

    # Start the object detection run with the provided parameters
    run(args.model, args.width, args.height, args.threshold)


if __name__ == '__main__':
    main()