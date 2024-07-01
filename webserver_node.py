#!/usr/bin/env python3

# Imports for ROS2
import rclpy
from sensor_msgs.msg import Image

# Imports for OpenCV operations
import cv2
from cv_bridge import CvBridge

# Imports for threading operations
import signal, sys
from threading import Thread, Event

# Import for flask application
from flask import Flask, render_template, Response

# Global variables for frames
frame_raw = None
frame_black = None

# Objects of cvbridge and event
bridge = CvBridge()
event_raw = Event()
event_black = Event()

# Function to handle raw image callback
def raw_image_callback(msg):
    global frame_raw
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    frame_raw = cv2.imencode(".jpg", cv_image)[1].tobytes()
    event_raw.set()

# Function to handle black image callback
def black_image_callback(msg):
    global frame_black
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    frame_black = cv2.imencode(".jpg", cv_image)[1].tobytes()
    event_black.set()

# Initializing the node
rclpy.init(args=None)
node = rclpy.create_node('webserver_node')

Thread(target=lambda:node).start()

# Create subscriptions
subscription_cam_raw = node.create_subscription(Image, "/line_follow_cam_raw", raw_image_callback, 1)
subscription_cam_black = node.create_subscription(Image, "/line_follow_cam_black", black_image_callback, 1)

# Flask app initialization
app = Flask(__name__)


# Function to get frame based on stream type
def get_frame(stream_type):
	if stream_type == 'raw':
		try:
		        rclpy.spin_once(node, timeout_sec=1.0)
		        event_raw.wait()
		        event_raw.clear()
		        return frame_raw
		except:
			return frame_raw
		    
	elif stream_type == 'black':
		try:
		        rclpy.spin_once(node, timeout_sec=1.0)
		        event_black.wait()
		        event_black.clear()
		        return frame_black
		except:
			return frame_black
	else:
	        return None

# Generator function for raw video feed
def gen_raw():
	while True:
	        frame = get_frame('raw')
	        yield (b'--frame\r\n'
	               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Generator function for black video feed
def gen_black():
    while True:
        frame = get_frame('black')
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

## Function that loads the template (Flask app)
@app.route('/')
def index():
    return render_template('index.html')

# Route for raw video feed
@app.route('/video_feed_raw')
def video_feed_raw():
    return Response(gen_raw(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Route for black video feed
@app.route('/video_feed_black')
def video_feed_black():
    return Response(gen_black(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Signal handler for shutting down
def signal_handler(sig, frame):
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Main function to run the Flask app
def main(args=None):
    app.run(port=8080, debug=True, use_reloader=False, host="0.0.0.0")

if __name__ == '__main__':
    main()
