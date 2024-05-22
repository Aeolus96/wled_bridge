import http.server
import socketserver
import threading
import time

import cv2  # Needed on arm64 systems because cv_bridge is commonly amd64
import requests
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from std_msgs.msg import String

from wled_bridge.cfg import WledBridgeParamsConfig

# Define global variables
wled_device_address = ""  # Define wled_device_address as a global variable
matrix_width = 32
matrix_height = 8
brightness = 128
debug_mode = False
shutdown_event = threading.Event()  # Event to signal thread termination


class WLEDServerHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=".", **kwargs)

    def do_GET(self):
        rospy.loginfo(f"Received GET request: {self.path}")
        super().do_GET()


def wled_server_thread_func():
    PORT = 8080
    Handler = WLEDServerHandler
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        rospy.loginfo(f"Serving at port {PORT} for WLED device at {wled_device_address}")
        while not shutdown_event.is_set():
            httpd.handle_request()
            time.sleep(1)  # Sleep for 1 second to avoid flooding
        httpd.shutdown()
    rospy.loginfo("WLED server thread exited...")


def dyn_rcfg_callback(config, level):
    global matrix_width, matrix_height, debug_mode, brightness
    # Update dynamic parameters based on reconfiguration
    matrix_width = config["matrix_width"]
    matrix_height = config["matrix_height"]
    brightness = config["brightness"]
    debug_mode = config["debug"]
    return config


# Handle String messages (e.g., update ticker text or control lights)
def string_callback(msg):
    rospy.loginfo("Received String message")
    command = msg.data.lower()

    if debug_mode:
        rospy.loginfo(f"Received String: {command}")

    if command == "on":
        # Command to turn on WLED
        payload = {"on": "t", "v": True}
    elif command == "off":
        # Command to turn off WLED
        payload = {"on": "t", "v": False}
    else:
        rospy.loginfo(f"Updating WLED text to: {msg.data}")
        # Construct the JSON payload
        payload = {
            "seg": [
                {
                    "id": 0,
                    "fx": 122,  # 122 corresponds to the scrolling text effect
                    "sx": 200,  # Relative effect speed
                    "ix": 128,  # Effect intensity - Y Offset
                    "c1": 0,  # Effect custom slider 1 - Trail
                    "c2": 128,  # Effect custom slider 2 - Font size
                    "c3": 16,  # Effect custom slider 3 - ?
                    "n": msg.data,  # Set the text content
                }
            ],
            # Additional global settings
            "on": True,  # WLED on/off state
            "bri": brightness,  # Brightness (0 to 255)
            "cct": 127,  # White spectrum color temperature (0 to 255 or 1900 to 10091)
            "pal": 0,  # ID of the color palette
            "sel": True,  # Segment selected state
            "rev": False,  # Segment flip state
            "mi": False,  # Segment mirror state
            "rY": False,  # Segment Rotate state
            "mY": False,  # Segment Mirror state
            "tp": False,  # Effect option 1
            "o1": False,  # Effect option 2
            "o2": False,  # Effect option 3
            "o3": False,  # Effect option 4
            "frz": False,  # Freeze/unfreeze the current effect
            "m12": 0,  # Setting of segment field 'Expand 1D FX'
            "si": 0,  # Setting of the sound simulation type for audio enhanced effects
        }

    # Send the JSON payload to the WLED device
    try:
        response = requests.post(f"http://{wled_device_address}/json/state", json=payload)
        response.raise_for_status()
        rospy.loginfo(f"Command sent successfully to WLED device. Response: {response.text}")
    except requests.RequestException as e:
        rospy.logerr(f"Error sending command to WLED device: {e}")


# Handle Image messages (e.g., update served image into WLED matrix)
def image_callback(msg):
    rospy.loginfo("Received Image message")
    # Convert Image message to OpenCV format using CvBridge
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Resize the image to match the number of LEDs in your WLED strip
    cv_image_resized = cv2.resize(cv_image, (matrix_width, matrix_height))

    if debug_mode:
        # Resize the image 10x in all dimensions
        display_image = cv2.resize(
            cv_image_resized, (matrix_width * 10, matrix_height * 10), interpolation=cv2.INTER_AREA
        )
        cv2.imshow("Image sent to WLED", display_image)
        cv2.waitKey(0)

    # Initialize an empty list to store LED index and hex color values
    led_colors = []

    # Iterate through each pixel in the resized image
    for row in range(matrix_height):
        for col in range(matrix_width):
            # Extract RGB values from the pixel
            pixel_color = cv_image_resized[row, col]
            r, g, b = pixel_color

            # Convert BGR to hex
            hex_color = "{:02x}{:02x}{:02x}".format(int(b), int(g), int(r))

            # Append LED index and hex color to the list
            led_colors.extend([col + row * matrix_width, hex_color])

    # Construct the JSON payload
    payload = {
        "seg": [
            {
                "i": led_colors,  # Individual LED hex color values
            }
        ],
        "on": True,  # WLED on/off state
        "bri": brightness,  # Brightness (0 to 255)
        # Additional settings...
    }

    # Send the JSON payload to the WLED device
    try:
        response = requests.post(f"http://{wled_device_address}/json/state", json=payload)
        response.raise_for_status()
        rospy.loginfo(f"Image sent successfully to WLED device. Response: {response.text}")
    except requests.RequestException as e:
        rospy.logerr(f"Error sending image to WLED device: {e}")


def main():
    global wled_device_address
    rospy.init_node("wled_server_node", disable_signals=True)

    # Load the dynamic reconfigure server
    srv = Server(WledBridgeParamsConfig, dyn_rcfg_callback)

    # Retrieve WLED device address from the parameter server
    wled_device_address = rospy.get_param("~wled_device_address", "localhost")
    rospy.loginfo(f"WLED device address: {wled_device_address}")

    # Start WLED server thread with the retrieved address
    wled_server_thread = threading.Thread(target=wled_server_thread_func, daemon=True)
    wled_server_thread.start()

    # Subscribe to String and Image messages
    rospy.Subscriber("wled_bridge/ticker_text", String, string_callback)
    rospy.Subscriber("wled_bridge/image", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Terminating WLED server...")
        shutdown_event.set()  # Set the shutdown event to signal threads to stop
        wled_server_thread.join()  # Wait for the server thread to finish
        cv2.destroyAllWindows()  # Close the imshow windows
        time.sleep(1)  # Sleep for 1 second to give time for the server thread to finish
        rospy.signal_shutdown("KeyboardInterrupt detected")  # Shut down the ROS node


if __name__ == "__main__":
    main()
