import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import http.server
import socketserver
import threading
import requests

class WLEDServerHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=".", **kwargs)

    def do_GET(self):
        print(f"Received GET request: {self.path}")
        super().do_GET()

wled_device_address = ""  # Define wled_device_address as a global variable

def wled_server_thread_func():
    PORT = 8080
    Handler = WLEDServerHandler

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"Serving at port {PORT} for WLED device at {wled_device_address}")
        httpd.serve_forever()

def string_callback(msg):
    global wled_device_address
    # Handle String messages (e.g., update ticker text or control lights)
    command = msg.data.lower()

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
                    "sx": 200,    # Relative effect speed
                    "ix": 128,  # Effect intensity - Y Offset
                    "c1": 0,    # Effect custom slider 1 - Trail
                    "c2": 128,  # Effect custom slider 2 - Font size
                    "c3": 0,   # Effect custom slider 3 - ?
                    "n": msg.data,  # Set the text content
                }
            ],
            # Additional global settings
            "on": True,         # WLED on/off state
            "bri": 128,         # Brightness (0 to 255)
            "cct": 127,         # White spectrum color temperature (0 to 255 or 1900 to 10091)
            "pal": 0,           # ID of the color palette
            "sel": True,        # Segment selected state
            "rev": False,       # Segment flip state
            "mi": False,        # Segment mirror state
            "rY": False,        # Segment Rotate state
            "mY": False,        # Segment Mirror state
            "tp": False,        # Effect option 1
            "o1": False,        # Effect option 2
            "o2": False,        # Effect option 3
            "o3": False,        # Effect option 4
            "frz": False,       # Freeze/unfreeze the current effect
            "m12": 0,           # Setting of segment field 'Expand 1D FX'
            "si": 0             # Setting of the sound simulation type for audio enhanced effects
        }

        # Send the JSON payload to the WLED device
        try:
            response = requests.post(f"http://{wled_device_address}/json/state", json=payload)
            response.raise_for_status()
            rospy.loginfo(f"Command sent successfully to WLED device. Response: {response.text}")
        except requests.RequestException as e:
            rospy.logerr(f"Error sending command to WLED device: {e}")

def image_callback(msg):
    # Handle Image messages (e.g., update served image)
    rospy.loginfo("Received Image message")
    # Convert Image message to OpenCV format using CvBridge
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # Your image processing logic here

def main():
    global wled_device_address
    rospy.init_node("wled_server_node")

    # Retrieve WLED device address from the parameter server
    wled_device_address = rospy.get_param("~wled_device_address", "localhost")
    rospy.loginfo(f"WLED device address: {wled_device_address}")

    # Start WLED server thread with the retrieved address
    wled_server_thread = threading.Thread(target=wled_server_thread_func)
    wled_server_thread.start()

    # Subscribe to String and Image messages
    rospy.Subscriber("wled_bridge/ticker_text", String, string_callback)
    rospy.Subscriber("wled_bridge/image", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Terminating WLED server...")
        # Perform cleanup actions here if needed
        wled_server_thread.join()  # Wait for the server thread to finish

if __name__ == "__main__":
    main()

