from flask import Flask, jsonify  # type: ignore
import rospy  # type: ignore
from std_msgs.msg import String  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from sensor_msgs.msg import Image  # type: ignore
from sensor_msgs.msg import LaserScan  # type: ignore
from gazebo_msgs.msg import ModelStates  # type: ignore
from rosgraph_msgs.msg import Clock  # type: ignore
import subprocess
import sys

# -----------------------------------
# Install Flask-CORS if not available
# -----------------------------------
try:
    from flask_cors import CORS  # type: ignore
except ImportError:
    print("Flask-CORS not found. Installing it now...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask-cors"])
    from flask_cors import CORS  # type: ignore

# -----------------------------------
# Flask App and Global Data
# -----------------------------------
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Dictionary to store messages for each topic
message_history = {
    '/ros_message': [],
    '/odom': [],
    '/camera/rgb/image_raw': [],
    '/scan': [],
    '/gazebo/model_states': [],
    '/clock': []
}

# -----------------------------------
# ROS Node Initialization
# -----------------------------------
rospy.init_node('flask_ros_node')

# -----------------------------------
# /ros_message Topic Functionality
# -----------------------------------
# Callback for /ros_message topic
def ros_message_callback(msg):
    # Store the message in the history
    message_history['/ros_message'].append(msg.data)
    # Keep only the last 10 messages
    if len(message_history['/ros_message']) > 10:
        message_history['/ros_message'].pop(0)

# Subscriber for /ros_message
rospy.Subscriber('/ros_message', String, ros_message_callback)

# Function to automatically publish messages to /ros_message every 2 seconds
message_counter = 1

def publish_ros_message(event):
    global message_counter
    pub = rospy.Publisher('/ros_message', String, queue_size=10)
    msg = String()
    msg.data = f"Automated message #{message_counter} from ROS"
    pub.publish(msg)
    message_counter += 1  # Increment the counter after each message

# Timer to publish messages every 2 seconds
rospy.Timer(rospy.Duration(2), publish_ros_message)

@app.route('/ros_message', methods=['GET'])
def get_ros_message():
    # Return the latest message and message history for /ros_message
    latest_message = message_history['/ros_message'][-1] if message_history['/ros_message'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/ros_message']})

# -----------------------------------
# /odom Topic Functionality
# -----------------------------------
def odom_callback(msg):
    odom_data = {
        'position': {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        },
        'orientation': {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
    }
    message_history['/odom'].append(odom_data)
    if len(message_history['/odom']) > 10:
        message_history['/odom'].pop(0)

rospy.Subscriber('/odom', Odometry, odom_callback)

@app.route('/odom', methods=['GET'])
def get_odom_message():
    latest_message = message_history['/odom'][-1] if message_history['/odom'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/odom']})

# -----------------------------------
# /camera/rgb/image_raw Topic Functionality
# -----------------------------------
# def image_callback(msg):
#     # Store the complete raw message as a dictionary
#     message_history['/camera/rgb/image_raw'].append({
#         "header": {"seq": msg.header.seq, "stamp": str(msg.header.stamp), "frame_id": msg.header.frame_id},
#         "height": msg.height,
#         "width": msg.width,
#         "encoding": msg.encoding,
#         "is_bigendian": msg.is_bigendian,
#         "step": msg.step,
#         "data": list(msg.data)  # This is the complete raw image data
#     })
#     if len(message_history['/camera/rgb/image_raw']) > 10:
#         message_history['/camera/rgb/image_raw'].pop(0)

def image_callback(msg):
    # Simplified representation of image data (actual image processing not included)
    image_data = {"height": msg.height, "width": msg.width, "encoding": msg.encoding}
    message_history['/camera/rgb/image_raw'].append(image_data)
    if len(message_history['/camera/rgb/image_raw']) > 10:
        message_history['/camera/rgb/image_raw'].pop(0)

rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

@app.route('/camera/rgb/image_raw', methods=['GET'])
def get_image_message():
    latest_message = message_history['/camera/rgb/image_raw'][-1] if message_history['/camera/rgb/image_raw'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/camera/rgb/image_raw']})

# -----------------------------------
# /scan Topic Functionality
# -----------------------------------
# def scan_callback(msg):
#     # Store the complete raw message as a dictionary
#     scan_data = {
#         "angle_min": msg.angle_min,
#         "angle_max": msg.angle_max,
#         "angle_increment": msg.angle_increment,
#         "time_increment": msg.time_increment,
#         "scan_time": msg.scan_time,
#         "range_min": msg.range_min,
#         "range_max": msg.range_max,
#         "ranges": list(msg.ranges),         # Full range data
#         "intensities": list(msg.intensities)  # Full intensity data
#     }
#     message_history['/scan'].append(scan_data)
#     if len(message_history['/scan']) > 10:
#         message_history['/scan'].pop(0)

def scan_callback(msg):
    scan_data = {"ranges": msg.ranges[:10], "intensities": msg.intensities[:10]}  # Send a slice for brevity
    message_history['/scan'].append(scan_data)
    if len(message_history['/scan']) > 10:
        message_history['/scan'].pop(0)

rospy.Subscriber('/scan', LaserScan, scan_callback)

@app.route('/scan', methods=['GET'])
def get_scan_message():
    latest_message = message_history['/scan'][-1] if message_history['/scan'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/scan']})

# -----------------------------------
# /gazebo/model_states Topic Functionality
# -----------------------------------
def model_states_callback(msg):
    models = [{"name": name, "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}} 
              for name, pose in zip(msg.name, msg.pose)]
    message_history['/gazebo/model_states'].append(models)
    if len(message_history['/gazebo/model_states']) > 10:
        message_history['/gazebo/model_states'].pop(0)

rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

@app.route('/gazebo/model_states', methods=['GET'])
def get_model_states_message():
    latest_message = message_history['/gazebo/model_states'][-1] if message_history['/gazebo/model_states'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/gazebo/model_states']})

# -----------------------------------
# /clock Topic Functionality
# -----------------------------------
def clock_callback(msg):
    clock_data = {"secs": msg.clock.secs, "nsecs": msg.clock.nsecs}
    message_history['/clock'].append(clock_data)
    if len(message_history['/clock']) > 10:
        message_history['/clock'].pop(0)

rospy.Subscriber('/clock', Clock, clock_callback)

@app.route('/clock', methods=['GET'])
def get_clock_message():
    latest_message = message_history['/clock'][-1] if message_history['/clock'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/clock']})

# -----------------------------------
# Flask App Runner
# -----------------------------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Make Flask accessible externally
