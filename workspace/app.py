from flask import Flask, jsonify  # type: ignore
import rospy  # type: ignore
from std_msgs.msg import String  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
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
    '/odom': []
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
# Callback for /odom topic
def odom_callback(msg):
    # Convert Odometry message to a simplified dictionary (e.g., position and orientation)
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
    # Store the message in the history
    message_history['/odom'].append(odom_data)
    # Keep only the last 10 messages
    if len(message_history['/odom']) > 10:
        message_history['/odom'].pop(0)

# Subscriber for /odom
rospy.Subscriber('/odom', Odometry, odom_callback)

@app.route('/odom', methods=['GET'])
def get_odom_message():
    # Return the latest message and message history for /odom
    latest_message = message_history['/odom'][-1] if message_history['/odom'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/odom']})

# -----------------------------------
# Flask App Runner
# -----------------------------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Make Flask accessible externally
