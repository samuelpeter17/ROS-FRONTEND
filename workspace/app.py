from flask import Flask, jsonify # type: ignore
import rospy # type: ignore
from std_msgs.msg import String # type: ignore
import subprocess
import sys

# Check and install flask-cors if not already installed
try:
    from flask_cors import CORS # type: ignore
except ImportError:
    print("Flask-CORS not found. Installing it now...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask-cors"])
    from flask_cors import CORS # type: ignore

# Initialize the Flask app
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Dictionary to store messages for each topic
message_history = {
    '/ros_message': []
}

# Initialize a counter for unique messages
message_counter = 1

# Callback function that stores messages for the /ros_message topic
def message_callback(msg):
    # Add the received message to the message history
    message_history['/ros_message'].append(msg.data)
    # Keep only the last 10 messages
    if len(message_history['/ros_message']) > 10:
        message_history['/ros_message'].pop(0)

# Initialize the ROS node
rospy.init_node('flask_ros_node')

# Subscribe to the /ros_message topic
rospy.Subscriber('/ros_message', String, message_callback)

# Function to automatically publish messages to /ros_message every 2 seconds
def publish_message(event):
    global message_counter
    pub = rospy.Publisher('/ros_message', String, queue_size=10)
    msg = String()
    msg.data = f"Automated message #{message_counter} from ROS"
    pub.publish(msg)
    message_counter += 1  # Increment the counter after each message

# Set a timer to publish messages every 2 seconds
rospy.Timer(rospy.Duration(2), publish_message)

@app.route('/ros_message', methods=['GET'])
def get_ros_message():
    # Return the latest message and message history
    latest_message = message_history['/ros_message'][-1] if message_history['/ros_message'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/ros_message']})

if __name__ == '__main__':
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)  # Make Flask accessible externally
