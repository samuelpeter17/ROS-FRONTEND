from flask import Flask, jsonify
import rospy
from std_msgs.msg import String
import subprocess
import sys

# Check and install flask-cors if not already installed
try:
    from flask_cors import CORS
except ImportError:
    print("Flask-CORS not found. Installing it now...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask-cors"])
    from flask_cors import CORS

# Initialize the Flask app
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Dictionary to store messages for each topic
message_history = {
    '/ros_message': []
}

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

# Publisher for the /ros_message topic
pub = rospy.Publisher('/ros_message', String, queue_size=10)

# Function to periodically publish messages
def publish_ros_message(event):
    msg = String()
    msg.data = "Automated message from ROS at every interval"
    pub.publish(msg)

# Timer to call the publish_ros_message function every 5 seconds
rospy.Timer(rospy.Duration(5), publish_ros_message)

@app.route('/ros_message', methods=['GET'])
def get_ros_message():
    # Return the latest message and message history
    latest_message = message_history['/ros_message'][-1] if message_history['/ros_message'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/ros_message']})

@app.route('/publish_message', methods=['POST'])
def publish_ros_message_route():
    # Publish a new message to the /ros_message topic via HTTP request
    msg = String()
    msg.data = "Hello from Flask to ROS!"
    pub.publish(msg)
    return jsonify({"status": "Message published to ROS"})

if __name__ == '__main__':
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)  # Make Flask accessible externally
