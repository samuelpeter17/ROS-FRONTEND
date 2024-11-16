from flask import Flask, jsonify, send_file  # type: ignore
import rospy  # type: ignore
from std_msgs.msg import String  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from sensor_msgs.msg import Image  # type: ignore
from sensor_msgs.msg import LaserScan  # type: ignore
from gazebo_msgs.msg import ModelStates  # type: ignore
from rosgraph_msgs.msg import Clock  # type: ignore
import subprocess
import sys
from PIL import Image as PILImage # type: ignore
import numpy as np # type: ignore
import io
from io import BytesIO
import matplotlib.pyplot as plt

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
    # Convert structured ROS message to JSON
    data = {
        "position": {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
        },
        "orientation": {
            "x": msg.pose.pose.orientation.x,
            "y": msg.pose.pose.orientation.y,
            "z": msg.pose.pose.orientation.z,
            "w": msg.pose.pose.orientation.w,
        },
        "linear_velocity": {
            "x": msg.twist.twist.linear.x,
            "y": msg.twist.twist.linear.y,
            "z": msg.twist.twist.linear.z,
        },
        "angular_velocity": {
            "x": msg.twist.twist.angular.x,
            "y": msg.twist.twist.angular.y,
            "z": msg.twist.twist.angular.z,
        },
    }
    message_history['/odom'].append(data)
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
def image_callback(msg):
    # Store the latest message as a dictionary
    message_history['/camera/rgb/image_raw'] = {
        "header": {"seq": msg.header.seq, "stamp": str(msg.header.stamp), "frame_id": msg.header.frame_id},
        "height": msg.height,
        "width": msg.width,
        "encoding": msg.encoding,
        "is_bigendian": msg.is_bigendian,
        "step": msg.step,
        "data": list(msg.data)  # Complete raw image data
    }

rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

@app.route('/camera/rgb/image_raw', methods=['GET'])
def get_image():
    latest_message = message_history.get('/camera/rgb/image_raw', None)
    if not latest_message:
        return jsonify({"error": "No image data available."}), 404

    # Extract image data
    width = latest_message['width']
    height = latest_message['height']
    raw_data = latest_message['data']
    image_array = np.array(raw_data, dtype=np.uint8).reshape((height, width, 3))  # Assuming RGB format

    # Convert to PIL image
    pil_image = PILImage.fromarray(image_array, mode='RGB')

    # Save to a BytesIO buffer as PNG
    buffer = io.BytesIO()
    pil_image.save(buffer, format="PNG")
    buffer.seek(0)

    return send_file(buffer, mimetype='image/png')
# -----------------------------------
# /scan Topic Functionality
# -----------------------------------
# Global variable to store the latest scan data
latest_scan_data = {}

def scan_callback(msg):
    # Filter out Infinity values and calculate range_max
    valid_ranges = [r for r in msg.ranges if r > 0 and r < float('inf')]
    range_max = max(valid_ranges) if valid_ranges else 0

    # Store the latest scan data
    scan_data = {
        "ranges": [r if r != float('inf') else range_max for r in msg.ranges],
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "range_max": range_max
    }
    
    # Update latest scan data
    latest_scan_data.update(scan_data)

# Subscribe to /scan topic to get laser scan data
rospy.Subscriber('/scan', LaserScan, scan_callback)

@app.route('/scan', methods=['GET'])
def get_scan_message():
    # Return JSON scan data to frontend
    return jsonify(latest_scan_data)

@app.route('/scan-image', methods=['GET'])
def get_scan_image():
    # Create the scan image from the data
    if latest_scan_data:
        # Extract scan data
        ranges = latest_scan_data.get("ranges", [])
        angle_min = latest_scan_data.get("angle_min", 0)
        angle_max = latest_scan_data.get("angle_max", 0)
        range_max = latest_scan_data.get("range_max", 0)

        # Calculate the angles for the scan data
        angles = np.linspace(angle_min, angle_max, len(ranges))

        # Create the plot
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection='polar')
        ax.plot(angles, ranges, color='blue')
        ax.set_ylim(0, range_max)  # Set the range limits

        # Save the plot to a BytesIO object
        img = BytesIO()
        plt.savefig(img, format='png')
        img.seek(0)
        plt.close(fig)  # Close the plot to free memory

        # Send the image as a response
        return send_file(img, mimetype='image/png')
    else:
        return jsonify({"error": "No scan data available"}), 404
    
# -----------------------------------
# /gazebo/model_states Topic Functionality
# -----------------------------------
def model_states_callback(msg):
    data = []
    for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
        # Only append velocities if they are non-zero or valid
        linear_velocity = {
            "x": twist.linear.x if twist.linear.x != 0 else None,
            "y": twist.linear.y if twist.linear.y != 0 else None,
            "z": twist.linear.z if twist.linear.z != 0 else None
        }
        
        angular_velocity = {
            "x": twist.angular.x if twist.angular.x != 0 else None,
            "y": twist.angular.y if twist.angular.y != 0 else None,
            "z": twist.angular.z if twist.angular.z != 0 else None
        }
        
        # Add the model's data
        data.append({
            "name": name,
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "linear_velocity": linear_velocity,
            "angular_velocity": angular_velocity
        })
    
    # Save the message to history
    message_history['/gazebo/model_states'].append(data)
    
    # Maintain history limit
    if len(message_history['/gazebo/model_states']) > 10:
        message_history['/gazebo/model_states'].pop(0)

# Subscriber for /gazebo/model_states topic
rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

@app.route('/gazebo/model_states', methods=['GET'])
def get_model_states_message():
    latest_message = message_history['/gazebo/model_states'][-1] if message_history['/gazebo/model_states'] else "No messages yet."
    return jsonify({"message": latest_message, "message-history": message_history['/gazebo/model_states']})

# -----------------------------------
# /clock Topic Functionality
# -----------------------------------
def clock_callback(msg):
    data = {"secs": msg.clock.secs, "nsecs": msg.clock.nsecs}
    message_history['/clock'].append(data)
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
