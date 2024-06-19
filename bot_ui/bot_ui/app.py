from flask import Flask, render_template, request, redirect, url_for
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bot_msgs.action import BotTaskAction

app = Flask(__name__)

class TaskClient(Node):
    def __init__(self):
        super().__init__('task_client')
        self._action_client = ActionClient(self, BotTaskAction, 'task_server')

    def send_goal(self, task_number):
        goal_msg = BotTaskAction.Goal()
        goal_msg.task_number = task_number

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

rclpy.init()
task_client = TaskClient()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_goal', methods=['POST'])
def send_goal():
    task_number = int(request.form['task_number'])
    task_client.send_goal(task_number)
    return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(debug=True)
    rclpy.spin(task_client)
    task_client.destroy_node()
    rclpy.shutdown()
