import csv

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

STEP_TOPIC = 'step_candidates' # step topic name
MATCH_WINDOW = 0.3 # Seconds that tap must come within for step be a match
WAIT_SEC = 2.0 # Seconds to wait after step before labeling
OUTPUT_FILE = 'labeled_steps.csv'

def to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
    
class LabelingNode(Node):

    def __init__(self):
        super().__init__('labeling_node')
        self.tap_subscription = self.create_subscription(Header, 'tap', self.tap_callback, 10)
        self.tap_subscription # From tutorial
        self.step_subscription = self.create_subscription(PointStamped, STEP_TOPIC, self.step_callback, 10)
        self.step_subscription # From tutorial
        self.waiting_taps = []
        self.waiting_steps = []
        
        self.real_count = 0
        self.false_count = 0
        self.missed_count = 0
        
        self.file = open(OUTPUT_FILE, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'label'])
        timer_period = 0.5 # in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Labeling node started. Writing to %s' % OUTPUT_FILE)
        
    def tap_callback(self, msg):
        self.waiting_taps.append(to_seconds(msg.stamp))
        
    def step_callback(self, msg):
        self.waiting_steps.append(to_seconds(msg.header.stamp))
        
    def timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.label_old_steps(now)
        
    def label_old_steps(self, now): # queue handling for steps
        still_waiting = []
        for step_time in self.waiting_steps:
            if now - step_time < WAIT_SEC:
                still_waiting.append(step_time)
                continue
                
            label = 0 # assuming false step until matching tap occurs
            
            for tap_time in self.waiting_taps:
                delay = tap_time - step_time
                if 0 <= delay <= MATCH_WINDOW:
                    label = 1 # true step
                    self.waiting_taps.remove(tap_time) # takes out tap so not labeled twice
                    break
                    
            self.save_row(step_time, label)
            
        self.waiting_steps = still_waiting # updates list
            
        still_waiting_taps = []
        for tap_time in self.waiting_taps: # Queue handling for taps
            if now - tap_time > WAIT_SEC + MATCH_WINDOW:
                self.missed_count += 1
            else:
                still_waiting_taps.append(tap_time)
                    
        self.waiting_taps = still_waiting_taps
            
    def save_row(self, step_time, label):
        self.writer.writerow([step_time, label])
        self.file.flush() # saves file
        
        if label == 1:
            self.real_count += 1
        else:
            self.false_count +=1
            
        self.get_logger().info('real: %d    false: %d   missed: %d' % (self.real_count, self.false_count, self.missed_count))
        
    def finish(self): # runs after canceling (ctrl + C)
        self.label_old_steps(float('inf'))
        self.file.close()
        print('Saved %s' % OUTPUT_FILE)
        print('Real: %d   False: %d   Missed: %d' % (self.real_count, self.false_count, self.missed_count))
        
def main(args=None):
    rclpy.init(args=args)
    labeling_node = LabelingNode()
    try:
        rclpy.spin(labeling_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        labeling_node.finish()
        labeling_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
