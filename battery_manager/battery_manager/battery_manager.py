#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
import diagnostic_msgs.msg
from task_manager_interfaces.srv import AddTask

import json
import datetime
import time


class Battery(Node):

    # Class Variables
    batF = BatteryState()       # The battery filtered (moving average)
    list_battery_obs = []       # list of most current battery observations (for filtering)
    list_battery_obs_to_log = []
    docking_request = False
    dock_task_id = -2           # set initially to unused value


    def __init__(self):
        super().__init__("battery_manager")

        # read configuration params
        in_battery = self.declare_parameter('~input_battery_topic',"/battery").value

        out_battery = self.declare_parameter('~output_battery_topic',"/battery_filtered").value

        self.filter_lenght = self.declare_parameter('~filter_lenght_samples',30).value

        publish_rate = self.declare_parameter('~publish_rate',1).value

        self.log_data = self.declare_parameter('~create_log',False).value

        path = self.declare_parameter('~log_directory_path',"~").value

        self.log_interval = self.declare_parameter('~log_interval_sec',20).value

        self.low_bat_th = self.declare_parameter('~low_battery_voltage',12.5).value

        self.critical_bat_th = self.declare_parameter('~critial_battery_voltage',12).value

        self.docking_pose = self.declare_parameter("~docking_pose", []).value # list 2Dpose [x, y , phi]

        self.verbose = self.declare_parameter('~verbose',True).value

        # Create a new log with the starting date
        now = datetime.datetime.now()
        date = now.strftime('%Y-%m-%d-%H-%M-%S')
        # Daba error poniendo directamente path(pendiente areglar)
        self.filename = path + "/battery_manager_log_" + date
        
        # Subscriber
        self.sub1 = self.create_subscription(BatteryState, in_battery, self.battery_msg_cb, 10)
        self.sub2 = self.create_subscription(diagnostic_msgs.msg.KeyValue, "/ros2mqtt", self.task_ended_cb, 10)

        # Publisher
        self.pub = self.create_publisher(BatteryState, out_battery,100)

        # Service Client
        self.add_task = self.create_client(AddTask, '/bt_manager/add_new_task')

        # Log?
        if self.log_data in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']:
            self.log_data = True
            with open(self.filename, 'w') as log:
                log.write('#Battery Manager Log: Time Voltage Current Chager_status')

        # Init
        self.last_log_time = time.time()
        self.last_low_bat_time = self.get_clock().now()
        self.last_crit_bat_time = self.get_clock().now()
        self.list_battery_obs = []
        self.list_battery_obs_to_log = []

        # Create a timer Callback
        timer_period = 1/publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # show config:
        if self.verbose:
            self.get_logger().info("Battery_Manager running with: ")
            self.get_logger().info("low_battery_voltage: %.2f V" % (self.low_bat_th))
            self.get_logger().info("critial_battery_voltage: %.2f V" % (self.critical_bat_th))
              
    #---------
    # LOOP
    #--------- 
    def timer_callback(self):
            # Publish filtered battery
            if rclpy.ok(): 

                if self.verbose and len(self.list_battery_obs) == 0:
                    self.get_logger().info("No battery data received! Check input topic...")
                else:
                    # Filter data (low pass filtering)
                    average_voltage = 0.0
                    average_current = 0.0
                    for obs in self.list_battery_obs:
                        average_voltage += obs.voltage
                        average_current += obs.current
                    average_voltage = average_voltage / len(self.list_battery_obs)
                    average_current = average_current / len(self.list_battery_obs)

                    # check charger_status--> -1:Unknowm, 2/3:Discharging, 1:Charging, 4:Full
                    charger_status = self.list_battery_obs[-1].power_supply_status
                    # Check full condition (current must be close to 0)
                    if self.list_battery_obs[-1].power_supply_status == 4 and self.list_battery_obs[-1].current > 0.5:
                        charger_status = 1 #Still current comming in, set charging status.

                    # Publish filtered values
                    self.batF.header.stamp = self.get_clock().now().to_msg()
                    self.batF.header.frame_id = 'base_link'
                    self.batF.present = True
                    self.batF.voltage = average_voltage
                    self.batF.current = self.list_battery_obs[-1].current
                    self.batF.power_supply_status = charger_status
                    self.batF.percentage = self.list_battery_obs[-1].percentage
                    #Publish
                    self.pub.publish(self.batF)

                    if self.verbose:
                        self.get_logger().info("Filterd battery--> V: " + str(self.batF.voltage) + " I: " + str(self.batF.current) + " State:" + str(self.batF.power_supply_status) )

                    # Check Low/Critical Battery (only if discharging)
                    if self.batF.power_supply_status == 2 or self.batF.power_supply_status == 3:
                        self.check_battery_levels()

                # Save to file
                if self.log_data and (time.time()-self.last_log_time) > self.log_interval:
                    self.last_log_time = self.get_clock().now()
                    with open(self.filename, 'a') as log:
                        # save all input data to file
                        for b in self.list_battery_obs_to_log:
                            #Time Voltage Current Chager_status
                            log.write(str(b.header.stamp) + ' %.3f %.3f %2s\n' % (b.voltage, b.current, b.power_supply_status))
                            #log.write(str(b.header.stamp) + " " + str(b.voltage) + " " + str(b.current) + " " + str(b.power_supply_status) + "\n")
                    #clean the log list
                    self.list_battery_obs_to_log = []
            

    # New battery obs.
    def battery_msg_cb(self, new_bat_state):
        # For filterint: Keep only the N most recent ones
        self.list_battery_obs.append(new_bat_state)
        if len(self.list_battery_obs) > self.filter_lenght:
            self.list_battery_obs.pop(0)
        # For log, keep all
        self.list_battery_obs_to_log.append(new_bat_state)


    # Check for Low/Critial battery levels
    def check_battery_levels(self):
        # Critial battery (timeOut = 300s)
        if float(self.batF.voltage)<float(self.critical_bat_th):
            if self.docking_request == False:
                # infor users
                self.last_crit_bat_time = rclpy.Time.now()
                self.get_logger().info("BATTERY IS CRITICAL (%.2f) --> COMMANDING HIGH PRIORITY DOCKING!! " %(self.batF.voltage))
                resp = self.add_task(task_name="say", task_type="say", task_priority=True, task_repetitions=1, task_impact="None", task_args=["Carefull!. Battery level is Critial. Cleaning Task Manager and Requesting Docking"])

                # Command Dock
                resp = self.add_task(task_name="recharge_till_full", task_priority=True, task_repetitions=1, task_impact="reset", task_args=[])
                if resp.success:
                    self.docking_request = True
                    self.dock_task_id = resp.task_id
                else:
                    self.get_logger().info("ERROR --> Unable to request DOCK action to TaskManager")
                

        # Low battery (timeOut = 60s)
        elif float(self.batF.voltage)<float(self.low_bat_th):
            # just inform the user
            self.get_logger().info("BATTERY IS GETTING LOW (%.2f < %.2f)!! " %(self.batF.voltage, self.low_bat_th))
            self.add_task(task_name="say", task_type="say", task_priority=True, task_repetitions=1, task_impact="None", task_args=["Your attention please!. Battery level is low. Consider recharging soon..."])


    # Task Eneded
    def task_ended_cb(self,msg):
        # This CB is executted each time a Task is completed in the TaskManager
        # We are only interested in the Docking Task that was originated by us!.
        # msg.key = "tasks_results"
        # msg.value = "\"task_id\":\"{}\", \"task_name\":\"{}\", \"task_status\":\"{}\", \"task_result\":\"{}\"".format(str(c.id.hex), str(c.name), str(c.feedback_message))
        try:
            # parse msg.value as a Python dictionary:
            d = json.loads(msg.value)

            # Check if this msg is our Docking Request result
            if "task_id" in d.keys():
                if d["task_id"] == str(self.dock_task_id):
                    # This is the result of our Dock request!
                    self.docking_request = False    # Allow new Docking requests in the future

                    # If we commanded a Dock Action and it failed, then we should announce it!
                    if d["task_status"] == "FAILURE":
                        self.add_task(task_type="say", task_name="say",task_priority=True, task_repetitions=1, task_impact="None", task_args=["Docking action failed for some reason. Trying again."])

            else:
                raise Exception("task_id not defined in msg")

        except Exception as excp:
            feedback_message = "Exception parsing ros2mqtt (json format invalid?): " + str(excp) + ". Skipping request."
            self.get_logger().info(feedback_message)


# =============================================================
# ==========================  MAIN  ===========================
# =============================================================
def main():
    rclpy.init()
    node = Battery()
    rclpy.spin(node)

if __name__ == '__main__':
    main()






