#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from threading import Thread

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
    dock_request_sent = False
    dock_task_id = -2           # set initially to unused value


    def __init__(self):
        super().__init__("battery_manager")

        # declare configuration params
        self.declare_parameter('input_battery_topic',"/battery")
        self.declare_parameter('output_battery_topic',"/battery_filtered")
        self.declare_parameter('filter_lenght_samples',30)
        self.declare_parameter('publish_rate',1.0)
        self.declare_parameter('low_battery_voltage',12.5)
        self.declare_parameter('critial_battery_voltage',12.0)
        self.declare_parameter('verbose',False)
        # read values
        in_battery = self.get_parameter('input_battery_topic').get_parameter_value().string_value
        out_battery = self.get_parameter('output_battery_topic').get_parameter_value().string_value
        self.filter_lenght = self.get_parameter('filter_lenght_samples').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.low_bat_th = self.get_parameter('low_battery_voltage').get_parameter_value().double_value
        self.critical_bat_th = self.get_parameter('critial_battery_voltage').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # logging
        self.declare_parameter('create_log',False)
        self.declare_parameter('log_directory_path',"~")
        self.declare_parameter('log_interval_sec',20)
        self.log_data = self.get_parameter('create_log').get_parameter_value().bool_value
        path = self.get_parameter('log_directory_path').get_parameter_value().string_value
        self.log_interval = self.get_parameter('log_interval_sec').get_parameter_value()._integer_value

        # Subscribers
        self.sub1 = self.create_subscription(BatteryState, in_battery, self.battery_msg_cb, 10)
        self.sub2 = self.create_subscription(diagnostic_msgs.msg.KeyValue, "/ros2mqtt", self.task_ended_cb, 10)

        # Publisher
        self.pub = self.create_publisher(BatteryState, out_battery, 100)

        # AddTask srv client (task_manager)
        self.srv_cli = self.create_client(AddTask, "task_manager/add_task")
        while not self.srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('task_manager/add_task service not available, waiting ...')
        self.srv_req = AddTask.Request()


        # Log to file?
        if self.log_data in ['true', 'True', '1', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']:
            self.log_data = True
            
            # Create a new log with the starting date
            now = datetime.datetime.now()
            date = now.strftime('%Y-%m-%d-%H-%M-%S')
            self.filename = path + "/battery_manager_log_" + date
            with open(self.filename, 'w') as log:
                log.write('#Battery Manager Log: Time Voltage Current Chager_status')

        # Init
        self.timeout = rclpy.time.Duration(seconds=60.0)        
        self.last_low_bat_time = self.get_clock().now() - self.timeout
        self.last_crit_bat_time = self.get_clock().now() - self.timeout
        self.timeout_log = rclpy.time.Duration(seconds=self.log_interval)
        self.last_log_time = self.get_clock().now() - self.timeout_log
        self.list_battery_obs = []
        self.list_battery_obs_to_log = []

        # Show config:
        if self.verbose:
            self.get_logger().info("Battery_Manager running with: ")
            self.get_logger().info("low_battery_voltage: %.2f V" % (self.low_bat_th))
            self.get_logger().info("critial_battery_voltage: %.2f V" % (self.critical_bat_th))
        
        # Create a timer Callback (not good! problems with service deadlocks)
        #if publish_rate == 0.0:
        #    publish_rate = 1
        #timer_period = 1/publish_rate  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)      
     

    def timer_callback(self):
        # Publish filtered battery
        if rclpy.ok(): 

            if len(self.list_battery_obs) == 0:
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
            if self.log_data and (self.get_clock().now() - self.last_log_time) > self.timeout_log:
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
        #self.get_logger().info("New battery msg ...")
        # For filtering: Keep only the N most recent ones
        self.list_battery_obs.append(new_bat_state)
        if len(self.list_battery_obs) > self.filter_lenght:
            self.list_battery_obs.pop(0)
        # For log, keep all
        self.list_battery_obs_to_log.append(new_bat_state)


    # Check for Low/Critial battery levels
    def check_battery_levels(self):
        # Critial battery (timeOut = 300s)
        if float(self.batF.voltage) < float(self.critical_bat_th):
            if (self.get_clock().now() - self.last_crit_bat_time > self.timeout) and (self.dock_request_sent == False):
                self.last_crit_bat_time = self.get_clock().now()
                # inform users
                self.get_logger().info("BATTERY IS CRITICAL (%.2f) --> COMMANDING HIGH PRIORITY DOCKING!! " %(self.batF.voltage))
                self.srv_req.task_name = "say_crit_bat"
                self.srv_req.task_type = "say"
                self.srv_req.task_priority = True
                self.srv_req.task_repetitions = 1
                self.srv_req.task_impact = "None"
                self.srv_req.task_args = ["Carefull!. Battery level is Critial. Cleaning Task Manager and Requesting Docking"]
                resp = self.srv_cli.call_async(self.srv_req)

                # Command Dock
                self.srv_req.task_name = "recharge_till_full"
                self.srv_req.task_type = "recharge_till_full"
                self.srv_req.task_priority = True
                self.srv_req.task_repetitions = 1
                self.srv_req.task_impact = "reset"
                self.srv_req.task_args = []
                
                # we call synchronously because spin is in a differet thread
                resp = self.srv_cli.call(self.srv_req)
                self.get_logger().info("SRV call done!")
                
                if resp.success == True:
                    # Docking was accepted
                    self.dock_request_sent = True
                    self.dock_task_id = resp.task_id
                else:
                    self.get_logger().info("ERROR --> Unable to request DOCK action to TaskManager: %s" % resp.error_msg)
                
        # Low battery (timeOut = 60s)
        elif (float(self.batF.voltage) < float(self.low_bat_th)) and (self.get_clock().now() - self.last_low_bat_time > self.timeout):
            self.last_low_bat_time = self.get_clock().now()             
            # inform the user (HRI)
            self.srv_req.task_name = "say_low_bat"
            self.srv_req.task_type = "say"
            self.srv_req.task_priority = True
            self.srv_req.task_repetitions = 1
            self.srv_req.task_impact = "None"
            self.srv_req.task_args = ["Your attention please!. Battery level is low. Consider recharging soon..."]
            resp = self.srv_cli.call_async(self.srv_req)
            # log
            self.get_logger().info("BATTERY IS GETTING LOW (%.2f <= %.2f)!! " %(self.batF.voltage, self.low_bat_th))


    # Task Eneded
    def task_ended_cb(self,msg):
        # This CB is executted each time a Task is completed in the TaskManager
        # We are only interested in the Docking Task that was originated by us!.
        # msg.key = "tasks_results"
        # msg.value = "\"task_id\":\"{}\", \"task_name\":\"{}\", \"task_status\":\"{}\", \"task_result\":\"{}\"".format(str(c.id.hex), str(c.name), str(c.feedback_message))        
        if msg.key == "task_results":
            try:            
                if self.verbose:
                    self.get_logger().info("Task ended with value: %s" % msg.value)

                # parse msg.value as a Python dictionary:
                d = json.loads(msg.value)

                # Check if this msg is our Docking Request result
                if "task_id" in d.keys():
                    if d["task_id"] == str(self.dock_task_id):
                        # This is the result of our Dock request!
                        self.dock_request_sent = False    # Allow new Docking requests in the future

                        # If we commanded a Dock Action and it failed, then we should announce it!
                        if d["task_status"] == "FAILURE":
                            self.srv_req.task_name = "say_dock_failed"
                            self.srv_req.task_type = "say"
                            self.srv_req.task_priority = True
                            self.srv_req.task_repetitions = 1
                            self.srv_req.task_impact = "None"
                            self.srv_req.task_args = ["Docking action failed for some reason. Trying again."]
                            resp = self.srv_cli.call_async(self.srv_req)
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

    # call rclpy.spin in a separate Thread to avoid deadlocks
    spin_thread = Thread(target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()

    # Check battery lvls according to specified rate
    rate = node.create_rate(node.publish_rate)
    try:
        while rclpy.ok():
            node.timer_callback()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    spin_thread.join()
        

if __name__ == '__main__':
    main()






