#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
from canbus_coder.msg import CxDCommand, CxDVehicleSpeed

def publish_command(pub):
    cmd = CxDCommand()
    cmd.Command = 0x55
    cmd.RegisterIndex = 0xC
    cmd.RegisterSelect = 6
    cmd.RegisterValues = 0x11223344
    rospy.loginfo("[TEST] Publishing Command_ext")
    pub.publish(cmd)

def publish_speed(pub):
    speed = CxDVehicleSpeed()
    speed.Value = 36.5
    rospy.loginfo("[TEST] Publishing VehicleSpeed_ext")
    pub.publish(speed)

def trigger_time_request(pub):
    rospy.loginfo("[TEST] Triggering TimeRequest_ext")
    pub.publish(Bool(True))

def on_state(msg: String):
    rospy.loginfo(f"[TEST] Handshake state: {msg.data}")

def main():
    rospy.init_node("test_can_nodes", anonymous=True)

    pub_cmd   = rospy.Publisher("/CxD/Command_ext", CxDCommand, queue_size=1, latch=True)
    pub_speed = rospy.Publisher("/CxD/VehicleSpeed_ext", CxDVehicleSpeed, queue_size=1, latch=True)
    pub_time  = rospy.Publisher("/CxD/TimeRequest_ext", Bool, queue_size=1, latch=True)

    rospy.Subscriber("/handshake/state", String, on_state)

    rospy.sleep(2.0)   # 等待节点初始化
    publish_command(pub_cmd)
    publish_speed(pub_speed)
    rospy.sleep(1.0)
    trigger_time_request(pub_time)

    rospy.loginfo("[TEST] External interface test running...")
    rospy.spin()

if __name__ == "__main__":
    main()
