#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_handler_node
- Strict handshake FSM (auto-start):
    * Start: NEGOTIATE_F1 -> keep sending Status.Status=0xF1 every 10 ms
    * When peer sends 0xF1: switch to Status.Status=0xF0 every 10 ms
    * When peer sends 0xF0: become ACTIVE
- In ACTIVE:
    * Publish Command + /tx every 100 ms
    * Publish VehicleSpeed + /tx every 100 ms
- TimeRequest is external; TimeResponse is auto by can_io_node.
"""
import rospy
from std_msgs.msg import String, Bool
from canbus_coder.msg import FACB, CxDStatus, CxDCommand, CxDVehicleSpeed


class CanHandlerNode:
    NEGOTIATE_F1 = "NEGOTIATE_F1"
    NEGOTIATE_F0 = "NEGOTIATE_F0"
    ACTIVE       = "ACTIVE"

    def __init__(self):
        # Timing params
        self.ms_status  = int(rospy.get_param("~status_period_ms", 10))
        self.ms_command = int(rospy.get_param("~command_period_ms", 100))
        self.ms_speed   = int(rospy.get_param("~speed_period_ms",   100))

        # --- Publishers to can_io_node ---
        self.p_status     = rospy.Publisher("/CxD/Status",       CxDStatus,       queue_size=1)
        self.p_status_tx  = rospy.Publisher("/CxD/Status/tx",    Bool,            queue_size=1)
        self.p_command    = rospy.Publisher("/CxD/Command",      CxDCommand,      queue_size=1)
        self.p_command_tx = rospy.Publisher("/CxD/Command/tx",   Bool,            queue_size=1)
        self.p_speed      = rospy.Publisher("/CxD/VehicleSpeed", CxDVehicleSpeed, queue_size=1)
        self.p_speed_tx   = rospy.Publisher("/CxD/VehicleSpeed/tx", Bool,         queue_size=1)
        self.p_time_req   = rospy.Publisher("/CxD/TimeRequest_bool", Bool,        queue_size=1)

        # Debug publishers
        self.p_state  = rospy.Publisher("/handshake/state",  String, queue_size=1, latch=True)
        self.p_active = rospy.Publisher("/handshake/active", Bool,   queue_size=1, latch=True)

        # --- Subscriptions (external input) ---
        rospy.Subscriber("/FACB", FACB, self.on_facb)
        rospy.Subscriber("/CxD/Command_ext",      CxDCommand,      self.cb_ext_command)
        rospy.Subscriber("/CxD/VehicleSpeed_ext", CxDVehicleSpeed, self.cb_ext_speed)
        rospy.Subscriber("/CxD/TimeRequest_ext",  Bool,            self.cb_ext_time_request)

        # --- FSM state ---
        self.state = self.NEGOTIATE_F1
        self.status_counter = 0
        self.cmd_counter    = 0

        # --- Latched values ---
        self.cxds = CxDStatus(Status=0xF1, RegisterIndex=0, RegisterSelect=0,
                              RegisterValues=0, Counter=0)
        self.cmd  = CxDCommand(Command=0, RegisterIndex=0, RegisterSelect=0,
                               RegisterValues=0, Counter=0)
        self.speed = CxDVehicleSpeed(Value=0.0)

        # --- Timers ---
        rospy.Timer(rospy.Duration(self.ms_status/1000.0), self.cb_status_tick)
        rospy.Timer(rospy.Duration(self.ms_command/1000.0), self.cb_command_tick)
        rospy.Timer(rospy.Duration(self.ms_speed/1000.0),   self.cb_speed_tick)

        # Initial state
        self.p_state.publish(String(self.state))
        self.p_active.publish(Bool(False))
        rospy.loginfo("[can_handler_node] started (auto-handshake F1->F0->ACTIVE)")

    # ---------------- State helpers ----------------
    def set_state(self, st: str):
        if st != self.state:
            self.state = st
            self.p_state.publish(String(st))
            self.p_active.publish(Bool(st == self.ACTIVE))
            rospy.loginfo(f"[can_handler_node] state -> {st}")

    # ---------------- Handshake detection ----------------
    def on_facb(self, msg: FACB):
        b0 = int(msg.SubsystemHeader) & 0xFF
        if b0 == 0xF3:
            rospy.logwarn("[can_handler_node] peer timeout (0xF3) -> restart")
            self.set_state(self.NEGOTIATE_F1)
            return
        if self.state == self.NEGOTIATE_F1 and b0 == 0xF1:
            rospy.loginfo("[can_handler_node] got peer 0xF1 -> switch to 0xF0")
            self.set_state(self.NEGOTIATE_F0)
        elif self.state == self.NEGOTIATE_F0 and b0 == 0xF0:
            rospy.loginfo("[can_handler_node] got peer 0xF0 -> ACTIVE")
            self.set_state(self.ACTIVE)

    # ---------------- External input ----------------
    def cb_ext_command(self, msg: CxDCommand):
        self.cmd.Command        = msg.Command
        self.cmd.RegisterIndex  = msg.RegisterIndex
        self.cmd.RegisterSelect = msg.RegisterSelect
        self.cmd.RegisterValues = msg.RegisterValues

    def cb_ext_speed(self, msg: CxDVehicleSpeed):
        self.speed.Value = msg.Value

    def cb_ext_time_request(self, msg: Bool):
        if msg.data:
            rospy.loginfo("[can_handler_node] external TimeRequest trigger")
            self.p_time_req.publish(Bool(True))

    # ---------------- Periodic ticks ----------------
    def cb_status_tick(self, _):
        self.status_counter = (self.status_counter + 1) & 0xFF
        self.cxds.Counter = self.status_counter
        self.cxds.Status  = 0xF1 if self.state == self.NEGOTIATE_F1 else 0xF0
        self.p_status.publish(self.cxds)
        self.p_status_tx.publish(Bool(True))

    def cb_command_tick(self, _):
        if self.state != self.ACTIVE:
            return
        self.cmd_counter = (self.cmd_counter + 1) & 0xFF
        self.cmd.Counter = self.cmd_counter
        self.p_command.publish(self.cmd)
        self.p_command_tx.publish(Bool(True))

    def cb_speed_tick(self, _):
        if self.state != self.ACTIVE:
            return
        self.p_speed.publish(self.speed)
        self.p_speed_tx.publish(Bool(True))


def main():
    rospy.init_node("can_handler_node", anonymous=True)
    CanHandlerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
