#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_io_node
- Pure CAN I/O (no logic).
- Subscribes to /CxD/Status, /CxD/Command, /CxD/VehicleSpeed messages + /tx triggers.
- Encodes and sends CAN frames.
- Decodes incoming CAN frames into FACB, FACC, Time ROS messages.
- Auto-responds: when DigiStop_TimeRequest is received, sends CxD_TimeResponse.
- Manual trigger: /CxD/TimeRequest_bool (Bool) will send a CxD_TimeRequest using internal system time.
"""
import os
import rospy
import cantools
from datetime import datetime
from can_msgs.msg import Frame
from std_msgs.msg import Bool
from canbus_coder.msg import FACB, FACC, Time, CxDStatus, CxDCommand, CxDVehicleSpeed


class CanIONode:
    def __init__(self):
        # --- CAN setup ---
        self.can_device = rospy.get_param("~can_device", "can0")
        self.recv_topic = f"/{self.can_device}/recv"
        self.send_topic = f"/{self.can_device}/send"

        # --- Load DBC ---
        dbc_folder = rospy.get_param("~dbc_folder", "")
        dbc_file   = rospy.get_param("~dbc_file", "LoopX-DigiStop-J1939-2025-09_v1.0.dbc")
        dbc_path = os.path.join(dbc_folder, dbc_file)
        self.db = cantools.database.load_file(dbc_path)

        # --- Message handles ---
        self.msg_CxD_Status       = self.db.get_message_by_name("CxD_Status")
        self.msg_CxD_Command      = self.db.get_message_by_name("CxD_Command")
        self.msg_CxD_VehicleSpeed = self.db.get_message_by_name("CxD_VehicleSpeed")
        self.msg_CxD_TimeResp     = self.db.get_message_by_name("CxD_TimeResponse")
        self.msg_CxD_TimeReq      = self.db.get_message_by_name("CxD_TimeRequest")

        # --- Raw CAN pub/sub ---
        self.pub_can = rospy.Publisher(self.send_topic, Frame, queue_size=50)
        rospy.Subscriber(self.recv_topic, Frame, self.on_can_rx, queue_size=200)

        # --- Publishers for decoded messages ---
        self.pub_facb = rospy.Publisher("/FACB", FACB, queue_size=10)
        self.pub_facc = rospy.Publisher("/FACC", FACC, queue_size=10)
        self.pub_time_digistop_req  = rospy.Publisher("/DigiStop/TimeRequest",  Time, queue_size=10)
        self.pub_time_digistop_resp = rospy.Publisher("/DigiStop/TimeResponse", Time, queue_size=10)
        self.pub_time_cxd_req       = rospy.Publisher("/CxD/TimeRequest",       Time, queue_size=10)
        self.pub_time_cxd_resp      = rospy.Publisher("/CxD/TimeResponse",      Time, queue_size=10)

        # --- Subscriptions for TX ---
        rospy.Subscriber("/CxD/Status",          CxDStatus,       self.cb_set_status)
        rospy.Subscriber("/CxD/Status/tx",       Bool,            self.cb_tx_status)
        rospy.Subscriber("/CxD/Command",         CxDCommand,      self.cb_set_command)
        rospy.Subscriber("/CxD/Command/tx",      Bool,            self.cb_tx_command)
        rospy.Subscriber("/CxD/VehicleSpeed",    CxDVehicleSpeed, self.cb_set_speed)
        rospy.Subscriber("/CxD/VehicleSpeed/tx", Bool,            self.cb_tx_speed)
        rospy.Subscriber("/CxD/TimeRequest_bool", Bool,           self.cb_tx_time_request)

        # --- Latched messages ---
        self.last_status  = CxDStatus()
        self.last_command = CxDCommand()
        self.last_speed   = CxDVehicleSpeed()

        rospy.loginfo("[can_io_node] Ready")

    # ---------------- RX decode ----------------
    def on_can_rx(self, frame: Frame):
        try:
            msg = self.db.get_message_by_frame_id(frame.id)
            decoded = msg.decode(bytes(frame.data))
        except Exception:
            return

        try:
            if msg.name == "Machine_Data":
                facb = FACB()
                facb.SubsystemHeader   = int(decoded.get("FACB_SubsystemHeader", 0))
                facb.SubsystemData     = int(decoded.get("FACB_SubsystemData", 0))
                facb.UserConfig        = int(decoded.get("FACB_UserConfig", 0))
                facb.MessageIdentifier = int(decoded.get("FACB_MessageIdentifier", 0))
                self.pub_facb.publish(facb)

            elif msg.name == "Machine_Reply":
                v0 = int(decoded.get("FACC_Register_Value0", 0)) & 0xFF
                v1 = int(decoded.get("FACC_Register_Value1", 0)) & 0xFF
                v2 = int(decoded.get("FACC_Register_Value2", 0)) & 0xFF
                v3 = int(decoded.get("FACC_Register_Value3", 0)) & 0xFF
                reg32 = (v0 | (v1 << 8) | (v2 << 16) | (v3 << 24))
                facc = FACC()
                facc.RegisterValue     = reg32
                facc.RegisterFormat    = int(decoded.get("FACC_Register_Format", 0))
                facc.RegisterIndex     = int(decoded.get("FACC_Register_Index", 0))
                facc.Status            = int(decoded.get("FACC_Status", 0))
                facc.MessageIdentifier = int(decoded.get("FACC_Message_Identifier", 0))
                self.pub_facc.publish(facc)

            elif msg.name in ("DigiStop_TimeResponse", "DigiStop_TimeRequest",
                              "CxD_TimeResponse", "CxD_TimeRequest"):
                tmsg = Time()
                tmsg.Local_Hour_offset   = int(decoded.get("Local_Hour_offset", 0))
                tmsg.Local_Minute_offset = int(decoded.get("Local_Minute_offset", 0))
                tmsg.Year    = int(decoded.get("Year", 0))
                tmsg.Day     = float(decoded.get("Day", 0))
                tmsg.Month   = int(decoded.get("Month", 0))
                tmsg.Hours   = int(decoded.get("Hours", 0))
                tmsg.Minutes = int(decoded.get("Minutes", 0))
                tmsg.Seconds = float(decoded.get("Seconds", 0))

                if msg.name == "DigiStop_TimeRequest":
                    self.pub_time_digistop_req.publish(tmsg)
                    # auto-respond with CxD_TimeResponse
                    signals = self._now_signals()
                    self._send_frame(self.msg_CxD_TimeResp, signals)
                    rospy.loginfo(
                        f"[Auto-respond DigiStop_TimeRequest] "
                        f"time={signals['Year']:04d}-{signals['Month']:02d}-{int(signals['Day']):02d} "
                        f"{signals['Hours']:02d}:{signals['Minutes']:02d}:{int(signals['Seconds']):02d}"
                    )
                elif msg.name == "DigiStop_TimeResponse":
                    self.pub_time_digistop_resp.publish(tmsg)
                elif msg.name == "CxD_TimeRequest":
                    self.pub_time_cxd_req.publish(tmsg)
                elif msg.name == "CxD_TimeResponse":
                    self.pub_time_cxd_resp.publish(tmsg)

        except Exception as e:
            rospy.logwarn(f"[can_io_node] Decode failed {msg.name}: {e}")

    # ---------------- TX helpers ----------------
    def cb_set_status(self, m: CxDStatus): self.last_status = m
    def cb_set_command(self, m: CxDCommand): self.last_command = m
    def cb_set_speed(self, m: CxDVehicleSpeed): self.last_speed = m

    def _send_frame(self, message, signals: dict):
        try:
            raw = message.encode(signals)
            frame = Frame()
            frame.id = message.frame_id
            frame.is_extended = True
            frame.is_error = False
            frame.is_rtr = False
            frame.dlc = 8
            data_list = list(raw)
            if len(data_list) < 8:
                data_list.extend([0] * (8 - len(data_list)))
            frame.data = data_list[:8]
            self.pub_can.publish(frame)
        except Exception as e:
            rospy.logwarn(f"[can_io_node] Failed to send {message.name}: {e}")

    # ---------------- TX callbacks ----------------
    def cb_tx_status(self, m: Bool):
        if not m.data: return
        s = self.last_status
        signals = {
            "F210_Status":             int(s.Status) & 0xFF,
            "F210_Register_Index":     int(s.RegisterIndex) & 0xFF,
            "F210_Register_Select":    int(s.RegisterSelect) & 0xFF,
            "F210_Register_Value0":    (int(s.RegisterValues) >> 0) & 0xFF,
            "F210_Register_Value1":    (int(s.RegisterValues) >> 8) & 0xFF,
            "F210_Register_Value2":    (int(s.RegisterValues) >> 16) & 0xFF,
            "F210_Register_Value3":    (int(s.RegisterValues) >> 24) & 0xFF,
            "F210_Message_Identifier": int(s.Counter) & 0xFF,
        }
        self._send_frame(self.msg_CxD_Status, signals)

    def cb_tx_command(self, m: Bool):
        if not m.data: return
        c = self.last_command
        signals = {
            "F211_Command":            int(c.Command) & 0xFF,
            "F211_Register_Index":     int(c.RegisterIndex) & 0xFF,
            "F211_Register_Select":    int(c.RegisterSelect) & 0xFF,
            "F211_Register_Value0":    (int(c.RegisterValues) >> 0) & 0xFF,
            "F211_Register_Value1":    (int(c.RegisterValues) >> 8) & 0xFF,
            "F211_Register_Value2":    (int(c.RegisterValues) >> 16) & 0xFF,
            "F211_Register_Value3":    (int(c.RegisterValues) >> 24) & 0xFF,
            "F211_Message_Identifier": int(c.Counter) & 0xFF,
        }
        self._send_frame(self.msg_CxD_Command, signals)

    def cb_tx_speed(self, m: Bool):
        if not m.data: return
        v = self.last_speed
        signals = {"F212_VehicelSpeed": float(v.Value)}
        self._send_frame(self.msg_CxD_VehicleSpeed, signals)

    def cb_tx_time_request(self, m: Bool):
        if not m.data: return
        signals = self._now_signals()
        self._send_frame(self.msg_CxD_TimeReq, signals)
        rospy.loginfo(
            f"[Manual TimeRequest] Sent "
            f"time={signals['Year']:04d}-{signals['Month']:02d}-{int(signals['Day']):02d} "
            f"{signals['Hours']:02d}:{signals['Minutes']:02d}:{int(signals['Seconds']):02d}"
        )

    def _now_signals(self):
        now = datetime.now()
        return {
            "Local_Hour_offset":   125,  # UTC offset
            "Local_Minute_offset": 125,
            "Year": now.year,
            "Day": float(now.day),
            "Month": now.month,
            "Hours": now.hour,
            "Minutes": now.minute,
            "Seconds": float(now.second),
        }


def main():
    rospy.init_node("can_io_node", anonymous=True)
    CanIONode()
    rospy.spin()


if __name__ == "__main__":
    main()
