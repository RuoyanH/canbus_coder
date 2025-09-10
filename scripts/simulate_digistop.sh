#!/bin/bash
# 模拟 DigiStop 节点报文
# 使用 vcan0 发送握手和时间请求报文

DEV="vcan0"

echo "[DigiStop] Step 1: Send 0xF1 (negotiation response)"
cansend $DEV 18FACBF2#F100000000000001   #Machina CxD Data (PGN FACB)
sleep 1

echo "[DigiStop] Step 2: Send 0xF0 (negotiation complete)"
cansend $DEV 18FACBF2#F000000000000002
sleep 2

echo "[DigiStop] Step 3: Send TimeRequest"
cansend $DEV 00EA2AF2#0000000000000000   #Machina CxD Time Request

cansend $DEV 18FACCF2#FF00000000000001   #Machina CxD Reply (PGN FACC)
