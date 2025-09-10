uint8 Status            # 0xF1 or 0xF0 handshake/heartbeat
uint8 RegisterIndex
uint8 RegisterSelect
uint64 RegisterValues   # low 32 bits: Value0..3 (LSB..MSB)
uint8 Counter           # F210_Message_Identifier
