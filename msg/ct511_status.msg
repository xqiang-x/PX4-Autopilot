uint64 timestamp                # time since system start
uint8 state                     # 0=starting 1=probing 2=netopen 3=connect 4=transparent 5=error
bool link_up                    # TCP transparent link to relay established
int32 csq                       # signal strength 0-31, -1 unknown
uint32 tx_bytes                 # MAVLink bytes uplinked via 4G (board -> server)
uint32 rx_bytes                 # MAVLink bytes downlinked via 4G (server -> board)
uint32 reconnects               # number of TCP link (re)establish cycles
