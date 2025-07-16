import ns3ai
import numpy as np

@ns3ai.register()
def ai_main():
    state_shape = (5, 3)  # numStas x numLinks
    action_shape = (5,)   # one selected link per STA

    while True:
        state = ns3ai.recv_tensor("state", shape=state_shape)
        actions = np.argmax(state, axis=1)  # dummy: choose best RSSI link
        ns3ai.send_tensor("action", actions)
