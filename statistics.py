import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt
import os

# Load and parse the XML file
xml_file = "flowmon-results.xml"  # Change this to your actual filename
tree = ET.parse(xml_file)
root = tree.getroot()

# Create output folder
output_dir = "temp"
os.makedirs(output_dir, exist_ok=True)

# Extract FlowStats
flows_data = []
for flow in root.findall(".//FlowStats/Flow"):
    flow_id = int(flow.get("flowId"))
    delay_sum = float(flow.get("delaySum").replace("+", "").replace("ns", ""))
    jitter_sum = float(flow.get("jitterSum").replace("+", "").replace("ns", ""))
    tx_packets = int(flow.get("txPackets"))
    rx_packets = int(flow.get("rxPackets"))
    tx_bytes = int(flow.get("txBytes"))
    rx_bytes = int(flow.get("rxBytes"))
    time_first_tx = float(flow.get("timeFirstTxPacket").replace("+", "").replace("ns", ""))
    time_last_rx = float(flow.get("timeLastRxPacket").replace("+", "").replace("ns", ""))
    
    duration_ns = time_last_rx - time_first_tx if time_last_rx > time_first_tx else 1
    duration_sec = duration_ns * 1e-9
    
    avg_delay_ms = (delay_sum / rx_packets) * 1e-6 if rx_packets > 0 else 0
    avg_jitter_ms = (jitter_sum / rx_packets) * 1e-6 if rx_packets > 0 else 0
    pdr = rx_packets / tx_packets if tx_packets > 0 else 0
    throughput_kbps = (rx_bytes * 8 / duration_sec) / 1000 if duration_sec > 0 else 0

    flows_data.append({
        "FlowId": flow_id,
        "AvgDelay (ms)": avg_delay_ms,
        "AvgJitter (ms)": avg_jitter_ms,
        "PDR": pdr,
        "Throughput (kbps)": throughput_kbps
    })

# Convert to DataFrame
df = pd.DataFrame(flows_data)
df.to_csv(os.path.join(output_dir, "flow_stats.csv"), index=False)

# Plotting
def save_bar_plot(metric, ylabel, filename):
    plt.figure(figsize=(8, 5))
    plt.bar(df["FlowId"], df[metric], color="skyblue", edgecolor="black")
    plt.xlabel("Flow ID")
    plt.ylabel(ylabel)
    plt.title(f"{ylabel} per Flow")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

save_bar_plot("AvgDelay (ms)", "Average Delay (ms)", "avg_delay.png")
save_bar_plot("AvgJitter (ms)", "Average Jitter (ms)", "avg_jitter.png")
save_bar_plot("PDR", "Packet Delivery Ratio", "pdr.png")
save_bar_plot("Throughput (kbps)", "Throughput (kbps)", "throughput.png")

print(f"All plots and data saved in: {output_dir}")
