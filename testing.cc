#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ai-module.h"
#include "ns3/ai-socket-bridge.h"
#include "ns3/tensor.h"

// testing ns3 features
// Node (STA / AP)
//  └── WifiNetDevice
//       ├── MloManager
//       │    ├── MloLink[0] ──> WifiMac + WifiPhy
//       │    ├── MloLink[1] ──> WifiMac + WifiPhy
//       │    └── MloScheduler (AP only)
//       └── Internet stack (TCP/IP)

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DynamicMloSimulation");


void StepEnvironment(Ptr<AiSocketBridge> bridge,
                    std::vector<std::vector<double>>& linkStats,
                    std::vector<std::vector<Ptr<WifiNetDevice>>> staDevices,
                    Time interval)
{
    // Send current state (e.g., RSSI, load, queue, etc.)
    Tensor state = bridge->GetInput("state");
    for (uint32_t i = 0; i < linkStats.size(); ++i)
        for (uint32_t j = 0; j < linkStats[i].size(); ++j)
            state.SetValue(i, j, linkStats[i][j]);

    bridge->Send();

    // Wait until Python replies
    bridge->Recv();

    // Get action from Python: e.g., best link for each STA

    Tensor action = bridge->GetOutput("action");
    if (!action.Defined() || action.Shape().size() != 1 || action.Shape()[0] != linkStats.size()) {
        NS_LOG_WARN("Invalid action tensor received");
        Simulator::Schedule(interval, &StepEnvironment, bridge, linkStats, staDevices, interval);
        return;
    }

    for (uint32_t i = 0; i < staDevices.size(); ++i)
    {
        uint32_t selectedLink = std::min((uint32_t)action.GetValue(i), (uint32_t)staDevices[i].size() - 1);
        for (uint32_t l = 0; l < staDevices[i].size(); ++l)
        {
            Ptr<WifiNetDevice> dev = staDevices[i][l];
            Ptr<WifiMac> mac = dev->GetMac();
            mac->SetAttribute("ReceiveEnabled", BooleanValue(l == selectedLink));
        }
        NS_LOG_INFO("STA " << i << " switched to link " << selectedLink);
    }

    // Re-schedule
    Simulator::Schedule(interval, &StepEnvironment, bridge, linkStats, staDevices, interval);
}

int main(int argc, char* argv[])
{
    uint32_t numStas = 5;
    uint32_t numLinks = 3;
    CommandLine cmd;

    cmd.AddValue ("numStas", "Number of station nodes", numStas);
    cmd.Parse (argc, argv);

    Time::SetResolution(Time::NS); // smallest time unit
    LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);


    // Relevant classes
    NodeContainer apNode;
    apNode.Create (1);

    NodeContainer staNodes;
    staNodes.Create (numStas);

    // Install internet stack on all nodes
    InternetStackHelper internet;
    internet.Install(apNode);
    internet.Install(staNodes);

    // Mobility model - static for simplicity
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(5.0),
                                  "DeltaY", DoubleValue(5.0),
                                  "GridWidth", UintegerValue(3),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNode);
    mobility.Install(staNodes);

    // YANS wifi simulation: Combines large-scale path loss 
    // with small-scale fading effects, giving a more realistic 
    // representation of wireless channel behavior.
    // DEFINES THE SIMULATION (WIRELESS) MEDIUM
    std::vector<YansWifiChannelHelper> channels(numLinks);
    std::vector<YansWifiPhyHelper> phys(numLinks);
    std::vector<WifiMacHelper> apMacs(numLinks);
    std::vector<WifiMacHelper> staMacs(numLinks);
    std::vector<NetDeviceContainer> apDevices(numLinks);
    std::vector<std::vector<Ptr<WifiNetDevice>>> staDevices(numStas);

    // Create Wifi helper and set standard to 802.11be (WiFi 8)
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211be);

    // Create 3 links (frequencies)
    for (uint32_t l = 0; l < numLinks; ++l)
    {
        channels[l] = YansWifiChannelHelper::Default();
        phys[l].SetChannel(channels[l].Create());

        Ssid ssid = Ssid("link" + std::to_string(l));
        apMacs[l].SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        apDevices[l] = wifi.Install(phys[l], apMacs[l], apNode);

        staMacs[l].SetType("ns3::StaWifiMac",
                          "Ssid", SsidValue(ssid),
                          "ActiveProbing", BooleanValue(false));

        for (uint32_t s = 0; s < numStas; ++s)
        {
            NetDeviceContainer staDev = wifi.Install(phys[l], staMacs[l], staNodes.Get(s));
            Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(staDev.Get(0));
            staDevices[s].push_back(wifiDev);
        }
    }

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    for (uint32_t l = 0; l < numLinks; ++l)
    {
        ipv4.SetBase(("10.1." + std::to_string(l + 1) + ".0").c_str(), "255.255.255.0");
        ipv4.Assign(apDevices[l]);
        for (uint32_t s = 0; s < numStas; ++s)
        {
            ipv4.Assign(NetDeviceContainer(staDevices[s][l]));
        }
    }

    // Correct: Install just one UDP server on AP node listening on port 9
    uint16_t port = 9;
    UdpServerHelper udpServer(port);
    ApplicationContainer serverApps = udpServer.Install(apNode.Get(0)); // AP node
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));


    // UDP Client on each STA to send uplink traffic to AP
    for (uint32_t s = 0; s < numStas; ++s)
    {
        UdpClientHelper udpClient(Ipv4Address("10.1.1.1"), port);
        udpClient.SetAttribute("MaxPackets", UintegerValue(100));
        udpClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
        udpClient.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer clientApps = udpClient.Install(staNodes.Get(s));
        clientApps.Start(Seconds(2.0 + s));
        clientApps.Stop(Seconds(20.0));
    }


    std::vector<std::vector<double>> linkStats(numStas, std::vector<double>(numLinks, 1.0));

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

    AiModule::Init();
   // Define the input and output tensors between NS-3 and Python
    Ptr<AiSocketBridge> bridge = CreateObject<AiSocketBridge>();
    bridge->SetInputShape("state", {numStas, numLinks});
    bridge->SetOutputShape("action", {numStas});

    // Bind to localhost (or remote if needed)
    if (!bridge->Connect("127.0.0.1", 50051)) {
        NS_FATAL_ERROR("Failed to connect to AI bridge.");
    }
    AiModule::RegisterBridge(bridge);

    Time stepInterval = Seconds(0.5);
    Simulator::Schedule(Seconds(1.0), &StepEnvironment, bridge, linkStats, staDevices, stepInterval);


    // General Stats
    std::cout << "Number of STA nodes: " << numStas << std::endl;
    std::cout << "Number of links: " << numLinks << std::endl;
    std::cout << "Simulation running with uplink UDP traffic from STAs to AP on port 9." << std::endl;

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();
    Simulator::Destroy();
    // Flow monitor results
    flowMonitor->SerializeToXmlFile("flowmon-results.xml", true, true);
    Simulator::Destroy();

    return 0;

}