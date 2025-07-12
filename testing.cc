#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <ns3/ai-module.h>

// testing ns3 features
// Node (STA / AP)
//  └── WifiNetDevice
//       ├── MloManager
//       │    ├── MloLink[0] ──> WifiMac + WifiPhy
//       │    ├── MloLink[1] ──> WifiMac + WifiPhy
//       │    └── MloScheduler (AP only)
//       └── Internet stack (TCP/IP)

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TestingScriptExample");

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
    std::vector<YansWifiChannelHelper> channelHelpers(numLinks);
    std::vector<YansWifiPhyHelper> phyHelpers(numLinks);
    std::vector<WifiMacHelper> macHelpers(numLinks);
    std::vector<NetDeviceContainer> apDevices(numLinks);
    std::vector<NetDeviceContainer> staDevices(numLinks);

    // Create Wifi helper and set standard to 802.11be (WiFi 8)
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211be);

    // Create 3 links (frequencies)
    for (uint32_t i = 0; i < 3; ++i)
    {
        channelHelpers[i] = YansWifiChannelHelper::Default();
        phyHelpers[i] = YansWifiPhyHelper();  //::Default()
        // phyHelpers[i].SetChannel(channelHelpers[i].Create());
        phyHelpers[i].SetChannel(channelHelpers[i].Create());

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211be);

        macHelpers[i].SetType("ns3::ApWifiMac",
                            "Ssid", SsidValue(Ssid("link" + std::to_string(i))));

        apDevices[i] = wifi.Install(phyHelpers[i], macHelpers[i], apNode);

        // Assign 1/3 of STAs to this link
        NodeContainer staSlice;
        for (uint32_t j = i; j < numStas; j += 3)
            staSlice.Add(staNodes.Get(j));

        WifiMacHelper staMac;
        staMac.SetType("ns3::StaWifiMac",
                    "Ssid", SsidValue(Ssid("link" + std::to_string(i))),
                    "ActiveProbing", BooleanValue(false));

        staDevices[i] = wifi.Install(phyHelpers[i], staMac, staSlice);
    }

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    std::vector<Ipv4InterfaceContainer> apInterfaces(numLinks);
    std::vector<Ipv4InterfaceContainer> staInterfaces(numLinks);

    for (uint32_t i = 0; i < numLinks; ++i)
    {
        std::string subnet = "10.1." + std::to_string(i+1) + ".0";
        ipv4.SetBase(subnet.c_str(), "255.255.255.0");
        apInterfaces[i] = ipv4.Assign(apDevices[i]);
        staInterfaces[i] = ipv4.Assign(staDevices[i]);
    }

    // Correct: Install just one UDP server on AP node listening on port 9
    uint16_t port = 9;
    UdpServerHelper udpServer(port);
    ApplicationContainer serverApps = udpServer.Install(apNode.Get(0)); // AP node
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));


    // UDP Client on each STA to send uplink traffic to AP
    for (uint32_t i = 0; i < numLinks; ++i)
    {
        for (uint32_t j = 0; j < staDevices[i].GetN(); ++j)
        {
            UdpClientHelper udpClient(apInterfaces[i].GetAddress(0), port);
            udpClient.SetAttribute("MaxPackets", UintegerValue(100));
            udpClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
            udpClient.SetAttribute("PacketSize", UintegerValue(1024));

            ApplicationContainer clientApps = udpClient.Install(staDevices[i].Get(j)->GetNode());
            clientApps.Start(Seconds(2.0 + j));
            clientApps.Stop(Seconds(20.0));
        }
    }



    // UdpEchoServerHelper echoServer(9);
    // ApplicationContainer serverApps = echoServer.Install(apNode.Get(0));
    // serverApps.Start(Seconds(1.0));
    // serverApps.Stop(Seconds(20.0)); 

    // UDP Echo Client on each STA to AP (simulate traffic)
    // for (uint32_t i = 0; i < numLinks; ++i)
    // {
    //     for (uint32_t j = 0; j < staDevices[i].GetN(); ++j)
    //     {
    //         UdpEchoClientHelper echoClient(apInterfaces[i].GetAddress(0), 9);
    //         echoClient.SetAttribute("MaxPackets", UintegerValue(100));
    //         echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    //         echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    //         ApplicationContainer clientApps = echoClient.Install(staDevices[i].Get(j)->GetNode());
    //         clientApps.Start(Seconds(2.0 + j));
    //         clientApps.Stop(Seconds(20.0));
    //     }
    // } ##### THIS WOULD CREATE DOWNLINK TRAFFIC #####

    // Flow monitor to track traffic
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;

    flowMonitor = flowHelper.InstallAll();

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