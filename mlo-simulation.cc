#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3-ai-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MloSimulation");

int main(int argc, char *argv[]) {
    uint32_t M = 3; // number of STAs
    uint32_t L = 2; // number of links at AP

    CommandLine cmd;
    cmd.AddValue("M", "Number of STA nodes", M);
    cmd.AddValue("L", "Number of AP links (radios)", L);
    cmd.Parse(argc, argv);

    // Set up AI Environment
    AiInterface::Init(0, "mlo_round_robin.py");  // Python AI script
    AiInterface::Start();

    // Create nodes
    NodeContainer apNode;
    apNode.Create(1);
    NodeContainer staNodes;
    staNodes.Create(M);

    // Each link = a separate NetDevice and PHY
    std::vector<NetDeviceContainer> apDevices;
    std::vector<NetDeviceContainer> staDevices;

    for (uint32_t i = 0; i < L; ++i) {
        YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
        YansWifiPhyHelper phy;
        phy.SetChannel(channel.Create());

        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211ax);
        WifiMacHelper mac;

        Ssid ssid = Ssid("mlo-link-" + std::to_string(i));

        // STAs
        mac.SetType("ns3::StaWifiMac",
                    "Ssid", SsidValue(ssid),
                    "ActiveProbing", BooleanValue(false));
        NetDeviceContainer staDev = wifi.Install(phy, mac, staNodes);
        staDevices.push_back(staDev);

        // AP
        mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer apDev = wifi.Install(phy, mac, apNode);
        apDevices.push_back(apDev);
    }

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNode);
    mobility.Install(staNodes);

    // Internet stack
    InternetStackHelper stack;
    stack.Install(apNode);
    stack.Install(staNodes);

    // IP address assignment
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces, apInterfaces;

    for (uint32_t i = 0; i < L; ++i) {
        NetDeviceContainer all = apDevices[i];
        all.Add(staDevices[i]);
        Ipv4InterfaceContainer interfaces = address.Assign(all);
        apInterfaces.Add(interfaces.Get(0)); // AP
        for (uint32_t j = 1; j <= M; ++j) {
            staInterfaces.Add(interfaces.Get(j)); // STAs
        }
        address.NewNetwork();
    }

    // Traffic: AP sends to each STA over Round-Robin link
    uint16_t port = 9000;
    ApplicationContainer servers, clients;

    for (uint32_t i = 0; i < M; ++i) {
        // Server on STA
        UdpServerHelper server(port + i);
        servers.Add(server.Install(staNodes.Get(i)));

        // AP Client: delay start, decide link via Python
        Ptr<Node> ap = apNode.Get(0);
        AiInterface::BindCallback([=](uint32_t t) {
            static uint32_t rr = 0;
            uint32_t linkId = AiInterface::Call("get_link", {int(i), int(L)});
            Ptr<NetDevice> dev = apDevices[linkId].Get(0);
            Ipv4Address staAddr = staInterfaces.GetAddress(i);

            UdpClientHelper client(staAddr, port + i);
            client.SetAttribute("MaxPackets", UintegerValue(1));
            client.SetAttribute("Interval", TimeValue(Seconds(0.1)));
            client.SetAttribute("PacketSize", UintegerValue(512));
            client.Install(ap);
        }, MilliSeconds(1000 + i * 100));
    }

    servers.Start(Seconds(1.0));
    Simulator::Stop(Seconds(10.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
