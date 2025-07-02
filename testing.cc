#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
// #include "ns3/mlo-module.h"


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

    LogComponentEnable("WifiNetDevice", LOG_LEVEL_INFO); // Enable logging for WifiNetDevice
    //LogComponentEnable("MloManager", LOG_LEVEL_INFO); // Enable logging for MLO manager 

    // Relevant classes
    NodeContainer apNode;
    apNode.Create (1);

    NodeContainer staNodes;
    staNodes.Create (numStas);

    // Create Wifi helper and set standard to 802.11be (WiFi 8)
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211be);

    // YANS wifi simulation: Combines large-scale path loss 
    // with small-scale fading effects, giving a more realistic 
    // representation of wireless channel behavior.
    // DEFINES THE SIMULATION (WIRELESS) MEDIUM
    std::vector<YansWifiChannelHelper> channelHelpers(3);
    std::vector<YansWifiPhyHelper> phyHelpers(3);
    std::vector<WifiMacHelper> macHelpers(3);
    std::vector<NetDeviceContainer> apDevices(3);
    std::vector<NetDeviceContainer> staDevices(3);

    // Create 3 links (frequencies)
    for (uint32_t i = 0; i < 3; ++i)
    {
        channelHelpers[i] = YansWifiChannelHelper::Default();
        phyHelpers[i] = YansWifiPhyHelper();  //::Default()
        phyHelpers[i].SetChannel(channelHelpers[i].Create());
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
//   for (uint32_t i = 0; i < numLinks; i++)
//     {
//       phyHelpers.emplace_back (YansWifiPhyHelper::Default ());
//       phyHelpers[i].SetChannel (channelHelper.Create ());
//     }
    
}