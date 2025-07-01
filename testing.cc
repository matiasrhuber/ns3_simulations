#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/mlo-module.h"


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
    Time::SetResolution(Time::NS); // smallest time unit

    LogComponentEnable("WifiNetDevice", LOG_LEVEL_INFO); // Enable logging for WifiNetDevice
    LogComponentEnable("MloManager", LOG_LEVEL_INFO); // Enable logging for MLO manager 
    
    uint32_t numStas = 5;
    uint32_t numLinks = 3;
    
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
    YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phyHelpers = YansWifiPhyHelper::Default();

  for (uint32_t i = 0; i < numLinks; i++)
    {
      phyHelpers.emplace_back (YansWifiPhyHelper::Default ());
      phyHelpers[i].SetChannel (channelHelper.Create ());
    }
}