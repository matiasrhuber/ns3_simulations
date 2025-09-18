#include "ns3/ai-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

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

// // 1) Minimal env that does nothing but is valid
// Minimal OpenGym env: 1D obs (time), 1D action (ignored), periodic Notify()
class MloEnv : public OpenGymEnv {
public:
  static TypeId GetTypeId() {
    static TypeId tid = TypeId("ns3::MloEnv")
      .SetParent<OpenGymEnv>()
      .SetGroupName("OpenGym")
      .AddConstructor<MloEnv>()
      .AddAttribute("StepTime", "Notify interval",
                    TimeValue(MilliSeconds(200)),
                    MakeTimeAccessor(&MloEnv::m_step), 
                    MakeTimeChecker());
    return tid;
  }

  // wire pointers (keep it dead simple)
  void Wire(NodeContainer* staNodes, Ptr<Node> ap, uint32_t numStas, uint32_t numLinks) {
    m_staNodes  = staNodes;
    m_ap        = ap;
    m_numStas   = numStas;
    m_numLinks  = numLinks;
    m_assignment.resize(m_numStas, 0); // default all to link 0
  }

  // ---- Spaces ----
  Ptr<OpenGymSpace> GetObservationSpace() override {
    // distances: double[numStas]
    return CreateObject<OpenGymBoxSpace>(
      /*low*/  0.0,
      /*high*/ 1e6,                                  // meters upper bound (big enough)
      std::vector<uint32_t>{m_numStas},
      TypeNameGet<double>());
  }


  Ptr<OpenGymSpace> GetActionSpace() override {
    // per-STA link index: uint32[numStas] in [0, numLinks-1]
    return CreateObject<OpenGymBoxSpace>(
      /*low*/  0.0,
      /*high*/ static_cast<double>(m_numLinks - 1),
      std::vector<uint32_t>{m_numStas},
      TypeNameGet<uint32_t>());
  }

  // ---- Data exchange ----
  Ptr<OpenGymDataContainer> GetObservation() override {
    auto box = CreateObject<OpenGymBoxContainer<double>>(std::vector<uint32_t>{m_numStas});
    // AP position (single AP in your script)
    Vector apPos = m_ap->GetObject<MobilityModel>()->GetPosition();
    for (uint32_t i = 0; i < m_numStas; ++i) {
      Vector staPos = m_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
      double d = CalculateDistance(staPos, apPos);   // meters
      box->AddValue(d);
    }
    return box;
  }


  float GetReward() override { return 0.0f; }
  bool  GetGameOver() override { return false; }
  std::string GetExtraInfo() override { return ""; }

  bool ExecuteActions(Ptr<OpenGymDataContainer> action) override {
    // Expect uint32[numStas] link IDs
    auto box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
    if (!box) return false;

    for (uint32_t i = 0; i < m_numStas; ++i) {
      uint32_t linkId = box->GetValue(i);
      if (linkId >= m_numLinks) linkId = m_numLinks - 1;  // clamp just in case
      m_assignment[i] = linkId;
    }
    // TODO: apply m_assignment to your Wi-Fi link selection logic, if/when you want
    return true;
  }

  void Start() {
    Notify(); // triggers SimInit handshake
    Simulator::Schedule(m_step, &MloEnv::Tick, this);
  }
private:
  void Tick() {
    Notify(); // push obs, receive action
    Simulator::Schedule(m_step, &MloEnv::Tick, this);
  }

  // wired-in context
  NodeContainer* m_staNodes = nullptr;
  Ptr<Node>      m_ap;
  uint32_t       m_numStas = 0;
  uint32_t       m_numLinks = 0;

  // state
  std::vector<uint32_t> m_assignment; // per-STA chosen link
  Time m_step;
};





int main(int argc, char* argv[])
{
    uint32_t numStas = 5;
    uint32_t numLinks = 3;
    CommandLine cmd;
    bool genStats = true;
    bool simLogs = false;
    double duration = 1000.0;
    double tcpEnvTimeStep = 0.1;
    std::string transport_prot = "TcpRlTimeBased";

    cmd.AddValue("numStas", "Number of station nodes", numStas);
    cmd.Parse(argc, argv);
    // cmd.AddValue("duration", "Time to allow flows to run in seconds", duration);

    Time::SetResolution(Time::NS); // smallest time unit
    if (simLogs){
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
    }


    // Config::SetDefault("ns3::TcpTimeStepEnv::StepTime", TimeValue(Seconds(tcpEnvTimeStep)));
    // transport_prot = std::string("ns3::") + transport_prot;
    // Config::SetDefault("ns3::TcpL4Protocol::SocketType",
    //                 TypeIdValue(TypeId::LookupByName(transport_prot)));

    // // OpenGym Env --- has to be created before any other thing
    // Ptr<OpenGymInterface> openGymInterface;
    // openGymInterface = OpenGymInterface::Get();
    

    // Relevant classes
    NodeContainer apNode;
    apNode.Create(1);

    NodeContainer staNodes;
    staNodes.Create(numStas);

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
    
    // OpenGym interface + env
    Ptr<OpenGymInterface> iface = OpenGymInterface::Get();

    Ptr<MloEnv> env = CreateObject<MloEnv>();
    env->Wire(&staNodes, apNode.Get(0), numStas, numLinks);
    env->SetOpenGymInterface(iface);
    env->Start();  // <-- this calls Notify(), which sends SimInitMsg and unblocks Python

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

    // // Wait for user input before simulation starts
    // std::cin.get();

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();


    // Flow monitor results
    flowMonitor->SerializeToXmlFile("flowmon-results.xml", true, true);



    // General Stats
    if (genStats){
        std::cout << "Number of STA nodes: " << numStas << std::endl;
        std::cout << "Number of links: " << numLinks << std::endl;
        std::cout << "Simulation running with uplink UDP traffic from STAs to AP on port 9." << std::endl;

        auto printPos = [](Ptr<Node> n, const std::string& name) {
            Vector p = n->GetObject<MobilityModel>()->GetPosition();
            std::cout << name << " @ (" << p.x << ", " << p.y << ")\n";
        };

        printPos(apNode.Get(0), "AP");
        for (uint32_t i = 0; i < staNodes.GetN(); ++i) {
            printPos(staNodes.Get(i), "STA" + std::to_string(i));
        }
    }
    
    iface->NotifySimulationEnd();
    Simulator::Destroy();


    return 0;

}