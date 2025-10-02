#include "ns3/ai-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include <map>
#include "ns3/wifi-phy-operating-channel.h"
#include <limits>  // for numeric_limits

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
                        TimeValue(MilliSeconds(500)),
                        MakeTimeAccessor(&MloEnv::m_step), 
                        MakeTimeChecker());
        return tid;
    }

    static Ptr<WifiNetDevice> FindWifiDev(Ptr<Node> node) {
        for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
            auto w = DynamicCast<WifiNetDevice>(node->GetDevice(i));
            if (w) return w;
        }
        return nullptr;
    }


    // wire pointers (keep it dead simple)
    void Wire(NodeContainer* staNodes, Ptr<Node> ap,
            uint32_t numStas, uint32_t numLinks,
            std::vector<std::vector<Ptr<Application>>>* clientApps) {

        m_staNodes  = staNodes;
        m_ap        = ap;
        m_numStas   = numStas;
        m_numLinks  = numLinks;
        m_assignment.resize(m_numStas, 0);
        m_prevAssignment.assign(m_numStas, std::numeric_limits<uint32_t>::max());
        m_clientApps = clientApps;             


        m_snrSumLink.assign(m_numLinks, 0.0);
        m_snrCntLink.assign(m_numLinks, 0);
        m_ulPktsLink.assign(m_numLinks, 0);
        m_busyUsLink.assign(m_numLinks, 0.0);

        // Allocate one sink per AP WifiNetDevice and connect its methods
        m_linkSinks.clear();
        m_linkSinks.reserve(m_numLinks);   

        // Count WiFi devices and assign compact link indices 0..L-1
        uint32_t wifiIdx = 0;
        for (uint32_t i = 0; i < m_ap->GetNDevices(); ++i) {
            Ptr<WifiNetDevice> apDev = DynamicCast<WifiNetDevice>(m_ap->GetDevice(i));
            if (!apDev) continue;

            m_linkSinks.push_back(LinkSinks{this, wifiIdx});   // <-- use wifiIdx, not i
            LinkSinks* sink = &m_linkSinks.back();

            apDev->GetPhy()->TraceConnectWithoutContext(
                "MonitorSnifferRx", MakeCallback(&LinkSinks::Sniff, sink));
            apDev->GetPhy()->TraceConnectWithoutContext(
                "PhyRxEnd",         MakeCallback(&LinkSinks::RxEnd,  sink));
            apDev->GetPhy()->GetState()->TraceConnectWithoutContext(
                "State",            MakeCallback(&LinkSinks::State,  sink));

            ++wifiIdx;
        }

        // Optional safety check (helps catch mismatches early)
        NS_ASSERT_MSG(wifiIdx == m_numLinks,
                    "numLinks does not match number of WifiNetDevices on the AP");



    }

    void OnApSniffRxLink(Ptr<const Packet> pkt,
                        uint16_t /*freqMHz*/,
                        WifiTxVector /*txv*/,
                        MpduInfo /*mpdu*/,
                        SignalNoiseDbm sn,
                        uint16_t /*extra*/,
                        uint32_t linkId)
    {
        // If you want only data later, add the filter back once you see non-zeros.
        m_snrSumLink[linkId] += (sn.signal - sn.noise);
        m_snrCntLink[linkId] += 1;
    }

    void OnApPhyRxEndLink(Ptr<const Packet> pkt, uint32_t linkId)
    {
        WifiMacHeader hdr;
        Ptr<Packet> copy = pkt->Copy();
        if (!copy->PeekHeader(hdr)) return;
        if (!hdr.IsData()) return;

        m_ulPktsLink[linkId] += 1;
    }

    void OnApPhyStateLink(Time /*start*/,
                        Time duration,
                        WifiPhyState state,
                        uint32_t linkId)
    {
        if (state != WifiPhyState::IDLE) {
            m_busyUsLink[linkId] += duration.GetMicroSeconds();
        }
    }

  // ---- Spaces ----
    Ptr<OpenGymSpace> GetObservationSpace() override {
    // low/high are broad; tweak if you want stricter ranges
    return CreateObject<OpenGymBoxSpace>(
        /*low*/  0.0,
        /*high*/ 1e9,
        std::vector<uint32_t>{3u * m_numLinks},
        TypeNameGet<double>()); // we’ll emit all as double for simplicity
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
        const uint32_t L = m_numLinks;
        auto box = CreateObject<OpenGymBoxContainer<double>>(std::vector<uint32_t>{3u * L});

        // SNR block
        for (uint32_t l = 0; l < L; ++l) {
            double meanSnr = m_snrCntLink[l] ? (m_snrSumLink[l] / m_snrCntLink[l]) : 0.0;
            box->AddValue(meanSnr);
        }
        // UL pkts block
        for (uint32_t l = 0; l < L; ++l) box->AddValue(double(m_ulPktsLink[l]));
        // Busy time block
        for (uint32_t l = 0; l < L; ++l) box->AddValue(m_busyUsLink[l] / 1e6);

        // reset per-step accumulators
        std::fill(m_snrSumLink.begin(), m_snrSumLink.end(), 0.0);
        std::fill(m_snrCntLink.begin(), m_snrCntLink.end(), 0);
        std::fill(m_ulPktsLink.begin(), m_ulPktsLink.end(), 0);
        std::fill(m_busyUsLink.begin(), m_busyUsLink.end(), 0.0);
        return box;
    }


  float GetReward() override { return 0.0f; }
  bool  GetGameOver() override { return false; }
  std::string GetExtraInfo() override { return ""; }

bool ExecuteActions(Ptr<OpenGymDataContainer> action) override {
    auto box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
    auto acts = box->GetData();
    for (auto& a : acts) if (a >= m_numLinks) a = m_numLinks - 1;

    std::cout << "Applied STA->link mapping: ";
    for (uint32_t i = 0; i < m_numStas; ++i) std::cout << i << "->" << acts[i] << " ";
    std::cout << std::endl;

    const Time now = Simulator::Now();
    if (now < Seconds(1.0)) {           // ignore actions before traffic epoch
        m_assignment = acts;
        return true;
    }

    for (uint32_t s = 0; s < m_numStas; ++s) {
        const uint32_t newLink = acts[s];
        const uint32_t oldLink = (s < m_prevAssignment.size()
                                  ? m_prevAssignment[s]
                                  : std::numeric_limits<uint32_t>::max());
        for (uint32_t link = 0; link < m_numLinks; ++link) {
            Ptr<Application> app = (*m_clientApps)[s][link];
            if (!app) continue;
            Ptr<UdpClient> uc = DynamicCast<UdpClient>(app);
            if (!uc) continue;

            const bool active = (link == newLink);
            if (active) {
                // ON: 10 pps
                uc->SetAttribute("Interval", TimeValue(Seconds(0.1)));

                // If we just switched to this link, re-arm so a send is scheduled now
                if (newLink != oldLink) {
                    const Time t1 = now + MicroSeconds(1);
                    app->SetStopTime(now);             // cancel any pending old events
                    app->SetStartTime(t1);             // restart immediately
                    app->SetStopTime(Seconds(20.0));   // keep the original end
                }
            } else {
                // OFF: effectively silent
                uc->SetAttribute("Interval", TimeValue(Seconds(1e6)));
            }
        }
    }

    m_prevAssignment = acts;
    m_assignment     = acts;
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
    std::vector<std::vector<Ptr<Application>>>* m_clientApps = nullptr;

    std::vector<double>   m_snrSumLink;   
    std::vector<uint32_t> m_snrCntLink;   
    std::vector<uint32_t> m_ulPktsLink;
    std::vector<double>   m_busyUsLink;  

    std::map<Mac48Address, uint32_t> m_macToSta;

    // state
    std::vector<uint32_t> m_assignment; // per-STA chosen link
    Time m_step;

    std::vector<uint32_t> m_prevAssignment;  // same length as m_assignment

    struct LinkSinks {
    MloEnv*   env = nullptr;
    uint32_t  linkId = 0;

    // EXACT signatures of the trace sources:
    void Sniff(Ptr<const Packet> pkt, uint16_t freqMHz,
                WifiTxVector txv, MpduInfo mpdu,
                SignalNoiseDbm sn, uint16_t extra)
    {
        env->OnApSniffRxLink(pkt, freqMHz, txv, mpdu, sn, extra, linkId);
    }

    void RxEnd(Ptr<const Packet> pkt)
    {
        env->OnApPhyRxEndLink(pkt, linkId);
    }

    void State(Time start, Time duration, WifiPhyState state)
    {
        env->OnApPhyStateLink(start, duration, state, linkId);
    }
    };

    std::vector<LinkSinks> m_linkSinks;


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
    double roomDim = 10;

    std::string transport_prot = "TcpRlTimeBased";

    cmd.AddValue("numStas", "Number of station nodes", numStas);
    cmd.Parse(argc, argv);
    // cmd.AddValue("duration", "Time to allow flows to run in seconds", duration);

    Time::SetResolution(Time::NS); // smallest time unit
    if (simLogs){
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("ApWifiMac", LOG_LEVEL_INFO);
        LogComponentEnable("StaWifiMac", LOG_LEVEL_INFO);
        LogComponentEnable("WifiPhy", LOG_LEVEL_INFO);

    }
    

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
                                  "MinX", DoubleValue(-roomDim),
                                  "MinY", DoubleValue(-roomDim),
                                  "DeltaX", DoubleValue(roomDim),
                                  "DeltaY", DoubleValue(roomDim),
                                  "GridWidth", UintegerValue(3),
                                  "LayoutType", StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
    "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)),
    "Speed",  StringValue("ns3::UniformRandomVariable[Min=0.5|Max=1.5]"), // m/s
    "Distance", DoubleValue(2.0),
    "Bounds", RectangleValue(Rectangle(-roomDim, roomDim, -roomDim, roomDim)));

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

    // choose three non-overlapping 5 GHz channels and widths
    const std::array<std::string, 3> CHCFG = {
        "{1, 20, BAND_2_4GHZ, 0}",   // link 0: ch36  @ 20 MHz
        "{46, 40, BAND_5GHZ, 0}",   // link 1: ch44* @ 40 MHz (primary lower 20)
        "{103, 80, BAND_6GHZ, 0}"   // link 2: ch149 @ 80 MHz (primary index 0)
    };

    // Create 3 links (frequencies)
    for (uint32_t i = 0; i < 3; ++i)
    {
        channelHelpers[i] = YansWifiChannelHelper::Default();

        phyHelpers[i] = YansWifiPhyHelper();
        phyHelpers[i].SetChannel(channelHelpers[i].Create());
        phyHelpers[i].Set("ChannelSettings", StringValue(CHCFG[i]));

        macHelpers[i].SetType("ns3::ApWifiMac",
                            "Ssid", SsidValue(Ssid("link" + std::to_string(i))));

        apDevices[i] = wifi.Install(phyHelpers[i], macHelpers[i], apNode);

        // STA installation on every link
        WifiMacHelper staMac;
        staMac.SetType("ns3::StaWifiMac",
                   "Ssid", SsidValue(Ssid("link" + std::to_string(i))),
                   "ActiveProbing", BooleanValue(false));
        staDevices[i] = wifi.Install(phyHelpers[i], staMac, staNodes);
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

    std::vector<std::vector<Ptr<Application>>> clientApps(numStas,
        std::vector<Ptr<Application>>(numLinks, nullptr));

    uint16_t port = 9;
    UdpServerHelper udpServer(port);
    ApplicationContainer serverApps = udpServer.Install(apNode.Get(0)); // AP node
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));


    // UDP Client on each STA to send uplink traffic to AP
    for (uint32_t link = 0; link < numLinks; ++link)
    {
        Ipv4Address apAddr = apInterfaces[link].GetAddress(0); // AP IP on this link
        
    for (uint32_t s = 0; s < numStas; ++s) {
        UdpClientHelper udpClient(apAddr, port);
        udpClient.SetAttribute("MaxPackets", UintegerValue(0));           // unlimited
        udpClient.SetAttribute("Interval",   TimeValue(Seconds(0.1)));
        udpClient.SetAttribute("PacketSize", UintegerValue(1024));

        ApplicationContainer apps = udpClient.Install(staNodes.Get(s));

        Ptr<Application> app = apps.Get(0);
        // keep it OFF by default
        app->SetStartTime(Seconds(1.0));  // far in the future
        app->SetStopTime(Seconds(20.0));

        clientApps[s][link] = app;
    }
    }
    
    // OpenGym interface + env
    Ptr<OpenGymInterface> iface = OpenGymInterface::Get();

    Ptr<MloEnv> env = CreateObject<MloEnv>();
    env->Wire(&staNodes, apNode.Get(0), numStas, numLinks, &clientApps);
    env->SetOpenGymInterface(iface);
    env->Start();

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

    for (uint32_t i = 0; i < serverApps.GetN(); ++i) {
        Ptr<UdpServer> srv = DynamicCast<UdpServer>(serverApps.Get(i));
        if (srv) {
            std::cout << "Server received: " << srv->GetReceived() << " packets\n";
        }
        }
    
    iface->NotifySimulationEnd();
    Simulator::Destroy();


    return 0;

}