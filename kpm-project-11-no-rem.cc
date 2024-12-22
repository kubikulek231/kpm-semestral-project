/**
 *  __  __   ______  __    __       ______  ______   ______      __   ______   ______   ______ 
 * /\ \/ /  /\  == \/\ "-./  \     /\  == \/\  == \ /\  __ \    /\ \ /\  ___\ /\  ___\ /\__  _\ 
 * \ \  _"-.\ \  _-/\ \ \-./\ \    \ \  _-/\ \  __< \ \ \/\ \  _\_\ \\ \  __\ \ \ \____\/_/\ \/ 
 *  \ \_\ \_\\ \_\   \ \_\ \ \_\    \ \_\   \ \_\ \_\\ \_____\/\_____\\ \_____\\ \_____\  \ \_\ 
 *   \/_/\/_/ \/_/    \/_/  \/_/     \/_/    \/_/ /_/ \/_____/\/_____/ \/_____/ \/_____/   \/_/
 * 
 * \file kpm-project.cc
 * \brief 5G NR simulation with multiple gNBs and UEs.
 *
 * This simulation is based on the **cttc-nr-demo.cc** example from the **ns3 LENA 5G** module. It uses a grid 
 * topology with configurable parameters such as frequency bands following the 3GPP TS 38.300 and TR 38.901 models.
 *
 * By default, it meets the requirements of **Assignment 11** for the **MPA-KPM Project**. Custom parameters 
 * can be provided through command-line arguments to modify the configuration.
 *
 * Simulation results are displayed on-screen and written to a file.
 *
 * \code{.unparsed}
$ ./ns3 run "kpm-project --help"
 * \endcode
 *
 * \author
 * Matej Baranyk, Slavek Rylich, Jakub Lepik, Martin Dolak
 * Team for the **MPA-KPM Project**, Assignment 11
 * 
 * \date December 2024
 *
 */


#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("KpmProject");

int
main(int argc, char* argv[])
{
    /** ______  ______   ______   ______   __    __   ______    
    *  /\  == \/\  __ \ /\  == \ /\  __ \ /\ "-./  \ /\  ___\   
    *  \ \  _-/\ \  __ \\ \  __< \ \  __ \\ \ \-./\ \\ \___  \  
    *   \ \_\   \ \_\ \_\\ \_\ \_\\ \_\ \_\\ \_\ \ \_\\/\_____\ 
    *    \/_/    \/_/\/_/ \/_/ /_/ \/_/\/_/ \/_/  \/_/ \/_____/
    */

    // Command line argument parsing
    std::string direction = "DL";  // Default is "DL"
    std::string mode = "COVERAGE_AREA";  // Default is "COVERAGE_AREA"
    bool rem = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("direction", "Direction of the REM: 'UL' or 'DL'", direction);
    cmd.AddValue("mode", "Mode for the REM: 'BEAM_SHAPE', 'COVERAGE_AREA', or 'UE_COVERAGE'", mode);
    cmd.AddValue("rem", "Enable or disable REM.", rem);

    // If --PrintHelp is provided, display the help message and exit
    cmd.Parse(argc, argv);

    // Scenario parameters (that we will use inside this script):
    uint16_t numGnb = 3;
    uint16_t numUePerGnb = 2;
    uint32_t numTotalUe = numGnb * numUePerGnb;
    uint32_t totalUesCall = 2; // Total voice UEs
    uint32_t totalUesBrowse = 3; // Total browsing UEs

    int logging = 1;

    // Traffic parameters (that we will use inside this script):
    // Packet size in bytes
    uint32_t udpPacketSizeBrowsing = 25;
    uint32_t udpPacketSizeVoiceCall = 50;
    // Number of UDP packets in one second
    uint32_t lambdaBrowsing = 10000;
    uint32_t lambdaVoiceCall = 10000;

    // Simulation parameters.
    Time simTime = MilliSeconds(100);
    Time udpAppStartTime = MilliSeconds(10);

    // NR parameters (Reference: 3GPP TR 38.901 V17.0.0 (Release 17)
    // Table 7.8-1 for the power and BW).

    // Two separate BWPs
    // Voice Call
    uint16_t numerologyBwp1 = 4;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 50e6;
    double totalTxPower = 35;
    // Web browsing
    uint16_t numerologyBwp2 = 2; 
    double centralFrequencyBand2 = 28.2e9;
    double bandwidthBand2 = 50e6;

    // Where we will store the output files.
    std::string simTag = "default";
    std::string outputDir = "./";

    // Rem parameters
    double xMin = -40.0;
    double xMax = 80.0;
    uint16_t xRes = 50;
    double yMin = -70.0;
    double yMax = 50.0;
    uint16_t yRes = 50;
    double z = 1.5;

    /*
    * Ensure that the frequency band is in the mmWave range 
    * and the number of UEs matches the assignment (section 2.2, 2.3).
    */
    NS_ABORT_IF(centralFrequencyBand1 == centralFrequencyBand2);
    NS_ABORT_IF(centralFrequencyBand1 < 2e9 && centralFrequencyBand1 > 100e9);
    NS_ABORT_IF(centralFrequencyBand2 < 2e9 && centralFrequencyBand2 > 100e9);
    NS_ABORT_IF(numTotalUe < 5 or numGnb < 2);

    // Enable logging for the components
    if (logging > 0)
    {
        LogComponentEnable("KpmProject", LOG_LEVEL_INFO);
    }
    if (logging > 1) {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    /** ______   ______  ______   __  __   ______   ______  __  __   ______   ______    
    *  /\  ___\ /\__  _\/\  == \ /\ \/\ \ /\  ___\ /\__  _\/\ \/\ \ /\  == \ /\  ___\   
    *  \ \___  \\/_/\ \/\ \  __< \ \ \_\ \\ \ \____\/_/\ \/\ \ \_\ \\ \  __< \ \  __\   
    *   \/\_____\  \ \_\ \ \_\ \_\\ \_____\\ \_____\  \ \_\ \ \_____\\ \_\ \_\\ \_____\ 
    *    \/_____/   \/_/  \/_/ /_/ \/_____/ \/_____/   \/_/  \/_____/ \/_/ /_/ \/_____/ 
    */

    // Define the mobility using the GridScenarioHelper class as specified in section 2.2 of the assignment.
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(numGnb);
    // All units below are in meters
    gridScenario.SetHorizontalBsDistance(10.0);
    gridScenario.SetVerticalBsDistance(10.0);
    gridScenario.SetBsHeight(10);
    gridScenario.SetUtHeight(1.5);
    // must be set before BS number
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(numGnb);
    gridScenario.SetUtNumber(numUePerGnb * numGnb);
    gridScenario.SetScenarioHeight(3); // Create a 3x3 scenario where the UE will
    gridScenario.SetScenarioLength(3); // be distributed.
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    /*
    * Create two separate NodeContainers for different traffic types:
    * - ueBrowsingWebContainer: Devices browsing the web.
    * - uePhoneCallContainer: Devices in a call, each connected to a different gNB.
    * This is implemented as specified in section 2.3 of the assignment.
    *
    * Required:
    * - At least 2 UEs from different gNBs must be placed in the voice call container (uePhoneCallContainer).
    * - At least 3 UEs must be placed in the browsing container (ueBrowsingWebContainer).
    */
    NodeContainer ueBrowsingWebContainer;
    NodeContainer uePhoneCallContainer;

    // Distribute UEs to containers
    uint32_t ueCountForBrowsing = 0;
    uint32_t ueCountForVoice = 0;

    // Ensure correct UE distribution among the containers
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); j++)
    {
        // Get the UE at index j
        Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);

        // Alternate between adding UEs to the voice and browsing containers
        if (j % 2 == 0) {
            // Add to the voice call container
            uePhoneCallContainer.Add(ue);
            ueCountForVoice++;           // Increment the voice call UE count
            NS_LOG_INFO("Adding UE with ID" << ue->GetId() << " to voice Phone Call container.");
        } else {
            // Add to the browsing container
            ueBrowsingWebContainer.Add(ue);
            ueCountForBrowsing++;        // Increment the browsing UE count
            NS_LOG_INFO("Adding UE with ID" << ue->GetId() << " to Web Browsing container.");
        }
    }

    // Check if the conditions hold (this is done after the UEs have been assigned)
    NS_ABORT_IF(uePhoneCallContainer.GetN() < 2); // Ensure at least 2 UEs in voice container
    NS_ABORT_IF(ueBrowsingWebContainer.GetN() < 3); // Ensure at least 3 UEs in browsing container

    NS_LOG_INFO("Creating " << gridScenario.GetUserTerminals().GetN() << " user terminals and "
                            << gridScenario.GetBaseStations().GetN() << " gNBs");

    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - nrEpcHelper, which will setup the core network
     * - IdealBeamformingHelper, which takes care of the beamforming part
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    /*
    * Spectrum division. We create a single operational band containing
    * one component carrier (CC), and the CC containing a single bandwidth part
    * centered at the frequency specified by the input parameters.
    * The spectrum length is specified by the input parameters.
    * This band uses the StreetCanyon channel modeling.
    */

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; // Only one CC in this single band

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
    // a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::UMi_StreetCanyon);
    CcBwpCreator::SimpleOperationBandConf bandConf2(centralFrequencyBand2,
                                                    bandwidthBand2,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::UMi_StreetCanyon);

    // By using the configuration created, it is time to make the operation bands
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf2);

    /*
     * The configured spectrum division is:
     * ------------Band1--------------|--------------Band2-----------------
     * ------------CC1----------------|--------------CC2-------------------
     * ------------BWP1---------------|--------------BWP2------------------
     */

    /*
     * Attributes of ThreeGppChannelModel still cannot be set in our way.
     * TODO: Coordinate with Tommaso
     */
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    /*
    * Initialize channel and pathloss, plus other things inside band. 
    */
    nrHelper->InitializeOperationBand(&band1);

    /*
    * Start to account for the bandwidth used by the example, as well as
    * the total power that has to be divided among the BWP.
    */
    double x = pow(10, totalTxPower / 10);
    double totalBandwidth = bandwidthBand1;

    // Initialize channel and pathloss, plus other things inside band2
    nrHelper->InitializeOperationBand(&band2);
    totalBandwidth += bandwidthBand2;
    allBwps = CcBwpCreator::GetAllBwps({band1, band2});

    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    Packet::EnableChecking();
    Packet::EnablePrinting();

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
                                     
    // Declare BWP IDs for different traffic types
    uint32_t bwpIdForBrowsing = 0;  // BWP ID for browsing web traffic (eMBB)
    uint32_t bwpIdForCall = 1;      // BWP ID for voice call traffic (GBR)

    // gNB routing between Bearer and Bandwidth Part
    // This routes the different traffic types to their respective BWPs
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForBrowsing)); // eMBB (web browsing)
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForCall));         // GBR (voice call)

    // UE routing between Bearer and Bandwidth Part
    // This routes the different traffic types to their respective BWPs for UEs
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForBrowsing)); // eMBB (web browsing)
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForCall));         // GBR (voice call)

    // Loop through all UEs in the voice call container
    for (uint32_t i = 0; i < uePhoneCallContainer.GetN(); ++i)
    {
        Ptr<Node> ue = uePhoneCallContainer.Get(i);
        // Assign the voice call BWP (GBR)
        nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForCall));
        NS_LOG_INFO("Assigning GBR_CONV_VOICE BWP to UE with ID" << ue->GetId() << " for Voice Call.");
    }

    // Loop through all UEs in the web browsing container
    for (uint32_t i = 0; i < ueBrowsingWebContainer.GetN(); ++i)
    {
        Ptr<Node> ue = ueBrowsingWebContainer.Get(i);
        // Assign the browsing BWP (eMBB)
        nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForBrowsing));
        NS_LOG_INFO("Assigning NGBR_LOW_LAT_EMBB BWP to UE with ID" << ue->GetId() << " for Web Browsing.");
    }

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // DEFAULTS IN THIS CASE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */

    NetDeviceContainer gnbNetDev =
        nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueBrowsingWebNetDev = nrHelper->InstallUeDevice(ueBrowsingWebContainer, allBwps);
    NetDeviceContainer uePhoneCallNetDev = nrHelper->InstallUeDevice(uePhoneCallContainer, allBwps);

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueBrowsingWebNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(uePhoneCallNetDev, randomStream);
    
    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // Set the appropriate bandwith and TxPower for each gNB
    // Iterate through all gNBs in the gnbNetDev container
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {        
        // Calculate the TxPower for the current gNB, considering the total bandwidth and other parameters
        double txPower = 10 * log10((bandwidthBand1 / totalBandwidth) * x);

        // Get the first bandwidth part (0)
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
        nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
            ->SetAttribute("TxPower", DoubleValue(txPower));

        // Get the second bandwidth part (1)
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 1)
            ->SetAttribute("Numerology", UintegerValue(numerologyBwp2));
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 1)
            ->SetTxPower(txPower);
    }

    // When all the configuration is done, explicitly call UpdateConfig ()
    nrHelper->UpdateDeviceConfigs(gnbNetDev);
    nrHelper->UpdateDeviceConfigs(ueBrowsingWebNetDev);
    nrHelper->UpdateDeviceConfigs(uePhoneCallNetDev);

    // In a typical EPC architecture, we have:
    // - **SGW (Serving Gateway)**: Acts as the gateway between the Radio Access Network (RAN) and the core network.
    // - **PGW (Packet Gateway)**: Interfaces the core network to the external internet, handling IP addressing and routing.
    // Here, we set up these components and connect them to simulate data flow between the UEs and the internet.

    // Get the PGW (Packet Gateway) node from the EPC helper
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();

    // Create a remote host to simulate an external network (internet)
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // Install the internet stack (IP, routing, etc.) on the remote host
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Connect the remote host to the PGW, simulating the internet connection
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s"))); // High data rate between PGW and remote host
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500)); // Maximum Transmission Unit (MTU) set
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000))); // Minimal delay
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Set up IPv4 address for the internet devices and configure routing
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0"); // IP address range for the internet connection
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    // Configure routing for the remote host, simulating a route to the mobile UE's network
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Install the internet stack on all UEs in the simulation grid
    internet.Install(gridScenario.GetUserTerminals());

    // Assign IPv4 addresses to the UEs (web-browsing and voice UEs)
    Ipv4InterfaceContainer ueLowLatIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueBrowsingWebNetDev));
    Ipv4InterfaceContainer ueVoiceIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(uePhoneCallNetDev));

    NS_LOG_INFO("Assigned IP addresses for web-browsing UEs:");
    for (uint32_t i = 0; i < ueLowLatIpIface.GetN(); ++i)
    {
        Ptr<NetDevice> ueDev = ueBrowsingWebNetDev.Get(i);
        Ipv4Address ipAddr = ueLowLatIpIface.GetAddress(i);
        NS_LOG_INFO("- UE with ID " << ueDev->GetNode()->GetId() << " has IP address: " << ipAddr);
    }

    // Log the assigned IP addresses for voice call devices
    NS_LOG_INFO("Assigned IP addresses for voice call UEs:");
    for (uint32_t i = 0; i < ueVoiceIpIface.GetN(); ++i)
    {
        Ptr<NetDevice> ueDev = uePhoneCallNetDev.Get(i);
        Ipv4Address ipAddr = ueVoiceIpIface.GetAddress(i);
        NS_LOG_INFO("- UE with ID " << ueDev->GetNode()->GetId() << " has IP address: " << ipAddr);
    }

    // Set the default gateway for each UE to route traffic through the SGW/PGW
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(
            gridScenario.GetUserTerminals().Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // // Attach UEs to the closest gNB (Next Generation NodeB), enabling the air interface communication
    // nrHelper->AttachToClosestGnb(ueBrowsingWebNetDev, gnbNetDev);
    // nrHelper->AttachToClosestGnb(uePhoneCallNetDev, gnbNetDev);

    // Attach manually
    uint32_t callIndex = 0;   // Current index for voice UEs
    uint32_t browseIndex = 0; // Current index for browsing UEs

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        Ptr<NetDevice> bs = gnbNetDev.Get(i); // Get the base station device

        for (uint32_t j = 0; j < numUePerGnb; j++)
        {
            Ptr<NetDevice> ueDev;

            // Alternate between voice and browsing UEs, ensuring no overflow
            if (j % 2 == 0) {
                if (callIndex < totalUesCall) {
                    ueDev = uePhoneCallNetDev.Get(callIndex++);
                } else {
                    // Log specific UE ID that won't be added to BS
                    NS_LOG_WARN("UE with ID " << uePhoneCallNetDev.Get(callIndex)->GetNode()->GetId()
                                << " won't be added to BS " << bs->GetNode()->GetId() << " due to the limit on voice UEs.");
                    continue; // Skip this iteration if no voice UE is available
                }
            } else {
                if (browseIndex < totalUesBrowse) {
                    ueDev = ueBrowsingWebNetDev.Get(browseIndex++);
                } else {
                    // Log specific UE ID that won't be added to BS
                    NS_LOG_WARN("UE with ID " << ueBrowsingWebNetDev.Get(browseIndex)->GetNode()->GetId()
                                << " won't be added to BS " << bs->GetNode()->GetId() << " due to the limit on browsing UEs.");
                    continue; // Skip this iteration if no browsing UE is available
                }
            }

            // Attach the UE to the base station
            nrHelper->AttachToGnb(ueDev, bs);
            NS_LOG_INFO("Adding UE with ID " << ueDev->GetNode()->GetId() << " to BS " << bs->GetNode()->GetId());
        }
    }

    // Final check for any unassigned UEs
    if (callIndex < totalUesCall) {
        NS_LOG_WARN("Some voice UEs were not assigned to any gNB.");
    }
    if (browseIndex < totalUesBrowse) {
        NS_LOG_WARN("Some browsing UEs were not assigned to any gNB.");
    }

    /** ______  ______   ______   ______  ______  __   ______    
     * /\__  _\/\  == \ /\  __ \ /\  ___\/\  ___\/\ \ /\  ___\   
     * \/_/\ \/\ \  __< \ \  __ \\ \  __\\ \  __\\ \ \\ \ \____  
     *    \ \_\ \ \_\ \_\\ \_\ \_\\ \_\   \ \_\   \ \_\\ \_____\ 
     *     \/_/  \/_/ /_/ \/_/\/_/ \/_/    \/_/    \/_/ \/_____/ 
     */

    /*
     * Traffic part. Install two kind of traffic: low-latency and voice, each
     * identified by a particular source port.
     */
    uint16_t dlPortBrowsing = 1234;
    uint16_t dlPortVoiceCall = 1235;

    ApplicationContainer serverApps;

    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSinkBrowsing(dlPortBrowsing);
    UdpServerHelper dlPacketSinkVoiceCall(dlPortVoiceCall);

    NS_LOG_INFO("Setting up Web Browsing and Voice Call Server");

    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add(dlPacketSinkBrowsing.Install(ueBrowsingWebContainer));
    serverApps.Add(dlPacketSinkVoiceCall.Install(uePhoneCallContainer));

    // Web browsing traffic configuration    
    UdpClientHelper dlClientBrowsing;
    dlClientBrowsing.SetAttribute("RemotePort", UintegerValue(dlPortBrowsing));
    dlClientBrowsing.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientBrowsing.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBrowsing));
    dlClientBrowsing.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambdaBrowsing)));
    NrEpsBearer bearerBrowsing(NrEpsBearer::NGBR_LOW_LAT_EMBB);

    // The filter for the Web Browsing traffic
    Ptr<NrEpcTft> tftBrowsing = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpfLowLat;
    dlpfLowLat.localPortStart = dlPortBrowsing;
    dlpfLowLat.localPortEnd = dlPortBrowsing;
    tftBrowsing->Add(dlpfLowLat);

    // Voice configuration and object creation for both client and server
    UdpClientHelper dlClientVoice;
    dlClientVoice.SetAttribute("RemotePort", UintegerValue(dlPortVoiceCall));
    dlClientVoice.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientVoice.SetAttribute("PacketSize", UintegerValue(udpPacketSizeVoiceCall));
    dlClientVoice.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambdaVoiceCall)));

    // Create the voice bearer
    NrEpsBearer bearerVoice(NrEpsBearer::GBR_CONV_VOICE);

    // The filter for the voice call traffic (same for client and server)
    Ptr<NrEpcTft> tftVoice = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpfVoice;
    dlpfVoice.localPortStart = dlPortVoiceCall;
    dlpfVoice.localPortEnd = dlPortVoiceCall;
    tftVoice->Add(dlpfVoice);

    /*
    * Set up and install applications for web browsing and voice call traffic on UEs.
    * We install UDP clients and servers for both browsing and voice traffic.
    */
    ApplicationContainer clientApps;

    ///////////////////////////////////////////////
    // Web Browsing Traffic Setup -- Client
    ///////////////////////////////////////////////

    for (uint32_t i = 0; i < ueBrowsingWebContainer.GetN(); ++i)
    {
        // Get the UE node and its corresponding network device
        Ptr<Node> ue = ueBrowsingWebContainer.Get(i);
        Ptr<NetDevice> ueDevice = ueBrowsingWebNetDev.Get(i);

        // Log the UE and device being set up for web browsing
        NS_LOG_INFO("Setting up Web Browsing Client for UE ID: " << ue->GetId());

        // Get the IP address of the UE for browsing
        Address ueAddress = ueLowLatIpIface.GetAddress(i);
        
        // Configure the UDP client for web browsing traffic:
        dlClientBrowsing.SetAttribute("RemoteAddress", AddressValue(ueAddress));

        // Install the client application on the remote host (the server in this case)
        clientApps.Add(dlClientBrowsing.Install(remoteHost));

        // Activate a dedicated bearer for browsing traffic with the specified TFT (Traffic Flow Template)
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearerBrowsing, tftBrowsing);
    }

    /////////////////////////////////////////////
    // Voice Call Traffic Setup -- Client
    ///////////////////////////////////////////

    for (uint32_t i = 0; i < uePhoneCallContainer.GetN(); ++i)
    {
        // Get the UE node and its corresponding network device
        Ptr<Node> ue = uePhoneCallContainer.Get(i);
        Ptr<NetDevice> ueDevice = uePhoneCallNetDev.Get(i);

        // Log the UE and device being set up for voice call traffic
        NS_LOG_INFO("Setting up Voice Call Client for UE ID: " << ue->GetId());

        // Get the IP address of the UE for voice call
        Address ueAddress = ueVoiceIpIface.GetAddress(i);

        // Configure the UDP client for voice call traffic:
        dlClientVoice.SetAttribute("RemoteAddress", AddressValue(ueAddress));

        // Install the client application on the remote host (the server in this case)
        clientApps.Add(dlClientVoice.Install(remoteHost));

        // Activate a dedicated bearer for voice call traffic with the specified TFT (Traffic Flow Template)
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearerVoice, tftVoice);
    }

    ///////////////////////////////////////////////
    // Starting and Stopping Applications
    ///////////////////////////////////////////////

    // Start both the server and client applications at the specified time
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);

    // Stop both the server and client applications at the end of the simulation time
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    FlowMonitorHelper flowmonHelper;
    flowmonHelper.InstallAll();  // Install Flow Monitor on all nodes and devices
    NodeContainer endpointNodes;
    endpointNodes.Add(gridScenario.GetUserTerminals());

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    Simulator::Stop(simTime);
    NS_LOG_INFO("Starting the simulation ...");
    Simulator::Run();
    NS_LOG_INFO("Simulation finished ...");

    /*
     * To check what was installed in the memory, i.e., BWPs of gNB Device, and its configuration.
     * Example is: Node 1 -> Device 0 -> BandwidthPartMap -> {0,1} BWPs -> NrGnbPhy -> Numerology,
    GtkConfigStore config;
    config.ConfigureAttributes ();
    */

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    std::string filename = outputDir + "/" + simTag;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    double flowDuration = (simTime - udpAppStartTime).GetSeconds();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
                << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        outFile << "  Lost Packets: " << i->second.txPackets - i->second.rxPackets << "\n";
        outFile << "  Packet loss: " << (((i->second.txPackets - i->second.rxPackets) * 1.0) / i->second.txPackets) * 100 << "%" << "\n";

        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000
                    << " Mbps\n";
            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            // outFile << "  Mean upt:  " << i->second.uptSum / i->second.rxPackets / 1000/1000 << "
            // Mbps \n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    double meanFlowThroughput = averageFlowThroughput / stats.size();
    double meanFlowDelay = averageFlowDelay / stats.size();

    outFile << "\n\n  Mean flow throughput: " << meanFlowThroughput << "\n";
    outFile << "  Mean flow delay: " << meanFlowDelay << "\n";

    outFile.close();

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();

    if (argc == 0)
    {
        double toleranceMeanFlowThroughput = 0.0001 * 56.258560;
        double toleranceMeanFlowDelay = 0.0001 * 0.553292;

        if (meanFlowThroughput >= 56.258560 - toleranceMeanFlowThroughput &&
            meanFlowThroughput <= 56.258560 + toleranceMeanFlowThroughput &&
            meanFlowDelay >= 0.553292 - toleranceMeanFlowDelay &&
            meanFlowDelay <= 0.553292 + toleranceMeanFlowDelay)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else if (argc == 1 and numUePerGnb == 9) // called from examples-to-run.py with these parameters
    {
        double toleranceMeanFlowThroughput = 0.0001 * 47.858536;
        double toleranceMeanFlowDelay = 0.0001 * 10.504189;

        if (meanFlowThroughput >= 47.858536 - toleranceMeanFlowThroughput &&
            meanFlowThroughput <= 47.858536 + toleranceMeanFlowThroughput &&
            meanFlowDelay >= 10.504189 - toleranceMeanFlowDelay &&
            meanFlowDelay <= 10.504189 + toleranceMeanFlowDelay)
        {
            return EXIT_SUCCESS;
        }
        else
        {
            return EXIT_FAILURE;
        }
    }
    else
    {
        return EXIT_SUCCESS; // we dont check other parameters configurations at the moment
    }
}
