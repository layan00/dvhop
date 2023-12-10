/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/dvhop-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <unistd.h>
using namespace ns3;

/**
 * \brief Test script.
 *
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 *
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 *
 *
 */
 
 struct point
{
    double x,y;
};

float norm (point p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}

double calculateError(const point& p1, double r1,
                      const point& p2, double r2,
                      const point& p3, double r3,
                      const point& result) {
    double error1 = std::abs(std::sqrt((p1.x - result.x) * (p1.x - result.x) + (p1.y - result.y) * (p1.y - result.y)) - r1);
    double error2 = std::abs(std::sqrt((p2.x - result.x) * (p2.x - result.x) + (p2.y - result.y) * (p2.y - result.y)) - r2);
    double error3 = std::abs(std::sqrt((p3.x - result.x) * (p3.x - result.x) + (p3.y - result.y) * (p3.y - result.y)) - r3);

    return error1 + error2 + error3;
}

// trilateration function that changes result and returns point
point trilateration(const point& p1, const point& p2, const point& p3, double d1, double d2, double d3)
{
    point result;
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;
    double x3 = p3.x;
    double y3 = p3.y;

    double A = 2 * (x2 - x1);
    double B = 2 * (y2 - y1);
    double C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
    double D = 2 * (x3 - x2);
    double E = 2 * (y3 - y2);
    double F = d2 * d2 - d3 * d3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3;

    double det = A * E - B * D;

    result.x = (C * E - B * F) / det;
    result.y = (A * F - C * D) / det;
    
    return result;
}

class DVHopExample
{
public:
  DVHopExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);
  int getBeacons () {return beacons;}
  int getSeed () {return seed;}
  //int getNodesKilled () {return nodesKilled;}
  std::vector<int>*  getBeaconCoords() {return &beaconCoords;}
  std::vector<float> getHopSizes() {return hopSizes;}
/*
  int KillCalculator ();
  double KillTimeCalculator ();
  void NodeKiller ();
*/


private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  bool randomSeed;

  float beaconPercent;

  //\}
  int beacons;
  std::vector<int> beaconCoords;
  std::vector<float> hopSizes;

  uint32_t remainingNodes;

  void NodeShutdown(uint32_t i);
  void RecycleNodes();

  int seed = 25565;

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  //\}



private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
  void CreateBeacons();
  void CalculateHopSize();
  void CalculateCoordinates();

};

void testPrint(){std::cout << "TESTING----------------------------------------" << std::endl;}

int main (int argc, char **argv)
{
  DVHopExample test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");


  test.Run ();
  test.Report (std::cout);



  //Debugging Reports----------------------------------
  //Beacon count
  std::cout << "Beacons: " << test.getBeacons() << std::endl;
  //std::cout << "Nodes Killed: " << test.getNodesKilled() << std::endl;
  //Beacon Locations
  int bcount = 0;
  int itr = 0;
  for(int x = (test.getBeaconCoords()->size())/2; x > 0; x--){
    std::cout << "Beacon " << bcount << " is at " << test.getBeaconCoords()->at(itr) << " " << test.getBeaconCoords()->at(itr+1) << std::endl;
    bcount++;
    itr += 2;
  }
  //Seed #
  if(test.getSeed() == 25565)
    std::cout << "Seed: " << test.getSeed() << ". Not random." << std::endl;
  else
    std::cout << "Seed: " << test.getSeed() << ". Is random." << std::endl;
  return 0;

  std::vector<float> hopSizes = test.getHopSizes();
  for(int x = 0; x < std::ceil(hopSizes.size()); x++){
  	std::cout << hopSizes[x] << std::endl;
  }

}

//-----------------------------------------------------------------------------
DVHopExample::DVHopExample () :
  size (10),
  step (100),
  totalTime (10),
  pcap (true),
  printRoutes (true),
  randomSeed (false),
  beaconPercent (.1)
{
}

bool
DVHopExample::Configure (int argc, char **argv)
{
  // Enable DVHop logs by default. Comment this if too noisy
  //LogComponentEnable("DVHopRoutingProtocol", LOG_LEVEL_ALL);


  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("random", "Randomize seed", randomSeed);
  cmd.AddValue ("beaconPercent", "Set percentage of beacons", beaconPercent);

  cmd.Parse (argc, argv);

  if(randomSeed)
    seed = rand() % (9999-1000 + 1) + 1000;

  SeedManager::SetSeed (seed);
  return true;
}

void
DVHopExample::Run ()
{
  //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  remainingNodes = size;
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  CreateBeacons();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  AnimationInterface anim("animation.xml");

  //Change beacons colors
  int nodeitr = 0;
  Ptr<Node> node;
  for(int x = beacons; x > 0; x--){
    node = nodes.Get(nodeitr);
    anim.UpdateNodeColor(node->GetId(), 0, 0, 255);
    ++nodeitr;
  }

  Simulator::Run();

  //RecycleNodes();

  // --Uncomment the three lines below for critical conditions--
  //Simulator::Stop(Seconds(totalTime));
  //NodeShutdown(5);
  //Simulator::Run();

  CalculateHopSize();

  Simulator::Destroy ();
}

void
DVHopExample::Report (std::ostream &)
{
}

void
DVHopExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      std::cout << "Creating node: "<< os.str ()<< std::endl ;
      Names::Add (os.str (), nodes.Get (i));
    }

  Ptr<UniformRandomVariable> xs = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> ys = CreateObject<UniformRandomVariable> ();
  xs->SetAttribute("Max", DoubleValue(100));
  ys->SetAttribute("Max", DoubleValue(100));

  Ptr<ns3::RandomRectanglePositionAllocator> allocator = CreateObject<ns3::RandomRectanglePositionAllocator> ();
  allocator -> SetX(xs);
  allocator -> SetY(ys);

  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator(allocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel", "");
  mobility.Install (nodes);
}

void
DVHopExample::CreateBeacons ()
{

beacons = 0;
NodeContainer::Iterator i = nodes.Begin();
if(size > 30){
 for(int x = 0; x < std::ceil((size)*beaconPercent); x++){
	Ptr<Ipv4RoutingProtocol> proto = nodes.Get (x)->GetObject<Ipv4>()->GetRoutingProtocol ();
 	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
	dvhop->SetIsBeacon(true);


  //Tell dvhop algorithim real location
  Ptr<Node> node = *i;
  Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
  if (!nodeMB)
    continue;
  Vector actualPosition = nodeMB->GetPosition();
  dvhop->SetPosition(actualPosition.x, actualPosition.y);
  beaconCoords.push_back(actualPosition.x);
  beaconCoords.push_back(actualPosition.y);
	beacons++;
  ++i;
  }
}
else{
  for(int x = 0; x < 3; x++){
    Ptr<Ipv4RoutingProtocol> proto = nodes.Get (x)->GetObject<Ipv4>()->GetRoutingProtocol ();
    Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
    dvhop->SetIsBeacon (true);
    Ptr<Node> node = *i;
    Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
    if (!nodeMB)
      continue;
    Vector actualPosition = nodeMB->GetPosition();
  	dvhop->SetPosition(actualPosition.x, actualPosition.y);
    beaconCoords.push_back(actualPosition.x);
    beaconCoords.push_back(actualPosition.y);
  	beacons++;
    ++i;
  }
}
}

void DVHopExample::CalculateHopSize (){
  std::map<int, float> hopsize;

  std::ofstream outfile;
  outfile.open("data.csv");

  for(int x = 0; x < beacons; x++){
  	Ptr<Ipv4RoutingProtocol> nodeProto = nodes.Get(x) -> GetObject<Ipv4>() -> GetRoutingProtocol ();
	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (nodeProto);
	ns3::dvhop::DistanceTable table = dvhop -> GetDistanceTable();
	std::map<Ipv4Address, dvhop::BeaconInfo> inner = table.Inner();

	double x1 = dvhop -> GetXPosition();
	double y1 = dvhop -> GetYPosition();

	int hops = 0;
	double sum = 0;

	for(const auto& kv : inner){
		ns3::dvhop::BeaconInfo info = kv.second;

		double x2 = info.GetPosition().first;
		double y2 = info.GetPosition().second;

		sum += sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		hops += info.GetHops();
	}

	//Ipv4Address ipv4 = (dvhop -> GetIpv4() -> GetAddress(0, 0)).GetAddress();

	if (hops == 0)
		hopsize[x];
	else
		hopsize[x] = sum/hops;

	std::cout << hopsize[x] << std::endl;

  }


  NodeContainer::Iterator itr = nodes.Begin() + beacons;

  for(int i = beacons; i < std::ceil(size); i++){
  	Ptr<Ipv4RoutingProtocol> proto = nodes.Get(i) -> GetObject<Ipv4>() -> GetRoutingProtocol();
	Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol> (proto);
	ns3::dvhop::DistanceTable table = dvhop -> GetDistanceTable();
	std::map<Ipv4Address, ns3::dvhop::BeaconInfo> inner = table.Inner();


	if(inner.size() >= 3){

		auto a = inner.begin();
      		double xa = a->second.GetPosition().first;
      		double ya = a->second.GetPosition().second;

      		// hops size * hops to node

      		double ra = hopsize[(a -> first).Get() - 167772161] * a->second.GetHops();

      		auto b = inner.begin();
		b++;
      		double xb = b->second.GetPosition().first;
      		double yb = b->second.GetPosition().second;

      		// hops size * hops to node
      		double rb = hopsize[(b -> first).Get() - 167772161] * b->second.GetHops();

      		auto c = inner.begin();
		c++;
		c++;
      		double xc = c->second.GetPosition().first;
      		double yc = c->second.GetPosition().second;

      		// hops size * hops to node
      		double rc = hopsize[(c -> first).Get() - 167772161] * c->second.GetHops();

		/*
		std::cout << (a -> first).Get() << std::endl;
		std::cout << a->second.GetHops() << b->second.GetHops() << c->second.GetHops() << std::endl;
		std::cout << "Beacon1 (x, y): (" << xa << ", " << ya << ") " << "Beacon2 (x, y): (" << xb << ", " << yb << ") " << "Beacon3 (x, y): (" << xc << ", " << yc << ") " <<std::endl;
	        std::cout << "Beacon1 r: " << ra << " Beacon2 r: " << rb << " Beacon3 r: " << rc << std::endl;
		*/

		point aPoint;
		aPoint.x = xa;
		aPoint.y = ya;

		point bPoint;
                bPoint.x = xb;
                bPoint.y = yb;

		point cPoint;
                cPoint.x = xc;
                cPoint.y = yc;

		point nodeGuess = trilateration(aPoint, bPoint, cPoint, ra, rb, rc);

		std::cout << "NODE " << i << " GUESS: "  << nodeGuess.x << " " << nodeGuess.y << std::endl;


    //Get actual node position
     Ptr<Node> node = *itr;
     Ptr<MobilityModel> nodeMB = node->GetObject<MobilityModel> ();
     if (!nodeMB)
       continue;
     Vector actualPosition = nodeMB->GetPosition();
     std::cout << "NODE " << i << " ACTUAL: "  << actualPosition.x << " " << actualPosition.y << std::endl;
     ++itr;

     //Write to csv file

     outfile << i << ", " << nodeGuess.x << ", " << nodeGuess.y << ", " << actualPosition.x << ", " << actualPosition.y << std::endl;
	}
  }

}

void
DVHopExample::CreateDevices ()
{
  WifiMacHelper wifiMac = WifiMacHelper ();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi = WifiHelper ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("aodv"));
    }
}

void
DVHopExample::InstallInternetStack ()
{
  DVHopHelper dvhop;
  // you can configure DVhop attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (dvhop); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  Ptr<OutputStreamWrapper> distStream = Create<OutputStreamWrapper>("dvhop.distances", std::ios::out);
  dvhop.PrintDistanceTableAllAt(Seconds(9), distStream);

  /*
  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dvhop.routes", std::ios::out);
      dvhop.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
   */
}

void DVHopExample::NodeShutdown(uint32_t i){
    std::pair<Ptr<Ipv4>, uint32_t> returnValue = interfaces.Get(i);
    Ptr<Ipv4> ipv4 = returnValue.first;
    uint32_t index = returnValue.second;
    Ptr<Ipv4Interface> iface = ipv4->GetObject<Ipv4L3Protocol> ()->GetInterface(index);
    NS_LOG_UNCOND (Simulator::Now().GetSeconds() << "s Set" << iface->GetAddress(0).GetLocal() << " down.");
    ipv4->SetDown(index);
}

void DVHopExample::RecycleNodes(){
    NodeContainer newNodes = NodeContainer();
    //srand(getSeed());
    //uint32_t randomNode = 4;
    
    for(uint32_t i = 0; i < remainingNodes; i++){
        if(i != 4){
            newNodes.Add(nodes.Get(i));
        }
    }
    nodes = newNodes;
    remainingNodes = remainingNodes - 1;
    NodeShutdown(4);
}



