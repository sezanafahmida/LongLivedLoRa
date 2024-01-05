/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/end-device-status.h"
#include "ns3/node-list.h"
#include <algorithm>
#include <ctime>
#include <cmath>


using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 200;     //300? check with examples 
int nGateways = 1;
double radius = 1200;
double simulationTime = 86400;   
bool baseline = false;
int h=1 ; //heuristic num
int cellNum=8; //number of cells
int cellSize=25; //size of each cell
int chanNum=8; //number of channels
double prob =0.5;  //probability of smaller period at a node /probability of offloading node
int CAD_count=0;
// Channel model
bool realisticChannelModel = false;
std::string adrType = "ns3::AdrComponent";
bool adrEnabled = true;
int rtxOver =2;
double budget = 6.5;

int appPeriodSeconds = 2;   //300
int periodsToSimulate = 1;
int transientPeriods = 0;
int totalPackets=0;
NodeContainer endDevices;
std::vector <double> batteryLevels;
// Output control
bool print = true;

struct LoraNode {
int id =0;
double battery = budget;  //joules
int txCount = 0;
bool isLading = false;
bool isOff = false;
double period =0;
double e_x =0;
double t_l=0;
int dr =0;
int rtx=0;
double lading_endTime=0;
double e_lading=0;
double e_o=0;
double e_act=0; //actual energy consumed in offloading
std::vector <LoraNode> assignedOff;
int cell=0;
bool isAssigned= false;
uint8_t txPower = 0;
/*battery lifespan variables**/ 
double bCap = battery; //original battery capacity
double Age = 0; //battery Age,higher->older battery
std::vector<double>SOC; //SOC trace every min
double curE = 0; //energy consumed by transmission (including retransmission attempts) accessed by updateSOC() periodically
std::vector<float> EstHarvestedE; // Estimated Harvested Energy
double curSoC = battery; //SoC level
};

std::vector <LoraNode> nodeInfo;

struct OffNode{
int o_id; //offloading node
int l_id; //assigned lading node id
double tx_power; //tx_power to use for this lading node
};

void configureLading(int l_id);

//return the nnumber of symbols
double GetToA(uint32_t dr){

//std::vector <double> ToA ={71.94,123.39,226.30,452.61,905.22,1646.59}; //collected from https://www.loratools.nl/#/airtime , with payload size =30, preamble=0, bw =125Khz, CR =1 (default)

std::vector<double>ToA = {1810.0,987.0,493.0,267.0,154.0,82.0};                                //{1646.59,905.22,452.61,226.30,123.39,71.94};

/*Airtime for sf 12 1810
Airtime for sf 11 987
Airtime for sf 10 493
Airtime for sf 9 267
Airtime for sf 8 154
Airtime for sf 7 82*/

return ToA[dr]; 

}



double GetCADenergy() {

int sf = 7 ; ///offloading nodes use SF=7 
int bw =125000; //offloading nodes use BW=125

double f1 = std:: pow( 2,sf)+ 32;
f1 = f1/bw;
double p_rx = 11.5;
double p_rcON = 1.5/1000;
double T2 = 4.1/1000; 
double total = f1* (p_rx- p_rcON) + T2*p_rcON; 
//std::cout << "E_CAD " << total/1000 <<;
return total/1000;

}


double GetTxCurrent(uint8_t txpower){

double x1 = 7;
double y1 = 20;
double x2 = unsigned(txpower);
double x3 = 13;
double y3 = 29;
//NS_LOG_DEBUG("x2: " << x2);
double y2 = (((x2 - x1)*(y3-y1)) / (x3-x1))+y1;  //linear interpolation from values in sx1276 datasheet. 

return y2;  

}

double GettxEnergy(uint8_t txpower, uint32_t sf)
{
double ToA = GetToA(sf)/1000;  //convert in seconds
double txCurrent = GetTxCurrent(txpower)/1000; //convert in Amps
//NS_LOG_DEBUG("Energy at sf:" <<sf << ":" << "txPower"<< unsigned(txpower) << "txcurrent" << txCurrent<< " " << ToA*txCurrent*3.3);
return (ToA*txCurrent*3.3);

}

void GetAckTxEnergy (int id,int oldValue,int newValue) {

Ptr <Node> node = ns3::NodeList::GetNode(id);
Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
uint8_t txpower = edLorawanMac1->GetTransmissionPower();
double ToA =41.0/1000;
double txCurrent = GetTxCurrent(1)/1000;
double total= (ToA*txCurrent*3.3);
//NS_LOG_DEBUG("Subtracting " << unsigned(txpower)) ;
//if(id == 46) std::cout << "node 46 " << nodeInfo.at(id).isLading << " "  << nodeInfo.at(id).battery << " " << Simulator::Now().GetSeconds() << "\n";
nodeInfo.at(id).battery -= total;
nodeInfo.at(id).e_act += total;
}

double getTxPower(int l_id, int o_id, Ptr <LoraChannel> channel){
 
 Ptr <Node> receiver = ns3::NodeList::GetNode(l_id);
 Ptr<MobilityModel> r_mobility = receiver->GetObject<MobilityModel> ();
 double txPower = 0;
 double rxPower= -99999; 
 double sensitivity = EndDeviceLoraPhy::sensitivity[0]; //for sf 7

 Ptr <Node> sender = ns3::NodeList::GetNode(o_id);
 //get sender mobility
 Ptr<MobilityModel> s_mobility = sender->GetObject<MobilityModel> ();

 
 while(rxPower<sensitivity){
 if(txPower>=14) break;
 txPower++; //increase txPower 

 rxPower = channel->GetRxPower(txPower,s_mobility,r_mobility); //calc rxPower
 //NS_LOG_DEBUG("rx power at " << txPower << " " << rxPower) ;
 }
 return txPower;
}

void EnableChannel(int chInd,int ind)
{ 
 Ptr <Node> node = ns3::NodeList::GetNode(ind);
 
 Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
 LogicalLoraChannelHelper channelHelper = edMac1->GetLogicalLoraChannelHelper(); 
 
for(int i=0;i<chanNum;i++)            //assuming there are only chanNum channels in the network
  {
  
  if(i!=chInd) {
   channelHelper.DisableChannel(i);  //disable all channels except the assigned one 
   // NS_LOG_DEBUG("Node " << ind << "Disabling channel " << i) ;
   }
  }

}

void ResetChannels (int ind)

{
 Ptr <Node> node = ns3::NodeList::GetNode(ind);
 Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
 LogicalLoraChannelHelper channelHelper = edMac1->GetLogicalLoraChannelHelper(); 
 
  for(int i=0;i<chanNum;i++)            //assuming there are only chanNum channels in the network
  {
   channelHelper.EnableChannel(i);  //enable all channels 
  }

} 




void configureLading(int l_id){
  int index=0;
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j ,index++)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
      LogicalLoraChannelHelper channelHelper = edMac1->GetLogicalLoraChannelHelper();
      Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
      Ptr<EndDeviceLoraPhy> edPhy = phy->GetObject<EndDeviceLoraPhy> ();
      
      //edPhy->SetFrequency(868.1);
      if(index==l_id) 
  
      {
       int cellId = nodeInfo.at(l_id).cell%chanNum;
       EnableChannel(cellId,l_id);
       edPhy->SwitchToStandby();
       edPhy->SetSpreadingFactor(7);
       edLorawanMac1->isLading=true;
       edLorawanMac1->cellId=cellId;
       Ptr<LogicalLoraChannel> lc = channelHelper.GetChannel(cellId);
       edPhy->SetFrequency(lc->GetFrequency());
       nodeInfo.at(l_id).isLading = true;
       //Simulator::Schedule(MilliSeconds(10.1),&configureSleep,l_id);
      
     }
    }

}


void configureConv(int l_id){
  int index=0;
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j ,index++)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
      Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
      
      Ptr<EndDeviceLoraPhy> edPhy = phy->GetObject<EndDeviceLoraPhy> ();
     // edPhy->SetFrequency(868.1);
      if((index==l_id) && nodeInfo.at(l_id).isLading) 
 
      {
       ResetChannels(l_id);
       edPhy->SwitchToSleep();
       edLorawanMac1->isLading=false;      
       nodeInfo.at(l_id).isLading=false;
       double e_cad = GetCADenergy();
       double cad_count = nodeInfo.at(l_id).t_l*1000/8.2;
     //  NS_LOG_DEBUG("CAD energy" << cad_count*e_cad);
       if(l_id == 26) std::cout << "node 26 " << nodeInfo.at(l_id).isLading << " "  << nodeInfo.at(l_id).battery << " " << Simulator::Now().GetSeconds() << "\n";
       nodeInfo.at(l_id).battery -= (e_cad*cad_count);     //compensating for cad
       if(l_id == 26) std::cout << "node 26 " << nodeInfo.at(l_id).isLading << " "  << nodeInfo.at(l_id).battery << " " << Simulator::Now().GetSeconds() << "\n";
      }
    }

}


void configureOff(int o_id, double txPower){
  int index=0;
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j ,index++)
    {
      if(index==o_id) 
      {
       int cellId = nodeInfo.at(o_id).cell%chanNum;
       EnableChannel(cellId,o_id);
       Ptr <Node> node = *j;
       Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
       Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
       nodeInfo.at(o_id).dr= edLorawanMac1->GetDataRate();
       nodeInfo.at(o_id).txPower =edLorawanMac1->GetTransmissionPower();
       edLorawanMac1->SetDataRate(5);
       edLorawanMac1->SetTransmissionPower(txPower);
      }
    }

}

void configureConv_ofl(int o_id, int dr, int txPower){
  int index=0;
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j ,index++)
    {
      if(index==o_id) 
      {
        ResetChannels(o_id);
       Ptr <Node> node = *j;
       Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
       Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
       edLorawanMac1->SetDataRate(dr);
       edLorawanMac1->SetTransmissionPower(txPower);
      }
    }

}

//schedules the assigned offloading nodes for a ladingNode
void scheduleOffloading(Ptr<LoraChannel> channel, int l_id, double startTime) 
{
LoraNode ladingNode = nodeInfo.at(l_id);
for (uint32_t j=0;j<ladingNode.assignedOff.size(); j++)
      {                                               //schedule assigned offloading nodes by calculating the txPower;
        int o_id = ladingNode.assignedOff.at(j).id;
        double txPower = getTxPower(l_id,o_id, channel);
        NS_LOG_DEBUG("node " << o_id  << " starts offloading from time(s) " <<startTime << " on tx Power " << txPower);
        if(!baseline) Simulator::Schedule(Seconds(startTime),&configureOff,o_id,txPower);
      } 
}


/*int findLadingNode(std::vector <int> ladingNodes, int oldId) 
{
  LoraNode oldLadingNode = nodeInfo.at(oldId);
 for(int i=0;i<ladingNodes.size();i++)  //start looking for lading nodes after  
   {
   int id = ladingNodes.at(i);
   if((!nodeInfo.at(id).isAssigned) && (nodeInfo.at(id).assignedOff.id == nodeInfo.at(oldId).assignedOff))
    {
    
     double startTime = Simulator::Now().GetSeconds();
     if(!baseline) Simulator::Schedule(Seconds(startTime),&configureLading,id);
     if(!baseline) Simulator::Schedule(Seconds(oldLadingNode.lading_endTime), &configureConv,id);
     NS_LOG_DEBUG("node " << id << " starts lading from time(s) " << startTime <<" end time(s) " << oldLadingNode.lading_endTime);
    // 
     return id;
    }
  }
  return -1;
}
*/
 
void checkLading( uint32_t id)
{


}

void getTimeonAir(uint32_t id, Ptr<Packet> packet ) {

Ptr <Node> node = ns3::NodeList::GetNode(id);
Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
for(uint32_t dr =0;dr< 6; dr++){
//uint32_t dr = //edLorawanMac1->GetDataRate();
LoraTxParameters params;
params.sf = edMac1->GetSfFromDataRate (dr);  
params.headerDisabled = 0;
params.codingRate = 1;
params.bandwidthHz = edMac1->GetBandwidthFromDataRate (dr);
params.nPreamble = 8;
params.crcEnabled = 1;
params.lowDataRateOptimizationEnabled = 0;
  
Time duration = phy->GetOnAirTime (packet, params);
//return duration.GetSeconds();
NS_LOG_DEBUG("Airtime for sf " << unsigned(params.sf) <<" " << duration.GetMilliSeconds());}
}


double maxlifetime= simulationTime;
std::vector<int> txCount;


void txEnergy(Ptr <LoraChannel> channel, Ptr<const Packet> packet, uint32_t id)
{

  Ptr <Node> node = ns3::NodeList::GetNode(id);
  Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
  Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
  Ptr <EndDeviceLoraPhy> edphy = phy->GetObject<EndDeviceLoraPhy>();
  Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
  Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
  Ptr <PeriodicSender> app = node->GetApplication(0)->GetObject<PeriodicSender>();
  //getTimeonAir(id,packet->Copy());
  uint8_t txpower = edLorawanMac1->GetTransmissionPower();
  uint32_t dr = edLorawanMac1->GetDataRate();
  if(!nodeInfo.at(id).isOff)   //only update if not an offloading node
  {
  nodeInfo.at(id).txPower=txpower;
  nodeInfo.at(id).dr = dr;
  }
  double e_tx = GettxEnergy(txpower,dr);
  txCount.at(id)++;
 // if(id==6) NS_LOG_DEBUG( " node 6 decreasing battery " << e_tx << " " <<txCount.at(id) << " " << nodeInfo.at(id).battery );
  nodeInfo.at(id).battery -= e_tx;//(3.3*20*77.1)/1000000;
  nodeInfo.at(id).curE = e_tx;
  
 /* if(nodeInfo.at(id).isLading)
  {
   if(nodeInfo.at(id).e_act>=nodeInfo.at(id).e_lading)
   {
    if(!baseline) configureConv(id); //stop lading
    std::cout<<"Stopping lading for node " << id << " at " << Simulator::Now().GetSeconds() << "from tx " << std::endl;
    nodeInfo.at(id).isLading=false;
    for (int i=0;i<nodeInfo.at(id).assignedOff.size();i++)
    {
    int o_id = nodeInfo.at(id).assignedOff.at(i).id;
    int txpower = nodeInfo.at(o_id).txPower;
    int dr =nodeInfo.at(o_id).dr;
    if(!baseline) configureConv_ofl(o_id,dr,txpower);
    }
   }
  }*/
 
  if(nodeInfo.at(id).battery <0 ){
  app->SetBattery(true);
  nodeInfo.at(id).battery =0;
  double lifetime = Simulator::Now().GetSeconds();
    if (lifetime<maxlifetime) 
    {
       maxlifetime = lifetime;
       std::cout<< "node " << id << " ran out of energy " << lifetime << "\n";
    }
  }

}

double GetRxEnergy(uint32_t dr){
double toa = GetToA(dr)/1000; 
double e_rx = (toa * 11.5 *3.3)/1000;
return e_rx;
}


void rxEnergy(uint32_t id, Ptr<const Packet> packet)
{
/*if(nodeInfo.at(id).isOff){
nodeInfo.at(id).battery -=(3.3*11.5*12.29)/1000000; }
else if (nodeInfo.at(id).isLading) {
nodeInfo.at(id).battery -=(3.3*11.5*77.1)/1000000;
}*/
 
  Ptr <Node> node = ns3::NodeList::GetNode(id);
  Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
 
  Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
  Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
  Ptr <PeriodicSender> app = node->GetApplication(0)->GetObject<PeriodicSender>();
  uint32_t dr = edLorawanMac1->GetDataRate();
  //uint32_t sf = edphy->GetSpreadingFactor();
  double toa = 41.0/1000; 
  double e_rx = (toa * 11.5 *3.3)/1000; 
  nodeInfo.at(id).battery -=e_rx;
  nodeInfo.at(id).e_act += e_rx;
/*    if(nodeInfo.at(id).isLading)
  {
   if(nodeInfo.at(id).e_act>=nodeInfo.at(id).e_lading)
   {
    if(!baseline) configureConv(id); //stop lading
    std::cout<<"Stopping lading for node " << id << " at " << Simulator::Now().GetSeconds() << "rxenr "  << std::endl;
    nodeInfo.at(id).isLading=false;
    for (int i=0;i<nodeInfo.at(id).assignedOff.size();i++)
    {
    int o_id = nodeInfo.at(id).assignedOff.at(i).id;
    int txpower = nodeInfo.at(o_id).txPower;
    int dr =nodeInfo.at(o_id).dr;
    if(!baseline) configureConv_ofl(o_id,dr,txpower);
    }
   }
  }*/
 
  if(nodeInfo.at(id).battery <0 ){
  app->SetBattery(true);
  nodeInfo.at(id).battery =0;
  double lifetime = Simulator::Now().GetSeconds();
    if (lifetime<maxlifetime) 
    {
       maxlifetime = lifetime;
       std::cout<< "node " << id << " ran out of energy " << lifetime << "\n";
    }
  }
 // if(id == 46) std::cout << "node 46 " << nodeInfo.at(id).isLading << " "  << nodeInfo.at(id).battery << " " << Simulator::Now().GetSeconds() << "\n";
}


void GetRxWindowEnergy(uint32_t id, double oldValue, double newValue){

double total = (newValue*1.6*3.3)/1000;
nodeInfo.at(id).battery -=total;
nodeInfo.at(id).e_act += total;
Ptr <Node> node = ns3::NodeList::GetNode(id);
Ptr <PeriodicSender> app = node->GetApplication(0)->GetObject<PeriodicSender>();
/*  if(nodeInfo.at(id).isLading)
  {
   if(nodeInfo.at(id).e_act>=nodeInfo.at(id).e_lading)
   {
    if(!baseline) configureConv(id); //stop lading
    std::cout<<"Stopping lading for node " << id << " at " << Simulator::Now().GetSeconds() << " rxwnd " << std::endl;
    nodeInfo.at(id).isLading=false;
    for (int i=0;i<nodeInfo.at(id).assignedOff.size();i++)
    {
    int o_id = nodeInfo.at(id).assignedOff.at(i).id;
    int txpower = nodeInfo.at(o_id).txPower;
    int dr =nodeInfo.at(o_id).dr;
    if(!baseline) configureConv_ofl(o_id,dr,txpower);
    }
   }
  }*/
 
  if(nodeInfo.at(id).battery <0 ){
  app->SetBattery(true);
  nodeInfo.at(id).battery =0;
  double lifetime = Simulator::Now().GetSeconds();
    if (lifetime<maxlifetime) 
    {
       maxlifetime = lifetime;
       std::cout<< "node " << id << " ran out of energy " << lifetime << "\n";
    }
  }
}

void gwPacketCount(Ptr<const Packet> packet)
{
totalPackets++;
}


void
packetCounter (uint32_t id,uint8_t reqTx, bool success, Time firstAttempt, Ptr<Packet> packet)
{
   nodeInfo.at(id).rtx += reqTx; 
}


int total_offloaded=0;


/*received an offloaded packet*/
void uplinkCounter (uint32_t id ,int oldvalue, int newvalue){

//double e_cad = GetCADenergy();
Ptr <Node> node = ns3::NodeList::GetNode(id);
Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();

uint32_t dr = edLorawanMac1->GetDataRate();
  //uint32_t sf = edphy->GetSpreadingFactor();
  //double toa = GetToA(dr)/1000; 
double e_rx = GetRxEnergy(dr);//(toa * 11.5 *3.3)/1000; 
nodeInfo.at(id).battery -=e_rx;
nodeInfo.at(id).e_act += e_rx + GettxEnergy(nodeInfo.at(id).txPower, nodeInfo.at(id).dr);

//nodeInfo.at(id).battery-= e_cad;
total_offloaded++;

//NS_LOG_DEBUG("Node " << id << " received total " << newvalue << " offloaded packets" ) ;  

}




std:: vector <int> lostforTx; //(nDevices,0);
std:: vector <int> lostforRx ;//(nDevices,0);
std:: vector <int> lostforSleep;//(nDevices,0);
void 
TxLostCounter(Ptr <const Packet> packet, uint32_t id){

lostforTx.at(id)++;
//NS_LOG_DEBUG("Node " << id << " received total " << newvalue << " offloaded packets" ) ;  

}

void 
RxLostCounter(Ptr < const Packet> packet, uint32_t id ){

lostforRx.at(id)++;
//NS_LOG_DEBUG("Node " << id << " received total " << newvalue << " offloaded packets" ) ;  

}

void 
SleepLostCounter(Ptr <const Packet> packet, uint32_t id){

lostforSleep.at(id)++;
//NS_LOG_DEBUG("Node " << id << " received total " << newvalue << " offloaded packets" ) ;  

}


bool checkPeriod(double t_l,std::vector <LoraNode> assignedOff){
for (uint32_t i=0;i<assignedOff.size();i++){
 int o_id= assignedOff.at(i).id;
 if (t_l<nodeInfo.at(o_id).period) return true;
 }
 return false;
}

/*compares lading node n1 and n2, the node with highest residual energy should be ordered first thus returning true if n1.e_x>n2.e_x*/
//lading nodes should be travesed from the beginning, highest residual energy lading nodes should be considered first
bool compareByEnergy (const LoraNode &n1, const LoraNode &n2)

{
return n1.e_x > n2.e_x;

}


/*compares offloading node n1 and n2, the node with lowest offloading overhead should be ordered first thus returning true if n1.e_0<n2.e_0*/
//offloading nodes should be deleted from the end of the list, (highest offloading overhead nodes should be deleted)
bool compareByOverhead (const LoraNode &n1, const LoraNode &n2)

{
return n1.e_o <  n2.e_o;

}


/*compares offloading node n1 and n2, the node with lowest residual energy should be ordered first thus returning true if n1.e_x<n2.e_x*/
////offloading nodes should be deleted from the end of the list, (highest remaining energy nodes should be deleted)
bool compareByDepletion (const LoraNode &n1, const LoraNode &n2)

{
return n1.e_x < n2.e_x;

}


std::vector <MobilityHelper> mobility_vector;
void selectposition(double x, double y){
  MobilityHelper temp_mobility;
  temp_mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (x),
                                 "Y", DoubleValue (y));
  temp_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility_vector.push_back(temp_mobility);
  
 // NS_LOG_DEBUG("adding positions" << mobility_vector.size());

}


/******new code to add energy harvesting file and battery lifespan estimation*******/


//reads green energy generation data for one year nad stores in a vector

int gsLen=525600; //green energy data length; 365*24*60
std::vector <double>greenSource; // actual green source data container
std::vector <double> readEnergyData()
{
std::ifstream input;
double temp=0;
std::vector <double> trace;
input.open("OneMin_EnergyHarvested.csv");
    if(!input.is_open())
    {
     std::cout << "input file not found. Green energy source not initialized\n";
     std::vector <double> empty_vector(gsLen,0.0);
     return empty_vector;
    }
    while(input>>temp)
    {    
    trace.push_back(temp*7);
    }
    input.close();
return trace;
}



void updateSOC(LoraNode* b) 
{

  int curMin = int(Simulator::Now().GetMinutes());  // b->prevUpdate + int(Simulator::Now().GetMinutes());
  //int tsIndex = floor(curMin/60) ;
 // std::cout<<curMin<<std::endl;
 // b->lastUpdate = b->prevUpdate+curMin;
/*  double greenEnergy =0;
  for (int i=b->lastUpdate;i<curMin && curMin<greenSource.size();i++) 
  {
  greenEnergy+= greenSource.at(i%gsLen);
 // std::cout<<"green energy " <<  i%gsLen << " " << greenSource.at(i%gsLen) << std::endl;
   
  
  } */
  double greenEnergy = 0;//b->EstHarvestedE[curMin];
 // std::cout<< greenEnergy/3000 <<std::endl;
// std::cout<< "Energy harvested for node " << b->id << " in Update SOC() for timeslot " << tsIndex << " " << greenEnergy << "\n";
  Ptr<Node> node = NodeList::GetNode(b->id);
  Ptr <ns3::lorawan::PeriodicSender> ps = node->GetApplication(0)->GetObject<ns3::lorawan::PeriodicSender>();
  double curCap = std::min(b->bCap, b->bCap* (1-b->Age));
  b->curSoC = std::min((b->curSoC + greenEnergy - b->curE),curCap ); 
//  std::cout<< "CurE for node " << b->id <<  " in Update SOC() for timeslot " << tsIndex << " " << b->curE << "\n"; 
//  std::cout<< "CurSOC for node " << b->id <<  " in Update SOC() for timeslot " << tsIndex << " " << b->curSOC << "\n";
  if(b->curSoC <=0) 
  {
  
  //ps->alive = false;
  b->curSoC =0;
 // if(Simulator::Now().GetMinutes() < networkLife) networkLife = Simulator::Now().GetMinutes();
 // std::cout << "Battery " << b->id << "died on min " <<curMin << " Network life " << networkLife << std::endl; 
  
  }
  //else ps->alive =true;*/
  
  b->SOC.push_back(b->curSoC);
//  b->lastUpdate = curMin;
  b->curE =0;
  Simulator::Schedule(Seconds(60), &updateSOC, b);
}

void printTrace(int id)
{

std::stringstream ss;
ss<<"Trace/trace_"<< id << ".csv";
std::ofstream tracefile;
tracefile.open(ss.str());

std::vector<double> tempSOC = nodeInfo[id].SOC;

for(double val:tempSOC)
{
tracefile<< val/nodeInfo[id].bCap << std::endl;
}

}


int main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices",
                "Number of end devices to include in the simulation",
                nDevices);
  cmd.AddValue ("radius",
                "The radius of the area to simulate",
                radius);
  cmd.AddValue ("simulationTime",
                "The time for which to simulate",
                simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("print",
                "Whether or not to print various informations",
                print),
  cmd.AddValue("prob", "probability of offloading node", prob);
  cmd.AddValue("baseline", "baseline" , baseline);
  cmd.AddValue("cellSize", "cellSize", cellSize);
  cmd.AddValue("cellNum", "cellNum", cellNum);
  cmd.AddValue("h", "heuristic", h);
  cmd.Parse (argc, argv);

  // Set up logging
   // LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
   //LogComponentEnable("SimpleEndDeviceLoraPhy",LOG_LEVEL_ALL);
 //LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
 // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
 // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  //  LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceStatus",LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
   //LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
   //LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPacketTracker",LOG_LEVEL_ALL);
  // LogComponentEnable ("AdrComponent", LOG_LEVEL_ALL);

  /***********
   *  Setup  *
   ***********/
   greenSource = readEnergyData();
   
   
   //std::cout << "Green" << greenSource.at(570) << std::endl;
  // Create the time value from the period
  //Time appPeriod = Seconds (appPeriodSeconds);
//  LoraNode temp;
//std::vector <LoraNode> nodeInfo(nDevices, temp); //vector to hold the info about every node 
Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));

//std::vector <MobilityHelper> mobility_vector;



  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (2000.0),
                                 "Y", DoubleValue (2000.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility_vector.push_back(mobility);

 //2nd clusterr
  
  MobilityHelper mobility2;
  mobility2.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (-2000.0),
                                 "Y", DoubleValue (-2000.0));
  mobility2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility_vector.push_back(mobility2);

  MobilityHelper mobility3;
  mobility3.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (-2000.0),
                                 "Y", DoubleValue ( 2000.0));
  mobility3.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility_vector.push_back(mobility3);
  
  MobilityHelper mobility4;
  mobility4.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (  2000.0),
                                 "Y", DoubleValue ( -2000.0));
  mobility4.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  
  mobility_vector.push_back(mobility4);
 
  MobilityHelper mobility5;
  mobility5.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (  3000.0),
                                 "Y", DoubleValue ( 0.0));
  mobility5.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility_vector.push_back(mobility5);

  MobilityHelper mobility6;
  mobility6.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (  -3000.0),
                                 "Y", DoubleValue ( 0.0));
  mobility6.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  mobility_vector.push_back(mobility6);
  
  MobilityHelper mobility7;
  mobility7.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (  -2000.0),
                                 "Y", DoubleValue ( 0.0));
  mobility7.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_vector.push_back(mobility6);


  MobilityHelper mobility8;
  mobility8.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (radius),
                                 "X", DoubleValue (  2000.0),
                                 "Y", DoubleValue ( 0.0));
  mobility8.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_vector.push_back(mobility6);
 

  std::vector <double> xy_coordinates ={3000.0,3000.0, 2500.0,2500.0, 3000.0,0.0, 0.0,3000.0,-3000.0,-3000.0, -3000.0,0.0, 0.0,-3000.0,2500.0,0.0,0.0,2500.0,-2500.0,0.0,0.0,-2500.0,-2500.0,-2500.0};
  
  for (int i=0;i<8;i++){
 // selectposition(xy_coordinates.at(i),xy_coordinates.at(i+1));
  }
 
  LoraNode temp;
  Ptr <RandomVariableStream> budget_rv = CreateObjectWithAttributes<UniformRandomVariable>("Min", DoubleValue (6), "Max", DoubleValue (20));
  
  for (int i=0;i<nDevices;i++){
  temp.id=i;
 // temp.battery = budget_rv->GetInteger();
  
  nodeInfo.push_back(temp);
  txCount.push_back(0);
  lostforTx.push_back(0);
  lostforRx.push_back(0);
  lostforSleep.push_back(0);
  } 
  /*Ptr<ListPositionAllocator> allocator_2 = CreateObject<ListPositionAllocator> ();   //allocates positions from a fixed list 
  allocator_2->Add (Vector(5000,0,0));  //x,y,z coordinates of first node
  allocator_2->Add(Vector(5020,0,0)); 
  allocator_2->Add(Vector(4500,0,0));
  allocator_2->Add(Vector(4550,0,0));
  allocator_2->Add(Vector(4000,0,0));
  allocator_2->Add (Vector (0,0,0));
  mobility.SetPositionAllocator (allocator_2);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");*/
  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  if (realisticChannelModel)
    {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);

      // Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      shadowing->SetNext (buildingLoss);
    }

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename
  //Time t= Minutes(10);
  //helper.EnableSimulationTimePrinting (t);

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();
  nsHelper.EnableAdr (adrEnabled);
  nsHelper.SetAdr (adrType);
  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  //NodeContainer endDevices;
  endDevices.Create (nDevices);
  
  // Assign a mobility model to each node
 // mobility.Install (endDevices);
 int nodeId =0;
 int endId =1000000;
 int cellId=0;
 //std::vector <MobilityHelper> mobility_vector;
 for (NodeContainer::Iterator j = endDevices.Begin(); j!= endDevices.End();j++, nodeId++) {
   Ptr <Node> node = *j;
   if(nodeId>=endId){ 
                cellId+=1; 
                if(cellId>=cellNum) cellId=0; //start from the beginning again
                }
   endId = cellSize*(cellId+1);
   nodeInfo.at(nodeId).cell=cellId;
   MobilityHelper mobility = mobility_vector.at(cellId);
   mobility.Install(node);
  /*
   if(nodeId < 15){ mobility.Install(node);  //cluster 1
   
   }
   else if(nodeId>=15 && nodeId<30) mobility2.Install(node);            //cluster 2
   else if(nodeId>=30 && nodeId<45) mobility3.Install(node);
   else if(nodeId>=45 && nodeId<60) mobility4.Install(node);
   else if(nodeId>=60 && nodeId<75) mobility5.Install(node);
   else if(nodeId>=75 && nodeId<=90) mobility6.Install(node);*/
  }

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
     NS_LOG_INFO("Node " << (*j)->GetId() << " position " << position.x << " , "<< position.y <<" , "<< position.z <<" "<<nodeInfo.at((*j)->GetId()).cell);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED);
  helper.Install (phyHelper, macHelper, endDevices);



  

  // Now end devices are connected to the channel



  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc) */

  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);


  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************
   *  Handle buildings  *
   **********************/

  double xLength = 130;
  double deltaX = 32;
  double yLength = 64;
  double deltaY = 17;
  int gridWidth = 2 * radius / (xLength + deltaX);
  int gridHeight = 2 * radius / (yLength + deltaY);
  if (realisticChannelModel == false)
    {
      gridWidth = 0;
      gridHeight = 0;
    }
  Ptr<GridBuildingAllocator> gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  gridBuildingAllocator->SetAttribute ("MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  gridBuildingAllocator->SetAttribute ("MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);
  BuildingsHelper::MakeMobilityModelConsistent ();

  // Print the buildings
  if (print)
    {
      std::ofstream myfile;
      myfile.open ("buildings.txt");
      std::vector<Ptr<Building> >::const_iterator it;
      int j = 1;
      for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
        {
          Box boundaries = (*it)->GetBoundaries ();
          myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
        }
      myfile.close ();

    }

  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  std::vector <int> sfQuantity= macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
  
  //for(uint32_t i=0;i<sfQuantity.size();i++) {std::cout<< sfQuantity.at(i);}
  //NS_LOG_DEBUG(" sf ");
 //save the spreading factors for reverting to conventional mode 
 int ind=0; 
// int startId =0;
 endId =1000000;
 cellId=0;
 for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j,++ind)
    {
  if(ind>=endId){ 
                cellId+=1; 
                if(cellId>=cellNum) cellId=0; //start from the beginning again
                }
 // startId= cellSize*cellId;
  endId = cellSize*(cellId+1);
  Ptr <Node> node = *j;
  Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
  Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
  Ptr<LoraPhy> phy = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ();
  Ptr <EndDeviceLoraPhy> edphy = phy->GetObject<EndDeviceLoraPhy>();
  //NS_LOG_DEBUG("Max transmission for node " << ind << " is " << unsigned(edLorawanMac1->GetMaxNumberOfTransmissions() ));
  
  nodeInfo.at(ind).dr = edLorawanMac1->GetDataRate();
  nodeInfo.at(ind).txPower = edLorawanMac1->GetTransmissionPower();
  nodeInfo.at(ind).cell= cellId;
  ResetChannels(ind);
  
 // NS_LOG_DEBUG ("Node " << ind << " cell id " << cellId);
 // EnableChannel(cellId,ind);
  }
  


  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/
  
  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  Ptr <RandomVariableStream> rv = CreateObject<UniformRandomVariable> ();
  Ptr <RandomVariableStream> interval_rv = CreateObjectWithAttributes<UniformRandomVariable>("Min", DoubleValue (5), "Max", DoubleValue (10));
  Ptr <RandomVariableStream> small_period = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (100), "Max", DoubleValue (200));
  Ptr <RandomVariableStream> large_period = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (1000), "Max", DoubleValue (2000)); 
 // Ptr <RandomVariableStream> large_period = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (200), "Max", DoubleValue (400));
 //OneShotSenderHelper appHelper = OneShotSenderHelper();
  appHelper.SetPacketSize (30);
  int index =0;
  ApplicationContainer appContainer;
  for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j ,index++)
  {
 // appHelper.SetPeriod(Seconds(5+index*5));
 // int chosenPeriod = large_period->GetInteger();
 // nodeInfo.at(index).period =  chosenPeriod;
 // nodeInfo.at(index).txCount = simulationTime/chosenPeriod;
 // appHelper.SetPeriod(Seconds(chosenPeriod));
  if(rv->GetValue() < prob) { 
  appHelper.SetPeriod(Seconds(small_period->GetInteger()));
  nodeInfo.at(index).period =  appHelper.GetPeriod().GetSeconds();               
  nodeInfo.at(index).txCount = appStopTime.GetSeconds()/nodeInfo.at(index).period ;
  nodeInfo.at(index).battery = nodeInfo.at(index).txCount*GettxEnergy(7,3);  
  }
  else 
  {
  appHelper.SetPeriod(Seconds(large_period->GetInteger()));
  nodeInfo.at(index).period =  appHelper.GetPeriod().GetSeconds();               
  nodeInfo.at(index).txCount = appStopTime.GetSeconds()/nodeInfo.at(index).period ;
  nodeInfo.at(index).battery = 8*nodeInfo.at(index).txCount*GettxEnergy(7,3);
  }
  //appHelper.SetIntervalRandomVariable(interval_rv);
  
  appContainer.Add(appHelper.Install (*j));
  
  std::cout<<"Period for node " << nodeInfo.at(index).id <<" set as " << nodeInfo.at(index).period  << " Seconds "<< " total Tx " <<  nodeInfo.at(index).txCount << " battery " << nodeInfo.at(index).battery << "\n"  ;                          
  }

  //initialize the green energy source

  for (int i=0;i<nDevices;i++)
  {
  int totalMin = simulationTime/60; 
  
  for(int curMin=0;curMin<totalMin;curMin++)
    {
    nodeInfo[i].EstHarvestedE.push_back(greenSource[curMin]);
    }
  }
  
  

  
 //start if
 //calculate the lading and offloading nodes set
 double e_tx; //transmission energy;
 double e1=0;  //total estimated energy consumption
 std::vector <LoraNode> ladingNodes;   //set of lading nodes;
 std::vector <LoraNode> off;           //set of offloading nodes;

  for (int i =0;i<nDevices;i++)     
  {
  Ptr <Node> node = ns3::NodeList::GetNode(i);
  Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
  Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
  Ptr <EndDeviceLoraPhy> edphy = phy->GetObject<EndDeviceLoraPhy>();
  Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
  Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
  uint8_t txpower = edLorawanMac1->GetTransmissionPower();
  uint32_t dr = edLorawanMac1->GetDataRate();
  //NS_LOG_DEBUG (" node " << i << " dr  in conventional mode " << unsigned(dr));
  e_tx = GettxEnergy(txpower,dr);
  
  e1= nodeInfo.at(i).txCount*rtxOver* e_tx;   //retransmission overhead 8

  NS_LOG_DEBUG("Node " << i << " tx_count "<< nodeInfo.at(i).txCount << " e_tx "<< e_tx << " e_1 " << e1 << " battery " << nodeInfo.at(i).battery );
  if (e1< (nodeInfo.at(i).battery)){
     //nodeInfo.at(i).isLading =true;
     nodeInfo.at(i).e_x = nodeInfo.at(i).battery - e1 -1 ;  //leaving some safe space
     //std::cout << "e_x" << nodeInfo.at(i).e_x <<std::endl;
     ladingNodes.push_back(nodeInfo.at(i)); 
  }
  else  {
  //nodeInfo.at(i).isLading =false;
  nodeInfo.at(i).e_x = e1 - nodeInfo.at(i).battery; //energy needed by depleting node
  nodeInfo.at(i).isOff = true;
  off.push_back(nodeInfo.at(i));
  }
 }


  //sort the lading nodes in order of e_x

 std::sort(ladingNodes.begin(),ladingNodes.end(),compareByEnergy);

  //NS_LOG_DEBUG("lading nodes: " );
 // for(uint32_t h =0;h<ladingNodes.size();h++) { NS_LOG_DEBUG(ladingNodes.at(h));}
  

  //assign offloading nodes to lading nodes 
  
  std::vector <LoraNode> tempOff;  //temporary place holder for offloading nodes
  std::vector <int>newtempOff;
  std::vector <OffNode> offloading;  
  double e_x;                       //extra energy 
  double e_cad;                    //cad energy
  double e_for;                     ///forwarding energy
  double e_total = 0;               //total energy 
  double t_l=0;                     //time in lading mode
  
  for (uint32_t i=0; i<ladingNodes.size() ; i++)
  {

  tempOff= off;  //assign all;
  t_l=0;
  
  //int l_id = ladingNodes.at(i);  //get the ladingnode id
  //calculate T_l
  LoraNode ladingNode = ladingNodes.at(i);
  int l_id = ladingNode.id;
  e_x = nodeInfo.at(l_id).e_x;   
  e_cad =  GetCADenergy()/(8.2/1000);//(3.3*11.5*77.1)/1000000;
  e_total = 0;
  for (auto it =tempOff.begin(); it!=tempOff.end();it++)
    {
     int id = it->id;           //offloading node id
     double txPower = getTxPower(l_id,id, channel);
     double e_o = nodeInfo.at(id).txCount*GettxEnergy(txPower,5);   // overhead for offloading node
     it->e_o = e_o;
     //NS_LOG_DEBUG("Calculated offloading overhead for node " <<l_id <<" and " <<id << " is " << nodeInfo.at(id).txCount*2*e_o);
     
     if((txPower>10)||(nodeInfo.at(id).cell != nodeInfo.at(l_id).cell)) 
    {
     tempOff.erase(it--);  //using txpower >10 is not beneficial, remove these nodes
     //NS_LOG_DEBUG("removing off node " << id<< "from lading node " << l_id << " with txPower " <<txPower );
     }
  }
  if (h==1 || h==0) 
  {std::sort(tempOff.begin(),tempOff.end(),compareByOverhead);}
  else if (h==2) 
  {
  std::sort(tempOff.begin(),tempOff.end(),compareByDepletion);
  }
  for (uint32_t j =0; j<tempOff.size() && !tempOff.empty() ;j++){
  int id = tempOff.at(j).id;
  //NS_LOG_DEBUG("id" << id);
  double p = nodeInfo.at(id).period;
  int dr =nodeInfo.at(l_id).dr;  //lading nodes sf
  double txPower = nodeInfo.at(l_id).txPower;        //lading node uses highest tx power for now
  e_for = (GettxEnergy(txPower,dr)+GetRxEnergy(dr));
  e_total += e_for/ p;
   
  }
  e_total += e_cad;
  //e_x -= e_cad;
  t_l = e_x / e_total;
  //std::cout<< "T_L" << t_l<< std::endl;
  //int myindex=0;
  while(checkPeriod(t_l, tempOff) && tempOff.size()>0)
  {
  //std::cout << "removing node " << tempOff.at(myindex).id << " " << tempOff.size() << "\n";
  tempOff.erase(tempOff.end()-1);  //remove last element which has the highest offloading overhead if using the compareByOverhead function
 // tempOff.erase(tempOff.begin());
  //NS_LOG_DEBUG("removing node with t_L" <<t_l );
  std::cout<< "removed node \n";
  std::cout<< "size " << tempOff.size();
 // myindex+=1;
  e_total=0; //reset e_total
 
   for (uint32_t j =0; j<tempOff.size();j++){
    
    int id = tempOff.at(j).id;
    
    double p = nodeInfo.at(id).period;
  
    e_total += e_for/ p;
   //std:: cout << " id "<<id <<" "<< p << " "<< e_total<< std::endl;
    }
   e_total += e_cad;
  
  t_l = e_x / e_total;
  }
  nodeInfo.at(l_id).t_l = std::min(t_l, 8500.00);
  nodeInfo.at(l_id).e_lading =e_total;
  //if(l_id ==2) { 
  double e_lading =0;
  int n_total=0;
  for (uint32_t k= 0;k<tempOff.size();k++){
  int n= t_l/nodeInfo.at(tempOff.at(k).id).period;
  n_total+= n;
  e_lading += n*e_for;
  
  // }
 // NS_LOG_DEBUG("lading overhead " << e_lading << " e_for " << e_for << " n " << n_total << " n_c " << nodeInfo.at(l_id).txCount );

   }
  nodeInfo.at(l_id).assignedOff= tempOff;
  } //endwhile
  
  

  
 
  




//calculate txpower for offloading modes and then schedule lading and offloading modes , this needs to be dynamic



 //double remTime=simulationTime;
 //double startTime =0;

 //for( int cid =0; cid< cellNum; cid++) 
//{ 
  //  int startId = cid*cellSize;
 //   int endId =cellSize*(cid+1);
    double remTime=simulationTime;
    double startTime =0;
    double endTime =0;
 //NS_LOG_DEBUG( "start Id " <<startId << " end Id " << endId);
    for (uint32_t i =0;i<ladingNodes.size();i++)
    {
      int l_id= ladingNodes.at(i).id;
      LoraNode ladingNode = nodeInfo.at(l_id); 
   //   if (l_id>=startId && l_id <endId) 
      //   {
          std::cout << "HERE" << std::endl;
          if(ladingNode.assignedOff.empty()) break;                                                                  
          
            if(ladingNode.t_l<remTime)
            {
              std::cout<< "node " << l_id << " starts lading from time(s) " << startTime <<" end time(s) " << startTime+ladingNode.t_l << std::endl;   //schedule lading nodes
              if(!baseline) Simulator::Schedule(Seconds(startTime),&configureLading,l_id);
              if(!baseline) Simulator::Schedule(Seconds(startTime+ladingNode.t_l), &configureConv,l_id);
              nodeInfo.at(l_id).lading_endTime = startTime+ladingNode.t_l;
              remTime-=(ladingNode.t_l);
              //startTime= startTime+ladingNode.t_l;
              endTime = startTime+ladingNode.t_l;
              
              nodeInfo.at(l_id).isAssigned= true;
            }
            else
            {
              std::cout<<"node " << l_id << " starts lading from time(s) " << startTime <<" end time(s) " << startTime+ladingNode.t_l <<std::endl;   //schedule lading nodes
              if(!baseline) Simulator::Schedule(Seconds(startTime),&configureLading,l_id);
              nodeInfo.at(l_id).lading_endTime = startTime+ladingNode.t_l;
              
              nodeInfo.at(l_id).isAssigned= true;
              endTime = startTime+ladingNode.t_l;
              remTime=0;
              break;  //break out of the loop as we dont need anymore lading nodes;
            }
           
           for (uint32_t j=0;j<ladingNode.assignedOff.size(); j++)
           {                  //schedule assigned offloading nodes by calculating the txPower;
              int o_id = ladingNode.assignedOff.at(j).id;
              double txPower = getTxPower(l_id,o_id, channel);
              std::cout << "node " << o_id  << " starts offloading from time(s) " <<startTime << " on tx Power " << txPower << "\n";
              if(!baseline) Simulator::Schedule(Seconds(startTime),&configureOff,o_id,txPower);
              if(!baseline) Simulator::Schedule(Seconds(startTime+ladingNode.t_l), &configureConv_ofl,o_id,nodeInfo.at(o_id).dr,nodeInfo.at(o_id).txPower);
            } 
           
            startTime= startTime+ladingNode.t_l;
 
        
        //  }
        //
 
       }

//}


//}//end if

  
  //enabling confirmed messages for all nodes 
index=0;
for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j, ++index)
    {
  Ptr <Node> node = *j;
  Ptr<LorawanMac> edMac1 = node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
  Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
  //if(nodeInfo.at(index).isOff) 
  edLorawanMac1->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
   } 

    // Connect trace sources
   for (NodeContainer::Iterator j = endDevices.Begin ();
       j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      Ptr <EndDeviceLoraPhy> edphy = phy->GetObject<EndDeviceLoraPhy>();
      Ptr <EndDeviceLoraPhy> sphy = edphy->GetObject<SimpleEndDeviceLoraPhy>();
      Ptr<LorawanMac> edMac1 = loraNetDevice->GetMac ();
      Ptr<EndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<EndDeviceLorawanMac> ();
      edMac1->TraceConnectWithoutContext("ReceivedPacket",MakeBoundCallback(&rxEnergy, node->GetId()));
      edLorawanMac1->TraceConnectWithoutContext("RequiredTransmissions", MakeBoundCallback(&packetCounter, node->GetId()));
      edLorawanMac1->TraceConnectWithoutContext("UplinkCount", MakeBoundCallback(&uplinkCounter, node->GetId()));
      edLorawanMac1->TraceConnectWithoutContext("sentAck", MakeBoundCallback(&GetAckTxEnergy, node->GetId()));
      edLorawanMac1->TraceConnectWithoutContext("closeRx1", MakeBoundCallback(&GetRxWindowEnergy, node->GetId()));
      edLorawanMac1->TraceConnectWithoutContext("closeRx2", MakeBoundCallback(&GetRxWindowEnergy, node->GetId()));
     // uint8_t txpower = edLorawanMac1->GetTransmissionPower();
    //  uint32_t sf = edphy->GetSpreadingFactor();
      phy->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&txEnergy, channel));
     // phy->TraceConnectWithoutContext("PhyRxBegin", MakeBoundCallback(&rxEnergy, node->GetId()));
      
      phy->TraceConnectWithoutContext("LostPacketBecauseWrongSpreadingFactor", MakeCallback(&TxLostCounter));
      sphy->TraceConnectWithoutContext("LostPacketBecauseWrongFrequency", MakeCallback(&RxLostCounter));
     // sphy->TraceConnectWithoutContext("SleepLost", MakeBoundCallback(&SleepLostCounter, node->GetId()));
    }

     for (NodeContainer::Iterator j = gateways.Begin ();
       j != gateways.End (); ++j)
     {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LorawanMac> gwMac1 = loraNetDevice->GetMac ();
      Ptr<GatewayLorawanMac> gwLorawanMac1 = gwMac1->GetObject<GatewayLorawanMac> ();
      gwLorawanMac1->TraceConnectWithoutContext("ReceivedPacket" , MakeCallback(&gwPacketCount));   
       }
  
  

    NS_LOG_DEBUG ("Completed configuration");
   // GetCADenergy();
  
  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  ////////////////
  // Simulation //
  ////////////////
  /* for(int i=0;i<nDevices;i++)
  {
  Simulator::Schedule(Seconds(60),&updateSOC, &nodeInfo[i]);
  }
  */
  Simulator::Stop (appStopTime + Hours (1));

  //NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  //NS_LOG_INFO ("Computing performance metrics...");

  LoraPacketTracker& tracker = helper.GetPacketTracker ();
  std::cout << " total: " << tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours (1)) << std::endl;
  std::cout<< "ack " <<tracker.CountMacPacketsGloballyCpsr(Seconds(0), appStopTime + Hours (1)) << std::endl;

  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
  {std::cout << tracker.PrintPhyPacketsPerGw(Seconds(0), appStopTime + Hours(1) , (*j)->GetId()) <<std::endl;}
  
  std:: vector <int> phyPacketsGW = tracker.CountPhyPacketsPerGw(Seconds(0), appStopTime + Hours(1) , gateways.Get(0)->GetId()); 

  
  NS_LOG_DEBUG("Offloading nodes");
  //for (auto it = off.begin();it!=off.end();it++) NS_LOG_DEBUG( *it);
  
  ///NS_LOG_DEBUG("lading nodes:) ;
 // for (auto it = ladingnodes.begin();it!=ladingnodes.end();it++) NS_LOG_DEBUG( *it);
   std::vector <double> received_delay = tracker.CalculateLatency(Seconds(0), appStopTime + Hours (1),nDevices);
   std::ofstream logfile;
   logfile.open ("ICNP_logs/extension_test.csv",std::ios_base::app);
  for (int i =0;i<nDevices;i++){
  std::cout <<"Node " <<i << " " << nodeInfo.at(i).battery <<" "<< nodeInfo.at(i).txCount  <<" " <<nodeInfo.at(i).isLading  << " " << nodeInfo.at(i).t_l << " " <<nodeInfo.at(i).e_x << " "  <<nodeInfo.at(i).e_lading<<std::endl;
  std::cout<< "Assigned Offloading nodes \n" ;
  for (uint32_t j=0;j<nodeInfo.at(i).assignedOff.size();j++){ std::cout << " node "<< nodeInfo.at(i).assignedOff.at(j).id << " ";}
  std::cout << "\n";
  }
   
double total_delay =0;  
double avg_delay=0;
int count=0;
for (int i=0;i<nDevices;i++) {
   if(i==1)NS_LOG_DEBUG( "node " << i << "wrong frequency: "<< lostforRx.at(i) << " wrong sf " << lostforTx.at(i));
   if(received_delay.at(i)>0)  count++;
  // NS_LOG_DEBUG( "node " << i << " delay " << received_delay.at(i) );
   total_delay += received_delay.at(i);
   
  }
avg_delay= total_delay/count;

 
logfile << h << ", "<< cellNum << "," <<cellSize <<","<<chanNum << ","<<nDevices <<"," <<simulationTime << " , " << prob <<" , " << off.size() << "," << total_offloaded <<"," << maxlifetime << " ,"<<tracker.total_received << " , " << tracker.CountMacPackets(Seconds(0), appStopTime + Hours (1)) <<" , " << phyPacketsGW.at(1) <<" , "<<avg_delay <<std::endl;
  


std::cout << " heuristic " <<h << " numdevices, "<<nDevices << " p " << prob <<" ,off " << off.size() << " ,totalOff " << total_offloaded <<" ,maxLifetime " << maxlifetime << " ,total packets "<< tracker.total_received  << " , " <<tracker.CountMacPackets(Seconds(0), appStopTime + Hours (1)) << " , "<< tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours (1)) << phyPacketsGW.at(1) << " , " << avg_delay <<std::endl; 

for (int i=0;i<nDevices;i++) 
{NS_LOG_DEBUG("Node " << i << " transmitted " << txCount.at(i) << " times " << " energy " << GettxEnergy(nodeInfo.at(i).txPower,nodeInfo.at(i).dr) << " attempts " << nodeInfo.at(i).rtx << "  estimated " << nodeInfo.at(i).txCount << "sf" << nodeInfo.at(i).dr) ;}

/*for(int i =0;i<nDevices;i++){

 printTrace(i);
}*/



  return 0;
}
