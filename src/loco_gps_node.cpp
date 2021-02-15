#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "loco_gps_node.h"
//#include <sys/ioctl.h>

#include <unistd.h>
#include <fcntl.h>

#include <string.h> 
#include <string>
#include <iostream> 

#include <arpa/inet.h>
#include <sys/socket.h>
#include <cerrno>

/**
 * UDP In block mode - therefore, cannot destroy node properly
 * Think of using select
 * Think of using own sigint handler
 * http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
 */

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "loco_gps");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  int rateInHz = 10;
  if (argc == 2)
  {
    rateInHz = atoi(argv[1]);  
  }


  ClocoGpsNode locoGpsNode;
  ROS_INFO("\nrateInHz=%d", rateInHz);
  locoGpsNode.start();

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher loco_gps_pub = n.advertise<gps_common::GPSFix>("/loco/gps_data", 1000);

  ros::Rate loop_rate(rateInHz);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    gps_common::GPSFix msgFix = {};
    //gps_common::GPSStatus msgStatus = {};

    // std::stringstream ss;
    // ss << "hello world " << count;
    //msgFix.latitude = 32.4f;
    
    //get the data
    locoGpsNode.getReadData(msgFix);

    ROS_DEBUG("\nlat:%lf lon:%lf alt:%lf\nsat:%d speed:%lf\nhdop:%lf track:%lf\nstatus=%d", msgFix.latitude, msgFix.longitude,msgFix.altitude, msgFix.status.satellites_used, msgFix.speed, msgFix.hdop, msgFix.track,msgFix.status.status);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    loco_gps_pub.publish(msgFix);
    //loco_gps_pub1.publish(msgStatus);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}


ClocoGpsNode::ClocoGpsNode()
{  
}

ClocoGpsNode::~ClocoGpsNode()
{
  m_exit = true;

  if (m_receiveDataThread.joinable())
  {
    m_receiveDataThread.join();
  }
  
  close(m_socket);
}


void ClocoGpsNode::start()
{
  openCommunication("192.168.168.184",20175);
  m_receiveDataThread = std::thread(&ClocoGpsNode::receiveData, this);
}


void ClocoGpsNode::getReadData(gps_common::GPSFix& fixMsg)
{
  std::lock_guard<std::mutex>lock(m_mutex);
  
  //gps fix
  fixMsg.latitude = m_rmcData.Latitude;  //double GGA/RMC
  
  //N/S indicator GGA/RMC
  if ((m_rmcData.NorthOrSouth =='S') || (m_rmcData.NorthOrSouth == 's'))
  {
    fixMsg.latitude *= (-1);
  }
 
  fixMsg.longitude = m_rmcData.Longitude; //double GGA/RMC
  
  //E/W indicator GGA/RMC
  if ((m_rmcData.EastOrWest == 'W') || (m_rmcData.EastOrWest == 'w'))
  {
    fixMsg.longitude *= (-1);
  }

  fixMsg.hdop = m_ggaData.HourDilutionOfPrescision; //GGA 
  fixMsg.altitude = m_ggaData.AntenaAlt; //float64 GGA
  fixMsg.speed = m_rmcData.SpeedOverGroundKnot; //double RMC/VTG - take from RMC
  fixMsg.track = m_rmcData.TrackMadeGoodDeg; //double RMC/VTG - take from RMC


  //gps status
  fixMsg.status.status = m_ggaData.Quality; //Position fix indicator int16 GGA/RMC
  fixMsg.status.satellites_used = m_ggaData.SatNum; //int16  satellite_used  GGA/RMC - take from GGA
}

bool ClocoGpsNode::isSameChksumOk()
{
   unsigned char cksumResult =0;
   cksumResult = calcCksum();

   if (cksumResult == m_rcvBuffer[strlen(m_sentenceStr) -1 ])
   {
     return true;
   }
   return false;
}

void ClocoGpsNode::rotateData()
{
  int count = 0;

  if ( m_byteToRemove > 0 )
  {
    //rotate the buffer
    for (count = m_byteToRemove ;count < m_bufferSize;count++)
    {
      m_rcvBuffer[count - m_byteToRemove] = m_rcvBuffer[count];
    }
    //put 0 in the end
    count--;
    for (;(((count- m_byteToRemove) < m_bufferSize) && (count < 1024));count++)
    {
      m_rcvBuffer[count - m_byteToRemove] = 0;
    }

    m_bufferSize -=  m_byteToRemove;   
  }
}

void ClocoGpsNode::receiveData()
{
  
  while(!m_exit)
  {
    if (receiveFromSensor() != (-1) )
    {      
      while(m_bufferSize)
      {        
        //std::cout<<"\nThe data:\n"<<m_rcvBuffer<< std::endl;
        //parse the msg
        parseData();

        //clear the handle msg
        rotateData();        
      }
    }
  }
}

bool ClocoGpsNode::parseData()
{
    bool parsed = false;
  
    char tmpStr[6] = {};

    m_byteToRemove = 0;



    //m_bufferSize = strlen(m_rcvBuffer);

    if (m_bufferSize < 1)
    {
      return false;
    }

    memset(m_typeStr,0,1024);
    memset(m_sentenceStr,0,1024);

    if(m_rcvBuffer[0] != '$')
    {
      m_byteToRemove = 1;
    }
    else
    { 
      //std::cout<<m_rcvBuffer<<std::endl;
      int index = 0; 
      bool endOfStrFound = false;
      bool newMsgBeforeEnd = false;
      bool checkSum = false;
      int checkSumBeginIndex = 0;
      if ( m_bufferSize >= 6 )
      {
        memcpy(tmpStr,&m_rcvBuffer[1],5);
      }

      while (( index< m_bufferSize) && (!endOfStrFound) && (!newMsgBeforeEnd))
      {
        m_sentenceStr[index] = m_rcvBuffer[index];

        if (index>0 && m_rcvBuffer[index] == '\n' && m_rcvBuffer[index-1]=='\r')
        {
          endOfStrFound= true;     
        }

        if (index>0 && m_rcvBuffer[index]== '*')
        {
          checkSumBeginIndex = index + 1;
        }

        if (index>0 && m_rcvBuffer[index]== '$')
        {
          newMsgBeforeEnd = true;
        }

        index++;
      }

      if (newMsgBeforeEnd)
      {
        m_byteToRemove = index - 1;
      }
      else if((endOfStrFound) && (checkSumBeginIndex > 0))
      {
        //int value;
        //sscanf_s((char *)&m_rcvBuffer[checkSumBeginIndex],"%x:",&value);
        m_byteToRemove = index;

        /*if (isSameChksumOk())
        {
          std::cout<<"\ncksum ok"<<std::endl;*/
          
          parsed = true;        

          if (!strcmp(tmpStr,"GPGGA"))
          {
            handleGPGGA();
          }
          else if (!strcmp(tmpStr,"GPGSA"))
          {
            handleGPGSA();
          }
          else if (!strcmp(tmpStr,"GPGSV"))
          {
            handleGPGSV();
          }
          else if (!strcmp(tmpStr,"GPRMC"))
          {
            handleGPRMC();
          }
          else if (!strcmp(tmpStr,"GPVTG"))
          {
            handleGPVTG();
          }
          else
          {
            ROS_DEBUG("\nUnknown msg name:%s", tmpStr);
          }                  
      }
    }

}

unsigned char ClocoGpsNode::calcCksum()
{
  unsigned char sum = 0;
  unsigned int len = strlen(m_sentenceStr);
  for( unsigned int count = 0;count < len ; count++)
  {
    sum = sum ^ m_sentenceStr[count];
  }
  return sum;
}

void ClocoGpsNode::handleGPGGA()
{
  nmeaGGAData data = {};
  
  parseGPGGA(data);
  setGPGGA(data);
}


void ClocoGpsNode::parseGPGGA(nmeaGGAData& data)
{
  char tmpStr[100] ={};
  char tmpStrIn[100] ={};
  int strLength = (int) strlen(m_sentenceStr);
  int i = 0 , j = 0, pointer = 0;

  for (i = 0; i<strLength; i++)
  {
    tmpStr[j] = m_sentenceStr[i];
    j++;
    if(m_sentenceStr[i] == ',')
    {
        tmpStr[j-1] = 0;
        switch (pointer)
        {
          case 0:
          case 14:
          case 15:
          break;
          
          case 1: //HHMMSS.SS
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
            data.Hour = atoi(tmpStrIn);
            
            memcpy(tmpStrIn,&tmpStr[2],2);
            tmpStrIn[3] = 0;
            data.Minuts = atoi(tmpStrIn);

            memcpy(tmpStrIn,&tmpStr[4],2);
            tmpStrIn[3] = 0;
            data.Seconds = atoi(tmpStrIn);

            memcpy(tmpStrIn,&tmpStr[6],2);
            tmpStrIn[3] = 0;
            data.miliseconds = atoi(tmpStrIn);
          break;

          case 2:
            data.Latitude = atof(tmpStr);
            data.Latitude /= 100.0;

          break;

          case 3:
            data.NorthOrSouth = tmpStr[0];
          break;

          case 4:
            data.Longitude = atof(tmpStr);
            data.Longitude /= 100.0;
          break;

          case 5:
            data.EastOrWest = tmpStr[0];
          break;
 
          case 6:
            data.Quality = atoi(tmpStr);
          break;

          case 7:
            data.SatNum = atoi(tmpStr);
          break;

          case 8:
            data.HourDilutionOfPrescision = atof(tmpStr);
          break;
    
          case 9:
            data.AntenaAlt = atof(tmpStr);
          break;

          case 10:
            data.HighUnits = atoi(tmpStr);
          break;

          case 11:
            data.GeoidalSeparation = atof(tmpStr);
          break;

          case 12:
            data.eoidalSeperationUnits = tmpStr[0];
          break;
  
          case 13:
            data.AgeOfDifferantionalGPSData = atof(tmpStr);
          break;
        
        default:
        break;
        }
        pointer++;
        j=0;
    }
  }
}


void ClocoGpsNode::handleGPGSA ()
{
  nmeaGSAData data = {};
  
  parseGPGSA(data);
  setGPGSA(data);
}

void ClocoGpsNode::parseGPGSA (nmeaGSAData& data)
{
 char tmpStr[100] ={};
  char tmpStrIn[100] ={};
  int strLength = (int) strlen(m_sentenceStr);
  int i = 0 , j = 0, pointer = 0;

  for (i = 0; i<strLength; i++)
  {
    tmpStr[j] = m_sentenceStr[i];
    j++;
    if(m_sentenceStr[i] == ',')
    {
        tmpStr[j-1] = 0;
        switch (pointer)
        {
          case 0:
          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          case 13:
          case 14:
          break;

          case 15:
            data.PDOPinMtr = atof(tmpStr);
          break;

          case 16:
            data.HDOPinMtr = tmpStr[0];
          break;

          case 17:
            data.VDOPinMtr = atof(tmpStr);
          break;          
        
        default:
        break;
        }
        pointer++;
        j=0;
    }
  }
}


void ClocoGpsNode::handleGPGSV ()
{
  nmeaGSVData data = {};
  
  parseGPGSV(data);
  setGPGSV(data);
}

void ClocoGpsNode::parseGPGSV (nmeaGSVData& data)
{
 char tmpStr[100] ={};
  char tmpStrIn[100] ={};
  int strLength = (int) strlen(m_sentenceStr);
  int i = 0 , j = 0, pointer = 0;

  for (i = 0; i<strLength; i++)
  {
    tmpStr[j] = m_sentenceStr[i];
    j++;
    if(m_sentenceStr[i] == ',')
    {
        tmpStr[j-1] = 0;
        switch (pointer)
        {
          case 0:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
          break;
         
          case 1:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
          break;

          case 2:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
          break;

          case 3:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
            data.NuberOfSatellites = atoi(tmpStrIn);
          break;

          case 4:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
          break;

          case 5:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
            data.Pitch = atoi(tmpStrIn);
          break;

          case 6:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[4] = 0;
            data.Yaw = atoi(tmpStrIn);
          break;
            
          case 7:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
          break;

          default:
          break;
        }
        pointer++;
        j=0;
    }
  }
}


void ClocoGpsNode::handleGPRMC ()
{
  nmeaRMCData data = {};
  
  parseGPRMC(data);
  setGPRMC(data);
}

void ClocoGpsNode::parseGPRMC (nmeaRMCData& data)
{
  char tmpStr[100] ={};
  char tmpStrIn[100] ={};
  int strLength = (int) strlen(m_sentenceStr);
  int i = 0 , j = 0, pointer = 0, tempStrLenght = 0;
  char degStr[100] = {};
  char minStr[100] ={};

  for (i = 0; i<strLength; i++)
  {
    tmpStr[j] = m_sentenceStr[i];
    j++;
    if(m_sentenceStr[i] == ',')
    {
        tmpStr[j-1] = 0;
        switch (pointer)
        {
          case 0:
          break;
          
          case 1: //HHMMSS.SS
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
            data.Hour = atoi(tmpStrIn);
            
            memcpy(tmpStrIn,&tmpStr[2],2);
            tmpStrIn[3] = 0;
            data.Minuts = atoi(tmpStrIn);

            memcpy(tmpStrIn,&tmpStr[4],2);
            tmpStrIn[3] = 0;
            data.Seconds = atoi(tmpStrIn);

            memcpy(tmpStrIn,&tmpStr[6],2);
            tmpStrIn[3] = 0;
            
            memcpy(tmpStrIn,&tmpStr[7],2);
            tmpStrIn[3] = 0;
            data.miliseconds = atoi(tmpStrIn);
          break;

          case 2:
            data.NavReceiverWarning = false;
            memcpy(tmpStrIn,tmpStr,1);
            tmpStrIn[1] = 0;
            if (strcmp(tmpStrIn,"V") == 0)
            {
               data.NavReceiverWarning = true;
            }
          break;

          case 3:
            tempStrLenght = (int) strlen(tmpStr);
            memcpy(degStr,tmpStr,2);
            degStr[2] = 0;

            if (tempStrLenght > 2)
            {
              memcpy(minStr,&tmpStr[2],tempStrLenght - 2);
              data.Latitude = atof(degStr) +atof(minStr)/60.0;
            }
          break;
          
          case 4:
          data.NorthOrSouth = tmpStr[0];
          if (data.NorthOrSouth == 'S')
          {
            data.Latitude *= -1.0;
          }
          break;


          case 5:
            tempStrLenght = (int) strlen(tmpStr);
            memcpy(degStr,tmpStr,3);
            degStr[4] = 0;

            if (tempStrLenght > 3)
            {
              memcpy(minStr,&tmpStr[3],tempStrLenght - 3);
              data.Longitude = atof(degStr) +atof(minStr)/60.0;
            }
          break;
          
          case 6:
            data.EastOrWest = tmpStr[0];
            if (data.NorthOrSouth == 'W')
            {
              data.Longitude *= -1.0;
            }
          break;

          case 7:
            data.SpeedOverGroundKnot = atof(tmpStr);
          break;

          case 8:
            data.TrackMadeGoodDeg = atof(tmpStr);
          break;
        
          case 9:
            memcpy(tmpStrIn,tmpStr,2);
            tmpStrIn[3] = 0;
            data.Day = atoi(tmpStrIn);
            
            memcpy(tmpStrIn,&tmpStr[2],2);
            tmpStrIn[3] = 0;
            data.Month = atoi(tmpStrIn);
            
            memcpy(tmpStrIn,&tmpStr[4],2);
            tmpStrIn[3] = 0;
            data.Year = atoi(tmpStrIn) + 2000; 
          break;

          case 10:
            data.MagneticVariationDeg = atof(tmpStr);
          break;
        
        /*SYSTEMTIME St;*/
    
          default:
          break;
        }
        pointer++;
        j=0;
    }
  }
}


void ClocoGpsNode::handleGPVTG ()
{
  nmeaVTGData data = {};
  
  parseGPVTG(data);
  setGPVTG(data);
}

void ClocoGpsNode::parseGPVTG (nmeaVTGData& data)
{
 char tmpStr[100] ={};
  char tmpStrIn[100] ={};
  int strLength = (int) strlen(m_sentenceStr);
  int i = 0 , j = 0, pointer = 0;

  for (i = 0; i<strLength; i++)
  {
    tmpStr[j] = m_sentenceStr[i];
    j++;
    if(m_sentenceStr[i] == ',')
    {
        tmpStr[j-1] = 0;
        switch (pointer)
        {
         
          case 0:
          break;

          case 1:
            data.TrakeDegree1 = atof(tmpStr);
          break;

          case 2:
            data.TrueChar = tmpStr[0];
          break;

          case 3:
            data.TrakeDegree2 = atof(tmpStr);
          break;   

          case 4:
            data.MagneticChar = tmpStr[0];
          break;

          case 5:
            data.SpeedKnots = atof(tmpStr);
          break;  

          case 6:
            data.KnotChar = tmpStr[0];
          break;    
          
          case 7:
            data.SpeedKilometerPerHour = atof(tmpStr);
          break;    
        
        /* char KilometerPerHourChar;*/

          case 8:
          default:
          break;
          }
        pointer++;
        j=0;
    }
  }
}


void ClocoGpsNode::setGPGGA(nmeaGGAData ggaData)
{  
  std::lock_guard<std::mutex>lock(m_mutex);
  m_ggaData = ggaData;
}
void ClocoGpsNode::setGPGSA(nmeaGSAData gsaData)
{
  std::lock_guard<std::mutex>lock(m_mutex);
  m_gsaData = gsaData;
}
void ClocoGpsNode::setGPGSV(nmeaGSVData gsvData)
{
  std::lock_guard<std::mutex>lock(m_mutex);
  m_gsvData = gsvData;
}
void ClocoGpsNode::setGPRMC(nmeaRMCData rmcData)
{
  std::lock_guard<std::mutex>lock(m_mutex);
  m_rmcData = rmcData;
}
void ClocoGpsNode::setGPVTG(nmeaVTGData vtgData)
{
  std::lock_guard<std::mutex>lock(m_mutex);
  m_vtgData = vtgData;
}




/*udp communication*/

bool ClocoGpsNode::openCommunication(std::string ipAddr, int portNumber)
{

  if ((m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) 
	{ 
		printf("\n Socket creation error \n"); 
		return false; 
	} 

  memset(&m_serverAddr,0,sizeof(m_serverAddr));
  memset(&m_otherAddr,0,sizeof(m_otherAddr));

	m_serverAddr.sin_family = AF_INET; 
  m_serverAddr.sin_addr.s_addr = INADDR_ANY;
  m_serverAddr.sin_port = htons(portNumber); 

  if(0 > bind(m_socket, (const struct sockaddr *)&m_serverAddr, sizeof(m_serverAddr)))
  {
    ROS_INFO("\n bind fail. err -%s",std::strerror(errno));
    
    return false;
  }
  return true;
  
}

unsigned int ClocoGpsNode::receiveFromSensor()
{

	socklen_t len = sizeof(m_otherAddr);
  ssize_t readNum;

  readNum = recv(m_socket , (char *)m_rcvBuffer + m_bufferSize, 1024, 0); 
  if (readNum  == -1)
  {
    ROS_INFO("\nReceiveFromSensor fail %s", std::strerror(errno));
  }
  else
  {
    m_bufferSize = strlen (m_rcvBuffer);  
  }

  return readNum;
}

void ClocoGpsNode::sendToSensor(std::string msg)
{
	send(m_socket , msg.c_str(), msg.length() , 0 ); 
}
