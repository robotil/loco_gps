#include <string>
#include <thread>
#include <mutex>

#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string>

#include "gps_common/GPSFix.h"
#include "gps_common/GPSStatus.h"


class ClocoGpsNode {

public:
    ClocoGpsNode();
    ~ClocoGpsNode();
    void start();
    void getReadData(gps_common::GPSFix& fixMsg);

  
private:
  
    struct nmeaGGAData
    {
        int Hour;
        int Minuts;
        int Seconds;
        int miliseconds;
        double Latitude;
        double Longitude;
        char NorthOrSouth;
        char EastOrWest;
        int Quality;
        int SatNum;
        double HourDilutionOfPrescision;
        double AntenaAlt;
        char HighUnits;
        double GeoidalSeparation;
        char eoidalSeperationUnits;
        double AgeOfDifferantionalGPSData;
    };

    struct nmeaRMCData
    {
        int Day;
        int Month;
        int Year;
        int Hour;
        int Minuts;
        int Seconds;
        int miliseconds;
        bool NavReceiverWarning;
        double Latitude;
        double Longitude;
        char NorthOrSouth;
        char EastOrWest;
        double SpeedOverGroundKnot;
        double TrackMadeGoodDeg;
        double MagneticVariationDeg;
        //SYSTEMTIME St; /*FIXME*/
    };


    struct nmeaGSAData
    {
        double PDOPinMtr;
        double HDOPinMtr;
        double VDOPinMtr;        
    };

    struct  nmeaGSVData
    {
        double Yaw;
        double Pitch;
        int NuberOfSatellites;        
    };


    struct  nmeaVTGData
    {
        double TrakeDegree1;
        double TrakeDegree2;
        double SpeedKnots;        
        double SpeedKilometerPerHour;
        char TrueChar;
        char MagneticChar;
        char KnotChar;
        char KilometerPerHourChar;
    };

    enum MsgType
    {
        GPGGA_MSG_TYPE,
        GPGSA_MSG_TYPE,
        GPGSV_MSG_TYPE,
        GPRMC_MSG_TYPE,
        GPVTG_MSG_TYPE
    } ;

    void receiveData();
    bool parseData();
    unsigned char calcCksum();
    bool isSameChksumOk();

    //communication udp
    bool openCommunication(std::string ipAddr,int portNumber);
    unsigned int receiveFromSensor();
    void rotateData();
    void sendToSensor(std::string msg);


    void parseGPGGA(nmeaGGAData& data);
    void parseGPGSA(nmeaGSAData& data);
    void parseGPGSV(nmeaGSVData& data);
    void parseGPRMC(nmeaRMCData& data);
    void parseGPVTG(nmeaVTGData& data);
    
    void handleGPGGA();
    void handleGPGSA();
    void handleGPGSV();
    void handleGPRMC();
    void handleGPVTG();

    void setGPGGA(nmeaGGAData data);
    void setGPGSA(nmeaGSAData data);
    void setGPGSV(nmeaGSVData data);
    void setGPRMC(nmeaRMCData data);
    void setGPVTG(nmeaVTGData data);
    
    std::thread m_receiveDataThread;
    bool m_exit = false;

    //communication
	int m_socket = 0; 
	struct sockaddr_in m_serverAddr; 
    struct sockaddr m_otherAddr; 		
    
	char m_rcvBuffer[1024] = {}; 
    char m_typeStr[1024] = {};
    char m_sentenceStr[1024] = {};
    unsigned int m_bufferSize = 0;
    unsigned int m_byteToRemove = 0;
  


    std::mutex m_mutex;

    nmeaGGAData m_ggaData = {};
    nmeaGSAData m_gsaData = {};
    nmeaGSVData m_gsvData = {};
    nmeaRMCData m_rmcData = {};
    nmeaVTGData m_vtgData = {};     

    gps_common::GPSFix m_msgFixLast = {}; 
    double m_prevTrackMadeGoodDeg = 0.0;


};
