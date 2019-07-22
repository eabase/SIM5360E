/**
 * @file       TinyGsmClientSIM5360.h
 * @author     eabase
 * @license    LGPL-3.0
 * @copyright  Copyleft (c) 2019 eabase
 * @date       2019-07-22
 */

#ifndef TinyGsmClientSIM5360_h
#define TinyGsmClientSIM5360_h

//#define TINY_GSM_DEBUG Serial
//#define TINY_GSM_USE_HEX

#if !defined(TINY_GSM_RX_BUFFER)
    #define TINY_GSM_RX_BUFFER  64
#endif

#define TINY_GSM_MUX_COUNT      10

//-----------------------------------------------------
//  Dependencies...
//-----------------------------------------------------
#include <TinyGsmCommon.h>
// ToDo: 
//#include <Arduino.h>
// extern HardwareSerial xbSerial;
//void xbPurge() {    xbSerial.flush();   }

//-----------------------------------------------------
//  Useful Constants
//-----------------------------------------------------
#define GSM_NL "\r\n"
// We do this to save memory space otherwise used up by common strings.
static const char GSM_OK[]    TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;

enum SimStatus {
    SIM_ERROR   = 0,
    SIM_READY   = 1,
    SIM_LOCKED  = 2,
};

enum RegStatus {
    REG_UNREGISTERED = 0,
    REG_SEARCHING    = 2,
    REG_DENIED       = 3,
    REG_OK_HOME      = 1,
    REG_OK_ROAMING   = 5,
    REG_UNKNOWN      = 4,
};

enum TinyGSMDateTimeFormat {
    DATE_FULL = 0,
    DATE_TIME = 1,
    DATE_DATE = 2
};


//=====================================================================
//  CLASS - START
//=====================================================================
class TinyGsmSim5360 {
public:

class GsmClient : public Client {
    friend class TinyGsmSim5360;
    typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
    GsmClient() {}
    GsmClient(TinyGsmSim5360& modem, uint8_t mux = 1) {
       init(&modem, mux);
    }

    virtual ~GsmClient(){}

    bool init(TinyGsmSim5360* modem, uint8_t mux = 1) {
        this->at       = modem;
        this->mux      = mux;
        sock_available = 0;
        prev_check     = 0;
        sock_connected = false;
        got_data       = false;
        at->sockets[mux] = this;
        return true;
    }

public:
    virtual int connect(const char *host, uint16_t port, int timeout_s) {
        stop();
        TINY_GSM_YIELD();
        rx.clear();
        sock_connected = at->modemConnect(host, port, mux, false, timeout_s);
        return sock_connected;
    }

    TINY_GSM_CLIENT_CONNECT_OVERLOADS()

    virtual void stop(uint32_t maxWaitMs) {
        TINY_GSM_CLIENT_DUMP_MODEM_BUFFER()
        at->sendAT(GF("+CIPCLOSE="), mux);
        sock_connected = false;
        at->waitResponse();
    }

    virtual void stop() { stop(15000L); }

    TINY_GSM_CLIENT_WRITE()
    TINY_GSM_CLIENT_AVAILABLE_WITH_BUFFER_CHECK()
    TINY_GSM_CLIENT_READ_WITH_BUFFER_CHECK()
    TINY_GSM_CLIENT_PEEK_FLUSH_CONNECTED()

    //-----------------------------------------------------------------
    //  Extended API
    //-----------------------------------------------------------------
    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

private:
    TinyGsmSim5360* at;
    uint8_t         mux;
    uint16_t        sock_available;
    uint32_t        prev_check;
    bool            sock_connected;
    bool            got_data;
    RxFifo          rx;
};

public:

    TinyGsmSim5360(Stream& stream) : stream(stream) {
       memset(sockets, 0, sizeof(sockets));
    }

    virtual ~TinyGsmSim5360(){}

    //-----------------------------------------------------------------
    //  Basic functions
    //-----------------------------------------------------------------
    bool begin(const char* pin = NULL) {
       return init(pin);
    }

    bool init(const char* pin = NULL) {
        DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
        if (!testAT()) {
            return false;
        }
        sendAT(GF("E0"));   // Echo Off
        if (waitResponse() != 1) {
            return false;
        }
        DBG(GF("### Modem:"), getModemName());
        getSimStatus();
        return true;
    }

    String getModemName() {
        String name =  "SIMCom SIM5360";
        sendAT(GF("+CGMM"));
        String res2;
        if (waitResponse(1000L, res2) != 1) {
            return name;
        }
        res2.replace(GSM_NL "OK" GSM_NL, "");
        res2.replace("_", " ");
        res2.trim();
        name = res2;
        //DBG("### Modem:", name);
        return name;
    }

    TINY_GSM_MODEM_SET_BAUD_IPR()
    TINY_GSM_MODEM_TEST_AT()
    TINY_GSM_MODEM_MAINTAIN_CHECK_SOCKS()

    bool factoryDefault() {  
        // ToDo: See others 
        return false;
    }

    TINY_GSM_MODEM_GET_INFO_ATI()

    bool hasSSL()  { return false; }   // ToDo:  Module supports SSL, but not yet implemented
    bool hasWifi() { return false; }   
    bool hasGPRS() { return true; }

    //-----------------------------------------------------------------
    //  Power functions
    //-----------------------------------------------------------------
    bool restart() {
        if (!testAT()) {
            return false;
        }
        Serial.println("=====> We are supposed to AT+REBOOT, but I refuse!");
        //DBG("=====> We are supposed to AT+REBOOT, but I refuse!");
    /*    sendAT(GF("+REBOOT"));
        if (waitResponse(10000L) != 1) {
            return false;
        }
    */
        delay(3000L);  //TODO:  Test this delay
        return init();
    }

    bool poweroff() {
        sendAT(GF("+CPOF"));
        return waitResponse() == 1;
    }

    bool radioOff() {
        sendAT(GF("+CFUN=4"));
        if (waitResponse(10000L) != 1) {
            return false;
        }
        delay(3000);
        return true;
    }

    bool sleepEnable(bool enable = true) {
        sendAT(GF("+CSCLK="), enable);
        return waitResponse() == 1;
    }

    //-----------------------------------------------------------------
    //  SIM card functions
    //-----------------------------------------------------------------
    TINY_GSM_MODEM_SIM_UNLOCK_CPIN()

    // Gets the CCID of a sim card via AT+CCID
    String getSimCCID() {
        sendAT(GF("+CICCID"));
        if (waitResponse(GF(GSM_NL "+ICCID:")) != 1) {
            return "";
        }
        String res = stream.readStringUntil('\n');
        waitResponse();
        res.trim();
        return res;
    }

    TINY_GSM_MODEM_GET_IMEI_GSN()

    SimStatus getSimStatus(unsigned long timeout_ms = 10000L) {
        for (unsigned long start = millis(); millis() - start < timeout_ms; ) {
            sendAT(GF("+CPIN?"));
            if (waitResponse(GF(GSM_NL "+CPIN:")) != 1) {
                delay(1000);
                continue;
            }
            int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"));
            waitResponse();
            switch (status) {
                case 2:
                case 3:  return SIM_LOCKED;
                case 1:  return SIM_READY;
                default: return SIM_ERROR;
            }
        }
        return SIM_ERROR;
    }

    //-----------------------------------------------------------------
    //  Generic Mobile Network functions
    //-----------------------------------------------------------------
    // Result should be: "+CGREG: 0,1" where:
    //  0 – disable network registration unsolicited result code
    //  1 – registered, home network
    TINY_GSM_MODEM_GET_REGISTRATION_XREG(CGREG)
    TINY_GSM_MODEM_GET_OPERATOR_COPS()
    TINY_GSM_MODEM_GET_CSQ()

    bool isNetworkConnected() {
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
    }

    String getNetworkModes() {
    sendAT(GF("+CNMP=?"));
    if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) {
        return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    return res;
    }

    TINY_GSM_MODEM_WAIT_FOR_NETWORK()

    String setNetworkMode(uint8_t mode) {
        sendAT(GF("+CNMP="), mode);
        if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) {
        return "OK";
        }
    String res = stream.readStringUntil('\n');
    waitResponse();
    return res;
    }


    //-----------------------------------------------------------------
    //  GPRS Connection functions
    //-----------------------------------------------------------------
    bool gprsConnect(const char* apn, const char* user = NULL, const char* pwd = NULL) {

        gprsDisconnect();     // Make sure we're not already connected!
        delay(2000L);

        //-------------------------------------------------------------------
        // NOTE:  For manual AT connections (using uart_forward.ino), there 
        //        seem to be some conenction info saved, so the following 
        //        commands will usually produce a successful connection.
        //        But strangely not always...needing manual reboots.
        //-------------------------------------------------------------------
        //  # Check what's already stored:
        //      AT+CGSOCKCONT?
        //      AT+CSOCKAUTH?
        //      AT+CGDCONT?
        //
        //      # AT+CGSOCKCONT=1,"IP","internet.xxxx.de","0.0.0.0",0,0
        //      AT+CSOCKAUTH=1,1,"xxxx","xxxx"
        //      AT+CGDCONT=1,"IP","internet.xxxx.de","0.0.0.0",0,0
        //      AT+NETOPEN           -- wait for this! (or ^^^ that?)
        //      AT+IPADDR
        //-------------------------------------------------------------------

    

        // Define the PDP context
        // Using CGSOCKCONT commands defines a PDP context for Embedded TCP/IP application
        // CGDCONT commands could be used for an external PDP context
        // AT+CGSOCKCONT= <cid>[,<PDP_type> [,<APN>[,<PDP_addr> [,<d_comp>[,<h_comp>]]]]]
/*
        sendAT(GF("+CGSOCKCONT=1,\"IP\",\""), apn, '"');
        waitResponse();
*/
        // Set the user name and password
        // AT+CSOCKAUTH     Set type of authentication for PDP-IP connections of socket
        // NOTE:    This seem to influence AT+CGAUTH ...
        if (user && strlen(user) > 0) {
            sendAT(GF("+CSOCKAUTH=1,1,\""), user, "\",\"", pwd, '"');
            waitResponse();
        }
/*
        // Set active PDP context’s profile number
        sendAT(GF("+CSOCKSETPN=1"));
        waitResponse();
*/
        // AT+CIPSENDMODE Select sending mode
        //      0 – sending without waiting peer TCP ACK mode 
        //      1 – sending wait peer TCP ACK mode
    /*
        sendAT(GF("+CIPSENDMODE=0"));
        waitResponse();
    */
        // Configure TCP parameters:
        //-----------------------------------------------------------------
        // AT+CIPCCFG= [<NmRetry>][,[<DelayTm>][,[<Ack>][,[<errMode>][,]<HeaderType>][,[[<AsyncMode>][,[<TimeoutVal>]]]]]]]]
        //      NmRetry     = number of retransmission to be made for an IP packet = 10 (default)
        //      DelayTm     = number of milliseconds to delay to output data of Receiving = 0 (default)
        //      Ack         = sets whether reporting a string “Send ok” = 0 (don't report)
        //      errMode     = mode of reporting error result code = 0 (numberic values)
        //      HeaderType  = which data header of receiving data in multi-client mode = 1 (“+RECEIVE,<link num>,<data length>”)
        //      AsyncMode   = sets mode of executing commands = 0 (synchronous command executing)
        //      TimeoutVal  = minimum retransmission timeout in milliseconds = 75000
        //-----------------------------------------------------------------
        // NOTE:  This is probably not needed for the SIM5360E...
        //-----------------------------------------------------------------
    /*
        sendAT(GF("+CIPCCFG=10,0,0,0,1,0,75000"));
        if (waitResponse() != 1) {
            return false;
        }
    */
        // Select TCP/IP application mode (command mode)
/*        
        sendAT(GF("+CIPMODE=0"));
        waitResponse();
*/
        // Configure timeouts for open and close socket
        // AT+CIPTIMEOUT=[<netopen_timeout>][, [<cipopen_timeout>][, [<cipsend_timeout>]]]
        //-----------------------------------------------------------------
        // NOTE:  This is probably not needed for the SIM5360E...
        //-----------------------------------------------------------------
    /*    
        sendAT(GF("+CIPTIMEOUT="), 75000, ',', 15000, ',', 15000);
        waitResponse();
    */

/*
        //-----------------------------------------------------------------
        // AT+CIPRXGET  Get the network data manually
        //-----------------------------------------------------------------
        //      1. If single-client:     (AT+CIPRXGET=0): AT+CIPRXGET=<mode>[,<len>]
        //      2. If multi-client:      (AT+CIPRXGET=1): AT+CIPRXGET=<mode>,<cid>[,<len>]
        //  Where MODE:  
        //      0 – set the way to get the network data automatically 
        //      1 – set the way to get the network data manually 
        //      2 – read data, the max read length is 1500 
        //      3 – read data in HEX form, the max read length is 750 
        //      4 – get the rest data length
        //-----------------------------------------------------------------
        sendAT(GF("+CIPRXGET=1"));
        if (waitResponse() != 1) {
            return false;
        }
*/
        // Attach to GPRS Packet domain
/*        
        sendAT(GF("+CGATT=1"));
        if (waitResponse(360000L) != 1) {
            return false;
        }
*/

/*
        // AT+CGAUTH    Set type of authentication for PDP-IP connections of GPRS
        // AT+CGAUTH = <cid>[,<auth_type>[,<passwd>[,<user>]]]
        // <cid>        = specify the particular PDP context.   [1-16]
        // <auth_type>  = Indicate the type of authentication to be used for the specified context:
        //                where:  [0=none, 1=PAP, 2=CHAP, 3=PAP||CHAP]
        // 
        // NOTE: It seem that <auth_type> = 0, is not allowed!
        if (user && strlen(user) > 0) {
            //sendAT(GF("+CGAUTH=1,0,\""), user, GF("\",\""), pwd, '"');  // auth = 0 is not allowed
            sendAT(GF("+CGAUTH=1,1,\""), user, GF("\",\""), pwd, '"');
            waitResponse();
        }
*/        
        // AT+CGDCONT   Define the PDP context 1
        // AT+CGDCONT = <cid>[,<PDP_type> [,<APN>[,<PDP_addr> [,<d_comp>[,<h_comp>]]]]]
        // NOTE:    Using CGDCONT sets up an "external" PCP context, i.e. a data connection
        //          using the external IP stack (e.g. Windows dial up) and PPP link over the
        //          serial interface. 
        
        //const char zero_ip[] = "0.0.0.0";
        //sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');
        // AT+CGDCONT=1,"IP","internet.xxxx.de","0.0.0.0",0,0
        sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"',",\"0.0.0.0\",0,0");
        waitResponse();

/*
        // Activate the PDP profile/context 1
        sendAT(GF("+CGACT=1,1")); // AT+CGACT=1
        if (waitResponse(75000L) != 1) 
            return false;
*/
        // Actually open the packet network socket
        // NOTE:  The AT Command Manual hints this might be depricated or that other 
        //        are options preferred, but all application notes still use it.
        // For a NETOPEN? :   The response should be: "+NETOPEN: 1,0" for an open/activated state
        // For a NETOPEN  :   The response should be: "+NETOPEN: 0" for successful open command 
        sendAT(GF("+NETOPEN"));
        //if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 1,0")) != 1) {
        if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 0")) != 1) {
            return false;
        }

        //-----------------------------------------------------------------
        // AT+CIPOPEN       Establish connection in multi-socket mode
        // AT+CIPOPEN = <link_num>,”TCP”,<serverIP>,<serverPort>[,<localPort>]
        // AT+CIPOPEN = <link_num>,”UDP”,,,<localPort>
        // Examples: 
        //      AT+CIPOPEN=0,"TCP","1.2.3.4",100
        //      AT+CIPOPEN=1,”UDP”,,,8080
        //-----------------------------------------------------------------
        /*  ToDo: Not sure this is the place    
        sendAT(GF("+CIPOPEN=0,\"TCP\",\""), serverIP, GF("\",\""), port, '"'); // <-- CHECK COMMAND!
        if (waitResponse(75000L) != 1) {
            return false;
        }*/
        return true;
    }

    bool gprsDisconnect() {
        // Before NETCLOSE we need to close all open TCP/UDP sockets with:
        // AT+CIPCLOSE=<link_num>
        // stop() is using that!        
        for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
          GsmClient *sock = sockets[mux];
          if (sock) {
            sock->stop();
          }
        }

        // ToDo: improve close return when conenction is not open:
        // Response: +NETCLOSE 2 
        // (see picture) from page 418
        sendAT(GF("+NETCLOSE"));            // Close the network (NOTE: ALL sockets should be closed first)
        if (waitResponse(60000L) != 1)
            return false;

        sendAT(GF("+CGACT=1,0"));           // Deactivate PDP context 1
        if (waitResponse(40000L) != 1) {
            return false;
        }

        sendAT(GF("+CGATT=0"));             // Detach from GPRS
        if (waitResponse(360000L) != 1) {
            return false;
        }

        return true;
    }

    bool isGprsConnected() {
        // The response should be: "+NETOPEN: 1,0" for an open/activated state
        // Also the response may take up to 120 seconds, so may need to increase?
        sendAT(GF("+NETOPEN?"));
        //if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 1")) != 1) {        // QQQ with a "," or not??
        //if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 1,")) != 1) {       // QQQ with a "," or not??
        if (waitResponse(75000L, GF(GSM_NL "+NETOPEN: 1,0")) != 1) {        // QQQ with a "," or not??
            return false;
        }
        waitResponse();

        sendAT(GF("+IPADDR")); // Inquire Socket PDP address
        //if (waitResponse(40000L) != 1) {
        if (waitResponse() != 1) {
            return false;
        }

        return true;
    }

    //-----------------------------------------------------------------
    //  IP Address functions
    //-----------------------------------------------------------------
    String getLocalIP() {
        sendAT(GF("+IPADDR"));  // Inquire Socket PDP address
        // sendAT(GF("+CGPADDR=1")); // Show PDP address
        // ToDo: We should wait for: +IPADDR
        String res;
        if (waitResponse(10000L, res) != 1) {
            return "";
        }
        res.replace(GSM_NL "OK" GSM_NL, "");
        res.replace(GSM_NL, "");
        res.trim();
        return res;
    }

    IPAddress localIP() {
        return TinyGsmIpFromString(getLocalIP());
    }

    //-----------------------------------------------------------------
    //  Phone Call functions
    //-----------------------------------------------------------------
    bool setGsmBusy() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callAnswer() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callNumber() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callHangup() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool dtmfSend()   TINY_GSM_ATTR_NOT_IMPLEMENTED;

    //-----------------------------------------------------------------
    //  Messaging functions
    //-----------------------------------------------------------------
    String sendUSSD(const String& code) {
        sendAT(GF("+CMGF=1"));
        waitResponse();
        sendAT(GF("+CSCS=\"HEX\""));
        waitResponse();
        sendAT(GF("+CUSD=1,\""), code, GF("\""));
        if (waitResponse() != 1) {
            return "";
        }
        if (waitResponse(10000L, GF(GSM_NL "+CUSD:")) != 1) {
            return "";
        }
        stream.readStringUntil('"');
        String hex = stream.readStringUntil('"');
        stream.readStringUntil(',');
        int dcs = stream.readStringUntil('\n').toInt();
        if (dcs == 15) {
            return TinyGsmDecodeHex8bit(hex);
        } else if (dcs == 72) {
            return TinyGsmDecodeHex16bit(hex);
        } else {
            return hex;
        }
    }

    bool sendSMS(const String& number, const String& text) {
        sendAT(GF("+AT+CSCA?"));
        waitResponse();
        sendAT(GF("+CMGF=1"));
        waitResponse();
        //Set GSM 7 bit default alphabet (3GPP TS 23.038)
        sendAT(GF("+CSCS=\"GSM\""));
        waitResponse();
        sendAT(GF("+CMGS=\""), number, GF("\""));
        if (waitResponse(GF(">")) != 1) {
            return false;
        }
        stream.print(text);
        stream.write((char)0x1A);
        stream.flush();
        return waitResponse(60000L) == 1;
    }

    bool sendSMS_UTF16(const String& number, const void* text, size_t len) {
        sendAT(GF("+CMGF=1"));
        waitResponse();
        sendAT(GF("+CSCS=\"HEX\""));
        waitResponse();
        sendAT(GF("+CSMP=17,167,0,8"));
        waitResponse();
        sendAT(GF("+CMGS=\""), number, GF("\""));
        if (waitResponse(GF(">")) != 1) {
            return false;
        }

        uint16_t* t = (uint16_t*)text;
        for (size_t i=0; i<len; i++) {
            uint8_t c = t[i] >> 8;
            if (c < 0x10) { stream.print('0'); }
            stream.print(c, HEX);
            c = t[i] & 0xFF;
            if (c < 0x10) { stream.print('0'); }
            stream.print(c, HEX);
        }
        stream.write((char)0x1A);
        stream.flush();
        return waitResponse(60000L) == 1;
    }


    //-----------------------------------------------------------------
    //  Location functions
    //-----------------------------------------------------------------
    String getGsmLocation()  TINY_GSM_ATTR_NOT_IMPLEMENTED;

    //-----------------------------------------------------------------
    //  GPS location functions
    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
    //  Time functions
    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
    //  Battery & temperature functions
    //-----------------------------------------------------------------
    // Use: float vBatt = modem.getBattVoltage() / 1000.0;
    uint16_t getBattVoltage() {
        sendAT(GF("+CBC"));
        if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
            return 0;
        }
        streamSkipUntil(','); // Skip battery charge status
        streamSkipUntil(','); // Skip battery charge level
        // return voltage in mV
        uint16_t res = stream.readStringUntil(',').toInt();
        // Wait for final OK
        waitResponse();
        return res;
    }

    int8_t getBattPercent() {
        sendAT(GF("+CBC"));
        if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
            return false;
        }
        streamSkipUntil(','); // Skip battery charge status
        // Read battery charge level
        int res = stream.readStringUntil(',').toInt();
        // Wait for final OK
        waitResponse();
        return res;
        }

        uint8_t getBattChargeState() {
        sendAT(GF("+CBC?"));
        if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
            return false;
        }
        // Read battery charge status
        int res = stream.readStringUntil(',').toInt();
        // Wait for final OK
        waitResponse();
        return res;
        }

        bool getBattStats(uint8_t &chargeState, int8_t &percent, uint16_t &milliVolts) {
        sendAT(GF("+CBC?"));
        if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
            return false;
        }
        chargeState = stream.readStringUntil(',').toInt();
        percent = stream.readStringUntil(',').toInt();
        milliVolts = stream.readStringUntil('\n').toInt();
        // Wait for final OK
        waitResponse();
        return true;
    }

    float getTemperature() TINY_GSM_ATTR_NOT_AVAILABLE;
    // ToDo: 
    // # Enable Temparature Reading:
    //AT+CMTE=1
    //AT+CMTE?


    //-----------------------------------------------------
    //  GRPS TCP/IP Client related functions
    //-----------------------------------------------------
    protected:
    bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false, int timeout_s = 75) {
        // Make sure we'll be getting data manually on this connection
        sendAT(GF("+CIPRXGET=1"));
        if (waitResponse() != 1) {
            return false;
        }

        // Establish a connection in multi-socket mode
        sendAT(GF("+CIPOPEN="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
        // The reply is +CIPOPEN: ## of socket created
        if (waitResponse(15000L, GF(GSM_NL "+CIPOPEN:")) != 1) {  
            return false;
        }
        return true; //QQQ waitResponse() == 1;
    }

    int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
        
        // ToDo:  Do we need to set mode first?
        //-------------------------------------------------------------
        // AT+CIPSENDMODE   Select sending mode
        // AT+CIPSENDMODE=<mode>    where: 
        //      0 – sending without waiting peer TCP ACK mode 
        //      1 – sending wait peer TCP ACK mode
        //-------------------------------------------------------------
        // AT+CIPSEND  Send data through TCP or UDP connection
        // 
        //  For TCP:    AT+CIPSEND=<link_num>,<length><CR><data-to-send>
        //  For UDP:    AT+CIPSEND=<link_num>,<length>,<serverIP>,<serverPort><CR><data-to-send>
        // 
        // This command is used to send data to remote side. 
        // The <length> field can be empty. When it is empty:
        //      Each <Ctrl+Z> = character in the data should be coded as:  <ETX><Ctrl+Z>. 
        //      Each <ESC>    = character in the data should be coded as:  <ETX><ESC>. 
        //      Each <ETX>    = character will be coded as:  <ETX><ETX>. 
        //      * A single <Ctrl+Z> means end of the input data. 
        //      * A single <ESC> is used to cancel the sending. 
        //  <ETX>    = 0x03
        //  <Ctrl+Z> = 0x1A
        //  <ESC>    = 0x1B
        //-------------------------------------------------------------
        sendAT(GF("+CIPSEND="), mux, ',', len);
        if (waitResponse(GF(">")) != 1) {
            return 0;
        }
        stream.write((uint8_t*)buff, len);
        stream.flush();
        if (waitResponse(GF(GSM_NL "+CIPSEND:")) != 1) {
            return 0;
        }
        streamSkipUntil(','); // Skip mux
        streamSkipUntil(','); // Skip requested bytes to send
        // TODO:  make sure requested and confirmed bytes match
        return stream.readStringUntil('\n').toInt();
    }

    size_t modemRead(size_t size, uint8_t mux) {
#ifdef TINY_GSM_USE_HEX
        sendAT(GF("+CIPRXGET=3,"), mux, ',', size);
        if (waitResponse(GF("+CIPRXGET:")) != 1) {
            return 0;
        }
#else
        sendAT(GF("+CIPRXGET=2,"), mux, ',', size);
        if (waitResponse(GF("+CIPRXGET:")) != 1) {
            return 0;
        }
#endif
        streamSkipUntil(','); // Skip Rx mode 2/normal or 3/HEX
        streamSkipUntil(','); // Skip mux/cid (connecion id)
        size_t len_requested = stream.readStringUntil(',').toInt();
        //  ^^ Requested number of data bytes (1-1460 bytes)to be read
        size_t len_confirmed = stream.readStringUntil('\n').toInt();
        // ^^ The data length which not read in the buffer
        for (size_t i=0; i<len_requested; i++) {
            uint32_t startMillis = millis();
#ifdef TINY_GSM_USE_HEX
            while (stream.available() < 2 && (millis() - startMillis < sockets[mux]->_timeout)) { TINY_GSM_YIELD(); }
            char buf[4] = { 0, };
            buf[0] = stream.read();
            buf[1] = stream.read();
            char c = strtol(buf, NULL, 16);
#else
            while (!stream.available() && (millis() - startMillis < sockets[mux]->_timeout)) { TINY_GSM_YIELD(); }
            char c = stream.read();
#endif
            sockets[mux]->rx.put(c);
        }

        DBG("### READ:", len_requested, "from", mux);
        // sockets[mux]->sock_available = modemGetAvailable(mux);
        sockets[mux]->sock_available = len_confirmed;
        waitResponse();
        return len_requested;
    }

    size_t modemGetAvailable(uint8_t mux) {
        sendAT(GF("+CIPRXGET=4,"), mux);
        size_t result = 0;
        if (waitResponse(GF("+CIPRXGET:")) == 1) {
            streamSkipUntil(','); // Skip mode 4
            streamSkipUntil(','); // Skip mux
            result = stream.readStringUntil('\n').toInt();
            waitResponse();
        }
        DBG("### Available:", result, "on", mux);
        if (!result) {
            sockets[mux]->sock_connected = modemGetConnected(mux);
        }
        return result;
    }

    bool modemGetConnected(uint8_t mux) {
        // Read the status of all sockets at once
        sendAT(GF("+CIPCLOSE?"), mux);
        if (waitResponse(GFP(GSM_OK), GF(GSM_NL "+CIPCLOSE: ")) != 2)
            return false;
        for (int muxNo = 0; muxNo <= TINY_GSM_MUX_COUNT; muxNo++) {
            // +CIPCLOSE:<link0_state>,<link1_state>,...,<link9_state>
            sockets[muxNo]->sock_connected = stream.parseInt();
        }
        waitResponse();  // Should be an OK at the end
        return sockets[mux]->sock_connected;
    }

public:

    //-----------------------------------------------------
    // Utilities
    //-----------------------------------------------------
    TINY_GSM_MODEM_STREAM_UTILITIES()

    // TODO: Optimize this!
    //-----------------------------------------------------
    // This function parses the modem's ATCoP response codes.
    //
    // NOTE: 
    //    Some of the codes that also need to be parsed 
    //    are Unsolicitated Response Codes (URC)s.
    //    Those codes may arrive at any time. 
    //    The ones relevant for Network connection (p.418):
    //      +CIPEVENT: NETWORK CLOSED UNEXPECTEDLY
    //      +IPCLOSE: <client_index>, <close_reason>
    //      +CLIENT: < link_num >,<server_index>,<client_IP>:<port>
    //
    //    The ones relevant for Receiving Data (p.414):
    //      +CIPRXGET:
    //          1. If <mode> = 0 or 1:      OK
    //          2. If <mode> = 2 or 3:
    //                          a. If single-client:    +CIPRXGET: <mode>,<read_len>,<rest_len> <data>
    //                          b. If multi-client:     +CIPRXGET: <mode>,<cid>,<read_len>,<rest_len> <data>
    //          3. If<mode> = 4: 
    //                          a. If single-client:    +CIPRXGET: 4,<rest_len>
    //                          b. If multi-client:     +CIPRXGET: 4,<cid>,<rest_len>
    //          4. If an error occurs:                  +IP ERROR: <error message>
    //      +RECEIVE: 
    //    The ones relevant for Receiving HTTP (p.474):
    //      +CHTTPS: RECV EVENT         - When there is data cached in the receiving buffer
    //      +CHTTPSNOTIFY: PEER CLOSED  - The HTTPS session is closed by the server.
    // 
    //    The ones relevant for Common Channel (p.474):
    //      +CCHRECV: DATA, <session_id>,<len>
    //-----------------------------------------------------
    uint8_t waitResponse(uint32_t timeout_ms, 
                        String& data,
                        GsmConstStr r1=GFP(GSM_OK), 
                        GsmConstStr r2=GFP(GSM_ERROR),
                        GsmConstStr r3=NULL, 
                        GsmConstStr r4=NULL, 
                        GsmConstStr r5=NULL) {
        /*String r1s(r1); r1s.trim();
        String r2s(r2); r2s.trim();
        String r3s(r3); r3s.trim();
        String r4s(r4); r4s.trim();
        String r5s(r5); r5s.trim();
        DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
        data.reserve(64);
        int index = 0;
        unsigned long startMillis = millis();

        do {
            TINY_GSM_YIELD();
            while (stream.available() > 0) {
                TINY_GSM_YIELD();
                int a = stream.read();
                if (a <= 0) continue; // Skip 0x00 bytes, just in case
                data += (char)a;

                if (r1 && data.endsWith(r1)) {          index = 1; goto finish;
                } else if (r2 && data.endsWith(r2)) {   index = 2; goto finish;
                } else if (r3 && data.endsWith(r3)) {   index = 3; goto finish;
                } else if (r4 && data.endsWith(r4)) {   index = 4; goto finish;
                } else if (r5 && data.endsWith(r5)) {   index = 5; goto finish;
                // Test 
                } else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
                    String mode = stream.readStringUntil(',');
                    if (mode.toInt() == 1) {
                        int mux = stream.readStringUntil('\n').toInt();
                        if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                            sockets[mux]->got_data = true;
                        }
                        data = "";
                        DBG("### Got Data:", mux);
                    } else {
                        data += mode;
                    }
                } else if (data.endsWith(GF(GSM_NL "+RECEIVE:"))) {
                    int mux = stream.readStringUntil(',').toInt();
                    int len = stream.readStringUntil('\n').toInt();
                    if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                        sockets[mux]->got_data = true;
                        sockets[mux]->sock_available = len;
                    }
                    data = "";
                    DBG("### Got Data:", len, "on", mux);
                } else if (data.endsWith(GF("+IPCLOSE:"))) {
                    int mux = stream.readStringUntil(',').toInt();
                    streamSkipUntil('\n');  // Skip the reason code
                    if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
                        sockets[mux]->sock_connected = false;
                    }
                    data = "";
                    DBG("### Closed: ", mux);
                }
            }
        } while (millis() - startMillis < timeout_ms);

    finish:
        if (!index) {
            data.trim();
            if (data.length()) {
                DBG("### Unhandled:", data);
            }
            data = "";
        }
        //data.replace(GSM_NL, "/");
        //DBG('<', index, '>', data);
        return index;
    }

    // Parsing an OK/ERROR response when given a timeout 
    uint8_t waitResponse(   uint32_t timeout_ms,
        GsmConstStr r1=GFP(GSM_OK),GsmConstStr r2=GFP(GSM_ERROR),GsmConstStr r3=NULL,GsmConstStr r4=NULL,GsmConstStr r5=NULL) {
        String data; //???
        return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
    }

    // Parsing an OK/ERROR response without a timeout --> default to 1 sec
    uint8_t waitResponse(
        GsmConstStr r1=GFP(GSM_OK),GsmConstStr r2=GFP(GSM_ERROR),GsmConstStr r3=NULL,GsmConstStr r4=NULL,GsmConstStr r5=NULL) {
        return waitResponse(1000, r1, r2, r3, r4, r5);
    }

public:
    Stream&  stream;

protected:
    GsmClient* sockets[TINY_GSM_MUX_COUNT];
};

#endif
