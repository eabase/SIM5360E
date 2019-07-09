/**
* @file       TinyGsmClientSIM5360.h
* @author     Volodymyr Shymanskyy
* @license    LGPL-3.0
* @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
* @date       Nov 2016
*/
#include <Arduino.h>
#ifndef TinyGsmClientSIM5360_h
#define TinyGsmClientSIM5360_h

//#define TINY_GSM_DEBUG Serial


#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 64
#endif

#define TINY_GSM_MUX_COUNT 1
#define PIN_SIM_POWER 27
#define PIN_SIM_UART_RXD 16
#define PIN_SIM_UART_TXD 17
#include <TinyGsmCommon.h>
extern HardwareSerial xbSerial;
#define GSM_NL "\r"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#define BEE_BAUDRATE 115200L
enum SimStatus {
    SIM_ERROR = 0,
    SIM_READY = 1,
    SIM_LOCKED = 2,
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

#define HTTP_CONN_TIMEOUT 5000


typedef enum {
    HTTP_DISCONNECTED = 0,
    HTTP_CONNECTED,
    HTTP_SENT,
    HTTP_ERROR,
} HTTP_STATES;

class GsmClient5360;

class TinyGsmSim5360
{
    friend class GsmClient5360;
private:
    char m_buffer[384] = {0};
    void xbTogglePower()
    {
        digitalWrite(PIN_SIM_POWER, HIGH);
        delay(50);
        digitalWrite(PIN_SIM_POWER, LOW);
        delay(2000);
        digitalWrite(PIN_SIM_POWER, HIGH);
        delay(1000);
        digitalWrite(PIN_SIM_POWER, LOW);
    }
    int dumpLine(char* buffer, int len)
    {
        int bytesToDump = len >> 1;
        for (int i = 0; i < len; i++) {
            // find out first line end and discard the first line
            if (buffer[i] == '\r' || buffer[i] == '\n') {
                // go through all following \r or \n if any
                while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
                bytesToDump = i;
                break;
            }
        }
        memmove(buffer, buffer + bytesToDump, len - bytesToDump);
        return bytesToDump;
    }  
    
    void xbPurge()
    {
        xbSerial.flush();
    }
    
    bool xbBegin(unsigned long baudrate)
    {
        pinMode(PIN_SIM_POWER, OUTPUT);
        digitalWrite(PIN_SIM_POWER, HIGH);
        xbSerial.begin(BEE_BAUDRATE, SERIAL_8N1, PIN_SIM_UART_RXD, PIN_SIM_UART_TXD);
        return true;
    }
    
    void xbWrite(const char* cmd)
    {
        xbSerial.print(cmd);
    }

    int xbRead(char* buffer, int bufsize, unsigned int timeout)
    {
        int n = 0;
        uint32_t t = millis();
        do {
            while (xbSerial.available() && n < bufsize - 1) {
                buffer[n++] = xbSerial.read();
            }
        } while (millis() - t <= timeout);
        buffer[n] = 0;
        return n;
    }
    
    int xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
    {
        int bytesRecv = 0;
        uint32_t t = millis();
        do {
            if (bytesRecv >= bufsize - 16) {
                bytesRecv -= dumpLine(buffer, bytesRecv);
            }
            int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 100);
            if (n > 0) {
                bytesRecv += n;
                buffer[bytesRecv] = 0;
                for (byte i = 0; i < expectedCount; i++) {
                    // match expected string(s)
                    if (expected[i] && strstr(buffer, expected[i])) return i + 1;
                }
            } else if (n == -1) {
                // an erroneous reading
                break;
            }
        } while (millis() - t < timeout);
        buffer[bytesRecv] = 0;
        return 0;
    } 
public:
    char udpIP[16] = {0};
    uint16_t udpPort;
    
    TinyGsmSim5360(Stream& _stream)/*: stream(_stream)*/
    {
        memset(sockets, 0, sizeof(sockets));
    }

    bool begin() {
        return init();
    }

    bool init() {
        xbBegin(BEE_BAUDRATE);
        for (byte n = 0; n < 10; n++) {
            // try turning on module
            xbTogglePower();
            delay(2000);
            xbPurge();
            for (byte m = 0; m < 3; m++) {
                if (sendCommand("AT\r")) {
                    return true;
                }
            }
        }
        return false;
    }
    void stop()
    {
        sendCommand("AT+CPOWD=1\r");
    }

    bool restart() {
        stop();
        delay(3000);
        return init();
    }

    /*
    * GPRS functions
    */
    bool gprsConnect(const char* apn, const char* user = NULL, const char* pwd = NULL) {
        uint32_t t = millis();
        bool success = false;
        if (!sendCommand("ATE0\r")) return false;
        bool only3G = true;
        bool roaming = true;
        if (only3G) sendCommand("AT+CNMP=14\r"); // use WCDMA only
        do {
            do {
                Serial.print('.');
                delay(500);
                success = sendCommand("AT+CPSI?\r", 1000, "Online");
                if (success) {
                    if (!strstr(m_buffer, "NO SERVICE"))
                    break;
                    success = false;
                }
                if (strstr(m_buffer, "Off")) break;
            } while (millis() - t < 30000);
            if (!success) break;

            t = millis();
            do {
                success = sendCommand("AT+CREG?\r", 5000, roaming ? "+CREG: 0,5" : "+CREG: 0,1");
            } while (!success && millis() - t < 30000);
            if (!success) break;

            do {
                success = sendCommand("AT+CGREG?\r",1000, roaming ? "+CGREG: 0,5" : "+CGREG: 0,1");
            } while (!success && millis() - t < 30000);
            if (!success) break;

            do {
                sprintf(m_buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
                success = sendCommand(m_buffer);
            } while (!success && millis() - t < 30000);
            if (!success) break;

            //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");

            success = sendCommand("AT+CSOCKSETPN=1\r");
            if (!success) break;

            success = sendCommand("AT+CIPMODE=0\r");
            if (!success) break;

            sendCommand("AT+NETOPEN\r");
        } while(0);
        if (!success) Serial.println(m_buffer);
        return success;
    }

    bool isGprsConnected() {
        uint32_t t = millis();
        unsigned int timeout = 5000;
        bool success=false;
        do {
            success = sendCommand("AT+CGATT?\r", 3000, "+CGATT: 1");
        } while (!success && millis() - t < timeout);
        /*
        sprintf(m_buffer, "AT+CSTT=\"%s\"\r", apn);
        if (!sendCommand(m_buffer)) {
            return false;
        }
        sendCommand("AT+CIICR\r");
        */
        return success;
    }

    String getLocalIP() {
        uint32_t t = millis();
        do {
            if (sendCommand("AT+IPADDR\r", 3000, "\r\nOK\r\n", true)) {
                char *p = strstr(m_buffer, "+IPADDR:");
                if (p) {
                    char *ip = p + 9;
                    if (*ip != '0') {
                        char *q = strchr(ip, '\r');
                        if (q) *q = 0;
                        return ip;
                    }
                }
            }
            delay(500);
        } while (millis() - t < 15000);
        return "";
    }

    bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false) {
        /*         int rsp;
        sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
        rsp = waitResponse(75000L,
        GF("CONNECT OK" GSM_NL),
        GF("CONNECT FAIL" GSM_NL),
        GF("ALREADY CONNECT" GSM_NL),
        GF("ERROR" GSM_NL),
        GF("CLOSE OK" GSM_NL)   // Happens when HTTPS handshake fails
        );
        return (1 == rsp); */
        String ip = queryIP(host);
        strncpy(udpIP, ip.c_str(), sizeof(udpIP) - 1);
        udpPort = port;
        sprintf(m_buffer, "AT+CIPSTART=\"%s\",%u,1\r", host, port);
        if (sendCommand(m_buffer, HTTP_CONN_TIMEOUT)) {
            //m_state = HTTP_CONNECTED;
            return true;
        } else {
            //m_state = HTTP_ERROR;
            return false;
        }
        
    }
    String queryIP(const char* host)
    {
        sprintf(m_buffer, "AT+CDNSGIP=\"%s\"\r", host);
        if (sendCommand(m_buffer, 10000)) {
            char *p = strstr(m_buffer, host);
            if (p) {
                p = strstr(p, ",\"");
                if (p) {
                    char *ip = p + 2;
                    p = strchr(ip, '\"');
                    if (p) *p = 0;
                    return ip;
                }
            }
        }
        return "";
    }
    int modemSend(const void* buff, size_t len, uint8_t mux) {
        /*         sendAT(GF("+CIPSEND="), mux, ',', len);
        if (waitResponse(GF(">")) != 1) {
            return 0;
        }
        stream.write((uint8_t*)buff, len);
        stream.flush();
        if (waitResponse(GF(GSM_NL "DATA ACCEPT:")) != 1) {
            return 0;
        }
        streamSkipUntil(','); // Skip mux
        return stream.readStringUntil('\n').toInt(); */
        
        sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP, udpPort);
        if (sendCommand(m_buffer, 100, ">")) {
            xbWrite((const char*)buff);
            return sendCommand(0, 1000);
        }
        return false;
    }
    char* receive(int* pbytes, unsigned int timeout)
    {
        char *data = checkIncoming(pbytes);
        if (data) return data;
        if (sendCommand(0, timeout, "+IPD")) {
            return checkIncoming(pbytes);
        }
        return 0;
    }

    char* checkIncoming(int* pbytes)
    {
        char *p = strstr(m_buffer, "+IPD");
        if (p) {
            *p = '-'; // mark this datagram as checked
            int len = atoi(p + 4);
            if (pbytes) *pbytes = len;
            p = strchr(p, '\n');
            if (p) {
                *(++p + len) = 0;
                return p;
            }
        }
        return 0;
    }
    size_t modemRead(size_t size, uint8_t mux) {

        /*
        sendAT(GF("+CIPRXGET=2,"), mux, ',', size);
        if (waitResponse(GF("+CIPRXGET:")) != 1) {
            return 0;
        }
        streamSkipUntil(','); // Skip mode 2/3
        streamSkipUntil(','); // Skip mux
        size_t len = stream.readStringUntil(',').toInt();
        sockets[mux]->sock_available = stream.readStringUntil('\n').toInt();

        for (size_t i=0; i<len; i++) {
            while (!stream.available()) {  }
            char c = stream.read();
            sockets[mux]->rx.put(c);
        }
        waitResponse();
        return len;
        */
        return 1;
    }

public:
    bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n", bool terminated = false)
    {
        if (cmd) {
            xbWrite(cmd);
        }
        m_buffer[0] = 0;
        byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
        if (ret) {
            if (terminated) {
                char *p = strstr(m_buffer, expected);
                if (p) *p = 0;
            }
            return true;
        } else {
            return false;
        }
    }    
    
public:
    //Stream&       stream;
    GsmClient5360*    sockets[TINY_GSM_MUX_COUNT];
};


class GsmClient5360 : public Client
{
    friend class TinyGsmSim5360;
    typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
    GsmClient5360() {}

    GsmClient5360(TinyGsmSim5360& modem, uint8_t mux = 1) {
        init(&modem, mux);
    }

    bool init(TinyGsmSim5360* modem, uint8_t mux = 1) {
        this->at = modem;
        this->mux = mux;
        sock_available = 0;
        prev_check = 0;
        sock_connected = false;
        got_data = false;
        at->sockets[mux] = this;
        return true;
    }

    virtual int connect(const char *host, uint16_t port) {
        stop();
        
        rx.clear();
        sock_connected = at->modemConnect(host, port, mux);
        return sock_connected;
    }

    virtual int connect(IPAddress ip, uint16_t port) {
        String host; host.reserve(16);
        host += ip[0];
        host += ".";
        host += ip[1];
        host += ".";
        host += ip[2];
        host += ".";
        host += ip[3];
        return connect(host.c_str(), port);
    }

    virtual void stop() {
        at->stop();
        sock_connected = false;
        //at->waitResponse();
        //rx.clear();
    }

    virtual size_t write(const uint8_t *buf, size_t size) {
        return at->modemSend(buf, size, mux);
    }

    virtual size_t write(uint8_t c) {
        return write(&c, 1);
    }

    virtual size_t write(const char *str) {
        if (str == NULL) return 0;
        return write((const uint8_t *)str, strlen(str));
    }

    virtual int available() {
        
        if (!rx.size() && sock_connected) {
            // Workaround: sometimes SIM5360 forgets to notify about data arrival.
            // TODO: Currently we ping the module periodically,
            // but maybe there's a better indicator that we need to poll
            if (millis() - prev_check > 500) {
                got_data = true;
                prev_check = millis();
            }
            //at->maintain();
        }
        return rx.size() + sock_available;
    }

    virtual int read(uint8_t *buf, size_t size) {
        
        //at->maintain();
        size_t cnt = 0;
        while (cnt < size && sock_connected) {
            size_t chunk = TinyGsmMin(size-cnt, rx.size());
            if (chunk > 0) {
                rx.get(buf, chunk);
                buf += chunk;
                cnt += chunk;
                continue;
            }
            // TODO: Read directly into user buffer?
            //at->maintain();
            if (sock_available > 0) {
                at->modemRead(rx.free(), mux);
            } else {
                break;
            }
        }
        return cnt;
    }

    virtual int read() {
        uint8_t c;
        if (read(&c, 1) == 1) {
            return c;
        }
        return -1;
    }

    virtual int peek() { return -1; } //TODO
    virtual void flush() { at->xbPurge(); }

    virtual uint8_t connected() {
        if (available()) {
            return true;
        }
        return sock_connected;
    }
    virtual operator bool() { return connected(); }

private:
    TinyGsmSim5360* at;
    uint8_t        mux;
    uint16_t       sock_available;
    uint32_t       prev_check;
    bool           sock_connected;
    bool           got_data;
    RxFifo         rx;
};
#endif
