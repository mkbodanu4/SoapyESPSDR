#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <zlib.h>

// SoapySDR intentionally keeps Stream opaque; modules own the concrete state.
namespace SoapySDR { class Stream {}; }

namespace {

using namespace SoapySDR;

static int baudConstant()
{
#ifdef B2000000
    return B2000000;
#else
    return B115200;
#endif
}

class ESPDevice final : public Device {
public:
    explicit ESPDevice(const Kwargs &args)
    {
        path = args.count("device") ? args.at("device") : "/dev/ttyACM0";
        burstMode = args.count("mode") && args.at("mode") == "burst";
        if (burstMode) { rates={16e6,40e6,80e6}; sampleRate=16e6; }
        fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
        termios tio{};
        if (tcgetattr(fd, &tio) == 0) {
            cfmakeraw(&tio);
            cfsetispeed(&tio, baudConstant()); cfsetospeed(&tio, baudConstant());
            tio.c_cflag |= CLOCAL | CREAD;
            tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 10;
            tcsetattr(fd, TCSANOW, &tio);
        }
        try {
            info = command("INFO");
            caps = command("CAPS");
            parseLimits(command("LIMITS?"));
            parseRange(command("RANGE?"));
            setFrequency(SOAPY_SDR_RX, 0, frequency);
        } catch (...) { ::close(fd); fd = -1; throw; }
    }

    ~ESPDevice() override { stopWorker(); if (fd >= 0) ::close(fd); }

    std::string getHardwareKey() const override { return path; }
    Kwargs getHardwareInfo() const override { return {{"firmware", info}, {"transport", "native USB serial/JTAG"}, {"streaming", burstMode?"CAP16 bursts; gaps expected":"IQS decimated continuous stream"}}; }
    size_t getNumChannels(const int direction) const override { return direction == SOAPY_SDR_RX ? 1 : 0; }
    std::string getDriverKey(void) const override { return "espsdr"; }
    long long getHardwareTime(const std::string &) const override { return 0; }

    std::vector<std::string> getStreamFormats(const int d, const size_t c) const override {
        checkRx(d, c); return {SOAPY_SDR_CS8, SOAPY_SDR_CF32};
    }
    std::string getNativeStreamFormat(const int d, const size_t c, double &full) const override {
        checkRx(d, c); full = 128.0; return SOAPY_SDR_CS8;
    }
    std::vector<double> listSampleRates(const int d, const size_t c) const override { checkRx(d,c); return rates; }
    RangeList getSampleRateRange(const int d, const size_t c) const override { checkRx(d,c); RangeList out; for (const double r : rates) out.emplace_back(r, r, 0.0); return out; }
    void setSampleRate(const int d, const size_t c, const double r) override {
        checkRx(d,c); auto it=std::min_element(rates.begin(),rates.end(),[r](double a,double b){return std::abs(a-r)<std::abs(b-r);});
        // gr-osmosdr may issue a zero/unset rate while constructing its source.
        // Use the safest nominal ESP-SDR rate for that initialization request.
        if (it==rates.end() || !std::isfinite(r) || r<=0.0) it=std::find(rates.begin(), rates.end(), 16e6);
        if (it==rates.end()) throw std::invalid_argument("no supported sample rate");
        const bool restart=active!=nullptr && workerRunning.load(); if(restart) stopWorker();
        { std::lock_guard<std::mutex> l(stateMutex); sampleRate=*it; selectIqRate(*it); }
        if(restart) startWorker();
    }
    double getSampleRate(const int d, const size_t c) const override { checkRx(d,c); return sampleRate; }

    RangeList getFrequencyRange(const int d, const size_t c) const override { checkRx(d,c); return {Range(freqMin*1e6,freqMax*1e6,1e6)}; }
    void setFrequency(const int d, const size_t c, const double f, const Kwargs & = {}) override {
        checkRx(d,c); if (!std::isfinite(f) || f<freqMin*1e6 || f>freqMax*1e6) throw std::invalid_argument("frequency outside firmware range");
        unsigned mhz=(unsigned)std::llround(f/1e6);
        mhz=std::clamp(mhz,freqMin,freqMax);
        const bool restart=active!=nullptr && workerRunning.load(); if(restart) stopWorker();
        { std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("FREQ " + std::to_string(mhz))); frequency=mhz*1e6; }
        if(restart) startWorker();
    }
    double getFrequency(const int d, const size_t c) const override { checkRx(d,c); return frequency; }

    std::vector<double> listBandwidths(const int d, const size_t c) const override { checkRx(d,c); std::vector<double> r; for(unsigned i=bwMin;i<=bwMax;i++) r.push_back(i*1e6); return r; }
    RangeList getBandwidthRange(const int d, const size_t c) const override { checkRx(d,c); return {Range(bwMin*1e6,bwMax*1e6,1e6)}; }
    void setBandwidth(const int d, const size_t c, const double b) override { checkRx(d,c); unsigned mhz; if(!std::isfinite(b)||b<=0.0) mhz=bwMax; else mhz=(unsigned)std::llround(b/1e6); mhz=std::clamp(mhz,bwMin,bwMax); const bool restart=active!=nullptr && workerRunning.load(); if(restart) stopWorker(); { std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("BANDWIDTH "+std::to_string(mhz))); bandwidth=mhz*1e6; } if(restart) startWorker(); }
    double getBandwidth(const int d, const size_t c) const override { checkRx(d,c); return bandwidth; }

    std::vector<std::string> listGains(const int d, const size_t c) const override { checkRx(d,c); return {"RF"}; }
    bool hasGainMode(const int d, const size_t c) const override { checkRx(d,c); return true; }
    void setGainMode(const int d, const size_t c, const bool automatic) override { checkRx(d,c); const bool restart=active!=nullptr && workerRunning.load(); if(restart) stopWorker(); { std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked(automatic?"GAIN HARDWARE":"GAIN MANUAL "+std::to_string((unsigned)gain))); autoGain=automatic; } if(restart) startWorker(); }
    bool getGainMode(const int d, const size_t c) const override { checkRx(d,c); return autoGain; }
    void setGain(const int d, const size_t c, const double g) override { checkRx(d,c); if(g<gainMin||g>gainMax) throw std::invalid_argument("gain outside firmware range"); const bool restart=active!=nullptr && workerRunning.load(); if(restart) stopWorker(); { std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("GAIN MANUAL "+std::to_string((unsigned)std::llround(g)))); gain=g; autoGain=false; } if(restart) startWorker(); }
    double getGain(const int d, const size_t c) const override { checkRx(d,c); return gain; }
    Range getGainRange(const int d, const size_t c) const override { checkRx(d,c); return Range(gainMin,gainMax,1); }

    Stream *setupStream(const int d, const std::string &format, const std::vector<size_t> &chs, const Kwargs & = {}) override {
        checkRx(d, chs.empty()?0:chs.front()); if(!chs.empty() && (chs.size()!=1||chs[0]!=0)) throw std::invalid_argument("only RX channel 0 exists");
        if(format!=SOAPY_SDR_CS8 && format!=SOAPY_SDR_CF32) throw std::invalid_argument("format must be CS8 or CF32");
        return new StreamState{format};
    }
    void closeStream(Stream *s) override { auto *st=static_cast<StreamState*>(s); if(st==active) { stopWorker(); active=nullptr; } delete st; }
    size_t getStreamMTU(Stream *) const override { return 16380; }
    int activateStream(Stream *s, const int, const long long, const size_t) override { auto *st=static_cast<StreamState*>(s); if(active && active!=st) return SOAPY_SDR_STREAM_ERROR; active=st; {std::lock_guard<std::mutex> l(fifoMutex); fifo.clear();} startWorker(); return 0; }
    int deactivateStream(Stream *s, const int, const long long) override { if(s!=active) return SOAPY_SDR_STREAM_ERROR; stopWorker(); active=nullptr; return 0; }
    int readStream(Stream *s, void * const *buffs, const size_t n, int &flags, long long &, const long timeoutUs) override {
        auto *st=static_cast<StreamState*>(s); if(st!=active) return SOAPY_SDR_STREAM_ERROR; flags=0; size_t got=0; auto deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(timeoutUs<0?1000000000LL:timeoutUs);
        std::unique_lock<std::mutex> l(fifoMutex); while(fifo.size()<2 && workerRunning) { if(timeoutUs==0) return SOAPY_SDR_TIMEOUT; if(fifoCv.wait_until(l,deadline)==std::cv_status::timeout && fifo.size()<2) return SOAPY_SDR_TIMEOUT; }
        if(fifo.empty()) return workerRunning?SOAPY_SDR_TIMEOUT:SOAPY_SDR_STREAM_ERROR;
        size_t bytes=n*2; std::vector<uint8_t> raw(std::min(bytes,fifo.size())); for(auto &x:raw){x=fifo.front();fifo.pop_front();} got=raw.size()/2;
        if(st->format==SOAPY_SDR_CS8) std::memcpy(buffs[0],raw.data(),got*2); else {float *out=static_cast<float*>(buffs[0]); for(size_t i=0;i<got;i++){out[2*i]=raw[2*i]/128.0f;out[2*i+1]=raw[2*i+1]/128.0f;}}
        return (int)got;
    }

private:
    struct StreamState : Stream { explicit StreamState(std::string f):format(std::move(f)){} std::string format; };
    int fd=-1; std::string path,info,caps; double frequency=2412e6, sampleRate=250e3, bandwidth=13e6, gain=0; unsigned iqDecimation=64, iqRateCode=6, freqMin=100, freqMax=6000,bwMin=13,bwMax=69,gainMin=0,gainMax=82; std::vector<double> rates{15625.0,31250.0,62500.0,125000.0,250000.0,312500.0,625000.0}; bool autoGain=true, burstMode=false;
    StreamState *active=nullptr; std::thread worker; std::atomic<bool> workerRunning{false}, stopRequested{false}; std::mutex controlMutex,stateMutex,fifoMutex; std::condition_variable fifoCv; std::deque<uint8_t> fifo;
    void checkRx(int d,size_t c) const { if(d!=SOAPY_SDR_RX||c!=0) throw std::invalid_argument("ESP-SDR has only RX channel 0"); }
    void selectIqRate(double requested) { auto it=std::min_element(rates.begin(),rates.end(),[requested](double a,double b){return std::abs(a-requested)<std::abs(b-requested);}); sampleRate=*it; if(*it==625000.0){iqRateCode=1;iqDecimation=64;} else if(*it==312500.0){iqRateCode=0;iqDecimation=256;} else if(*it==250000.0){iqRateCode=6;iqDecimation=64;} else if(*it==125000.0){iqRateCode=6;iqDecimation=128;} else if(*it==62500.0){iqRateCode=6;iqDecimation=256;} else if(*it==31250.0){iqRateCode=6;iqDecimation=512;} else {iqRateCode=6;iqDecimation=1024;} }
    static void expectOk(const std::string &s) { if(s.rfind("OK",0)!=0) throw std::runtime_error("ESP-SDR: "+s); }
    std::string command(const std::string &s) { std::lock_guard<std::mutex> l(controlMutex); return commandLocked(s); }
    std::string commandLocked(const std::string &s) { std::string x=s+"\n"; if(::write(fd,x.data(),x.size())!=(ssize_t)x.size()) throw std::runtime_error("ESP-SDR write failed"); return readLine(); }
    std::string readLine() { std::string s; char c; for(;;){ssize_t n=::read(fd,&c,1); if(n==1){if(c=='\n') return s; if(c!='\r') s+=c; if(s.size()>512) throw std::runtime_error("ESP-SDR line too long");} else if(n<0 && errno!=EINTR) throw std::runtime_error("ESP-SDR read failed"); else if(n==0) throw std::runtime_error("ESP-SDR timeout/disconnect");} }
    void readExact(uint8_t *p,size_t n){while(n){ssize_t k=::read(fd,p,n);if(k>0){p+=k;n-=k;}else if(k<0&&errno==EINTR)continue;else throw std::runtime_error("ESP-SDR payload read failed");}}
    void parseLimits(const std::string &s){ std::smatch m; if(std::regex_search(s,m,std::regex("\\\"gain\\\":\\[([0-9]+),([0-9]+)"))){gainMin=std::stoul(m[1]);gainMax=std::stoul(m[2]);} if(std::regex_search(s,m,std::regex("\\\"bandwidth\\\":\\[([0-9]+),([0-9]+)"))){bwMin=std::stoul(m[1]);bwMax=std::stoul(m[2]);} }
    void parseRange(const std::string &s){unsigned step; if(std::sscanf(s.c_str(),"RANGE %u %u %u",&freqMin,&freqMax,&step)!=3) throw std::runtime_error("bad RANGE response");}
    bool captureBurst(std::vector<uint8_t> &payload) { unsigned code=sampleRate>=79e6?0:(sampleRate>=39e6?1:6); std::lock_guard<std::mutex> l(controlMutex); std::string x="CAP16 16380 "+std::to_string(code)+"\n"; if(::write(fd,x.data(),x.size())!=(ssize_t)x.size()) return false; std::string h=readLine(); unsigned n,crc; if(std::sscanf(h.c_str(),"DATA %u %x",&n,&crc)!=2||n==0||n>16380) return false; payload.resize((size_t)n*2); readExact(payload.data(),payload.size()); return (uint32_t)::crc32(0,payload.data(),payload.size())==crc; }
    bool readIQFrame(std::vector<uint8_t> &payload) {
        uint8_t prefix[4]; readExact(prefix, sizeof(prefix));
        if (std::memcmp(prefix, "IQS1", 4) != 0) {
            // A stopped IQS run ends with an ASCII IQSEND report. Consume the
            // complete line so the next command starts on a clean boundary.
            std::string line(reinterpret_cast<char *>(prefix), sizeof(prefix));
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
            while (line.find('\n')==std::string::npos) {
                char c; ssize_t n=::read(fd,&c,1);
                if(n==1) line.push_back(c);
                else if(n<0 && errno==EINTR) continue;
                else if(n==0) { if(std::chrono::steady_clock::now()>=deadline) break; continue; }
                else break;
            }
            return false;
        }
        uint8_t h[24]; std::memcpy(h, prefix, 4); readExact(h+4, sizeof(h)-4);
        uint16_t n=(uint16_t)h[16]|((uint16_t)h[17]<<8); if(!n || n>1024 || h[18]!=8) return false;
        payload.resize((size_t)n*2); readExact(payload.data(), payload.size()); uint8_t crcBytes[4]; readExact(crcBytes,4);
        uint32_t expected=(uint32_t)crcBytes[0]|((uint32_t)crcBytes[1]<<8)|((uint32_t)crcBytes[2]<<16)|((uint32_t)crcBytes[3]<<24);
        uLong crc=::crc32(0,h,sizeof(h)); crc=::crc32(crc,payload.data(),payload.size()); return (uint32_t)crc==expected;
    }
    void startWorker(){if(workerRunning.exchange(true))return;stopRequested=false;worker=std::thread([this]{std::vector<uint8_t> p;try{if(burstMode){while(workerRunning){if(!captureBurst(p)) throw std::runtime_error("invalid CAP16 response");std::lock_guard<std::mutex> l(fifoMutex);if(fifo.size()+p.size()>4*16380*2)fifo.erase(fifo.begin(),fifo.begin()+std::min(fifo.size(),p.size()));fifo.insert(fifo.end(),p.begin(),p.end());fifoCv.notify_all();}}
            else {unsigned dec;{std::lock_guard<std::mutex> l(controlMutex); std::string x="IQS 0 "+std::to_string(iqDecimation)+" 8 "+std::to_string(iqRateCode)+"\n"; if(::write(fd,x.data(),x.size())!=(ssize_t)x.size()) throw std::runtime_error("IQS write failed"); std::string start=readLine(); unsigned rate,bits,shift,mode,mhz; if(std::sscanf(start.c_str(),"IQS %u %u %u %u %u %u",&rate,&dec,&bits,&shift,&mode,&mhz)!=6||bits!=8) throw std::runtime_error("bad IQS start response");}
                while(workerRunning){if(!readIQFrame(p)){if(stopRequested) break; throw std::runtime_error("invalid IQS frame");}std::lock_guard<std::mutex> l(fifoMutex);if(fifo.size()+p.size()>4*1024*2)fifo.erase(fifo.begin(),fifo.begin()+std::min(fifo.size(),p.size()));fifo.insert(fifo.end(),p.begin(),p.end());fifoCv.notify_all();}
                if(stopRequested){while(readIQFrame(p)) {}}
            }}catch(...){workerRunning=false;fifoCv.notify_all();}});}
    void stopWorker(){stopRequested=true;if(workerRunning.exchange(false)){const char stop='\n';(void)::write(fd,&stop,1);fifoCv.notify_all();}if(worker.joinable())worker.join(); tcflush(fd,TCIFLUSH);}
};

KwargsList findESP(const Kwargs &args){Kwargs k; k["driver"]="espsdr"; k["label"]="ESP-SDR ("+(args.count("device")?args.at("device"):"/dev/ttyACM0")+")"; if(args.count("device"))k["device"]=args.at("device"); return {k};}
Device *makeESP(const Kwargs &args){return new ESPDevice(args);}
SoapySDR::Registry registerESP("espsdr", &findESP, &makeESP, SOAPY_SDR_ABI_VERSION);
}
