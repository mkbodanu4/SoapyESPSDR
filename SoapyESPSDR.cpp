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
    Kwargs getHardwareInfo() const override { return {{"firmware", info}, {"transport", "native USB serial/JTAG"}, {"streaming", "burst captures; gaps expected"}}; }
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
    RangeList getSampleRateRange(const int d, const size_t c) const override { checkRx(d,c); return {Range(16e6,80e6)}; }
    void setSampleRate(const int d, const size_t c, const double r) override {
        checkRx(d,c); auto it=std::min_element(rates.begin(),rates.end(),[r](double a,double b){return std::abs(a-r)<std::abs(b-r);});
        // gr-osmosdr may issue a zero/unset rate while constructing its source.
        // Use the safest nominal ESP-SDR rate for that initialization request.
        if (it==rates.end() || !std::isfinite(r) || r<=0.0) it=std::find(rates.begin(), rates.end(), 16e6);
        if (it==rates.end()) throw std::invalid_argument("no supported sample rate");
        std::lock_guard<std::mutex> l(stateMutex); sampleRate=*it; rateIndex=(*it==80e6?0:(*it==40e6?1:6));
    }
    double getSampleRate(const int d, const size_t c) const override { checkRx(d,c); return sampleRate; }

    RangeList getFrequencyRange(const int d, const size_t c) const override { checkRx(d,c); return {Range(freqMin*1e6,freqMax*1e6,1e6)}; }
    void setFrequency(const int d, const size_t c, const double f, const Kwargs & = {}) override {
        checkRx(d,c); if (!std::isfinite(f) || f<freqMin*1e6 || f>freqMax*1e6) throw std::invalid_argument("frequency outside firmware range");
        unsigned mhz=(unsigned)std::llround(f/1e6);
        mhz=std::clamp(mhz,freqMin,freqMax);
        std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("FREQ " + std::to_string(mhz))); frequency=mhz*1e6;
    }
    double getFrequency(const int d, const size_t c) const override { checkRx(d,c); return frequency; }

    std::vector<double> listBandwidths(const int d, const size_t c) const override { checkRx(d,c); std::vector<double> r; for(unsigned i=bwMin;i<=bwMax;i++) r.push_back(i*1e6); return r; }
    RangeList getBandwidthRange(const int d, const size_t c) const override { checkRx(d,c); return {Range(bwMin*1e6,bwMax*1e6,1e6)}; }
    void setBandwidth(const int d, const size_t c, const double b) override { checkRx(d,c); unsigned mhz; if(!std::isfinite(b)||b<=0.0) mhz=bwMax; else mhz=(unsigned)std::llround(b/1e6); mhz=std::clamp(mhz,bwMin,bwMax); std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("BANDWIDTH "+std::to_string(mhz))); bandwidth=mhz*1e6; }
    double getBandwidth(const int d, const size_t c) const override { checkRx(d,c); return bandwidth; }

    std::vector<std::string> listGains(const int d, const size_t c) const override { checkRx(d,c); return {"RF"}; }
    bool hasGainMode(const int d, const size_t c) const override { checkRx(d,c); return true; }
    void setGainMode(const int d, const size_t c, const bool automatic) override { checkRx(d,c); std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked(automatic?"GAIN HARDWARE":"GAIN MANUAL "+std::to_string((unsigned)gain))); autoGain=automatic; }
    bool getGainMode(const int d, const size_t c) const override { checkRx(d,c); return autoGain; }
    void setGain(const int d, const size_t c, const double g) override { checkRx(d,c); if(g<gainMin||g>gainMax) throw std::invalid_argument("gain outside firmware range"); std::lock_guard<std::mutex> l(controlMutex); expectOk(commandLocked("GAIN MANUAL "+std::to_string((unsigned)std::llround(g)))); gain=g; autoGain=false; }
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
    int fd=-1; std::string path,info,caps; double frequency=2412e6, sampleRate=16e6, bandwidth=13e6, gain=0; unsigned rateIndex=6, freqMin=100, freqMax=6000,bwMin=13,bwMax=69,gainMin=0,gainMax=82; std::vector<double> rates{16e6,40e6,80e6}; bool autoGain=true;
    StreamState *active=nullptr; std::thread worker; std::atomic<bool> workerRunning{false}; std::mutex controlMutex,stateMutex,fifoMutex; std::condition_variable fifoCv; std::deque<uint8_t> fifo;
    void checkRx(int d,size_t c) const { if(d!=SOAPY_SDR_RX||c!=0) throw std::invalid_argument("ESP-SDR has only RX channel 0"); }
    static void expectOk(const std::string &s) { if(s.rfind("OK",0)!=0) throw std::runtime_error("ESP-SDR: "+s); }
    std::string command(const std::string &s) { std::lock_guard<std::mutex> l(controlMutex); return commandLocked(s); }
    std::string commandLocked(const std::string &s) { std::string x=s+"\n"; if(::write(fd,x.data(),x.size())!=(ssize_t)x.size()) throw std::runtime_error("ESP-SDR write failed"); return readLine(); }
    std::string readLine() { std::string s; char c; for(;;){ssize_t n=::read(fd,&c,1); if(n==1){if(c=='\n') return s; if(c!='\r') s+=c; if(s.size()>512) throw std::runtime_error("ESP-SDR line too long");} else if(n<0 && errno!=EINTR) throw std::runtime_error("ESP-SDR read failed"); else if(n==0) throw std::runtime_error("ESP-SDR timeout/disconnect");} }
    void readExact(uint8_t *p,size_t n){while(n){ssize_t k=::read(fd,p,n);if(k>0){p+=k;n-=k;}else if(k<0&&errno==EINTR)continue;else throw std::runtime_error("ESP-SDR payload read failed");}}
    void parseLimits(const std::string &s){ std::smatch m; if(std::regex_search(s,m,std::regex("\\\"gain\\\":\\[([0-9]+),([0-9]+)"))){gainMin=std::stoul(m[1]);gainMax=std::stoul(m[2]);} if(std::regex_search(s,m,std::regex("\\\"bandwidth\\\":\\[([0-9]+),([0-9]+)"))){bwMin=std::stoul(m[1]);bwMax=std::stoul(m[2]);} rates.clear(); std::regex rr("\\\"rates\\\":\\[([^]]+)\\]"); if(std::regex_search(s,m,rr)){std::string t=m[1]; size_t p=0; while(p<t.size()){size_t q=t.find(',',p); try { rates.push_back(std::stod(t.substr(p,q==std::string::npos?t.size()-p:q-p))); } catch (...) {} if(q==std::string::npos)break;p=q+1;}} if(rates.empty()) rates={16e6,40e6,80e6}; }
    void parseRange(const std::string &s){unsigned step; if(std::sscanf(s.c_str(),"RANGE %u %u %u",&freqMin,&freqMax,&step)!=3) throw std::runtime_error("bad RANGE response");}
    bool capture(std::vector<uint8_t> &payload){std::lock_guard<std::mutex> l(controlMutex); std::string x="CAP16 16380 "+std::to_string(rateIndex)+"\n"; if(::write(fd,x.data(),x.size())!=(ssize_t)x.size()) return false; std::string h=readLine(); unsigned n; unsigned crc; if(std::sscanf(h.c_str(),"DATA %u %x",&n,&crc)!=2||n==0||n>16380) return false; payload.resize((size_t)n*2); readExact(payload.data(),payload.size()); return ::crc32(0,payload.data(),payload.size())==crc; }
    void startWorker(){if(workerRunning.exchange(true))return;worker=std::thread([this]{std::vector<uint8_t> p;while(workerRunning){try{if(!capture(p)){std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}std::lock_guard<std::mutex> l(fifoMutex);if(fifo.size()+p.size()>4*16380*2)fifo.erase(fifo.begin(),fifo.begin()+std::min(fifo.size(),p.size()));fifo.insert(fifo.end(),p.begin(),p.end());fifoCv.notify_all();}catch(...){workerRunning=false;fifoCv.notify_all();}}});}
    void stopWorker(){if(!workerRunning.exchange(false)){if(worker.joinable())worker.join();return;}fifoCv.notify_all();if(worker.joinable())worker.join();}
};

KwargsList findESP(const Kwargs &args){Kwargs k; k["driver"]="espsdr"; k["label"]="ESP-SDR ("+(args.count("device")?args.at("device"):"/dev/ttyACM0")+")"; if(args.count("device"))k["device"]=args.at("device"); return {k};}
Device *makeESP(const Kwargs &args){return new ESPDevice(args);}
SoapySDR::Registry registerESP("espsdr", &findESP, &makeESP, SOAPY_SDR_ABI_VERSION);
}
