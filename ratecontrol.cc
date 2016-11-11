#include <valarray>

#include "Dim.h"
#include "Event.h"
#include "Shell.h"
#include "StateMachineDim.h"
#include "Connection.h"
#include "Configuration.h"
#include "Console.h"
#include "externals/PixelMap.h"

#include "tools.h"

#include "LocalControl.h"

#include "HeadersFTM.h"
#include "HeadersLid.h"
#include "HeadersDrive.h"
#include "HeadersRateScan.h"
#include "HeadersRateControl.h"

namespace ba    = boost::asio;
namespace bs    = boost::system;
namespace dummy = ba::placeholders;

using namespace std;

// ------------------------------------------------------------------------

#include "DimDescriptionService.h"
#include "DimState.h"

// ------------------------------------------------------------------------

class StateMachineRateControl : public StateMachineDim//, public DimInfoHandler
{
private:
    struct config
    {
        uint16_t fCalibrationType;
        uint16_t fTargetRate;
        uint16_t fMinThreshold;
        uint16_t fAverageTime;
        uint16_t fRequiredEvents;
    };

    map<string, config> fRunTypes;

    PixelMap fMap;

    bool fPhysTriggerEnabled;
    bool fTriggerOn;

    DimVersion fDim;
    DimDescribedState fDimFTM;
    DimDescribedState fDimRS;
    DimDescribedState fDimLid;
    DimDescribedState fDimDrive;

    DimDescribedService fDimThreshold;

    float  fTargetRate;
    float  fTriggerRate;

    uint16_t fThresholdMin;
    uint16_t fThresholdReference;

    uint16_t fAverageTime;
    uint16_t fRequiredEvents;

    list<Time> fCurrentsTime;
    list<float> fCurrentsMed;
    list<float> fCurrentsDev;
    list<vector<float>> fCurrentsVec;

    bool fVerbose;
    bool fCalibrateByCurrent;

    uint64_t fCounter;

    Time fCalibrationTimeStart;

    double self_patch_rate_median;
    double self_patch_rate_std;

    double self_board_rate_median;
    double self_board_rate_std;


    vector<bool> should_this_FTU_be_ommited_next_time;
    vector<bool> has_this_FTU_been_mofified_this_time;


    bool CheckEventSize(const EventImp &evt, size_t size)
    {
        if (size_t(evt.GetSize())==size)
            return true;

        if (evt.GetSize()==0)
            return false;

        ostringstream msg;
        msg << evt.GetName() << " - Received event has " << evt.GetSize() << " bytes, but expected " << size << ".";
        Fatal(msg);
        return false;
    }

    vector<uint32_t> fThresholds;

    void PrintThresholds(const FTM::DimStaticData &sdata)
    {
        if (!fVerbose)
            return;

        if (fThresholds.empty())
            return;

        if (GetCurrentState()<=RateControl::State::kConnected)
            return;
        Out() << "Min. DAC=" << fThresholdMin << endl;
        for (int j=0; j<10; j++) {
            for (int k=0; k<4; k++) {
                for (int i=0; i<4; i++) {
                    const int p = i + k*4 + j*16;
                    if (fThresholds[p]!=fThresholdMin)
                        Out() << setw(3) << fThresholds[p];
                    else
                        Out() << " - ";

                    if (fThresholds[p]!=sdata.fThreshold[p])
                        Out() << "!";
                    else
                        Out() << " ";
                }
                Out() << "   ";
            }
            Out() << endl;
        }
        Out() << endl;
    }

    bool Step(int patch_id, float step)
    {
        uint32_t diff = fThresholds[patch_id]+int16_t(truncf(step));
        if (diff<fThresholdMin)
            diff=fThresholdMin;
        if (diff>0xffff)
            diff = 0xffff;

        if (diff==fThresholds[patch_id])
            return false;

        if (fVerbose)
        {
            Out() << "Apply: Patch " << setw(3) << patch_id << " [" << patch_id/40 << "|" << (patch_id/4)%10 << "|" << patch_id%4 << "]";
            Out() << (step>0 ? " += " : " -= ");
            Out() << fabs(step) << " (old=" << fThresholds[patch_id] << ", new=" << diff << ")" << endl;
        }

        fThresholds[patch_id] = diff;
        return true;
    }

    void ProcessPatches(const FTM::DimTriggerRates &sdata)
    {
        const int n_total_patches = 160;

        // Caluclate Median and deviation
        vector<float> medb(sdata.fBoardRate, sdata.fBoardRate+40);
        vector<float> medp(sdata.fPatchRate, sdata.fPatchRate+160);

        sort(medb.begin(), medb.end());
        sort(medp.begin(), medp.end());

        vector<float> devb(40);
        for (int i=0; i<40; i++)
            devb[i] = fabs(sdata.fBoardRate[i]-medb[i]);

        vector<float> devp(160);
        for (int i=0; i<160; i++)
            devp[i] = fabs(sdata.fPatchRate[i]-medp[i]);

        sort(devb.begin(), devb.end());
        sort(devp.begin(), devp.end());

        self_board_rate_median = (medb[19]+medb[20])/2;
        if (self_board_rate_median){
            Out() << "Median Board Rate is zero, something is wrong" << endl;
            return;
        }

        self_patch_rate_median = (medp[79]+medp[80])/2;
        if (self_patch_rate_median){
            Out() << "Patch Board Rate is zero, something is wrong" << endl;
            return;
        }

        self_board_rate_std = devb[27];
        if (self_board_rate_std){
            Out() << "Board Rate std deviation is zero, something is wrong" << endl;
            return;
        }

        self_patch_rate_std = devp[109];
        if (self_patch_rate_std){
            Out() << "Patch Rate std deviation is zero, something is wrong" << endl;
            return;
        }

        if (fVerbose)
            Out() << Tools::Form(
                "Boards: Med=%3.1f +- %3.1f Hz   Patches: Med=%3.1f +- %3.1f Hz",
                self_board_rate_median, self_board_rate_std,
                self_patch_rate_median, self_patch_rate_std) << endl;

        bool changed = false;

        for (int patch_id=0; patch_id < n_total_patches; patch_id++){
            float this_patch_rate = sdata.fPatchRate[patch_id];
            int board_id = patch_id / 4;
            if (should_this_FTU_be_ommited_next_time[board_id]){
                continue;
            }
            // Adjust thresholds of all patches towards the median patch rate
            if (this_patch_rate < self_patch_rate_median)
            {
                const float step = (
                    log10(this_patch_rate)
                    - log10(self_patch_rate_median + 3.5 * self_patch_rate_std)
                    ) / 0.039;
            } else {
                const float step =  -1.5 * (log10(self_patch_rate_median + self_patch_rate_std) - log10(self_patch_rate_median))/0.039;
            }
            changed |= Step(patch_id, step);
            has_this_FTU_been_mofified_this_time[board_id] = true;
        }
        should_this_FTU_be_ommited_next_time = has_this_FTU_been_mofified_this_time;

        if (changed)
            Dim::SendCommandNB("FTM_CONTROL/SET_SELECTED_THRESHOLDS", fThresholds);
    }

    int HandleStaticData(const EventImp &evt)
    {
        if (!CheckEventSize(evt, sizeof(FTM::DimStaticData)))
            return GetCurrentState();

        const FTM::DimStaticData &sdata = *static_cast<const FTM::DimStaticData*>(evt.GetData());
        fPhysTriggerEnabled = sdata.HasTrigger();
        fTriggerOn = (evt.GetQoS()&FTM::kFtmStates)==FTM::kFtmRunning;

        Out() << "\n" << evt.GetTime() << ": " << (bool)fTriggerOn << " " << (bool)fPhysTriggerEnabled << endl;
        PrintThresholds(sdata);

        if (GetCurrentState()==RateControl::State::kSettingGlobalThreshold && fCalibrateByCurrent)
        {
            if (fThresholds.empty())
                return RateControl::State::kSettingGlobalThreshold;

            if (!std::equal(sdata.fThreshold, sdata.fThreshold+160, fThresholds.begin()))
                return RateControl::State::kSettingGlobalThreshold;

            return RateControl::State::kGlobalThresholdSet;
        }

        fThresholds.assign(sdata.fThreshold, sdata.fThreshold+160);

        return GetCurrentState();
    }

    int HandleTriggerRates(const EventImp &evt)
    {
        fTriggerOn = (evt.GetQoS()&FTM::kFtmStates)==FTM::kFtmRunning;

        if (fThresholds.empty())
            return GetCurrentState();

        if (GetCurrentState()<=RateControl::State::kConnected ||
            GetCurrentState()==RateControl::State::kGlobalThresholdSet)
            return GetCurrentState();

        if (!CheckEventSize(evt, sizeof(FTM::DimTriggerRates)))
            return GetCurrentState();

        const FTM::DimTriggerRates &sdata = *static_cast<const FTM::DimTriggerRates*>(evt.GetData());

        if (GetCurrentState()==RateControl::State::kInProgress)
            ProcessPatches(sdata);

        return GetCurrentState();
    }

    int HandleCalibratedCurrents(const EventImp &evt)
    {
        // Check if received event is valid
        if (!CheckEventSize(evt, (2*416+8)*4))
            return GetCurrentState();

        // Record only currents when the drive is tracking to avoid
        // bias from the movement
        if (fDimDrive.state()<Drive::State::kTracking || fDimLid.state()==Lid::State::kClosed)
            return GetCurrentState();

        // Get time and median current (FIXME: check N?)
        const Time &time = evt.GetTime();
        const float med  = evt.Get<float>(416*4+4+4);
        const float dev  = evt.Get<float>(416*4+4+4+4);
        const float *cur = evt.Ptr<float>();

        // Keep all median currents of the past 10 seconds
        fCurrentsTime.emplace_back(time);
        fCurrentsMed.emplace_back(med);
        fCurrentsDev.emplace_back(dev);
        fCurrentsVec.emplace_back(vector<float>(cur, cur+320));
        while (!fCurrentsTime.empty())
        {
            if (time-fCurrentsTime.front() < boost::posix_time::seconds(fAverageTime))
                break;

            fCurrentsTime.pop_front();
            fCurrentsMed.pop_front();
            fCurrentsDev.pop_front();
            fCurrentsVec.pop_front();
        }

        // If we are not doing a calibration no further action necessary
        if (!fCalibrateByCurrent)
            return GetCurrentState();

        // We are not setting thresholds at all
        if (GetCurrentState()!=RateControl::State::kSettingGlobalThreshold)
            return GetCurrentState();

        // Target thresholds have been assigned already
        if (!fThresholds.empty())
            return GetCurrentState();

        // We want at least 8 values for averaging
        if (fCurrentsMed.size()<fRequiredEvents)
            return GetCurrentState();


        double avg = RateControl::vector_mean(fCurrentsMed);
        double avg_dev = RateControl::vector_std(fCurrentsMed);

        // from the history of the last 10 seconds, we have a list
        // of N, 320 bias current values. We want to assign them to
        // their respective patches and kind of average over the last 10 seconds
        // so we sum these currents up and devide by the number of
        // entries of fCurrentsVec and the number of pixels a patch has.
        vector<double> patch_currents(160);
        for (auto it=fCurrentsVec.begin(); it!=fCurrentsVec.end(); it++)
            for (int i=0; i<320; i++)
            {
                const PixelMapEntry &hv = fMap.hv(i);
                if (hv)
                    patch_currents[hv.hw()/9] += (*it)[i]*hv.count() / (fCurrentsVec.size()*9);
            }


        fThresholdMin = max(uint16_t(156.3*pow(avg, 0.3925)+1), fThresholdReference);
        fThresholds.assign(160, fThresholdMin);

        int number_of_individual_thresholds_set = 0;
        double avg2 = RateControl::vector_mean(patch_currents);
        for (int i=0; i<160; i++)
        {
            if (patch_currents[i] > avg+3.5*avg_dev)
            {
                number_of_individual_thresholds_set += 1;
                fThresholds[i] = max(uint16_t(40.5*pow(patch_currents[i], 0.642)+164), fThresholdMin);
            }
        }



        Dim::SendCommandNB("FTM_CONTROL/SET_ALL_THRESHOLDS", fThresholds);


        const RateControl::DimThreshold data = { fThresholdMin, fCalibrationTimeStart.Mjd(), Time().Mjd() };
        fDimThreshold.setQuality(2);
        fDimThreshold.Update(data);

        ostringstream out;
        out << setprecision(3);
        out << "Measured average current " << avg << "uA +- " << avg_dev << "uA [N=" << fCurrentsMed.size() << "]... minimum threshold set to " << fThresholdMin;
        Info(out);
        Info("Set "+to_string(number_of_individual_thresholds_set)+" individual thresholds.");

        fTriggerOn = false;
        fPhysTriggerEnabled = false;

        return RateControl::State::kSettingGlobalThreshold;
    }

    int CalibrateByCurrent()
    {
        fCounter = 0;
        fCalibrateByCurrent = true;
        fCalibrationTimeStart = Time();
        has_this_FTU_been_mofified_this_time.assign(40, false);
        should_this_FTU_be_ommited_next_time.assign(40, false);

        fThresholds.clear();

        ostringstream out;
        out << "Rate calibration by current with min. threshold of " << fThresholdReference << ".";
        Info(out);

        return RateControl::State::kSettingGlobalThreshold;
    }

    int CalibrateRun(const EventImp &evt)
    {
        const string name = evt.GetText();

        auto it = fRunTypes.find(name);
        if (it==fRunTypes.end())
        {
            Info("CalibrateRun - Run-type '"+name+"' not found... trying 'default'.");

            it = fRunTypes.find("default");
            if (it==fRunTypes.end())
            {
                Error("CalibrateRun - Run-type 'default' not found.");
                return GetCurrentState();
            }
        }

        const config &conf = it->second;

        if (conf.fCalibrationType!=0)
        {

            if (!fPhysTriggerEnabled)
            {
                Info("Calibration requested, but physics trigger not enabled... CALIBRATE command ignored.");

                fTriggerOn = false;
                fPhysTriggerEnabled = false;
                return RateControl::State::kGlobalThresholdSet;
            }

            if (fDimLid.state()==Lid::State::kClosed)
            {
                Info("Calibration requested, but lid closed... setting all thresholds to "+to_string(conf.fMinThreshold)+".");

                const int32_t val[2] = { -1, conf.fMinThreshold };
                Dim::SendCommandNB("FTM_CONTROL/SET_THRESHOLD", val);

                fThresholds.assign(160, conf.fMinThreshold);

                const double mjd = Time().Mjd();

                const RateControl::DimThreshold data = { conf.fMinThreshold, mjd, mjd };
                fDimThreshold.setQuality(3);
                fDimThreshold.Update(data);

                fCalibrateByCurrent = true;
                fTriggerOn = false;
                fPhysTriggerEnabled = false;
                return RateControl::State::kSettingGlobalThreshold;
            }

            if (fDimDrive.state()<Drive::State::kMoving)
                Warn("Calibration requested, but drive not even moving...");
        }

        switch (conf.fCalibrationType)
        {
        case 0:
            Info("No calibration requested.");
            fTriggerOn = false;
            fPhysTriggerEnabled = false;
            return RateControl::State::kGlobalThresholdSet;
            break;

        case 2:
            fThresholdReference = conf.fMinThreshold;
            fAverageTime = conf.fAverageTime;
            fRequiredEvents = conf.fRequiredEvents;
            return CalibrateByCurrent();
        }

        Error("CalibrateRun - Calibration type "+to_string(conf.fCalibrationType)+" unknown.");
        return GetCurrentState();
    }

    int StopRC()
    {
        Info("Stop received.");
        return RateControl::State::kConnected;
    }

    int SetMinThreshold(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 4))
            return kSM_FatalError;

        // FIXME: Check missing

        fThresholdReference = evt.GetUShort();

        return GetCurrentState();
    }

    int SetTargetRate(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 4))
            return kSM_FatalError;

        fTargetRate = evt.GetFloat();

        return GetCurrentState();
    }

    int Print() const
    {
        Out() << fDim << endl;
        Out() << fDimFTM << endl;
        Out() << fDimRS << endl;
        Out() << fDimLid << endl;
        Out() << fDimDrive << endl;

        return GetCurrentState();
    }

    int SetVerbosity(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 1))
            return kSM_FatalError;

        fVerbose = evt.GetBool();

        return GetCurrentState();
    }

    int Execute()
    {
        if (!fDim.online())
            return RateControl::State::kDimNetworkNA;

        // All subsystems are not connected
        if (fDimFTM.state()<FTM::State::kConnected || fDimDrive.state()<Drive::State::kConnected)
            return RateControl::State::kDisconnected;

        // Do not allow any action while a ratescan is configured or in progress
        if (fDimRS.state()>=RateScan::State::kConfiguring)
            return RateControl::State::kConnected;

        switch (GetCurrentState())
        {
        case RateControl::State::kSettingGlobalThreshold:
            return RateControl::State::kSettingGlobalThreshold;

        case RateControl::State::kGlobalThresholdSet:

            // Wait for the trigger to get switched on before starting control loop
            if (fTriggerOn && fPhysTriggerEnabled)
                return RateControl::State::kInProgress;

            return RateControl::State::kGlobalThresholdSet;

        case RateControl::State::kInProgress:

            // Go back to connected when the trigger has been switched off
            if (!fTriggerOn || !fPhysTriggerEnabled)
                return RateControl::State::kConnected;

            return RateControl::State::kInProgress;
        }

        return RateControl::State::kConnected;
    }

public:
    StateMachineRateControl(ostream &out=cout) : StateMachineDim(out, "RATE_CONTROL"),
        fPhysTriggerEnabled(false), fTriggerOn(false),
        has_this_FTU_been_mofified_this_time(40),
        should_this_FTU_be_ommited_next_time(40),
        fDimFTM("FTM_CONTROL"),
        fDimRS("RATE_SCAN"),
        fDimLid("LID_CONTROL"),
        fDimDrive("DRIVE_CONTROL"),
        fDimThreshold("RATE_CONTROL/THRESHOLD", "S:1;D:1;D:1",
                      "Resulting threshold after calibration"
                      "|threshold[dac]:Resulting threshold from calibration"
                      "|begin[mjd]:Start time of calibration"
                      "|end[mjd]:End time of calibration")
    {
        // ba::io_service::work is a kind of keep_alive for the loop.
        // It prevents the io_service to go to stopped state, which
        // would prevent any consecutive calls to run()
        // or poll() to do nothing. reset() could also revoke to the
        // previous state but this might introduce some overhead of
        // deletion and creation of threads and more.

        fDim.Subscribe(*this);
        fDimFTM.Subscribe(*this);
        fDimRS.Subscribe(*this);
        fDimLid.Subscribe(*this);
        fDimDrive.Subscribe(*this);

        Subscribe("FTM_CONTROL/TRIGGER_RATES")
            (bind(&StateMachineRateControl::HandleTriggerRates, this, placeholders::_1));
        Subscribe("FTM_CONTROL/STATIC_DATA")
            (bind(&StateMachineRateControl::HandleStaticData,   this, placeholders::_1));
        Subscribe("FEEDBACK/CALIBRATED_CURRENTS")
            (bind(&StateMachineRateControl::HandleCalibratedCurrents, this, placeholders::_1));

        // State names
        AddStateName(RateControl::State::kDimNetworkNA, "DimNetworkNotAvailable",
                     "The Dim DNS is not reachable.");

        AddStateName(RateControl::State::kDisconnected, "Disconnected",
                     "The Dim DNS is reachable, but the required subsystems are not available.");

        AddStateName(RateControl::State::kConnected, "Connected",
                     "All needed subsystems are connected to their hardware, no action is performed.");

        AddStateName(RateControl::State::kSettingGlobalThreshold, "Calibrating",
                     "A global minimum threshold is currently determined.");

        AddStateName(RateControl::State::kGlobalThresholdSet, "GlobalThresholdSet",
                     "A global threshold has ben set, waiting for the trigger to be switched on.");

        AddStateName(RateControl::State::kInProgress, "InProgress",
                     "Rate control in progress.");

        AddEvent("CALIBRATE_BY_CURRENT")
            (bind(&StateMachineRateControl::CalibrateByCurrent, this))
            ("Set the global threshold from the median current");

        AddEvent("CALIBRATE_RUN", "C")
            (bind(&StateMachineRateControl::CalibrateRun, this, placeholders::_1))
            ("Start a threshold calibration as defined in the setup for this run-type, state change to InProgress is delayed until trigger enabled");

        AddEvent("STOP", RateControl::State::kSettingGlobalThreshold, RateControl::State::kGlobalThresholdSet, RateControl::State::kInProgress)
            (bind(&StateMachineRateControl::StopRC, this))
            ("Stop a calibration or ratescan in progress");

        AddEvent("SET_MIN_THRESHOLD", "I:1")
            (bind(&StateMachineRateControl::SetMinThreshold, this, placeholders::_1))
            ("Set a minimum threshold at which th rate control starts calibrating");

        AddEvent("SET_TARGET_RATE", "F:1")
            (bind(&StateMachineRateControl::SetTargetRate, this, placeholders::_1))
            ("Set a target trigger rate for the calibration");

        AddEvent("PRINT")
            (bind(&StateMachineRateControl::Print, this))
            ("Print current status");

        AddEvent("SET_VERBOSE", "B")
            (bind(&StateMachineRateControl::SetVerbosity, this, placeholders::_1))
            ("set verbosity state"
             "|verbosity[bool]:disable or enable verbosity for received data (yes/no), except dynamic data");

    }

    bool GetConfig(Configuration &conf, const string &name, const string &sub, uint16_t &rc)
    {
        if (conf.HasDef(name, sub))
        {
            rc = conf.GetDef<uint16_t>(name, sub);
            return true;
        }

        Error("Neither "+name+"default nor "+name+sub+" found.");
        return false;
    }

    int EvalOptions(Configuration &conf)
    {
        fVerbose = !conf.Get<bool>("quiet");

        if (!fMap.Read(conf.Get<string>("pixel-map-file")))
        {
            Error("Reading mapping table from "+conf.Get<string>("pixel-map-file")+" failed.");
            return 1;
        }

        fThresholdReference = 300;
        fThresholdMin       = 300;
        fTargetRate         =  75;

        fAverageTime        =  10;
        fRequiredEvents     =   8;

        // ---------- Setup run types ---------
        const vector<string> types = conf.Vec<string>("run-type");
        if (types.empty())
            Warn("No run-types defined.");
        else
            Message("Defining run-types");

        for (auto it=types.begin(); it!=types.end(); it++)
        {
            Message(" -> "+ *it);

            if (fRunTypes.count(*it)>0)
            {
                Error("Run-type "+*it+" defined twice.");
                return 1;
            }

            config &c = fRunTypes[*it];
            if (!GetConfig(conf, "calibration-type.", *it, c.fCalibrationType) ||
                !GetConfig(conf, "target-rate.",      *it, c.fTargetRate)      ||
                !GetConfig(conf, "min-threshold.",    *it, c.fMinThreshold)    ||
                !GetConfig(conf, "average-time.",     *it, c.fAverageTime)     ||
                !GetConfig(conf, "required-events.",  *it, c.fRequiredEvents))
                return 2;
        }

        return -1;
    }
};

// ------------------------------------------------------------------------

#include "Main.h"

template<class T>
int RunShell(Configuration &conf)
{
    return Main::execute<T, StateMachineRateControl>(conf);
}

void SetupConfiguration(Configuration &conf)
{
    po::options_description control("Rate control options");
    control.add_options()
        ("quiet,q", po_bool(),  "Disable printing more informations during rate control.")
        ("pixel-map-file", var<string>()->required(), "Pixel mapping file. Used here to get the default reference voltage.")
       //("max-wait",   var<uint16_t>(150), "The maximum number of seconds to wait to get the anticipated resolution for a point.")
       // ("resolution", var<double>(0.05) , "The minimum resolution required for a single data point.")
        ;

    conf.AddOptions(control);

    po::options_description runtype("Run type configuration");
    runtype.add_options()
        ("run-type",           vars<string>(),  "Name of run-types (replace the * in the following configuration by the case-sensitive names defined here)")
        ("calibration-type.*", var<uint16_t>(), "Calibration type (0: none, 1: by rate, 2: by current)")
        ("target-rate.*",      var<uint16_t>(), "Target rate for calibration by rate")
        ("min-threshold.*",    var<uint16_t>(), "Minimum threshold which can be applied in a calibration")
        ("average-time.*",     var<uint16_t>(), "Time in seconds to average the currents for a calibration by current.")
        ("required-events.*",  var<uint16_t>(), "Number of required current events to start a calibration by current.");
    ;

    conf.AddOptions(runtype);
}

/*
 Extract usage clause(s) [if any] for SYNOPSIS.
 Translators: "Usage" and "or" here are patterns (regular expressions) which
 are used to match the usage synopsis in program output.  An example from cp
 (GNU coreutils) which contains both strings:
  Usage: cp [OPTION]... [-T] SOURCE DEST
    or:  cp [OPTION]... SOURCE... DIRECTORY
    or:  cp [OPTION]... -t DIRECTORY SOURCE...
 */
void PrintUsage()
{
    cout <<
        "The ratecontrol program is a keep the rate reasonable low.\n"
        "\n"
        "Usage: ratecontrol [-c type] [OPTIONS]\n"
        "  or:  ratecontrol [OPTIONS]\n";
    cout << endl;
}

void PrintHelp()
{
    Main::PrintHelp<StateMachineRateControl>();

    /* Additional help text which is printed after the configuration
     options goes here */

    /*
     cout << "bla bla bla" << endl << endl;
     cout << endl;
     cout << "Environment:" << endl;
     cout << "environment" << endl;
     cout << endl;
     cout << "Examples:" << endl;
     cout << "test exam" << endl;
     cout << endl;
     cout << "Files:" << endl;
     cout << "files" << endl;
     cout << endl;
     */
}

int main(int argc, const char* argv[])
{
    Configuration conf(argv[0]);
    conf.SetPrintUsage(PrintUsage);
    Main::SetupConfiguration(conf);
    SetupConfiguration(conf);

    if (!conf.DoParse(argc, argv, PrintHelp))
        return 127;

    if (!conf.Has("console"))
        return RunShell<LocalStream>(conf);

    if (conf.Get<int>("console")==0)
        return RunShell<LocalShell>(conf);
    else
        return RunShell<LocalConsole>(conf);

    return 0;
}


