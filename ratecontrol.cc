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

// The threshold T vs. current I dependency is modelled as:
// T = factor * pow(I, power);
// where:
//  - I is the patch current in uA and
//  - T is the threshold needed to keed the overall camera rate stable.
struct threshold_vs_current_fit_parameter
{
    double factor;
    double power;
};

threshold_vs_current_fit_parameter fits_parameters[160] = {
        { 133.013, 0.375834 },
        { 128.099, 0.379173 },
        { 122.742, 0.385934 },
        { 126.484, 0.378278 },
        { 124.888, 0.385421 },
        { 128.446, 0.377539 },
        { 128.835, 0.380569 },
        { 133.943, 0.37223 },
        { 127.869, 0.380412 },
        { 127.561, 0.37882 },
        { 129.938, 0.37441 },
        { 132.453, 0.36959 },
        { 122.688, 0.38506 },
        { 132.523, 0.368778 },
        { 126.869, 0.381342 },
        { 131.274, 0.367824 },
        { 126.334, 0.380328 },
        { 125.131, 0.380225 },
        { 123.636, 0.384097 },
        { 152.458, 0.378981 },
        { 142.583, 0.354521 },
        { 130.987, 0.369479 },
        { 126.518, 0.373482 },
        { 135.726, 0.359904 },
        { 130.756, 0.369985 },
        { 123.898, 0.384574 },
        { 126.815, 0.382375 },
        { 133.12, 0.364099 },
        { 132.314, 0.365364 },
        { 128.083, 0.372832 },
        { 132.313, 0.365847 },
        { 130.793, 0.368687 },
        { 126.886, 0.380878 },
        { 29.9048, 0.683431 },
        { 129.893, 0.37009 },
        { 125.271, 0.375155 },
        { 129.404, 0.372061 },
        { 134.74, 0.367484 },
        { 128.143, 0.372102 },
        { 126.996, 0.368019 },
        { 129.105, 0.368584 },
        { 123.333, 0.379091 },
        { 124.62, 0.376659 },
        { 130.371, 0.3664 },
        { 130.576, 0.366824 },
        { 125.766, 0.372625 },
        { 126.174, 0.36966 },
        { 119.427, 0.377902 },
        { 121.358, 0.389343 },
        { 127.504, 0.377013 },
        { 119.685, 0.383594 },
        { 125.671, 0.376627 },
        { 124.841, 0.385652 },
        { 124.492, 0.382397 },
        { 124.113, 0.3814 },
        { 130.144, 0.368337 },
        { 133.269, 0.362138 },
        { 126.004, 0.372253 },
        { 126.015, 0.37349 },
        { 125.626, 0.376248 },
        { 123.298, 0.383032 },
        { 122.797, 0.380446 },
        { 126.876, 0.369579 },
        { 125.865, 0.369971 },
        { 126.119, 0.378165 },
        { 126.465, 0.373227 },
        { 128.082, 0.368034 },
        { 129.656, 0.36659 },
        { 126.802, 0.373607 },
        { 129.032, 0.369289 },
        { 125.789, 0.376017 },
        { 123.406, 0.377417 },
        { 125.587, 0.372674 },
        { 124.601, 0.373024 },
        { 123.124, 0.377988 },
        { 127.254, 0.373606 },
        { 130.285, 0.368494 },
        { 125.707, 0.378455 },
        { 128.874, 0.36793 },
        { 129.988, 0.3683 },
        { 121.285, 0.379878 },
        { 124.11, 0.376639 },
        { 118.825, 0.383447 },
        { 122.641, 0.378084 },
        { 124.358, 0.378208 },
        { 118.154, 0.388323 },
        { 122.635, 0.382389 },
        { 124.663, 0.377213 },
        { 124.362, 0.378559 },
        { 122.109, 0.387443 },
        { 129.434, 0.370125 },
        { 131.968, 0.361709 },
        { 100.338, 0.42398 },
        { 124.872, 0.373463 },
        { 124.503, 0.37437 },
        { 30.3253, 0.722283 },
        { 21.7886, 0.765998 },
        { 124.599, 0.382034 },
        { 123.205, 0.381387 },
        { 128.994, 0.368062 },
        { 123.267, 0.376835 },
        { 124.145, 0.373479 },
        { 123.343, 0.373145 },
        { 136.856, 0.353926 },
        { 122.261, 0.387211 },
        { 129.808, 0.370719 },
        { 125.961, 0.377113 },
        { 127.699, 0.370337 },
        { 127.324, 0.370959 },
        { 130.438, 0.36132 },
        { 127.274, 0.366389 },
        { 129.735, 0.363779 },
        { 123.186, 0.383019 },
        { 119.729, 0.386525 },
        { 126.548, 0.374928 },
        { 125.882, 0.372251 },
        { 121.207, 0.392908 },
        { 124.44, 0.38904 },
        { 123.748, 0.384949 },
        { 125.283, 0.377073 },
        { 128.193, 0.371632 },
        { 126.473, 0.371209 },
        { 128.88, 0.368648 },
        { 127.842, 0.371151 },
        { 128.731, 0.370886 },
        { 132.286, 0.363405 },
        { 130.26, 0.366119 },
        { 126.392, 0.368475 },
        { 128.674, 0.381107 },
        { 124.848, 0.385849 },
        { 129.608, 0.374135 },
        { 127.953, 0.368464 },
        { 129.033, 0.383137 },
        { 124.124, 0.38623 },
        { 129.477, 0.371357 },
        { 124.915, 0.375954 },
        { 124.599, 0.377758 },
        { 124.998, 0.378663 },
        { 125.853, 0.377599 },
        { 137.324, 0.364129 },
        { 127.941, 0.381429 },
        { 131.787, 0.369858 },
        { 127.341, 0.375803 },
        { 131.761, 0.362445 },
        { 120.351, 0.392628 },
        { 128.575, 0.3733 },
        { 130.817, 0.369728 },
        { 129.714, 0.366464 },
        { 132.365, 0.37093 },
        { 129.231, 0.376199 },
        { 131.433, 0.373933 },
        { 126.149, 0.373766 },
        { 120.712, 0.387427 },
        { 127.684, 0.380379 },
        { 128.479, 0.373847 },
        { 127.451, 0.381095 },
        { 129.811, 0.380881 },
        { 127.573, 0.3816 },
        { 130.984, 0.374114 },
        { 127.381, 0.381882 }
    };

double threshold_from_current(double current, threshold_vs_current_fit_parameter& fit){
    return fit.factor * pow(current, fit.power);
};

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

    vector<bool> has_this_FTU_been_mofified_this_time;
    vector<bool> should_this_FTU_be_ommited_next_time;


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

    uint64_t fCounter;

    Time fCalibrationTimeStart;

    double self_patch_rate_median;
    double self_patch_rate_std;

    double self_board_rate_median;
    double self_board_rate_std;

    vector<uint32_t> fThresholds;

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
        // write the current rates into a json file with the current
        // timestamp that's all

        const int n_total_patches = 160;
        vector<float> board_rates(sdata.fBoardRate, sdata.fBoardRate+40);
        vector<float> patch_rates(sdata.fPatchRate, sdata.fPatchRate+160);

        self_board_rate_median = RateControl::vector_median(board_rates);
        self_patch_rate_median = RateControl::vector_median(patch_rates);
        self_board_rate_std = RateControl::vector_std_from_cdf(board_rates);
        self_patch_rate_std = RateControl::vector_std_from_cdf(patch_rates);

        if (fVerbose)
            Out() << Tools::Form(
                "Boards: Med=%3.1f +- %3.1f Hz   Patches: Med=%3.1f +- %3.1f Hz",
                self_board_rate_median, self_board_rate_std,
                self_patch_rate_median, self_patch_rate_std) << endl;

        if (self_board_rate_median){
            Out() << "Median Board Rate is zero, something is wrong" << endl;
            return;
        }

        if (self_patch_rate_median){
            Out() << "Patch Board Rate is zero, something is wrong" << endl;
            return;
        }

        if (self_board_rate_std){
            Out() << "Board Rate std deviation is zero, something is wrong" << endl;
            return;
        }

        if (self_patch_rate_std){
            Out() << "Patch Rate std deviation is zero, something is wrong" << endl;
            return;
        }


        bool changed = false;

        for (int patch_id=0; patch_id < n_total_patches; patch_id++){
            float this_patch_rate = sdata.fPatchRate[patch_id];
            int board_id = patch_id / 4;
            if (should_this_FTU_be_ommited_next_time[board_id]){
                continue;
            }
            // Adjust thresholds of all patches towards the median patch rate
            float step;
            if (this_patch_rate < self_patch_rate_median)
            {
                 step = (
                    log10(this_patch_rate)
                    - log10(self_patch_rate_median + 3.5 * self_patch_rate_std)
                    ) / 0.039;
            } else {
                step =  -1.5 * (log10(self_patch_rate_median + self_patch_rate_std) - log10(self_patch_rate_median))/0.039;
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

        if (GetCurrentState()==RateControl::State::kSettingGlobalThreshold)
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
        //const FTM::DimTriggerRates &sdata = *static_cast<const FTM::DimTriggerRates*>(evt.GetData());
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
            if (time - fCurrentsTime.front() < boost::posix_time::seconds(fAverageTime))
                break;

            fCurrentsTime.pop_front();
            fCurrentsMed.pop_front();
            fCurrentsDev.pop_front();
            fCurrentsVec.pop_front();
        }


        // from the history of the last 10 seconds, we have a list
        // of N, 320 bias current values. We want to assign them to
        // their respective patches and kind of average over the last 10 seconds
        // so we sum these currents up and devide by the number of
        // entries of fCurrentsVec and the number of pixels a patch has.
        vector<double> patch_currents(160);
        for (auto currents=fCurrentsVec.begin(); currents!=fCurrentsVec.end(); currents++)
            for (int i=0; i<320; i++)
            {
                const PixelMapEntry &hv = fMap.hv(i);
                if (hv)
                    patch_currents[hv.hw()/9] += (*currents)[i] * hv.count() / (fCurrentsVec.size()*9);
            }


        for (int i=0; i<160; i++)
        {
            fThresholds[i] = uint32_t(threshold_from_current(patch_currents[i], fits_parameters[i]));
        }

        Dim::SendCommandNB("FTM_CONTROL/SET_ALL_THRESHOLDS", fThresholds);


        const RateControl::DimThreshold data = { 10, fCalibrationTimeStart.Mjd(), Time().Mjd() };
        fDimThreshold.setQuality(2);
        fDimThreshold.Update(data);

        return RateControl::State::kSettingGlobalThreshold;
    }

    int CalibrateRun(const EventImp &evt)
    {
        Info("Starting to control thresholds.");
        return RateControl::State::kInProgress;
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

void PrintUsage() {
    cout <<
        "The ratecontrol program is a keep the rate reasonable low.\n"
        "\n"
        "Usage: ratecontrol [-c type] [OPTIONS]\n"
        "  or:  ratecontrol [OPTIONS]\n";
    cout << endl;
}

void PrintHelp() {
    Main::PrintHelp<StateMachineRateControl>();
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


