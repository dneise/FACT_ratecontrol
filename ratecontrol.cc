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
#include "HeadersFeedback.h"
#include "HeadersRateControl.h"

namespace ba    = boost::asio;
namespace bs    = boost::system;
namespace dummy = ba::placeholders;

using namespace std;

// ------------------------------------------------------------------------

#include "DimDescriptionService.h"
#include "DimState.h"

#include "threshold_from_currents.cpp"

// ------------------------------------------------------------------------

class StateMachineRateControl : public StateMachineDim
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

    DimVersion fDim;
    DimDescribedState fDimFTM;
    DimDescribedState fDimRS;
    DimDescribedService fDimThreshold;

    bool fVerbose;
    bool fPhysTriggerEnabled;
    bool fTriggerOn;

    deque< vector<double>> fHistoricCurrents;
    vector<uint32_t> fLastThresholds;

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


    int HandleStaticData(const EventImp &evt) {
        if (!CheckEventSize(evt, sizeof(FTM::DimStaticData))) return GetCurrentState();

        const FTM::DimStaticData &sdata = *static_cast<const FTM::DimStaticData*>(evt.GetData());

        fPhysTriggerEnabled = sdata.HasTrigger();
        fTriggerOn = (evt.GetQoS()&FTM::kFtmStates)==FTM::kFtmRunning;
        return GetCurrentState();
    }

    int HandleTriggerRates(const EventImp &evt) {
        //const FTM::DimTriggerRates &sdata = *static_cast<const FTM::DimTriggerRates*>(evt.GetData());
        return GetCurrentState();
    }

    int HandleCalibratedCurrents(const EventImp &evt) {
        if (!CheckEventSize(evt, sizeof(Feedback::CalibratedCurrentsData))) return GetCurrentState();

        const Feedback::CalibratedCurrentsData &calibrated_currents = (
            *static_cast<const Feedback::CalibratedCurrentsData*>(evt.GetData()) );

        vector<double> bias_currents(
            calibrated_currents.I,
            calibrated_currents.I + BIAS::kNumChannels);

        auto median_bias_currents = CalcRunningMedianWith(bias_currents);
        auto bias_patch_thresholds = CalcThresholdsFromCurrents(median_bias_currents);
        auto trigger_patch_thresholds = CombineThresholds(bias_patch_thresholds);

        if (GetCurrentState() == RateControl::State::kInProgress){
            SetThresholds(trigger_patch_thresholds);
            fLastThresholds = trigger_patch_thresholds;
        }
        return GetCurrentState();
    }

    vector<double>
    CalcRunningMedianWith(const vector<double>& bias_currents){
        const unsigned int history_length = 3;
        // append bias_currents to history
        fHistoricCurrents.push_back(bias_currents);
        while (fHistoricCurrents.size() > history_length){
            fHistoricCurrents.pop_front();
        }

        // calculate running median on history
        vector<double> medians(BIAS::kNumChannels, 0.);

        for (unsigned int b_id=0; b_id < BIAS::kNumChannels; b_id++){
            vector<double> buffer;
            for(auto it=fHistoricCurrents.begin(); it<fHistoricCurrents.end(); it++){
                buffer.push_back(it->at(b_id));
            }
            medians[b_id] = RateControl::vector_median(buffer);
        }

        return move(medians);
    }

    vector<uint32_t>
    CombineThresholds(const vector<uint32_t>& bias_patch_thresholds){
        // t : TriggerPatch ID
        // b_4 : BiasPatch ID of associated bias patch with 4 pixel
        // b_5 : BiasPatch ID of associated bias patch with 5 pixel
        const int pixel_per_patch = 9;
        vector<uint32_t> trigger_patch_thresholds(160, 0);
        for(unsigned int t=0; t < trigger_patch_thresholds.size(); t++){
            const int b_4 = fMap.hw(t*pixel_per_patch).hv();
            const int b_5 = fMap.hw(t*pixel_per_patch + pixel_per_patch/2).hv();
            trigger_patch_thresholds[t] = max(b_4, b_5);
        }
        // exceptions for broken patches:
        // ---------------------------------------------------------------
        // We have 4 bias patches, which are broken, i.e. the current vs threshold
        // dependency was not fittable.
        // Luckily the neighboring bias patch works and can be used for setting
        // the threshold. In case there is a star in the broken patch, this patch
        // will fire like crazy since the threshold is derived from the neighboring
        // patch, which has a much lower current.
        trigger_patch_thresholds[19] = bias_patch_thresholds[39];  // 38 is dead
        trigger_patch_thresholds[33] = bias_patch_thresholds[67];  // 66 is crazy
        trigger_patch_thresholds[95] = bias_patch_thresholds[190];  // 191 is crazy
        trigger_patch_thresholds[96] = bias_patch_thresholds[192];  // 193 is crazy
        return move(trigger_patch_thresholds);
    }

    void SetThresholds(vector<uint32_t>& thresholds){
        if (fTriggerOn){
            Dim::SendCommandNB("FTM_CONTROL/SET_SELECTED_THRESHOLDS", thresholds);
        } else {
            Dim::SendCommandNB("FTM_CONTROL/SET_ALL_THRESHOLDS", thresholds);
        }
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

    int Print() const
    {
        Out() << fDim << endl;
        Out() << fDimFTM << endl;
        Out() << fDimRS << endl;
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
        if ( fDimFTM.state() < FTM::State::kConnected )
        {
            return RateControl::State::kDisconnected;
        }

        // Do not allow any action while a ratescan is configured or in progress
        if (fDimRS.state()>=RateScan::State::kConfiguring)
            return RateControl::State::kConnected;

	if (GetCurrentState() < RateControl::State::kConnected)
	    return RateControl::State::kConnected;

        return GetCurrentState();
    }

public:
    StateMachineRateControl(ostream &out=cout) : StateMachineDim(out, "RATE_CONTROL"),
        fDimFTM("FTM_CONTROL"),
        fDimRS("RATE_SCAN"),
        fDimThreshold("RATE_CONTROL/THRESHOLD", "S:1;D:1;D:1",
                      "Resulting threshold after calibration"
                      "|threshold[dac]:Resulting threshold from calibration"
                      "|begin[mjd]:Start time of calibration"
                      "|end[mjd]:End time of calibration"),
        fPhysTriggerEnabled(false),
        fTriggerOn(false)
    {
        fDim.Subscribe(*this);
        fDimFTM.Subscribe(*this);
        fDimRS.Subscribe(*this);

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


