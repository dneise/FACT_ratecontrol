#include <valarray>
#include <array>

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
#include "numeric_stl.hpp"

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

    deque<currents_t> fHistoricCurrents;
    thresholds_t fLastThresholds;

    Time fTimeOfLastCalibratedCurrents;

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
        long ms_since_last = (evt.GetTime() - fTimeOfLastCalibratedCurrents).total_milliseconds();
        if (ms_since_last < 1000){
            return GetCurrentState();
        }
        fTimeOfLastCalibratedCurrents = evt.GetTime();

        if (!CheckEventSize(evt, sizeof(Feedback::CalibratedCurrentsData))) return GetCurrentState();

        const Feedback::CalibratedCurrentsData &calibrated_currents = (
            *static_cast<const Feedback::CalibratedCurrentsData*>(evt.GetData()) );

        currents_t bias_currents;
        std::copy_n(calibrated_currents.I,
                  RateControl::kNumBiasChannels,
                  bias_currents.begin());


        AppendToHistoricCurrents(bias_currents);
        bias_currents = GetMedianOfHistoricCurrents();

        auto thresholds = CalcThresholdsFromCurrents(bias_currents);
        auto replaced = ReplaceBrokenBiasPatches(thresholds);
        auto sorted_v = SortThresholdsIntoDualTriggerPatchOrder(replaced, fMap);
        auto proposed_thresholds = CombineThresholds(sorted_v);

        auto new_thresholds = SelectSignificantChanges(proposed_thresholds);

        PrintThresholdsOutOfRange(proposed_thresholds);
        if (GetCurrentState() == RateControl::State::kInProgress){
            SetThresholds(new_thresholds);
            fLastThresholds = new_thresholds;
        }
        return GetCurrentState();
    }

    thresholds_t SelectSignificantChanges(const thresholds_t& proposed_thresholds)
    {
        const uint32_t significance_limit = 5;
        thresholds_t new_thresholds{};

        for(int i=0; i < fLastThresholds.size(); i++){
            int32_t diff = int32_t(proposed_thresholds[i]) - fLastThresholds[i];

            if (abs(diff) >= significance_limit){
                new_thresholds[i] = proposed_thresholds[i];
            }
            else{
                new_thresholds[i] = fLastThresholds[i];
            }
        }

        return move(new_thresholds);
    }

    void PrintThresholdsOutOfRange(const thresholds_t& proposed_thresholds)
    {
        for(int i=0; i < proposed_thresholds.size(); i++){
            if (
                (proposed_thresholds[i] < 0)
                || (proposed_thresholds[i] > 0xffff)
            )
            {
                Out() << "proposed_thresholds["<<i<<"]=" << proposed_thresholds[i] << endl;
            }
        }
    }

    void AppendToHistoricCurrents(const currents_t& bias_currents){
        const unsigned int history_length = 3;
        fHistoricCurrents.push_back(bias_currents);
        while (fHistoricCurrents.size() > history_length){
            fHistoricCurrents.pop_front();
        }
    }

    currents_t GetMedianOfHistoricCurrents(void){
        currents_t medians{};
        for (unsigned int b_id=0; b_id < medians.size(); b_id++){
            vector<double> buffer;
            for(auto it=fHistoricCurrents.begin(); it<fHistoricCurrents.end(); it++){
                buffer.push_back(it->at(b_id));
            }
            medians[b_id] = NumericStl::median(buffer);
        }
        return move(medians);
    }

    void SetThresholds(thresholds_t& thresholds){
        Out() << "SetThresholds: current FTM state" << fDimFTM.state();
        Out() << "==? FTM::State::kTriggerOn:" << bool(fDimFTM.state() == FTM::State::kTriggerOn) << endl;
        if (fDimFTM.state() == FTM::State::kTriggerOn){

            Dim::SendCommandNB("FTM_CONTROL/SET_SELECTED_THRESHOLDS", thresholds);
        }
        else{
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

    void PrintThresholds() const
    {
        for (int j=0; j<10; j++) {
            for (int k=0; k<4; k++) {
                for (int i=0; i<4; i++) {
                    const int p = i + k*4 + j*16;
                    Out() << setw(3) << fLastThresholds[p] << " ";
                }
                Out() << "   ";
            }
            Out() << endl;
        }
        Out() << endl;
    }


    int Print() const
    {
        Out() << fDim << endl;
        Out() << fDimFTM << endl;
        Out() << fDimRS << endl;
        PrintThresholds();
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
        fTriggerOn(false),
        fLastThresholds(),
        fTimeOfLastCalibratedCurrents()
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


