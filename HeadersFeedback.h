#ifndef FACT_HeadersFeedback
#define FACT_HeadersFeedback

#include "HeadersBIAS.h"

namespace Feedback
{
    namespace State
    {
        enum states_t
        {
            kDimNetworkNA = 1,
            kDisconnected,
            kConnecting,
            kConnected,

            kCalibrating,
            kCalibrated,

            kWaitingForData,
            kInProgress,

            kWarning,
            kCritical,
            kOnStandby,


        };
    }

    struct CalibratedCurrentsData
    {
        float I[BIAS::kNumChannels];
        float Iavg;
        float Irms;
        float Imed;
        float Idev;
        uint32_t N;
        float Tdiff;
        float Uov[BIAS::kNumChannels];
        float Unom;
        float dUtemp;

        CalibratedCurrentsData() { memset(this, 0, sizeof(CalibratedCurrentsData)); }
    } __attribute__((__packed__));
}

#endif
