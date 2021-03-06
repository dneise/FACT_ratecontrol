#ifndef FACT_HeadersRateControl
#define FACT_HeadersRateControl

namespace RateControl
{
    namespace State{
        enum state_t
        {
            kDimNetworkNA = 1,
            kDisconnected,
            kConnecting,              // obsolete, not used
            kConnected,

            kSettingGlobalThreshold,
            kGlobalThresholdSet,

            kInProgress,
        };
    }


    struct DimThreshold
    {
        uint16_t threshold;
        double   begin;
        double   end;
    }  __attribute__((__packed__));

    enum
    {
        kNumBiasChannels = 320
    };
}


typedef std::array<uint32_t, 160> thresholds_t;
typedef std::array<uint32_t, RateControl::kNumBiasChannels> thresholds_for_bias_patches_t;
typedef std::array<double, RateControl::kNumBiasChannels> currents_t;

#endif
