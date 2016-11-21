#ifndef FACT_HeadersRateControl
#define FACT_HeadersRateControl

namespace RateControl
{
    namespace State
    {
        enum states_t
        {
            kDimNetworkNA = 1,
            kDisconnected,
            kConnecting,              // obsolete, not used
            kConnected,

            kSettingGlobalThreshold,
            kGlobalThresholdSet,

            kInProgress,
        };
    };

    struct DimThreshold
    {
        uint16_t threshold;
        double   begin;
        double   end;
    }  __attribute__((__packed__));


    template<typename T>
    double vector_mean(T v){
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        return mean;
    }

    template<typename T>
    double vector_std(T v){
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        std::vector<double> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size());
        return stdev;
    }

    template<typename T>
    double vector_median(T v){
        T copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());
        const int N = copy_v.size()/2;
        if (copy_v.size() % 2 == 0) {

            return (copy_v[N-1] + copy_v[N]) / 2;
        } else {
            return copy_v[N];
        }
    }

    // estimate the stddev from the cdf
    template<typename T>
    double vector_std_from_cdf(T v){
        T copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());


        // This magic number is calculated like this:
        //      0.84134474606854293 = 2 * (1 + erf(1 / sqrt(2) )
        // Is the point on the y-axis of the cdf of a random variable one
        // has too look for, when one wants to find the point on the x-axis,
        // where (assuming a normal distribution) one is one sigma
        // away from the median.
        double frac = copy_v.size() * 0.84134474606854293;
        const int left = int(floor(frac));
        const int right = int(ceil(frac));

        double upper = copy_v[right];
        double lower = copy_v[left];

        double median_plus_sigma = lower + (upper - lower) * (frac - left);

        return median_plus_sigma - vector_median(v);
    }

}

#endif
