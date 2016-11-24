#ifndef NUMERIC_STL
#define NUMERIC_STL

#include <numeric>
#include <functional>
#include <cmath>

namespace NumericStl
{
    template<typename Container>
    double mean(const Container& v){
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        return sum / v.size();
    }

    template<typename Container>
    double std(const Container& v){
        double m = mean(v);


        std::vector<double> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), m));

        double sq_sum = std::inner_product(
            diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size());
        return stdev;
    }

    template<typename Container>
    double median(const Container& v){
        Container copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());
        const int N = copy_v.size() / 2;
        if (N==0){
            return std::nan("");
        }
        if (copy_v.size() % 2 == 0) {
            return (copy_v[N-1] + copy_v[N]) / 2.;
        } else {
            return copy_v[N];
        }
    }

    template<typename Container>
    double min(const Container& v){
        Container copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());
        return copy_v[0];
    }

    template<typename Container>
    double max(const Container& v){
        Container copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());
        return copy_v[copy_v.size()-1];
    }

    // estimate the stddev from the cdf
    template<typename Container>
    double std_from_cdf(const Container& v){
        Container copy_v(v);
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

        return median_plus_sigma - median<double>(v);
    }

    template <typename Wheretype, typename Xtype, typename Ytype>
    double interpolate(Wheretype where, Xtype x1, Xtype x2, Ytype y1, Ytype y2){
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;
        return slope * where + intercept;
    }

    template<typename Container>
    double percentile(double q, const Container& v){
        Container copy_v(v);
        std::sort(copy_v.begin(), copy_v.end());

        double where = (copy_v.size()-1) * q;
        const int left = int(floor(where));
        const int right = left + 1;
        return interpolate(where, left, right, copy_v[left], copy_v[right]);
    }

}
#endif // NUMERIC_STL