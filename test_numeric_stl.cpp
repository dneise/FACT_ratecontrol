#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include <vector>
#include <cmath>
#include "catch.hpp"
#include "numeric_stl.hpp"

TEST_CASE( "mean", "[mean]" ) {

    std::vector<double> v = {1, 2, 3, 4};

    REQUIRE( NumericStl::mean(v) == Approx(2.5) );
    REQUIRE( NumericStl::std(v) == Approx(1.11803) );
    REQUIRE( NumericStl::median(v) == Approx(2.5) );
}

TEST_CASE( "min_max_perc", "[min_max_perc]" ) {
    std::vector<double> v = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    double min = NumericStl::min(v);
    double max = NumericStl::max(v);
    double median = NumericStl::median(v);

    REQUIRE( min == Approx(0) );
    REQUIRE( max == Approx(9) );
    REQUIRE( median == Approx(4.5) );

    REQUIRE( NumericStl::percentile(0., v) == min );
    //CHECK( NumericStl::percentile(1., v) == max );
    REQUIRE( NumericStl::percentile(0.5, v) == median );

    REQUIRE( NumericStl::percentile(0.1, v) == Approx(0.9) );
    REQUIRE( NumericStl::percentile(0.2, v) == Approx(1.8) );
    REQUIRE( NumericStl::percentile(0.3, v) == Approx(2.7) );
    REQUIRE( NumericStl::percentile(0.4, v) == Approx(3.6) );
    REQUIRE( NumericStl::percentile(0.6, v) == Approx(5.4) );
    REQUIRE( NumericStl::percentile(0.7, v) == Approx(6.3) );
    REQUIRE( NumericStl::percentile(0.8, v) == Approx(7.2) );
    REQUIRE( NumericStl::percentile(0.9, v) == Approx(8.1) );
}


TEST_CASE( "interpolate", "[interpolate]" ) {
    double x1 = 0;
    double x2 = 1;
    double y1 = 0;
    double y2 = 1;

    REQUIRE( NumericStl::interpolate(0., x1, x2, y1, y2) == Approx(0.) );
    REQUIRE( NumericStl::interpolate(1., x1, x2, y1, y2) == Approx(1.) );
    REQUIRE( NumericStl::interpolate(0.5, x1, x2, y1, y2) == Approx(0.5) );

    x1 = 0;
    x2 = 1;
    y1 = 1;
    y2 = 0;

    REQUIRE( NumericStl::interpolate(0., x1, x2, y1, y2) == Approx(1.) );
    REQUIRE( NumericStl::interpolate(1., x1, x2, y1, y2) == Approx(0.) );
    REQUIRE( NumericStl::interpolate(0.5, x1, x2, y1, y2) == Approx(0.5) );
}


TEST_CASE( "mean_in_case_of_empty", "[mean]" ) {
    std::vector<double> v;
    REQUIRE( std::isnan(NumericStl::mean(v)) );
    REQUIRE( std::isnan(NumericStl::std(v)) );
    REQUIRE( std::isnan(NumericStl::median(v)) );
}
