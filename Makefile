test_stl: test_numeric_stl.cpp numeric_stl.hpp
	g++ test_numeric_stl.cpp -o test_numeric_stl -std=c++11
	./test_numeric_stl

test_c2t:
	g++ test_current_to_threshold_prediction.cpp -o test_current_to_threshold_prediction -std=c++11
	./test_current_to_threshold_prediction

