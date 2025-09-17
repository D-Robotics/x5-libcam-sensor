#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>

typedef uint32_t (*AGainToIndex_t)(double);
typedef double (*IndexToAGain_t)(uint32_t);
typedef uint32_t (*DGainToIndex_t)(double);
typedef double (*IndexToDGain_t)(uint32_t);

typedef struct {
	AGainToIndex_t againtoindex_callback;
	IndexToAGain_t indextoagain_callback;
	DGainToIndex_t dgaintoindex_callback;
	IndexToDGain_t indextodgain_callback;
} Callbacks;


//again to index
uint32_t againtoindex_function(double input) {

    double x = 32 * log2(input);
    int x_rounded = (int)round(x);

	return x_rounded;
}

//index to again
double indextoagain_function(uint32_t input) {

    double result = pow(2, (double)input / 32); 

	return result;
}

//dgain to index
uint32_t dgaintoindex_function(double input) {
	double x = 32 * log2(input);
    int x_rounded = (int)round(x);

	return x_rounded;
}

//index to dgain
double indextodgain_function(uint32_t input) {
	double result = pow(2, (double)input / 32); 

	return result;
}

Callbacks cb = {againtoindex_function, indextoagain_function,
		dgaintoindex_function, indextodgain_function,};

//get_gain_index_callbacks
Callbacks* get_gain_index_callbacks() {
	return &cb;
}

