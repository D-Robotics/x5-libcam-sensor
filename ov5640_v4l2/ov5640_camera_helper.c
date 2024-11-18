#include <stdio.h>
#include <stdint.h>

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

	uint32_t index = 0;

	index = (input - 1.0) / 0.0625;

	return index;
}

//index to again
double indextoagain_function(uint32_t input) {

	double gain = 1.0;

	gain = gain + input * 0.0625;

	return gain;
}

//dgain to index
uint32_t dgaintoindex_function(double input) {
	return (uint32_t)(input);
}

//index to dgain
double indextodgain_function(uint32_t input) {
	return (double)(input);
}


Callbacks cb = {againtoindex_function, indextoagain_function,
		dgaintoindex_function, indextodgain_function,};

//get_gain_index_callbacks
Callbacks* get_gain_index_callbacks() {
	return &cb;
}

