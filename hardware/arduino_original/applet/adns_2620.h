//ADNS register names

#define CONFIGURATION 0x40
#define STATUS        0x41

// how much the mouse sensor has moved in the y-direction since last poll, in 2's complement
#define DELTA_Y       0x42

// how much the mouse sensor has moved in the y-direction since last poll, in 2's complement
#define DELTA_X       0x43

// intensity median
#define SQUAL         0x44

// max brightness in a pixel
#define MAX_PIXEL     0x45

// minimum brightness in a pixel
#define MIN_PIXEL     0x46

// sum of intensities mod 255
#define PIXEL_SUM     0x47

// array of pixel intensities (18^2 values)
#define PIXEL_DATA    0x48

// light calibration stuff (see datasheet)
#define SHUTTER_UPPER 0x49
#define SHUTTER_LOWER 0x4A

#define PRODUCT_ID    0x48


//config register
#define RESET 7
#define POWER_DOWN 6
#define FORCE_AWAKE 0

//status register
#define ID2 7
#define ID1 6
#define ID0 5

// is awake?
#define AWAKE 0
