#ifdef VERSION1
#include "../versions/version1_accel.cpp"
#elif defined(VERSION2)
#include "../versions/version2_manual.cpp"
#elif defined(VERSION3)
#include "../versions/version3_constantSpeed.cpp"
#elif defined(VERSION4)
#include "../versions/version4_angleControl.cpp"
#else
#error "No version selected. Use -DVERSION1 or -DVERSION2 etc."
#endif
