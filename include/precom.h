#define PC           0
#define MANIFOLD     1

#define VIDEO_FILE   0
#define VIDEO_CAMERA 1

#define NO_SHOW      0
#define SHOW_ALL     1

#define OPENMP_STOP  0
#define OPENMP_RUN   1

#define PI 3.14159265358979323

#if defined __arm__
#   define PLATFORM MANIFOLD
#else
#   define PLATFORM PC
#endif

#if PLATFORM == PC
#   define VIDEO         VIDEO_FILE
#   define DRAW          SHOW_ALL
#   define OPENMP_SWITCH OPENMP_STOP
#elif PLATFORM == MANIFOLD
#   define VIDEO         VIDEO_CAMERA
#   define DRAW          NO_SHOW
#   define OPENMP_SWITCH OPENMP_RUN
#endif
