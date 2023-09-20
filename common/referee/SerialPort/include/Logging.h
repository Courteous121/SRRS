#include <iostream>

#ifdef NDEBUG
#define DEBUG_WARNING_(msg) ((void)0)
#define DEBUG_ERROR_(msg) ((void)0)
#define DEBUG_HIGHLIGHT_(msg) ((void)0)
#define DEBUG_INFO_(msg) ((void)0)
#else
#define DEBUG_WARNING_(msg) WARNING_(msg)
#define DEBUG_ERROR_(msg) ERROR_(msg)
#define DEBUG_HIGHLIGHT_(msg) HIGHLIGHT_(msg)
#define DEBUG_INFO_(msg) INFO_(msg)
#endif

#define ERROR_(msg)                                                 \
    do                                                              \
    {                                                               \
        std::cout << "\033[1;31m" << msg << "\033[0m" << std::endl; \
    } while (0)
#define WARNING_(msg)                                             \
    do                                                            \
    {                                                             \
        std::cout << "\033[33m" << msg << "\033[0m" << std::endl; \
    } while (0)
#define HIGHLIGHT_(msg)                                             \
    do                                                              \
    {                                                               \
        std::cout << "\033[1;34m" << msg << "\033[0m" << std::endl; \
    } while (0)
#define INFO_(msg)                     \
    do                                 \
    {                                  \
        std::cout << msg << std::endl; \
    } while (0)
    