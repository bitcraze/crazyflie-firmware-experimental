#define BUILD_PILOT_APP
// #define BUILD_SNIFFER_APP

// check if both apps are defined
#if defined(BUILD_PILOT_APP) && defined(BUILD_SNIFFER_APP)
    #error "Only one app can be defined to be built!"
#endif