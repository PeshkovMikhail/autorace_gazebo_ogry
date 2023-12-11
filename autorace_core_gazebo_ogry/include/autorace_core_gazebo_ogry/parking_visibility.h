#ifndef PARKING_ACTION_CPP__VISIBILITY_CONTROL_H_
#define PARKING_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PARKING_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define PARKING_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define PARKING_ACTION_CPP_EXPORT __declspec(dllexport)
    #define PARKING_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef PARKING_ACTION_CPP_BUILDING_DLL
    #define PARKING_ACTION_CPP_PUBLIC PARKING_ACTION_CPP_EXPORT
  #else
    #define PARKING_ACTION_CPP_PUBLIC PARKING_ACTION_CPP_IMPORT
  #endif
  #define PARKING_ACTION_CPP_PUBLIC_TYPE PARKING_ACTION_CPP_PUBLIC
  #define PARKING_ACTION_CPP_LOCAL
#else
  #define PARKING_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define PARKING_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define PARKING_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define PARKING_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PARKING_ACTION_CPP_PUBLIC
    #define PARKING_ACTION_CPP_LOCAL
  #endif
  #define PARKING_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_