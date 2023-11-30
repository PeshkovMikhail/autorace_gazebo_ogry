#ifndef INTERSECTION_ACTION_CPP__VISIBILITY_CONTROL_H_
#define INTERSECTION_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTERSECTION_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define INTERSECTION_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define INTERSECTION_ACTION_CPP_EXPORT __declspec(dllexport)
    #define INTERSECTION_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTERSECTION_ACTION_CPP_BUILDING_DLL
    #define INTERSECTION_ACTION_CPP_PUBLIC INTERSECTION_ACTION_CPP_EXPORT
  #else
    #define INTERSECTION_ACTION_CPP_PUBLIC INTERSECTION_ACTION_CPP_IMPORT
  #endif
  #define INTERSECTION_ACTION_CPP_PUBLIC_TYPE INTERSECTION_ACTION_CPP_PUBLIC
  #define INTERSECTION_ACTION_CPP_LOCAL
#else
  #define INTERSECTION_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define INTERSECTION_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define INTERSECTION_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define INTERSECTION_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTERSECTION_ACTION_CPP_PUBLIC
    #define INTERSECTION_ACTION_CPP_LOCAL
  #endif
  #define INTERSECTION_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_