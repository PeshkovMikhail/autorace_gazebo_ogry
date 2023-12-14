#ifndef CROSSWALK_ACTION_CPP__VISIBILITY_CONTROL_H_
#define CROSSWALK_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CROSSWALK_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define CROSSWALK_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CROSSWALK_ACTION_CPP_EXPORT __declspec(dllexport)
    #define CROSSWALK_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CROSSWALK_ACTION_CPP_BUILDING_DLL
    #define CROSSWALK_ACTION_CPP_PUBLIC CROSSWALK_ACTION_CPP_EXPORT
  #else
    #define CROSSWALK_ACTION_CPP_PUBLIC CROSSWALK_ACTION_CPP_IMPORT
  #endif
  #define CROSSWALK_ACTION_CPP_PUBLIC_TYPE CROSSWALK_ACTION_CPP_PUBLIC
  #define CROSSWALK_ACTION_CPP_LOCAL
#else
  #define CROSSWALK_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CROSSWALK_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CROSSWALK_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CROSSWALK_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CROSSWALK_ACTION_CPP_PUBLIC
    #define CROSSWALK_ACTION_CPP_LOCAL
  #endif
  #define CROSSWALK_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ACTION_CPP__VISIBILITY_CONTROL_H_