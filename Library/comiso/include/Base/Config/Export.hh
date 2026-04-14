
#ifndef BASE_EXPORT_H
#define BASE_EXPORT_H

#ifdef BASE_STATIC_DEFINE
#  define BASE_EXPORT
#  define BASE_NO_EXPORT
#else
#  ifndef BASE_EXPORT
#    ifdef CoMISo_EXPORTS
        /* We are building this library */
#      define BASE_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define BASE_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef BASE_NO_EXPORT
#    define BASE_NO_EXPORT 
#  endif
#endif

#ifndef BASE_DEPRECATED
#  define BASE_DEPRECATED __declspec(deprecated)
#endif

#ifndef BASE_DEPRECATED_EXPORT
#  define BASE_DEPRECATED_EXPORT BASE_EXPORT BASE_DEPRECATED
#endif

#ifndef BASE_DEPRECATED_NO_EXPORT
#  define BASE_DEPRECATED_NO_EXPORT BASE_NO_EXPORT BASE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef BASE_NO_DEPRECATED
#    define BASE_NO_DEPRECATED
#  endif
#endif

#endif /* BASE_EXPORT_H */
