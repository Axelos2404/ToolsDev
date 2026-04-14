
#ifndef COMISO_EXPORT_H
#define COMISO_EXPORT_H

#ifdef COMISO_STATIC_DEFINE
#  define COMISO_EXPORT
#  define COMISO_NO_EXPORT
#else
#  ifndef COMISO_EXPORT
#    ifdef CoMISo_EXPORTS
        /* We are building this library */
#      define COMISO_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define COMISO_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef COMISO_NO_EXPORT
#    define COMISO_NO_EXPORT 
#  endif
#endif

#ifndef COMISO_DEPRECATED
#  define COMISO_DEPRECATED __declspec(deprecated)
#endif

#ifndef COMISO_DEPRECATED_EXPORT
#  define COMISO_DEPRECATED_EXPORT COMISO_EXPORT COMISO_DEPRECATED
#endif

#ifndef COMISO_DEPRECATED_NO_EXPORT
#  define COMISO_DEPRECATED_NO_EXPORT COMISO_NO_EXPORT COMISO_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef COMISO_NO_DEPRECATED
#    define COMISO_NO_DEPRECATED
#  endif
#endif

#endif /* COMISO_EXPORT_H */
