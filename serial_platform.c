#if defined(__WIN32__) || defined(__CYGWIN__)
#   include "serial_w32.c"
#else
#if defined(USE_FTDI)
#	include "serial_ftdi.c"
#else
#   include "serial_posix.c"
#endif
#endif
