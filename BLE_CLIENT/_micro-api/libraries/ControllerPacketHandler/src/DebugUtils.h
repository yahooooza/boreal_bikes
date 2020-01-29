#ifndef __DEBUG_UTILS_H
#define __DEBUG_UTILS_H

#define DEBUGLEVEL 1

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#if DEBUGLEVEL > 0
#define debug(fmt, ...)		(fprintf(stderr, fmt, ##__VA_ARGS__))
#define psize(type)			(error_at_line(0, 0, __FILE__, __LINE__, "sizeof(" #type ") : %lu", sizeof(type)))
#define trace(fmt, var)		(error_at_line(0, 0, __FILE__, __LINE__, "%s : " fmt, #var, var))
#else
#define debug(fmt, ...)
#define psize(type)
#define trace(fmt, var)
#endif

#endif	/* #ifndef __DEBUG_UTILS_H */