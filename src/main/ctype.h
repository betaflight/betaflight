/*
 * Replacemnet for system header file <ctype.h> to avoid macro definitions
 * Functions are implemented in common/string_light.c
 */

#ifdef __cplusplus
// use original implementation for C++
# include_next <ctype.h>
#endif

#ifndef _CTYPE_H_
#define _CTYPE_H_

#ifndef _EXFUN
# define _EXFUN(name,proto) name proto
#endif

int _EXFUN(isalnum, (int __c));
int _EXFUN(isalpha, (int __c));
int _EXFUN(iscntrl, (int __c));
int _EXFUN(isdigit, (int __c));
int _EXFUN(isgraph, (int __c));
int _EXFUN(islower, (int __c));
int _EXFUN(isprint, (int __c));
int _EXFUN(ispunct, (int __c));
int _EXFUN(isspace, (int __c));
int _EXFUN(isupper, (int __c));
int _EXFUN(isxdigit,(int __c));
int _EXFUN(tolower, (int __c));
int _EXFUN(toupper, (int __c));
int _EXFUN(isblank, (int __c));

#endif /* _CTYPE_H_ */
