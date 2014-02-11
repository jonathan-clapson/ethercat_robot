#ifndef __SUPPORT_H__
#define __SUPPORT_H__

#include "TcInterfaces.h"

#define printf(a, ...) printf_m_Trace(m_Trace, a, __VA_ARGS__)

void printf_m_Trace(CTcTrace &m_Trace, const char *fmt, ...);

#endif /* __SUPPORT_H__ */