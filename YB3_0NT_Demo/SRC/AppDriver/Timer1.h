#ifndef	__TIMER1_H
#define	__TIMER1_H

#include "config.h"

#ifdef	__TIMER1_C
#define	T1_EXT
#include "WDT.h"
#else
#define	T1_EXT	extern
#endif

//��ʼ��
T1_EXT	void Time1Init(void);


#endif

