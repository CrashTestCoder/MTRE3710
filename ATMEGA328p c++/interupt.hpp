#ifndef _INTERUPT_H_P_P_
#define _INTERUPT_H_P_P_

#define attatchInterrupt(vect, func, cond)\
inline ISR(vect) inline { func(); }

#endif /* _INTERUPT_H_P_P_ */