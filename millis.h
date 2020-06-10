//
//  File: millis.h
//  Project: yafocuser
//
//  Created by Sven Kreiensen on 06.11.19.
//  Copyright © 2019 Sven Kreiensen. All rights reserved.
//
#ifndef _millis_h_
#define _millis_h_

void init_millis(unsigned long f_cpu);
uint32_t millis(void);
uint32_t secs(void);

#endif
