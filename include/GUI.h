#ifndef __GUI_H__
#define __GUI_H__

#ifndef _PROS_MAIN_H_
#include "api.h"
#endif

using namespace pros;

extern lv_obj_t * posBtn;
void setupScreen(void);
extern lv_obj_t * turrSlider;
extern lv_obj_t * outLabels[20];

#endif