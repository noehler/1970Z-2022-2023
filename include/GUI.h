#ifndef __GUI_H__
#define __GUI_H__

#include "api.h"

using namespace pros;

extern lv_obj_t * posBtn;
extern void setupScreen(void);
extern void preMatchCheck(void);
extern lv_obj_t * turrSlider;
extern lv_obj_t * outLabels[20];
extern lv_obj_t * icon;
extern int current_field;

#endif 