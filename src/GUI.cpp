#include "main.h"
#include "robotConfig.h"
#include <cmath>

lv_obj_t * createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height,
    int id, const char * title)
{
    lv_obj_t * btn = lv_btn_create(parent, NULL);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, width, height);
    lv_obj_set_free_num(btn, id);

    lv_obj_t * label = lv_label_create(btn, NULL);
    lv_label_set_text(label, title);
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

    return btn;
}

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t myButtonStyleREL; //relesed style
lv_style_t myButtonStylePR; //pressed style

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
        sprintf(buffer, "button was clicked %i milliseconds from start", pros::millis());
		lv_label_set_text(myLabel, buffer);
    }

    return LV_RES_OK;
}

double targetAngleOffest = 0;

void controller2(void){
    static double offsetxbot = 0;
    static double offsetybot = 0;
    static double offsetxgoal = 0;
    static double offsetygoal = 0;

    double addxbot = 0;
    double addybot = 0;

    if (sidecar.get_digital(DIGITAL_UP)){
        addybot+=1;
    }
    if (sidecar.get_digital(DIGITAL_DOWN)){
        addybot-=1;
    }
    if (sidecar.get_digital(DIGITAL_RIGHT)){
        addxbot+=1;
    }
    if (sidecar.get_digital(DIGITAL_LEFT)){
        addxbot-=1;
    }

    if (sidecar.get_digital(DIGITAL_A) && sidecar.get_digital(DIGITAL_B)
    && sidecar.get_digital(DIGITAL_X) && sidecar.get_digital(DIGITAL_Y)){
        boomShackalacka.set_value(true);
        std::cout << "\n false";
    }

    targetAngleOffest += float(sidecar.get_analog(ANALOG_LEFT_X))/64;
    robot.xpos += float(sidecar.get_analog(ANALOG_RIGHT_X))/64;
    robot.ypos += float(sidecar.get_analog(ANALOG_RIGHT_Y))/64;
    
    if (sidecar.get_digital(DIGITAL_A)){
        targetAngleOffest = 0;
    }

    offsetxbot+=addxbot;
    offsetybot+=addybot;

    robot.xpos +=addxbot;
    robot.ypos +=addybot;

    bool multchanged = false;
    if (sidecar.get_digital(DIGITAL_R1)){
        flySpdMult +=.01;
        multchanged = true;
    }
    else if (sidecar.get_digital(DIGITAL_R2)){
        flySpdMult -=.01;
        multchanged = true;
    }

    delay(50);
    sidecar.clear();
    delay(50);
    sidecar.print(0,1,"(%.0f|%.0f), FM: %.2f, Toff: %.2f", robot.xpos, robot.ypos, flySpdMult, targetAngleOffest);
    delay(50);
    sidecar.print(1,1,"Toff: %.2f", targetAngleOffest);
    
}
/*
static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        printf("Value: %d\n", lv_slider_get_value(obj));
      }
}*/

void guiInit(void){
    /*lv_obj_t * slider = lv_slider_create(lv_scr_act(), NULL);
    lv_obj_align(slider, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_action(slider, event_handler);*/
}
