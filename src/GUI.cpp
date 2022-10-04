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

void controller2(void){
    double offsetxbot = 0;
    double offsetybot = 0;
    double offsetxgoal = 0;
    double offsetgoal = 0;

    while(sidecar.is_connected()){
        double addxbot = float(sidecar.get_analog(ANALOG_LEFT_X))/64;
        double addybot = float(sidecar.get_analog(ANALOG_LEFT_Y))/64;

        static double offsetxbot = 0;
        static double offsetybot = 0;

        offsetxbot+=addxbot;
        offsetybot+=addybot;

        robot.xpos +=addxbot;
        robot.ypos +=addybot;

        double addxgoal = float(sidecar.get_analog(ANALOG_RIGHT_X))/64;
        double addygoal = float(sidecar.get_analog(ANALOG_RIGHT_Y))/64;
        
        static double offsetxgoal = 0;
        static double offsetygoal = 0;
        offsetxgoal+=addxgoal;
        offsetygoal+=addygoal;

        homeGoal.xpos +=addxgoal;
        homeGoal.ypos +=addygoal;

        bool multchanged = false;
        if (sidecar.get_digital(DIGITAL_R1)){
            flySpdMult +=.01;
            multchanged = true;
        }
        else if (sidecar.get_digital(DIGITAL_R2)){
            flySpdMult -=.01;
            multchanged = true;
        }

        if (addxbot != 0 || addybot != 0 || addxgoal != 0 || addygoal != 0 || multchanged){
            delay(50);
            sidecar.clear();
            delay(50);
            sidecar.print(0,1,"Goal: X: %.2f, Y: %.2f, oX: %.2f, oY: %.2f", robot.xpos, robot.ypos, addxbot, addybot);
            delay(50);
            sidecar.print(1,1,"Bot: X: %.2f, Y: %.2f, oX: %.2f, oY: %.2f", homeGoal.xpos, homeGoal.ypos, addxgoal, addygoal);
            delay(50);
            sidecar.print(2,1,"FlyMult: %.2f", flySpdMult);
        }
        else{
            delay(400);
        }
    }
}

void guiInit(void){
    /*lv_style_copy(&myButtonStyleREL, &lv_style_plain);
    myButtonStyleREL.body.main_color = LV_COLOR_MAKE(150, 0, 0);
    myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(0, 0, 150);
    myButtonStyleREL.body.radius = 0;
    myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR, &lv_style_plain);
    myButtonStylePR.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    myButtonStylePR.body.grad_color = LV_COLOR_MAKE(0, 0, 255);
    myButtonStylePR.body.radius = 0;
    myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    myButton = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(myButton, 0); //set button id to 0
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); //set the relesed style
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); //set the pressed style
    lv_obj_set_size(myButton, 200, 50); //set the button size
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); //set the position to top mid

    //myButton = createBtn(lv_scr_act(), 10, 10, 200, 50, 0, myButton);

    myButtonLabel = lv_label_create(myButton, NULL); //create label and puts it inside of the button
    lv_label_set_text(myButtonLabel, "Click the Button"); //sets label text

    myLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(myLabel, "Button has not been clicked yet"); //sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_LEFT_MID, 10, 30); //set the position to center*/
}