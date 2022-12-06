#include "main.h"

//global variables
bool isRed;
bool devMode = false;
bool autonChecker = false;


lv_obj_t * autonTypeLabel;
extern lv_obj_t * autonTypeLabel;
lv_obj_t * colorBtn;
lv_obj_t * posBtn;


/*This section has functions to make buttons*/

//used to make a button quickly
static lv_obj_t * createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height,
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

//used to create a button style
static lv_style_t * createBtnStyle(lv_style_t * copy, lv_color_t rel, lv_color_t pr,
                            lv_color_t tglRel, lv_color_t tglPr, lv_color_t tglBorder, lv_color_t textColor, lv_obj_t * btn)
{
    lv_style_t * btnStyle = (lv_style_t *)malloc(sizeof(lv_style_t) * 4);

    for(int i = 0; i < 4; i++) lv_style_copy(&btnStyle[i], copy);

    btnStyle[0].body.main_color = rel;
    btnStyle[0].body.grad_color = rel;
    btnStyle[0].text.color = textColor;

    btnStyle[1].body.main_color = pr;
    btnStyle[1].body.grad_color = pr;
    btnStyle[1].text.color = textColor;

    btnStyle[2].body.main_color = tglRel;
    btnStyle[2].body.grad_color = tglRel;
    btnStyle[2].body.border.width = 2;
    btnStyle[2].body.border.color = tglBorder;
    btnStyle[2].text.color = textColor;

    btnStyle[3].body.main_color = tglPr;
    btnStyle[3].body.grad_color = tglPr;
    btnStyle[3].body.border.width = 2;
    btnStyle[3].body.border.color = tglBorder;
    btnStyle[3].text.color = textColor;

    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);

    return btnStyle;
}

//used to set a button style
static void setBtnStyle(lv_style_t * btnStyle, lv_obj_t * btn)
{
    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);
}

//"This function is nice to have to toggle a button because the defualt function is not very good."
//                                  -- https://team81k.github.io/ProsLVGLTutorial/
void btnSetToggled(lv_obj_t * btn, bool toggled)
{
    if(toggled != (lv_btn_get_state(btn) >= 2)) lv_btn_toggle(btn);
}


/*this section has the events called when buttons are clicked*/
lv_res_t colorSwitchClick(lv_obj_t * btn){
    uint8_t id = lv_obj_get_free_num(btn); //id useful when there are multiple buttons

    if(id == 0)
    {
        isRed = !isRed;
        if (isRed){
            setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(255, 100, 100), LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(150, 150, 150), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
        }
        else{
            setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(100, 100, 255), LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(150, 150, 150), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
        }
    }

    return LV_RES_OK;
}

lv_res_t autonSwitchClick(lv_obj_t * btn){
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons
    std::cout << id;
    if(id == 1)
    {
        switch (autonType){
            case basicAuton:
                autonType = winPoint;
                lv_label_set_text(autonTypeLabel, "winPoint auton selected"); //sets label text
                break;
            case winPoint:
                autonType = basicAuton;
                lv_label_set_text(autonTypeLabel, "Basic auton selected"); //sets label text
                break;
        }
    }

    return LV_RES_OK;
}

void setupScreen(void){
    colorBtn = createBtn(lv_scr_act(), 10,10,180,60,0,"Color");
    setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(180, 180, 80), LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(100, 100, 100), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
    lv_btn_set_action(colorBtn, LV_BTN_ACTION_CLICK, colorSwitchClick);
    
    posBtn = createBtn(lv_scr_act(), 10,80,180,60,1,"Auton Type");
    setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 200, 0), LV_COLOR_MAKE(0, 200, 80), LV_COLOR_MAKE(0, 80, 0), LV_COLOR_MAKE(0, 200, 0), LV_COLOR_MAKE(100, 100, 100), LV_COLOR_MAKE(0, 0, 0), posBtn), posBtn);
    lv_btn_set_action(posBtn, LV_BTN_ACTION_CLICK, autonSwitchClick);

    autonTypeLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(autonTypeLabel, "No choice yet"); //sets label text
    lv_obj_align(autonTypeLabel, NULL, LV_ALIGN_CENTER, 30, -10); //set the position to center
}
