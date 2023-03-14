#include "Autons/autonSetup.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"
#include "display/lv_objx/lv_slider.h"
#include "main.h"
#include "pros/misc.hpp"

//global variables
bool devMode = false;
bool autonChecker = false;


bool colorSet = false;
bool autonSet = false;

lv_obj_t * autonTypeLabel;
lv_obj_t * checklistLabel;
lv_obj_t * icon;
lv_obj_t *tab1;
lv_obj_t * colorBtn;
lv_obj_t * turrSlider;
lv_obj_t * posBtn;
lv_obj_t * CalibrateBtn;
lv_obj_t * outLabels[20];


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
    colorSet = true;
    preMatchCheck();

    if(id == 0)
    {
        color +=1 ;
        if (color > 2){
            color = 0;
        }
        if (color == 1){
            setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(255, 100, 100), LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(255, 0, 0), LV_COLOR_MAKE(150, 150, 150), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
        }
        else if (color == 0){
            setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(100, 100, 255), LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(0, 0, 255), LV_COLOR_MAKE(150, 150, 150), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
        }
        else{
            setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(255, 0, 255), LV_COLOR_MAKE(255, 100, 255), LV_COLOR_MAKE(255, 0, 255), LV_COLOR_MAKE(255, 0, 255), LV_COLOR_MAKE(150, 150, 150), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
        }
    }
    updateScreen[0] = true;

    return LV_RES_OK;
}

lv_res_t autonSwitchClick(lv_obj_t * btn){
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons
    //std::cout << id;

    if(id == 1)
    {
        switch (autonType){
            case winPointFar:
                autonType = winPointClose;
                lv_label_set_text(autonTypeLabel, "winPoint near auton selected"); //sets label text
                break;
            case winPointClose:
                autonType = winPointBoth;
                lv_label_set_text(autonTypeLabel, "winPoint Both selected"); //sets label text
                break;
            case winPointBoth:
                autonType = noAuton;
                lv_label_set_text(autonTypeLabel, "no auton selected"); //sets label text
                break;
            case noAuton:
                autonType = skillsAuton;
                lv_label_set_text(autonTypeLabel, "skills auton selected"); //sets label text
                break;
            case skillsAuton:
                autonType = winPointFar;
                lv_label_set_text(autonTypeLabel, "winPoint far auton selected"); //sets label text
                break;
            default:
                autonType = winPointClose;
                lv_label_set_text(autonTypeLabel, "winPoint near auton selected"); //sets label text
                break;
        }
    }
    
    updateScreen[0] = true;
    autonSet = true;
    preMatchCheck();

    return LV_RES_OK;
}

void setupScreen(void){
    /*Create a Tab view object*/
    lv_obj_t *tabview;
    tabview = lv_tabview_create(lv_scr_act(), NULL);

    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    tab1 = lv_tabview_add_tab(tabview, "Auton Selector");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Outputted Values");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Dev");

    /*Auton selector screen*/
    colorBtn = createBtn(tab1, 10,10,120,30,0,"Color");
    setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(180, 180, 80), LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(180, 180, 0), LV_COLOR_MAKE(100, 100, 100), LV_COLOR_MAKE(0, 0, 0), colorBtn), colorBtn);
    lv_btn_set_action(colorBtn, LV_BTN_ACTION_CLICK, colorSwitchClick);
    
    posBtn = createBtn(tab1, 10,80,120,30,1,"Auton Type");
    setBtnStyle(createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(0, 200, 0), LV_COLOR_MAKE(0, 200, 80), LV_COLOR_MAKE(0, 80, 0), LV_COLOR_MAKE(0, 200, 0), LV_COLOR_MAKE(100, 100, 100), LV_COLOR_MAKE(0, 0, 0), posBtn), posBtn);
    lv_btn_set_action(posBtn, LV_BTN_ACTION_CLICK, autonSwitchClick);

    checklistLabel = lv_label_create(tab1, NULL); //create label and puts it on the screen
    lv_label_set_text(checklistLabel, "making"); //sets label text
    lv_obj_align(checklistLabel, NULL, LV_ALIGN_IN_TOP_MID, 30, 00); //set the position to center

    autonTypeLabel = lv_label_create(tab1, NULL); //create label and puts it on the screen
    lv_label_set_text(autonTypeLabel, "No choice yet"); //sets label text
    lv_obj_align(autonTypeLabel, NULL, LV_ALIGN_IN_BOTTOM_MID, 30, 0); //set the position to center

    preMatchCheck();


    /*Value Output Screen*/
    for (int i = 0; i < 20; i++){
        outLabels[i] = lv_label_create(tab2, NULL); //create label and puts it on the screen
        lv_label_set_text(outLabels[i], "No choice yet"); //sets label text
        if (i < 7){
            lv_obj_align(outLabels[i], NULL, LV_ALIGN_IN_TOP_LEFT, 5, 20*i+20); //set the position to left
        }
        else if (i < 14){
            lv_obj_align(outLabels[i], NULL, LV_ALIGN_IN_TOP_MID, 0, 20*(i-7)+20); //set the position to center
        }
        else{
            lv_obj_align(outLabels[i], NULL, LV_ALIGN_IN_TOP_RIGHT, -20, 20*(i-14)+20); //set the position to right
        }
    }
}

void preMatchCheck(void){
    char buffer[70];
    if(colorSet){
        sprintf(buffer, "Color Set: Done\n");
    }
    else{
        sprintf(buffer, "Color Set: X\n");
    }
    
    if(autonSet){
        sprintf(buffer + strlen(buffer), "Auton Set: Done\n");
    }
    else{
        sprintf(buffer + strlen(buffer), "Auton Set: X\n"); 
    }
    
    if(pros::battery::get_capacity() > 80){
        sprintf(buffer + strlen(buffer), "Battery: Good\n");
    }
    else{
        sprintf(buffer + strlen(buffer), "Battery: Bad\n"); 
    }
    sprintf(buffer + strlen(buffer), "Stupid: For Sure"); 
    lv_label_set_text(checklistLabel, buffer);
    delay(100);
} 