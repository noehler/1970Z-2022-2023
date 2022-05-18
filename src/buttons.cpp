#include "main.h"

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

lv_style_t * createBtnStyle(lv_style_t * copy, lv_color_t rel, lv_color_t pr,
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

void setBtnStyle(lv_style_t * btnStyle, lv_obj_t * btn)
{
    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
    lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);
}

void btnSetToggled(lv_obj_t * btn, bool toggled)
{
    if(toggled != (lv_btn_get_state(btn) >= 2)) lv_btn_toggle(btn);
}

lv_obj_t * drawRectangle( int x, int y, int width, int height, lv_color_t color ) {
  lv_obj_t * obj1 = lv_obj_create(lv_scr_act(), NULL);

  lv_style_t *style1 = (lv_style_t *)malloc( sizeof( lv_style_t ));
  lv_style_copy(style1, &lv_style_plain_color);    /*Copy a built-in style to initialize the new style*/
  style1->body.empty = 1;
  style1->body.border.color = color;
  style1->body.border.width = 1;
  style1->body.border.part = LV_BORDER_FULL;

  lv_obj_set_style(obj1, style1);
  lv_obj_set_pos(obj1, x, y);
  lv_obj_set_size(obj1, width, height);

  return obj1;
}
