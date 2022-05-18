#ifndef __BUTTONS_H__
#define __BUTTONS_H__


extern lv_obj_t * createBtn(lv_obj_t * , lv_coord_t , lv_coord_t , lv_coord_t , lv_coord_t ,
    int, const char *);

extern lv_style_t * createBtnStyle(lv_style_t * , lv_color_t , lv_color_t ,
        lv_color_t , lv_color_t , lv_color_t , lv_color_t , lv_obj_t * );


extern void setBtnStyle(lv_style_t * , lv_obj_t * );

extern lv_obj_t * drawRectangle( int , int , int , int , lv_color_t  );

extern void btnSetToggled(lv_obj_t * , bool );

#endif
