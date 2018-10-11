struct ui_checkbox {
   element *e;
   ui_click inter;
   bool clicked;
};

#define CheckBox(...) _CheckBox(GEN_UI_ID, __VA_ARGS__)
ui_checkbox _CheckBox(ui_id id, element *parent, bool checked,
                      v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   element *e = _Panel(id, parent, NULL, size, padding, margin);
   ui_click click = DefaultClickInteraction(e);
   if(checked)
   Background(e, RED);
   Outline(e, BLACK);

   ui_checkbox result = {};
   result.e = e;
   result.inter = click;
   result.clicked = click.clicked;
   return result;
}

//TODO: one frame delay here, fix
ui_checkbox _CheckBox(ui_id id, element *parent, u32 *value, u32 flag,
                      v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   
   ui_checkbox box = _CheckBox(id, parent, *value & flag, size, padding, margin);
   if(box.clicked) {
      *value = *value ^ flag;
   }

   return box;
}