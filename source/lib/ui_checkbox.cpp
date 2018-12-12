struct ui_checkbox {
   element *e;
   bool clicked;
};

#define CheckBox(...) _CheckBox(GEN_UI_ID, __VA_ARGS__)
ui_checkbox _CheckBox(ui_id id, element *parent, bool checked,
                      v2 size, v2 p = V2(0, 0), v2 m = V2(0, 0))
{
   element *e = _Panel(id, parent, size, Padding(p).Margin(m).Captures(INTERACTION_CLICK));
   if(checked)
      Background(e, RED);
   Outline(e, BLACK);

   ui_checkbox result = {};
   result.e = e;
   result.clicked = WasClicked(e);
   return result;
}

ui_checkbox _CheckBox(ui_id id, element *parent, u32 *value, u32 flag,
                      v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   
   ui_checkbox box = _CheckBox(id, parent, *value & flag, size, padding, margin);
   if(box.clicked) {
      *value = *value ^ flag;
   }

   return box;
}