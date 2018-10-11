#define SettingsRow(...) _SettingsRow(GEN_UI_ID, __VA_ARGS__)
ui_numberbox _SettingsRow(ui_id id, element *page, char *label, u32 *num, char *suffix = "") {
   element *row = Panel(page, RowLayout, V2(Size(page->bounds).x, 20), V2(0, 0), V2(0, 5));
   Label(row, label, 20);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20);
   return result;
}

ui_numberbox _SettingsRow(ui_id id, element *page, char *label, f32 *num, char *suffix = "") {
   element *row = Panel(page, RowLayout, V2(Size(page->bounds).x, 20), V2(0, 0), V2(0, 5));
   Label(row, label, 20);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20);
   return result;
}

ui_textbox _SettingsRow(ui_id id, element *page, char *label, TextBoxData *text, char *suffix = "") {
   element *row = Panel(page, RowLayout, V2(Size(page->bounds).x, 20), V2(0, 0), V2(0, 5));
   Label(row, label, 20);
   ui_textbox result = _TextBox(id, row, text, 20);
   if(IsSelected(result.e))
      Outline(result.e, GREEN);
   Label(row, suffix, 20);
   return result;
}

ui_checkbox _SettingsRow(ui_id id, element *page, char *label, u32 *value, u32 flag, 
                         v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   element *row = Panel(page, RowLayout, V2(Size(page->bounds).x, 20), V2(0, 0), V2(0, 5));
   Label(row, label, 20);
   ui_checkbox result = _CheckBox(id, row, value, flag, size, padding, margin);
   return result;
}

#define SlidingSidePanel(...) _SlidingSidePanel(GEN_UI_ID, __VA_ARGS__)
element *_SlidingSidePanel(ui_id id, element *parent, f32 width, f32 height_padding, f32 tab_width, bool right) {
   UI_SCOPE(parent->context, id.a);
   ui_slide_animation *anim = SlideAnimation(parent->context, 0, width - tab_width, 3);
   v2 pos = {};

   if(right) {
      pos = V2(parent->bounds.max.x - anim->value - tab_width, 
               parent->bounds.min.y + height_padding);
   } else {
      pos = V2(parent->bounds.min.x + anim->value - width + tab_width, 
               parent->bounds.min.y + height_padding);
   }

   element *panel = Panel(parent, NULL, 
               RectMinSize(pos, V2(width, Size(parent->bounds).y - 2 * height_padding)));
   anim->open = ContainsCursor(panel);
   return panel;
}