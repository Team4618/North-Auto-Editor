//TODO: rewrite all this stuff, its rlly ugly

#define SettingsRow(...) _SettingsRow(GEN_UI_ID, __VA_ARGS__)
ui_numberbox _SettingsRow(ui_id id, element *page, char *label, u32 *num, char *suffix = "") {
   element *row = Panel(page, V2(Size(page).x, 20), Layout(RowLayout).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_numberbox _SettingsRow(ui_id id, element *page, char *label, f32 *num, char *suffix = "") {
   element *row = Panel(page, V2(Size(page).x, 20), Layout(RowLayout).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_textbox _SettingsRow(ui_id id, element *page, char *label, TextBoxData *text, char *suffix = "") {
   element *row = Panel(page, V2(Size(page).x, 20), Layout(RowLayout).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_textbox result = _TextBox(id, row, text, 20);
   if(IsSelected(result.e))
      Outline(result.e, GREEN);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_checkbox _SettingsRow(ui_id id, element *page, char *label, u32 *value, u32 flag, 
                         v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   element *row = Panel(page, V2(Size(page).x, 20), Layout(RowLayout).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_checkbox result = _CheckBox(id, row, value, flag, size, padding, margin);
   return result;
}