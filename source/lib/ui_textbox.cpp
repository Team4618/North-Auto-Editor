//TODO: copy/paste
struct TextBoxData {
   char *text;
   u32 size;
   u32 used;
   u32 cursor;
};

struct ui_textbox {
   element *e;
   TextBoxData *data;
   bool clicked;
   bool enter;
   bool changed;
};

//TODO: use stb_textedit?

//TODO: padding & margin
#define TextBox(...) _TextBox(GEN_UI_ID, __VA_ARGS__)
ui_textbox _TextBox(ui_id id, element *parent, TextBoxData *data, f32 line_height) {
   UIContext *context = parent->context;
   element *e = _Panel(id, parent, Size(/*GetMaxCharWidth(font, line_height)*/ 15 * data->size, line_height).Captures(INTERACTION_SELECT));

   string drawn_text = String(data->text, data->used);
   Background(e, V4(0.7, 0.7, 0.7, 1));
   Text(e, drawn_text, e->bounds.min, line_height, BLACK);

   data->cursor = Clamp(0, data->used, data->cursor);

   if(data->used > 0) {
      v2 cursor_pos;
      if(data->cursor == data->used) {
         cursor_pos = V2(GetCharBounds(context, drawn_text, data->cursor - 1, e->bounds.min, line_height).max.x,
                         e->bounds.min.y);
      } else {
         cursor_pos = V2(GetCharBounds(context, drawn_text, data->cursor, e->bounds.min, line_height).min.x,
                         e->bounds.min.y);
      }

      Rectangle(e, RectMinSize(cursor_pos, V2(5, line_height)), V4(0, 0, 0, 0.2));
   }

   bool changed = false;
   InputState *input = &context->input_state;
   if(IsSelected(e)) {
      if((input->key_char != 0) && (data->used < data->size)) {
         data->used++;
         for(u32 i = data->used - 1; i > data->cursor; i--) {
            data->text[i] = data->text[i - 1];
         }
         data->text[data->cursor++] = input->key_char;
         changed = true;
      } 
      if(input->key_backspace && (data->used > 0) && (data->cursor > 0)) {
         for(u32 i = data->cursor - 1; i < data->used - 1; i++) {
            data->text[i] = data->text[i + 1];
         }
         data->used--;
         data->cursor--;
         changed = true;
      }
      if(input->key_left_arrow && (data->cursor > 0)) {
         data->cursor--;
      }
      if(input->key_right_arrow && (data->cursor < data->used)) {
         data->cursor++;
      }
   }

   ui_textbox result = {};
   result.e = e;
   result.data = data;
   result.clicked = WasClicked(e);
   result.enter = IsSelected(e) && input->key_enter;
   result.changed = changed;
   return result;
}

#define StaticTextBoxData(name, char_count) static TextBoxData name = {}; do{ static char __buffer[char_count] = {}; name.text = __buffer; name.size = ArraySize(__buffer); }while(false) 
#define InitTextBoxData(data, buffer) do{ (data)->text = (buffer); (data)->size = ArraySize(buffer); }while(false)

string GetText(TextBoxData data) {
   return String(data.text, data.used);
}

string GetText(ui_textbox textbox) {
   return GetText(*textbox.data);
}

void SetText(TextBoxData *data, string text) {
   u32 size = Min(text.length, data->size);
   data->used = size;
   Copy(text.text, size, data->text);
}

void Clear(ui_textbox textbox) {
   textbox.data->used = 0;
}

void CopyStringToTextbox(string s, TextBoxData *textbox) {
   u32 length = Min(textbox->size, s.length);
   Copy(s.text, length, textbox->text);
   textbox->used = length;
}

struct numberbox_persistent_data { 
   bool init;
   char buffer[8];
   TextBoxData text;
};

struct ui_numberbox {
   ui_textbox textbox;
   bool valid_enter;
   bool enter;
   bool valid;

   union {
      u32 u32_value;
      f32 f32_value;
   };
};

typedef bool (*numberbox_is_valid_callback)(string text);

ui_numberbox _TextBox(ui_id id, element *parent, string value, f32 line_height, 
                      numberbox_is_valid_callback valid_func = IsUInt)
{
   UIContext *context = parent->context;
   numberbox_persistent_data *data = (numberbox_persistent_data *) _GetOrAllocate(id, context, sizeof(numberbox_persistent_data));
   InputState *input = &context->input_state;

   if(!data->init) {
      data->init = true;
      data->text.text = data->buffer;
      data->text.size = ArraySize(data->buffer);

      CopyStringToTextbox(value, &data->text);
   }

   ui_textbox textbox = _TextBox(id, parent, &data->text, line_height);
   bool valid = (data->text.used > 0) && valid_func(GetText(data->text));
   bool different_values = valid ? (GetText(data->text) != value) : true;

   v4 outline_colour;
   if(!valid) { 
      outline_colour = RED;
   } else if(different_values) {
      outline_colour = V4(1, 165.0/255.0, 0, 1);
   } else {
      outline_colour = GREEN;
   }
   
   Outline(textbox.e, outline_colour, IsSelected(textbox.e) ? 4 : 2);

   if(textbox.enter && input->ctrl_down) {
      CopyStringToTextbox(value, &data->text);
   }

   if((IsHot(textbox.e) || IsSelected(textbox.e)) && different_values) {
      v2 overlay_pos = textbox.e->bounds.min - V2(0, Size(textbox.e).y);
      Rectangle(context->overlay, RectMinSize(overlay_pos, Size(textbox.e)), V4(0.7, 0.7, 0.7, 1));
      Text(context->overlay, Concat(Literal("Current Value:"), value), overlay_pos, line_height, BLACK);
   }

   ui_numberbox result = {};
   result.textbox = textbox;
   result.valid_enter = textbox.enter && valid && !input->ctrl_down;
   result.enter = textbox.enter;
   result.valid = valid;
   return result;
}

ui_numberbox _TextBox(ui_id id, element *parent, u32 value, f32 line_height, 
                      numberbox_is_valid_callback valid_func = IsUInt)
{
   ui_numberbox result = _TextBox(id, parent, ToString(value), line_height, valid_func);
   result.u32_value = result.valid ? ToU32(GetText(result.textbox)) : 0;
   return result;
}

ui_numberbox _TextBox(ui_id id, element *parent, u32 *value, f32 line_height, 
                      numberbox_is_valid_callback valid_func = IsUInt)
{
   ui_numberbox result = _TextBox(id, parent, *value, line_height, valid_func);
   if(result.valid_enter) {
      *value = result.u32_value;
   }
   return result;
}

ui_numberbox _TextBox(ui_id id, element *parent, f32 value, f32 line_height, 
                      numberbox_is_valid_callback valid_func = IsNumber)
{
   ui_numberbox result = _TextBox(id, parent, ToString(value), line_height, valid_func);
   result.f32_value = result.valid ? ToF32(GetText(result.textbox)) : 0;
   return result;
}

ui_numberbox _TextBox(ui_id id, element *parent, f32 *value, f32 line_height, 
                      numberbox_is_valid_callback valid_func = IsNumber)
{
   ui_numberbox result = _TextBox(id, parent, *value, line_height, valid_func);
   if(result.valid_enter) {
      *value = result.f32_value;
   }
   return result;
}