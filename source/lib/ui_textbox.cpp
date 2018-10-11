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

//TODO: padding & margin
#define TextBox(...) _TextBox(GEN_UI_ID, __VA_ARGS__)
ui_textbox _TextBox(ui_id id, element *parent, TextBoxData *data, f32 line_height) {
   UIContext *context = parent->context;
   sdfFont *font = context->font;
   element *e = _Panel(id, parent, NULL, V2(GetMaxCharWidth(font, line_height) * data->size, 
                       line_height));
   ui_click inter = DefaultClickInteraction(e);

   string drawn_text = { data->text, data->used };
   Background(e, V4(0.7, 0.7, 0.7, 1));
   Text(e, drawn_text, e->bounds.min, line_height);

   data->cursor = Clamp(0, data->used, data->cursor);

   if(data->used > 0) {
      v2 cursor_pos;
      if(data->cursor == data->used) {
         cursor_pos = V2(GetCharBounds(font, drawn_text, data->cursor - 1, e->bounds.min, line_height).max.x,
                         e->bounds.min.y);
      } else {
         cursor_pos = V2(GetCharBounds(font, drawn_text, data->cursor, e->bounds.min, line_height).min.x,
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
   result.clicked = inter.clicked;
   result.enter = IsSelected(e) && input->key_enter;
   result.changed = changed;
   return result;
}

string GetText(TextBoxData data) {
   return String(data.text, data.used);
}

void CopyStringToTextbox(string s, TextBoxData *textbox) {
   u32 length = Min(textbox->size, s.length);
   Copy(s.text, length, textbox->text);
   textbox->used = length;
}

//TODO: clean up number boxes
struct numberbox_persistent_data { 
   bool init;
   char buffer[8];
   TextBoxData text;
   union {
      u32 last_u32;
      f32 last_f32;
   };
};

struct ui_numberbox {
   ui_textbox textbox;
   bool valid_enter;
   bool valid_changed;
   bool enter;
   bool valid;
};

//TODO: do we only want to set "value" when we actually change the textbox (textbox.changed)?
ui_numberbox _TextBox(ui_id id, element *parent, u32 *value, f32 line_height) {
   UIContext *context = parent->context;
   numberbox_persistent_data *data = (numberbox_persistent_data *) _GetOrAllocate(id, context, sizeof(numberbox_persistent_data));
   
   if(!data->init) {
      data->init = true;
      data->text.text = data->buffer;
      data->text.size = ArraySize(data->buffer);

      CopyStringToTextbox(ToString(*value), &data->text);
   }

   if(*value != data->last_u32) {
      CopyStringToTextbox(ToString(*value), &data->text);
   }

   ui_textbox textbox = _TextBox(id, parent, &data->text, line_height);
   bool valid = (data->text.used > 0) && IsUInt(GetText(data->text));

   if(valid) {
      *value = ToU32(GetText(data->text));
   }

   if(IsSelected(textbox.e)) {
      Outline(textbox.e, valid ? GREEN : RED);
   }

   data->last_u32 = *value;

   ui_numberbox result = {};
   result.textbox = textbox;
   result.valid_enter = textbox.enter && valid;
   result.valid_changed = textbox.changed && valid;
   result.enter = textbox.enter;
   result.valid = valid;
   return result;
}

ui_numberbox _TextBox(ui_id id, element *parent, f32 *value, f32 line_height) {
   UIContext *context = parent->context;
   numberbox_persistent_data *data = (numberbox_persistent_data *) _GetOrAllocate(id, context, sizeof(numberbox_persistent_data));
   
   if(!data->init) {
      data->init = true;
      data->text.text = data->buffer;
      data->text.size = ArraySize(data->buffer);

      CopyStringToTextbox(ToString(*value), &data->text);
   }

   if(*value != data->last_f32) {
      CopyStringToTextbox(ToString(*value), &data->text);
   }

   ui_textbox textbox = _TextBox(id, parent, &data->text, line_height);
   bool valid = (data->text.used > 0) && IsNumber(GetText(data->text));

   if(valid) {
      *value = ToF32(GetText(data->text));
   }

   if(IsSelected(textbox.e)) {
      Outline(textbox.e, valid ? GREEN : RED);
   }

   data->last_f32 = *value;

   ui_numberbox result = {};
   result.textbox = textbox;
   result.valid_enter = textbox.enter && valid; 
   result.valid_changed = textbox.changed && valid; 
   result.enter = textbox.enter;
   result.valid = valid;
   return result;
}