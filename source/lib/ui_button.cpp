//TODO button corner styles
struct button_style {
   v4 colour;
   f32 height;
   v2 padding;
   v2 margin;
};

button_style ButtonStyle(v4 colour, f32 height, v2 padding, v2 margin) {
   button_style result = {};
   result.colour = colour;
   result.height = height;
   result.padding = padding;
   result.margin = margin;
   return result;
}

struct ui_button {
   ui_click interaction;
   element *e;
   bool clicked;
};

#define Button(...) _Button(GEN_UI_ID, __VA_ARGS__)
ui_button _Button(ui_id id, element *parent, button_style style, string text, bool enabled = true) {
   UIContext *context = parent->context;
   InputState input = context->input_state;
   f32 width = TextWidth(context->font, text, style.height);
   element *e = _Panel(id, parent, NULL, V2(width, style.height), style.padding, style.margin);
   ui_click inter = DefaultClickInteraction(e);

   Background(e, (IsActive(e) && enabled) ? 0.5 * style.colour : style.colour);
   Text(e, text, e->bounds.min, Size(e->bounds).y);
   if(IsHot(e) && enabled) {
      Outline(e, V4(0, 0, 0, 1));
   }

   ui_button result = {};
   result.interaction = inter;
   result.e = e;
   result.clicked = inter.clicked && enabled;
   return result;
}

ui_button _Button(ui_id id, element *parent, button_style style, char *s, bool enabled = true) {
   return _Button(id, parent, style, Literal(s), enabled);
}