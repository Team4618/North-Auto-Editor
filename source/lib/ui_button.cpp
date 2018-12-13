enum button_visual_flags {
   BUTTON_DISABLED = (1 << 0),
   BUTTON_SELECTED = (1 << 1)
};

//TODO: text padding
struct button_style {
   v4 colour;
   v4 selected_colour;
   v4 disabled_colour;

   v4 outline;
   v4 hot_outline;
   v4 active_outline;

   v4 text_colour;
   v4 disabled_text_colour;

   f32 height;
   v2 padding;
   v2 margin;

   u32 flags;

   button_style IsSelected(bool selected) {
      button_style result = *this;
      result.flags |= (selected ? BUTTON_SELECTED : 0);
      return result;
   }

   button_style IsEnabled(bool enabled) {
      button_style result = *this;
      result.flags |= (enabled ? 0 : BUTTON_DISABLED);
      return result;
   }

   button_style Padding(f32 x, f32 y) {
      button_style result = *this;
      result.padding = V2(x, y);
      return result;
   }
};

button_style ButtonStyle(v4 colour, v4 selected_colour, v4 disabled_colour, 
                         v4 outline, v4 hot_outline, v4 active_outline, 
                         v4 text_colour, v4 disabled_text_colour,
                         f32 height, v2 padding, v2 margin) {
   button_style result = {};
   result.colour = colour;
   result.selected_colour = selected_colour;
   result.disabled_colour = disabled_colour;
   
   result.outline = outline;
   result.hot_outline = hot_outline;
   result.active_outline = active_outline;
   
   result.text_colour = text_colour;
   result.disabled_text_colour = disabled_text_colour;

   result.height = height;
   result.padding = padding;
   result.margin = margin;
   return result;
}
 
struct ui_button {
   element *e;
   bool clicked;
};

//TODO: icons in buttons
// Should we just make buttons a layout and put elements inside them (eg. labels, icons)?

#define Button(...) _Button(GEN_UI_ID, __VA_ARGS__)
ui_button _Button(ui_id id, element *parent, string text, button_style style) {
   UIContext *context = parent->context;
   InputState input = context->input_state;
   f32 width = TextWidth(context, text, style.height);
   element *e = _Panel(id, parent, V2(width, style.height), Padding(style.padding).Margin(style.margin).Captures(INTERACTION_CLICK));
   bool disabled = style.flags & BUTTON_DISABLED;

   if(disabled) {
      Background(e, style.disabled_colour);
   } else if (style.flags & BUTTON_SELECTED) {
      Background(e, style.selected_colour);
   } else {
      Background(e, style.colour);
   }

   Text(e, text, e->bounds.min, Size(e->bounds).y, disabled ? style.disabled_text_colour : style.text_colour);
   
   if(IsActive(e) && !disabled) {
      Outline(e, style.active_outline);
   } else if(IsHot(e) && !disabled) {
      Outline(e, style.hot_outline);
   } else {
      Outline(e, style.outline);
   }

   ui_button result = {};
   result.e = e;
   result.clicked = WasClicked(e) && !disabled;
   return result;
}

ui_button _Button(ui_id id, element *parent, char *s, button_style style) {
   return _Button(id, parent, Literal(s), style);
}