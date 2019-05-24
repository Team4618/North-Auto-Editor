struct image {
   u32 *texels;
   u32 width;
   u32 height;
   bool valid;
};

image ReadImage(char *path, bool in_exe_directory = false);
image ReadImage(string path, bool in_exe_directory = false);
void FreeImage(image *i);

struct texture {
   v2 size;
   GLuint handle;
};

texture loadTexture(char *path, bool in_exe_directory = false);
texture createTexture(u32 *texels, u32 width, u32 height);
void deleteTexture(texture tex);

struct glyph_texture {
   u32 codepoint;
   texture tex;
   v2 size_over_line_height;
   f32 xadvance_over_line_height;
   f32 ascent_over_line_height;
};

struct loaded_font {
   MemoryArena *arena; //NOTE: the loaded_font ownes this arena, noone else should reset it
   glyph_texture *glyphs[128];
   stbtt_fontinfo fontinfo;

   f32 baseline_from_top_over_line_height;
};

glyph_texture *getOrLoadGlyph(loaded_font *font, u32 codepoint);

enum RenderCommandType {
   RenderCommand_Texture,
   RenderCommand_Rectangle,
   RenderCommand_Line
};

struct RenderCommand {
   RenderCommandType type;
   RenderCommand *next;
   
   union {
      struct { 
         rect2 bounds;
         rect2 uvBounds; //NOTE: specified in texture space, not 0 to 1
         texture tex; 
         v4 colour;
      } drawTexture;
      
      struct {
         rect2 bounds;
         v4 colour;

         bool background;
      } drawRectangle;
      
      struct {
         v4 colour;
         f32 thickness;
         u32 point_count;
         v2 *points;
         bool closed;

         bool outline;
      } drawLine;
   };
};

struct ui_id {
   char *loc;
   u64 a;
   u64 b;
};

ui_id UIID(char *loc, u64 a, u64 b) {
   ui_id result = {loc, a, b};
   return result;
}

bool operator== (ui_id a, ui_id b) {
   return (a.a == b.a) && (a.b == b.b);
}

bool operator!= (ui_id a, ui_id b) {
   return (a.a != b.a) || (a.b != b.b);
}

//TODO: im not a fan of this, profile to see how spread out GEN_UI_ID id's are
ui_id operator+ (ui_id a, ui_id b) {
   return UIID(a.loc, a.a + b.a, a.b + b.b);
}

#define __STRINGIFY(x) #x
#define __TOSTRING(x) __STRINGIFY(x)

#define GEN_UI_ID UIID(__FILE__ __TOSTRING(__LINE__), (u64) (__FILE__ __TOSTRING(__LINE__)), 0)
#define POINTER_UI_ID(pointer) UIID(__FILE__ __TOSTRING(__LINE__), (u64) (__FILE__ __TOSTRING(__LINE__)), (u64) pointer)
#define NULL_UI_ID UIID((char *)0, (u64) 0, (u64) 0)

struct InputState {
   //NOTE: Mouse pos, scroll & buttons
   v2 last_pos;
   v2 pos;
   f32 vscroll;
   f32 hscroll;
   
   bool left_down;
   bool left_up;

   bool right_down;
   bool right_up;
   
   //NOTE: keyboard text entry stuff
   char key_char;
   bool key_backspace;
   bool key_enter;
   bool key_up_arrow;
   bool key_down_arrow;
   bool key_left_arrow;
   bool key_right_arrow;
   bool key_esc;
   bool alt_down;
   bool ctrl_down;
};

struct persistent_hash_link {
   persistent_hash_link *next;
   ui_id id;
   u8 *data;
};

enum UIDebugMode {
   UIDebugMode_Disabled,
   UIDebugMode_ElementPick,
   UIDebugMode_ElementSelected,
   UIDebugMode_Memory,
   UIDebugMode_Performance,
};

struct element;
struct UIContext {
   MemoryArena *frame_arena; //NOTE: owned by UIContext
   loaded_font *font;

   element *overlay;

   UIDebugMode debug_mode;
   element *debug_hot_e;
   ui_id debug_selected;
   element *debug_selected_e;

   MemoryArena *persistent_arena; //NOTE: owned by UIContext
   persistent_hash_link *persistent_hash[128];
  
   ui_id hot_e;
   v2 local_cursor;
   ui_id new_hot_e;
   v2 new_local_cursor;

   ui_id last_active_e;
   ui_id active_e;
   ui_id new_active_e;   
   
   ui_id clicked_e;
   ui_id new_clicked_e;
   
   ui_id selected_e;
   
   ui_id dragged_e;
   v2 drag;
   ui_id new_dragged_e;
   v2 new_drag;
   
   ui_id vscroll_e;
   f32 vscroll;
   ui_id new_vscroll_e;
   f32 new_vscroll;

   ui_id hscroll_e;
   f32 hscroll;
   ui_id new_hscroll_e;
   f32 new_hscroll;

   ui_id filedrop_e;
   ui_id new_filedrop_e;
   MemoryArena *filedrop_arena; //NOTE: owned by UIContext
   u32 filedrop_count;
   string *filedrop_names;

   f64 curr_time;
   f64 dt;
   f32 fps;

   ui_id scope_id;

   //below will probably get rewritten   
   InputState input_state;
};

typedef void (*layout_calculate_size_callback)(element *e, u8 *layout_data);
typedef rect2 (*layout_elem_callback)(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size);

enum layout_flags {
   Layout_Width = (1 << 0),
   Layout_Height = (1 << 1),
   Layout_Placed = (1 << 2),
};

struct element {
   UIContext *context;
   element *parent;
   element *next;
   
   element *curr_child;
   element *first_child;
   
   ui_id id;
   
   rect2 bounds;
   v2 padding;
   v2 margin;

   rect2 cliprect;
   RenderCommand *first_command;
   RenderCommand *curr_command;
   
   u32 captures;

   u8 *layout_data;
   layout_calculate_size_callback calculate_size;
   layout_elem_callback layout_elem;

   u32 layout_flags;
   bool layout_locked; //NOTE: this means theres a child element that isnt finalized yet
};

v2 Size(element *e) {
   return Size(e->bounds);
}

v2 Center(element *e) {
   return Center(e->bounds);
}

bool IsFinalized(element *e) {
   return (e->layout_flags & Layout_Width) &&
          (e->layout_flags & Layout_Height) &&
          (e->layout_flags & Layout_Placed);
}

//----------------------------------------------------
//TODO: test this, remove most uses of POINTER_UI_ID
#define UI_SCOPE(context_or_e, ptr) _ui_scope _UI_SCOPE_##__LINE__ ((context_or_e), POINTER_UI_ID(ptr)); 
struct _ui_scope {
   UIContext *context;
   ui_id prev_scope_id;

   _ui_scope(UIContext *context_in, ui_id scope_id) {
      context = context_in;
      prev_scope_id = context->scope_id;
      context->scope_id = scope_id;
   }

   _ui_scope(element *e, ui_id scope_id) {
      context = e->context;
      prev_scope_id = context->scope_id;
      context->scope_id = scope_id;
   }

   ~_ui_scope() {
      context->scope_id = prev_scope_id;
   }
};
//----------------------------------------------------

element *beginFrame(v2 window_size, UIContext *context, f32 dt) {
   context->hot_e = context->new_hot_e;
   context->local_cursor = context->new_local_cursor;
   context->new_hot_e = NULL_UI_ID;
   context->new_local_cursor = V2(0, 0);

   context->last_active_e = context->active_e;
   context->active_e = context->new_active_e;
   context->new_active_e = NULL_UI_ID;
   
   context->clicked_e = context->new_clicked_e;
   context->new_clicked_e = NULL_UI_ID;
   
   context->dragged_e = context->new_dragged_e;
   context->drag = context->new_drag;
   context->new_dragged_e = NULL_UI_ID;
   context->new_drag = V2(0, 0);

   context->vscroll_e = context->new_vscroll_e;
   context->vscroll = context->new_vscroll;
   context->new_vscroll_e = NULL_UI_ID;
   context->new_vscroll = 0;

   context->hscroll_e = context->new_hscroll_e;
   context->hscroll = context->new_hscroll;
   context->new_hscroll_e = NULL_UI_ID;
   context->new_hscroll = 0;

   context->filedrop_e = context->new_filedrop_e;
   context->new_filedrop_e = NULL_UI_ID;
   
   context->curr_time += dt;
   context->dt = dt;
   context->fps = 1.0 / dt;

   context->debug_hot_e = NULL;
   context->debug_selected_e = NULL;

   Reset(context->frame_arena);
   context->overlay = PushStruct(context->frame_arena, element);
   context->overlay->context = context;
   context->overlay->bounds = RectMinSize(V2(0, 0), window_size);
   context->overlay->cliprect = RectMinSize(V2(0, 0), window_size);
   context->overlay->id.loc = "overlay";
   context->overlay->layout_flags = Layout_Width | Layout_Height | Layout_Placed;

   element *root = PushStruct(context->frame_arena, element);
   root->context = context;
   root->bounds = RectMinSize(V2(0, 0), window_size);
   root->cliprect = RectMinSize(V2(0, 0), window_size);
   root->id.loc = "root";
   root->layout_flags = Layout_Width | Layout_Height | Layout_Placed;
   return root;
}

void addCommand(element *e, RenderCommand *command) {
   if(e->first_command == NULL) {
      e->first_command = command;
   } else {
      e->curr_command->next = command;
   }
   
   e->curr_command = command;
}

//Render command helper functions---------------------------------
#define RED V4(1, 0, 0, 1)
#define BLUE V4(0, 0, 1, 1)
#define GREEN V4(0, 1, 0, 1)
#define WHITE V4(1, 1, 1, 1)
#define BLACK V4(0, 0, 0, 1)

RenderCommand *Rectangle(element *e, rect2 bounds, v4 colour) {
   UIContext *context = e->context;
   RenderCommand *result = PushStruct(context->frame_arena, RenderCommand);
   result->type = RenderCommand_Rectangle;
   result->next = NULL;
   result->drawRectangle.bounds = bounds;
   result->drawRectangle.colour = colour;
   
   addCommand(e, result);
   return result;
}

RenderCommand *Background(element *e, v4 colour) {
   RenderCommand *result = Rectangle(e, e->bounds, colour);
   result->drawRectangle.background = true;
   return result;
}

RenderCommand *_Line(element *e, v4 colour, f32 thickness, v2 *points, u32 point_count, bool closed = false) {
   UIContext *context = e->context;
   MemoryArena *arena = context->frame_arena;

   RenderCommand *result = PushStruct(arena, RenderCommand);
   result->type = RenderCommand_Line;
   result->next = NULL;
   
   result->drawLine.colour = colour;
   result->drawLine.thickness = thickness;
   result->drawLine.point_count = point_count;
   result->drawLine.points = PushArrayCopy(arena, v2, points, point_count);
   result->drawLine.closed = closed;
   
   addCommand(e, result);
   return result;
}

#define Line(e, colour, thickness, ...) do {v2 __points[] = {__VA_ARGS__}; _Line(e, colour, thickness, __points, ArraySize(__points), false); } while(false)
#define Loop(e, colour, thickness, ...) do {v2 __points[] = {__VA_ARGS__}; _Line(e, colour, thickness, __points, ArraySize(__points), true); } while(false)

void Outline(element *e, rect2 b, v4 colour, f32 thickness = 2) {
   v2 __points[] = {
      V2(b.min.x, b.min.y), V2(b.max.x, b.min.y), 
      V2(b.max.x, b.max.y), V2(b.min.x, b.max.y)
   };
   RenderCommand *result = _Line(e, colour, thickness, __points, ArraySize(__points), true); 
   result->drawLine.outline = true;
}

void Outline(element *e, v4 colour, f32 thickness = 2) {
   Outline(e, e->bounds, colour, thickness);
}

void Circle(element *e, v2 center, f32 radius, v4 colour, f32 thickness = 2) {
   u32 point_count = 20;
   v2 *points = PushTempArray(v2, point_count);
   for(u32 i = 0; i < point_count; i++) {
      f32 angle = i * (360.0f / (f32)point_count);
      points[i] = center + radius * V2(cosf(ToRadians(angle)), -sinf(ToRadians(angle)));
   }
   _Line(e, colour, thickness, points, point_count, true);
}

void Arc(element *e, v2 center, f32 radius, f32 angle1, f32 angle2, bool clockwise, v4 colour, f32 thickness = 2) {
   f32 abetween = AngleBetween(angle1, angle2, clockwise);
   u32 point_count = 20;
   v2 *points = PushTempArray(v2, point_count);
   for(u32 i = 0; i < point_count; i++) {
      f32 angle = angle1 + i * (abetween / (f32)(point_count - 1));
      points[i] = center + radius * V2(cosf(ToRadians(angle)), -sinf(ToRadians(angle)));
   }
   _Line(e, colour, thickness, points, point_count, false);
}

v2 DirectionNormal(f32 angle) {
   return V2(cosf(ToRadians(angle)), 
             -sinf(ToRadians(angle)));
}

f32 Angle(v2 v) {
   return ToDegrees(atan2(-v.y, v.x));
}

RenderCommand *Texture(element *e, texture tex, rect2 bounds, v4 colour = WHITE) {
   UIContext *context = e->context;
   RenderCommand *result = PushStruct(context->frame_arena, RenderCommand);
   result->type = RenderCommand_Texture;
   result->next = NULL;
   result->drawTexture.bounds = bounds;
   result->drawTexture.uvBounds = RectMinSize(V2(0, 0), tex.size);
   result->drawTexture.tex = tex;
   result->drawTexture.colour = colour;
   
   addCommand(e, result);
   return result;
}

struct ui_glyph_layout {
   glyph_texture *glyph_tex;
   rect2 bounds;
};

struct ui_text_layout {
   u32 glyph_count;
   ui_glyph_layout *glyphs;
   f32 baseline;
   rect2 text_bounds;
};

//TODO: kerning
//TODO: multiple lines
ui_text_layout LayoutText(UIContext *context, string text, f32 line_height) {
   loaded_font *font = context->font;
   
   ui_text_layout result = {};
   result.baseline = font->baseline_from_top_over_line_height * line_height;
   result.glyph_count = text.length;
   result.glyphs = PushTempArray(ui_glyph_layout, text.length);

   f32 x = 0;
   for(u32 i = 0; i < text.length; i++) {
      ui_glyph_layout *glyph_layout = result.glyphs + i;
      glyph_texture *glyph = getOrLoadGlyph(font, text.text[i]);
      
      v2 size = line_height * glyph->size_over_line_height;
      f32 xadvance = line_height * glyph->xadvance_over_line_height;
      f32 ascent = line_height * glyph->ascent_over_line_height;
      v2 glyph_pos = V2(x, result.baseline + ascent);

      glyph_layout->bounds = RectMinSize(glyph_pos, size);
      glyph_layout->glyph_tex = glyph;

      x += xadvance;
   }
   result.text_bounds = RectMinSize(V2(0, 0), V2(x, line_height));

   return result;
}

f32 TextWidth(UIContext *context, string text, f32 line_height) {   
   return Size(LayoutText(context, text, line_height).text_bounds).x;
}

rect2 GetCharBounds(UIContext *context, string text, u32 i, v2 pos, f32 line_height) {
   ui_text_layout layed_out_text = LayoutText(context, text, line_height);
   return pos + layed_out_text.glyphs[i].bounds;
}

void Text(element *e, ui_text_layout layed_out_text, v2 pos, v4 colour) {   
   UIContext *context = e->context;

   if(context->debug_mode == UIDebugMode_ElementPick) {
      Line(e, BLACK, 2,
           pos + V2(0, layed_out_text.baseline), 
           pos + V2(Size(layed_out_text.text_bounds).x, layed_out_text.baseline));
      Outline(e, pos + layed_out_text.text_bounds, BLACK);
   }

   for(u32 i = 0; i < layed_out_text.glyph_count; i++) {
      ui_glyph_layout *glyph = layed_out_text.glyphs + i;
      Texture(e, glyph->glyph_tex->tex, pos + glyph->bounds, colour);
   }
}

void Text(element *e, string text, v2 pos, f32 line_height, v4 colour) {   
   UIContext *context = e->context;
   ui_text_layout layed_out_text = LayoutText(context, text, line_height);
   Text(e, layed_out_text, pos, colour);
}

void Text(element *e, char *s, v2 pos, f32 height, v4 colour) {
   Text(e, Literal(s), pos, height, colour);
}

//---------------------------------------------------------------------

v2 Cursor(element *e) {
   UIContext *context = e->context;
   return context->input_state.pos;
}

bool ContainsCursor(element *e) {
   UIContext *context = e->context;
   return Contains(e->cliprect, context->input_state.pos);
}

//Interactions---------------------------------------------------------
enum ui_interaction_captures {
   INTERACTION_HOT = (1 << 0),
   _INTERACTION_ACTIVE = (1 << 1),
   INTERACTION_ACTIVE = INTERACTION_HOT | _INTERACTION_ACTIVE,
   _INTERACTION_CLICK = (1 << 2),
   INTERACTION_CLICK = INTERACTION_ACTIVE | _INTERACTION_CLICK,
   _INTERACTION_SELECT = (1 << 3),
   INTERACTION_SELECT = INTERACTION_CLICK | _INTERACTION_SELECT,
   _INTERACTION_DRAG = (1 << 4),
   INTERACTION_DRAG = INTERACTION_ACTIVE | _INTERACTION_DRAG,
   _INTERACTION_VERTICAL_SCROLL = (1 << 5),
   INTERACTION_VERTICAL_SCROLL = INTERACTION_HOT | _INTERACTION_VERTICAL_SCROLL,
   _INTERACTION_HORIZONTAL_SCROLL = (1 << 6),
   INTERACTION_HORIZONTAL_SCROLL = INTERACTION_HOT | _INTERACTION_HORIZONTAL_SCROLL,
   _INTERACTION_FILEDROP = (1 << 7),
   INTERACTION_FILEDROP = INTERACTION_HOT | _INTERACTION_FILEDROP
};

#define AssertHasFlags(var, flags) Assert(((var) & (flags)) == (flags))

bool IsHot(element *e) {
   AssertHasFlags(e->captures, INTERACTION_HOT);
   return e->id == e->context->hot_e;
}

v2 GetLocalCursor(element *e) {
   AssertHasFlags(e->captures, INTERACTION_HOT);
   return (e->id == e->context->hot_e) ? e->context->local_cursor : V2(0, 0);
}

bool IsActive(element *e) {
   AssertHasFlags(e->captures, INTERACTION_ACTIVE);
   return e->id == e->context->active_e;
}

bool BecameActive(element *e) {
   AssertHasFlags(e->captures, INTERACTION_ACTIVE);
   return IsActive(e) && (e->id != e->context->last_active_e);
}

bool WasClicked(element *e) {
   AssertHasFlags(e->captures, INTERACTION_CLICK);
   return e->id == e->context->clicked_e;
}

bool IsSelected(element *e) {
   AssertHasFlags(e->captures, INTERACTION_SELECT);
   return e->id == e->context->selected_e;
}

void ClearSelected(element *e) {
   e->context->selected_e = NULL_UI_ID;
}

v2 GetDrag(element *e) {
   AssertHasFlags(e->captures, INTERACTION_DRAG);
   return (e->id == e->context->dragged_e) ? e->context->drag : V2(0, 0);
}

f32 GetVerticalScroll(element *e) {
   AssertHasFlags(e->captures, INTERACTION_VERTICAL_SCROLL);
   return (e->id == e->context->vscroll_e) ? e->context->vscroll : 0;
}

f32 GetHorizontalScroll(element *e) {
   AssertHasFlags(e->captures, INTERACTION_HORIZONTAL_SCROLL);
   return (e->id == e->context->hscroll_e) ? e->context->hscroll : 0;
}

struct ui_dropped_files {
   u32 count;
   string *names;
};

ui_dropped_files GetDroppedFiles(element *e) {
   AssertHasFlags(e->captures, INTERACTION_FILEDROP);
   
   ui_dropped_files result = {};
   if(e->id == e->context->filedrop_e) {
      result.count = e->context->filedrop_count;
      result.names = e->context->filedrop_names;
   }
   return result;
}

void uiTick(element *e) {
   UIContext *context = e->context;
   InputState *input = &context->input_state; 
   Assert(IsFinalized(e));

   if(context->debug_mode == UIDebugMode_ElementPick) {
      if(Contains(e->cliprect, input->pos)) {
         context->debug_hot_e = e;
      }
   } else {
      if(e->captures & INTERACTION_HOT) {
         bool can_become_hot = (context->active_e == NULL_UI_ID) || (context->active_e == e->id);
         if(Contains(e->cliprect, input->pos) && can_become_hot) {
            context->new_hot_e = e->id;
            context->new_local_cursor = input->pos - e->bounds.min;
         }
      }

      if(e->captures & _INTERACTION_ACTIVE) {
         bool can_become_active = IsHot(e) && input->left_down;
         bool should_remain_active = IsActive(e) && input->left_down;
         if(can_become_active || should_remain_active) {
            context->new_active_e = e->id;
         }
      }

      if(e->captures & _INTERACTION_CLICK) {
         if(IsActive(e) && IsHot(e) && input->left_up && 
            (e->id != e->context->dragged_e))
         {
            context->new_clicked_e = e->id;

            if(e->captures & _INTERACTION_SELECT) {
               context->selected_e = IsSelected(e) ? NULL_UI_ID : e->id;
            }
         }
      }

      if(e->captures & _INTERACTION_DRAG) {
         v2 drag_vector = input->pos - input->last_pos;
         if(IsActive(e) && 
            ((Length(drag_vector) > 0) || (e->id == e->context->dragged_e)))
         {
            context->new_dragged_e = e->id;
            context->new_drag = drag_vector;
         }
      }

      if(e->captures & _INTERACTION_VERTICAL_SCROLL) {
         if(ContainsCursor(e)) {
            context->new_vscroll_e = e->id;
            context->new_vscroll = input->vscroll;
         }
      }

      if(e->captures & _INTERACTION_HORIZONTAL_SCROLL) {
         if(ContainsCursor(e)) {
            context->new_hscroll_e = e->id;
            context->new_hscroll = input->hscroll;
         }
      }

      if(e->captures & _INTERACTION_FILEDROP) {
         if(IsHot(e)) {
            context->new_filedrop_e = e->id;
         }
      }
   }

   if(e->id == context->debug_selected) {
      context->debug_selected_e = e;
   }
}

//---------------------------------------------------------------------

typedef void (*layout_setup_callback)(element *e);

struct panel_args {
   v2 size;
   u32 layout_flags;

   layout_setup_callback layout_setup;
   v2 padding;
   v2 margin;
   u32 captures;

   panel_args Layout(layout_setup_callback layout_setup) {
      panel_args result = *this;
      result.layout_setup = layout_setup;
      return result;
   }

   panel_args Padding(v2 padding) {
      panel_args result = *this;
      result.padding = padding;
      return result;
   }

   panel_args Padding(f32 x, f32 y) {
      panel_args result = *this;
      result.padding = V2(x, y);
      return result;
   }

   panel_args Margin(v2 margin) {
      panel_args result = *this;
      result.margin = margin;
      return result;
   }

   panel_args Margin(f32 x, f32 y) {
      panel_args result = *this;
      result.margin = V2(x, y);
      return result;
   }

   panel_args Captures(u32 captures) {
      panel_args result = *this;
      result.captures |= captures;
      return result;
   }
};

panel_args Size(v2 size) {
   panel_args result = {};
   result.size = size;
   result.layout_flags |= (Layout_Width | Layout_Height);
   return result;   
}

panel_args Size(f32 width, f32 height) {
   return Size(V2(width, height));
}

panel_args Width(f32 width) {
   panel_args result = {};
   result.size = V2(width, 0);
   result.layout_flags |= Layout_Width;
   return result; 
}

panel_args Height(f32 height) {
   panel_args result = {};
   result.size = V2(0, height);
   result.layout_flags |= Layout_Height;
   return result;
}

//----------------------REMOVE----------------------
panel_args Layout(layout_setup_callback _layout_setup) {
   panel_args result = {};
   return result.Layout(_layout_setup);
}

panel_args Padding(v2 _padding) {
   panel_args result = {};
   return result.Padding(_padding);
}

panel_args Padding(f32 x, f32 y) {
   panel_args result = {};
   return result.Padding(V2(x, y));
}

panel_args Margin(v2 _margin) {
   panel_args result = {};
   return result.Margin(_margin);
}

panel_args Margin(f32 x, f32 y) {
   panel_args result = {};
   return result.Margin(V2(x, y));
}

panel_args Captures(u32 _captures) {
   panel_args result = {};
   return result.Captures(_captures);
}
//---------------------------------------------------

element *addElement(ui_id id, element *parent, u32 captures) {
   UIContext *context = parent->context;
   element *e = PushStruct(context->frame_arena, element);
   
   e->context = context;
   e->parent = parent;
   e->id = id + context->scope_id;
   e->captures = captures;
   e->layout_locked = false;
   
   if(parent->first_child == NULL) {
      parent->first_child = e;
   } else {
      parent->curr_child->next = e;
   }
   
   parent->curr_child = e;
   return e;
}

#define Panel(...) _Panel(GEN_UI_ID, __VA_ARGS__)
element *_Panel(ui_id id, element *parent, rect2 bounds, panel_args args = {}) {
   // Assert(IsFinalized(parent));
   element *e = addElement(id, parent, args.captures);
   
   e->bounds = bounds;
   e->cliprect = Overlap(parent->cliprect, bounds);
   e->layout_flags = Layout_Width | Layout_Height | Layout_Placed;
   if(args.layout_setup != NULL) {
         args.layout_setup(e);
   }
   
   return e;
}

// element *_Panel(ui_id id, element *parent, v2 size, panel_args args = {}) {
//    Assert(parent->layout_func != NULL);
//    rect2 bounds = parent->layout_func(parent, parent->layout_data, size, args._padding, args._margin);
//    return _Panel(id, parent, bounds, args);
// }

element *_Panel(ui_id id, element *parent, panel_args args) {
   //If this fails, parent doesnt have a layout set
   Assert(parent->layout_elem != NULL);
   //If this fails, one of the children of "parent" is using defered layout and hasn't finished yet
   Assert(!parent->layout_locked);

   if((args.layout_flags & Layout_Width) &&
      (args.layout_flags & Layout_Height) &&
      IsFinalized(parent))
   {
      rect2 bounds = parent->layout_elem(parent, parent->layout_data, 
                                         args.size, args.padding, args.margin);
      element *e = _Panel(id, parent, bounds, args);
      
      e->padding = args.padding;
      e->margin = args.margin;
      return e;
   } else {
      element *e = addElement(id, parent, args.captures);
   
      e->bounds = RectMinSize(V2(0, 0), args.size);
      e->layout_flags = args.layout_flags;
      if(args.layout_setup != NULL) {
            args.layout_setup(e);
      }
      
      if(IsFinalized(parent)) {
         parent->layout_locked = true;
      }

      e->padding = args.padding;
      e->margin = args.margin;
      return e;
   }   
}

void CalculateSizes(element *e) {
   for(element *child = e->first_child; child; child = child->next) {
      if(!(child->layout_flags & Layout_Width) || 
         !(child->layout_flags & Layout_Height)) 
      {
         CalculateSizes(child);
      }
   }

   e->calculate_size(e, e->layout_data);
}

void LayoutChildren(element *e) {
   if(IsFinalized(e)) {
      e->bounds = e->parent->bounds.min + e->bounds;
   } else {
      e->bounds = e->parent->layout_elem(e->parent, e->parent->layout_data, Size(e), e->padding, e->margin);
      e->layout_flags |= Layout_Placed;
   }

   e->cliprect = Overlap(e->parent->cliprect, e->bounds);
   
   for(RenderCommand *command = e->first_command; command; command = command->next) {
      switch(command->type) {
         case RenderCommand_Texture: {
            command->drawTexture.bounds = e->bounds.min + command->drawTexture.bounds;
         } break;
         
         case RenderCommand_Rectangle: {
            command->drawRectangle.bounds = command->drawRectangle.background ? e->bounds : (e->bounds.min + command->drawRectangle.bounds);
         } break;
         
         case RenderCommand_Line: {
            if(command->drawLine.outline) {
               v2 points[] = {
                  V2(e->bounds.min.x, e->bounds.min.y), V2(e->bounds.max.x, e->bounds.min.y), 
                  V2(e->bounds.max.x, e->bounds.max.y), V2(e->bounds.min.x, e->bounds.max.y)
               };

               Copy(points, sizeof(points), command->drawLine.points);
            } else {
               for(u32 i = 0; i < command->drawLine.point_count; i++) {
                  command->drawLine.points[i] = command->drawLine.points[i] + e->bounds.min;
               }
            }
         } break;
      }
   }

   for(element *child = e->first_child; child; child = child->next) {
      LayoutChildren(child);
   }
}

void FinalizeLayout(element *e) {
   e->parent->layout_locked = false;
   CalculateSizes(e);
   LayoutChildren(e);
}

//Common layout types-----------------------------------
//TODO: make all the layouts work properly
rect2 column_layout_elem(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   v2 *at = (v2 *) layout_data;
   v2 pos = *at;
   at->y += element_size.y + padding_size.y + margin_size.y;
   return RectMinSize(e->bounds.min + pos + padding_size, element_size);
}

void column_calculate_size(element *e, u8 *layout_data) {
   if(e->layout_flags & Layout_Width) {
      f32 height = 0;
      for(element *child = e->first_child; child; child = child->next) {
         height += Size(child).y + child->padding.y + child->margin.y;
      }
      e->bounds = RectMinSize(e->bounds.min, V2(Size(e).x, height));
      e->layout_flags |= Layout_Height;
   } else if(e->layout_flags & Layout_Height) {
      f32 width = 0;
      for(element *child = e->first_child; child; child = child->next) {
         width = Max(width, Size(child).x);
      }
      e->bounds = RectMinSize(e->bounds.min, V2(width, Size(e).y));
      e->layout_flags |= Layout_Width;
   } else {
      Assert(false);
   }
}

void ColumnLayout(element *e) {
   UIContext *context = e->context;
   e->layout_elem = column_layout_elem;
   e->calculate_size = column_calculate_size;
   e->layout_data = (u8 *) PushStruct(context->frame_arena, v2);
}

#define ColumnPanel(...) _ColumnPanel(GEN_UI_ID, __VA_ARGS__)
element *_ColumnPanel(ui_id id, element *parent, panel_args args) {
   return _Panel(id, parent, args.Layout(ColumnLayout));
}  

element *_ColumnPanel(ui_id id, element *parent, rect2 bounds, panel_args args = {}) {
   return _Panel(id, parent, bounds, args.Layout(ColumnLayout));
}

rect2 row_layout_elem(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   v2 *at = (v2 *) layout_data;
   v2 pos = *at;
   at->x += element_size.x + padding_size.x + margin_size.x;
   return RectMinSize(e->bounds.min + pos + padding_size, element_size);
}

void row_calculate_size(element *e, u8 *layout_data) {
   if(e->layout_flags & Layout_Width) {
      f32 height = 0;
      for(element *child = e->first_child; child; child = child->next) {
         height = Max(height, Size(child).y);
      }
      e->bounds = RectMinSize(e->bounds.min, V2(Size(e).x, height));
      e->layout_flags |= Layout_Height;
   } else if(e->layout_flags & Layout_Height) {
      f32 width = 0;
      for(element *child = e->first_child; child; child = child->next) {
         width += Size(child).x + child->padding.x + child->margin.x;
      }
      e->bounds = RectMinSize(e->bounds.min, V2(width, Size(e).y));
      e->layout_flags |= Layout_Width;
   } else {
      Assert(false);
   }
}

void RowLayout(element *e) {
   UIContext *context = e->context;
   e->layout_elem = row_layout_elem;
   e->calculate_size = row_calculate_size;
   e->layout_data = (u8 *) PushStruct(context->frame_arena, v2);
}

#define RowPanel(...) _RowPanel(GEN_UI_ID, __VA_ARGS__)
element *_RowPanel(ui_id id, element *parent, panel_args args) {
   return _Panel(id, parent, args.Layout(RowLayout));
}  

element *_RowPanel(ui_id id, element *parent, rect2 bounds, panel_args args = {}) {
   return _Panel(id, parent, bounds, args.Layout(RowLayout));
}

rect2 stack_layout_elem(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   return RectMinSize(e->bounds.min + padding_size + margin_size, element_size);
}

void StackLayout(element *e) {
   e->layout_elem = stack_layout_elem;
   // e->calculate_size
}

#define StackPanel(...) _StackPanel(GEN_UI_ID, __VA_ARGS__)
element *_StackPanel(ui_id id, element *parent, panel_args args) {
   return _Panel(id, parent, args.Layout(StackLayout));
}  

element *_StackPanel(ui_id id, element *parent, rect2 bounds, panel_args args = {}) {
   return _Panel(id, parent, bounds, args.Layout(StackLayout));
}

//TODO: flow layout

//--------------------------------------------------------

#define Label(...) _Label(GEN_UI_ID, __VA_ARGS__)
element *_Label(ui_id id, element *parent, string text, f32 line_height, v4 text_colour, 
                v2 p = V2(0, 0), v2 m = V2(0, 0)) 
{
   UIContext *context = parent->context;
   f32 width = TextWidth(context, text, line_height);

   element *result = _Panel(id, parent, Size(width, line_height).Padding(p).Margin(m));
   Text(result, text, result->bounds.min, line_height, text_colour);
   return result;
}

element *_Label(ui_id id, element *parent, char *text, f32 line_height, v4 text_colour,
                v2 p = V2(0, 0), v2 m = V2(0, 0)) 
{
   return _Label(id, parent, Literal(text), line_height, text_colour, p, m);
}

element *_Label(ui_id id, element *parent, string text, v2 size, f32 line_height, v4 text_colour) {
   UIContext *context = parent->context;
   ui_text_layout layed_out_text = LayoutText(context, text, line_height);
   
   element *result = _Panel(id, parent, Size(size));
   Text(result, layed_out_text, Center(result) - 0.5 * Size(layed_out_text.text_bounds), text_colour);
   return result;
}

element *_Label(ui_id id, element *parent, char *text, v2 size, f32 line_height, v4 text_colour) {
   return _Label(id, parent, Literal(text), size, line_height, text_colour);
}

//Persistent-Data-------------------------------------------------
#define GetOrAllocate(e, type) (type *) _GetOrAllocate(e->id, e->context, sizeof(type))
u8 *_GetOrAllocate(ui_id in_id, UIContext *context, u32 size) {
   u8 *result = NULL;

   ui_id id = in_id + context->scope_id;
   u32 hash = (3 * id.a + 4 * id.b) % ArraySize(context->persistent_hash); //TODO: better hash function :)
   for(persistent_hash_link *link = context->persistent_hash[hash]; link; link = link->next) {
      if(link->id == id) {
         result = link->data;
         break;
      }
   }
   
   if(result == NULL) {
      result = PushSize(context->persistent_arena, size);
      OutputDebugStringA("[ui_core] Allocating new persistant data\n");

      persistent_hash_link *new_link = PushStruct(context->persistent_arena, persistent_hash_link);
      new_link->id = id;
      new_link->data = result;
      new_link->next = context->persistent_hash[hash];
      context->persistent_hash[hash] = new_link;
   }
   
   return result;
}

#define UIPersistentData(ctx, type) (type *) _GetOrAllocate(GEN_UI_ID, ctx, sizeof(type))