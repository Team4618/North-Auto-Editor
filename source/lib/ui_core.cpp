struct texture {
   v2 size;
   GLuint handle;
};

struct glyphInfo {
   v2 textureLocation;
   v2 size;
   v2 offset;
   f32 xadvance;
};

struct sdfFont {
   texture sdfTexture;
   f32 native_line_height;
   f32 max_char_width;
   glyphInfo glyphs[128];
};

enum RenderCommandType {
   RenderCommand_SDF,
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
         texture sdf;
         v4 colour;
      } drawSDF;
      
      struct { 
         rect2 bounds;
         rect2 uvBounds; //NOTE: specified in texture space, not 0 to 1
         texture tex; 
      } drawTexture;
      
      struct {
         rect2 bounds;
         v4 colour;
      } drawRectangle;
      
      struct {
         v4 colour;
         v2 a;
         v2 b;
         f32 thickness;
      } drawLine;
   };
};

texture loadTexture(char *path, bool in_exe_directory = false);
texture createTexture(u32 *texels, u32 width, u32 height);
void deleteTexture(texture tex);

/**
TODO: rewrite the ui_id system so its less ad-hoc
   -ability to bind a scope to an "object"
   
   eg.
   for(LineGraph *graph = data->first; graph; graph = graph->next) {
      ui_button btn = _Button(POINTER_UI_ID(curr_graph), control_row,
                              control_button_style, curr_graph->name);
      ...
   }

   Becomes:
   for(LineGraph *graph = data->first; graph; graph = graph->next) {
      BindUIScope(graph); //macro for -> ui_scope __scope(graph);
      ui_button btn = Button(control_row, control_button_style, curr_graph->name);
      ...
   }

   keep the file & line number as a part of the id, make it a char *, it helps with debugging
*/

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
   v2 pos;
   f32 scroll;
   
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
};

struct persistent_hash_link {
   persistent_hash_link *next;
   ui_id id;
   u8 *data;
};

struct interaction {
   ui_id id;
   f32 start_time;
   v2 start_pos;
   v2 last_pos;
};

struct element;
struct UIContext {
   sdfFont *font;
   MemoryArena frame_arena;
   RenderCommand *first_command;
   RenderCommand *curr_command;
   
   MemoryArena persistent_arena;
   persistent_hash_link *persistent_hash[128];

   element *marked_hot;
   interaction hot;
   bool hot_element_changed;

   interaction active;
   bool active_element_refreshed;
   
   interaction selected;
   bool selected_element_refreshed;

   string tooltip;   
   f64 curr_time;
   f32 fps;

   ui_id scope_id;

   //below will probably get rewritten   
   InputState input_state;
};

//----------------------------------------------------
//TODO: test this, remove most uses of POINTER_UI_ID
#define UI_SCOPE(context, ptr) _ui_scope _UI_SCOPE_##__LINE__ ((context), POINTER_UI_ID(ptr)); 
struct _ui_scope {
   UIContext *context;
   ui_id prev_scope_id;

   _ui_scope(UIContext *context_in, ui_id scope_id) {
      context = context_in;
      prev_scope_id = context->scope_id;
      context->scope_id = scope_id;
   }

   ~_ui_scope() {
      context->scope_id = prev_scope_id;
   }
};
//----------------------------------------------------

interaction Interaction(ui_id id, UIContext *context) {
   interaction result = { id, context->curr_time, context->input_state.pos, context->input_state.pos };
   return result;
}

typedef rect2 (*layout_callback)(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size);

struct element {
   UIContext *context;
   element *parent;
   element *next;
   
   element *curr_child;
   element *first_child;
   
   ui_id id;
   
   rect2 bounds;
   rect2 cliprect;
   RenderCommand *first_command;
   RenderCommand *curr_command;
   
   bool visited;

   u8 *layout_data;
   layout_callback layout_func;
};

element *addElement(element *parent, ui_id id) {
   UIContext *context = parent->context;
   element *result = PushStruct(&context->frame_arena, element);
   
   result->context = context;
   result->parent = parent;
   result->id = id + context->scope_id;
   
   if(parent->first_child == NULL) {
      parent->first_child = result;
   } else {
      parent->curr_child->next = result;
   }
   
   parent->curr_child = result;

   return result;
}

element *beginFrame(v2 window_size, UIContext *context, f32 dt) {
   if(context->marked_hot != NULL) {
      context->hot_element_changed = (context->hot.id != context->marked_hot->id);
      if(context->hot_element_changed)
         context->hot = Interaction(context->marked_hot->id, context);
   } else {
      context->hot_element_changed = false;
      context->hot.id = NULL_UI_ID;
   }
   context->marked_hot = NULL;

   if(!context->active_element_refreshed) {
      context->active.id = NULL_UI_ID;
   }
   context->active_element_refreshed = false;
   
   if(!context->selected_element_refreshed) {
      context->selected.id = NULL_UI_ID;
   }
   context->selected_element_refreshed = false;
   
   context->tooltip = EMPTY_STRING;
   context->curr_time += dt;
   context->fps = 1.0 / dt;

   //TODO: defragment/garbage collect persistent hash table
   
   Reset(&context->frame_arena);
   element *root = PushStruct(&context->frame_arena, element);
   root->context = context;
   root->bounds = RectMinSize(V2(0, 0), window_size);
   root->cliprect = RectMinSize(V2(0, 0), window_size);
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
RenderCommand *Rectangle(element *e, rect2 bounds, v4 colour) {
   UIContext *context = e->context;
   RenderCommand *result = PushStruct(&context->frame_arena, RenderCommand);
   result->type = RenderCommand_Rectangle;
   result->next = NULL;
   result->drawRectangle.bounds = bounds;
   result->drawRectangle.colour = colour;
   
   addCommand(e, result);
   return result;
}

RenderCommand *Background(element *e, v4 colour) {
   return Rectangle(e, e->bounds, colour);
}

RenderCommand *Line(element *e, v2 a, v2 b, v4 colour, f32 thickness = 2) {
   UIContext *context = e->context;
   RenderCommand *result = PushStruct(&context->frame_arena, RenderCommand);
   result->type = RenderCommand_Line;
   result->next = NULL;
   result->drawLine.a = a;
   result->drawLine.b = b;
   result->drawLine.colour = colour;
   result->drawLine.thickness = thickness;
   
   addCommand(e, result);
   return result;
}

void Outline(element *e, v4 colour, f32 thickness = 2) {
   rect2 b = e->bounds;
   Line(e, V2(b.min.x, b.min.y), V2(b.max.x, b.min.y), colour, thickness);
   Line(e, V2(b.max.x, b.min.y), V2(b.max.x, b.max.y), colour, thickness);
   Line(e, V2(b.max.x, b.max.y), V2(b.min.x, b.max.y), colour, thickness);
   Line(e, V2(b.min.x, b.max.y), V2(b.min.x, b.min.y), colour, thickness);
}

RenderCommand *Texture(element *e, texture tex, rect2 bounds) {
   UIContext *context = e->context;
   RenderCommand *result = PushStruct(&context->frame_arena, RenderCommand);
   result->type = RenderCommand_Texture;
   result->next = NULL;
   result->drawTexture.bounds = bounds;
   result->drawTexture.uvBounds = RectMinSize(V2(0, 0), tex.size);
   result->drawTexture.tex = tex;
   
   addCommand(e, result);
   return result;
}

f32 TextWidth(sdfFont *font, string text, f32 line_height) {   
   f32 width = 0;
   f32 scale = line_height / font->native_line_height;

   for(u32 i = 0; i < text.length; i++) {
      u32 id = (u32) text.text[i];
      
      if(id < ArraySize(font->glyphs)) {
         glyphInfo glyph = font->glyphs[id];
               
         width += scale * glyph.xadvance;
      }
   }

   return width;
}

f32 GetMaxCharWidth(sdfFont *font, f32 line_height) {
   f32 scale = line_height / font->native_line_height;
   return font->max_char_width * scale;      
}

rect2 GetCharBounds(sdfFont *font, string text, u32 char_i, v2 pos, f32 line_height) {
   f32 scale = line_height / font->native_line_height;
   
   for(u32 i = 0; i < text.length; i++) {
      u32 id = (u32) text.text[i];
      
      if(id < ArraySize(font->glyphs)) {
         glyphInfo glyph = font->glyphs[id];
         f32 width = scale * glyph.size.x;
         f32 height = scale * glyph.size.y; 

         if(i == char_i) {
            return RectMinSize((scale * glyph.offset) + pos, V2(width, height));
         }

         pos = pos + V2(scale * glyph.xadvance, 0);
      }
   }

   return RectMinSize(V2(0, 0), V2(0, 0));
}

//TODO: kerning
void Text(element *e, string text, v2 pos, f32 line_height) {   
   UIContext *context = e->context;
   sdfFont *font = context->font;
   f32 scale = line_height / font->native_line_height;
   
   for(u32 i = 0; i < text.length; i++) {
      u32 id = (u32) text.text[i];
      
      if(id < ArraySize(font->glyphs)) {
         glyphInfo glyph = font->glyphs[id];
         f32 width = scale * glyph.size.x;
         f32 height = scale * glyph.size.y; 

         RenderCommand *command = PushStruct(&context->frame_arena, RenderCommand);
         command->type = RenderCommand_SDF;
         command->next = NULL;
         command->drawSDF.bounds = RectMinSize((scale * glyph.offset) + pos, 
                                               V2(width, height));
         command->drawSDF.uvBounds = RectMinSize(glyph.textureLocation, glyph.size);
         command->drawSDF.sdf = font->sdfTexture;
         command->drawSDF.colour = V4(0, 0, 0, 1);
         
         addCommand(e, command);
         pos = pos + V2(scale * glyph.xadvance, 0);
      }
   }
}

void Text(element *e, char *s, v2 pos, f32 height) {
   Text(e, Literal(s), pos, height);
}

#define RED V4(1, 0, 0, 1)
#define BLUE V4(0, 0, 1, 1)
#define GREEN V4(0, 1, 0, 1)
#define WHITE V4(1, 1, 1, 1)
#define BLACK V4(0, 0, 0, 1)
//---------------------------------------------------------------------

typedef void (*layout_setup_callback)(element *e);

#define Panel(...) _Panel(GEN_UI_ID, __VA_ARGS__)
element *_Panel(ui_id id, element *parent, layout_setup_callback layout_setup, rect2 bounds) {
   element *result = addElement(parent, id);
   result->bounds = bounds;
   result->cliprect = Overlap(parent->cliprect, bounds);
   if(layout_setup != NULL) {
         layout_setup(result);
   }
   
   return result;
}

element *_Panel(ui_id id, element *parent, layout_setup_callback layout_setup, 
                v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0)) {
   Assert(parent->layout_func != NULL);
   return _Panel(id, parent, layout_setup, parent->layout_func(parent, parent->layout_data, size, padding, margin));
}

//Common layout types-----------------------------------
rect2 columnLayout(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   v2 *at = (v2 *) layout_data;
   v2 pos = *at;
   at->y += element_size.y + padding_size.y + margin_size.y;
   return RectMinSize(e->bounds.min + pos + padding_size, element_size);
}

void ColumnLayout(element *e) {
   UIContext *context = e->context;
   e->layout_func = columnLayout;
   e->layout_data = (u8 *) PushStruct(&context->frame_arena, v2);
}

rect2 rowLayout(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   v2 *at = (v2 *) layout_data;
   v2 pos = *at;
   at->x += element_size.x + padding_size.x + margin_size.x;
   return RectMinSize(e->bounds.min + pos + padding_size, element_size);
}

void RowLayout(element *e) {
   UIContext *context = e->context;
   e->layout_func = rowLayout;
   e->layout_data = (u8 *) PushStruct(&context->frame_arena, v2);
}

rect2 stackLayout(element *e, u8 *layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   return RectMinSize(e->bounds.min + padding_size + margin_size, element_size);
}

void StackLayout(element *e) {
   e->layout_func = stackLayout;
}
//--------------------------------------------------------

#define Label(...) _Label(GEN_UI_ID, __VA_ARGS__)
element *_Label(ui_id id, element *parent, string text, f32 line_height, v2 padding = V2(0, 0),
                v2 margin = V2(0, 0)) 
{
   UIContext *context = parent->context;
   f32 width = TextWidth(context->font, text, line_height);

   element *result = _Panel(id, parent, NULL, V2(width, line_height), padding, margin);
   Text(result, text, result->bounds.min, line_height);
   return result;
}

element *_Label(ui_id id, element *parent, char *text, f32 line_height, v2 padding = V2(0, 0),
                v2 margin = V2(0, 0)) 
{
   return _Label(id, parent, Literal(text), line_height, padding, margin);
}

//Interaction Helper Functions----------------------------
bool IsAbove(element *a, element *b) {
   for(element *e = a; e; e = e->parent) {
      e->visited = true;
   }
   
   element *common_parent = NULL;
   for(element *e = b; e; e = e->parent) {
      if(e->visited) {
         common_parent = e;
         break;
      }
   }

   for(element *e = a; e; e = e->parent) {
      e->visited = false;
   }

   Assert(common_parent != NULL);
   if(common_parent == a) {
      //a is the parent of b, b is above a
      return false;
   } else if(common_parent == b) {
      //b is the parent of a, a is above b
      return true;
   } else {
      //see who is higher on the stack

      //TODO: optimize this path
      element *a_root = NULL;
      for(element *e = a; e; e = e->parent) {
         if(e->parent == common_parent) {
            a_root = e;
            break;
         }
      }
      
      element *b_root = NULL;
      for(element *e = b; e; e = e->parent) {
         if(e->parent == common_parent) {
            b_root = e;
            break;
         }
      }
      
      u32 a_stack = 0;
      u32 b_stack = 0;
      u32 stack = 0;
      for(element *child = common_parent->first_child; child; child = child->next) {
         if(child == a_root) {
            a_stack = stack;
         }

         if(child == b_root) {
            b_stack = stack;
         }

         stack++;
      }

      return a_stack > b_stack;
   }
}

void MarkAsHot(element *e) {
   UIContext *context = e->context;
   if((context->marked_hot == NULL) || IsAbove(e, context->marked_hot)) {
      context->marked_hot = e;
   }
}

bool IsHot(element *e) {
   UIContext *context = e->context;
   return (e->id == context->hot.id);
}

void ClearHotElement(UIContext *context) {
   context->hot.id = NULL_UI_ID;
}

bool IsActive(element *e) {
   UIContext *context = e->context;
   return (e->id == context->active.id);
} 

bool NoActiveElement(UIContext *context) {
   return context->active.id == NULL_UI_ID;
}

void SetActive(element *e) {
   UIContext *context = e->context;
   context->active = Interaction(e->id, e->context);
}

void ClearActiveElement(UIContext *context) {
   context->active.id = NULL_UI_ID;
}

bool IsSelected(element *e) {
   UIContext *context = e->context;
   return (e->id == context->selected.id);
} 

void SetSelected(element *e) {
   UIContext *context = e->context;
   context->selected = Interaction(e->id, e->context);
}
//-------------------------------------------------------------

v2 Cursor(element *e) {
   UIContext *context = e->context;
   return context->input_state.pos;
}

bool ContainsCursor(element *e) {
   UIContext *context = e->context;
   return Contains(e->cliprect, context->input_state.pos);
}

//Common interactions------------------------------------------
struct ui_click {
   bool became_hot;
   bool became_active;
   bool clicked;
};

ui_click ClickInteraction(element *e, bool trigger_cond, 
                          bool active_cond, bool hot_cond) {
   ui_click result = {};
   UIContext *context = e->context;

   bool can_become_hot = NoActiveElement(context) || IsActive(e);
   if(hot_cond && can_become_hot) {
      MarkAsHot(e);
   }
   result.became_hot = IsHot(e) && context->hot_element_changed;
   
   if(IsHot(e) && active_cond && !IsActive(e)) {
      result.became_active = true;
      SetActive(e);
   } 
   
   if(IsActive(e)) {
      if(IsHot(e) && trigger_cond) {
         result.clicked = true;
         SetSelected(e);
      }

      if(!active_cond)
         ClearActiveElement(context);
   }

   return result;
}

ui_click DefaultClickInteraction(element *e) {
   UIContext *context = e->context;
   return ClickInteraction(e, context->input_state.left_up,
                           context->input_state.left_down, 
                           ContainsCursor(e));
}

struct ui_drag {
   bool became_hot;
   bool became_active;
   v2 drag;
};

ui_drag DragInteraction(element *e, bool active_cond, bool hot_cond) {
   ui_drag result = {};
   UIContext *context = e->context;
   v2 cursor = context->input_state.pos;

   bool can_become_hot = NoActiveElement(context) || IsActive(e);
   if(hot_cond && can_become_hot) {
      MarkAsHot(e);
   }
   result.became_hot = IsHot(e) && context->hot_element_changed;
   
   if(IsHot(e) && active_cond && !IsActive(e)) {
      result.became_active = true;
      SetActive(e);
   } 
   
   if(IsActive(e)) {
      result.drag = cursor - context->active.last_pos;
      
      if(!active_cond)
         ClearActiveElement(context);
   }

   return result;
}

ui_drag DefaultDragInteraction(element *e) {
   UIContext *context = e->context;
   return DragInteraction(e, context->input_state.left_down, 
                          ContainsCursor(e));
}

void HoverTooltip(element *e, string tooltip, f32 time = 0.5) {
   UIContext *context = e->context;
   if(IsHot(e) && ((context->curr_time - context->hot.start_time) >= time)) {
      context->tooltip = PushCopy(&context->frame_arena, tooltip);
   }
}

void HoverTooltip(element *e, char *tooltip, f32 time = 0.5) {
   HoverTooltip(e, Literal(tooltip), time);
}
//---------------------------------------------------------------------

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
      result = PushSize(&context->persistent_arena, size);
      OutputDebugStringA("[ui_core] Allocating new persistant data\n");

      persistent_hash_link *new_link = PushStruct(&context->persistent_arena, persistent_hash_link);
      new_link->id = id;
      new_link->data = result;
      new_link->next = context->persistent_hash[hash];
      context->persistent_hash[hash] = new_link;
   }
   
   return result;
}

//Basic animations-------------------------------------------------
struct ui_slide_animation {
   bool init;
   bool open;
   f32 min;
   f32 max;
   f32 value;
};

#define SlideAnimation(...) _SlideAnimation(GEN_UI_ID, __VA_ARGS__)
ui_slide_animation *_SlideAnimation(ui_id id, UIContext *context, f32 min, f32 max, f32 time) {
   ui_slide_animation *result = (ui_slide_animation *) _GetOrAllocate(id, context, sizeof(ui_slide_animation));
   
   if(!result->init) {
      result->init = true;
      result->min = min;
      result->value = min;
      result->max = max;
   } else {
      //TODO: this
      if(min != result->min){

      }

      if(max != result->max) {

      }
   }
   
   //TODO: time stuff, proper animations
   result->value = Clamp(min, max, result->value + (result->open ? 5 : -5));

   return result;
}
//---------------------------------------------------------------------