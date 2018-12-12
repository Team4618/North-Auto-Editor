struct GraphEntry {
   f32 value;
   f32 time;
};

struct GraphEntryBlock {
   GraphEntryBlock *next;
   u32 i;
   GraphEntry entries[256];
};

struct UnitSettings {
   string suffix;
   f32 max_value;
   f32 min_value;
};

struct LineGraph {
   LineGraph *next_in_hash;
   LineGraph *next;

   string name;
   v4 colour;
   bool hidden;

   GraphEntryBlock first_block;
   GraphEntryBlock *curr_block;

   u32 begin_i;
   GraphEntryBlock *begin_block;
   u32 end_i;
   GraphEntryBlock *end_block;
   
   f32 max_value;
   f32 min_value;

   //NOTE: 0 means unitless, unit_id > 0 needs to be looked up in the units table
   u32 unit_id;  
};

struct MultiLineGraphData {
   MemoryArena arena;
   LineGraph *first;
   LineGraph *line_hash[64];

   f32 min_time;
   f32 max_time;

   f32 abs_min_time;
   f32 abs_max_time;

   UnitSettings units[32];

   bool automatic_max_time;

   bool dragging;
   f32 dragBeginT;
};

void ResetMultiLineGraph(MultiLineGraphData *data) {
   data->first = NULL;
   for(u32 i = 0; i < ArraySize(data->line_hash); i++) {
      data->line_hash[i] = NULL;
   }
   Reset(&data->arena);

   data->automatic_max_time = true;
   data->min_time = F32_MAX;
   data->max_time = -F32_MAX;
   data->abs_min_time = F32_MAX;
   data->abs_max_time = -F32_MAX;
   for(u32 i = 0; i < ArraySize(data->units); i++) {
      data->units[i].min_value = F32_MAX;
      data->units[i].max_value = -F32_MAX;
   }
}

MultiLineGraphData NewMultiLineGraph(MemoryArena arena) {
   MultiLineGraphData result = {};
   result.arena = arena;
   ResetMultiLineGraph(&result);
   return result;
}

v2 GetPointFor(MultiLineGraphData *data, LineGraph *graph, rect2 bounds, GraphEntry entry) {
   f32 t = (entry.time - data->min_time) / (data->max_time - data->min_time);
   Assert(graph->unit_id < ArraySize(data->units));
   f32 min_value = (graph->unit_id == 0) ? graph->min_value : data->units[graph->unit_id].min_value;
   f32 max_value = (graph->unit_id == 0) ? graph->max_value : data->units[graph->unit_id].max_value;
   f32 v = (entry.value - min_value) / (max_value - min_value);
   return bounds.min + V2(Size(bounds).x * t, Size(bounds).y * (1 - v));
}

f32 GetValueAt(LineGraph *graph, f32 t) {
   GraphEntry prev_entry = graph->first_block.entries[0];
   
   //TODO: optimize
   for(GraphEntryBlock *block = &graph->first_block; block; block = block->next) {
      bool is_begin_block = (block == graph->begin_block);
      u32 begin_i = is_begin_block ? (graph->begin_i + 1) : 0;
      
      for(u32 i = begin_i; i < block->i; i++) {
         GraphEntry curr_entry = block->entries[i];
         if((prev_entry.time <= t) && (t <= curr_entry.time)) {
            f32 s = (t - prev_entry.time) / (curr_entry.time - prev_entry.time);
            return lerp(prev_entry.value, s, curr_entry.value);
         }
         prev_entry = curr_entry;
      }
   }

   return 0;
}

void SetRange(MultiLineGraphData *data, f32 min_time, f32 max_time) {
   data->min_time = min_time;
   data->max_time = max_time;

   for(u32 i = 0; i < ArraySize(data->units); i++) {
      data->units[i].min_value = F32_MAX;
      data->units[i].max_value = -F32_MAX;
   }

   for(LineGraph *graph = data->first; graph; graph = graph->next) { 
      graph->max_value = -F32_MAX;
      graph->min_value = F32_MAX;

      f32 max_t = -F32_MAX;
      f32 min_t = F32_MAX;

      //TODO: optimize
      for(GraphEntryBlock *block = &graph->first_block; block; block = block->next) {
         for(u32 i = 0; i < block->i; i++) {
            GraphEntry entry = block->entries[i];
            if((min_time <= entry.time) && (entry.time <= max_time)) {
               if(graph->unit_id == 0) {
                  graph->min_value = Min(graph->min_value, entry.value);
                  graph->max_value = Max(graph->max_value, entry.value);
               } else {
                  UnitSettings *unit = data->units + graph->unit_id;
                  unit->min_value = Min(unit->min_value, entry.value);
                  unit->max_value = Max(unit->max_value, entry.value);
               }

               if(entry.time < min_t) {
                  min_t = entry.time;
                  graph->begin_block = block;
                  graph->begin_i = i;
               }

               if(entry.time > max_t) {
                  max_t = entry.time;
                  graph->end_block = block;
                  graph->end_i = i + 1;
               }
            }      
         }
      }
   }
}

#define MultiLineGraph(...) _MultiLineGraph(GEN_UI_ID, __VA_ARGS__)
void _MultiLineGraph(ui_id id, element *parent, MultiLineGraphData *data, 
                     v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0),
                     bool immutable = false)
{
   element *base = _Panel(id + GEN_UI_ID, parent, size, Padding(padding).Margin(margin).Layout(ColumnLayout));
   element *graph = _Panel(id + GEN_UI_ID, base, V2(size.x, size.y - 40), Layout(ColumnLayout).Captures(INTERACTION_CLICK));
   element *control_row = _Panel(id + GEN_UI_ID, base, V2(size.x, 40), Padding(0, 5).Layout(RowLayout));
   
   Background(graph, V4(0.7, 0.7, 0.7, 1));

   button_style control_button_style = ButtonStyle(V4(0.5, 0.5, 0.5, 1), V4(0.5, 0.5, 0.5, 1), V4(0.5, 0.5, 0.5, 1),
                                                   V4(0.5, 0.5, 0.5, 1), V4(0.5, 0.5, 0.5, 1), V4(0.5, 0.5, 0.5, 1),
                                                   BLACK, BLACK,
                                                   30, V2(0, 0), V2(5, 5));
   
   if(!immutable) {
      if(_Button(id + GEN_UI_ID, control_row, "Clear", control_button_style).clicked) {
         ResetMultiLineGraph(data);
      }

      if(_Button(id + GEN_UI_ID, control_row, "Pause", control_button_style).clicked) {
         data->automatic_max_time = false;
      }

      if(_Button(id + GEN_UI_ID, control_row, "Reset Min Time", control_button_style).clicked) {
         SetRange(data, data->abs_max_time, data->abs_max_time);
         data->automatic_max_time = true;
      }
   }

   if(_Button(id + GEN_UI_ID, control_row, "Full Time", control_button_style).clicked) {
      SetRange(data, data->abs_min_time, data->abs_max_time);
      data->automatic_max_time = true;
   }

   for(LineGraph *curr_graph = data->first; curr_graph; curr_graph = curr_graph->next) {
      ui_button btn = _Button(POINTER_UI_ID(curr_graph), control_row, curr_graph->name, control_button_style);

      if(btn.clicked) {
         curr_graph->hidden = !curr_graph->hidden;
      }

      bool highlight = IsHot(btn.e);
      if(!curr_graph->hidden) {
         GraphEntry prev_entry = curr_graph->begin_block->entries[curr_graph->begin_i];
         
         //TODO: optimize
         //maybe we can upload the data points to the GPU & do all the lerping in shader
         for(GraphEntryBlock *block = curr_graph->begin_block; ; block = block->next) {
            bool is_begin_block = (block == curr_graph->begin_block);
            bool is_end_block = (block == curr_graph->end_block);
            u32 begin_i = is_begin_block ? (curr_graph->begin_i + 1) : 0;
            u32 end_i = is_end_block ? curr_graph->end_i : block->i;
            
            for(u32 i = begin_i; i < end_i; i++) {
               GraphEntry curr_entry = block->entries[i];
               v2 a = GetPointFor(data, curr_graph, graph->bounds, prev_entry);
               v2 b = GetPointFor(data, curr_graph, graph->bounds, curr_entry);
               Line(graph, curr_graph->colour, highlight ? 4 : 2, a, b);
               Rectangle(graph, RectCenterSize(b, V2(3, 3)), BLACK);
               
               prev_entry = curr_entry;
            }

            if(is_end_block)
               break;
         }

         if(highlight) {
            rect2 bounds = graph->bounds;
            f32 min_value = (curr_graph->unit_id == 0) ? curr_graph->min_value : data->units[curr_graph->unit_id].min_value;
            f32 max_value = (curr_graph->unit_id == 0) ? curr_graph->max_value : data->units[curr_graph->unit_id].max_value;
            f32 y = Size(bounds).y * (1 - ((0 - min_value) / (max_value - min_value)));
            Line(graph, BLACK, 2, V2(bounds.min.x, bounds.min.y + y), V2(bounds.max.x, bounds.min.y + y));
         }
      }
   }

   rect2 b = graph->bounds;
   f32 cursor_x = Cursor(graph).x;
   cursor_x = Min(cursor_x, b.max.x);
   cursor_x = Max(cursor_x, b.min.x);

   f32 cursor_t = lerp(data->min_time, (cursor_x - b.min.x) / Size(b).x, data->max_time);

   if(BecameActive(graph)) {
      data->dragging = true;
      data->dragBeginT = cursor_t; 
   }

   if(!IsActive(graph)) {
      if(data->dragging) {
         SetRange(data, Min(data->dragBeginT, cursor_t), Max(data->dragBeginT, cursor_t));
         data->automatic_max_time = false;
      }

      data->dragging = false;
   }

   if(data->dragging) {
      f32 min_s = (data->dragBeginT - data->min_time) / (data->max_time - data->min_time);
      f32 max_s = (cursor_t - data->min_time) / (data->max_time - data->min_time);
      Rectangle(graph, RectMinMax(V2(lerp(b.min.x, min_s, b.max.x), b.min.y), 
                                  V2(lerp(b.min.x, max_s, b.max.x), b.max.y)), V4(0, 0, 0, 0.2));
   }

   if(IsHot(graph)) {
      Label(graph, Concat(Literal("time = "), ToString(cursor_t)), 20, BLACK, V2(5, 5));
      for(LineGraph *curr_graph = data->first; curr_graph; curr_graph = curr_graph->next) {
         string prefix = Literal(curr_graph->hidden ? "- " : "+ ");
         f32 value = GetValueAt(curr_graph, cursor_t);
         Assert(curr_graph->unit_id < ArraySize(data->units));
         string suffix = (curr_graph->unit_id == 0) ? Literal("") : data->units[curr_graph->unit_id].suffix;
         string text = Concat(prefix, curr_graph->name, Literal(" = "), ToString(value), suffix); 
         Label(graph, text, 20, BLACK);
      }
   }
}

#define ImmutableMultiLineGraph(...) _ImmutableMultiLineGraph(GEN_UI_ID, __VA_ARGS__)
void _ImmutableMultiLineGraph(ui_id id, element *parent, MultiLineGraphData *data, 
                     v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   _MultiLineGraph(id, parent, data, size, padding, margin, true);  
}

//NOTE: the "suffix" string must exist for the entire lifetime of the graph
void SetUnit(MultiLineGraphData *data, u32 unit_id, string suffix) {
   Assert(unit_id < ArraySize(data->units));
   data->units[unit_id].suffix = suffix;
}

//NOTE: the "name" string must exist for the entire lifetime of the graph
void AddEntry(MultiLineGraphData *data, string name, f32 value, f32 time, u32 unit_id) {
   u32 i = Hash(name) % ArraySize(data->line_hash);
   LineGraph *graph = NULL;
   for(LineGraph *g = data->line_hash[i]; g; g = g->next_in_hash) {
      if(g->name == name) {
         graph = g;
         break;
      }
   }

   bool first_entry = (data->first == NULL);

   if(graph == NULL) {
      graph = PushStruct(&data->arena, LineGraph);
      graph->next = data->first;
      data->first = graph;
      graph->next_in_hash = data->line_hash[i];
      data->line_hash[i] = graph;

      graph->name = name;
      graph->hidden = false;
      graph->colour = V4(Random01(), Random01(), Random01(), 1);
      graph->unit_id = unit_id;

      graph->curr_block = &graph->first_block;
      graph->begin_block = &graph->first_block;
      graph->end_block = &graph->first_block;

      graph->max_value = -F32_MAX;
      graph->min_value = F32_MAX;
   }

   Assert(graph->unit_id < ArraySize(data->units));
   Assert(graph->unit_id == unit_id);
   
   GraphEntryBlock *curr_block = graph->curr_block;
   if(curr_block->i == ArraySize(graph->curr_block->entries)) {
      GraphEntryBlock *new_block = PushStruct(&data->arena, GraphEntryBlock);
      curr_block->next = new_block;
      graph->curr_block = new_block;
      curr_block = new_block;
   }

   bool overwrite_previous_entry = false;
   if(curr_block->i >= 2) {
      f32 a = curr_block->entries[curr_block->i - 2].value;
      f32 b = curr_block->entries[curr_block->i - 1].value;
      overwrite_previous_entry = (value == a) && (value == b);
   }

   GraphEntry *entry = curr_block->entries + (overwrite_previous_entry ? (curr_block->i - 1) : curr_block->i++);
   entry->value = value;
   entry->time = time;

   data->abs_min_time = Min(data->abs_min_time, time);;
   data->abs_max_time = Max(data->abs_max_time, time);

   if(data->automatic_max_time) {
      graph->end_block = curr_block;
      graph->end_i = curr_block->i;

      if(first_entry)
         data->min_time = time;

      data->max_time = Max(data->max_time, time);
      
      if(graph->unit_id == 0) {
         graph->min_value = Min(graph->min_value, value);
         graph->max_value = Max(graph->max_value, value);
      } else {
         UnitSettings *unit = data->units + graph->unit_id;
         unit->min_value = Min(unit->min_value, value);
         unit->max_value = Max(unit->max_value, value);
      }
   }
}