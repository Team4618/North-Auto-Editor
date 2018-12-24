#include "north/north_common_definitions.h"
#include "north/north_file_definitions.h"
#include "north/north_network_definitions.h"

#include "theme.cpp"

#include "auto_project_utils.cpp"
#define INCLUDE_DRAWSETTINGS
#include "north_settings_utils.cpp"
#define INCLUDE_DRAWPROFILES
#include "robot_profile_utils.cpp"

enum EditorPage {
   EditorPage_Home,
   EditorPage_Robots,
   EditorPage_Settings
};

enum EditorView {
   EditorView_Blank,
   EditorView_OpenFile,
   EditorView_NewFile,
   EditorView_Editing,
   EditorView_Simulating,
};

enum EditorSelectedType {
   NothingSelected,
   NodeSelected,
   PathSelected
};

enum PathEditMode {
   EditControlPoints,
   AddControlPoint,
   RemoveControlPoint,
};

struct EditorState {
   EditorPage page;
   EditorView view;
   element *top_bar;

   EditorSelectedType selected_type;
   union {
      AutoNode *selected_node;
      AutoPath *selected_path;
   };
   bool path_got_selected;
   PathEditMode path_edit;

   MemoryArena project_arena;
   AutoProjectLink *project;

   TextBoxData project_name_box;
   char _project_name_box[20];

   FileWatcher file_watcher;
   bool directory_changed;

   NorthSettings settings;
   RobotProfiles profiles;

   MemoryArena file_lists_arena;
   FileListLink *ncff_files;
   FileListLink *ncrp_files;
   FileListLink *ncap_files;
};

//--------------------move-to-common---------------------------
#define ArrayInsert(arena, type, original, insert_index, new_elem, count) (type *) _ArrayInsert(arena, (u8 *) (original), insert_index, (u8 *) (new_elem), sizeof(type), count)
u8 *_ArrayInsert(MemoryArena *arena, u8 *original, u32 insert_index, 
                 u8 *new_elem, u32 element_size, u32 count)
{
   u8 *result = PushSize(arena, element_size * (count + 1));

   u32 before_count = insert_index;
   u32 after_count = count - insert_index;
   Copy(original, before_count * element_size, result);
   Copy(original + element_size * before_count, after_count * element_size, result + (before_count + 1) * element_size);
   Copy(new_elem, element_size, result + element_size * insert_index); 
   return result;
}

#define ArrayRemove(arena, type, original, remove_index, count) (type *) _ArrayRemove(arena, (u8 *) (original), remove_index, sizeof(type), count)
u8 *_ArrayRemove(MemoryArena *arena, u8 *original, 
                 u32 remove_index, u32 element_size, u32 count)
{
   u8 *result = PushSize(arena, element_size * (count - 1));

   u32 before_count = remove_index;
   Copy(original, before_count * element_size, result);
   Copy(original + element_size * (remove_index + 1), 
         (count - remove_index - 1) * element_size, result + before_count * element_size);
   return result;
}

#define ArrayMove(type, array, count, from, to) _ArrayMove(sizeof(type), (u8 *) (array), count, from, to)
void _ArrayMove(u32 size, u8 *array, u32 count, u32 from, u32 to) {
   TempArena temp_arena;
   u8 *temp_array = PushSize(&temp_arena.arena, count * size);

   if(to == from)
      return;

   if(to > from) {
      Copy(array, from * size, temp_array);
      Copy(array + (from + 1) * size, (to - from) * size, temp_array + from * size);
      Copy(array + from * size, size, temp_array + to * size);
      Copy(array + (to + 1) * size, (count - to) * size, temp_array + (to + 1) * size);
   } else if(to < from) {
      Copy(array, to * size, temp_array);
      Copy(array + from * size, size, temp_array + to * size);
      Copy(array + to * size, (from - to) * size, temp_array + (to + 1) * size);
      Copy(array + (from + 1) * size, (count - from) * size, temp_array + (from + 1) * size);
   }

   Copy(temp_array, count * size, array);
}
//---------------------------------------------------------------

//--------------------Kinda-specific-utils-----------------------
//TODO: debug this, its pretty janky with curved lines right now
f32 MinDistFrom(ui_field_topdown *field, North_HermiteControlPoint *control_points, u32 control_point_count) {
   v2 p = Cursor(field->e);
   f32 result = F32_MAX;

   for(u32 i = 1; i < control_point_count; i++) {
      North_HermiteControlPoint cpa = control_points[i - 1];
      North_HermiteControlPoint cpb = control_points[i];
      
      u32 point_count = 20;
      f32 step = (f32)1 / (f32)(point_count - 1);
      
      for(u32 i = 1; i < point_count; i++) {
         v2 a = GetPoint(field, CubicHermiteSpline(cpa, cpb, (i - 1) * step));
         v2 b = GetPoint(field, CubicHermiteSpline(cpa, cpb, i * step));
         result = Min(DistFromLine(a, b, p), result);
      }
   }

   return result;
}
//---------------------------------------------------------------

#include "auto_editor_ui.cpp"
#include "auto_simulator_ui.cpp"

void initEditor(EditorState *state) {
   state->page = EditorPage_Home;
   state->settings.arena = PlatformAllocArena(Megabyte(1));
   state->profiles.current.arena = PlatformAllocArena(Megabyte(10));
   state->profiles.loaded.arena = PlatformAllocArena(Megabyte(10));
   state->project_arena = PlatformAllocArena(Megabyte(30));
   state->file_lists_arena = PlatformAllocArena(Megabyte(10));

   InitTextBoxData(&state->project_name_box, state->_project_name_box);
   InitFileWatcher(&state->file_watcher, PlatformAllocArena(Kilobyte(512)), "*.*");
}

void reloadFiles(EditorState *state) {
   Reset(&state->file_lists_arena);
   state->ncff_files = ListFilesWithExtension("*.ncff", &state->file_lists_arena);
   state->ncrp_files = ListFilesWithExtension("*.ncrp", &state->file_lists_arena);
   state->ncap_files = ListFilesWithExtension("*.ncap", &state->file_lists_arena);

   ReadSettingsFile(&state->settings);
}

void DrawBlankView(element *page, EditorState *state) {
   Panel(state->top_bar, Size(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "New", menu_button).clicked) {
      state->view = EditorView_NewFile;
   }

   if(Button(state->top_bar, "Open", menu_button).clicked) {
      state->view = EditorView_OpenFile;
   }
}

void DrawOpenFileView(element *page, EditorState *state) {
   Panel(state->top_bar, Size(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "<- Back", menu_button).clicked) {
      state->view = EditorView_Blank;
   }
   
   if(state->ncap_files != NULL) {
      for(FileListLink *file = state->ncap_files; file; file = file->next) {
         UI_SCOPE(page->context, file);

         //TODO: draw previews, not just buttons
         if(Button(page, file->name, menu_button).clicked) {
            Reset(&state->project_arena);
            state->project = ReadAutoProject(file->name, &state->project_arena);
            SetText(&state->project_name_box, state->project->name);
            state->view = EditorView_Editing;
            state->selected_type = NothingSelected;
         }
      }
   } else {
      Label(page, "No Auto Projects", V2(Size(page).x, 80), 50, BLACK);
   }
}

void DrawNewFileView(element *page, EditorState *state) {
   Panel(state->top_bar, Size(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "<- Back", menu_button).clicked) {
      state->view = EditorView_Blank;
   }

   RobotProfile *profile = &state->profiles.current;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size,
                                         Clamp(0, Size(page->bounds).x, 700));

   v2 robot_size_px =  FeetToPixels(&field, profile->size);
   for(u32 i = 0; i < state->settings.field.starting_position_count; i++) {
      Field_StartingPosition *starting_pos = state->settings.field.starting_positions + i;
      UI_SCOPE(page->context, starting_pos);
      
      element *field_starting_pos = Panel(field.e, RectCenterSize(GetPoint(&field, starting_pos->pos), robot_size_px),
                                          Captures(INTERACTION_CLICK));
      
      v2 direction_arrow = V2(cosf(starting_pos->angle * (PI32 / 180)), 
                              -sinf(starting_pos->angle * (PI32 / 180)));
      Background(field_starting_pos, IsHot(field_starting_pos) ? GREEN : RED);
      Line(field_starting_pos, BLACK, 2,
            Center(field_starting_pos->bounds),
            Center(field_starting_pos->bounds) + 10 * direction_arrow);           

      if(WasClicked(field_starting_pos)) {
         Reset(&state->project_arena);
         state->project = PushStruct(&state->project_arena, AutoProjectLink);
         state->project->starting_angle = starting_pos->angle;

         state->project->starting_node = PushStruct(&state->project_arena, AutoNode);
         state->project->starting_node->pos = starting_pos->pos;
         
         state->view = EditorView_Editing;
      }   
   }
}

void DrawNodeGraphic(element *e, EditorState *state, AutoNode *node) {
   bool selected = (state->selected_type == NodeSelected) && (state->selected_node == node);
   Background(e, selected ? GREEN : RED);
   if(IsHot(e)) {
      Outline(e, BLACK);
   }
}

void DrawPath(ui_field_topdown *field, EditorState *state, AutoPath *path);
void DrawNode(ui_field_topdown *field, EditorState *state, AutoNode *node, bool can_drag) {
   UI_SCOPE(field->e->context, node);
   element *e = Panel(field->e, RectCenterSize(GetPoint(field, node->pos), V2(10, 10)),
                      Captures(INTERACTION_DRAG | INTERACTION_SELECT));
   DrawNodeGraphic(e, state, node);
   
   if(WasClicked(e)) {
      state->selected_type = NodeSelected;
      state->selected_node = node;
   }
   
   if(can_drag && (state->view == EditorView_Editing)) {
      v2 drag_vector = GetDrag(e);
      node->pos = ClampTo(node->pos + PixelsToFeet(field, drag_vector),
                        RectCenterSize(V2(0, 0), field->size_in_ft));

      if(Length(drag_vector) > 0) {
         if(node->in_path != NULL) {
            RecalculateAutoPath(node->in_path);
         }

         for(u32 i = 0; i < node->path_count; i++) {
            RecalculateAutoPath(node->out_paths[i]);
         }
      }
   }

   for(u32 i = 0; i < node->path_count; i++) {
      DrawPath(field, state, node->out_paths[i]);
   }
}

void DrawPath(ui_field_topdown *field, EditorState *state, AutoPath *path) {
   UI_SCOPE(field->e->context, path);
   
   if(path->hidden)
      return;

   AutoPathSpline path_spline = GetAutoPathSpline(path);
   bool hot = IsHot(field->e) && (MinDistFrom(field, path_spline.points, path_spline.point_count) < 2);

   //TODO: make this better, dont just call "Line" 20 times
   {
      u32 point_count = 20;
      f32 step = path->length / (f32)(point_count - 1);

      for(u32 i = 1; i < point_count; i++) {
         v2 point_a = GetAutoPathPoint(path, step * (i - 1));
         v2 point_b  = GetAutoPathPoint(path, step * i);
         f32 t_velocity = GetVelocityAt(path, step * i) / 20;
         Line(field->e, hot ? GREEN : lerp(BLUE, t_velocity, RED), 2, GetPoint(field, point_a), GetPoint(field, point_b));
      }
   }

   for(u32 i = 0; i < path->discrete_event_count; i++) {
      Rectangle(field->e, RectCenterSize(GetPoint(field, GetAutoPathPoint(path, path->discrete_events[i].distance)), V2(5, 5)), BLACK);
   }

   if((state->selected_type == PathSelected) && (state->selected_path == path) && (state->view == EditorView_Editing)) {
      bool point_clicked = false;
      u32 point_index = 0;
      
      for(u32 i = 0; i < path->control_point_count; i++) {
         North_HermiteControlPoint *point = path->control_points + i;
         UI_SCOPE(field->e->context, point);
         element *handle = Panel(field->e, RectCenterSize(GetPoint(field, point->pos), V2(10, 10)), Captures(INTERACTION_DRAG | INTERACTION_CLICK));
         
         Background(handle, RED);
         if(IsHot(handle)) {
            Outline(handle, BLACK);
         }

         if((state->path_edit == RemoveControlPoint) && WasClicked(handle)) {
            point_clicked = true;
            point_index = i;
         }

         point->pos = ClampTo(point->pos + PixelsToFeet(field, GetDrag(handle)),
                              RectCenterSize(V2(0, 0), field->size_in_ft));      
      }

      if(point_clicked) {
         North_HermiteControlPoint *new_control_points =
            PushArray(&state->project_arena, North_HermiteControlPoint, path->control_point_count - 1);

         u32 before_count = point_index;
         Copy(path->control_points, before_count * sizeof(North_HermiteControlPoint), new_control_points);
         Copy(path->control_points + (point_index + 1), 
               (path->control_point_count - point_index - 1) * sizeof(North_HermiteControlPoint), new_control_points + before_count);
               
         path->control_points = new_control_points;
         path->control_point_count--;
         RecalculateAutoPath(path);
      }
   }

   if(field->clicked && hot) {
      state->selected_type = PathSelected;
      state->selected_path = path;
      state->path_got_selected = true;
   }

   DrawNode(field, state, path->out_node);
}

void DrawHome(element *full_page, EditorState *state) {
   StackLayout(full_page);
   element *page = VerticalList(full_page);

   if(state->settings.field.loaded && IsValid(&state->profiles.current)) {
      switch(state->view) {
         case EditorView_Blank: DrawBlankView(page, state); break;
         case EditorView_OpenFile: DrawOpenFileView(page, state); break;
         case EditorView_NewFile: DrawNewFileView(page, state); break;
         case EditorView_Editing: DrawEditingView(page, state); break;
         case EditorView_Simulating: DrawSimulationView(page, state); break;
      }
   } else {
      if(!state->settings.field.loaded)
         Label(page, "No Field", V2(Size(page).x, 80), 50, BLACK);
      
      if(!IsValid(&state->profiles.current))
         Label(page, "No Robot", V2(Size(page).x, 80), 50, BLACK);
   }
}

#define PageButton(...) _PageButton(GEN_UI_ID, __VA_ARGS__)
void _PageButton(ui_id id, char *name, EditorPage page, EditorState *state) {
   button_style style = menu_button.IsSelected(state->page == page);
   if(_Button(id, state->top_bar, name, style).clicked) {
      state->page = page;
   }
}

void DrawUI(element *root, EditorState *state) {
   if(state->directory_changed) {
      reloadFiles(state);
   }

   ColumnLayout(root);
   Background(root, light_grey);
   Texture(root, logoTexture, RectCenterSize(Center(root->bounds), logoTexture.size));

   element *status_bar = RowPanel(root, Size(Size(root).x, status_bar_height));
   Background(status_bar, dark_grey);
   if(state->profiles.current.state == RobotProfileState::Connected) {
      Label(status_bar, Concat(state->profiles.current.name, Literal(" Connected")), 20, WHITE, V2(10, 0));
   } else if(state->profiles.current.state == RobotProfileState::Loaded) {
      Label(status_bar, Concat(state->profiles.current.name, Literal(" Loaded")), 20, WHITE, V2(10, 0));
   } else {
      Label(status_bar, "No Robot", 20, WHITE, V2(5, 0));
   }
   
   state->top_bar = RowPanel(root, Size(Size(root).x, page_tab_height));
   Background(state->top_bar, dark_grey);
   PageButton("Home", EditorPage_Home, state);
   PageButton("Robots", EditorPage_Robots, state);
   PageButton("Settings", EditorPage_Settings, state);

   element *page = ColumnPanel(root, RectMinMax(root->bounds.min + V2(0, status_bar_height + page_tab_height), root->bounds.max));
   switch(state->page) {
      case EditorPage_Home: DrawHome(page, state); break;
      case EditorPage_Robots: DrawProfiles(page, &state->profiles, state->ncrp_files); break;
      case EditorPage_Settings: {
         v2 robot_size_ft = V2(2, 2);
         string robot_size_label = Literal("No robot loaded, defaulting to 2x2ft");
         DrawSettings(page, &state->settings, robot_size_ft, robot_size_label, state->ncff_files);
      } break;
   }
}