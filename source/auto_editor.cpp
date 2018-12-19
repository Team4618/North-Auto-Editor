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
};

enum EditorSelectedType {
   NothingSelected,
   NodeSelected,
   PathSelected
};

enum PathEditMode {
   EditControlPoints,
   AddControlPoint,
   RemoveControlPoint
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
   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "New", menu_button).clicked) {
      state->view = EditorView_NewFile;
   }

   if(Button(state->top_bar, "Open", menu_button).clicked) {
      state->view = EditorView_OpenFile;
   }
}

void DrawOpenFileView(element *page, EditorState *state) {
   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
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
   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
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
void DrawNode(ui_field_topdown *field, EditorState *state, AutoNode *node, bool can_drag = true) {
   UI_SCOPE(field->e->context, node);
   element *e = Panel(field->e, RectCenterSize(GetPoint(field, node->pos), V2(10, 10)),
                      Captures(INTERACTION_DRAG | INTERACTION_SELECT));
   DrawNodeGraphic(e, state, node);
   
   if(WasClicked(e)) {
      state->selected_type = NodeSelected;
      state->selected_node = node;
   }
   
   if(can_drag) {
      node->pos = ClampTo(node->pos + PixelsToFeet(field, GetDrag(e)),
                        RectCenterSize(V2(0, 0), field->size_in_ft));
   }

   for(u32 i = 0; i < node->path_count; i++) {
      DrawPath(field, state, node->out_paths[i]);
   }
}

v2 CubicHermiteSpline(AutonomousProgram_ControlPoint a, AutonomousProgram_ControlPoint b, f32 t) {
   return CubicHermiteSpline(a.pos, a.tangent, b.pos, b.tangent, t);
}

v2 CubicHermiteSplineTangent(AutonomousProgram_ControlPoint a, AutonomousProgram_ControlPoint b, f32 t) {
   return CubicHermiteSplineTangent(a.pos, a.tangent, b.pos, b.tangent, t);
}

//TODO: debug this, its pretty janky with curved lines right now
f32 MinDistFrom(ui_field_topdown *field, AutonomousProgram_ControlPoint *control_points, u32 control_point_count) {
   v2 p = Cursor(field->e);
   f32 result = F32_MAX;

   for(u32 i = 1; i < control_point_count; i++) {
      AutonomousProgram_ControlPoint cpa = control_points[i - 1];
      AutonomousProgram_ControlPoint cpb = control_points[i];
      
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

string GetName(AutoCommand *command) {
   return Concat(command->subsystem_name, Literal(":"), command->command_name);
}

void DrawPath(ui_field_topdown *field, EditorState *state, AutoPath *path) {
   UI_SCOPE(field->e->context, path);
   
   if(path->hidden)
      return;

   u32 control_point_count = path->control_point_count + 2;
   AutonomousProgram_ControlPoint *control_points = PushTempArray(AutonomousProgram_ControlPoint, control_point_count);
   control_points[0] = { path->in_node->pos, path->in_tangent };
   control_points[control_point_count - 1] = { path->out_node->pos, path->out_tangent };
   Copy(path->control_points, path->control_point_count * sizeof(AutonomousProgram_ControlPoint), control_points + 1);

   bool hot = IsHot(field->e) && (MinDistFrom(field, control_points, control_point_count) < 2);

   for(u32 i = 1; i < control_point_count; i++) {
      AutonomousProgram_ControlPoint a = control_points[i - 1];
      AutonomousProgram_ControlPoint b = control_points[i];
      CubicHermiteSpline(field, a.pos, a.tangent, b.pos, b.tangent, hot ? RED : GREEN);
   }

   if((state->selected_type == PathSelected) && (state->selected_path == path)) {
      bool point_clicked = false;
      u32 point_index = 0;
      
      for(u32 i = 0; i < path->control_point_count; i++) {
         AutonomousProgram_ControlPoint *point = path->control_points + i;
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
         AutonomousProgram_ControlPoint *new_control_points =
            PushArray(&state->project_arena, AutonomousProgram_ControlPoint, path->control_point_count - 1);

         u32 before_count = point_index;
         Copy(path->control_points, before_count * sizeof(AutonomousProgram_ControlPoint), new_control_points);
         Copy(path->control_points + (point_index + 1), 
               (path->control_point_count - point_index - 1) * sizeof(AutonomousProgram_ControlPoint), new_control_points + before_count);
               
         path->control_points = new_control_points;
         path->control_point_count--;
      }
   }

   if(field->clicked && hot) {
      state->selected_type = PathSelected;
      state->selected_path = path;
      state->path_got_selected = true;
   }

   DrawNode(field, state, path->out_node);
}

#define DrawTangentHandle(...) _DrawTangentHandle(GEN_UI_ID, __VA_ARGS__) 
void _DrawTangentHandle(ui_id id, ui_field_topdown *field, v2 pos, v2 *tangent, bool along_normal, v2 normal) {
   v2 handle_pos = GetPoint(field, pos + *tangent);
   element *handle = _Panel(id, field->e, RectCenterSize(handle_pos, V2(10, 10)), Captures(INTERACTION_DRAG));
   Line(field->e, BLUE, 2, GetPoint(field, pos), handle_pos);
   Background(handle, BLUE);

   v2 drag_vector = along_normal ? (normal * Dot(normal, GetDrag(handle))) : GetDrag(handle);      
   *tangent = ClampTo(*tangent + PixelsToFeet(field, drag_vector),
                      (-pos) + RectCenterSize(V2(0, 0), field->size_in_ft));
}

void DrawEditingView(element *page, EditorState *state) {
   Assert(state->project != NULL);

   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
   
   ui_textbox project_name_box = TextBox(state->top_bar, &state->project_name_box, Size(state->top_bar).y);
   if(IsSelected(project_name_box.e))
      Outline(project_name_box.e, (GetText(project_name_box).length > 0) ? GREEN : RED);
   
   state->project->name = GetText(project_name_box);

   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "Save", menu_button.IsEnabled(GetText(project_name_box).length > 0)).clicked) {
      WriteProject(state->project);
   }

   static float field_width = 700;
   field_width = Clamp(0, Size(page->bounds).x, field_width);

   RobotProfile *profile = &state->profiles.current;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size, field_width);
   state->path_got_selected = false;

   element *resize_divider = Panel(page, V2(Size(page).x - 10, 10), Padding(5, 5).Captures(INTERACTION_DRAG));
   Background(resize_divider, dark_grey);
   field_width += GetDrag(resize_divider).y * (field.size_in_ft.x / field.size_in_ft.y);

   DrawNode(&field, state, state->project->starting_node, false);
   
   InputState *input = &page->context->input_state;
   bool field_clicked = field.clicked && !state->path_got_selected;

   switch(state->selected_type) {
      case NodeSelected: {
         AutoNode *selected_node = state->selected_node;
         
         if(field_clicked) {
            AutoPath *new_path = PushStruct(&state->project_arena, AutoPath);
            AutoNode *new_node = PushStruct(&state->project_arena, AutoNode);
            
            new_node->pos = PixelsToFeet(&field, Cursor(field.e) - field.bounds.min) - 0.5 * field.size_in_ft;
            new_node->in_path = new_path;
            new_path->in_node = state->selected_node;
            
            if(new_path->in_node == state->project->starting_node) {
               v2 direction_arrow = V2(cosf(state->project->starting_angle * (PI32 / 180)), 
                                       -sinf(state->project->starting_angle * (PI32 / 180)));
      
               new_path->in_tangent = direction_arrow;
            } else {
               new_path->in_tangent = Normalize(new_path->out_node->pos - new_path->in_node->pos);
            }
            new_path->out_node = new_node;
            new_path->out_tangent = Normalize(new_path->out_node->pos - new_path->in_node->pos);
            
            AutoPath **new_out_paths = PushArray(&state->project_arena, AutoPath *, selected_node->path_count + 1);
            Copy(selected_node->out_paths, selected_node->path_count * sizeof(AutoPath *), new_out_paths);
            new_out_paths[selected_node->path_count] = new_path;

            selected_node->path_count = selected_node->path_count + 1;
            selected_node->out_paths = new_out_paths;
         }

         //---------------annoying-layout-stuff-----------------
         f32 path_list_height = 5 + (40 + 5) * selected_node->path_count;
         f32 available_command_list_height = 0;
         f32 command_list_height = 5;
         
         ForEachArray(i, subsystem, profile->subsystem_count, profile->subsystems, {
            ForEachArray(j, command, subsystem->command_count, subsystem->commands, {
               available_command_list_height += 40;
            });
         });

         ForEachArray(i, command, selected_node->command_count, selected_node->commands, {
            RobotProfileCommand *command_template = GetCommand(profile, command->subsystem_name, command->command_name);
            Assert(command_template);
            command_list_height += 100 + 20 * command_template->param_count + 5;    
         });

         f32 edit_panel_height = page_tab_height + path_list_height + Max(available_command_list_height, command_list_height);
         //--------------------------------------------------

         element *edit_panel = ColumnPanel(page, V2(Size(page).x - 10, edit_panel_height), Padding(5, 5));
         Background(edit_panel, dark_grey);
         element *edit_buttons = RowPanel(edit_panel, V2(Size(edit_panel).x, page_tab_height));
         element *path_list = ColumnPanel(edit_panel, V2(Size(edit_panel).x, path_list_height)); 
         element *command_lists = RowPanel(edit_panel, V2(Size(edit_panel).x, Max(available_command_list_height, command_list_height))); 
         element *available_command_list = ColumnPanel(command_lists, V2(Size(command_lists).x / 2, available_command_list_height));
         element *command_list = ColumnPanel(command_lists, V2(Size(command_lists).x / 2, command_list_height));

         if(selected_node != state->project->starting_node) {
            if(Button(edit_buttons, "Delete", menu_button).clicked) {
               AutoPath *in_path = selected_node->in_path;
               AutoNode *parent = in_path->in_node;

               u32 remove_index = -1;
               for(u32 i = 0; i < parent->path_count; i++) {
                  if(parent->out_paths[i] == in_path) {
                     remove_index = i;
                     break;
                  }
               }

               Assert(remove_index != -1);
               AutoPath **new_out_paths = PushArray(&state->project_arena, AutoPath *, parent->path_count - 1);

               u32 before_count = remove_index;
               Copy(parent->out_paths, before_count * sizeof(AutoPath *), new_out_paths);
               Copy(parent->out_paths + (remove_index + 1), (parent->path_count - remove_index - 1) * sizeof(AutoPath *), new_out_paths + before_count);
                     
               parent->out_paths = new_out_paths;
               parent->path_count--;

               state->selected_type = NothingSelected;
               state->selected_node = NULL;
            }
         }

         ForEachArray(i, _curr_path, selected_node->path_count, selected_node->out_paths, {
            AutoPath *curr_path = *_curr_path;
            UI_SCOPE(path_list->context, curr_path);
            element *path_panel = RowPanel(path_list, V2(Size(path_list).x - 10, 40), Padding(5, 5));
            Outline(path_panel, light_grey);

            Label(path_panel, Concat(Literal("Path "), ToString(i)), 20, WHITE);
            if(Button(path_panel, "Hide", menu_button.IsSelected(curr_path->hidden)).clicked) {
               curr_path->hidden = !curr_path->hidden;
            }

            Label(path_panel, "Conditional", 20, WHITE);
            CheckBox(path_panel, &curr_path->has_conditional, V2(15, 15));
            if(curr_path->has_conditional) {
               //TODO: replace this with ui_list_selector
               for(u32 j = 0; j < profile->conditional_count; j++) {
                  if(Button(path_panel, profile->conditionals[j], menu_button.IsSelected(profile->conditionals[j] == curr_path->conditional)).clicked) {
                     curr_path->conditional = PushCopy(&state->project_arena, profile->conditionals[j]);
                  }
               }
            }
         });

         ForEachArray(i, subsystem, profile->subsystem_count, profile->subsystems, {
            ForEachArray(j, command, subsystem->command_count, subsystem->commands, {
               if(Button(available_command_list, Concat(subsystem->name, Literal(":"), command->name), menu_button).clicked) {
                  AutoCommand new_command = {};
                  new_command.subsystem_name = PushCopy(&state->project_arena, subsystem->name);
                  new_command.command_name = PushCopy(&state->project_arena, command->name);
                  new_command.param_count = command->param_count;
                  new_command.params = PushArray(&state->project_arena, f32, command->param_count);

                  selected_node->commands =
                     ArrayInsert(&state->project_arena, AutoCommand, selected_node->commands,
                                 0, &new_command, selected_node->command_count++); 
               }
            });
         });

         bool remove_command = false;
         u32 remove_index = 0;

         bool move_command = false;
         u32 from_index = 0;
         u32 to_index = 0;

         ForEachArray(i, command, selected_node->command_count, selected_node->commands, {
            RobotProfileCommand *command_template = GetCommand(profile, command->subsystem_name, command->command_name);
            Assert(command_template);
            
            UI_SCOPE(command_list, command);
            element *command_panel = ColumnPanel(command_list, V2(Size(command_list).x, (100 + 20 * command_template->param_count)), Padding(0, 5).Captures(INTERACTION_DRAG));
            if(IsActive(command_panel)) {
               Outline(command_panel, WHITE);
            } else if(IsHot(command_panel)) {
               Outline(command_panel, V4(120/255.0, 120/255.0, 120/255.0, 1));
            } else {
               Outline(command_panel, light_grey);
            }
            
            element *button_row = RowPanel(command_panel, V2(Size(command_panel).x, page_tab_height)); 
            if(Button(button_row, "Delete", menu_button).clicked) {
               remove_command = true;
               remove_index = i;
            }
            
            //TODO: replace these buttons with the ability to drag to reorder the commands
            if((selected_node->command_count - 1) > i) {
               if(Button(button_row, "Down", menu_button).clicked) {
                  move_command = true;
                  from_index = i;
                  to_index = i + 1;
               }
            }

            if(i > 0) {
               if(Button(button_row, "Up", menu_button).clicked) {
                  move_command = true;
                  from_index = i;
                  to_index = i - 1;
               }
            }

            element *conditional_row = RowPanel(command_panel, V2(Size(command_panel).x, 40));
            Label(conditional_row, "Conditional", 20, WHITE);
            CheckBox(conditional_row, &command->has_conditional, V2(15, 15));
            if(command->has_conditional) {
               //TODO: replace this with ui_list_selector
               for(u32 j = 0; j < profile->conditional_count; j++) {
                  if(Button(conditional_row, profile->conditionals[j], menu_button.IsSelected(profile->conditionals[j] == command->conditional)).clicked) {
                     command->conditional = PushCopy(&state->project_arena, profile->conditionals[j]);
                  }
               }
            }

            Label(command_panel, GetName(command), 20, WHITE);
            for(u32 j = 0; j < command_template->param_count; j++) {
               f32 *param_value = command->params + j;
               UI_SCOPE(command_panel->context, param_value);
               
               element *param_row = RowPanel(command_panel, V2(Size(command_panel).x, 20)); 
               Label(param_row, command_template->params[j], 20, WHITE);
               TextBox(param_row, param_value, 20);
            }    
         });

         if(remove_command) {
            selected_node->commands =
                     ArrayRemove(&state->project_arena, AutoCommand, selected_node->commands,
                                 remove_index, selected_node->command_count--); 
         } else if(move_command) {
            ArrayMove(AutoCommand, selected_node->commands, selected_node->command_count,
                      from_index, to_index);
         }
      } break;

      case PathSelected: {
         AutoPath *selected_path = state->selected_path;

         element *edit_panel = ColumnPanel(page, V2(Size(page).x - 10, 500), Padding(5, 5));
         Background(edit_panel, dark_grey);
         element *edit_buttons = RowPanel(edit_panel, V2(Size(edit_panel).x, page_tab_height));
         if(Button(edit_buttons, "(E)dit Path", menu_button.IsSelected(state->path_edit == EditControlPoints)).clicked ||
            (input->key_char == 'e'))
         {
            state->path_edit = EditControlPoints;
         }
         
         if(Button(edit_buttons, "(A)dd Control Point", menu_button.IsSelected(state->path_edit == AddControlPoint)).clicked ||
            (input->key_char == 'a'))
         {
            state->path_edit = AddControlPoint;
         }

         if(Button(edit_buttons, "(R)emove Control Point", menu_button.IsSelected(state->path_edit == RemoveControlPoint)).clicked ||
            (input->key_char == 'r'))
         {
            state->path_edit = RemoveControlPoint;
         }

         switch(state->path_edit) {
            case EditControlPoints: {
               if(selected_path->in_node == state->project->starting_node) {
                  v2 direction_arrow = V2(cosf(state->project->starting_angle * (PI32 / 180)), 
                                          -sinf(state->project->starting_angle * (PI32 / 180)));
                  
                  DrawTangentHandle(&field, selected_path->in_node->pos, &selected_path->in_tangent,
                                    true, direction_arrow);
                  
               } else {
                  DrawTangentHandle(&field, selected_path->in_node->pos, &selected_path->in_tangent,
                                    input->ctrl_down, Normalize(selected_path->in_tangent));
               }
               
               for(u32 i = 0; i < selected_path->control_point_count; i++) {
                  AutonomousProgram_ControlPoint *control_point = selected_path->control_points + i;
                  UI_SCOPE(field.e->context, control_point);
                  DrawTangentHandle(&field, control_point->pos, &control_point->tangent,
                                    input->ctrl_down, Normalize(control_point->tangent));
               }

               DrawTangentHandle(&field, selected_path->out_node->pos, &selected_path->out_tangent,
                                 input->ctrl_down, Normalize(selected_path->out_tangent));
            } break;

            case AddControlPoint: {
               u32 control_point_count = selected_path->control_point_count + 2;
               AutonomousProgram_ControlPoint *control_points = PushTempArray(AutonomousProgram_ControlPoint, control_point_count);
               control_points[0] = { selected_path->in_node->pos, selected_path->in_tangent };
               control_points[control_point_count - 1] = { selected_path->out_node->pos, selected_path->out_tangent };
               Copy(selected_path->control_points, selected_path->control_point_count * sizeof(AutonomousProgram_ControlPoint), control_points + 1);

               bool add_point = false;
               AutonomousProgram_ControlPoint new_point = {};
               u32 insert_index = 0;

               for(u32 i = 1; i < control_point_count; i++) {
                  UI_SCOPE(field.e, i);
                  v2 new_pos = CubicHermiteSpline(control_points[i - 1], control_points[i], 0.5);
                  element *new_button = Panel(field.e, RectCenterSize(GetPoint(&field, new_pos), V2(10, 10)), Captures(INTERACTION_CLICK));
                  Background(new_button, BLUE);
                  if(IsHot(new_button)) {
                     Outline(new_button, BLACK);
                  }

                  if(WasClicked(new_button)) {
                     insert_index = i - 1;
                     new_point.pos = new_pos;
                     new_point.tangent = CubicHermiteSplineTangent(control_points[i - 1], control_points[i], 0.5);
                     add_point = true;
                  }
               }

               if(add_point) {
                  selected_path->control_points = 
                     ArrayInsert(&state->project_arena, AutonomousProgram_ControlPoint, selected_path->control_points,
                                 insert_index, &new_point, selected_path->control_point_count);
                  selected_path->control_point_count++;
               }
            } break;
         }
      } break;
   }

   if(input->key_esc) {
      state->selected_type = NothingSelected;
   }

   if(Button(state->top_bar, "Exit", menu_button).clicked) {
      Reset(&state->project_arena);
      state->project = NULL;
      state->view = EditorView_Blank;
   }
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

   element *status_bar = RowPanel(root, V2(Size(root).x, status_bar_height));
   Background(status_bar, dark_grey);
   if(state->profiles.current.state == RobotProfileState::Connected) {
      Label(status_bar, Concat(state->profiles.current.name, Literal(" Connected")), 20, WHITE, V2(10, 0));
   } else if(state->profiles.current.state == RobotProfileState::Loaded) {
      Label(status_bar, Concat(state->profiles.current.name, Literal(" Loaded")), 20, WHITE, V2(10, 0));
   } else {
      Label(status_bar, "No Robot", 20, WHITE, V2(5, 0));
   }
   
   state->top_bar = RowPanel(root, V2(Size(root).x, page_tab_height));
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