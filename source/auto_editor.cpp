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

struct EditorState {
   EditorPage page;
   EditorView view;
   element *top_bar;

   EditorSelectedType selected_type;
   union {
      AutoNode *selected_node;
      AutoPath *selected_path;
   };

   MemoryArena project_arena;
   AutoProjectLink *project;

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
            state->view = EditorView_Editing;
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
                                         Size(page->bounds).x);

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
         state->project->name = Literal("auto_project");
         
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
void DrawNode(ui_field_topdown *field, EditorState *state, AutoNode *node) {
   UI_SCOPE(field->e->context, node);
   element *e = Panel(field->e, RectCenterSize(GetPoint(field, node->pos), V2(10, 10)),
                      Captures(INTERACTION_CLICK | INTERACTION_DRAG));
   DrawNodeGraphic(e, state, node);
   
   if(WasClicked(e)) {
      state->selected_type = NodeSelected;
      state->selected_node = node;
   }
   
   node->pos = ClampTo(node->pos + PixelsToFeet(field, GetDrag(e)),
                       RectCenterSize(V2(0, 0), field->size_in_ft));
            
   for(u32 i = 0; i < node->path_count; i++) {
      DrawPath(field, state, node->out_paths[i]);
   }
}

void DrawPath(ui_field_topdown *field, EditorState *state, AutoPath *path) {
   UI_SCOPE(field->e->context, path);
   
   u32 control_point_count = path->control_point_count + 2;
   AutonomousProgram_ControlPoint *control_points = PushTempArray(AutonomousProgram_ControlPoint, control_point_count);
   control_points[0] = { path->in_node->pos, path->in_tangent };
   control_points[control_point_count - 1] = { path->out_node->pos, path->out_tangent };
   Copy(path->control_points, path->control_point_count * sizeof(AutonomousProgram_ControlPoint), control_points + 1);

   for(u32 i = 1; i < control_point_count; i++) {
      AutonomousProgram_ControlPoint a = control_points[i - 1];
      AutonomousProgram_ControlPoint b = control_points[i];
      CubicHermiteSpline(field, a.pos, a.tangent, b.pos, b.tangent, GREEN);
   }

   if(path->in_node == state->project->starting_node) {

   }

   DrawNode(field, state, path->out_node);
}

void DrawEditingView(element *page, EditorState *state) {
   Assert(state->project != NULL);

   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
   Label(state->top_bar, state->project->name, Size(state->top_bar).y, WHITE);
   Panel(state->top_bar, V2(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "Save", menu_button).clicked) {
      
   }

   RobotProfile *profile = &state->profiles.current;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size, Size(page->bounds).x);
   element *field_click = Panel(field.e, field.bounds, Captures(INTERACTION_CLICK));

   {
      AutoNode *starting_node = state->project->starting_node;
      element *starting_node_panel = Panel(field.e, RectCenterSize(GetPoint(&field, starting_node->pos), V2(10, 10)),
                                          Captures(INTERACTION_CLICK));
      DrawNodeGraphic(starting_node_panel, state, starting_node);
      
      if(WasClicked(starting_node_panel)) {
         state->selected_type = NodeSelected;
         state->selected_node = starting_node;
      }
      
      for(u32 i = 0; i < starting_node->path_count; i++) {
         DrawPath(&field, state, starting_node->out_paths[i]);
      }
   }

   switch(state->selected_type) {
      case NodeSelected: {
         if(WasClicked(field_click)) {
            AutoPath *new_path = PushStruct(&state->project_arena, AutoPath);
            AutoNode *new_node = PushStruct(&state->project_arena, AutoNode);
            
            new_node->pos = PixelsToFeet(&field, Cursor(field_click) - field.bounds.min) - 0.5 * field.size_in_ft;
            new_path->in_node = state->selected_node;
            new_path->out_node = new_node;
            
            AutoPath **new_out_paths = PushArray(&state->project_arena, AutoPath *, state->selected_node->path_count + 1);
            Copy(state->selected_node->out_paths, state->selected_node->path_count * sizeof(AutoPath *), new_out_paths);
            new_out_paths[state->selected_node->path_count] = new_path;

            state->selected_node->path_count = state->selected_node->path_count + 1;
            state->selected_node->out_paths = new_out_paths;
         }

         element *node_panel = ColumnPanel(page, V2(Size(page).x - 10, 500), Padding(5, 5));
         Background(node_panel, dark_grey);
      } break;
   }

   InputState *input = &page->context->input_state;
   if(input->key_esc) {
      state->selected_type = NothingSelected;
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