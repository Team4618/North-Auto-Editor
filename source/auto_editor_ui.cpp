#define DrawTangentHandle(...) _DrawTangentHandle(GEN_UI_ID, __VA_ARGS__) 
void _DrawTangentHandle(ui_id id, ui_field_topdown *field, AutoPath *path,
                        v2 pos, v2 *tangent, bool along_normal, v2 normal) {
   v2 handle_pos = GetPoint(field, pos + *tangent);
   element *handle = _Panel(id, field->e, RectCenterSize(handle_pos, V2(10, 10)), Captures(INTERACTION_DRAG));
   Line(field->e, BLUE, 2, GetPoint(field, pos), handle_pos);
   Background(handle, BLUE);

   v2 drag_vector = along_normal ? (normal * Dot(normal, GetDrag(handle))) : GetDrag(handle);      
   *tangent = ClampTo(*tangent + PixelsToFeet(field, drag_vector),
                      (-pos) + RectCenterSize(V2(0, 0), field->size_in_ft));
   
   if(Length(drag_vector) > 0)
      RecalculateAutoPath(path);
}

void DrawSelectedNode(EditorState *state, ui_field_topdown *field, bool field_clicked, element *page) {
   AutoNode *selected_node = state->selected_node;
   RobotProfile *profile = &state->profiles.current;
   
   if(field_clicked) {
      AutoPath *new_path = PushStruct(&state->project_arena, AutoPath);
      AutoNode *new_node = PushStruct(&state->project_arena, AutoNode);
      
      new_node->pos = PixelsToFeet(field, Cursor(field->e) - field->bounds.min) - 0.5 * field->size_in_ft;
      new_node->in_path = new_path;
      new_path->in_node = state->selected_node;
      new_path->out_node = new_node;
      new_path->length_to_pos_memory = PlatformAllocArena(GetMapMemorySize());
      new_path->time_to_length_memory = PlatformAllocArena(GetMapMemorySize());
      
      if(new_path->in_node == state->project->starting_node) {
         v2 direction_arrow = V2(cosf(state->project->starting_angle * (PI32 / 180)), 
                                 -sinf(state->project->starting_angle * (PI32 / 180)));

         new_path->in_tangent = direction_arrow;
      } else {
         new_path->in_tangent = Normalize(new_path->out_node->pos - new_path->in_node->pos);
      }
      new_path->out_tangent = Normalize(new_path->out_node->pos - new_path->in_node->pos);
      
      RecalculateAutoPathLength(new_path);
      new_path->velocity_datapoint_count = 4;
      new_path->velocity_datapoints = PushArray(&state->project_arena, AutonomousProgram_DataPoint, 4);
      new_path->velocity_datapoints[0] = { 0, 0 };
      new_path->velocity_datapoints[1] = { new_path->length * 0.1f, 1 };
      new_path->velocity_datapoints[2] = { new_path->length * 0.9f, 1 };
      new_path->velocity_datapoints[3] = { new_path->length, 0 };
      RecalculateAutoPathTime(new_path);

      AutoPath **new_out_paths = PushArray(&state->project_arena, AutoPath *, selected_node->path_count + 1);
      Copy(selected_node->out_paths, selected_node->path_count * sizeof(AutoPath *), new_out_paths);
      new_out_paths[selected_node->path_count] = new_path;

      selected_node->path_count = selected_node->path_count + 1;
      selected_node->out_paths = new_out_paths;
   }

   element *edit_panel = ColumnPanel(page, Width(Size(page).x - 10).Padding(5, 5));
   Background(edit_panel, dark_grey);
   element *edit_buttons = RowPanel(edit_panel, Size(Size(edit_panel).x, page_tab_height));
   element *path_list = ColumnPanel(edit_panel, Width(Size(edit_panel).x)); 
   element *command_lists = RowPanel(edit_panel, Width(Size(edit_panel).x)); 
   element *available_command_list = ColumnPanel(command_lists, Width(Size(command_lists).x / 2));
   element *command_list = ColumnPanel(command_lists, Width(Size(command_lists).x / 2));

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
      element *path_panel = RowPanel(path_list, Size(Size(path_list).x - 10, 40).Padding(5, 5));
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
      element *command_panel = ColumnPanel(command_list, Size(Size(command_list).x, (100 + 20 * command_template->param_count)).Padding(0, 5).Captures(INTERACTION_DRAG));
      if(IsActive(command_panel)) {
         Outline(command_panel, WHITE);
      } else if(IsHot(command_panel)) {
         Outline(command_panel, V4(120/255.0, 120/255.0, 120/255.0, 1));
      } else {
         Outline(command_panel, light_grey);
      }
      
      element *button_row = RowPanel(command_panel, Size(Size(command_panel).x, page_tab_height)); 
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

      element *conditional_row = RowPanel(command_panel, Size(Size(command_panel).x, 40));
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
         
         element *param_row = RowPanel(command_panel, Size(Size(command_panel).x, 20)); 
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

   FinalizeLayout(edit_panel);
}

struct editable_graph_line {
   f32 min;
   f32 max;
   element *e;

   u32 sample_count;
   AutonomousProgram_DataPoint *samples;

   f32 length;
};

editable_graph_line GraphLine(element *e, f32 min, f32 max, 
                              AutoPath *path, AutonomousProgram_DataPoint *samples, u32 sample_count)
{
   editable_graph_line result = { min, max, e, sample_count, samples, path->length };
   return result;
}

v2 GetGraphPoint(editable_graph_line line, AutonomousProgram_DataPoint point) {
   f32 x = line.e->bounds.min.x + Size(line.e).x * (point.distance / line.length);
   f32 y = line.e->bounds.min.y + Size(line.e).y * (1 - ((point.value - line.min) / (line.max - line.min)));
   return V2(x, y);
}

v2 GetGraphPoint(editable_graph_line line, u32 i) {
   return GetGraphPoint(line, line.samples[i]);
}

AutonomousProgram_DataPoint GraphPointToDataPoint(editable_graph_line line, v2 point) {
   AutonomousProgram_DataPoint result = {};
   result.distance = ((point.x - line.e->bounds.min.x) / Size(line.e).x) * line.length;
   result.value = (1 - ((point.y - line.e->bounds.min.y) / Size(line.e).y) ) * (line.max - line.min) + line.min;
   return result;   
}

rect2 GetClampRectOnGraph(editable_graph_line line, u32 i) {
   f32 x1 = line.e->bounds.min.x;
   if(i > 0) {
      x1 = line.e->bounds.min.x + Size(line.e).x * (line.samples[i - 1].distance / line.length);
   }

   f32 x2 = line.e->bounds.max.x;
   if(i < (line.sample_count - 1)) {
      x2 = line.e->bounds.min.x + Size(line.e).x * (line.samples[i + 1].distance / line.length);
   }

   return RectMinMax(V2(x1, line.e->bounds.min.y), V2(x2, line.e->bounds.max.y));  
}

bool GraphHandle(editable_graph_line line, u32 i) {
   v2 point = GetGraphPoint(line, line.samples[i]);
   element *handle = Panel(line.e, RectCenterSize(point, V2(15, 15)), Captures(INTERACTION_DRAG));
   Background(handle, BLUE);

   if(IsHot(handle)) {
      Text(line.e, Concat(Literal("Value="), ToString(line.samples[i].value)), line.e->bounds.min, 20, WHITE);
   }

   v2 drag_vector = GetDrag(handle);
   if(Length(drag_vector) > 0) {
      v2 new_point = ClampTo(point + drag_vector, GetClampRectOnGraph(line, i));   
      line.samples[i] = GraphPointToDataPoint(line, new_point);
      return true;
   }

   return false;
}

void DrawSelectedPath(EditorState *state, ui_field_topdown *field, bool field_clicked, element *page) {
   AutoPath *selected_path = state->selected_path;
   RobotProfile *profile = &state->profiles.current;
   InputState *input = &page->context->input_state;

   element *edit_panel = ColumnPanel(page, Width(Size(page).x - 10).Padding(5, 5));
   Background(edit_panel, dark_grey);
   element *edit_buttons = RowPanel(edit_panel, Size(Size(edit_panel).x, page_tab_height));
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

   if(Button(edit_buttons, "Reverse", menu_button.IsSelected(selected_path->is_reverse)).clicked) {
      selected_path->is_reverse = !selected_path->is_reverse;
   }

   Label(edit_panel, Concat(Literal("Path length: "), ToString(selected_path->length)), 20, WHITE);
   Label(edit_panel, Concat(Literal("Time: "), ToString(selected_path->time)), 20, WHITE);

   element *graph = Panel(edit_panel, Size(Size(edit_panel).x - 10, 300).Padding(5, 5).Captures(INTERACTION_CLICK));
   Outline(graph, light_grey);

   bool new_velocity_sample = false;
   u32 new_velocity_sample_i = 0;

   editable_graph_line velocity_line = GraphLine(graph, 0, 20, selected_path, selected_path->velocity_datapoints, selected_path->velocity_datapoint_count);
   for(u32 i = 0; i < selected_path->velocity_datapoint_count; i++) {
      UI_SCOPE(graph, i);
      
      if((i != 0) && (i != (selected_path->velocity_datapoint_count - 1))) {
         if(GraphHandle(velocity_line, i)) {
            RecalculateAutoPath(selected_path);
         }
      }

      if(i > 0) {
         v2 point_a = GetGraphPoint(velocity_line, i - 1);
         v2 point_b = GetGraphPoint(velocity_line, i);
         
         Line(graph, GREEN, 2, point_a, point_b);
         element *new_sample_button = Panel(graph, RectCenterSize(Midpoint(point_a, point_b), V2(5, 5)), Captures(INTERACTION_CLICK));
         Background(new_sample_button, RED);
         if(WasClicked(new_sample_button)) {
            new_velocity_sample = true;
            new_velocity_sample_i = i;
         }
      }
   }

   if(new_velocity_sample) {
      AutonomousProgram_DataPoint new_datapoint = {};
      new_datapoint.distance = (selected_path->velocity_datapoints[new_velocity_sample_i - 1].distance + selected_path->velocity_datapoints[new_velocity_sample_i].distance) / 2;
      new_datapoint.value = (selected_path->velocity_datapoints[new_velocity_sample_i - 1].value + selected_path->velocity_datapoints[new_velocity_sample_i].value) / 2;
      
      selected_path->velocity_datapoints = 
                     ArrayInsert(&state->project_arena, AutonomousProgram_DataPoint, selected_path->velocity_datapoints,
                                 new_velocity_sample_i, &new_datapoint, selected_path->velocity_datapoint_count++);

      RecalculateAutoPath(selected_path);
   }

   ForEachArray(i, cevent, selected_path->continuous_event_count, selected_path->continuous_events, {
      UI_SCOPE(graph, cevent);
      
      bool new_sample = false;
      u32 new_sample_i = 0;

      editable_graph_line curr_line = GraphLine(graph, 0, 20, selected_path, cevent->samples, cevent->sample_count);
      for(u32 i = 0; i < cevent->sample_count; i++) {
         UI_SCOPE(graph, cevent->samples + i);
         
         GraphHandle(curr_line, i);

         if(i == 0) {
            cevent->samples[i].distance = 0;
         } else if(i == (cevent->sample_count - 1)) { 
            cevent->samples[i].distance = selected_path->length;
         }

         if(i > 0) {
            v2 point_a = GetGraphPoint(curr_line, i - 1);
            v2 point_b = GetGraphPoint(curr_line, i);
            
            Line(graph, GREEN, 2, point_a, point_b);
            element *new_sample_button = Panel(graph, RectCenterSize(Midpoint(point_a, point_b), V2(5, 5)), Captures(INTERACTION_CLICK));
            Background(new_sample_button, RED);
            if(WasClicked(new_sample_button)) {
               new_sample = true;
               new_sample_i = i;
            }
         }
      }

      if(new_sample) {
         AutonomousProgram_DataPoint new_datapoint = {};
         new_datapoint.distance = (cevent->samples[new_sample_i - 1].distance + cevent->samples[new_sample_i].distance) / 2;
         new_datapoint.value = (cevent->samples[new_sample_i - 1].value + cevent->samples[new_sample_i].value) / 2;
         
         cevent->samples = ArrayInsert(&state->project_arena, AutonomousProgram_DataPoint, cevent->samples,
                                       new_sample_i, &new_datapoint, cevent->sample_count++);

         RecalculateAutoPath(selected_path);
      }
   });

   bool remove_devent = false;
   u32 remove_devent_i = 0;
   ForEachArray(i, devent, selected_path->discrete_event_count, selected_path->discrete_events, {
      RobotProfileCommand *command_template = GetCommand(profile, devent->subsystem_name, devent->command_name);
      Assert(command_template);
      
      UI_SCOPE(edit_panel, devent);
      element *command_panel = ColumnPanel(edit_panel, Width(Size(edit_panel).x).Padding(0, 5).Captures(INTERACTION_DRAG));
      
      element *button_row = RowPanel(command_panel, Size(Size(command_panel).x, page_tab_height)); 
      if(Button(button_row, "Delete", menu_button).clicked) {
         remove_devent = true;
         remove_devent_i = i;
      }

      string text = Concat(devent->subsystem_name, Literal(":"), devent->command_name, Literal("@"), ToString(devent->distance));
      Label(command_panel, text, 20, WHITE);
      for(u32 j = 0; j < command_template->param_count; j++) {
         f32 *param_value = devent->params + j;
         UI_SCOPE(command_panel->context, param_value);
         
         element *param_row = RowPanel(command_panel, Size(Size(command_panel).x, 20)); 
         Label(param_row, command_template->params[j], 20, WHITE);
         TextBox(param_row, param_value, 20);
      }

      HorizontalSlider(command_panel, &devent->distance, 0, selected_path->length, V2(Size(command_panel).x, 20));
   });

   if(remove_devent) {
      selected_path->discrete_events =
         ArrayRemove(&state->project_arena, AutoDiscreteEvent, selected_path->discrete_events,
                     remove_devent_i, selected_path->discrete_event_count--);
   }
   
   for(u32 i = 0; i < profile->subsystem_count; i++) {
      RobotProfileSubsystem *subsystem = profile->subsystems + i;
      
      for(u32 j = 0; j < subsystem->command_count; j++) {
         RobotProfileCommand *command = subsystem->commands + j;
         
         if(command->type == North_CommandType::Continuous) {
            bool already_has_line = false;

            for(u32 k = 0; k < selected_path->continuous_event_count; k++) {
               AutoContinuousEvent *cevent = selected_path->continuous_events + k;
               if((cevent->subsystem_name == subsystem->name) && (cevent->command_name == command->name)) {
                  already_has_line = true;
                  break;
               }
            }

            if(!already_has_line) {
               if(Button(edit_panel, Concat(Literal("C "), subsystem->name, Literal(":"), command->name), menu_button).clicked) {
                  AutoContinuousEvent new_cevent = {};
                  new_cevent.subsystem_name = PushCopy(&state->project_arena, subsystem->name);
                  new_cevent.command_name = PushCopy(&state->project_arena, command->name);
                  new_cevent.sample_count = 3;
                  new_cevent.samples = PushArray(&state->project_arena, AutonomousProgram_DataPoint, new_cevent.sample_count);

                  new_cevent.samples[0] = { 0, 0 };
                  new_cevent.samples[1] = { selected_path->length * 0.5f, 5 };
                  new_cevent.samples[2] = { selected_path->length, 10 };

                  selected_path->continuous_events = 
                     ArrayInsert(&state->project_arena, AutoContinuousEvent, selected_path->continuous_events,
                                 0, &new_cevent, selected_path->continuous_event_count++);
               }
            }
         } else if(command->type == North_CommandType::NonBlocking) {
            if(Button(edit_panel, Concat(Literal("D "), subsystem->name, Literal(":"), command->name), menu_button).clicked) {
               AutoDiscreteEvent new_event = {};
               new_event.subsystem_name = PushCopy(&state->project_arena, subsystem->name);
               new_event.command_name = PushCopy(&state->project_arena, command->name);
               new_event.param_count = command->param_count;
               new_event.params = PushArray(&state->project_arena, f32, command->param_count);
               
               selected_path->discrete_events = 
                  ArrayInsert(&state->project_arena, AutoDiscreteEvent, selected_path->discrete_events,
                              0, &new_event, selected_path->discrete_event_count++);
            }
         }
      };
   };

   FinalizeLayout(edit_panel);

   //NOTE: this has to be after FinalizeLayout because graph->bounds isnt valid until after
   if(IsHot(graph)) {
      f32 cursor_d = selected_path->length * ((Cursor(graph) - graph->bounds.min).x / Size(graph).x);
      Text(graph, Concat(Literal("Distance="), ToString(cursor_d)), graph->bounds.min, 20, WHITE);
      Rectangle(field->e, RectCenterSize(GetPoint(field, GetAutoPathPoint(selected_path, cursor_d)), V2(5, 5)), BLACK);
   }

   switch(state->path_edit) {
      case EditControlPoints: {
         if(selected_path->in_node == state->project->starting_node) {
            v2 direction_arrow = V2(cosf(state->project->starting_angle * (PI32 / 180)), 
                                    -sinf(state->project->starting_angle * (PI32 / 180)));
            
            DrawTangentHandle(field, selected_path, selected_path->in_node->pos, &selected_path->in_tangent,
                              true, direction_arrow);
            
         } else {
            DrawTangentHandle(field, selected_path, selected_path->in_node->pos, &selected_path->in_tangent,
                              input->ctrl_down, Normalize(selected_path->in_tangent));
         }
         
         for(u32 i = 0; i < selected_path->control_point_count; i++) {
            AutonomousProgram_ControlPoint *control_point = selected_path->control_points + i;
            UI_SCOPE(field->e->context, control_point);
            DrawTangentHandle(field, selected_path, control_point->pos, &control_point->tangent,
                              input->ctrl_down, Normalize(control_point->tangent));
         }

         DrawTangentHandle(field, selected_path, selected_path->out_node->pos, &selected_path->out_tangent,
                           input->ctrl_down, Normalize(selected_path->out_tangent));
      } break;

      case AddControlPoint: {
         AutoPathSpline path_spline = GetAutoPathSpline(selected_path);

         bool add_point = false;
         AutonomousProgram_ControlPoint new_point = {};
         u32 insert_index = 0;

         for(u32 i = 1; i < path_spline.point_count; i++) {
            UI_SCOPE(field->e, i);
            v2 new_pos = CubicHermiteSpline(path_spline.points[i - 1], path_spline.points[i], 0.5);
            element *new_button = Panel(field->e, RectCenterSize(GetPoint(field, new_pos), V2(10, 10)), Captures(INTERACTION_CLICK));
            Background(new_button, BLUE);
            if(IsHot(new_button)) {
               Outline(new_button, BLACK);
            }

            if(WasClicked(new_button)) {
               insert_index = i - 1;
               new_point.pos = new_pos;
               new_point.tangent = CubicHermiteSplineTangent(path_spline.points[i - 1], path_spline.points[i], 0.5);
               add_point = true;
            }
         }

         if(add_point) {
            selected_path->control_points = 
               ArrayInsert(&state->project_arena, AutonomousProgram_ControlPoint, selected_path->control_points,
                           insert_index, &new_point, selected_path->control_point_count);
            selected_path->control_point_count++;
            RecalculateAutoPath(selected_path);
         }
      } break;
   }
}

void DrawNode(ui_field_topdown *field, EditorState *state, AutoNode *node, bool can_drag = true);
void DrawEditingView(element *page, EditorState *state) {
   Assert(state->project != NULL);

   Panel(state->top_bar, Size(40, Size(state->top_bar).y));
   
   ui_textbox project_name_box = TextBox(state->top_bar, &state->project_name_box, Size(state->top_bar).y);
   if(IsSelected(project_name_box.e))
      Outline(project_name_box.e, (GetText(project_name_box).length > 0) ? GREEN : RED);
   
   state->project->name = GetText(project_name_box);

   Panel(state->top_bar, Size(40, Size(state->top_bar).y));
   if(Button(state->top_bar, "Save", menu_button.IsEnabled(GetText(project_name_box).length > 0)).clicked) {
      WriteProject(state->project);
   }

   static float field_width = 700;
   field_width = Clamp(0, Size(page->bounds).x, field_width);

   RobotProfile *profile = &state->profiles.current;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size, field_width);
   state->path_got_selected = false;

   element *resize_divider = Panel(page, Size(Size(page).x - 10, 10).Padding(5, 5).Captures(INTERACTION_DRAG));
   Background(resize_divider, dark_grey);
   field_width += GetDrag(resize_divider).y * (field.size_in_ft.x / field.size_in_ft.y);

   DrawNode(&field, state, state->project->starting_node, false);
   
   InputState *input = &page->context->input_state;
   bool field_clicked = field.clicked && !state->path_got_selected;

   switch(state->selected_type) {
      case NodeSelected: {
         DrawSelectedNode(state, &field, field_clicked, page);
      } break;

      case PathSelected: {
         DrawSelectedPath(state, &field, field_clicked, page);
      } break;
   }

   if(input->key_esc) {
      state->selected_type = NothingSelected;
   }

   if(Button(state->top_bar, "Simulator", menu_button).clicked) {
      state->view = EditorView_Simulating;
      //TODO: generate robot along path texture
   }

   if(Button(state->top_bar, "Exit", menu_button).clicked) {
      Reset(&state->project_arena);
      state->project = NULL;
      state->view = EditorView_Blank;
   }
}