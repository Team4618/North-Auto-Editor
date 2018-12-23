void DrawSimulationView(element *page, EditorState *state) {
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

   InputState *input = &page->context->input_state;
   RobotProfile *profile = &state->profiles.current;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size, field_width);
   
   element *resize_divider = Panel(page, Size(Size(page).x - 10, 10).Padding(5, 5).Captures(INTERACTION_DRAG));
   Background(resize_divider, dark_grey);
   field_width += GetDrag(resize_divider).y * (field.size_in_ft.x / field.size_in_ft.y);

   DrawNode(&field, state, state->project->starting_node, false);
   
   switch(state->selected_type) {
      case NodeSelected: {
         
      } break;

      case PathSelected: {
         
      } break;
   }

   if(input->key_esc) {
      state->selected_type = NothingSelected;
   }
   
   if(Button(state->top_bar, "Editor", menu_button).clicked) {
      state->view = EditorView_Editing;
   }

   if(Button(state->top_bar, "Exit", menu_button).clicked) {
      Reset(&state->project_arena);
      state->project = NULL;
      state->view = EditorView_Blank;
   }
}
