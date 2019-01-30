void DrawSimRobot(ui_field_topdown *field, SimRobot state, v4 colour) {
   v2 heading = DirectionNormal(ToDegrees(state.angle));
   v2 a = state.pos + heading * 0.5*state.size.x + Perp(heading) * 0.5*state.size.y;
   v2 b = state.pos + heading * 0.5*state.size.x - Perp(heading) * 0.5*state.size.y;
   v2 c = state.pos - heading * 0.5*state.size.x - Perp(heading) * 0.5*state.size.y;
   v2 d = state.pos - heading * 0.5*state.size.x + Perp(heading) * 0.5*state.size.y;
   Loop(field->e, colour, 3, GetPoint(field, a), GetPoint(field, b), GetPoint(field, c), GetPoint(field, d));
   Line(field->e, colour, 3, GetPoint(field, state.pos), GetPoint(field, state.pos + heading*0.5*state.size.y));
}

void DrawSimField(element *page, EditorState *state) {
   static float field_width = 700;
   field_width = Clamp(0, Size(page->bounds).x, field_width);

   UIContext *context = page->context;
   ui_field_topdown field = FieldTopdown(page, state->settings.field.image, state->settings.field.size, field_width);
   
   element *resize_divider = Panel(page, Size(Size(page).x - 10, 10).Padding(5, 5).Captures(INTERACTION_DRAG));
   Background(resize_divider, dark_grey);
   field_width += GetDrag(resize_divider).y * (field.size_in_ft.x / field.size_in_ft.y);

   Rectangle(field.e, RectCenterSize(GetPoint(&field, V2(0, 0)), V2(5, 5)), GREEN);
   Rectangle(field.e, RectCenterSize(GetPoint(&field, V2(5, 0)), V2(5, 5)), GREEN);
   Rectangle(field.e, RectCenterSize(GetPoint(&field, V2(0, 5)), V2(5, 5)), GREEN);

   DrawSimRobot(&field, state->simulator.state, GREEN);
   element *sim_ui = ColumnPanel(page, Width(Size(page).x - 10));

   Label(sim_ui, Concat(Literal("X:"), ToString(state->simulator.state.pos.x)), 20, WHITE);
   TextBox(sim_ui, &state->simulator.state.pos.x, 20);
   
   Label(sim_ui, Concat(Literal("Y:"), ToString(state->simulator.state.pos.y)), 20, WHITE);
   TextBox(sim_ui, &state->simulator.state.pos.y, 20);
   
   Label(sim_ui, Concat(Literal("Angle:"), ToString(ToDegrees(state->simulator.state.angle))), 20, WHITE);
   f32 angle_in_degrees = ToDegrees(state->simulator.state.angle);
   if(TextBox(sim_ui, &angle_in_degrees, 20).valid_enter) {
      state->simulator.state.angle = CanonicalizeAngle_Radians(ToRadians(angle_in_degrees));
   }

   if(Button(sim_ui, "Clear Plan", menu_button).clicked) {
      state->simulator.path = NULL;
      state->simulator.pivot = NULL;
   }

   if(!state->simulator.run_sim) {
      element *robot_drag = Panel(field.e, RectCenterSize(GetPoint(&field, state->simulator.state.pos), FeetToPixels(&field, state->simulator.state.size)), Captures(INTERACTION_DRAG));
      Background(robot_drag, V4(0, 0, 0, 0.2));
      state->simulator.state.pos = state->simulator.state.pos + PixelsToFeet(&field, GetDrag(robot_drag));

      v2 robot_rotate_handle_pos = state->simulator.state.pos + DirectionNormal(ToDegrees(state->simulator.state.angle))*0.5*state->simulator.state.size.y;
      element *robot_rotate_handle = Panel(field.e, RectCenterSize(GetPoint(&field, robot_rotate_handle_pos), V2(10, 10)), Captures(INTERACTION_ACTIVE));
      Background(robot_rotate_handle, BLACK);

      if(IsActive(robot_rotate_handle)) {
         state->simulator.state.angle = CanonicalizeAngle_Radians(ToRadians(Angle(Cursor(robot_rotate_handle) - GetPoint(&field, state->simulator.state.pos))));
      }
   }

   Panel(sim_ui, Size(Size(sim_ui).x, 10));
   if(state->simulator.path != NULL) {
      element *path_panel = ColumnPanel(sim_ui, Width(Size(sim_ui).x));
      Background(path_panel, dark_grey);
      Label(path_panel, "Path", 40, WHITE);
      Label(path_panel, Concat(Literal("Simulation t="), ToString(state->simulator.t)), 20, WHITE);
      CheckBox(path_panel, &state->simulator.run_sim, V2(20, 20));
      Label(path_panel, Concat(Literal("Kp="), ToString(state->simulator.kp)), 20, WHITE);
      HorizontalSlider(path_panel, &state->simulator.kp, 0, 20, V2(Size(path_panel).x, 20));
      Label(path_panel, Concat(Literal("Drag="), ToString(state->simulator.drag)), 20, WHITE);
      HorizontalSlider(path_panel, &state->simulator.drag, 0, 10, V2(Size(path_panel).x, 20));

      PathPlan *plan = state->simulator.path;
      {
         u32 sample_count = 40;
         f32 step = plan->length / (sample_count - 1);
         for(u32 i = 0; i < sample_count; i++) {
            PathPlanSample sample = {};
            MapLookup(&plan->map, i * step, &sample);

            Rectangle(field.e, RectCenterSize(GetPoint(&field, sample.pose.pos), V2(4, 4)), RED);
         }
      }

      if(Button(path_panel, "Reset", menu_button).clicked) {
         state->simulator.t = 0;
         state->simulator.run_sim = false;

         PathPlanSample starting_sample = {};
         MapLookup(&plan->map, 0, &starting_sample);

         state->simulator.state.pos = starting_sample.pose.pos;
         state->simulator.state.angle = starting_sample.pose.angle;
         state->simulator.state.left_p = 0;
         state->simulator.state.left_v = 0;
         state->simulator.state.left_setpoint = 0;

         state->simulator.state.right_p = 0;
         state->simulator.state.right_v = 0;
         state->simulator.state.right_setpoint = 0;
      }

      if(Button(path_panel, "Dist Graph", menu_button).clicked) {
         ResetMultiLineGraph(&state->simulator.graph);
   
         AutoRobotPose curr_pose = { state->simulator.state.pos, state->simulator.state.angle };
         u32 sample_count = 40;
         f32 step = plan->length / (sample_count - 1);
         for(u32 i = 0; i < sample_count; i++) {
            f32 curr_s = i * step;
            PathPlanSample sample = {};
            MapLookup(&plan->map, curr_s, &sample);
            f32 curr_dist = Length(sample.pose.pos - curr_pose.pos);
            AddEntry(&state->simulator.graph, Literal("Dist"), curr_dist, curr_s, 1);
         }
      }

      AutoRobotPose curr_pose = {state->simulator.state.pos, state->simulator.state.angle};
      PathPoint curr_point = GetCurrentPoint(plan, curr_pose);

      Label(path_panel, curr_point.on_path ? "On Path" : "Off Path", 20, WHITE);
      Label(path_panel, Concat(Literal("Current S = "), ToString(curr_point.s)), 20, WHITE);
      Rectangle(field.e, RectCenterSize(GetPoint(&field, curr_point.point), V2(5, 5)), curr_point.on_path ? GREEN : RED);

      if(state->simulator.run_sim) {
         f32 dt = context->dt;
         state->simulator.t += dt;

         RunSim(&state->simulator, dt, &field);
      } else {
         static AdjustmentPathParams adj = {10, 10, 0};
         static bool run_gradient_descent = true;

         AdjustmentPathMetrics adj_metrics = {};
         f32 curr_cost = AdjustmentPathCost(plan, curr_pose, adj, &adj_metrics);
         Label(path_panel, Concat(Literal("(Toggle Automatic Optimization) Cost = "), ToString(curr_cost)), 20, WHITE);
         CheckBox(path_panel, &run_gradient_descent, V2(20, 20));
         Label(path_panel, Concat(Literal("length = "), ToString(adj_metrics.length)), 20, WHITE);
         Label(path_panel, Concat(Literal("max_theta_dot = "), ToString(adj_metrics.max_theta_dot)), 20, WHITE);
         Label(path_panel, Concat(Literal("avg_theta_dot = "), ToString(adj_metrics.avg_theta_dot)), 20, WHITE);

         if(run_gradient_descent) {
            adj = OptimizeAdjustmentPath(plan, curr_pose);
            Label(path_panel, Concat(Literal("A"), ToString(adj.a)), 20, WHITE);
            Label(path_panel, Concat(Literal("B"), ToString(adj.b)), 20, WHITE);
            Label(path_panel, Concat(Literal("S"), ToString(adj.s)), 20, WHITE);
         } else {
            Label(path_panel, Concat(Literal("A"), ToString(adj.a)), 20, WHITE);
            HorizontalSlider(path_panel, &adj.a, 0, 20, V2(Size(path_panel).x - 20, 20));
            Label(path_panel, Concat(Literal("B"), ToString(adj.b)), 20, WHITE);
            HorizontalSlider(path_panel, &adj.b, 0, 20, V2(Size(path_panel).x - 20, 20));
            Label(path_panel, Concat(Literal("S"), ToString(adj.s)), 20, WHITE);
            HorizontalSlider(path_panel, &adj.s, 0, plan->length, V2(Size(path_panel).x - 20, 20));
         }

         North_HermiteControlPoint A = { curr_pose.pos, adj.a * DirectionNormal(ToDegrees(curr_pose.angle)) };
         AutoRobotPose BPose = GetPoseAtS(plan, adj.s);
         North_HermiteControlPoint B = { BPose.pos, adj.b * DirectionNormal(ToDegrees(BPose.angle)) };

         CubicHermiteSpline(&field, A.pos, A.tangent, B.pos, B.tangent, BLACK, 2);
      }
   } else if(state->simulator.pivot != NULL) {
      
   } else {
      element *fk_panel = ColumnPanel(sim_ui, Width(Size(sim_ui).x));
      Background(fk_panel, dark_grey);
      Label(fk_panel, "Forward Kinematics", 40, WHITE);
      
      static f32 pr = 0;
      static f32 pl = 0;

      Label(fk_panel, Concat(Literal("Right Move: "), ToString(pr)), 20, WHITE);
      TextBox(fk_panel, &pr, 20);
      
      Label(fk_panel, Concat(Literal("Left Move: "), ToString(pl)), 20, WHITE);
      TextBox(fk_panel, &pl, 20);

      SimRobot new_state = ForwardKinematics(state->simulator.state, pr, pl);
      DrawSimRobot(&field, new_state, BLUE);

      v2 ICC = GetCurvatureCenter(state->simulator.state.pos, 
                                 DirectionNormal(ToDegrees(state->simulator.state.angle)),
                                 new_state.pos);
      Circle(field.e, GetPoint(&field, ICC), FeetToPixels(&field, Length(ICC - new_state.pos)), BLACK, 3);

      Label(fk_panel, Concat(Literal("X:"), ToString(new_state.pos.x), 
                             Literal("Y:"), ToString(new_state.pos.y), 
                             Literal("Angle:"), ToString(ToDegrees(new_state.angle))), 20, WHITE);

      if(Button(fk_panel, "Move", menu_button).clicked) {
         state->simulator.state = new_state;
      }
   }
   
   FinalizeLayout(sim_ui);
}

void DrawSimulator(element *full_page, EditorState *state) {
   StackLayout(full_page);
   element *page = VerticalList(full_page);
   
   static bool show_graph = false;
   CheckBox(page, &show_graph, V2(15, 15));
   if(show_graph) {
      MultiLineGraph(page, &state->simulator.graph, V2(Size(page).x, 500));
   }
   
   if(state->settings.field.loaded) {
      DrawSimField(page, state);
   } else {
      Label(page, "No Field", V2(Size(page).x, 80), 50, BLACK);
   }
}
