v2 GetCurvatureCenter(v2 A, v2 At, v2 B) {
   v2 perp = Perp(B - A);
   f32 a = (Dot(At, A) - Dot(At, (A + B) / 2)) / Dot(At, perp);
   return ((A + B) / 2) + a * perp;
}

struct SimRobot {
   v2 size;

   v2 pos;
   f32 angle;
   
   f32 left_p;
   f32 left_v;
   f32 left_a;
   
   f32 right_p;
   f32 right_v;
   f32 right_a;
};

AutoRobotPose GetPose(SimRobot bot) {
   AutoRobotPose pose = { bot.pos, bot.angle };
   return pose;
}

AutoRobotPose ForwardKinematics(AutoRobotPose pose, f32 drivebase, f32 pr, f32 pl) {
   f32 new_x = 0;
   f32 new_y = 0;
   if(pr == pl) {
      new_x = pr * cos(pose.angle);
      new_y = pr * sin(pose.angle);
   } else {
      new_x = (drivebase / 2) * ((pr + pl) / (pr - pl)) * ( sin((pr - pl) / drivebase + pose.angle) - sin(pose.angle) );
      new_y = (drivebase / 2) * ((pr + pl) / (pr - pl)) * ( cos((pr - pl) / drivebase + pose.angle) - cos(pose.angle) );
   }

   AutoRobotPose result = {};
   result.pos = pose.pos + V2(new_x, new_y);
   result.angle = CanonicalizeAngle_Radians(pose.angle + ( (pr - pl) / drivebase ));
   return result;
}

SimRobot ForwardKinematics(SimRobot curr, f32 pr, f32 pl) {
   f32 drivebase = curr.size.x;
   AutoRobotPose new_pose = ForwardKinematics(GetPose(curr), drivebase, pr, pl);
   
   SimRobot result = curr;
   result.pos = new_pose.pos;
   result.angle = new_pose.angle;
   return result;
}

struct PathPlan {
   AutoVelocityDatapoints original_velocity;
   f32 original_length;
   InterpolatingMap original_map;
   
   AutoVelocityDatapoints velocity;
   f32 length;
   InterpolatingMap map; //NOTE: f32 -> AutoPathData
};

struct PathPoint {
   bool on_path;
   v2 point;
   f32 s;
};

AutoPathData GetSampleAtS(PathPlan *plan, f32 s) {
   AutoPathData sample = {};
   MapLookup(&plan->map, s, &sample);
   return sample;
}

AutoRobotPose GetPoseAtS(PathPlan *plan, f32 s) {
   return GetSampleAtS(plan, s).pose;
}

f32 GetDistAtS(PathPlan *plan, f32 s, v2 pos) {
   return Length(GetPoseAtS(plan, s).pos - pos);
}

const f32 min_path_dist = 0.5;
const f32 min_path_heading_diff = ToRadians(30);

PathPoint GetCurrentPoint(PathPlan *plan, AutoRobotPose pose) {
   f32 closest_dist = F32_MAX;
   f32 closest_s = 0;
   
   u32 sample_count = 40;
   f32 step = plan->length / (sample_count - 1);
   for(u32 i = 0; i < sample_count; i++) {
      f32 curr_s = i * step;
      AutoPathData sample = {};
      MapLookup(&plan->map, curr_s, &sample);
      f32 curr_dist = Length(sample.pose.pos - pose.pos);

      if(curr_dist < closest_dist) {
         closest_dist = curr_dist;
         closest_s = curr_s;
      }
   }

   f32 lower_s = 0;
   f32 lower_dist = 0;
   f32 upper_s = 0;
   f32 upper_dist = 0;

   if((closest_s + step) > plan->length) {
      lower_s = closest_s - step;
      lower_dist = GetDistAtS(plan, lower_s, pose.pos);
      upper_s = closest_s;
      upper_dist = closest_dist;
   } else if((closest_s - step) < 0) {
      lower_s = closest_s;
      lower_dist = closest_dist;
      upper_s = closest_s + step;
      upper_dist = GetDistAtS(plan, upper_s, pose.pos);
   } else {
      f32 next_s = closest_s + step;
      f32 prev_s = closest_s - step;
      f32 next_dist = GetDistAtS(plan, next_s, pose.pos);
      f32 prev_dist = GetDistAtS(plan, prev_s, pose.pos);

      lower_s = (next_dist > prev_dist) ? prev_s : closest_s;
      lower_dist = (next_dist > prev_dist) ? prev_dist : closest_dist;
      upper_s = (next_dist > prev_dist) ? closest_s : next_s;
      upper_dist = (next_dist > prev_dist) ? closest_dist : next_dist;
   }
   
   u32 recurse_count = 5;
   for(u32 i = 0; i < recurse_count; i++) {
      f32 midpoint_s = (lower_s + upper_s) / 2.0;
      f32 midpoint_dist = GetDistAtS(plan, midpoint_s, pose.pos);

      bool upper_is_farther = upper_dist > lower_dist;
      lower_s = upper_is_farther ? lower_s : midpoint_s;
      lower_dist = upper_is_farther ? lower_dist : midpoint_dist;
      upper_s = upper_is_farther ? midpoint_s : upper_s;
      upper_dist = upper_is_farther ? midpoint_dist : upper_dist;
   }

   closest_s = (lower_s + upper_s) / 2.0;
   closest_dist = GetDistAtS(plan, closest_s, pose.pos);
   AutoRobotPose closest_pose = GetPoseAtS(plan, closest_s);

   PathPoint result = {};
   result.on_path = (closest_dist <= min_path_dist) && 
                    (abs(ShortestAngleBetween_Radians(closest_pose.angle, pose.angle)) <= min_path_heading_diff);
   result.point = closest_pose.pos;
   result.s = closest_s;
   return result;
} 

struct AdjustmentPathParams {
   f32 a;
   f32 b;
   f32 s;
};

struct AdjustmentPathMetrics {
   f32 length;
   f32 max_theta_dot;
   f32 avg_theta_dot;
};

f32 AdjustmentPathCost(PathPlan *plan, AutoRobotPose pose, f32 a, f32 b, f32 s, AdjustmentPathMetrics *metrics = NULL) {
   North_HermiteControlPoint A = { pose.pos, a * DirectionNormal(ToDegrees(pose.angle)) };
   AutoRobotPose BPose = GetPoseAtS(plan, s);
   North_HermiteControlPoint B = { BPose.pos, b * DirectionNormal(ToDegrees(BPose.angle)) };
   
   f32 length = 0;
   f32 max_theta_dot = 0;
   f32 avg_theta_dot = 0;

   u32 sample_count = 20;
   f32 step = 1.0 / (sample_count - 1);
   f32 last_t = 0;
   for(u32 i = 0; i < sample_count; i++) {
      f32 curr_t = step * i;
      v2 last_pos = CubicHermiteSpline(A.pos, A.tangent, B.pos, B.tangent, last_t);
      v2 last_tangent = CubicHermiteSplineTangent(A.pos, A.tangent, B.pos, B.tangent, last_t);
      v2 curr_pos = CubicHermiteSpline(A.pos, A.tangent, B.pos, B.tangent, curr_t);
      v2 curr_tangent = CubicHermiteSplineTangent(A.pos, A.tangent, B.pos, B.tangent, curr_t);

      f32 theta_dot = abs(ShortestAngleBetween_Radians(ToRadians(Angle(last_tangent)), ToRadians(Angle(curr_tangent))) / step);

      length += Length(curr_pos - last_pos);
      max_theta_dot = Max(max_theta_dot, theta_dot);
      avg_theta_dot += theta_dot;

      last_t = curr_t;
   }

   avg_theta_dot /= length;
   
   if(metrics != NULL) {
      metrics->length = length;
      metrics->max_theta_dot = max_theta_dot;
      metrics->avg_theta_dot = avg_theta_dot;
   }

   return (1.0*length + 4.0*max_theta_dot + 2.0*avg_theta_dot) / (1);
}

f32 AdjustmentPathCost(PathPlan *plan, AutoRobotPose pose, AdjustmentPathParams params, AdjustmentPathMetrics *metrics = NULL) {
   return AdjustmentPathCost(plan, pose, params.a, params.b, params.s, metrics);
}

AdjustmentPathParams OptimizeAdjustmentPath(PathPlan *plan, AutoRobotPose pose) {
   PathPoint curr_point = GetCurrentPoint(plan, pose);
   AdjustmentPathParams adj = { 10, 10, curr_point.s };

   //NOTE: initial s value seeding
   u32 seed_sample_count = 10;
   f32 seed_start_s = Clamp(0, plan->length, adj.s - plan->length * 0.25);
   f32 seed_end_s = Clamp(0, plan->length, adj.s + plan->length * 0.25);
   f32 seed_step = (seed_end_s - seed_start_s) / (seed_sample_count - 1);

   f32 min_seed_cost = F32_MAX;

   for(u32 i = 0; i < seed_sample_count; i++) {
      f32 curr_s = seed_start_s + seed_step * i;
      f32 curr_cost = AdjustmentPathCost(plan, pose, adj.a, adj.b, curr_s);
      if(curr_cost < min_seed_cost) {
         min_seed_cost = curr_cost;
         adj.s = curr_s;
      }
   }

   //NOTE: gradient descent 
   f32 d_step = 0.001;
   f32 step_scale = 0.01;
   u32 gradient_descent_count = 100;
   f32 max_grad_value = 10;
   f32 max_a_or_b = 20;

   for(u32 i = 0; i < gradient_descent_count; i++) {
      f32 curr_cost = AdjustmentPathCost(plan, pose, adj.a,          adj.b,          adj.s);
      
      // string text = Concat(ToString(i), Literal(": "), ToString(curr_cost), Literal(", "), ToString(adj.a), Concat(Literal(", "), ToString(adj.b), Literal(", "), ToString(adj.s), Literal("\n")));
      // OutputDebugString(ToCString(text));
      
      f32 dcost_da = (AdjustmentPathCost(plan, pose, adj.a + d_step, adj.b,          adj.s) - curr_cost) / d_step;
      f32 dcost_db = (AdjustmentPathCost(plan, pose, adj.a,          adj.b + d_step, adj.s) - curr_cost) / d_step;
      f32 dcost_ds = (AdjustmentPathCost(plan, pose, adj.a,          adj.b,          adj.s + d_step) - curr_cost) / d_step;

      adj.a -= step_scale * Sign(dcost_da) * Clamp(0, max_grad_value, abs(dcost_da));
      adj.a = Clamp(0, max_a_or_b, adj.a);
      adj.b -= step_scale * Sign(dcost_db) * Clamp(0, max_grad_value, abs(dcost_db));
      adj.b = Clamp(0, max_a_or_b, adj.b);
      adj.s -= step_scale * Sign(dcost_ds) * Clamp(0, max_grad_value, abs(dcost_ds));
      adj.s = Clamp(0, plan->length, adj.s);
   }

   return adj;
}

AutoRobotPose GetPoseAt(f32 curr_s, PathPlan *plan, InterpolatingMap *adj_pose_map, f32 adj_length, f32 adj_s_start) {
   if(curr_s <= adj_length) {
      AutoRobotPose curr_pose = {};
      MapLookup(adj_pose_map, curr_s, &curr_pose);
      return curr_pose;
   } else {
      f32 path_s = (curr_s - adj_length) + adj_s_start;
      return GetPoseAtS(plan, path_s);
   }
}

bool SimFollowLoop(PathPlan *plan, SimRobot *bot, ui_field_topdown *field, f32 time, 
                   bool acceleration_control, MultiLineGraphData *graph)
{
   AutoRobotPose curr_pose = { bot->pos, bot->angle };
   PathPoint curr_point = GetCurrentPoint(plan, curr_pose);
   
   if(!curr_point.on_path) {
      //TODO: build this using the original path, not the current path
      AdjustmentPathParams adj_params = OptimizeAdjustmentPath(plan, curr_pose);
      
      North_HermiteControlPoint A = { bot->pos, adj_params.a * DirectionNormal(ToDegrees(bot->angle)) };
      AutoRobotPose BPose = GetPoseAtS(plan, adj_params.s);
      North_HermiteControlPoint B = { BPose.pos, adj_params.b * DirectionNormal(ToDegrees(BPose.angle)) };
      North_HermiteControlPoint adj_spline[] = { A, B };

      CubicHermiteSpline(field, A.pos, A.tangent, B.pos, B.tangent, BLACK, 2);
      InterpolatingMap adj_pose_map = {};
      adj_pose_map.arena = PushTempArena(Megabyte(1));
      adj_pose_map.sample_exp = 7;
      adj_pose_map.lerp_callback = map_lerp_pose;
      f32 adj_length = BuildPathMap(&adj_pose_map, adj_spline, ArraySize(adj_spline));
      f32 original_path_length = plan->length - adj_params.s;
      f32 total_length = adj_length + original_path_length;

      TempArena temp;
      u32 sample_count = Power(2, plan->map.sample_exp);
      AutoPathData *new_samples = PushArray(&temp.arena, AutoPathData, sample_count);
      f32 step = total_length / (sample_count - 1);

      for(u32 i = 0; i < sample_count; i++) {
         f32 curr_s = i * step;
         AutoRobotPose curr_pose = GetPoseAt(curr_s, plan, &adj_pose_map, adj_length, adj_params.s);

         f32 dtheta_ds = 0;
         f32 d2theta_ds2 = 0;

         if(i == 0) {
            AutoRobotPose next_pose = GetPoseAt(curr_s + step, plan, &adj_pose_map, adj_length, adj_params.s);
            dtheta_ds = ShortestAngleBetween_Radians(curr_pose.angle, next_pose.angle) / step;
         } else {
            AutoRobotPose last_pose = GetPoseAt(curr_s - step, plan, &adj_pose_map, adj_length, adj_params.s);
            dtheta_ds = ShortestAngleBetween_Radians(last_pose.angle, curr_pose.angle) / step;
         }

         if((i != 0) && (i != sample_count - 1)) {
            AutoRobotPose next_pose = GetPoseAt(curr_s + step, plan, &adj_pose_map, adj_length, adj_params.s);
            AutoRobotPose last_pose = GetPoseAt(curr_s - step, plan, &adj_pose_map, adj_length, adj_params.s);

            d2theta_ds2 = (ShortestAngleBetween_Radians(curr_pose.angle, next_pose.angle) / step - ShortestAngleBetween_Radians(last_pose.angle, curr_pose.angle) / step) / step;
         }

         new_samples[i].pose = curr_pose;
         new_samples[i].dtheta_ds = dtheta_ds;
         new_samples[i].d2theta_ds2 = d2theta_ds2;
      }

      //TODO: rebuild velocity map
      //TODO: output to the graph

      InterpolatingMapSamples samples = ResetMap(&plan->map);
      for(u32 i = 0; i < samples.count; i++) {
         f32 curr_s = i * step;

         AutoPathData *sample = PushStruct(samples.arena, AutoPathData);
         *sample = new_samples[i];
         samples.data[i].len = curr_s;
         samples.data[i].data_ptr = sample;
      }
      BuildMap(&plan->map);

      //TODO: reset curr_point because we just rebuilt the plan
      curr_point = GetCurrentPoint(plan, curr_pose);
      return true;
   }

   DVTA_Data dvta = GetDVTA(&plan->velocity);
   //TODO: using curr_point.s doesnt work too well when going around suuper tight turns
   f32 curr_s = false ? GetDistanceAt(dvta, time) : curr_point.s;
   // AddEntry(graph, Literal("curr_point.s"), curr_point.s, time, 0);
   // AddEntry(graph, Literal("GetDistanceAt"), GetDistanceAt(dvta, time), time, 0);

   f32 curr_v = (bot->right_v + bot->left_v) / 2.0;
   f32 curr_a = GetAccelerationAt(dvta, curr_s);
   
   f32 drivebase = bot->size.x;
   AutoPathData curr_sample = GetSampleAtS(plan, curr_s);
   bool done = false;

   if(acceleration_control) {
      bot->right_a = curr_a - (drivebase / 2) * (curr_sample.d2theta_ds2 * curr_v * curr_v + curr_sample.dtheta_ds * curr_a);
      bot->left_a = curr_a + (drivebase / 2) * (curr_sample.d2theta_ds2 * curr_v * curr_v + curr_sample.dtheta_ds * curr_a);

      done = ((plan->length - curr_s) < 0.1) && (curr_v < 0.001);
   } else {
      curr_v = GetVelocityAt(dvta, curr_s);

      bot->right_a = 0;
      bot->left_a = 0;
      bot->right_v = curr_v - (drivebase / 2) * (curr_sample.dtheta_ds * curr_v);
      bot->left_v = curr_v + (drivebase / 2) * (curr_sample.dtheta_ds * curr_v);

      done = ((plan->length - curr_s) < 0.1);
   }

   if(done) {
      bot->right_a = 0;
      bot->left_a = 0;

      //NOTE: we'll go into hold mode, so velocity should be 0
      bot->right_v = 0;
      bot->left_v = 0;
   }

   return done;
}

struct PivotPlan {
   //NOTE: Theta(t) = (clockwise ? 1 : -1) * GetDistanceAt + start_angle
   AutoVelocityDatapoints velocity;
   f32 start_angle;
   bool clockwise;
};

struct SimulatorState {
   MultiLineGraphData graph;
   MemoryArena arena;
   SimRobot state;

   f32 t;
   f32 last_s;
   bool run_sim;
   bool acceleration_control;
   
   PathPlan *path;
   PivotPlan *pivot;
};

void RunSim(SimulatorState *state, f32 dt, ui_field_topdown *field) {
   SimRobot *bot = &state->state;
   bool done = SimFollowLoop(state->path, bot, field, state->t, state->acceleration_control, &state->graph);

   f32 d_left_p = bot->left_v * dt + 0.5 * bot->left_a * dt * dt;
   bot->left_p += d_left_p;
   bot->left_v += bot->left_a * dt;
   
   f32 d_right_p = bot->right_v * dt + 0.5 * bot->right_a * dt * dt;
   bot->right_p += d_right_p;
   bot->right_v += bot->right_a * dt;

   *bot = ForwardKinematics(*bot, d_left_p, d_right_p);
   if(!Contains(RectCenterSize(V2(0, 0), field->size_in_ft), bot->pos)) {
      bot->pos = ClampTo(bot->pos, RectCenterSize(V2(0, 0), field->size_in_ft));
      bot->right_v = 0;
      bot->left_v = 0;
   }

   state->run_sim = !done;
}