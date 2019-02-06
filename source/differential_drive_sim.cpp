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
   f32 left_setpoint;

   f32 right_p;
   f32 right_v;
   f32 right_setpoint;
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

struct PathPlanSample {
   AutoRobotPose pose;
   f32 time;
   f32 left_pos;
   f32 left_vel;
   f32 right_pos;
   f32 right_vel;
};

void path_plan_sample_lerp(InterpolatingMap_Leaf *leaf_a, InterpolatingMap_Leaf *leaf_b, 
                           f32 t, void *result_in)
{
   PathPlanSample *result = (PathPlanSample *) result_in;
   PathPlanSample *a = (PathPlanSample *) leaf_a->data_ptr;
   PathPlanSample *b = (PathPlanSample *) leaf_b->data_ptr;

   result->pose = lerp(a->pose, t, b->pose);
   result->time = lerp(a->time, t, b->time);
   result->left_pos = lerp(a->left_pos, t, b->left_pos);
   result->left_vel = lerp(a->left_vel, t, b->left_vel);
   result->right_pos = lerp(a->right_pos, t, b->right_pos);
   result->right_vel = lerp(a->right_vel, t, b->right_vel);
}

struct PathPlan {
   AutoVelocityDatapoints velocity;

   f32 length;
   f32 time;
   InterpolatingMap map; //NOTE: f32 -> PathPlanSample
};

struct PathPoint {
   bool on_path;
   v2 point;
   f32 s;
};

const f32 min_path_dist = 0.5;
const f32 min_path_heading_diff = ToRadians(10);

PathPlanSample GetSampleAtS(PathPlan *plan, f32 s) {
   PathPlanSample sample = {};
   MapLookup(&plan->map, s, &sample);
   return sample;
}

AutoRobotPose GetPoseAtS(PathPlan *plan, f32 s) {
   return GetSampleAtS(plan, s).pose;
}

f32 GetDistAtS(PathPlan *plan, f32 s, v2 pos) {
   return Length(GetPoseAtS(plan, s).pos - pos);
}

PathPoint GetCurrentPoint(PathPlan *plan, AutoRobotPose pose) {
   f32 closest_dist = F32_MAX;
   f32 closest_s = 0;
   
   u32 sample_count = 40;
   f32 step = plan->length / (sample_count - 1);
   for(u32 i = 0; i < sample_count; i++) {
      f32 curr_s = i * step;
      PathPlanSample sample = {};
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

//TODO: make this better?
f32 LookaheadDifficulty(PathPlan *plan, f32 from, f32 to, f32 drivebase) {
   PathPlanSample start_sample = GetSampleAtS(plan, from);
   PathPlanSample end_sample = GetSampleAtS(plan, to);

   f32 d_left_p = end_sample.left_pos - start_sample.left_pos;
   f32 d_right_p = end_sample.right_pos - start_sample.right_pos;

   u32 path_sample_count = 20;
   f32 path_step = plan->length / (path_sample_count - 1);
   AutoRobotPose *path_samples = PushTempArray(AutoRobotPose, path_sample_count);
   for(u32 i = 0; i < path_sample_count; i++) {
      path_samples[i] = GetPoseAtS(plan, path_step * i);
   }

   u32 sample_count = 20;
   f32 step = 1.0 / (sample_count - 1);

   f32 result = 0;
   for(u32 i = 0; i < sample_count; i++) {
      f32 percent = step * i;
      v2 new_pos = ForwardKinematics(start_sample.pose, drivebase, d_right_p * percent, d_left_p * percent).pos;
      f32 min_dist = F32_MAX;

      for(u32 j = 0; j < path_sample_count; j++) {
         min_dist = Min(min_dist, Length(new_pos - path_samples[j].pos));
      }
      
      result += min_dist;
   }

   return result;
}

f32 GetSTarget(PathPlan *plan, SimRobot bot, f32 bot_s, f32 time) {
   f32 min_s_target = bot_s;
   DVTA_Data dvta = GetDVTA(&plan->velocity);
   f32 max_s_target = Clamp(min_s_target, plan->length, GetDistanceAt(dvta, time));
   f32 s_target = 0;
   
   //TODO: find a reasonable value for this
   f32 max_lookahead_difficulty = 5;
   
   u32 sample_count = 10;
   f32 step = (max_s_target - min_s_target) / (sample_count - 1);

   for(u32 i = 0; i < sample_count; i++) {
      f32 curr_s = min_s_target + step * i;
      if(LookaheadDifficulty(plan, bot_s, curr_s, bot.size.x) < max_lookahead_difficulty) {
         s_target = curr_s;
      } else {
         break;
      }
   }
   
   return s_target;
}

void SimFollowLoop(PathPlan *plan, SimRobot *bot, ui_field_topdown *field, f32 time, f32 manual_s_target, bool use_manual_target) {
   AutoRobotPose curr_pose = { bot->pos, bot->angle };
   PathPoint curr_point = GetCurrentPoint(plan, curr_pose);
   
   if(curr_point.on_path) {
      f32 s_target = use_manual_target ? manual_s_target : GetSTarget(plan, *bot, curr_point.s, time);
      
      PathPlanSample target_sample = {};
      MapLookup(&plan->map, s_target, &target_sample);
      Rectangle(field->e, RectCenterSize(GetPoint(field, target_sample.pose.pos), V2(5, 5)), BLACK);
      
      bot->left_setpoint = target_sample.left_pos;
      bot->right_setpoint = target_sample.right_pos;
   } else {
      AdjustmentPathParams adj_params = OptimizeAdjustmentPath(plan, curr_pose);
      
      North_HermiteControlPoint A = { bot->pos, adj_params.a * DirectionNormal(ToDegrees(bot->angle)) };
      AutoRobotPose BPose = GetPoseAtS(plan, adj_params.s);
      North_HermiteControlPoint B = { BPose.pos, adj_params.b * DirectionNormal(ToDegrees(BPose.angle)) };

      CubicHermiteSpline(field, A.pos, A.tangent, B.pos, B.tangent, BLACK, 2);
      f32 adj_length = 0; //TODO: calculate
      f32 original_path_length = plan->length - adj_params.s;
      f32 total_length = adj_length + original_path_length;

      TempArena temp;
      u32 sample_count = Power(2, plan->map.sample_exp);
      PathPlanSample *new_samples = PushArray(&temp.arena, PathPlanSample, sample_count);
      f32 step = total_length / (sample_count - 1);

      for(u32 i = 0; i < sample_count; i++) {
         f32 curr_s = i * step;
         if(curr_s <= adj_length) {
            //in adj path
         } else {
            f32 path_s = (curr_s - adj_length) + adj_params.s;
            //in existing path
         }
      }

      //TODO: generate samples & build adjusted path
      // InterpolatingMapSamples samples = ResetMap(&plan->map);
      // BuildMap(&plan->map);
   }
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
   bool run_sim;
   f32 kp;
   f32 drag;
   
   PathPlan *path;
   f32 target_s;
   bool use_manual_target_s;

   PivotPlan *pivot;
};

//TODO: clean this up its such a mess
f32 GetTorque(f32 free_speed, f32 stall_torque, f32 speed, f32 voltage) {
   voltage = Clamp(-12.0, 12.0, voltage);
   f32 percent_v = abs(voltage) / 12.0;
   speed = abs((voltage > 0) ? Clamp(0, free_speed * percent_v, speed) : Clamp(-free_speed * percent_v, 0, speed));
   return Sign(voltage) * ( stall_torque * percent_v - (stall_torque / free_speed) * speed );
}

void RunSim(SimulatorState *state, f32 dt, ui_field_topdown *field) {
   SimRobot *bot = &state->state;
   SimFollowLoop(state->path, bot, field, state->t, state->target_s, state->use_manual_target_s);

   f32 s_free = 16;
   f32 t_stall = 40;

   f32 left_voltage = state->kp * (bot->left_setpoint - bot->left_p);
   f32 left_accel = GetTorque(s_free, t_stall, bot->left_v, left_voltage);
   f32 d_left_p = bot->left_v * dt + 0.5 * left_accel * dt * dt;
   bot->left_p += d_left_p;
   bot->left_v += left_accel * dt;
   
   f32 right_voltage = state->kp * (bot->right_setpoint - bot->right_p);
   f32 right_accel = GetTorque(s_free, t_stall, bot->right_v, right_voltage);
   f32 d_right_p = bot->right_v * dt + 0.5 * right_accel * dt * dt;
   bot->right_p += d_right_p;
   bot->right_v += right_accel * dt;

   //TODO: this drag/friction stuff is totally wrong
   bot->left_v -= (bot->left_v > 0 ? 1 : -1) * state->drag * dt;
   bot->right_v -= (bot->right_v > 0 ? 1 : -1) * state->drag * dt;

   *bot = ForwardKinematics(*bot, d_left_p, d_right_p);
   bot->pos = ClampTo(bot->pos, RectCenterSize(V2(0, 0), field->size_in_ft));
}