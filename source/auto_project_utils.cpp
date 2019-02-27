//NOTE: needs common, north_file_definitions & robot_profile_utils

//--------------------------------------------
struct AutoVelocityDatapoints {
   u32 datapoint_count;
   North_PathDataPoint *datapoints;
};

struct DVTA_Data {
   u32 datapoint_count;
   f32 *d;
   f32 *v;
   f32 *t;
   f32 *a;
};

DVTA_Data GetDVTA(AutoVelocityDatapoints *data) {
   Assert(data->datapoint_count >= 2);

   f32 *d = PushTempArray(f32, data->datapoint_count);
   f32 *v = PushTempArray(f32, data->datapoint_count);
   f32 *t = PushTempArray(f32, data->datapoint_count);
   f32 *a = PushTempArray(f32, data->datapoint_count - 1);

   for(u32 i = 0; i < data->datapoint_count; i++) {
      d[i] = data->datapoints[i].distance;
      v[i] = data->datapoints[i].value;
   }

   t[0] = 0;

   for(u32 i = 0; i < (data->datapoint_count - 1); i++) {
      t[1 + i] = ((2 * (d[i + 1] - d[i])) / (v[i + 1] + v[i]))  + t[i];
      a[i] = (v[i+1]*v[i+1] - v[i]*v[i]) / (2 * (d[i + 1] - d[i]));
   }
   
   DVTA_Data result = {data->datapoint_count, d, v, t, a};
   return result;
}

f32 GetTotalTime(DVTA_Data dvta) {
   return dvta.t[dvta.datapoint_count - 1];
}

f32 GetTotalLength(DVTA_Data dvta) {
   return dvta.d[dvta.datapoint_count - 1];
}

//NOTE: V(s)
f32 GetVelocityAt(DVTA_Data dvta, f32 distance) {
   distance = Clamp(0, GetTotalLength(dvta), distance);
   for(u32 i = 0; i < (dvta.datapoint_count - 1); i++) {
      if((dvta.d[i] <= distance) && (distance <= dvta.d[i+1])) {
         f32 delta_t;
         if(dvta.a[i] == 0) {
            delta_t = (distance - dvta.d[i]) / dvta.v[i];
         } else {
            delta_t = (sqrtf(2*dvta.a[i]*(distance - dvta.d[i]) + dvta.v[i]*dvta.v[i]) - dvta.v[i]) / dvta.a[i];
         }

         return dvta.v[i] + dvta.a[i]*delta_t;
      }
   }

   Assert(false);
   return 0;
}

f32 GetAccelerationAt(DVTA_Data dvta, f32 distance) {
   distance = Clamp(0, GetTotalLength(dvta), distance);
   for(u32 i = 0; i < (dvta.datapoint_count - 1); i++) {
      if((dvta.d[i] <= distance) && (distance <= dvta.d[i+1])) {
         return dvta.a[i];
      }
   }

   Assert(false);
   return 0;
}

//NOTE: T(s)
f32 GetArrivalTimeAt(DVTA_Data dvta, f32 distance) {
   distance = Clamp(0, GetTotalLength(dvta), distance);
   for(u32 i = 0; i < (dvta.datapoint_count - 1); i++) {
      if((dvta.d[i] <= distance) && (distance <= dvta.d[i+1])) {
         f32 delta_t;
         if(dvta.a[i] == 0) {
            delta_t = (distance - dvta.d[i]) / dvta.v[i];
         } else {
            delta_t = (sqrtf(2*dvta.a[i]*(distance - dvta.d[i]) + dvta.v[i]*dvta.v[i]) - dvta.v[i]) / dvta.a[i];
         }

         return dvta.t[i] + delta_t;
      }
   }

   Assert(false);
   return 0;
}

//NOTE: D(t)
f32 GetDistanceAt(DVTA_Data dvta, f32 time) {
   time = Clamp(0, GetTotalTime(dvta), time);
   for(u32 i = 0; i < (dvta.datapoint_count - 1); i++) {
      if((dvta.t[i] <= time) && (time <= dvta.t[i+1])) {
         f32 dt = time - dvta.t[i];
         return dvta.d[i] + dvta.v[i]*dt + 0.5*dvta.a[i]*dt*dt;
      }
   }

   Assert(false);
   return 0;
}
//--------------------------------------------

struct AutoContinuousEvent {
   string subsystem_name;
   string command_name;
   u32 sample_count;
   North_PathDataPoint *samples;
   bool hidden;
};

struct AutoDiscreteEvent {
   f32 distance;
   string subsystem_name;
   string command_name;
   u32 param_count;
   f32 *params;
};

struct AutoPathlikeData {
   AutoVelocityDatapoints velocity;

   u32 continuous_event_count;
   AutoContinuousEvent *continuous_events;

   u32 discrete_event_count;
   AutoDiscreteEvent *discrete_events;
};

struct AutoCommand {
   North_CommandType::type type;
   
   union {
      struct {
         string subsystem_name;
         string command_name;
         u32 param_count;
         f32 *params;
      } generic;

      struct {
         f32 duration;
      } wait;

      struct {
         f32 start_angle;
         f32 end_angle;
         
         bool turns_clockwise;

         AutoPathlikeData data;
      } pivot;
   };
};

struct AutoPath;
struct AutoNode {
   v2 pos;
   AutoPath *in_path;

   u32 command_count;
   AutoCommand **commands;
   
   u32 path_count;
   AutoPath **out_paths;
};

struct AutoRobotPose {
   v2 pos;
   f32 angle; //NOTE: in radians
};

AutoRobotPose lerp(AutoRobotPose a, f32 t, AutoRobotPose b) {
   AutoRobotPose result = {};
   result.pos = lerp(a.pos, t, b.pos);
   f32 d_theta = ShortestAngleBetween_Radians(a.angle, b.angle);
   result.angle = CanonicalizeAngle_Radians(a.angle + d_theta * t);
   return result;
}

void map_lerp_pose(InterpolatingMap_Leaf *leaf_a, InterpolatingMap_Leaf *leaf_b, 
                   f32 t, void *in_result)
{
   AutoRobotPose *result = (AutoRobotPose *) in_result;
   AutoRobotPose *a = (AutoRobotPose *) leaf_a->data_ptr;
   AutoRobotPose *b = (AutoRobotPose *) leaf_b->data_ptr;
   
   *result = lerp(*a, t, *b);
}

v2 CubicHermiteSpline(North_HermiteControlPoint a, North_HermiteControlPoint b, f32 t) {
   return CubicHermiteSpline(a.pos, a.tangent, b.pos, b.tangent, t);
}

v2 CubicHermiteSplineTangent(North_HermiteControlPoint a, North_HermiteControlPoint b, f32 t) {
   return CubicHermiteSplineTangent(a.pos, a.tangent, b.pos, b.tangent, t);
}

struct AutoPathData {
   AutoRobotPose pose;
   f32 dtheta_ds;
   f32 d2theta_ds2;
};

void path_data_lerp(InterpolatingMap_Leaf *leaf_a, InterpolatingMap_Leaf *leaf_b, 
                    f32 t, void *result_in)
{
   AutoPathData *result = (AutoPathData *) result_in;
   AutoPathData *a = (AutoPathData *) leaf_a->data_ptr;
   AutoPathData *b = (AutoPathData *) leaf_b->data_ptr;

   result->pose = lerp(a->pose, t, b->pose);
   result->dtheta_ds = lerp(a->dtheta_ds, t, b->dtheta_ds);
   result->d2theta_ds2 = lerp(a->d2theta_ds2, t, b->d2theta_ds2);
}

f32 BuildPathMap(InterpolatingMap *len_to_data, North_HermiteControlPoint *points, u32 point_count) {
   InterpolatingMapSamples samples = ResetMap(len_to_data);
   f32 step = (f32)(point_count - 1) / (f32)(samples.count - 1);
   
   f32 length = 0;
   v2 last_pos = points[0].pos;
   for(u32 i = 0; i < samples.count; i++) {
      f32 t_full = step * i;
      u32 spline_i = Clamp(0, point_count - 2, (u32) t_full);
      v2 pos = CubicHermiteSpline(points[spline_i], points[spline_i + 1], t_full - (f32)spline_i);

      AutoPathData *data = PushStruct(samples.arena, AutoPathData);
      data->pose.pos = pos;
      data->pose.angle = ToRadians(Angle(CubicHermiteSplineTangent(points[spline_i], points[spline_i + 1], t_full - (f32)spline_i)));

      length += Length(pos - last_pos);
      last_pos = pos;
      
      samples.data[i].len = length;
      samples.data[i].data_ptr = data;
   }

   for(u32 i = 0; i < samples.count; i++) {
      AutoPathData *data = ((AutoPathData *) samples.data[i].data_ptr);
      AutoRobotPose curr_pose = data->pose;

      f32 dtheta_ds = 0;
      f32 d2theta_ds2 = 0;

      if(i == 0) {
         AutoRobotPose next_pose = ((AutoPathData *) samples.data[i + 1].data_ptr)->pose;
         dtheta_ds = ShortestAngleBetween_Radians(curr_pose.angle, next_pose.angle) / step;
      } else {
         AutoRobotPose last_pose = ((AutoPathData *) samples.data[i - 1].data_ptr)->pose;
         dtheta_ds = ShortestAngleBetween_Radians(last_pose.angle, curr_pose.angle) / step;
      }

      if((i != 0) && (i != samples.count - 1)) {
         AutoRobotPose next_pose = ((AutoPathData *) samples.data[i + 1].data_ptr)->pose;
         AutoRobotPose last_pose = ((AutoPathData *) samples.data[i - 1].data_ptr)->pose;

         d2theta_ds2 = (ShortestAngleBetween_Radians(curr_pose.angle, next_pose.angle) / step - ShortestAngleBetween_Radians(last_pose.angle, curr_pose.angle) / step) / step;
      }
      
      data->dtheta_ds = dtheta_ds;
      data->d2theta_ds2 = d2theta_ds2;
   }

   BuildMap(len_to_data);
   return length;
}

struct AutoPath {   
   AutoNode *in_node;
   v2 in_tangent;
   AutoNode *out_node;
   v2 out_tangent;

   bool is_reverse;
   bool hidden;

   bool has_conditional;
   string conditional;

   AutoPathlikeData data;

   u32 control_point_count;
   North_HermiteControlPoint *control_points;

   InterpolatingMap len_to_data;
   f32 length;
};

void InitAutoPath(AutoPath *path) {
   path->len_to_data.arena = PlatformAllocArena(Megabyte(2));
   path->len_to_data.sample_exp = 7;
   path->len_to_data.lerp_callback = path_data_lerp;
}

struct AutoProjectLink {
   AutoNode *starting_node;
   f32 starting_angle;
   string name;

   AutoProjectLink *next;
};

struct AutoProjectList {
   MemoryArena arena;
   AutoProjectLink *first;
};

struct AutoPathSpline {
   u32 point_count;
   North_HermiteControlPoint *points;
};

AutoPathSpline GetAutoPathSpline(AutoPath *path, MemoryArena *arena = &__temp_arena) {
   u32 control_point_count = path->control_point_count + 2;
   North_HermiteControlPoint *control_points = PushArray(arena, North_HermiteControlPoint, control_point_count);
   control_points[0] = { path->in_node->pos, path->in_tangent };
   control_points[control_point_count - 1] = { path->out_node->pos, path->out_tangent };
   Copy(path->control_points, path->control_point_count * sizeof(North_HermiteControlPoint), control_points + 1);

   AutoPathSpline result = { control_point_count, control_points };
   return result;
}

f32 GetVelocityAt(AutoPathlikeData *data, f32 distance) {
   DVTA_Data dvta = GetDVTA(&data->velocity);
   return GetVelocityAt(dvta, distance);
}

f32 GetVelocityAt(AutoPath *path, f32 distance) {
   return GetVelocityAt(&path->data, distance); 
}

void RecalculateAutoPathLength(AutoPath *path) {
   AutoPathSpline spline = GetAutoPathSpline(path);
   path->length = BuildPathMap(&path->len_to_data, spline.points, spline.point_count);
}

void RecalculatePathlikeData(AutoPathlikeData *pathlike, f32 length) {
   Assert(pathlike->velocity.datapoint_count >= 2);
   pathlike->velocity.datapoints[0].distance = 0;
   pathlike->velocity.datapoints[0].value = 0;
   pathlike->velocity.datapoints[pathlike->velocity.datapoint_count - 1].distance = length;
   pathlike->velocity.datapoints[pathlike->velocity.datapoint_count - 1].value = 0;
   for(u32 i = 0; i < pathlike->velocity.datapoint_count; i++) {
      pathlike->velocity.datapoints[i].distance = Min(pathlike->velocity.datapoints[i].distance, length);
   }
   
   ForEachArray(j, cevent, pathlike->continuous_event_count, pathlike->continuous_events, {
      cevent->samples[0].distance = 0;
      cevent->samples[cevent->sample_count - 1].distance = length;
      for(u32 k = 0; k < cevent->sample_count; k++) {
         cevent->samples[k].distance = Min(cevent->samples[k].distance, length);
      }
   });

   ForEachArray(j, devent, pathlike->discrete_event_count, pathlike->discrete_events, {
      devent->distance = Clamp(0, length, devent->distance);
   });
}

void RecalculateAutoPath(AutoPath *path) {
   RecalculateAutoPathLength(path);
   RecalculatePathlikeData(&path->data, path->length);
}

AutoRobotPose GetAutoPathPose(AutoPath *path, f32 distance) {
   AutoPathData data = {};
   MapLookup(&path->len_to_data, distance, &data);
   return data.pose;
}

v2 GetAutoPathPoint(AutoPath *path, f32 distance) {
   return GetAutoPathPose(path, distance).pos;
}

void RecalculateAutoNode(AutoNode *node) {
   if(node->in_path != NULL) {
      f32 start_angle = Angle(node->in_path->out_tangent);
      for(u32 i = 0; i < node->command_count; i++) {
         AutoCommand *command = node->commands[i];
         if(command->type == North_CommandType::Pivot) {
            command->pivot.start_angle = start_angle;
            f32 pivot_length = abs(AngleBetween(command->pivot.start_angle, command->pivot.end_angle, command->pivot.turns_clockwise));

            RecalculatePathlikeData(&command->pivot.data, pivot_length);
            
            start_angle = command->pivot.end_angle;
         }
      }
   }

   for(u32 i = 0; i < node->path_count; i++) {
      RecalculateAutoNode(node->out_paths[i]->out_node);
   }
}

//TODO: return an error message instead of just a bool
bool IsPathlikeDataCompatible(AutoPathlikeData *data, RobotProfile *bot) {
   for(u32 i = 0; i < data->continuous_event_count; i++) {
      AutoContinuousEvent *cevent = data->continuous_events + i; 
      RobotProfileCommand *command = GetCommand(bot, cevent->subsystem_name, cevent->command_name);
      if(command == NULL) {
         return false;
      } else if(command->type != North_CommandExecutionType::Continuous) {
         return false;
      }
   }

   for(u32 i = 0; i < data->discrete_event_count; i++) {
      AutoDiscreteEvent *devent = data->discrete_events + i; 
      RobotProfileCommand *command = GetCommand(bot, devent->subsystem_name, devent->command_name);
      if(command == NULL) {
         return false;
      } else if((command->type != North_CommandExecutionType::NonBlocking) ||
                (command->param_count != devent->param_count))
      {
         return false;
      }
   }

   return true;
}

bool IsNodeCompatible(AutoNode *node, RobotProfile *bot) {
   for(u32 i = 0; i < node->command_count; i++) {
      AutoCommand *command = node->commands[i];
      if(command->type == North_CommandType::Generic) {
         if(GetCommand(bot, command->generic.subsystem_name, command->generic.command_name) == NULL) {
            return false;
         }
      } else if(command->type == North_CommandType::Pivot) {
         if(!IsPathlikeDataCompatible(&command->pivot.data, bot)) {
            return false;
         }
      }
   }

   for(u32 i = 0; i < node->path_count; i++) {
      AutoPath *path = node->out_paths[i];
      
      if(!IsPathlikeDataCompatible(&path->data, bot) || 
         !IsNodeCompatible(path->out_node, bot))
      {
         return false;
      }
   }

   return true;
}

bool IsProjectCompatible(AutoProjectLink *proj, RobotProfile *bot) {
   return IsNodeCompatible(proj->starting_node, bot);
}

//FILE-READING----------------------------------------------
AutoContinuousEvent ParseAutoContinuousEvent(buffer *file, MemoryArena *arena) {
   AutoContinuousEvent result = {};
   AutonomousProgram_ContinuousEvent *file_event = ConsumeStruct(file, AutonomousProgram_ContinuousEvent);
   
   result.subsystem_name = PushCopy(arena, ConsumeString(file, file_event->subsystem_name_length));
   result.command_name = PushCopy(arena, ConsumeString(file, file_event->command_name_length));
   result.sample_count = file_event->datapoint_count;
   result.samples = ConsumeAndCopyArray(arena, file, North_PathDataPoint, file_event->datapoint_count);
   
   return result;
}

AutoDiscreteEvent ParseAutoDiscreteEvent(buffer *file, MemoryArena *arena) {
   AutoDiscreteEvent result = {};
   AutonomousProgram_DiscreteEvent *file_event = ConsumeStruct(file, AutonomousProgram_DiscreteEvent);
            
   result.subsystem_name = PushCopy(arena, ConsumeString(file, file_event->subsystem_name_length));
   result.command_name = PushCopy(arena, ConsumeString(file, file_event->command_name_length));
   result.param_count = file_event->parameter_count;
   result.params = ConsumeAndCopyArray(arena, file, f32, file_event->parameter_count);
   result.distance = file_event->distance;

   return result;
}

AutoCommand *ParseAutoCommand(buffer *file, MemoryArena *arena) {
   AutoCommand *result = PushStruct(arena, AutoCommand);
   
   AutonomousProgram_CommandHeader *header = ConsumeStruct(file, AutonomousProgram_CommandHeader);
   result->type = (North_CommandType::type) header->type;
   
   switch(result->type) {
      case North_CommandType::Generic: {
         AutonomousProgram_CommandBody_Generic *body = ConsumeStruct(file, AutonomousProgram_CommandBody_Generic);
         result->generic.subsystem_name = PushCopy(arena, ConsumeString(file, body->subsystem_name_length));
         result->generic.command_name = PushCopy(arena, ConsumeString(file, body->command_name_length));
         result->generic.param_count = body->parameter_count;
         result->generic.params = ConsumeAndCopyArray(arena, file, f32, body->parameter_count);
      } break;

      case North_CommandType::Wait: {
         AutonomousProgram_CommandBody_Wait *body = ConsumeStruct(file, AutonomousProgram_CommandBody_Wait);
         result->wait.duration = body->duration;
      } break;

      case North_CommandType::Pivot: {
         AutonomousProgram_CommandBody_Pivot *body = ConsumeStruct(file, AutonomousProgram_CommandBody_Pivot);
         result->pivot.start_angle = body->start_angle;
         result->pivot.end_angle = body->end_angle;
         result->pivot.turns_clockwise = body->turns_clockwise ? true : false;

         result->pivot.data.velocity.datapoint_count = body->velocity_datapoint_count;
         Assert(result->pivot.data.velocity.datapoint_count >= 2);
         result->pivot.data.velocity.datapoints = ConsumeAndCopyArray(arena, file, North_PathDataPoint, body->velocity_datapoint_count);

         result->pivot.data.continuous_event_count = body->continuous_event_count;
         result->pivot.data.continuous_events = PushArray(arena, AutoContinuousEvent, body->continuous_event_count);
         for(u32 i = 0; i < body->continuous_event_count; i++) {
            result->pivot.data.continuous_events[i] = ParseAutoContinuousEvent(file, arena);
         }

         result->pivot.data.discrete_event_count = body->discrete_event_count;
         result->pivot.data.discrete_events = PushArray(arena, AutoDiscreteEvent, body->discrete_event_count);
         for(u32 i = 0; i < body->discrete_event_count; i++) {
            result->pivot.data.discrete_events[i] = ParseAutoDiscreteEvent(file, arena);
         }
      } break;

      default: Assert(false);
   }

   return result;
}

AutoPath *ParseAutoPath(buffer *file, MemoryArena *arena);
AutoNode *ParseAutoNode(buffer *file, MemoryArena *arena) {
   AutoNode *result = PushStruct(arena, AutoNode);
   AutonomousProgram_Node *file_node = ConsumeStruct(file, AutonomousProgram_Node);
   
   result->pos = file_node->pos;
   result->path_count = file_node->path_count;
   result->out_paths = PushArray(arena, AutoPath *, file_node->path_count);
   result->command_count = file_node->command_count;
   result->commands = PushArray(arena, AutoCommand *, result->command_count);

   for(u32 i = 0; i < file_node->command_count; i++) {
      result->commands[i] = ParseAutoCommand(file, arena);
   }
   
   for(u32 i = 0; i < file_node->path_count; i++) {
      AutoPath *path = ParseAutoPath(file, arena);
      path->in_node = result;
      RecalculateAutoPath(path);

      result->out_paths[i] = path;
   }

   return result;
}

AutoPath *ParseAutoPath(buffer *file, MemoryArena *arena) {
   AutoPath *path = PushStruct(arena, AutoPath);
   AutonomousProgram_Path *file_path = ConsumeStruct(file, AutonomousProgram_Path);
   
   path->in_tangent = file_path->in_tangent;
   path->out_tangent = file_path->out_tangent;
   path->is_reverse = file_path->is_reverse ? true : false;

   if(file_path->conditional_length == 0) {
      path->conditional = EMPTY_STRING;
      path->has_conditional = false;
   } else {
      path->conditional =  PushCopy(arena, ConsumeString(file, file_path->conditional_length));
      path->has_conditional = true;
   }

   path->control_point_count = file_path->control_point_count;
   path->control_points = ConsumeAndCopyArray(arena, file, North_HermiteControlPoint, file_path->control_point_count);
   
   path->data.velocity.datapoint_count = file_path->velocity_datapoint_count;
   Assert(path->data.velocity.datapoint_count >= 2);
   path->data.velocity.datapoints = ConsumeAndCopyArray(arena, file, North_PathDataPoint, file_path->velocity_datapoint_count);

   path->data.continuous_event_count = file_path->continuous_event_count;
   path->data.continuous_events = PushArray(arena, AutoContinuousEvent, path->data.continuous_event_count);
   for(u32 i = 0; i < file_path->continuous_event_count; i++) {
      path->data.continuous_events[i] = ParseAutoContinuousEvent(file, arena);
   }

   path->data.discrete_event_count = file_path->discrete_event_count;
   path->data.discrete_events = PushArray(arena, AutoDiscreteEvent, path->data.discrete_event_count);
   for(u32 i = 0; i < file_path->discrete_event_count; i++) {
      path->data.discrete_events[i] = ParseAutoDiscreteEvent(file, arena);
   }

   InitAutoPath(path);
   
   path->out_node = ParseAutoNode(file, arena);
   path->out_node->in_path = path;
   return path;
}

AutoProjectLink *ReadAutoProject(string file_name, MemoryArena *arena) {
   buffer file = ReadEntireFile(Concat(file_name, Literal(".ncap")));
   if(file.data != NULL) {
      FileHeader *file_numbers = ConsumeStruct(&file, FileHeader);
      AutonomousProgram_FileHeader *header = ConsumeStruct(&file, AutonomousProgram_FileHeader);

      AutoProjectLink *result = PushStruct(arena, AutoProjectLink);
      result->name = PushCopy(arena, file_name);
      result->starting_angle = header->starting_angle;
      result->starting_node = ParseAutoNode(&file, arena);
      return result;
   }

   return NULL;
}

void ReadProjectsStartingAt(AutoProjectList *list, u32 field_flags, v2 pos, RobotProfile *bot) {
   Reset(&list->arena);
   list->first = NULL;

   for(FileListLink *file = ListFilesWithExtension("*.ncap"); file; file = file->next) {
      AutoProjectLink *auto_proj = ReadAutoProject(file->name, &list->arena);
      if(auto_proj && IsProjectCompatible(auto_proj, bot)) {
         //TODO: do reflecting and stuff in here
         bool valid = false;

         if(Length(auto_proj->starting_node->pos - pos) < 0.5) {
            valid = true;
         }

         if(valid) {
            auto_proj->next = list->first;
            list->first = auto_proj;
         }
      }
   }
}
//FILE-READING----------------------------------------------

//FILE-WRITING----------------------------------------------
void WriteAutoContinuousEvent(buffer *file, AutoContinuousEvent *event) {
   WriteStructData(file, AutonomousProgram_ContinuousEvent, event_header, {
      event_header.subsystem_name_length = event->subsystem_name.length;
      event_header.command_name_length = event->command_name.length;
      event_header.datapoint_count = event->sample_count;
   });
   WriteString(file, event->subsystem_name);
   WriteString(file, event->command_name);
   WriteArray(file, event->samples, event->sample_count);
}

void WriteAutoDiscreteEvent(buffer *file, AutoDiscreteEvent *event) {
   WriteStructData(file, AutonomousProgram_DiscreteEvent, event_header, {
      event_header.distance = event->distance;
      event_header.subsystem_name_length = event->subsystem_name.length;
      event_header.command_name_length = event->command_name.length;
      event_header.parameter_count = event->param_count;
   });
   WriteString(file, event->subsystem_name);
   WriteString(file, event->command_name);
   WriteArray(file, event->params, event->param_count);
}

void WriteAutoCommand(buffer *file, AutoCommand *command) {
   WriteStructData(file, AutonomousProgram_CommandHeader, header, {
      header.type = (u8) command->type;
   });
   
   switch(command->type) {
      case North_CommandType::Generic: {
         WriteStructData(file, AutonomousProgram_CommandBody_Generic, body, {
            body.subsystem_name_length = command->generic.subsystem_name.length;
            body.command_name_length = command->generic.command_name.length;
            body.parameter_count = command->generic.param_count;
         });
         WriteString(file, command->generic.subsystem_name);
         WriteString(file, command->generic.command_name);
         WriteArray(file, command->generic.params, command->generic.param_count);
      } break;

      case North_CommandType::Wait: {
         WriteStructData(file, AutonomousProgram_CommandBody_Wait, body, {
            body.duration = command->wait.duration;
         });
      } break;

      case North_CommandType::Pivot: {
         WriteStructData(file, AutonomousProgram_CommandBody_Pivot, body, {
            body.start_angle = command->pivot.start_angle;
            body.end_angle = command->pivot.end_angle;
            body.turns_clockwise = command->pivot.turns_clockwise;
            
            body.velocity_datapoint_count = command->pivot.data.velocity.datapoint_count;
            body.continuous_event_count = command->pivot.data.continuous_event_count;
            body.discrete_event_count = command->pivot.data.discrete_event_count;
         });
         
         WriteArray(file, command->pivot.data.velocity.datapoints, command->pivot.data.velocity.datapoint_count);
         ForEachArray(i, event, command->pivot.data.continuous_event_count, command->pivot.data.continuous_events, {
            WriteAutoContinuousEvent(file, event);
         });
         ForEachArray(i, event, command->pivot.data.discrete_event_count, command->pivot.data.discrete_events, {
            WriteAutoDiscreteEvent(file, event);
         });
      } break;
   }
}

void WriteAutoPath(buffer *file, AutoPath *path);
void WriteAutoNode(buffer *file, AutoNode *node) {
   WriteStructData(file, AutonomousProgram_Node, node_header, {
      node_header.pos = node->pos;
      node_header.command_count = node->command_count;
      node_header.path_count = node->path_count;
   });

   ForEachArray(i, command, node->command_count, node->commands, {
      WriteAutoCommand(file, *command);
   });

   ForEachArray(i, path, node->path_count, node->out_paths, {
      WriteAutoPath(file, *path);
   });
}

void WriteAutoPath(buffer *file, AutoPath *path) {
   WriteStructData(file, AutonomousProgram_Path, path_header, {
      path_header.in_tangent = path->in_tangent;
      path_header.out_tangent = path->out_tangent;
      path_header.is_reverse = path->is_reverse ? 1 : 0;
      
      path_header.conditional_length = path->has_conditional ? path->conditional.length : 0;
      path_header.control_point_count = path->control_point_count;

      path_header.velocity_datapoint_count = path->data.velocity.datapoint_count;
      path_header.continuous_event_count = path->data.continuous_event_count;
      path_header.discrete_event_count = path->data.discrete_event_count;
   });

   if(path->has_conditional)
      WriteString(file, path->conditional);
   
   WriteArray(file, path->control_points, path->control_point_count);
   
   WriteArray(file, path->data.velocity.datapoints, path->data.velocity.datapoint_count);
   ForEachArray(i, event, path->data.continuous_event_count, path->data.continuous_events, {
      WriteAutoContinuousEvent(file, event);
   });
   ForEachArray(i, event, path->data.discrete_event_count, path->data.discrete_events, {
      WriteAutoDiscreteEvent(file, event);
   });

   WriteAutoNode(file, path->out_node);
}

void WriteProject(AutoProjectLink *project) {
   buffer file = PushTempBuffer(Megabyte(10));
   
   FileHeader file_numbers = header(AUTONOMOUS_PROGRAM_MAGIC_NUMBER, AUTONOMOUS_PROGRAM_CURR_VERSION);
   WriteStruct(&file, &file_numbers);
   
   AutonomousProgram_FileHeader header = {};
   header.starting_angle = project->starting_angle;
   WriteStruct(&file, &header);

   WriteAutoNode(&file, project->starting_node);
   WriteEntireFile(Concat(project->name, Literal(".ncap")), file);
}
//FILE-WRITING----------------------------------------------

//TODO: AutoProjectLink to packet