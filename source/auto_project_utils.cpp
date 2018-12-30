//NOTE: needs common & north_file_definitions

//TODO: redo this to use: 
//    file linking
//    starting point match & reflect

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
   u32 velocity_datapoint_count;
   North_PathDataPoint *velocity_datapoints;

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
         
         // bool path_tangent_end_angle;
         // union {
            f32 end_angle; //NOTE: path_tangent_end_angle == false
            // u32 path_index; //NOTE: path_tangent_end_angle == true
         // };
         
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

struct AutoPath_LenLeaf {
   f32 len;
   v2 pos;
};

struct AutoPath_LenBranch {
   f32 len;

   f32 greater_value;
   f32 less_value;

   union {
      struct {
         u32 greater_leaf;
         u32 less_leaf;
      };

      struct {
         AutoPath_LenBranch *greater_branch;
         AutoPath_LenBranch *less_branch;
      };
   };
};

struct AutoPath_TimeLeaf {
   f32 time;
   f32 len;
};

struct AutoPath_TimeBranch {
   f32 time;

   f32 greater_value;
   f32 less_value;

   union {
      struct {
         u32 greater_leaf;
         u32 less_leaf;
      };

      struct {
         AutoPath_TimeBranch *greater_branch;
         AutoPath_TimeBranch *less_branch;
      };
   };
};

//NOTE: this means we have 2^sample_exp samples
//TODO: the (dist -> pos) map is bigger than the (time -> dist) map, thats why this works
const u32 sample_exp = 7;
u64 GetMapMemorySize() {
   u64 result = Power(2, sample_exp) * sizeof(AutoPath_LenLeaf);
   for(u32 i = 0; i < sample_exp; i++) {
      result += Power(2, i) * sizeof(AutoPath_LenBranch);
   }
   return result;
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

   //-------------------------------------
   MemoryArena length_to_pos_memory;
   f32 length;
   AutoPath_LenBranch *len_root;
   AutoPath_LenBranch *len_layers[sample_exp];
   AutoPath_LenLeaf *len_samples;

   MemoryArena time_to_length_memory;
   f32 time;
   AutoPath_TimeBranch *time_root;
   AutoPath_TimeLeaf *time_samples;
};

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

v2 CubicHermiteSpline(North_HermiteControlPoint a, North_HermiteControlPoint b, f32 t) {
   return CubicHermiteSpline(a.pos, a.tangent, b.pos, b.tangent, t);
}

v2 CubicHermiteSplineTangent(North_HermiteControlPoint a, North_HermiteControlPoint b, f32 t) {
   return CubicHermiteSplineTangent(a.pos, a.tangent, b.pos, b.tangent, t);
}

void RecalculateAutoPathLength(AutoPath *path) {
   MemoryArena *arena = &path->length_to_pos_memory;
   Reset(arena);

   u32 sample_count = Power(2, sample_exp); 
   AutoPath_LenLeaf *samples = PushArray(arena, AutoPath_LenLeaf, sample_count);
   AutoPathSpline spline = GetAutoPathSpline(path);
   f32 step = (f32)(spline.point_count - 1) / (f32)(sample_count - 1);
   
   samples[0].len = 0;
   samples[0].pos = spline.points[0].pos;

   f32 length = 0;
   v2 last_pos = spline.points[0].pos;
   for(u32 i = 1; i < sample_count; i++) {
      f32 t_full = step * i;
      u32 spline_i = Clamp(0, spline.point_count - 2, (u32) t_full);
      v2 pos = CubicHermiteSpline(spline.points[spline_i], spline.points[spline_i + 1], t_full - (f32)spline_i);

      length += Length(pos - last_pos);
      last_pos = pos;
      
      samples[i].len = length;
      samples[i].pos = pos;
   }
   path->len_samples = samples;
   path->length = length;

   u32 last_layer_count = Power(2, sample_exp - 1);
   AutoPath_LenBranch *last_layer = PushArray(arena, AutoPath_LenBranch, last_layer_count);
   for(u32 i = 0; i < last_layer_count; i++) {
      AutoPath_LenLeaf *less = samples + (2 * i);
      AutoPath_LenLeaf *greater = samples + (2 * i + 1);

      last_layer[i].less_leaf = 2 * i;
      last_layer[i].less_value = less->len;
      last_layer[i].greater_leaf = 2 * i + 1;
      last_layer[i].greater_value = greater->len;
      last_layer[i].len = (less->len + greater->len) / 2;
   }
   
   u32 debug_layer_i = 0;
   path->len_layers[debug_layer_i++] = last_layer;

   for(u32 i = 1; i < sample_exp; i++) {
      u32 layer_count = Power(2, sample_exp - i - 1);
      AutoPath_LenBranch *curr_layer = PushArray(arena, AutoPath_LenBranch, layer_count);

      for(u32 i = 0; i < layer_count; i++) {
         AutoPath_LenBranch *less = last_layer + (2 * i);
         AutoPath_LenBranch *greater = last_layer + (2 * i + 1);

         curr_layer[i].less_branch = less;
         curr_layer[i].less_value = less->less_value;
         curr_layer[i].greater_branch = greater;
         curr_layer[i].greater_value = greater->greater_value;
         curr_layer[i].len = (less->greater_value + greater->less_value) / 2;
      }

      last_layer_count = layer_count;
      last_layer = curr_layer;
      path->len_layers[debug_layer_i++] = last_layer;
   }

   path->len_root = last_layer;
}

f32 GetVelocityAt(AutoPath *path, f32 distance) {
   for(u32 i = 1; i < path->data.velocity_datapoint_count; i++) {
      North_PathDataPoint a = path->data.velocity_datapoints[i - 1];
      North_PathDataPoint b = path->data.velocity_datapoints[i];
      if((a.distance <= distance) && (distance <= b.distance)) {
         f32 t = (distance - a.distance) / (b.distance - a.distance);
         return lerp(a.value, t, b.value);
      }
   }

   return 0;
}

void RecalculateAutoPathTime(AutoPath *path) {
   Assert(path->data.velocity_datapoint_count >= 2);
   path->data.velocity_datapoints[0].distance = 0;
   path->data.velocity_datapoints[0].value = 0;
   path->data.velocity_datapoints[path->data.velocity_datapoint_count - 1].distance = path->length;
   path->data.velocity_datapoints[path->data.velocity_datapoint_count - 1].value = 0;
   for(u32 i = 0; i < path->data.velocity_datapoint_count; i++) {
      path->data.velocity_datapoints[i].distance = Min(path->data.velocity_datapoints[i].distance, path->length);
   }

   MemoryArena *arena = &path->time_to_length_memory;
   Reset(arena);

   u32 sample_count = Power(2, sample_exp); 
   AutoPath_TimeLeaf *samples = PushArray(arena, AutoPath_TimeLeaf, sample_count);
   f32 ds = path->length / (f32)sample_count;

   f32 time = 0;
   for(u32 i = 1; i < (sample_count - 1); i++) {
      time += (1 / GetVelocityAt(path, ds * i)) * ds;

      samples[i].time = time;
      samples[i].len = ds * i;
   }
   path->time = time;

   //TODO: last sample is incorrect
   samples[sample_count - 1].time = samples[sample_count - 2].time;
   samples[sample_count - 1].len = path->length;;
   
   u32 last_layer_count = Power(2, sample_exp - 1);
   AutoPath_TimeBranch *last_layer = PushArray(arena, AutoPath_TimeBranch, last_layer_count);
   for(u32 i = 0; i < last_layer_count; i++) {
      AutoPath_TimeLeaf *less = samples + (2 * i);
      AutoPath_TimeLeaf *greater = samples + (2 * i + 1);

      last_layer[i].less_leaf = 2 * i;
      last_layer[i].less_value = less->time;
      last_layer[i].greater_leaf = 2 * i + 1;
      last_layer[i].greater_value = greater->time;
      last_layer[i].time = (less->time + greater->time) / 2;
   }

   for(u32 i = 1; i < sample_exp; i++) {
      u32 layer_count = Power(2, sample_exp - i - 1);
      AutoPath_TimeBranch *curr_layer = PushArray(arena, AutoPath_TimeBranch, layer_count);

      for(u32 i = 0; i < layer_count; i++) {
         AutoPath_TimeBranch *less = last_layer + (2 * i);
         AutoPath_TimeBranch *greater = last_layer + (2 * i + 1);

         curr_layer[i].less_branch = less;
         curr_layer[i].less_value = less->less_value;
         curr_layer[i].greater_branch = greater;
         curr_layer[i].greater_value = greater->greater_value;
         curr_layer[i].time = (less->greater_value + greater->less_value) / 2;
      }

      last_layer_count = layer_count;
      last_layer = curr_layer;
   }

   path->time_root = last_layer;
}

void RecalculateAutoPath(AutoPath *path) {
   RecalculateAutoPathLength(path);
   RecalculateAutoPathTime(path);

   ForEachArray(j, cevent, path->data.continuous_event_count, path->data.continuous_events, {
      cevent->samples[0].distance = 0;
      cevent->samples[cevent->sample_count - 1].distance = path->length;
      for(u32 k = 0; k < cevent->sample_count; k++) {
         cevent->samples[k].distance = Min(cevent->samples[k].distance, path->length);
      }
   });

   ForEachArray(j, devent, path->data.discrete_event_count, path->data.discrete_events, {
      devent->distance = Clamp(0, path->length, devent->distance);
   });
}

//TODO: fix the two below
v2 GetAutoPathPoint(AutoPath *path, f32 distance) {
   u32 sample_count = Power(2, sample_exp);
   AutoPath_LenBranch *branch = path->len_root;
   for(u32 i = 0; i < (sample_exp - 1); i++) {
      branch = (distance > branch->len) ? branch->greater_branch : branch->less_branch;
   }

   u32 a_i;
   u32 b_i;

   u32 center_leaf_i = (distance > branch->len) ? branch->greater_leaf : branch->less_leaf;
   if(distance > path->len_samples[center_leaf_i].len) {
      a_i = Clamp(0, sample_count - 1, center_leaf_i);
      b_i = Clamp(0, sample_count - 1, center_leaf_i + 1);
   } else {
      a_i = Clamp(0, sample_count - 1, center_leaf_i - 1);
      b_i = Clamp(0, sample_count - 1, center_leaf_i);
   }

   AutoPath_LenLeaf *leaf_a = path->len_samples + a_i;
   AutoPath_LenLeaf *leaf_b = path->len_samples + b_i;

   f32 t = (distance - leaf_a->len) / (leaf_b->len - leaf_a->len); 
   return lerp(leaf_a->pos, t, leaf_b->pos);
}

f32 GetAutoPathDistForTime(AutoPath *path, f32 time) {
   u32 sample_count = Power(2, sample_exp);
   AutoPath_TimeBranch *branch = path->time_root;
   for(u32 i = 0; i < (sample_exp - 1); i++) {
      branch = (time > branch->time) ? branch->greater_branch : branch->less_branch;
   }

   u32 a_i;
   u32 b_i;

   u32 center_leaf_i = (time > branch->time) ? branch->greater_leaf : branch->less_leaf;
   if(time > path->time_samples[center_leaf_i].time) {
      a_i = Clamp(0, sample_count - 1, center_leaf_i);
      b_i = Clamp(0, sample_count - 1, center_leaf_i + 1);
   } else {
      a_i = Clamp(0, sample_count - 1, center_leaf_i - 1);
      b_i = Clamp(0, sample_count - 1, center_leaf_i);
   }

   AutoPath_TimeLeaf *leaf_a = path->time_samples + a_i;
   AutoPath_TimeLeaf *leaf_b = path->time_samples + b_i;

   f32 t = (time - leaf_a->time) / (leaf_b->time - leaf_a->time); 
   return lerp(leaf_a->time, t, leaf_b->time);
}

void RecalculateAutoNode(AutoNode *node) {
   if(node->in_path != NULL) {
      f32 start_angle = Angle(node->in_path->out_tangent);
      for(u32 i = 0; i < node->command_count; i++) {
         AutoCommand *command = node->commands[i];
         if(command->type == North_CommandType::Pivot) {
            command->pivot.start_angle = start_angle;
            f32 pivot_length = abs(AngleBetween(command->pivot.start_angle, command->pivot.end_angle, command->pivot.turns_clockwise));

            Assert(command->pivot.data.velocity_datapoint_count >= 2);
            command->pivot.data.velocity_datapoints[0].distance = 0;
            command->pivot.data.velocity_datapoints[0].value = 0;
            command->pivot.data.velocity_datapoints[command->pivot.data.velocity_datapoint_count - 1].distance = pivot_length;
            command->pivot.data.velocity_datapoints[command->pivot.data.velocity_datapoint_count - 1].value = 0;
            for(u32 j = 0; j < command->pivot.data.velocity_datapoint_count; j++) {
               command->pivot.data.velocity_datapoints[j].distance = Min(command->pivot.data.velocity_datapoints[j].distance, pivot_length);
            }

            ForEachArray(j, cevent, command->pivot.data.continuous_event_count, command->pivot.data.continuous_events, {
               cevent->samples[0].distance = 0;
               cevent->samples[cevent->sample_count - 1].distance = pivot_length;
               for(u32 k = 0; k < cevent->sample_count; k++) {
                  cevent->samples[k].distance = Min(cevent->samples[k].distance, pivot_length);
               }
            });

            ForEachArray(j, devent, command->pivot.data.discrete_event_count, command->pivot.data.discrete_events, {
               devent->distance = Clamp(0, pivot_length, devent->distance);
            });

            start_angle = command->pivot.end_angle;
         }
      }
   }

   for(u32 i = 0; i < node->path_count; i++) {
      RecalculateAutoNode(node->out_paths[i]->out_node);
   }
}

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

         result->pivot.data.velocity_datapoint_count = body->velocity_datapoint_count;
         Assert(result->pivot.data.velocity_datapoint_count >= 2);
         result->pivot.data.velocity_datapoints = ConsumeAndCopyArray(arena, file, North_PathDataPoint, body->velocity_datapoint_count);

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
   
   path->data.velocity_datapoint_count = file_path->velocity_datapoint_count;
   Assert(path->data.velocity_datapoint_count >= 2);
   path->data.velocity_datapoints = ConsumeAndCopyArray(arena, file, North_PathDataPoint, file_path->velocity_datapoint_count);

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

   path->length_to_pos_memory = PlatformAllocArena(GetMapMemorySize());
   path->time_to_length_memory = PlatformAllocArena(GetMapMemorySize());

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

void ReadProjectsStartingAt(AutoProjectList *list, u32 field_flags, v2 pos) {
   Reset(&list->arena);
   list->first = NULL;

   for(FileListLink *file = ListFilesWithExtension("*.ncap"); file; file = file->next) {
      AutoProjectLink *auto_proj = ReadAutoProject(file->name, &list->arena);
      if(auto_proj) {
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
            
            body.velocity_datapoint_count = command->pivot.data.velocity_datapoint_count;
            body.continuous_event_count = command->pivot.data.continuous_event_count;
            body.discrete_event_count = command->pivot.data.discrete_event_count;
         });
         
         WriteArray(file, command->pivot.data.velocity_datapoints, command->pivot.data.velocity_datapoint_count);
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

      path_header.velocity_datapoint_count = path->data.velocity_datapoint_count;
      path_header.continuous_event_count = path->data.continuous_event_count;
      path_header.discrete_event_count = path->data.discrete_event_count;
   });

   if(path->has_conditional)
      WriteString(file, path->conditional);
   
   WriteArray(file, path->control_points, path->control_point_count);
   
   WriteArray(file, path->data.velocity_datapoints, path->data.velocity_datapoint_count);
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

//TODO: AutoProjectLink to packet