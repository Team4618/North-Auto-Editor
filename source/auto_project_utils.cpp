//NOTE: needs common & north_file_definitions

//TODO: redo this to use: 
//    file linking
//    starting point match & reflect

struct AutoCommand {
   string subsystem_name;
   string command_name;
   u32 param_count;
   f32 *params;

   bool has_conditional;
   string conditional;
};

struct AutoPath;
struct AutoNode {
   v2 pos;
   AutoPath *in_path;

   u32 command_count;
   AutoCommand *commands;
   
   u32 path_count;
   AutoPath **out_paths;
};

struct AutoContinuousEvent {
   string subsystem_name;
   string command_name;
   u32 sample_count;
   AutonomousProgram_DataPoint *samples;
};

struct AutoDiscreteEvent {
   f32 distance;
   string subsystem_name;
   string command_name;
   u32 param_count;
   f32 *params;
};

struct AutoPath_LenLeaf {
   f32 len;
   v2 pos;
};

struct AutoPath_LenBranch {
   f32 len;

   union {
      struct {
         AutoPath_LenLeaf *greater_leaf;
         AutoPath_LenLeaf *less_leaf;
      };

      struct {
         AutoPath_LenBranch *greater_branch;
         AutoPath_LenBranch *less_branch;
      };
   };
};

// struct AutoPath_TimeLeaf {
//    f32 time;
//    f32 len;
// };

// struct AutoPath_TimeBranch {
//    f32 time;

//    union {
//       struct {
//          AutoPath_TimeLeaf *greater_leaf;
//          AutoPath_TimeLeaf *less_leaf;
//       };

//       struct {
//          AutoPath_TimeBranch *greater_branch;
//          AutoPath_TimeBranch *less_branch;
//       };
//    };
// };

//NOTE: this means we have 2^length_to_pos_sample_exp samples
const u32 length_to_pos_sample_exp = 7;
u64 GetLenToPosMemorySize() {
   u64 result = Power(2, length_to_pos_sample_exp) * sizeof(AutoPath_LenLeaf);
   for(u32 i = 0; i < length_to_pos_sample_exp; i++) {
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

   u32 velocity_datapoint_count;
   AutonomousProgram_DataPoint *velocity_datapoints;

   u32 continuous_event_count;
   AutoContinuousEvent *continuous_events;

   u32 discrete_event_count;
   AutoDiscreteEvent *discrete_events;

   u32 control_point_count;
   AutonomousProgram_ControlPoint *control_points;

   //-------------------------------------
   MemoryArena length_to_pos_memory;
   f32 length;
   AutoPath_LenBranch *len_root;
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
   AutonomousProgram_ControlPoint *points;
};

AutoPathSpline GetAutoPathSpline(AutoPath *path, MemoryArena *arena = &__temp_arena) {
   u32 control_point_count = path->control_point_count + 2;
   AutonomousProgram_ControlPoint *control_points = PushArray(arena, AutonomousProgram_ControlPoint, control_point_count);
   control_points[0] = { path->in_node->pos, path->in_tangent };
   control_points[control_point_count - 1] = { path->out_node->pos, path->out_tangent };
   Copy(path->control_points, path->control_point_count * sizeof(AutonomousProgram_ControlPoint), control_points + 1);

   AutoPathSpline result = { control_point_count, control_points };
   return result;
}

v2 CubicHermiteSpline(AutonomousProgram_ControlPoint a, AutonomousProgram_ControlPoint b, f32 t) {
   return CubicHermiteSpline(a.pos, a.tangent, b.pos, b.tangent, t);
}

v2 CubicHermiteSplineTangent(AutonomousProgram_ControlPoint a, AutonomousProgram_ControlPoint b, f32 t) {
   return CubicHermiteSplineTangent(a.pos, a.tangent, b.pos, b.tangent, t);
}

void RecalculateAutoPath(AutoPath *path) {
   MemoryArena *arena = &path->length_to_pos_memory;
   Reset(arena);

   u32 sample_count = Power(2, length_to_pos_sample_exp); 
   AutoPath_LenLeaf *samples = PushArray(arena, AutoPath_LenLeaf, sample_count);
   AutoPathSpline spline = GetAutoPathSpline(path);
   f32 step = (f32)(spline.point_count - 1) / (f32)sample_count;
   
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
   path->length = length;

   u32 last_layer_count = Power(2, length_to_pos_sample_exp - 1);
   AutoPath_LenBranch *last_layer = PushArray(arena, AutoPath_LenBranch, last_layer_count);
   for(u32 i = 0; i < last_layer_count; i++) {
      AutoPath_LenLeaf *less = samples + (2 * i);
      AutoPath_LenLeaf *greater = samples + (2 * i + 1);

      last_layer[i].less_leaf = less;
      last_layer[i].greater_leaf = greater;
      last_layer[i].len = (less->len + greater->len) / 2;
   }

   for(u32 i = 1; i < length_to_pos_sample_exp; i++) {
      u32 layer_count = Power(2, length_to_pos_sample_exp - i - 1);
      AutoPath_LenBranch *curr_layer = PushArray(arena, AutoPath_LenBranch, layer_count);

      for(u32 i = 0; i < layer_count; i++) {
         AutoPath_LenBranch *less = last_layer + (2 * i);
         AutoPath_LenBranch *greater = last_layer + (2 * i + 1);

         curr_layer[i].less_branch = less;
         curr_layer[i].greater_branch = greater;
         curr_layer[i].len = (less->len + greater->len) / 2;
      }

      last_layer_count = layer_count;
      last_layer = curr_layer;
   }

   path->len_root = last_layer;

   Assert(path->velocity_datapoint_count > 2);
   path->velocity_datapoints[path->velocity_datapoint_count - 1].distance = path->length;
   for(u32 i = 0; i < path->velocity_datapoint_count; i++) {
      path->velocity_datapoints[i].distance = Min(path->velocity_datapoints[i].distance, path->length);
   }

   //TODO: calculate (time -> distance) map
}

v2 GetAutoPathPoint(AutoPath *path, f32 distance) {
   AutoPath_LenBranch *branch = path->len_root;
   for(u32 i = 0; i < (length_to_pos_sample_exp - 1); i++) {
      branch = (distance > branch->len) ? branch->greater_branch : branch->less_branch;
   }
   f32 t = (distance - branch->less_leaf->len) / (branch->greater_leaf->len - branch->less_leaf->len); 
   return lerp(branch->less_leaf->pos, t, branch->greater_leaf->pos);
}

AutoCommand CreateCommand(MemoryArena *arena, string subsystem_name, string command_name,
                           u32 param_count, f32 *params, string conditional) 
{
   AutoCommand result = {};
   result.subsystem_name = PushCopy(arena, subsystem_name);
   result.command_name = PushCopy(arena, command_name);
   result.param_count = param_count;
   result.params = PushArray(arena, f32, param_count);
   Copy(params, sizeof(f32) * param_count, result.params);
   result.conditional = PushCopy(arena, conditional);
   result.has_conditional = conditional.length > 0;
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
   result->commands = PushArray(arena, AutoCommand, result->command_count);

   for(u32 i = 0; i < file_node->command_count; i++) {
      AutonomousProgram_Command *file_command = ConsumeStruct(file, AutonomousProgram_Command);
      string subsystem_name = ConsumeString(file, file_command->subsystem_name_length);
      string command_name = ConsumeString(file, file_command->command_name_length);
      f32 *params = ConsumeArray(file, f32, file_command->parameter_count);
      string conditional = ConsumeString(file, file_command->conditional_length);
      
      result->commands[i] = CreateCommand(arena, subsystem_name, command_name, file_command->parameter_count, params, conditional);
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

   if(file_path->conditional_length == 0) {
      path->conditional = EMPTY_STRING;
      path->has_conditional = false;
   } else {
      path->conditional =  PushCopy(arena, ConsumeString(file, file_path->conditional_length));
      path->has_conditional = true;
   }

   path->control_point_count = file_path->control_point_count;
   path->control_points = ConsumeAndCopyArray(arena, file, AutonomousProgram_ControlPoint, file_path->control_point_count);
   
   path->velocity_datapoint_count = file_path->velocity_datapoint_count;
   Assert(path->velocity_datapoint_count > 2);
   path->velocity_datapoints = ConsumeAndCopyArray(arena, file, AutonomousProgram_DataPoint, file_path->velocity_datapoint_count);

   path->continuous_event_count = file_path->continuous_event_count;
   path->continuous_events = PushArray(arena, AutoContinuousEvent, path->continuous_event_count);
   for(u32 i = 0; i < file_path->continuous_event_count; i++) {
      AutonomousProgram_ContinuousEvent *file_event = ConsumeStruct(file, AutonomousProgram_ContinuousEvent);
      AutoContinuousEvent *event = path->continuous_events + i;

      event->subsystem_name = PushCopy(arena, ConsumeString(file, file_event->subsystem_name_length));
      event->command_name = PushCopy(arena, ConsumeString(file, file_event->command_name_length));
      event->sample_count = file_event->datapoint_count;
      event->samples = PushArray(arena, AutonomousProgram_DataPoint, event->sample_count);
      AutonomousProgram_DataPoint *data_points = ConsumeArray(file, AutonomousProgram_DataPoint, file_event->datapoint_count);
      for(u32 j = 0; j < event->sample_count; j++) {
         event->samples[j].distance = data_points[j].distance;
         event->samples[j].value = data_points[j].value;
      }
   }

   path->discrete_event_count = file_path->discrete_event_count;
   path->discrete_events = PushArray(arena, AutoDiscreteEvent, path->discrete_event_count);
   for(u32 i = 0; i < file_path->discrete_event_count; i++) {
      AutonomousProgram_DiscreteEvent *file_event = ConsumeStruct(file, AutonomousProgram_DiscreteEvent);
      AutoDiscreteEvent *event = path->discrete_events + i;
      
      string subsystem_name = ConsumeString(file, file_event->subsystem_name_length);
      string command_name = ConsumeString(file, file_event->command_name_length);
      f32 *params = ConsumeArray(file, f32, file_event->parameter_count);
      
      event->subsystem_name = PushCopy(arena, subsystem_name);
      event->command_name = PushCopy(arena, command_name);
      event->param_count = file_event->parameter_count;
      event->params = PushArrayCopy(arena, f32, params, file_event->parameter_count);
      event->distance = file_event->distance;
   }

   path->length_to_pos_memory = PlatformAllocArena(GetLenToPosMemorySize());

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

void WriteAutoPath(buffer *file, AutoPath *path);
void WriteAutoNode(buffer *file, AutoNode *node) {
   WriteStructData(file, AutonomousProgram_Node, node_header, {
      node_header.pos = node->pos;
      node_header.command_count = node->command_count;
      node_header.path_count = node->path_count;
   });

   ForEachArray(i, command, node->command_count, node->commands, {
      WriteStructData(file, AutonomousProgram_Command, command_header, {
         command_header.subsystem_name_length = command->subsystem_name.length;
         command_header.command_name_length = command->command_name.length;
         command_header.parameter_count = command->param_count;
         command_header.conditional_length = command->has_conditional ? command->conditional.length : 0;
      });
      WriteString(file, command->subsystem_name);
      WriteString(file, command->command_name);
      WriteArray(file, command->params, command->param_count);
      
      if(command->has_conditional)
         WriteString(file, command->conditional);
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

      path_header.velocity_datapoint_count = path->velocity_datapoint_count;
      path_header.continuous_event_count = path->continuous_event_count;
      path_header.discrete_event_count = path->discrete_event_count;
   });

   if(path->has_conditional)
      WriteString(file, path->conditional);
   
   WriteArray(file, path->control_points, path->control_point_count);
   
   WriteArray(file, path->velocity_datapoints, path->velocity_datapoint_count);
   ForEachArray(i, event, path->continuous_event_count, path->continuous_events, {
      WriteStructData(file, AutonomousProgram_ContinuousEvent, event_header, {
         event_header.subsystem_name_length = event->subsystem_name.length;
         event_header.command_name_length = event->command_name.length;
         event_header.datapoint_count = event->sample_count;
      });
      WriteString(file, event->subsystem_name);
      WriteString(file, event->command_name);
      WriteArray(file, event->samples, event->sample_count);
   });
   ForEachArray(i, event, path->discrete_event_count, path->discrete_events, {
      WriteStructData(file, AutonomousProgram_DiscreteEvent, event_header, {
         event_header.distance = event->distance;
         event_header.subsystem_name_length = event->subsystem_name.length;
         event_header.command_name_length = event->command_name.length;
         event_header.parameter_count = event->param_count;
      });
      WriteString(file, event->subsystem_name);
      WriteString(file, event->command_name);
      WriteArray(file, event->params, event->param_count);
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