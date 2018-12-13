//NOTE: needs common & north_file_definitions

//TODO: redo this to use: 
//    file linking
//    starting point match & reflect

struct AutoCommand {
   string subsystem_name;
   string command_name;
   u32 param_count;
   f32 *params;
};

struct AutoPath;
struct AutoNode {
   v2 pos;
   
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
   AutoCommand command;
};

struct AutoPath {   
   AutoNode *in_node;
   v2 in_tangent;
   AutoNode *out_node;
   v2 out_tangent;

   bool is_reverse;
   string conditional;

   u32 velocity_datapoint_count;
   AutonomousProgram_DataPoint *velocity_datapoints;

   u32 continuous_event_count;
   AutoContinuousEvent *continuous_events;

   u32 discrete_event_count;
   AutoDiscreteEvent *discrete_events;

   u32 control_point_count;
   AutonomousProgram_ControlPoint *control_points;
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

AutoCommand CreateCommand(MemoryArena *arena, string subsystem_name, string command_name,
                           u32 param_count, f32 *params) 
{
   AutoCommand result = {};
   result.subsystem_name = PushCopy(arena, subsystem_name);
   result.command_name = PushCopy(arena, command_name);
   result.param_count = param_count;
   result.params = PushArray(arena, f32, param_count);
   Copy(params, sizeof(f32) * param_count, result.params);
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
      result->commands[i] = CreateCommand(arena, subsystem_name, command_name, file_command->parameter_count, params);
   }
   
   for(u32 i = 0; i < file_node->path_count; i++) {
      AutoPath *path = ParseAutoPath(file, arena);
      path->in_node = result;
      result->out_paths[i] = path;
   }

   return result;
}

AutoPath *ParseAutoPath(buffer *file, MemoryArena *arena) {
   AutoPath *path = PushStruct(arena, AutoPath);
   AutonomousProgram_Path *file_path = ConsumeStruct(file, AutonomousProgram_Path);
   
   if(file_path->conditional_length == 0) {
      path->conditional = EMPTY_STRING;
   } else {
      path->conditional =  PushCopy(arena, ConsumeString(file, file_path->conditional_length));
   }

   path->control_point_count = file_path->control_point_count;
   path->control_points = ConsumeAndCopyArray(arena, file, AutonomousProgram_ControlPoint, file_path->control_point_count);
   
   path->velocity_datapoint_count = file_path->velocity_datapoint_count;
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
      
      event->command = CreateCommand(arena, subsystem_name, command_name, 
                                     file_event->parameter_count, params);
      event->distance = file_event->distance;
   }

   path->out_node = ParseAutoNode(file, arena);
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

//TODO: AutoProjectLink to file
void WriteProject(AutoProjectLink *project) {
   
}

//TODO: AutoProjectLink to packet