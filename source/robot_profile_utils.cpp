//NOTE: requires common.cpp & the north defs

struct RobotProfileCommand {
   North_CommandType::type type;
   string name;
   u32 param_count;
   string *params;
};

struct RobotProfileParameter {
   string name;
   bool is_array;
   u32 length; //ignored if is_array is false
   union {
      f32 value; //is_array = false
      f32 *values; //is_array = true
   };
};

struct RobotProfileSubsystem {
   string name;
   u32 command_count;
   RobotProfileCommand *commands;
   u32 param_count;
   RobotProfileParameter *params;
};

namespace RobotProfileState {
   enum type {
      Invalid,
      Loaded,
      Connected
   };
};

struct RobotProfile {
   MemoryArena arena;

   RobotProfileState::type state;
   string name;
   v2 size;
   
   u32 subsystem_count;
   RobotProfileSubsystem *subsystems;
};

bool IsValid(RobotProfile *profile) {
   return profile->state != RobotProfileState::Invalid;
}

void EncodeProfileFile(RobotProfile *profile, buffer *file) {
   WriteStructData(file, RobotProfile_FileHeader, s, {
      s.subsystem_count = profile->subsystem_count;
      s.robot_width = profile->size.x;
      s.robot_length = profile->size.y;
   });
   
   ForEachArray(i, subsystem, profile->subsystem_count, profile->subsystems, {
      WriteStructData(file, RobotProfile_SubsystemDescription, s, {
         s.name_length = subsystem->name.length;
         s.parameter_count = subsystem->param_count;
         s.command_count = subsystem->command_count; 
      });
      WriteString(file, subsystem->name);

      ForEachArray(j, param, subsystem->param_count, subsystem->params, {
         WriteStructData(file, RobotProfile_Parameter, s, {
            s.name_length = param->name.length; 
            s.is_array = param->is_array;
            s.value_count = param->is_array ? param->length : 1;
         });
         WriteString(file, param->name);
         if(param->is_array) {
            WriteArray(file, param->values, param->length);
         } else {
            WriteStruct(file, &param->value);
         }
      });
      
      ForEachArray(j, command, subsystem->command_count, subsystem->commands, {
         WriteStructData(file, RobotProfile_SubsystemCommand, s, {
            s.name_length = command->name.length;
            s.param_count = command->param_count;
            s.type = (u8) command->type;
         });
         WriteString(file, command->name);
         ForEachArray(k, param, command->param_count, command->params, {
            u8 length = param->length;
            WriteStruct(file, &length);
            WriteString(file, *param);
         });
      });
   });
}

void UpdateProfileFile(RobotProfile *profile) {
   buffer file = PushTempBuffer(Megabyte(1));
   EncodeProfileFile(profile, &file);
   WriteEntireFile(Concat(profile->name, Literal(".ncrp")), file);
}

void RecieveWelcomePacket(RobotProfile *profile, buffer packet) {
   MemoryArena *arena = &profile->arena;
   
   Reset(arena);
   profile->state = RobotProfileState::Connected;
   
   Welcome_PacketHeader *header = ConsumeStruct(&packet, Welcome_PacketHeader);
   profile->name = PushCopy(arena, ConsumeString(&packet, header->robot_name_length));
   profile->size = V2(header->robot_width, header->robot_length);
   profile->subsystem_count = header->subsystem_count;
   profile->subsystems = PushArray(arena, RobotProfileSubsystem, profile->subsystem_count);

   for(u32 i = 0; i < header->subsystem_count; i++) {
      Welcome_SubsystemDescription *desc = ConsumeStruct(&packet, Welcome_SubsystemDescription);
      RobotProfileSubsystem *subsystem = profile->subsystems + i;
      
      subsystem->name = PushCopy(arena, ConsumeString(&packet, desc->name_length));
      subsystem->command_count = desc->command_count;
      subsystem->commands = PushArray(arena, RobotProfileCommand, subsystem->command_count);

      for(u32 j = 0; j < desc->command_count; j++) {
         Welcome_SubsystemCommand *command_desc = ConsumeStruct(&packet, Welcome_SubsystemCommand);
         RobotProfileCommand *command = subsystem->commands + j;
         
         command->name = PushCopy(arena, ConsumeString(&packet, command_desc->name_length));
         command->type = (North_CommandType::type) command_desc->type;
         command->param_count = command_desc->param_count;
         command->params = PushArray(arena, string, command->param_count);

         for(u32 k = 0; k < command->param_count; k++) {
            u8 length = *ConsumeStruct(&packet, u8);
            command->params[k] = PushCopy(arena, ConsumeString(&packet, length));
         }
      }
   }

   UpdateProfileFile(profile);
}

RobotProfileSubsystem *GetSubsystem(RobotProfile *profile, string name) {
   for(u32 i = 0; i < profile->subsystem_count; i++) {
      RobotProfileSubsystem *subsystem = profile->subsystems + i;
      if(subsystem->name == name) {
         return subsystem;
      }
   }

   return NULL;
}

RobotProfileParameter *GetParameter(RobotProfileSubsystem *subsystem, string name) {
   for(u32 i = 0; i < subsystem->param_count; i++) {
      RobotProfileParameter *param = subsystem->params + i;
      if(param->name == name) {
         return param;
      }
   }

   return NULL;
}

void RecieveCurrentParametersPacket(RobotProfile *profile, buffer packet) {
   Assert(profile->state == RobotProfileState::Connected);
   MemoryArena *arena = &profile->arena;
   
   CurrentParameters_PacketHeader *header = ConsumeStruct(&packet, CurrentParameters_PacketHeader);
   for(u32 i = 0; i < header->subsystem_count; i++) {
      CurrentParameters_SubsystemParameters *subsystem_params = ConsumeStruct(&packet, CurrentParameters_SubsystemParameters);
      string name = ConsumeString(&packet, subsystem_params->name_length);
      RobotProfileSubsystem *subsystem = GetSubsystem(profile, name);
      Assert(subsystem != NULL);

      if(subsystem->params == NULL) {
         //NOTE: we're getting CurrentParameters for the first time
         
         subsystem->param_count = subsystem_params->param_count;
         subsystem->params = PushArray(arena, RobotProfileParameter, subsystem->param_count);   
      
         for(u32 j = 0; j < subsystem_params->param_count; j++) {
            RobotProfileParameter *param = subsystem->params + j;
            CurrentParameters_Parameter *packet_param = ConsumeStruct(&packet, CurrentParameters_Parameter);
            param->name = PushCopy(arena, ConsumeString(&packet, packet_param->name_length));
            param->is_array = packet_param->is_array;
            f32 *values = ConsumeArray(&packet, f32, param->is_array ? packet_param->value_count : 1);
            if(param->is_array) {
               param->length = packet_param->value_count;
               param->values = (f32 *) PushCopy(arena, values, param->length * sizeof(f32));
            } else {
               param->value = *values;
            }
         }
      } else {
         //NOTE: parameter values got changed

         for(u32 j = 0; j < subsystem_params->param_count; j++) {
            CurrentParameters_Parameter *packet_param = ConsumeStruct(&packet, CurrentParameters_Parameter);
            string name = ConsumeString(&packet, packet_param->name_length);
            f32 *values = ConsumeArray(&packet, f32, packet_param->is_array ? packet_param->value_count : 1);
            
            RobotProfileParameter *param = GetParameter(subsystem, name);
            Assert(param != NULL);

            if(param->is_array) {
               param->length = packet_param->value_count;
               param->values = (f32 *) PushCopy(arena, values, param->length * sizeof(f32));
            } else {
               param->value = *values;
            }
         }
      }
   }

   UpdateProfileFile(profile);
}

void ParseProfileFile(RobotProfile *profile, buffer file, string name) {
   MemoryArena *arena = &profile->arena;
   
   profile->state = RobotProfileState::Loaded;
   Reset(arena);

   FileHeader *file_numbers = ConsumeStruct(&file, FileHeader);
   RobotProfile_FileHeader *header = ConsumeStruct(&file, RobotProfile_FileHeader);
   profile->name = PushCopy(arena, name);
   profile->size = V2(header->robot_width, header->robot_length);
   profile->subsystem_count = header->subsystem_count;
   profile->subsystems = PushArray(arena, RobotProfileSubsystem, header->subsystem_count);

   for(u32 i = 0; i < header->subsystem_count; i++) {
      RobotProfile_SubsystemDescription *file_subsystem = ConsumeStruct(&file, RobotProfile_SubsystemDescription);
      RobotProfileSubsystem *subsystem = profile->subsystems + i;
      
      subsystem->name = PushCopy(arena, ConsumeString(&file, file_subsystem->name_length));
      subsystem->param_count = file_subsystem->parameter_count;
      subsystem->params = PushArray(arena, RobotProfileParameter, subsystem->param_count);
      subsystem->command_count = file_subsystem->command_count;
      subsystem->commands = PushArray(arena, RobotProfileCommand, subsystem->command_count);

      for(u32 j = 0; j < file_subsystem->parameter_count; j++) {
         RobotProfile_Parameter *file_param = ConsumeStruct(&file, RobotProfile_Parameter);
         RobotProfileParameter *param = subsystem->params + j;

         param->name = PushCopy(arena, ConsumeString(&file, file_param->name_length));
         param->is_array = file_param->is_array;
         f32 *values = ConsumeArray(&file, f32, file_param->is_array ? file_param->value_count : 1);
         if(param->is_array) {
            param->length = file_param->value_count;
            param->values = (f32 *) PushCopy(arena, values, param->length * sizeof(f32));
         } else {
            param->value = *values;
         }
      }

      for(u32 j = 0; j < file_subsystem->command_count; j++) {
         RobotProfile_SubsystemCommand *file_command = ConsumeStruct(&file, RobotProfile_SubsystemCommand);
         RobotProfileCommand *command = subsystem->commands + j;
         
         command->name = PushCopy(arena, ConsumeString(&file, file_command->name_length));
         command->type = (North_CommandType::type) file_command->type;
         command->param_count = file_command->param_count;
         command->params = PushArray(arena, string, command->param_count);

         for(u32 k = 0; k < file_command->param_count; k++) {
            u8 length = *ConsumeStruct(&file, u8);
            command->params[k] = PushCopy(arena, ConsumeString(&file, length));
         }
      }
   }
}

void LoadProfileFile(RobotProfile *profile, string file_name) {
   buffer loaded_file = ReadEntireFile(Concat(file_name, Literal(".ncrp")));
   if(loaded_file.data != NULL) 
      ParseProfileFile(profile, loaded_file, file_name);
   
   FreeEntireFile(&loaded_file);
}

#ifdef INCLUDE_DRAWPROFILES 
struct RobotProfiles {
   RobotProfile current;
   RobotProfile loaded; //NOTE: this is a file we're looking at
   RobotProfile *selected; //??
};

void DrawProfiles(element *full_page, RobotProfiles *profiles, FileListLink *ncrp_files) {
   StackLayout(full_page);
   element *page = VerticalList(full_page);
   
   element *top_bar = RowPanel(page, V2(Size(page).x - 10, page_tab_height), Padding(5, 5));
   Background(top_bar, dark_grey);

   static bool selector_open = false;

   if(Button(top_bar, "Open Profile", menu_button.IsSelected(selector_open)).clicked) {
      selector_open = !selector_open;
   }

   if(selector_open) {
      if(profiles->current.state == RobotProfileState::Connected) {
         if(Button(page, "Connected Robot", menu_button).clicked) {
            profiles->selected = &profiles->current;
         }

         element *divider = Panel(page, V2(Size(page->bounds).x - 40, 5), Padding(10, 0));
         Background(divider, BLACK);
      }

      for(FileListLink *file = ncrp_files; file; file = file->next) {
         UI_SCOPE(page->context, file);
         
         if((profiles->current.state == RobotProfileState::Connected) && 
            (profiles->current.name == file->name))
         {
            continue;
         }
         
         if(Button(page, file->name, menu_button).clicked) {
            LoadProfileFile(&profiles->loaded, file->name);

            if(IsValid(&profiles->loaded)) {
               profiles->selected = &profiles->loaded;
               selector_open = false;
            }
         }
      }
   } else {
      if(profiles->selected) {
         RobotProfile *profile = profiles->selected;
         Label(page, profile->name, 20, BLACK);
         
         if(Button(page, "Load", menu_button).clicked) {
            LoadProfileFile(&profiles->current, profile->name);
         }

         for(u32 i = 0; i < profile->subsystem_count; i++) {
            RobotProfileSubsystem *subsystem = profile->subsystems + i;
            element *subsystem_page = Panel(page, V2(Size(page).x - 60, 400), Padding(20, 0).Layout(ColumnLayout));
            Background(subsystem_page, V4(0.5, 0.5, 0.5, 0.5));

            Label(subsystem_page, subsystem->name, 20, BLACK);
            
            Label(subsystem_page, "Parameters", 20, BLACK, V2(0, 20));
            for(u32 j = 0; j < subsystem->param_count; j++) {
               RobotProfileParameter *param = subsystem->params + j;
               if(param->is_array) {
                  Label(subsystem_page, Concat(param->name, Literal(": ")), 18, BLACK, V2(20, 0));
                  for(u32 k = 0; k < param->length; k++) {
                     Label(subsystem_page, ToString(param->values[k]), 18, BLACK, V2(40, 0));
                  }
               } else {
                  Label(subsystem_page, Concat(param->name, Literal(": "), ToString(param->value)), 18, BLACK, V2(20, 0));
               }
            }

            Label(subsystem_page, "Commands", 20, BLACK, V2(0, 20));
            for(u32 j = 0; j < subsystem->command_count; j++) {
               RobotProfileCommand *command = subsystem->commands + j;
               //TODO: icon for command->type
               string params = (command->param_count > 0) ? command->params[0] : EMPTY_STRING;
               for(u32 k = 1; k < command->param_count; k++) {
                  params = Concat(params, Literal(", "), command->params[k]);
               }
               Label(subsystem_page, Concat(command->name, Literal("("), params, Literal(")")), 18, BLACK, V2(20, 0));  
            }
         }
      } else {
         selector_open = true;
      }
   }
}

#endif