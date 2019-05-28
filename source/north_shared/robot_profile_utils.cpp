//NOTE: requires common.cpp & the north defs

struct RobotProfileGroup;
struct RobotProfileParameter {
   RobotProfileGroup *group;

   string name;
   bool is_array;
   u32 length; //ignored if is_array is false
   union {
      f32 value; //is_array = false
      f32 *values; //is_array = true
   };
};

struct RobotProfileGroup {
   RobotProfileGroup *next;

   string name;
   bool collapsed; //NOTE: used for the UI

   u32 param_count;
   RobotProfileParameter *params;
};

struct RobotProfileCommand {
   North_CommandExecutionType::type type;
   string name;
   u32 param_count;
   string *params;
   v4 colour;
};

namespace RobotProfileState {
   enum type {
      Invalid,
      Loaded,
      Connected
   };
};

struct RobotProfile {
   MemoryArena *arena; //NOTE: owned by RobotProfile

   RobotProfileState::type state;
   string name;
   v2 size;
   
   RobotProfileGroup default_group;
   u32 group_count;
   RobotProfileGroup *first_group;

   u32 command_count;
   RobotProfileCommand *commands;

   u32 conditional_count;
   string *conditionals;
};

bool IsValid(RobotProfile *profile) {
   return profile->state != RobotProfileState::Invalid;
}

v4 ColourForName(string name) {
   u32 hash = Hash(name);
   v3 rgb = V3((f32)((hash << 0) & 0xFFFF) / (f32)0xFFFF, 
               (f32)((hash << 3) & 0xFFFF) / (f32)0xFFFF,
               (f32)((hash << 5) & 0xFFFF) / (f32)0xFFFF);
   rgb = Normalize(rgb);
   return V4(rgb.r, rgb.g, rgb.b, 1);
}

RobotProfileGroup *GetGroup(RobotProfile *profile, string name) {
   if(name.length == 0) {
      return &profile->default_group;
   } else {
      for(RobotProfileGroup *group = profile->first_group; 
          group; group = group->next)
      {
         if(group->name == name) {
            return group;
         }
      }

      return NULL;
   }
}

RobotProfileGroup *GetOrCreateGroup(RobotProfile *profile, string name) {
   RobotProfileGroup *result = GetGroup(profile, name);
   
   if(result == NULL) {
      MemoryArena *arena = profile->arena;
      result = PushStruct(arena, RobotProfileGroup);
      result->name = PushCopy(arena, name);

      result->next = profile->first_group;
      profile->first_group = result;
      profile->group_count++;
   }

   return result;
}

RobotProfileParameter *GetParameter(RobotProfileGroup *group, string name) {
   for(u32 i = 0; i < group->param_count; i++) {
      RobotProfileParameter *param = group->params + i;
      if(param->name == name) {
         return param;
      }
   }

   return NULL;
}

RobotProfileCommand *GetCommand(RobotProfile *profile, string name) {
   for(u32 i = 0; i < profile->command_count; i++) {
      RobotProfileCommand *command = profile->commands + i;
      if(command->name == name) {
         return command;
      }
   }

   return NULL;
}

//File-Writing-------------------------------------
void EncodeGroup(RobotProfileGroup *group, buffer *file) {
   WriteStructData(file, RobotProfile_Group, s, {
      s.name_length = group->name.length;
      s.parameter_count = group->param_count; 
   });
   WriteString(file, group->name);
   
   ForEachArray(j, param, group->param_count, group->params, {
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
}

void EncodeProfileFile(RobotProfile *profile, buffer *file) {
   FileHeader magic_numbers = header(ROBOT_PROFILE_MAGIC_NUMBER, ROBOT_PROFILE_CURR_VERSION);
   WriteStruct(file, &magic_numbers);
   
   WriteStructData(file, RobotProfile_FileHeader, s, {
      s.conditional_count = profile->conditional_count;
      s.group_count = profile->group_count;
      s.command_count = profile->command_count;

      s.robot_width = profile->size.x;
      s.robot_length = profile->size.y;
   });
   
   ForEachArray(i, cond, profile->conditional_count, profile->conditionals, {
      u8 len = cond->length;
      WriteStruct(file, &len);
      WriteString(file, *cond);
   });

   EncodeGroup(&profile->default_group, file);
   for(RobotProfileGroup *group = profile->first_group; group; group = group->next) {
      EncodeGroup(group, file);
   }

   ForEachArray(j, command, profile->command_count, profile->commands, {
      WriteStructData(file, RobotProfile_Command, s, {
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
}

void UpdateProfileFile(RobotProfile *profile) {
   buffer file = PushTempBuffer(Megabyte(1));
   EncodeProfileFile(profile, &file);
   WriteEntireFile(Concat(profile->name, Literal(".ncrp")), file);
}

//Packet-Parsing-------------------------------------
void RecieveWelcomePacket(RobotProfile *profile, buffer packet) {
   MemoryArena *arena = profile->arena;
   
   Reset(arena);
   profile->state = RobotProfileState::Connected;
   profile->first_group = NULL;
   profile->group_count = 0;

   Welcome_PacketHeader *header = ConsumeStruct(&packet, Welcome_PacketHeader);
   profile->name = PushCopy(arena, ConsumeString(&packet, header->robot_name_length));
   profile->size = V2(header->robot_width, header->robot_length);
   profile->command_count = header->command_count;
   profile->commands = PushArray(arena, RobotProfileCommand, profile->command_count);
   profile->conditional_count = header->conditional_count;
   profile->conditionals = PushArray(arena, string, profile->conditional_count);

   for(u32 i = 0; i < header->conditional_count; i++) {
      u8 len = *ConsumeStruct(&packet, u8);
      profile->conditionals[i] = PushCopy(arena, ConsumeString(&packet, len));
   }

   for(u32 j = 0; j < header->command_count; j++) {
      Welcome_Command *command_desc = ConsumeStruct(&packet, Welcome_Command);
      RobotProfileCommand *command = profile->commands + j;
      
      command->name = PushCopy(arena, ConsumeString(&packet, command_desc->name_length));
      command->type = (North_CommandExecutionType::type) command_desc->type;
      command->param_count = command_desc->param_count;
      command->params = PushArray(arena, string, command->param_count);
      command->colour = ColourForName(command->name);

      for(u32 k = 0; k < command->param_count; k++) {
         u8 length = *ConsumeStruct(&packet, u8);
         command->params[k] = PushCopy(arena, ConsumeString(&packet, length));
      }
   }

   UpdateProfileFile(profile);
}

void RecieveParamGroup(MemoryArena *arena, RobotProfile *profile, buffer *packet) {
   CurrentParameters_Group *param_group = ConsumeStruct(packet, CurrentParameters_Group);
   string name = ConsumeString(packet, param_group->name_length);
   RobotProfileGroup *group = GetOrCreateGroup(profile, name);
   Assert(group != NULL);

   if(group->params == NULL) {
      //NOTE: we're getting CurrentParameters for the first time
      
      group->param_count = param_group->param_count;
      group->params = PushArray(arena, RobotProfileParameter, group->param_count);   
   
      for(u32 j = 0; j < param_group->param_count; j++) {
         RobotProfileParameter *param = group->params + j;
         CurrentParameters_Parameter *packet_param = ConsumeStruct(packet, CurrentParameters_Parameter);
         
         param->name = PushCopy(arena, ConsumeString(packet, packet_param->name_length));
         param->group = group;
         param->is_array = packet_param->is_array;
         f32 *values = ConsumeArray(packet, f32, param->is_array ? packet_param->value_count : 1);
         if(param->is_array) {
            param->length = packet_param->value_count;
            param->values = (f32 *) PushCopy(arena, values, param->length * sizeof(f32));
         } else {
            param->value = *values;
         }
      }
   } else {
      //NOTE: parameter values got changed

      for(u32 j = 0; j < param_group->param_count; j++) {
         CurrentParameters_Parameter *packet_param = ConsumeStruct(packet, CurrentParameters_Parameter);
         string name = ConsumeString(packet, packet_param->name_length);
         f32 *values = ConsumeArray(packet, f32, packet_param->is_array ? packet_param->value_count : 1);
         
         RobotProfileParameter *param = GetParameter(group, name);
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

void RecieveCurrentParametersPacket(RobotProfile *profile, buffer packet) {
   Assert(profile->state == RobotProfileState::Connected);
   MemoryArena *arena = profile->arena;
   
   CurrentParameters_PacketHeader *header = ConsumeStruct(&packet, CurrentParameters_PacketHeader);
   
   RecieveParamGroup(arena, profile, &packet);
   for(u32 i = 0; i < header->group_count; i++) {
      RecieveParamGroup(arena, profile, &packet);  
   }

   UpdateProfileFile(profile);
}

void ParseGroup(MemoryArena *arena, buffer *file, RobotProfile *profile) {
   RobotProfile_Group *file_group = ConsumeStruct(file, RobotProfile_Group);
   string name = ConsumeString(file, file_group->name_length);

   RobotProfileGroup *group = GetOrCreateGroup(profile, name);
   group->param_count = file_group->parameter_count;
   group->params = PushArray(arena, RobotProfileParameter, group->param_count);
   group->collapsed = true;
   
   for(u32 j = 0; j < file_group->parameter_count; j++) {
      RobotProfile_Parameter *file_param = ConsumeStruct(file, RobotProfile_Parameter);
      RobotProfileParameter *param = group->params + j;

      param->name = PushCopy(arena, ConsumeString(file, file_param->name_length));
      param->is_array = file_param->is_array;
      f32 *values = ConsumeArray(file, f32, file_param->is_array ? file_param->value_count : 1);
      if(param->is_array) {
         param->length = file_param->value_count;
         param->values = (f32 *) PushCopy(arena, values, param->length * sizeof(f32));
      } else {
         param->value = *values;
      }
   }
}

void ParseProfileFile(RobotProfile *profile, buffer file, string name) {
   MemoryArena *arena = profile->arena;
   
   profile->state = RobotProfileState::Loaded;
   profile->first_group = NULL;
   ZeroStruct(&profile->default_group);
   Reset(arena);

   FileHeader *file_numbers = ConsumeStruct(&file, FileHeader);
   Assert(file_numbers->magic_number == ROBOT_PROFILE_MAGIC_NUMBER);
   Assert(file_numbers->version_number == ROBOT_PROFILE_CURR_VERSION);

   RobotProfile_FileHeader *header = ConsumeStruct(&file, RobotProfile_FileHeader);
   profile->name = PushCopy(arena, name);
   profile->size = V2(header->robot_width, header->robot_length);
   profile->command_count = header->command_count;
   profile->commands = PushArray(arena, RobotProfileCommand, header->command_count);
   profile->conditional_count = header->conditional_count;
   profile->conditionals = PushArray(arena, string, header->conditional_count);

   for(u32 i = 0; i < header->conditional_count; i++) {
      u8 len = *ConsumeStruct(&file, u8);
      profile->conditionals[i] = PushCopy(arena, ConsumeString(&file, len));
   }

   ParseGroup(arena, &file, profile);
   for(u32 i = 0; i < header->group_count; i++) {
      ParseGroup(arena, &file, profile);
   }

   for(u32 j = 0; j < profile->command_count; j++) {
         RobotProfile_Command *file_command = ConsumeStruct(&file, RobotProfile_Command);
         RobotProfileCommand *command = profile->commands + j;
         
         command->name = PushCopy(arena, ConsumeString(&file, file_command->name_length));
         command->type = (North_CommandExecutionType::type) file_command->type;
         command->param_count = file_command->param_count;
         command->params = PushArray(arena, string, command->param_count);
         command->colour = ColourForName(command->name);

         for(u32 k = 0; k < file_command->param_count; k++) {
            u8 length = *ConsumeStruct(&file, u8);
            command->params[k] = PushCopy(arena, ConsumeString(&file, length));
         }
      }
}

//File-Reading----------------------------------------
void LoadProfileFile(RobotProfile *profile, string file_name) {
   buffer loaded_file = ReadEntireFile(Concat(file_name, Literal(".ncrp")));
   if(loaded_file.data != NULL) 
      ParseProfileFile(profile, loaded_file, file_name);
}

buffer MakeParamOpPacket(RobotProfileParameter *param, ParameterOp_Type::type type, f32 value, u32 index) {
   buffer packet = PushTempBuffer(Kilobyte(10));

   u32 size = sizeof(ParameterOp_PacketHeader) + param->group->name.length + param->name.length;
   PacketHeader p_header = { size, (u8)PacketType::ParameterOp };

   ParameterOp_PacketHeader header = {};
   header.type = (u8) type;
   header.group_name_length = param->group->name.length;
   header.param_name_length = param->name.length;

   header.value = value;
   header.index = index;

   WriteStruct(&packet, &p_header);
   WriteStruct(&packet, &header);
   WriteString(&packet, param->group->name);
   WriteString(&packet, param->name);

   return packet;
}

void SendParamSetValue(RobotProfileParameter *param, f32 value, u32 index = 0) {
   buffer packet = MakeParamOpPacket(param, ParameterOp_Type::SetValue, value, index);
   SendPacket(packet);
}

void SendParamRemoveValue(RobotProfileParameter *param, u32 index) {
   buffer packet = MakeParamOpPacket(param, ParameterOp_Type::RemoveValue, 0, index);
   SendPacket(packet);
}

void SendParamAddValue(RobotProfileParameter *param, f32 value) {
   buffer packet = MakeParamOpPacket(param, ParameterOp_Type::AddValue, value, 0);
   SendPacket(packet);
}

//UI--------------------------------------------------
#ifdef INCLUDE_DRAWPROFILES 
struct RobotProfiles {
   RobotProfile current;
   RobotProfile loaded; //NOTE: this is a file we're looking at
   RobotProfile *selected; //??
};

void DrawProfiles_DrawParam(element *group_page, RobotProfile *profile, RobotProfileParameter *param) {
   UI_SCOPE(group_page, param);

   button_style hide_button = ButtonStyle(
      dark_grey, light_grey, BLACK,
      light_grey, V4(120/255.0, 120/255.0, 120/255.0, 1), WHITE, 
      off_white, light_grey,
      20, V2(0, 0), V2(0, 0));

   if(profile->state == RobotProfileState::Connected) {
      if(param->is_array) {
         Label(group_page, Concat(param->name, Literal(": ")), 18, BLACK, V2(20, 0));
         for(u32 i = 0; i < param->length; i++) {
            UI_SCOPE(group_page, i);

            element *param_row = RowPanel(group_page, Size(Size(group_page).x, 18));
            Label(param_row, Concat(ToString(i), Literal(": ")), 18, BLACK, V2(40, 0));
            ui_numberbox param_box = TextBox(param_row, param->values[i], 18);
            
            if(param_box.valid_enter) {
               SendParamSetValue(param, param_box.f32_value, i);
            }

            if(Button(param_row, "Remove", hide_button.Padding(5, 0).IsEnabled(param->length > 1)).clicked) {
               SendParamRemoveValue(param, i);
            }   
         }

         if(Button(group_page, "Add", hide_button.Padding(40, 0)).clicked) {
            SendParamAddValue(param, 0);
         }
      } else {
         element *param_row = RowPanel(group_page, Size(Size(group_page).x, 18));
         Label(param_row, Concat(param->name, Literal(": ")), 18, BLACK, V2(20, 0));
         ui_numberbox param_box = TextBox(param_row, param->value, 18);
         
         if(param_box.valid_enter) {
            SendParamSetValue(param, param_box.f32_value);
         }
      }
   } else {
      if(param->is_array) {
         Label(group_page, Concat(param->name, Literal(": ")), 18, BLACK, V2(20, 0));
         for(u32 i = 0; i < param->length; i++) {
            Label(group_page, Concat(ToString(i), Literal(": "), ToString(param->values[i])), 18, BLACK, V2(40, 0));
         }
      } else {
         Label(group_page, Concat(param->name, Literal(": "), ToString(param->value)), 18, BLACK, V2(20, 0));
      }
   }  
}

void DrawProfiles_DrawGroup(element *page, RobotProfile *profile, RobotProfileGroup *group) {
   UI_SCOPE(page, group);
   element *group_page = ColumnPanel(page, Width(Size(page).x - 60).Padding(20, 0));
   
   button_style hide_button = ButtonStyle(
      dark_grey, light_grey, BLACK,
      light_grey, V4(120/255.0, 120/255.0, 120/255.0, 1), WHITE, 
      off_white, light_grey,
      20, V2(0, 0), V2(0, 0));

   element *top_row = RowPanel(group_page, Size(Size(group_page).x, 20));
   Label(top_row, (group->name.length == 0) ? Literal("Default Group") : group->name, 20, BLACK);
   if(Button(top_row, group->collapsed ? "  +  " : "  -  ", hide_button).clicked) {
      group->collapsed = !group->collapsed;
   }

   if(!group->collapsed) {
      for(u32 i = 0; i < group->param_count; i++) {
         DrawProfiles_DrawParam(group_page, profile, group->params + i);
      }
   }
}

void DrawProfiles(element *full_page, RobotProfiles *profiles, FileListLink *ncrp_files) {
   StackLayout(full_page);
   element *page = VerticalList(full_page);
   
   element *top_bar = RowPanel(page, Size(Size(page).x - 10, page_tab_height).Padding(5, 5));
   Background(top_bar, dark_grey);

   static bool selector_open = false;

   if(Button(top_bar, "Open Profile", menu_button.IsSelected(selector_open)).clicked) {
      selector_open = !selector_open;
   }

   if(selector_open) {
      if(profiles->current.state == RobotProfileState::Connected) {
         if(Button(page, "Connected Robot", menu_button).clicked) {
            profiles->selected = &profiles->current;
            selector_open = false;
         }

         element *divider = Panel(page, Size(Size(page->bounds).x - 40, 5).Padding(10, 0));
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
         
         if(Button(page, "Load", menu_button.IsEnabled(profiles->current.name != profile->name)).clicked) {
            LoadProfileFile(&profiles->current, profile->name);
         }

         element *params_panel = ColumnPanel(page, Width( Size(page).x ));
         Background(params_panel, V4(0.5, 0.5, 0.5, 0.5));
         Label(params_panel, "Params", 20, BLACK);
         DrawProfiles_DrawGroup(params_panel, profile, &profile->default_group);
         for(RobotProfileGroup *group = profile->first_group;
             group; group = group->next)
         {  
            DrawProfiles_DrawGroup(params_panel, profile, group);
         }
         FinalizeLayout(params_panel);

         Label(page, "Commands", 20, BLACK);
         for(u32 j = 0; j < profile->command_count; j++) {
            RobotProfileCommand *command = profile->commands + j;
            //TODO: icon for command->type
            string params = (command->param_count > 0) ? command->params[0] : EMPTY_STRING;
            for(u32 k = 1; k < command->param_count; k++) {
               params = Concat(params, Literal(", "), command->params[k]);
            }
            Label(page, Concat(command->name, Literal("("), params, Literal(")")), 18, BLACK, V2(20, 0));  
         }

         Label(page, "Conditionals", 20, BLACK, V2(0, 20));
         for(u32 j = 0; j < profile->conditional_count; j++) {
            Label(page, profile->conditionals[j], 18, BLACK, V2(20, 0));
         }
      } else {
         selector_open = true;
      }
   }
}

#endif