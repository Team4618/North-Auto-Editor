struct NorthSettings {
   MemoryArena *arena; //NOTE: owned by NorthSettings 
   
   struct {
      u32 team_number;
      string field_name;

      struct {
         bool loaded;
         v2 size;
         u32 flags;
         u32 starting_position_count;
         Field_StartingPosition starting_positions[16];
      } field;
   } saved_data;

   u32 team_number;
   string field_name;

   struct {
      bool loaded;
      v2 size;
      u32 flags;
      u32 starting_position_count;
      Field_StartingPosition starting_positions[16];
      texture image;
   } field;
};

bool DifferentThanSaved(NorthSettings *state) {
   bool result = 
      (state->saved_data.team_number != state->team_number) ||
      (state->saved_data.field_name != state->field_name) ||
      (state->saved_data.field.loaded != state->field.loaded) ||
      (Length(state->saved_data.field.size - state->field.size) > 0) ||
      (state->saved_data.field.flags != state->field.flags) ||
      (state->saved_data.field.starting_position_count != state->field.starting_position_count);

   if(!result) {
      for(u32 i = 0; i < state->saved_data.field.starting_position_count; i++) {
         result |= Length(state->saved_data.field.starting_positions[i].pos - state->field.starting_positions[i].pos) > 0;
         result |= (state->saved_data.field.starting_positions[i].angle != state->field.starting_positions[i].angle);
      }
   }
   
   return result;
}

void ReadSettingsFile(NorthSettings *settings) {
   Reset(settings->arena);

   buffer settings_file = ReadEntireFile("settings.ncsf");
   if(settings_file.data != NULL) {
      FileHeader *file_numbers = ConsumeStruct(&settings_file, FileHeader);
      
      if((file_numbers->magic_number == SETTINGS_MAGIC_NUMBER) && 
         (file_numbers->version_number == SETTINGS_CURR_VERSION))
      {
         Settings_FileHeader *header = ConsumeStruct(&settings_file, Settings_FileHeader);
         
         settings->team_number = header->team_number;
         settings->field_name = PushCopy(settings->arena, ConsumeString(&settings_file, header->field_name_length));
      }
   }

   buffer field_file = ReadEntireFile(Concat(settings->field_name, Literal(".ncff")));
   settings->field.loaded = (field_file.data != NULL);
   if(field_file.data != NULL) {
      FileHeader *file_numbers = ConsumeStruct(&field_file, FileHeader);

      if((file_numbers->magic_number == FIELD_MAGIC_NUMBER) && 
         (file_numbers->version_number == FIELD_CURR_VERSION))
      {
         Field_FileHeader *header = ConsumeStruct(&field_file, Field_FileHeader);
         settings->field.size = V2(header->width, header->height);
         settings->field.flags = header->flags;

         u32 starting_positions_size = sizeof(Field_StartingPosition) * header->starting_position_count;
         Field_StartingPosition *starting_positions = ConsumeArray(&field_file, Field_StartingPosition,
                                                                   header->starting_position_count);
         
         //TODO: make some sort of error message instead?
         Assert(header->starting_position_count <= ArraySize(settings->field.starting_positions));
         
         settings->field.starting_position_count = header->starting_position_count;
         Copy(starting_positions, starting_positions_size, settings->field.starting_positions);
         
         deleteTexture(settings->field.image);
         u32 *image_texels = (u32 *) ConsumeSize(&field_file, header->image_width * header->image_height * sizeof(u32));
         settings->field.image = createTexture(image_texels, header->image_width, header->image_height);
      }
   }

   settings->saved_data.team_number = settings->team_number;
   settings->saved_data.field_name = PushCopy(settings->arena, settings->field_name);
   settings->saved_data.field.loaded = settings->field.loaded;
   settings->saved_data.field.size = settings->field.size;
   settings->saved_data.field.flags = settings->field.flags;
   settings->saved_data.field.starting_position_count = settings->field.starting_position_count;
   Copy(settings->field.starting_positions, sizeof(settings->field.starting_positions), settings->saved_data.field.starting_positions);
}

void UpdateFieldFile(NorthSettings *settings) {
   image img = {};
   {
      buffer old_field_file = ReadEntireFile(Concat(settings->field_name, Literal(".ncff")));
      ConsumeStruct(&old_field_file, FileHeader);
      Field_FileHeader *header = ConsumeStruct(&old_field_file, Field_FileHeader);
      ConsumeArray(&old_field_file, Field_StartingPosition, header->starting_position_count);
      img.width = header->image_width;
      img.height = header->image_height;
      img.texels = (u32 *) ConsumeSize(&old_field_file, img.width * img.height * 4);
   }

   FileHeader numbers = header(FIELD_MAGIC_NUMBER, FIELD_CURR_VERSION);
   Field_FileHeader header = {};
   header.width = settings->field.size.x;
   header.height = settings->field.size.y;
   header.flags = settings->field.flags;
   header.starting_position_count = settings->field.starting_position_count;
   header.image_width = img.width;
   header.image_height = img.height;

   u32 starting_positions_size = header.starting_position_count * sizeof(Field_StartingPosition);
   u32 image_size = img.width * img.height * 4;
   buffer field_file = PushTempBuffer(sizeof(numbers) + sizeof(header) +
                                      starting_positions_size + image_size);
   WriteStruct(&field_file, &numbers);
   WriteStruct(&field_file, &header);
   WriteSize(&field_file, settings->field.starting_positions, starting_positions_size);
   WriteSize(&field_file, img.texels, image_size);
   
   WriteEntireFile(Concat(settings->field_name, Literal(".ncff")), field_file);
   OutputDebugStringA("field updated\n");
}

void UpdateSettingsFile(NorthSettings *settings) {
   FileHeader settings_numbers = header(SETTINGS_MAGIC_NUMBER, SETTINGS_CURR_VERSION);
   Settings_FileHeader settings_header = {};
   settings_header.team_number = (u16) settings->team_number;
   settings_header.field_name_length = (u16) settings->field_name.length;

   buffer settings_file = PushTempBuffer(sizeof(settings_numbers) + sizeof(settings_header) + settings->field_name.length);
   WriteStruct(&settings_file, &settings_numbers);
   WriteStruct(&settings_file, &settings_header);
   WriteSize(&settings_file, settings->field_name.text, settings->field_name.length);

   WriteEntireFile("settings.ncsf", settings_file);
   OutputDebugStringA("settings updated\n");
}

void WriteNewFieldFile(string name, f32 width, f32 height, string img_file_name) {
   FileHeader numbers = header(FIELD_MAGIC_NUMBER, FIELD_CURR_VERSION);
   Field_FileHeader header = {};
   header.width = width;
   header.height = height;
   image img = ReadImage(img_file_name);
   header.image_width = img.valid ? img.width : 0;
   header.image_height = img.valid ? img.height : 0;

   u32 image_size = img.width * img.height * 4;

   buffer field_file = PushTempBuffer(sizeof(numbers) + sizeof(header) + image_size);
   WriteStruct(&field_file, &numbers);
   WriteStruct(&field_file, &header);
   if(img.valid)
      WriteSize(&field_file, img.texels, image_size);
   
   WriteEntireFile(Concat(name, Literal(".ncff")), field_file);
   OutputDebugStringA("new field written\n");
}

#ifdef INCLUDE_DRAWSETTINGS

#define SettingsRow(...) _SettingsRow(GEN_UI_ID, __VA_ARGS__)
ui_numberbox _SettingsRow(ui_id id, element *page, char *label, u32 *num, char *suffix = "") {
   element *row = RowPanel(page, Size(Size(page).x, 20).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_numberbox _SettingsRow(ui_id id, element *page, char *label, f32 *num, char *suffix = "") {
   element *row = RowPanel(page, Size(Size(page).x, 20).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_numberbox result = _TextBox(id, row, num, 20);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_textbox _SettingsRow(ui_id id, element *page, char *label, TextBoxData *text, char *suffix = "") {
   element *row = RowPanel(page, Size(Size(page).x, 20).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_textbox result = _TextBox(id, row, text, 20);
   if(IsSelected(result.e))
      Outline(result.e, GREEN);
   Label(row, suffix, 20, BLACK);
   return result;
}

ui_checkbox _SettingsRow(ui_id id, element *page, char *label, u32 *value, u32 flag, 
                         v2 size, v2 padding = V2(0, 0), v2 margin = V2(0, 0))
{
   element *row = RowPanel(page, Size(Size(page).x, 20).Margin(0, 5));
   Label(row, label, 20, BLACK);
   ui_checkbox result = _CheckBox(id, row, value, flag, size, padding, margin);
   return result;
}

enum SettingsPage {
   SettingsPage_Main,
   SettingsPage_Load,
   SettingsPage_New
};

void DrawSettings(element *full_page, NorthSettings *state, 
                  v2 robot_size_ft, string robot_size_label,
                  FileListLink *ncff_files)
{
   StackLayout(full_page);
   element *page = VerticalList(full_page);
   
   element *top_panel = RowPanel(page, Size(Size(page).x - 10, 40).Padding(5, 5));
   Background(top_panel, dark_grey);

   static SettingsPage curr_page = SettingsPage_Main;
   if(Button(top_panel, "Load Field", menu_button.IsSelected(curr_page == SettingsPage_Load)).clicked) {
       curr_page = SettingsPage_Load;
   }

   if(Button(top_panel, "New Field", menu_button.IsSelected(curr_page == SettingsPage_New)).clicked) {
      curr_page = SettingsPage_New;
   }

   static f32 new_field_width = 0;
   static f32 new_field_height = 0;
   StaticTextBoxData(new_field_name, 15);
   static texture image_preview = {};
   static string image_path = {};
   static v2 image_size = V2(0, 0);    

   if(curr_page == SettingsPage_Main) {
      ui_button save_button = Button(top_panel, "Save", menu_button.IsEnabled(DifferentThanSaved(state))); 
      if(save_button.clicked) {
         UpdateSettingsFile(state);

         if(state->field.loaded)
            UpdateFieldFile(state);
      }

      //TODO: if save_button is hot, highlight things that are different than the current saved file

      Label(page, "Settings", 50, BLACK);
      SettingsRow(page, "Team Number: ", &state->team_number);
      
      if(state->field.loaded) {
         Label(page, state->field_name, 20, BLACK);
         Label(page, "Field Dimensions", 20, BLACK, V2(0, 0), V2(0, 5));
         
         SettingsRow(page, "Field Width: ", &state->field.size.x, "ft");
         SettingsRow(page, "Field Height: ", &state->field.size.y, "ft");
         
         SettingsRow(page, "Field Mirrored (eg. Steamworks)", &state->field.flags, Field_Flags::MIRRORED, V2(20, 20));
         SettingsRow(page, "Field Symmetric (eg. Power Up)", &state->field.flags, Field_Flags::SYMMETRIC, V2(20, 20));

         ui_field_topdown field = FieldTopdown(page, state->field.image, state->field.size,
                                               Clamp(0, Size(page->bounds).x, 700));
         Label(page, robot_size_label, 20, BLACK);

         v2 robot_size_px =  FeetToPixels(&field, robot_size_ft);
         for(u32 i = 0; i < state->field.starting_position_count; i++) {
            Field_StartingPosition *starting_pos = state->field.starting_positions + i;
            UI_SCOPE(page->context, starting_pos);

            element *field_starting_pos = Panel(field.e, RectCenterSize(GetPoint(&field, starting_pos->pos), robot_size_px), Captures(INTERACTION_DRAG));
            v2 direction_arrow = V2(cosf(starting_pos->angle * (PI32 / 180)), 
                                    -sinf(starting_pos->angle * (PI32 / 180)));
            Background(field_starting_pos, RED);
            Line(field_starting_pos, BLACK, 2,
               Center(field_starting_pos),
               Center(field_starting_pos) + 10 * direction_arrow);
            
            starting_pos->pos = ClampTo(starting_pos->pos + PixelsToFeet(&field, GetDrag(field_starting_pos)), 
                                       RectCenterSize(V2(0, 0), field.size_in_ft));
            
            element *starting_pos_panel = RowPanel(page, Size(Size(page).x, 40).Margin(0, 5));
            Background(starting_pos_panel, BLUE); 

            Label(starting_pos_panel, "X: ", 20, BLACK);
            TextBox(starting_pos_panel, &starting_pos->pos.x, 20);
            Label(starting_pos_panel, "Y: ", 20, BLACK);
            TextBox(starting_pos_panel, &starting_pos->pos.y, 20);
            Label(starting_pos_panel, "Angle: ", 20, BLACK);
            TextBox(starting_pos_panel, &starting_pos->angle, 20);

            if(Button(starting_pos_panel, "Delete", menu_button).clicked) {
               for(u32 j = i; j < (state->field.starting_position_count - 1); j++) {
                  state->field.starting_positions[j] = state->field.starting_positions[j + 1];
               }
               state->field.starting_position_count--;
            }

            if(IsHot(field_starting_pos) || ContainsCursor(starting_pos_panel)) {
               Outline(field_starting_pos, BLACK);
               Outline(starting_pos_panel, BLACK);
            }
         }

         if((state->field.starting_position_count + 1) < ArraySize(state->field.starting_positions)) {
            element *add_starting_pos = RowPanel(page, Size(Size(page->bounds).x, 40).Margin(0, 5).Captures(INTERACTION_CLICK));
            Background(add_starting_pos, RED);
            if(WasClicked(add_starting_pos)) {
               state->field.starting_position_count++;
               state->field.starting_positions[state->field.starting_position_count - 1] = {};
            }
         }
      } else {
         Label(page, Concat(state->field_name, Literal(".ncff not found")), 20, BLACK);
      }

      if(Button(top_panel, "Reload", menu_button).clicked) {
         ReadSettingsFile(state);
      }
   } else if(curr_page == SettingsPage_Load) {
      if(Button(top_panel, "Exit", menu_button.Padding(20, 0)).clicked) {
         curr_page = SettingsPage_Main;
      }
      
      for(FileListLink *file = ncff_files; file; file = file->next) {
         UI_SCOPE(page->context, file);
         element *field_panel = RowPanel(page, Size(Size(page).x - 5, 60).Padding(5, 5));
         Background(field_panel, light_grey);

         if(Button(field_panel, file->name, menu_button).clicked) {
            state->field_name = PushCopy(state->arena, file->name);
            UpdateSettingsFile(state);
         }
      }
   } else if(curr_page == SettingsPage_New) {      
      Label(page, "Create New Field File", 20, off_white);
      ui_textbox name_box = SettingsRow(page, "Name: ", &new_field_name);
      ui_numberbox width_box = SettingsRow(page, "Width: ", &new_field_width, "ft");
      ui_numberbox height_box = SettingsRow(page, "Height: ", &new_field_height, "ft");
      
      bool valid = (GetText(name_box).length > 0) &&
                   width_box.valid && (new_field_width > 0) &&
                   height_box.valid && (new_field_height > 0) &&
                   (image_path.length > 0) && (image_preview.handle != 0);
      
      if(Button(top_panel, "Create", menu_button.IsEnabled(valid)).clicked) {
         WriteNewFieldFile(GetText(name_box), new_field_width,
                           new_field_height, image_path);
         curr_page = SettingsPage_Main;
      }

      if(Button(top_panel, "Exit", menu_button.Padding(20, 0)).clicked) {
         new_field_width = 0;
         new_field_height = 0;
         Clear(name_box);

         curr_page = SettingsPage_Main;
      }

      element *image_drop = Panel(page, Size(400, 400).Layout(ColumnLayout).Captures(INTERACTION_FILEDROP));
      Background(image_drop, dark_grey);
      ui_dropped_files dropped_files = GetDroppedFiles(image_drop);
      
      if(dropped_files.count > 0) {
         image_path = PushCopy(state->arena, dropped_files.names[0]);
         deleteTexture(image_preview);
         image new_field_img = ReadImage(image_path);
         image_preview = createTexture(new_field_img.texels, new_field_img.width, new_field_img.height);
         image_size = V2(new_field_img.width, new_field_img.height);
         FreeImage(&new_field_img);
      }

      if(image_preview.handle == 0) { 
         Label(image_drop, "Drop Field Image File Here", Size(image_drop), 40, off_white);
      } else {
         Texture(image_drop, image_preview, RectCenterSize(Center(image_drop), AspectRatio(image_size, Size(image_drop))));
      }
   }
}
#endif