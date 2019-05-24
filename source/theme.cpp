f32 status_bar_height = 20;
f32 page_tab_height = 40;

v4 dark_grey = V4(53/255.0, 56/255.0, 57/255.0, 1);
v4 light_grey = V4(89/255.0, 89/255.0, 89/255.0, 1);
v4 off_white = V4(238/255.0, 238/255.0, 238/255.0, 1);

button_style menu_button = ButtonStyle(
   dark_grey, light_grey, BLACK,
   light_grey, V4(120/255.0, 120/255.0, 120/255.0, 1), WHITE, 
   off_white, light_grey,
   page_tab_height, V2(0, 0), V2(0, 0));
texture logoTexture;
loaded_font theme_font;

void initTheme() {
   logoTexture = loadTexture("logo.png", true);
   
   MemoryArena *font_arena = PlatformAllocArena(Megabyte(5), "Font Arena");
   theme_font = loadFont(ReadEntireFile("OpenSans-Regular.ttf", true, font_arena), font_arena);
}

// element *_MyLabel(ui_id id, element *parent, string text, f32 line_height, 
//                   v2 p = V2(0, 0), v2 m = V2(0, 0))
// {
//    return _Label(id, parent, text, line_height, BLACK, p, m);
// }