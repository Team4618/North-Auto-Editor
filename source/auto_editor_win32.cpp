#include "windows.h"
#include "gl/gl.h"
#include "lib/wglext.h"
#include "lib/glext.h"

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "lib/stb_truetype.h"

#define COMMON_PLATFORM
#include "lib/common.cpp"
#include "lib/ui_core.cpp"
#include "lib/ui_button.cpp"
#include "lib/ui_textbox.cpp"
#include "lib/ui_checkbox.cpp"
#include "lib/ui_vertical_list.cpp"
#include "lib/ui_horizontal_slider.cpp"
#include "lib/ui_multiline_graph.cpp"
#include "lib/ui_field_topdown.cpp"

#include "lib/ui_impl_win32_opengl.cpp"

#include "auto_editor.cpp"

int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
   Win32CommonInit(PlatformAllocArena(Megabyte(10), "Temp"));
   ui_impl_win32_window window = createWindow("Auto Editor");
   
   HANDLE hIcon = LoadImageA(0, "icon.ico", IMAGE_ICON, 0, 0, LR_DEFAULTSIZE | LR_LOADFROMFILE);
   if(hIcon) {
       SendMessageA(window.handle, WM_SETICON, ICON_SMALL, (LPARAM) hIcon);
       SendMessageA(window.handle, WM_SETICON, ICON_BIG, (LPARAM) hIcon);

       SendMessageA(GetWindow(window.handle, GW_OWNER), WM_SETICON, ICON_SMALL, (LPARAM) hIcon);
       SendMessageA(GetWindow(window.handle, GW_OWNER), WM_SETICON, ICON_BIG, (LPARAM) hIcon);
   }

   initTheme();

   UIContext ui_context = {};
   ui_context.frame_arena = PlatformAllocArena(Megabyte(2), "Frame Arena");
   ui_context.persistent_arena = PlatformAllocArena(Megabyte(2), "Persistent Arena");
   ui_context.filedrop_arena = PlatformAllocArena(Megabyte(2), "Filedrop Arena");
   ui_context.font = &theme_font;

   EditorState state = {};
   initEditor(&state);

   Timer timer = InitTimer();
   while(PumpMessages(&window, &ui_context)) {
      Reset(__temp_arena);
      
      state.directory_changed = CheckFiles(&state.file_watcher);

      element *root_element = beginFrame(window.size, &ui_context, GetDT(&timer));
      DrawUI(root_element, &state);
      endFrame(&window, root_element);
   }

   return 0;
}