#include "windows.h"
#include "gl/gl.h"
#include "lib/wglext.h"
#include "lib/glext.h"

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

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
#include "lib/ui_misc_utils.cpp"

#include "string_and_lexer.cpp"
v2 window_size = V2(0, 0);
#include "lib/ui_impl_win32_opengl.cpp"

LRESULT CALLBACK WindowMessageEvent(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
   switch(message)
	{
		case WM_CLOSE:
			DestroyWindow(window);
         break;
		
		case WM_DESTROY:
			PostQuitMessage(0);
         break;
        
      case WM_SIZE:
      {
         RECT client_rect = {};
         GetClientRect(window, &client_rect);
         window_size = V2(client_rect.right, client_rect.bottom);
         glViewport(0, 0, window_size.x, window_size.y);
      } break;
	}
   
   return DefWindowProc(window, message, wParam, lParam);
}

int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
   //NOTE: 
   //we load app-crucial files (eg. shaders, font) from the directory that the exe is in, 
   //we load user files (eg. settings, saves) from the working directory   

   __temp_arena = PlatformAllocArena(Megabyte(10));

   char exepath[MAX_PATH + 1];
   if(0 == GetModuleFileNameA(0, exepath, MAX_PATH + 1))
      Assert(false);
   exe_directory = Literal(exepath);
   for(u32 i = exe_directory.length - 1; i >= 0; i--) {
      if((exe_directory.text[i] == '\\') || (exe_directory.text[i] == '/'))
         break;

      exe_directory.length--;
   }

   ui_impl_win32_window window = createWindow("North Auto Editor", WindowMessageEvent);
   
   //TODO: does this load exe-relative or from the working directory?
   HANDLE hIcon = LoadImageA(0, "icon.ico", IMAGE_ICON, 0, 0, LR_DEFAULTSIZE | LR_LOADFROMFILE);
   if(hIcon) {
       SendMessageA(window.handle, WM_SETICON, ICON_SMALL, (LPARAM) hIcon);
       SendMessageA(window.handle, WM_SETICON, ICON_BIG, (LPARAM) hIcon);

       SendMessageA(GetWindow(window.handle, GW_OWNER), WM_SETICON, ICON_SMALL, (LPARAM) hIcon);
       SendMessageA(GetWindow(window.handle, GW_OWNER), WM_SETICON, ICON_BIG, (LPARAM) hIcon);
   } else {
      MessageBox(window.handle, "Error", "Icon Not Found", 0);
   }

   bool running = true;

   ShowWindow(window.handle, nCmdShow);
   UpdateWindow(window.handle);
   
   UIContext ui_context = {};
   ui_context.frame_arena = PlatformAllocArena(Megabyte(2));
   ui_context.persistent_arena = PlatformAllocArena(Megabyte(2));
   //ui_context.font = &font;
   
   LARGE_INTEGER frequency;
   QueryPerformanceFrequency(&frequency); 

   LARGE_INTEGER timer;
   QueryPerformanceCounter(&timer);
   
   while(running) {
      LARGE_INTEGER new_time;
      QueryPerformanceCounter(&new_time);
      f32 dt = (f32)(new_time.QuadPart - timer.QuadPart) / (f32)frequency.QuadPart;
      timer = new_time;
      
      POINT cursor = {};
      GetCursorPos(&cursor);
      ScreenToClient(window.handle, &cursor);
      
      ui_context.input_state.pos.x = cursor.x;
      ui_context.input_state.pos.y = cursor.y;
      ui_context.input_state.scroll = 0;
   
      ui_context.input_state.left_up = false;
      ui_context.input_state.right_up = false;

      ui_context.input_state.key_char = 0;
      ui_context.input_state.key_backspace = false;
      ui_context.input_state.key_enter = false;
      ui_context.input_state.key_up_arrow = false;
      ui_context.input_state.key_down_arrow = false;
      ui_context.input_state.key_left_arrow = false;
      ui_context.input_state.key_right_arrow = false;
      
      MSG msg = {};
      while(PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
         switch(msg.message) {
            case WM_QUIT:
               running = false;
               break;
         
            case WM_LBUTTONUP:
               ui_context.input_state.left_down = false;
               ui_context.input_state.left_up = true;
               break;
         
            case WM_LBUTTONDOWN:
               ui_context.input_state.left_down = true;
               ui_context.input_state.left_up = false;
               break;
               
            case WM_RBUTTONUP:
               ui_context.input_state.right_down = false;
               ui_context.input_state.right_up = true;
               break;
         
            case WM_RBUTTONDOWN:
               ui_context.input_state.right_down = true;
               ui_context.input_state.right_up = false;
               break;   

            case WM_CHAR: {
               switch(msg.wParam) {
                  case 0x08:
                     ui_context.input_state.key_backspace = true;
                     break;
                  case 0x0D:
                     ui_context.input_state.key_enter = true;
                     break;
                  case 0x0A:  // linefeed 
                  case 0x1B:  // escape 
                  case 0x09:  // tab 
                     break;
                  default:
                     ui_context.input_state.key_char = (char) msg.wParam;
                     break;
               }
            } break;

            case WM_KEYDOWN: {
               switch(msg.wParam) {
                  case VK_UP:
                     ui_context.input_state.key_up_arrow = true;
                     break;
                  case VK_DOWN:
                     ui_context.input_state.key_down_arrow = true;
                     break;
                  case VK_LEFT:
                     ui_context.input_state.key_left_arrow = true;
                     break;
                  case VK_RIGHT:
                     ui_context.input_state.key_right_arrow = true;
                     break;
               }
            } break;

            case WM_MOUSEWHEEL: {
               ui_context.input_state.scroll = GET_WHEEL_DELTA_WPARAM(msg.wParam);
            } break;
         }
         
         TranslateMessage(&msg);
         DispatchMessageA(&msg);
      }
      
      Reset(&__temp_arena);

      glScissor(0, 0, window_size.x, window_size.y);
      glClearColor(1, 1, 1, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      
      mat4 transform = Orthographic(0, window_size.y, 0, window_size.x, 100, -100);
      
      element *root_element = beginFrame(window_size, &ui_context, dt);
      DrawElement(root_element, transform, &gl);
      
      if(ui_context.tooltip.length > 0) {
         element tooltip_element = {};
         tooltip_element.context = &ui_context;
         
         Text(&tooltip_element, ui_context.tooltip, ui_context.input_state.pos - V2(0, 20), 20);
         DrawRenderCommandBuffer(tooltip_element.first_command,
                                 RectMinSize(V2(0, 0), window_size), transform, &gl);
      }

      SwapBuffers(gl.dc);
   }
   
   return 0;
}