//NOTE: this needs windows.h, gl.h, wglext.h glext.h, stb_image, stb_truetype common & ui_core

/**
TODO: RENDERER REWRITE
   freelist of opengl buffers?
   seperate lists for opaque & transparent geometry
      opaque geometry can be drawn in one pass
      transparent geometry must be sorted and drawn in multiple passes
   
   seperate threads for:
      window message handling (wait on GetMessage)
      rendering
      ui
*/

PFNGLCREATEVERTEXARRAYSPROC glCreateVertexArrays;
PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays;
PFNGLBINDVERTEXARRAYPROC glBindVertexArray;

PFNGLCREATESHADERPROC glCreateShader;
PFNGLDELETESHADERPROC glDeleteShader;
PFNGLSHADERSOURCEPROC glShaderSource;
PFNGLCOMPILESHADERPROC glCompileShader;
PFNGLGETSHADERIVPROC glGetShaderiv;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog;

PFNGLCREATEPROGRAMPROC glCreateProgram;
PFNGLDELETEPROGRAMPROC glDeleteProgram;
PFNGLATTACHSHADERPROC glAttachShader;
PFNGLDETACHSHADERPROC glDetachShader;
PFNGLLINKPROGRAMPROC glLinkProgram;
PFNGLUSEPROGRAMPROC glUseProgram;
PFNGLGETPROGRAMIVPROC glGetProgramiv;
PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog;

PFNGLCREATEBUFFERSPROC glCreateBuffers;
PFNGLDELETEBUFFERSPROC glDeleteBuffers;
PFNGLBINDBUFFERPROC glBindBuffer;
PFNGLNAMEDBUFFERDATAPROC glNamedBufferData;

PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation;
PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv;
PFNGLUNIFORM4FVPROC glUniform4fv;
PFNGLUNIFORM1FPROC glUniform1f;

PFNGLVERTEXARRAYVERTEXBUFFERPROC glVertexArrayVertexBuffer;
PFNGLENABLEVERTEXARRAYATTRIBPROC glEnableVertexArrayAttrib;
PFNGLDISABLEVERTEXARRAYATTRIBPROC glDisableVertexArrayAttrib;
PFNGLVERTEXARRAYATTRIBBINDINGPROC glVertexArrayAttribBinding;
PFNGLVERTEXARRAYATTRIBFORMATPROC glVertexArrayAttribFormat;
PFNGLVERTEXARRAYELEMENTBUFFERPROC glVertexArrayElementBuffer;

PFNGLUNIFORM1IPROC glUniform1i;
PFNGLBINDTEXTUREUNITPROC glBindTextureUnit;
PFNGLCREATETEXTURESPROC glCreateTextures;
PFNGLTEXTURESTORAGE2DPROC glTextureStorage2D;
PFNGLTEXTURESUBIMAGE2DPROC glTextureSubImage2D;
PFNGLTEXTUREPARAMETERIPROC glTextureParameteri;

PFNGLDEBUGMESSAGECALLBACKPROC glDebugMessageCallback;

image ReadImage(char *path, bool in_exe_directory) {
   image result = {};
   buffer file = ReadEntireFile(path, in_exe_directory);

   if(file.data != NULL) {
      s32 width, height, channels;
      u32 *data = (u32 *) stbi_load_from_memory((u8 *) file.data, file.size,
                                                &width, &height, &channels, 0);
      result.width = width;
      result.height = height;
      result.texels = data;
      result.valid = true;

      FreeEntireFile(&file);
   }

   return result;
}

image ReadImage(string path, bool in_exe_directory) {
   return ReadImage(ToCString(path), in_exe_directory);
}

void FreeImage(image *i) {
   if(i->valid)
      stbi_image_free(i->texels);
}

texture loadTexture(char *path, bool in_exe_directory) {
   texture result = {};
   image img = ReadImage(path, in_exe_directory);
   if(img.valid)
      result = createTexture(img.texels, img.width, img.height);
   FreeImage(&img);
   return result;
}

texture createTexture(u32 *texels, u32 width, u32 height) {
   texture result = {};
   result.size = V2(width, height);
      
   glCreateTextures(GL_TEXTURE_2D, 1, &result.handle);
   glTextureStorage2D(result.handle, 1, GL_RGBA8, width, height);
   glTextureSubImage2D(result.handle, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, texels);
   glTextureParameteri(result.handle, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTextureParameteri(result.handle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   
   return result;
}

void deleteTexture(texture tex) {
   if(tex.handle != 0)
      glDeleteTextures(1, &tex.handle); 
}

glyph_texture *getOrLoadGlyph(loaded_font *font, u32 codepoint) {
   if(font->glyphs[codepoint] == NULL) {
      glyph_texture *new_glyph = PushStruct(&font->arena, glyph_texture);
      new_glyph->codepoint = codepoint;
      
      //TODO: make sure this works as expected
      TempArena temp_arena;

      f32 line_height = 100;    
      f32 scale = stbtt_ScaleForPixelHeight(&font->fontinfo, line_height);
      
      //NOTE: stb_truetype has SDF generation so we _could_ use that 
      s32 w, h;
      u8 *mono = stbtt_GetCodepointBitmap(&font->fontinfo, 0, scale, codepoint, &w, &h, 0, 0);
      u32 *rgba = PushArray(&temp_arena.arena, u32, w * h);
      
      u8 *mono_curr = mono;
      u32 *rgba_curr = rgba;
      for(u32 y = 0; y < h; y++) {
         for(u32 x = 0; x < w; x++) {
            u32 mono_val = *mono_curr;
            *rgba_curr = (mono_val << 0) | (mono_val << 8) | (mono_val << 16) | (mono_val << 24);
            
            mono_curr++;
            rgba_curr++;
         }  
      }
      stbtt_FreeBitmap(mono, 0);

      GLuint handle = 0;
      glCreateTextures(GL_TEXTURE_2D, 1, &handle);
      glTextureStorage2D(handle, 1, GL_RGBA8, w, h);
      glTextureSubImage2D(handle, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
      glTextureParameteri(handle, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTextureParameteri(handle, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      
      new_glyph->tex.size = V2(w, h);
      new_glyph->tex.handle = handle;
      new_glyph->size_over_line_height = V2(w, h) / line_height;
      
      s32 xadvance, left_side_bearing;
      stbtt_GetCodepointHMetrics(&font->fontinfo, codepoint, &xadvance, &left_side_bearing);
      s32 x0, y0, x1, y1;
      stbtt_GetCodepointBox(&font->fontinfo, codepoint, &x0, &y0, &x1, &y1);

      new_glyph->xadvance_over_line_height = (xadvance * scale) / line_height;
      new_glyph->ascent_over_line_height = (-y1 * scale) / line_height;

      font->glyphs[codepoint] = new_glyph;
   }

   return font->glyphs[codepoint];
}

loaded_font loadFont(buffer ttf_file, MemoryArena arena) {
   loaded_font result = {};
   result.arena = arena;
   stbtt_InitFont(&result.fontinfo, ttf_file.data, 
                  stbtt_GetFontOffsetForIndex(ttf_file.data, 0));
   
   f32 line_height = 100;    
   f32 scale = stbtt_ScaleForPixelHeight(&result.fontinfo, line_height);
   s32 ascent, descent, lineGap;
   stbtt_GetFontVMetrics(&result.fontinfo, &ascent, &descent, &lineGap);
   result.baseline_from_top_over_line_height = (ascent * scale) / line_height;

   return result;
}

GLuint loadShader(char *path, GLenum shaderType) {
   buffer shader_src = ReadEntireFile(path, true);
   GLuint shader = glCreateShader(shaderType);
   glShaderSource(shader, 1, (char **) &shader_src.data, (GLint *) &shader_src.size);
   glCompileShader(shader);
   
   GLint compiled = 0;
   glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
   OutputDebugStringA(path);
   OutputDebugStringA(compiled ? " Compiled\n" : " Didn't Compile\n");
   if(compiled == GL_FALSE)
   {
      GLint log_length = 0;
      glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);

      char *compile_log = (char *) malloc(sizeof(char) * log_length);
      glGetShaderInfoLog(shader, log_length, &log_length, compile_log);
      OutputDebugStringA(compile_log);
   }
   
   FreeEntireFile(&shader_src);
   return shader;
}

GLuint shaderProgram(GLuint vert, GLuint frag) {
   GLuint shader_program = glCreateProgram();
   glAttachShader(shader_program, vert);
   glAttachShader(shader_program, frag);
   glLinkProgram(shader_program);
   
   GLint shader_linked = 0;
   glGetProgramiv(shader_program, GL_LINK_STATUS, &shader_linked);
   OutputDebugStringA(shader_linked ? "Shader Program Linked\n" : "Shader Program Didn't Link\n");
   if(shader_linked == GL_FALSE)
   {
      GLint log_length = 0;
      glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &log_length);

      char *link_log = (char *) malloc(sizeof(char) * log_length);
      glGetProgramInfoLog(shader_program, log_length, &log_length, link_log);
      OutputDebugStringA(link_log);
   }
   
   return shader_program;
}

void opengl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity,
                           GLsizei length, const GLchar* message, const void* userParam)
{
   /*
   OutputDebugStringA("OGL Debug: ");
   OutputDebugStringA(message);
   OutputDebugStringA("\n");
   */
}

const GLuint position_slot = 0;
const GLuint uv_slot = 1;
const GLuint colour_slot = 2;
const GLuint normal_slot = 3;

struct openglContext {
   HDC dc;
   GLuint vao;
   
   struct {
      GLuint handle;
      GLint matrix_uniform;
      GLint colour_uniform;
   } col;
   
   struct {
      GLuint handle;
      GLint matrix_uniform; 
      GLint texture_uniform;
      GLint colour_uniform;
   } tex;
   
   struct {
      GLuint handle;
      GLint matrix_uniform;
      GLint colour_uniform;
      GLint width_uniform;
      GLint feather_uniform;
   } line;
   
   GLuint buffers[4];
};

struct ui_impl_win32_window {
   HWND handle;
   bool running;
   openglContext gl;
   v2 size;
};

//NOTE: these globals are so we can work around the windows callback thing being a nightmare
bool wm_size_recieved = false;
//touch event stuff

LRESULT CALLBACK impl_WindowMessageEvent(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
   switch(message) {
		case WM_CLOSE:
			DestroyWindow(window);
         break;
		
		case WM_DESTROY:
			PostQuitMessage(0);
         break;
        
      case WM_SIZE: {
         wm_size_recieved = true;
      } break;

      case WM_TOUCH: {
         OutputDebugStringA("Callback Touch\n");
         //TODO: touch compatability
         //https://docs.microsoft.com/en-us/windows/desktop/api/winuser/nf-winuser-registertouchwindow
      } break;
	}
   
   return DefWindowProc(window, message, wParam, lParam);
}

//TODO: cleanup all this opengl stuff once the actual apps are done

ui_impl_win32_window createWindow(char *title, WNDPROC window_proc = impl_WindowMessageEvent) {
   HINSTANCE hInstance = GetModuleHandle(NULL);
   
   WNDCLASSEX window_class = {};
   window_class.cbSize = sizeof(window_class);
   window_class.style = CS_OWNDC;
   window_class.lpfnWndProc = window_proc;
   window_class.hInstance = hInstance;
   window_class.hIcon = LoadIcon(NULL, IDI_APPLICATION);
   window_class.hCursor = LoadCursor(NULL, IDC_ARROW);
   window_class.lpszClassName = "WindowClass";
   
   RegisterClassExA(&window_class);
   
   HWND window = CreateWindowExA(WS_EX_CLIENTEDGE, "WindowClass", title,
                                 WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                                 CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL,
                                 hInstance, NULL);
   
   openglContext gl = {};
   gl.dc = GetDC(window);
   
   PIXELFORMATDESCRIPTOR pixel_format = {sizeof(pixel_format), 1};
   pixel_format.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
   pixel_format.iPixelType = PFD_TYPE_RGBA;
   pixel_format.cColorBits = 32;
   
   s32 pixel_format_index = ChoosePixelFormat(gl.dc, &pixel_format);
   SetPixelFormat(gl.dc, pixel_format_index, &pixel_format);
   
   HGLRC temp_gl_context = wglCreateContext(gl.dc);
   wglMakeCurrent(gl.dc, temp_gl_context);
   
   PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB =
      (PFNWGLCREATECONTEXTATTRIBSARBPROC) wglGetProcAddress("wglCreateContextAttribsARB");
   
   const int gl_attribs[] = 
   {
      WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
      WGL_CONTEXT_MINOR_VERSION_ARB, 5,
      WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
      WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB,
      0
   };

   HGLRC gl_context = wglCreateContextAttribsARB(gl.dc, 0, gl_attribs);
   wglMakeCurrent(gl.dc, gl_context);
   wglDeleteContext(temp_gl_context);
   
   OutputDebugStringA((char *) glGetString(GL_VERSION));
   OutputDebugStringA("\n");
   
   glCreateVertexArrays = (PFNGLCREATEVERTEXARRAYSPROC) wglGetProcAddress("glCreateVertexArrays");
   glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC) wglGetProcAddress("glDeleteVertexArrays");
   glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC) wglGetProcAddress("glBindVertexArray");
   
   glCreateShader = (PFNGLCREATESHADERPROC) wglGetProcAddress("glCreateShader");
   glDeleteShader = (PFNGLDELETESHADERPROC) wglGetProcAddress("glDeleteShader");
   glShaderSource = (PFNGLSHADERSOURCEPROC) wglGetProcAddress("glShaderSource");
   glCompileShader = (PFNGLCOMPILESHADERPROC) wglGetProcAddress("glCompileShader");
   glGetShaderiv = (PFNGLGETSHADERIVPROC) wglGetProcAddress("glGetShaderiv");
   glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC) wglGetProcAddress("glGetShaderInfoLog");

   glCreateProgram = (PFNGLCREATEPROGRAMPROC) wglGetProcAddress("glCreateProgram");
   glDeleteProgram = (PFNGLDELETEPROGRAMPROC) wglGetProcAddress("glDeleteProgram");
   glAttachShader = (PFNGLATTACHSHADERPROC) wglGetProcAddress("glAttachShader");
   glDetachShader = (PFNGLDETACHSHADERPROC) wglGetProcAddress("glDetachShader");
   glLinkProgram = (PFNGLLINKPROGRAMPROC) wglGetProcAddress("glLinkProgram");
   glUseProgram = (PFNGLUSEPROGRAMPROC) wglGetProcAddress("glUseProgram");
   glGetProgramiv = (PFNGLGETPROGRAMIVPROC) wglGetProcAddress("glGetProgramiv");
   glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC) wglGetProcAddress("glGetProgramInfoLog");
   
   glCreateBuffers = (PFNGLCREATEBUFFERSPROC) wglGetProcAddress("glCreateBuffers");
   glDeleteBuffers = (PFNGLDELETEBUFFERSPROC) wglGetProcAddress("glDeleteBuffers");
   glBindBuffer = (PFNGLBINDBUFFERPROC) wglGetProcAddress("glBindBuffer");
   glNamedBufferData = (PFNGLNAMEDBUFFERDATAPROC) wglGetProcAddress("glNamedBufferData");

   glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC) wglGetProcAddress("glGetUniformLocation");
   glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC) wglGetProcAddress("glUniformMatrix4fv");
   glUniform4fv = (PFNGLUNIFORM4FVPROC) wglGetProcAddress("glUniform4fv");
   glUniform1f = (PFNGLUNIFORM1FPROC) wglGetProcAddress("glUniform1f");

   glVertexArrayVertexBuffer = (PFNGLVERTEXARRAYVERTEXBUFFERPROC) wglGetProcAddress("glVertexArrayVertexBuffer");
   glEnableVertexArrayAttrib = (PFNGLENABLEVERTEXARRAYATTRIBPROC) wglGetProcAddress("glEnableVertexArrayAttrib");
   glDisableVertexArrayAttrib = (PFNGLDISABLEVERTEXARRAYATTRIBPROC) wglGetProcAddress("glDisableVertexArrayAttrib");
   glVertexArrayAttribBinding = (PFNGLVERTEXARRAYATTRIBBINDINGPROC) wglGetProcAddress("glVertexArrayAttribBinding");
   glVertexArrayAttribFormat = (PFNGLVERTEXARRAYATTRIBFORMATPROC) wglGetProcAddress("glVertexArrayAttribFormat");
   glVertexArrayElementBuffer = (PFNGLVERTEXARRAYELEMENTBUFFERPROC) wglGetProcAddress("glVertexArrayElementBuffer");
   
   glUniform1i = (PFNGLUNIFORM1IPROC) wglGetProcAddress("glUniform1i");
   glBindTextureUnit = (PFNGLBINDTEXTUREUNITPROC) wglGetProcAddress("glBindTextureUnit");
   
   glCreateTextures = (PFNGLCREATETEXTURESPROC) wglGetProcAddress("glCreateTextures");
   glTextureStorage2D = (PFNGLTEXTURESTORAGE2DPROC) wglGetProcAddress("glTextureStorage2D");
   glTextureSubImage2D = (PFNGLTEXTURESUBIMAGE2DPROC) wglGetProcAddress("glTextureSubImage2D");
   glTextureParameteri = (PFNGLTEXTUREPARAMETERIPROC) wglGetProcAddress("glTextureParameteri");

   glDebugMessageCallback = (PFNGLDEBUGMESSAGECALLBACKPROC) wglGetProcAddress("glDebugMessageCallback");
   glDebugMessageCallback((GLDEBUGPROC) opengl_debug_callback, NULL); //NOTE: this crashes in 32bit mode
   
   GLuint transformVert = loadShader("transform.vert", GL_VERTEX_SHADER);
   GLuint transformWithUVVert = loadShader("transform_UV.vert", GL_VERTEX_SHADER);
   GLuint colourFragment = loadShader("colour.frag", GL_FRAGMENT_SHADER);
   GLuint textureFragment = loadShader("texture.frag", GL_FRAGMENT_SHADER);
   
   gl.col.handle = shaderProgram(transformVert, colourFragment);
   gl.col.matrix_uniform = glGetUniformLocation(gl.col.handle, "Matrix");
   gl.col.colour_uniform = glGetUniformLocation(gl.col.handle, "Colour");
   
   gl.tex.handle = shaderProgram(transformWithUVVert, textureFragment);
   gl.tex.matrix_uniform = glGetUniformLocation(gl.tex.handle, "Matrix");
   gl.tex.texture_uniform = glGetUniformLocation(gl.tex.handle, "Texture");
   gl.tex.colour_uniform = glGetUniformLocation(gl.tex.handle, "Colour");
   
   GLuint lineVert = loadShader("line.vert", GL_VERTEX_SHADER);
   GLuint lineFrag = loadShader("line.frag", GL_FRAGMENT_SHADER);

   gl.line.handle = shaderProgram(lineVert, lineFrag);
   gl.line.matrix_uniform = glGetUniformLocation(gl.line.handle, "Matrix");
   gl.line.colour_uniform = glGetUniformLocation(gl.line.handle, "Colour");
   gl.line.width_uniform = glGetUniformLocation(gl.line.handle, "Width");
   gl.line.feather_uniform = glGetUniformLocation(gl.line.handle, "Feather");
   
   glCreateVertexArrays(1, &gl.vao);
   glBindVertexArray(gl.vao);
   
   glVertexArrayAttribFormat(gl.vao, position_slot, 2, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, position_slot, position_slot);
   
   glVertexArrayAttribFormat(gl.vao, uv_slot, 2, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, uv_slot, uv_slot);
   
   glVertexArrayAttribFormat(gl.vao, colour_slot, 4, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, colour_slot, colour_slot);
   
   glVertexArrayAttribFormat(gl.vao, normal_slot, 2, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, normal_slot, normal_slot);

   glCreateBuffers(ArraySize(gl.buffers), gl.buffers);

   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glEnable(GL_SCISSOR_TEST);

   DragAcceptFiles(window, true);
   RegisterTouchWindow(window, 0);

   ShowWindow(window, SW_SHOWNORMAL);
   UpdateWindow(window);

   ui_impl_win32_window result = {};
   result.handle = window; 
   result.running = true;
   result.gl = gl;
   return result;
}

void DrawRenderCommandBuffer(RenderCommand *first_command, rect2 bounds, mat4 transform, ui_impl_win32_window *window) {
   openglContext *gl = &window->gl;
   glScissor(bounds.min.x, window->size.y - bounds.max.y, Size(bounds).x, Size(bounds).y);

   for(RenderCommand *command = first_command; command; command = command->next) {
      switch(command->type) {
         case RenderCommand_Texture: {
            glUseProgram(gl->tex.handle);
            glUniformMatrix4fv(gl->tex.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->tex.colour_uniform, 1, command->drawTexture.colour.e);
            glUniform1i(gl->tex.texture_uniform, 0);
            glBindTextureUnit(0, command->drawTexture.tex.handle);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glEnableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            glDisableVertexArrayAttrib(gl->vao, normal_slot);
            
            rect2 bounds = command->drawTexture.bounds;
            v2 verts[6] = {
               bounds.min, bounds.min + YOf(Size(bounds)), bounds.max, 
               bounds.min, bounds.min + XOf(Size(bounds)), bounds.max
            };
            
            rect2 uvBounds = command->drawTexture.uvBounds;
            v2 uvs[6] = {
               uvBounds.min, uvBounds.min + YOf(Size(uvBounds)), uvBounds.max, 
               uvBounds.min, uvBounds.min + XOf(Size(uvBounds)), uvBounds.max
            };
            
            for(u32 i = 0; i < 6; i++)
               uvs[i] = uvs[i] / command->drawTexture.tex.size;

            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[position_slot], 0, sizeof(v2));
            glNamedBufferData(gl->buffers[position_slot], 2 * 3 * sizeof(v2), verts, GL_STREAM_DRAW);
            
            glVertexArrayVertexBuffer(gl->vao, uv_slot, gl->buffers[uv_slot], 0, sizeof(v2));
            glNamedBufferData(gl->buffers[uv_slot], 2 * 3 * sizeof(v2), uvs, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, 2 * 3);
         } break;
         
         case RenderCommand_Rectangle: {
            glUseProgram(gl->col.handle);
            glUniformMatrix4fv(gl->col.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->col.colour_uniform, 1, command->drawRectangle.colour.e);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glDisableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            glDisableVertexArrayAttrib(gl->vao, normal_slot);
            
            rect2 bounds = command->drawRectangle.bounds;
            v2 verts[6] = {
               bounds.min, bounds.min + YOf(Size(bounds)), bounds.max, 
               bounds.min, bounds.min + XOf(Size(bounds)), bounds.max
            };
            
            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[position_slot], 0, sizeof(v2));
            glNamedBufferData(gl->buffers[position_slot], 2 * 3 * sizeof(v2), verts, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, 2 * 3);
         } break;
         
         case RenderCommand_Line: {
            if(command->drawLine.point_count < 2)
               continue;

            glUseProgram(gl->line.handle);
            glUniformMatrix4fv(gl->line.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->line.colour_uniform, 1, command->drawLine.colour.e);
            glUniform1f(gl->line.width_uniform, command->drawLine.thickness);
            glUniform1f(gl->line.feather_uniform, 2);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glDisableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            glEnableVertexArrayAttrib(gl->vao, normal_slot);
            
            u32 vert_count = (command->drawLine.point_count - (command->drawLine.closed ? 0 : 1)) * 6;
            v2 *verts = PushTempArray(v2, vert_count);
            v2 *normals = PushTempArray(v2, vert_count);

            //TODO: fix line joints & clean this up
               
            v2 last_point = command->drawLine.points[0];
            for(u32 i = 1; i < command->drawLine.point_count; i++) {
               v2 point = command->drawLine.points[i];
               v2 line_normal = Normalize(Perp(point - last_point));
               v2 normal_a = line_normal;
               v2 normal_b = line_normal;

               // if(i > 1) {
               //    v2 prev_line_normal = Normalize(Perp(command->drawLine.points[i - 1] - point));
               //    normal_a = normal_a + prev_line_normal;
               // }

               // if(1 < (command->drawLine.point_count - 1)) {
               //    v2 next_line_normal = Normalize(Perp(point - command->drawLine.points[i + 1]));
               //    normal_b = normal_b + next_line_normal;
               // }

               v2 *vert = verts + 6 * (i - 1);
               v2 *normal = normals + 6 * (i - 1);
               
               vert[0] = last_point;
               vert[1] = last_point;
               vert[2] = point;
               vert[3] = last_point;
               vert[4] = point;
               vert[5] = point;

               normal[0] = normal_a;
               normal[1] = -normal_a;
               normal[2] = normal_b;
               normal[3] = -normal_a;
               normal[4] = normal_b;
               normal[5] = -normal_b;

               last_point = point;
            }

            if(command->drawLine.closed) {
               v2 point = command->drawLine.points[0];
               v2 line_normal = Normalize(Perp(point - last_point));
               v2 normal_a = line_normal;
               v2 normal_b = line_normal;

               v2 *vert = verts + 6 * (command->drawLine.point_count - 1);
               v2 *normal = normals + 6 * (command->drawLine.point_count - 1);
               
               //TODO: make a "write line" function or something
               vert[0] = last_point;
               vert[1] = last_point;
               vert[2] = point;
               vert[3] = last_point;
               vert[4] = point;
               vert[5] = point;

               normal[0] = normal_a;
               normal[1] = -normal_a;
               normal[2] = normal_b;
               normal[3] = -normal_a;
               normal[4] = normal_b;
               normal[5] = -normal_b;

               last_point = point;
            }

            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[position_slot], 0, sizeof(v2));
            glNamedBufferData(gl->buffers[position_slot], vert_count * sizeof(v2), verts, GL_STREAM_DRAW);
            
            glVertexArrayVertexBuffer(gl->vao, normal_slot, gl->buffers[normal_slot], 0, sizeof(v2));
            glNamedBufferData(gl->buffers[normal_slot], vert_count * sizeof(v2), normals, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, vert_count);
         } break;
      }
   }
}

void DrawElement(element *e, mat4 transform, ui_impl_win32_window *window) {
   DrawRenderCommandBuffer(e->first_command, e->cliprect, transform, window);
   
   uiTick(e);

   for(element *child = e->first_child; child; child = child->next) {
      DrawElement(child, transform, window);
   }
}

bool PumpMessages(ui_impl_win32_window *window, UIContext *ui) {
   InputState *input = &ui->input_state;
   POINT cursor = {};
   GetCursorPos(&cursor);
   ScreenToClient(window->handle, &cursor);
   
   input->last_pos = input->pos;
   input->pos = V2(cursor.x, cursor.y);
   input->vscroll = 0;
   input->hscroll = 0;

   input->left_up = false;
   input->right_up = false;

   input->key_char = 0;
   input->key_backspace = false;
   input->key_enter = false;
   input->key_up_arrow = false;
   input->key_down_arrow = false;
   input->key_left_arrow = false;
   input->key_right_arrow = false;
   input->key_esc = false;
   input->alt_down = GetKeyState(VK_MENU) & 0x8000;
   input->ctrl_down = GetKeyState(VK_CONTROL) & 0x8000;

   MSG msg = {};
   while(PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
      switch(msg.message) {
         case WM_QUIT:
            window->running = false;
            break;
      
         case WM_LBUTTONUP:
            input->left_down = false;
            input->left_up = true;
            break;
      
         case WM_LBUTTONDOWN:
            input->left_down = true;
            input->left_up = false;
            break;
            
         case WM_RBUTTONUP:
            input->right_down = false;
            input->right_up = true;
            break;
      
         case WM_RBUTTONDOWN:
            input->right_down = true;
            input->right_up = false;
            break;   

         case WM_CHAR: {
            switch(msg.wParam) {
               case 0x08:
                  input->key_backspace = true;
                  break;
               case 0x0D:  // enter
               case 0x0A:  // linefeed 
               case 0x1B:  // escape 
               case 0x09:  // tab 
                  break;
               default:
                  input->key_char = (char) msg.wParam;
                  break;
            }
         } break;

         case WM_KEYDOWN: {
            switch(msg.wParam) {
               case VK_UP:
                  input->key_up_arrow = true;
                  break;
               case VK_DOWN:
                  input->key_down_arrow = true;
                  break;
               case VK_LEFT:
                  input->key_left_arrow = true;
                  break;
               case VK_RIGHT:
                  input->key_right_arrow = true;
                  break;
               case VK_RETURN:
                  input->key_enter = true;
                  break;
            }
         } break;

         case WM_KEYUP: {
            switch(msg.wParam) {
               case VK_F1: {
                  if(ui->debug_mode == UIDebugMode_Disabled) {
                     ui->debug_mode = UIDebugMode_ElementPick;
                  } else {
                     ui->debug_mode = UIDebugMode_Disabled;
                  }
               } break;

               case VK_F2: {
                  if(ui->debug_mode == UIDebugMode_Disabled) {
                     ui->debug_mode = UIDebugMode_Memory;
                  } else {
                     ui->debug_mode = UIDebugMode_Disabled;
                  }
               } break;

               case VK_ESCAPE:
                  input->key_esc = true;
                  break;
            }
         } break;

         case WM_MOUSEWHEEL: {
            input->vscroll = GET_WHEEL_DELTA_WPARAM(msg.wParam);
         } break;

         case WM_MOUSEHWHEEL: {
            input->hscroll = GET_WHEEL_DELTA_WPARAM(msg.wParam);
         } break;

         case WM_DROPFILES: {
            HDROP drop = (HDROP) msg.wParam;
            MemoryArena *arena = &ui->filedrop_arena;
            Reset(arena);

            u32 file_count = DragQueryFileA(drop, 0xFFFFFFFF, NULL, 0);
            string *files = PushArray(arena, string, file_count);

            for(u32 i = 0; i < file_count; i++) {
               u32 size = DragQueryFileA(drop, i, NULL, 0) + 1;
               string *file = files + i;
               
               *file = String(PushArray(arena, char, size), size - 1);
               DragQueryFileA(drop, i, file->text, size);
            }
            DragFinish(drop);

            ui->filedrop_count = file_count;
            ui->filedrop_names = files;
         } break;

         default: {
            string msg_name;
            switch(msg.message) {
               case 275: {
                  msg_name = Literal("Timer");
               } break;

               case 512: {
                  msg_name = Literal("Mouse move");
               } break;

               default: {
                  msg_name = ToString(msg.message);
               } break;
            }

            string text = Concat(Literal("Unhandled Message "), msg_name, Literal("\n"));
            // OutputDebugStringA(ToCString(text));
         } break;
      }
      
      TranslateMessage(&msg);
      DispatchMessageA(&msg);
   }
   
   if(wm_size_recieved) {
      RECT client_rect = {};
      GetClientRect(window->handle, &client_rect);
      window->size = V2(client_rect.right, client_rect.bottom);
      glViewport(0, 0, window->size.x, window->size.y);
      wm_size_recieved = false;
   }

   return window->running;
}

void endFrame(ui_impl_win32_window *window, element *root) {
   UIContext *context = root->context;
   InputState *input = &context->input_state;
   Reset(&context->filedrop_arena);
   context->filedrop_count = 0;
   context->filedrop_names = NULL;
   
   glScissor(0, 0, window->size.x, window->size.y);
   glClearColor(1, 1, 1, 1);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   mat4 transform = Orthographic(0, window->size.y, 0, window->size.x, 100, -100);
   DrawElement(root, transform, window);
   DrawRenderCommandBuffer(context->overlay->first_command, context->overlay->cliprect, transform, window);
   
   element *debug_root = PushStruct(&context->frame_arena, element);
   debug_root->context = context;
   debug_root->bounds = RectMinSize(V2(0, 0), window->size);
   debug_root->cliprect = RectMinSize(V2(0, 0), window->size);
   debug_root->layout_flags = Layout_Width | Layout_Height | Layout_Placed;
   ColumnLayout(debug_root);

   button_style menu_button = ButtonStyle(
      V4(53/255.0, 56/255.0, 57/255.0, 1), V4(89/255.0, 89/255.0, 89/255.0, 1), BLACK,
      V4(89/255.0, 89/255.0, 89/255.0, 1), V4(120/255.0, 120/255.0, 120/255.0, 1), WHITE, 
      WHITE, V4(89/255.0, 89/255.0, 89/255.0, 1),
      30, V2(0, 0), V2(0, 0));

   switch(context->debug_mode) {
      case UIDebugMode_ElementPick: {
         if(context->debug_hot_e != NULL) {
            string hot_e_loc = Literal(context->debug_hot_e->id.loc);
            rect2 bounds = context->debug_hot_e->bounds;
            v2 padding = context->debug_hot_e->padding;
            v2 margin = context->debug_hot_e->margin;

            Outline(debug_root, bounds, BLACK, 2);
            Outline(debug_root, RectMinMax(bounds.min - padding, bounds.max + padding), BLACK, 2);
            Outline(debug_root, RectMinMax(bounds.min - padding - margin, bounds.max + padding + margin), BLACK, 2);

            if(input->left_up) {
               context->debug_selected = context->debug_hot_e->id;
               context->debug_mode = UIDebugMode_ElementSelected;
            }
         }
      } break;

      case UIDebugMode_ElementSelected: {
         element *debug_selected_e = context->debug_selected_e;
         Label(debug_root, context->debug_selected.loc, 30, WHITE);

         if(debug_selected_e == NULL) {
            Label(debug_root, "Selected ID not drawn", 30, WHITE);
         } else {
            Outline(debug_root, debug_selected_e->bounds, BLACK, 2);
            
            if(debug_selected_e->parent != NULL) {
               element *parent = debug_selected_e->parent;
               ui_button parent_button = Button(debug_root, "Parent", menu_button);
               
               if(IsHot(parent_button.e)) {
                  Outline(debug_root, parent->bounds, BLACK, 2);
               }
               
               if(parent_button.clicked) {
                  context->debug_selected = parent->id;
               }
            }

            for(element *child = debug_selected_e->first_child; 
                child; child = child->next)
            {
               UI_SCOPE(context, child);
               ui_button child_button = Button(debug_root, "Child", menu_button);
               
               if(IsHot(child_button.e)) {
                  Outline(debug_root, child->bounds, BLACK, 2);
               }
               
               if(child_button.clicked) {
                  context->debug_selected = child->id;
               }
            }
         }
      } break;

      case UIDebugMode_Memory: {
         Label(debug_root, Concat(Literal("Time: "), ToString((f32) context->curr_time)), 20, WHITE);
         Label(debug_root, Concat(Literal("FPS: "), ToString((f32) context->fps)), 20, WHITE);
         Label(debug_root, Concat(ToString(arena_blocks_allocated), Literal(" Arena Blocks Allocated")), 20, WHITE);
         Label(debug_root, Concat(ToString(arenas_allocated), Literal(" Arenas Allocated")), 20, WHITE);

         string memory_units[] = {
            Literal(" Bytes"),
            Literal(" KB"),
            Literal(" MB"),
            Literal(" GB")
         };

         u64 memory_value = total_size_allocated;
         u32 memory_unit_index = 0;

         while(memory_value >= 1024) {
            memory_value /= 1024;
            memory_unit_index++;
         }
   
         Assert(memory_unit_index < ArraySize(memory_units));
         Label(debug_root, Concat(ToString((u32) memory_value), memory_units[memory_unit_index], Literal(" Allocated")), 20, WHITE);
      
         //TODO: per arena diagnostics
      } break;
   }
   
   rect2 background_bounds = RectMinSize(V2(0, 0), V2(0, 0));
   for(element *child = debug_root->first_child; 
       child; child = child->next)
   {
      background_bounds = Union(background_bounds, child->bounds);
   }
   Rectangle(debug_root, background_bounds, V4(0, 0, 0, 0.75));   
   DrawElement(debug_root, transform, window);

   SwapBuffers(window->gl.dc);
}