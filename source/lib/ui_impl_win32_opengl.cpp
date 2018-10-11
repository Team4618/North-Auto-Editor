//NOTE: this needs windows.h, gl.h, wglext.h glext.h, stb_image, common & ui_core

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

struct image {
   u32 *texels;
   u32 width;
   u32 height;
   bool valid;
};

image ReadImage(char *path, bool in_exe_directory = false) {
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

image ReadImage(string path, bool in_exe_directory = false) {
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

sdfFont loadFont(char *metaPath, char *texturePath) {
   sdfFont result = {};
   result.sdfTexture = loadTexture(texturePath, true);
   
   buffer fontMeta = ReadEntireFile(metaPath, true);
   lexer metadata = { (char *) fontMeta.data };
   
   bool parsing = true;
   while(parsing) {
      token tkn = GetToken(&metadata);
      
      if(tkn.type == Token_End) {
         parsing = false;
      } else if(TokenIdentifier(tkn, Literal("char"))) {
         u32 id = 0;
         s32 x = 0;
         s32 y = 0;
         s32 width = 0;
         s32 height = 0;
         s32 xoffset = 0;
         s32 yoffset = 0;
         s32 xadvance = 0;
         
         while(true) {
            token first = GetToken(&metadata);
            
            if(first.type == Token_NewLine) {
               break;
            } else if(TokenIdentifier(first, Literal("id"))) {
               GetToken(&metadata); //Should always be =
               id = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("x"))) {
               GetToken(&metadata); //Should always be =
               x = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("y"))) {
               GetToken(&metadata); //Should always be =
               y = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("width"))) {
               GetToken(&metadata); //Should always be =
               width = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("height"))) {
               GetToken(&metadata); //Should always be =
               height = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("xoffset"))) {
               GetToken(&metadata); //Should always be =
               xoffset = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("yoffset"))) {
               GetToken(&metadata); //Should always be =
               yoffset = ParseS32(&metadata);
            } else if(TokenIdentifier(first, Literal("xadvance"))) {
               GetToken(&metadata); //Should always be =
               xadvance = ParseS32(&metadata);
            }
         }
         
         if(id < ArraySize(result.glyphs)) {
            glyphInfo glyph = {};
            glyph.textureLocation = V2(x, y);
            glyph.size = V2(width, height);
            glyph.offset = V2(xoffset, yoffset);
            glyph.xadvance = xadvance;
            result.glyphs[id] = glyph;

            result.max_char_width = Max(result.max_char_width, width);
         }
      } else if(TokenIdentifier(tkn, Literal("common"))) {
         while(true) {
            token first = GetToken(&metadata);
            
            if(first.type == Token_NewLine) {
               break;
            } else if(TokenIdentifier(first, Literal("lineHeight"))) {
               GetToken(&metadata); //Should always be =
               result.native_line_height = ParseS32(&metadata);
            }
         }
      } else if(TokenIdentifier(tkn, Literal("info"))) {
         
      }
   }
   
   FreeEntireFile(&fontMeta);
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
   } tex;
   
   struct {
      GLuint handle;
      GLint matrix_uniform; 
      GLint texture_uniform;
      GLint colour_uniform; 
   } sdf;
   
   GLuint buffers[3];
};

openglContext gl = {};

struct ui_impl_win32_window {
   HWND handle;
};

ui_impl_win32_window createWindow(char *title, WNDPROC window_proc) {
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
   GLuint sdfFragment = loadShader("sdf.frag", GL_FRAGMENT_SHADER);
   
   gl.col.handle = shaderProgram(transformVert, colourFragment);
   gl.col.matrix_uniform = glGetUniformLocation(gl.col.handle, "Matrix");
   gl.col.colour_uniform = glGetUniformLocation(gl.col.handle, "Colour");
   
   gl.tex.handle = shaderProgram(transformWithUVVert, textureFragment);
   gl.tex.matrix_uniform = glGetUniformLocation(gl.tex.handle, "Matrix");
   gl.tex.texture_uniform = glGetUniformLocation(gl.tex.handle, "Texture");
   
   gl.sdf.handle = shaderProgram(transformWithUVVert, sdfFragment);
   gl.sdf.matrix_uniform = glGetUniformLocation(gl.sdf.handle, "Matrix");
   gl.sdf.texture_uniform = glGetUniformLocation(gl.sdf.handle, "Texture");
   gl.sdf.colour_uniform = glGetUniformLocation(gl.sdf.handle, "Colour");
   
   glCreateVertexArrays(1, &gl.vao);
   glBindVertexArray(gl.vao);
   
   glVertexArrayAttribFormat(gl.vao, position_slot, 2, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, position_slot, position_slot);
   
   glVertexArrayAttribFormat(gl.vao, uv_slot, 2, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, uv_slot, uv_slot);
   
   glVertexArrayAttribFormat(gl.vao, colour_slot, 4, GL_FLOAT, GL_FALSE, 0);
   glVertexArrayAttribBinding(gl.vao, colour_slot, colour_slot);
   
   glCreateBuffers(3, gl.buffers);

   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glEnable(GL_SCISSOR_TEST);

   ui_impl_win32_window result = {};
   result.handle = window; 
   return result;
}

void DrawRenderCommandBuffer(RenderCommand *first_command, rect2 bounds, mat4 transform, openglContext *gl) {
   glScissor(bounds.min.x, window_size.y - bounds.max.y, Size(bounds).x, Size(bounds).y);

   for(RenderCommand *command = first_command; command; command = command->next) {
      switch(command->type) {
         case RenderCommand_SDF: {
            glUseProgram(gl->sdf.handle);
            glUniformMatrix4fv(gl->sdf.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->sdf.colour_uniform, 1, command->drawSDF.colour.e);
            glUniform1i(gl->sdf.texture_uniform, 0);
            glBindTextureUnit(0, command->drawSDF.sdf.handle);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glEnableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            
            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[0], 0, sizeof(v2));
            glVertexArrayVertexBuffer(gl->vao, uv_slot, gl->buffers[1], 0, sizeof(v2));
            
            rect2 bounds = command->drawSDF.bounds;
            v2 verts[6] = {
               bounds.min, bounds.min + YOf(Size(bounds)), bounds.max, 
               bounds.min, bounds.min + XOf(Size(bounds)), bounds.max
            };
            
            rect2 uvBounds = command->drawSDF.uvBounds;
            v2 uvs[6] = {
               uvBounds.min, uvBounds.min + YOf(Size(uvBounds)), uvBounds.max, 
               uvBounds.min, uvBounds.min + XOf(Size(uvBounds)), uvBounds.max
            };
            
            for(u32 i = 0; i < 6; i++)
               uvs[i] = uvs[i] / command->drawSDF.sdf.size;
            
            glNamedBufferData(gl->buffers[0], 2 * 3 * sizeof(v2), verts, GL_STREAM_DRAW);
            glNamedBufferData(gl->buffers[1], 2 * 3 * sizeof(v2), uvs, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, 2 * 3);
         } break;
         
         case RenderCommand_Texture: {
            glUseProgram(gl->tex.handle);
            glUniformMatrix4fv(gl->tex.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform1i(gl->tex.texture_uniform, 0);
            glBindTextureUnit(0, command->drawTexture.tex.handle);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glEnableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            
            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[0], 0, sizeof(v2));
            glVertexArrayVertexBuffer(gl->vao, uv_slot, gl->buffers[1], 0, sizeof(v2));
            
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
            
            glNamedBufferData(gl->buffers[0], 2 * 3 * sizeof(v2), verts, GL_STREAM_DRAW);
            glNamedBufferData(gl->buffers[1], 2 * 3 * sizeof(v2), uvs, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, 2 * 3);
         } break;
         
         case RenderCommand_Rectangle: {
            glUseProgram(gl->col.handle);
            glUniformMatrix4fv(gl->col.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->col.colour_uniform, 1, command->drawRectangle.colour.e);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glDisableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            
            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[0], 0, sizeof(v2));
            
            rect2 bounds = command->drawRectangle.bounds;
            v2 verts[6] = {
               bounds.min, bounds.min + YOf(Size(bounds)), bounds.max, 
               bounds.min, bounds.min + XOf(Size(bounds)), bounds.max
            };
            
            glNamedBufferData(gl->buffers[0], 2 * 3 * sizeof(v2), verts, GL_STREAM_DRAW);
            
            glDrawArrays(GL_TRIANGLES, 0, 2 * 3);
         } break;
         
         case RenderCommand_Line: {
            glUseProgram(gl->col.handle);
            glUniformMatrix4fv(gl->col.matrix_uniform, 1, GL_FALSE, transform.e);
            glUniform4fv(gl->col.colour_uniform, 1, command->drawLine.colour.e);
            glLineWidth(command->drawLine.thickness);
            
            glEnableVertexArrayAttrib(gl->vao, position_slot);
            glDisableVertexArrayAttrib(gl->vao, uv_slot);
            glDisableVertexArrayAttrib(gl->vao, colour_slot);
            
            glVertexArrayVertexBuffer(gl->vao, position_slot, gl->buffers[0], 0, sizeof(v2));
            
            v2 verts[2] = { command->drawLine.a, command->drawLine.b };
            
            glNamedBufferData(gl->buffers[0], 2 * sizeof(v2), verts, GL_STREAM_DRAW);
            
            glDrawArrays(GL_LINES, 0, 2);
         } break;
      }
   }
}

void DrawElement(element *e, mat4 transform, openglContext *gl) {
   DrawRenderCommandBuffer(e->first_command, e->cliprect, transform, gl);
   
   UIContext *context = e->context;
   v2 cursor = context->input_state.pos;

   if(IsActive(e)) {
      context->active_element_refreshed = true;
      context->active.last_pos = cursor;
   }

   if(IsSelected(e)) {
      context->selected_element_refreshed = true;
      context->selected.last_pos = cursor;
   }

   for(element *child = e->first_child; child; child = child->next) {
      DrawElement(child, transform, gl);
   }
}