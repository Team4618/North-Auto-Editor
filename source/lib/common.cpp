#include "stdint.h"
typedef int8_t s8;
typedef uint8_t u8;
typedef int16_t s16;
typedef uint16_t u16;
typedef int32_t s32;
typedef uint32_t u32;
typedef int64_t s64;
typedef uint64_t u64;
typedef float f32;
#define F32_MAX 3.402823e+38
typedef double f64;

#define Assert(condition) if(!(condition)){*(char *)0 = 0;}
#define ArraySize(array) (sizeof(array) / sizeof((array)[0]))
#define Min(a, b) (((a) > (b)) ? (b) : (a))
#define Max(a, b) (((a) < (b)) ? (b) : (a))
#define Clamp(min, max, in) Min(Max(min, in), max)

#define Kilobyte(BYTE) BYTE * 1024
#define Megabyte(BYTE) Kilobyte(BYTE) * 1024
#define Gigabyte(BYTE) Megabyte(BYTE) * 1024

#define RIFF_CODE(str) ( ((str)[0] << 0) | ((str)[1] << 8) | ((str)[2] << 16) | ((str)[3] << 24) )

#define ZeroStruct(var) _Zero((u8 *) (var), sizeof(*(var)))
void _Zero(u8 *data, u32 size) {
   for(u32 i = 0; i < size; i++) {
      data[i] = 0;
   }
}

#define ForEachArray(index, name, count, array, code) do { for(u32 index = 0; index < (count); index++){ auto name = (array) + index; code } } while(false)

void Copy(void *src_in, u32 size, void *dest_in) {
   u8 *src = (u8 *) src_in;
   u8 *dest = (u8 *) dest_in;
   for(u32 i = 0; i < size; i++) {
      dest[i] = src[i];
   }
}

//TODO: utf-8
struct string {
   char *text;
   u32 length;
};

const string EMPTY_STRING = {};

string String(char *text, u32 length) {
   string result = {};
   result.text = text;
   result.length = length;
   return result;
}

u32 StringLength(char *text) {
   u32 length = 0;
   while(*text) {
      length++;
      text++;
   }

   return length;
}

#define ArrayLiteral(array) String((array), ArraySize(array))
string Literal(char *text) {
   string result = {};

   result.text = text;
   result.length = StringLength(text);

   return result;
}

//TODO: better hash function
u32 Hash(string s) {
   u32 result = 1;
   for(u32 i = 0; i < s.length; i++) {
      result = 7 * s.text[i] + 3 * result; 
   }
   return result;
}

bool operator== (string a, string b) {
   if(a.length != b.length)
      return false;

   for(u32 i = 0; i < a.length; i++) {
      if(a.text[i] != b.text[i])
         return false;
   }

   return true;
} 

bool operator!=(string a, string b) {
   return !(a == b);
}

bool IsWhitespace(char c) {
   return ((c == ' ') || (c == '\r') || (c == '\t'));
}

bool IsAlpha(char c) {
   return (((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z')));
}

bool IsNumeric(char c) {
   return ((c >= '0') && (c <= '9'));
}

bool IsNumber(string s) {
   bool has_dot = false;
   bool hit_whitespace = false;
   for(u32 i = 0; i < s.length; i++) {
      char c = s.text[i];
      bool is_whitespace = IsWhitespace(c) || (c == '\0');
      
      if(hit_whitespace && !is_whitespace)
         return false;

      if(c == '-') {
         if((i != 0) || (s.length == 1))
            return false;
      } else if(c == '.') {
         if(has_dot)
            return false;

         if(i == (s.length - 1))
            return false;

         has_dot = true;
      } else if(is_whitespace) {
         hit_whitespace = true;
      } else if(!IsNumeric(c)) {
         return false;
      }
   }
   return true;
}

bool IsUInt(string s) {
   bool hit_whitespace = false;
   for(u32 i = 0; i < s.length; i++) {
      char c = s.text[i];
      bool is_whitespace = IsWhitespace(c) || (c == '\0');
      
      if(hit_whitespace && !is_whitespace)
         return false;

      if(is_whitespace) {
         hit_whitespace = true;
      } else if(!IsNumeric(c)) {
         return false;
      }
   }
   return true;
}

struct MemoryArenaBlock {
   MemoryArenaBlock *next;
   
   u64 size;
   u64 used;
   u8 *memory;
};

typedef MemoryArenaBlock *(*alloc_arena_block_callback)(u64 size);
struct MemoryArena {
   u64 initial_size;
   alloc_arena_block_callback alloc_block;

   bool valid;
   MemoryArena *parent;
   MemoryArenaBlock *latest_block;
   u64 latest_used;
   
   MemoryArenaBlock *first_block;
   MemoryArenaBlock *curr_block;
};

u8 *PushSize(MemoryArena *arena, u64 size) {
   Assert(arena->valid);

   MemoryArenaBlock *curr_block = arena->curr_block;
   if(curr_block->size <= (size + curr_block->used)) {
      if(curr_block->next == NULL) {
         //TODO: what size should we allocate?
         MemoryArenaBlock *new_block = arena->alloc_block(Max(arena->initial_size, size));
         
         curr_block->next = new_block;
         arena->curr_block = new_block;
      } else {
         arena->curr_block = curr_block->next; 
      }
      curr_block = arena->curr_block;
   }

   Assert(curr_block->size >= (curr_block->used + size));
   u8 *result = curr_block->memory + curr_block->used;
   curr_block->used += size;
   _Zero(result, size);

   return result;
}

MemoryArena BeginTemp(MemoryArena *arena) {
   MemoryArena result = {};
   result.initial_size = arena->initial_size;
   result.first_block = arena->first_block;
   result.curr_block = arena->curr_block;

   result.valid = true;
   result.parent = arena;
   result.latest_block = arena->curr_block;
   result.latest_used = arena->curr_block->used;

   arena->valid = false;
   return result;
}

void EndTemp(MemoryArena *temp) {
   temp->latest_block->used = temp->latest_used;
   for(MemoryArenaBlock *block = temp->latest_block->next;
       block; block = block->next)
   {
      block->used = 0;
   } 

   temp->valid = false;
   temp->parent->curr_block = temp->latest_block;
   temp->parent->valid = true;
}

#define PushStruct(arena, struct) (struct *) PushSize(arena, sizeof(struct))
#define PushArray(arena, struct, length) (struct *) PushSize(arena, (length) * sizeof(struct))
#define PushArrayCopy(arena, struct, first_elem, length) (struct *) PushCopy(arena, first_elem, length * sizeof(struct))

string PushCopy(MemoryArena *arena, string s) {
   string result = {};
   result.length = s.length;
   result.text = (char *) PushSize(arena, s.length);
   Copy((u8 *) s.text, s.length, (u8 *) result.text);
   return result;
}

u8 *PushCopy(MemoryArena *arena, void *src, u32 size) {
   u8 *result = PushSize(arena, size);
   Copy(src, size, result);
   return result;
}

void Reset(MemoryArena *arena) {
   for(MemoryArenaBlock *curr_block = arena->first_block;
       curr_block; curr_block = curr_block->next)
   {
      curr_block->used = 0;
   }
   
   arena->curr_block = arena->first_block;
}

struct buffer {
   u64 size;
   u64 offset;
   u8 *data;
};

buffer Buffer(u64 size, u8 *data) {
   buffer result = {};
   result.size = size;
   result.data = data;
   return result;
}

buffer PushBuffer(MemoryArena *arena, u64 size) {
   return Buffer(size, PushSize(arena, size));
}

#define ConsumeStruct(b, struct) (struct *) ConsumeSize(b, sizeof(struct))
#define ConsumeArray(b, struct, length) (struct *) ConsumeSize(b, (length) * sizeof(struct))
u8 *ConsumeSize(buffer *b, u64 size) {
   Assert((b->offset + size) <= b->size);
   u8 *result = b->data + b->offset;
   b->offset += size;
   return result;
}

string ConsumeString(buffer *b, u32 length) {
   string result = {};
   result.length = length;
   result.text = ConsumeArray(b, char, length);
   return result;
}

#define ConsumeAndCopyArray(arena, b, struct, length) PushArrayCopy(arena, struct, ConsumeArray(b, struct, length), length)

#define PeekStruct(b, struct) (struct *) PeekSize(b, sizeof(struct))
u8 *PeekSize(buffer *b, u64 size) {
   Assert((b->offset + size) <= b->size);
   return b->data + b->offset;
}

#define WriteStruct(b, struct_data) WriteSize(b, (u8 *)(struct_data), sizeof(*(struct_data)))
#define WriteArray(b, first_elem, length) WriteSize(b, (u8 *)(first_elem), (length) * sizeof(*(first_elem)))
void WriteSize(buffer *b, void *in_data, u64 size) {
   u8 *data = (u8 *) in_data;
   Assert((b->size - b->offset) >= size);
   Copy(data, size, b->data + b->offset);
   b->offset += size;
}

void WriteString(buffer *b, string str) {
   WriteArray(b, str.text, str.length);
}

#define WriteStructData(b, type, name, code) do { type name = {}; code WriteStruct(b, &name); } while(false)

MemoryArena __temp_arena;

struct TempArena {
   MemoryArena arena;

   TempArena(MemoryArena *_arena = &__temp_arena) {
      arena = BeginTemp(_arena);
   } 

   ~TempArena() {
      EndTemp(&arena);
   }
};

#define PushTempSize(size) PushSize(&__temp_arena, (size))
#define PushTempStruct(struct) (struct *) PushSize(&__temp_arena, sizeof(struct))
#define PushTempArray(struct, length) (struct *) PushSize(&__temp_arena, (length) * sizeof(struct))
#define PushTempCopy(string) PushCopy(&__temp_arena, (string))
#define PushTempBuffer(size) PushBuffer(&__temp_arena, (size))

MemoryArenaBlock *PushTempBlock(u64 size) {
   MemoryArenaBlock *result = (MemoryArenaBlock *) PushSize(&__temp_arena, sizeof(MemoryArenaBlock) + size);
   result->size = size;
   result->used = 0;
   result->next = NULL;
   result->memory = (u8 *) (result + 1);
   return result;
}

MemoryArena PushTempArena(u32 size) {
   MemoryArena result = {};
   result.first_block = PushTempBlock(size);
   result.curr_block = result.first_block;
   result.initial_size = size;
   result.valid = true;
   result.alloc_block = PushTempBlock;
   return result;
}

//--------------REWRITE THIS ASAP-----------------
//------------------------------------------------
string Concat(string a, string b, string c = EMPTY_STRING, string d = EMPTY_STRING,
              string e = EMPTY_STRING, string f = EMPTY_STRING) {
   string inputs[] = {a, b, c, d, e, f};
   string result = {};
   
   for(u32 i = 0; i < ArraySize(inputs); i++) {
      result.length += inputs[i].length;
   }

   result.text = PushTempArray(char, result.length);
   u8 *dest = (u8 *) result.text;
   for(u32 i = 0; i < ArraySize(inputs); i++) {
      string s = inputs[i];
      Copy((u8 *) s.text, s.length, dest);
      dest += s.length;
   }

   return result;
}

#include "stdio.h"
string ToString(f32 value) {
   //TODO: clean this up
   char buffer[128] = {};
   sprintf(buffer, "%f", value);
   string result = PushTempCopy(Literal(buffer));
   u32 length = 0;
   for(u32 i = result.length - 1; i > 0; i--) {
      if(result.text[i] == '.') {
         length = i;
         break;
      }
      
      if(result.text[i] != '0') {
         length = i + 1;
         break;
      }
   }
   result.length = Max(1, length);
   return result;
}

f32 ToF32(string number) {
   char number_buffer[256] = {};
   sprintf(number_buffer, "%.*s", number.length, number.text);
   return atof(number_buffer);
}

string ToString(u32 value) {
   char buffer[128] = {};
   sprintf(buffer, "%u", value);
   return PushTempCopy(Literal(buffer));
}

u32 ToU32(string number) {
   char number_buffer[256] = {};
   sprintf(number_buffer, "%.*s", number.length, number.text);
   return (u32) strtol(number_buffer, NULL, 10);
}

char *ToCString(string str) {
   char null_terminated[256] = {};
   sprintf(null_terminated, "%.*s", Min(str.length, 255), str.text);
   return (char *) PushCopy(&__temp_arena, null_terminated, ArraySize(null_terminated));
}

#include "stdlib.h"
f32 Random01() {
   return (f32)rand() / (f32)RAND_MAX;
}
//------------------------------------------------

#define PI32 3.141592653589793238462

f32 lerp(f32 a, f32 t, f32 b) {
   return a + t * (b - a);
}

f32 abs(f32 x) {
   return (x > 0) ? x : -x;
}

f64 abs(f64 x) {
   return (x > 0) ? x : -x;
}

f32 ToDegrees(f32 r) {
   return r * (180 / PI32);
}

f32 ToRadians(f32 d) {
   return d * (PI32 / 180);
}

f32 AngleBetween(f32 angle1, f32 angle2, bool clockwise) {
   if(angle2 > angle1) {
      return clockwise ? (angle2 - angle1 - 360) : (angle2 - angle1);
   } else {
      return clockwise ? (angle2 - angle1) : (angle2 - angle1 + 360);
   }
}

bool IsClockwiseShorter(f32 angle1, f32 angle2) {
   if(angle2 > angle1) {
      return abs(angle2 - angle1 - 360) < abs(angle2 - angle1);
   } else {
      return abs(angle2 - angle1) < abs(angle2 - angle1 + 360);
   }
}

f32 CanonicalizeAngle_Degrees(f32 rawAngle) {
   s32 revolutions = (s32) (rawAngle / 360);
   f32 modPI = (rawAngle - revolutions * 360);
   return modPI < 0 ? 360 + modPI : modPI;
}

f32 CanonicalizeAngle_Radians(f32 rawAngle) {
   s32 revolutions = (s32) (rawAngle / (2 * PI32));
   f32 modPI = (rawAngle - revolutions * (2 * PI32));
   return modPI < 0 ? (2 * PI32) + modPI : modPI;
}

f32 AngleBetween_Radians(f32 angle1, f32 angle2, bool clockwise) {
   if(angle2 > angle1) {
      return clockwise ? (angle2 - angle1 - 2*PI32) : (angle2 - angle1);
   } else {
      return clockwise ? (angle2 - angle1) : (angle2 - angle1 + 2*PI32);
   }
}

bool IsClockwiseShorter_Radians(f32 angle1, f32 angle2) {
   if(angle2 > angle1) {
      return abs(angle2 - angle1 - 2*PI32) < abs(angle2 - angle1);
   } else {
      return abs(angle2 - angle1) < abs(angle2 - angle1 + 2*PI32);
   }
}

f32 ShortestAngleBetween_Radians(f32 a, f32 b) {
   return AngleBetween_Radians(a, b, IsClockwiseShorter_Radians(a, b));
}

union v4 {
   struct { f32 r, g, b, a; };
   struct { f32 x, y, z, w; };
   f32 e[4];
};

inline v4 V4(f32 x, f32 y, f32 z, f32 w) {
   v4 result = {x, y, z, w}; 
   return result;
}

v4 operator* (f32 s, v4 v) {
	v4 output = {};
	output.x = v.x * s;
	output.y = v.y * s;
	output.z = v.z * s;
	output.w = v.w * s;
   return output;
}

v4 lerp(v4 a, f32 t, v4 b) {
   return V4(lerp(a.r, t, b.r),
             lerp(a.g, t, b.g),
             lerp(a.b, t, b.b), 
             lerp(a.a, t, b.a));
}

union v2 {
   struct { f32 u, v; };
   struct { f32 x, y; };
   f32 e[2];
};

inline v2 V2(f32 x, f32 y) {
   v2 result = {x, y};
   return result;
}

inline v2 XOf(v2 v) {
   v2 result = {v.x, 0};
   return result;
}

inline v2 YOf(v2 v) {
   v2 result = {0, v.y};
   return result;
}

v2 operator+ (v2 a, v2 b) {
	v2 output = {};
	output.x = a.x + b.x;
	output.y = a.y + b.y;
	return output;
}

v2 operator- (v2 v) {
	return V2(-v.x, -v.y);
}

v2 operator- (v2 a, v2 b) {
	v2 output = {};
	output.x = a.x - b.x;
	output.y = a.y - b.y;
	return output;
}

v2 operator* (v2 v, f32 s) {
	v2 output = {};
	output.x = v.x * s;
	output.y = v.y * s;
	return output;
}

v2 operator* (f32 s, v2 v) {
	v2 output = {};
	output.x = v.x * s;
	output.y = v.y * s;
	return output;
}

v2 operator/ (v2 v, f32 s) {
	v2 output = {};
	output.x = v.x / s;
	output.y = v.y / s;
	return output;
}

v2 operator/ (v2 a, v2 b) {
	v2 output = {};
	output.x = a.x / b.x;
	output.y = a.y / b.y;
	return output;
}

v2 Perp(v2 v) {
   return V2(v.y, -v.x);
}

v2 lerp(v2 a, f32 t, v2 b) {
   return V2(lerp(a.x, t, b.x), lerp(a.y, t, b.y));
}

#include "math.h"

u32 Power(u32 base, u32 exp) {
   u32 result = 1;
   while(exp) {
      if (exp & 1)
         result *= base;

      exp /= 2;
      base *= base;
   }

   return result;
}

f32 Sign(f32 x) {
   return x > 0 ? 1 : -1;
}

f32 Length(v2 a) { return sqrtf(a.x * a.x + a.y * a.y); }
f32 Distance(v2 a, v2 b) { return Length(a - b); }

v2 Normalize(v2 v) {
   f32 len = Length(v);
   return (len == 0) ? V2(0, 0) : (v / len);
}

v2 AspectRatio(v2 initial_size, v2 size_to_fit) {
   f32 rs = size_to_fit.x / size_to_fit.y;
   f32 ri = initial_size.x / initial_size.y; 
   return rs > ri ? V2(initial_size.x * size_to_fit.y / initial_size.y, size_to_fit.y) : 
                    V2(size_to_fit.x, initial_size.y * size_to_fit.x / initial_size.x);
}

f32 Dot(v2 a, v2 b) {
   return a.x * b.x + a.y * b.y;
}

v2 Midpoint(v2 a, v2 b) {
   return (a + b) / 2;
}

f32 DistFromLine(v2 a, v2 b, v2 p) {
   f32 num = (a.y - b.y) * p.x - (a.x - b.x) * p.y + a.x*b.y - a.y*b.x;
   return abs(num) / sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

v2 CubicHermiteSpline(v2 a_pos, v2 a_tan, v2 b_pos, v2 b_tan, f32 t) {
   return (2*t*t*t - 3*t*t + 1)*a_pos + (t*t*t - 2*t*t + t)*a_tan + 
          (-2*t*t*t + 3*t*t)*b_pos + (t*t*t - t*t)*b_tan;
}

v2 CubicHermiteSplineTangent(v2 a_pos, v2 a_tan, v2 b_pos, v2 b_tan, f32 t) {
   return 6*t*(t - 1)*a_pos + 
          (3*t*t - 4*t + 1)*a_tan + 
          (6*t - 6*t*t)*b_pos + 
          (3*t*t - 2*t)*b_tan;
}

union v3 {
   struct { f32 r, g, b; };
   struct { f32 x, y, z; };
   f32 e[3];
};

v3 V3(f32 x, f32 y, f32 z) {
   v3 result = {x, y, z};
   return result;
}

v3 operator/ (v3 v, f32 s) {
	v3 output = {};
	output.x = v.x / s;
	output.y = v.y / s;
   output.z = v.z / s;
	return output;
}

f32 Length(v3 a) { 
   return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); 
}

v3 Normalize(v3 v) {
   f32 len = Length(v);
   return (len == 0) ? V3(0, 0, 0) : (v / len);
}

struct rect2 {
   v2 min;
   v2 max;
};

rect2 operator+ (v2 offset, rect2 r) {
	rect2 result = r;
	result.min = result.min + offset;
	result.max = result.max + offset;
	return result;
}

inline rect2 RectMinSize(v2 min, v2 size) {
   rect2 result = {};
   
   result.min = min;
   result.max = min + size;
   
   return result;
}

inline rect2 RectMinMax(v2 min, v2 max) {
   rect2 result = {};
   
   result.min = min;
   result.max = max;
   
   return result;
}

inline rect2 RectCenterSize(v2 pos, v2 size) {
   rect2 result = {};
   
   result.min = pos - 0.5 * size;
   result.max = pos + 0.5 * size;
   
   return result;
}

inline v2 Size(rect2 r) {
   return r.max - r.min;
}

inline v2 Center(rect2 r) {
   return (r.min + r.max) / 2;
}

inline v2 ClampTo(v2 p, rect2 r) {
   return V2(Clamp(r.min.x, r.max.x, p.x), 
             Clamp(r.min.y, r.max.y, p.y));
}

inline bool Contains(rect2 rect, v2 vec) {
   bool result = (vec.x >= rect.min.x) && 
                 (vec.x <= rect.max.x) &&
                 (vec.y >= rect.min.y) &&
                 (vec.y <= rect.max.y);
                
   return result;
}

inline bool IsInside(rect2 a, rect2 b) {
   return Contains(b, a.min) && Contains(b, a.max);
}

inline rect2 Overlap(rect2 a, rect2 b) {
   rect2 result = {};
   
   result.min.x = Max(a.min.x, b.min.x);
   result.min.y = Max(a.min.y, b.min.y);
   result.max.x = Min(a.max.x, b.max.x);
   result.max.y = Min(a.max.y, b.max.y);
   
   if((result.min.x > result.max.x) || (result.min.y > result.max.y))
      return RectCenterSize(V2(0,0), V2(0, 0));
   
   return result;
}

inline rect2 Union(rect2 a, rect2 b) {
   rect2 result = {};
   
   result.min.x = Min(a.min.x, b.min.x);
   result.min.y = Min(a.min.y, b.min.y);
   result.max.x = Max(a.max.x, b.max.x);
   result.max.y = Max(a.max.y, b.max.y);
   
   return result;
}

union mat4 {
   f32 e[16];
};

mat4 Identity()
{
   mat4 result = {};
   result.e[0] = 1;
   result.e[5] = 1;
   result.e[10] = 1;
   result.e[15] = 1;
   return result;
}

mat4 Orthographic(f32 top, f32 bottom, f32 left, f32 right, f32 nearPlane, f32 far)
{
   mat4 result = {};
   
   result.e[0] = 2 / (right - left);
   result.e[5] = 2 / (top - bottom);
   result.e[10] = -2 / (far - nearPlane);
   result.e[12] = -(right + left) / (right - left);
   result.e[13] = -(top + bottom) / (top - bottom);
   result.e[14] = -(far + nearPlane) / (far - nearPlane);
   result.e[15] = 1;
   
   return result;
}

//----------------------------------------------
struct InterpolatingMap_Leaf {
   f32 len;
   
   union {
      v2 data_v2;
      f32 data_f32;
      void *data_ptr;
   };
};

struct InterpolatingMap_Branch {
   f32 len;

   f32 greater_value;
   f32 less_value;

   union {
      struct {
         u32 greater_leaf;
         u32 less_leaf;
      };

      struct {
         InterpolatingMap_Branch *greater_branch;
         InterpolatingMap_Branch *less_branch;
      };
   };
};

typedef void (*interpolation_map_callback)(InterpolatingMap_Leaf *a, InterpolatingMap_Leaf *b, f32 t, void *result);

struct InterpolatingMap {
   MemoryArena arena;
   u32 sample_exp;
   interpolation_map_callback lerp_callback;

   InterpolatingMap_Branch *root;
   InterpolatingMap_Branch **layers; //NOTE: has sample_exp arrays/layers in it
   InterpolatingMap_Leaf *samples;
};

struct InterpolatingMapSamples {
   u32 count;
   InterpolatingMap_Leaf *data;
   MemoryArena *arena;
};

InterpolatingMapSamples ResetMap(InterpolatingMap *map) {
   MemoryArena *arena = &map->arena;
   Reset(arena);

   u32 sample_count = Power(2, map->sample_exp); 
   InterpolatingMap_Leaf *samples = PushArray(arena, InterpolatingMap_Leaf, sample_count);
   map->samples = samples;

   InterpolatingMapSamples result = {sample_count, samples, arena};
   return result;
}

void BuildMap(InterpolatingMap *map) {
   MemoryArena *arena = &map->arena;

   u32 last_layer_count = Power(2, map->sample_exp - 1);
   InterpolatingMap_Branch *last_layer = PushArray(arena, InterpolatingMap_Branch, last_layer_count);
   for(u32 i = 0; i < last_layer_count; i++) {
      InterpolatingMap_Leaf *less = map->samples + (2 * i);
      InterpolatingMap_Leaf *greater = map->samples + (2 * i + 1);

      last_layer[i].less_leaf = 2 * i;
      last_layer[i].less_value = less->len;
      last_layer[i].greater_leaf = 2 * i + 1;
      last_layer[i].greater_value = greater->len;
      last_layer[i].len = (less->len + greater->len) / 2;
   }
   
   u32 debug_layer_i = 0;
   map->layers = PushArray(arena, InterpolatingMap_Branch *, map->sample_exp);
   map->layers[debug_layer_i++] = last_layer;

   for(u32 i = 1; i < map->sample_exp; i++) {
      u32 layer_count = Power(2, map->sample_exp - i - 1);
      InterpolatingMap_Branch *curr_layer = PushArray(arena, InterpolatingMap_Branch, layer_count);

      for(u32 i = 0; i < layer_count; i++) {
         InterpolatingMap_Branch *less = last_layer + (2 * i);
         InterpolatingMap_Branch *greater = last_layer + (2 * i + 1);

         curr_layer[i].less_branch = less;
         curr_layer[i].less_value = less->less_value;
         curr_layer[i].greater_branch = greater;
         curr_layer[i].greater_value = greater->greater_value;
         curr_layer[i].len = (less->greater_value + greater->less_value) / 2;
      }

      last_layer_count = layer_count;
      last_layer = curr_layer;
      map->layers[debug_layer_i++] = last_layer;
   }

   map->root = last_layer;
}

void MapLookup(InterpolatingMap *map, f32 distance, void *result) {
   u32 sample_count = Power(2, map->sample_exp);
   InterpolatingMap_Branch *branch = map->root;
   for(u32 i = 0; i < (map->sample_exp - 1); i++) {
      branch = (distance > branch->len) ? branch->greater_branch : branch->less_branch;
   }

   u32 a_i;
   u32 b_i;

   u32 center_leaf_i = (distance > branch->len) ? branch->greater_leaf : branch->less_leaf;
   if(distance > map->samples[center_leaf_i].len) {
      a_i = Clamp(0, sample_count - 1, center_leaf_i);
      b_i = Clamp(0, sample_count - 1, center_leaf_i + 1);
   } else {
      a_i = Clamp(0, sample_count - 1, center_leaf_i - 1);
      b_i = Clamp(0, sample_count - 1, center_leaf_i);
   }

   InterpolatingMap_Leaf *leaf_a = map->samples + a_i;
   InterpolatingMap_Leaf *leaf_b = map->samples + b_i;

   f32 t = (distance - leaf_a->len) / (leaf_b->len - leaf_a->len); 
   map->lerp_callback(leaf_a, leaf_b, t, result);
}

void interpolation_map_v2_lerp(InterpolatingMap_Leaf *a, InterpolatingMap_Leaf *b, f32 t, void *result) {
   *((v2 *)result) = lerp(a->data_v2, t, b->data_v2);
}
 
//----------------------------------------------

string exe_directory = {};

u64 total_size_requested = 0;
u64 total_size_allocated = 0;
u32 arenas_allocated = 0;
u32 arena_blocks_allocated = 0;

//------------------PLATFORM-SPECIFIC-STUFF---------------------
#ifdef COMMON_PLATFORM
   #if defined(_WIN32)
      //TODO: make stuff thread safe before we re-introduce this stuff

      //NOTE: on windows you need to include "windows.h" before common
      // u32 AtomicIncrement(volatile u32 *x) {
      //    Assert(( (u64)x & 0x3 ) == 0);
      //    return InterlockedIncrement(x);
      // }
      // #define READ_BARRIER MemoryBarrier()
      // #define WRITE_BARRIER MemoryBarrier()

      //TODO: take a look at the weird memory usage
      MemoryArenaBlock *PlatformAllocArenaBlock(u64 size) {
         OutputDebugStringA("Allocating Arena Block\n");
         total_size_requested += size;
         total_size_allocated += (sizeof(MemoryArenaBlock) + size);
         arena_blocks_allocated++;

         MemoryArenaBlock *result = (MemoryArenaBlock *) VirtualAlloc(0, sizeof(MemoryArenaBlock) + size, MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);
         result->size = size;
         result->used = 0;
         result->next = NULL;
         result->memory = (u8 *) (result + 1);
         return result;
      }

      MemoryArena PlatformAllocArena(u64 initial_size) {
         OutputDebugStringA("Allocating Arena\n");
         arenas_allocated++;
         
         MemoryArena result = {};
         result.first_block = PlatformAllocArenaBlock(initial_size);
         result.curr_block = result.first_block;
         result.initial_size = initial_size;
         result.valid = true;
         result.alloc_block = PlatformAllocArenaBlock;
         return result;
      }

      buffer ReadEntireFile(const char* path, bool in_exe_directory = false) {
         char full_path[MAX_PATH + 1];
         sprintf(full_path, "%.*s%s", exe_directory.length, exe_directory.text, path);

         HANDLE file_handle = CreateFileA(in_exe_directory ? full_path : path, GENERIC_READ, FILE_SHARE_READ,
                                          NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
                                          
         buffer result = {};
         if(file_handle != INVALID_HANDLE_VALUE) {
            DWORD number_of_bytes_read;
            result.size = GetFileSize(file_handle, NULL);
            result.data = (u8 *) VirtualAlloc(0, result.size, MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);
            ReadFile(file_handle, result.data, result.size, &number_of_bytes_read, NULL);
            CloseHandle(file_handle);
         } else {
            OutputDebugStringA("File read error\n");
            DWORD error_code = GetLastError();
         }

         return result;
      }

      buffer ReadEntireFile(string path, bool in_exe_directory = false) {
         return ReadEntireFile(ToCString(path), in_exe_directory);
      }

      void FreeEntireFile(buffer *file) {
         VirtualFree(file->data, 0, MEM_RELEASE);
         file->data = 0;
         file->size = 0;
      }

      void WriteEntireFile(const char* path, buffer file) {
         HANDLE file_handle = CreateFileA(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
                                          FILE_ATTRIBUTE_NORMAL, NULL);
                                          
         if(file_handle != INVALID_HANDLE_VALUE) {
            DWORD number_of_bytes_written;
            WriteFile(file_handle, file.data, file.offset, &number_of_bytes_written, NULL);
            CloseHandle(file_handle);
         }
      }

      void WriteEntireFile(string path, buffer file) {
         return WriteEntireFile(ToCString(path), file);
      }

      struct FileListLink {
         FileListLink *next;
         string name;
         string full_name;
      };

      FileListLink *ListFilesWithExtension(char *wildcard_extension, MemoryArena *arena = &__temp_arena) {
         WIN32_FIND_DATAA file = {};
         HANDLE handle = FindFirstFileA(wildcard_extension, &file);
         FileListLink *result = NULL;
         if(handle != INVALID_HANDLE_VALUE) {
            do {
               FileListLink *new_link = PushStruct(arena, FileListLink);
               string full_name = Literal(file.cFileName);
               
               if((full_name == Literal(".")) ||
                  (full_name == Literal("..")))
               {
                  continue;
               }
               
               u32 name_length = 0;
               for(u32 i = 0; i < full_name.length; i++) {
                  if(full_name.text[i] == '.') {
                     name_length = i;
                     break;
                  }
               }
               
               new_link->full_name = PushCopy(arena, full_name);
               new_link->name = String(new_link->full_name.text, name_length);
               new_link->next = result;
               result = new_link;
            } while(FindNextFileA(handle, &file));
            FindClose(handle);
         }
         return result;
      }
   
      u64 GetFileTimestamp(const char* path, bool in_exe_directory = false) {
         char full_path[MAX_PATH + 1];
         sprintf(full_path, "%.*s%s", exe_directory.length, exe_directory.text, path);
         
         FILETIME last_write_time = {};
         HANDLE file_handle = CreateFileA(in_exe_directory ? full_path : path, GENERIC_READ, FILE_SHARE_READ,
                                          NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
         
         if(file_handle != INVALID_HANDLE_VALUE) {
            GetFileTime(file_handle, NULL, NULL, &last_write_time);
            CloseHandle(file_handle);
         }
         
         return (u64)last_write_time.dwLowDateTime | 
               ((u64)last_write_time.dwHighDateTime << 32);
      }

      u64 GetFileTimestamp(string path, bool in_exe_directory = false) {
         return GetFileTimestamp(ToCString(path), in_exe_directory);
      }

      struct FileWatcherLink {
         FileWatcherLink *next_in_list;
         FileWatcherLink *next_in_hash;

         bool found;

         u64 timestamp;
         string name;
      };
      
      struct FileWatcher {
         MemoryArena arena; //NOTE: this needs its own memory because it has to persist
         string wildcard_extension;

         FileWatcherLink *first_in_list;
         FileWatcherLink *hash[64];
      };

      void InitFileWatcher(FileWatcher *watcher, MemoryArena arena, string wildcard_extension) {
         watcher->arena = arena;
         watcher->wildcard_extension = PushCopy(&watcher->arena, wildcard_extension);
      }
      
      void InitFileWatcher(FileWatcher *watcher, MemoryArena arena, char *wildcard_extension) {
         InitFileWatcher(watcher, arena, Literal(wildcard_extension));
      }

      //NOTE: updates file watcher, returns true if file timestamps have changed
      bool CheckFiles(FileWatcher *watcher) {
         MemoryArena *arena = &watcher->arena;

         for(FileWatcherLink *curr = watcher->first_in_list;
             curr; curr = curr->next_in_list)
         {
            curr->found = false;
         }

         bool changed = false;
         for(FileListLink *file = ListFilesWithExtension(ToCString(watcher->wildcard_extension)); 
             file; file = file->next)
         {
            FileWatcherLink *link = NULL;
            u32 hash = Hash(file->full_name) % ArraySize(watcher->hash);
            for(FileWatcherLink *curr = watcher->hash[hash];
                curr; curr = curr->next_in_hash)
            {
               if(curr->name == file->full_name) {
                  link = curr;
               }
            }

            if(link == NULL) {
               changed = true;
               FileWatcherLink *new_link = PushStruct(arena, FileWatcherLink);
               new_link->name = PushCopy(arena, file->full_name);
               
               new_link->next_in_list = watcher->first_in_list;
               watcher->first_in_list = new_link;
               new_link->next_in_hash = watcher->hash[hash];
               watcher->hash[hash] = new_link;

               link = new_link;
            }

            link->found = true;
            u64 timestamp = GetFileTimestamp(file->name);
            if(link->timestamp != timestamp) {
               link->timestamp = timestamp;
               changed = true;
            }
         }
         
         for(FileWatcherLink *curr = watcher->first_in_list;
             curr; curr = curr->next_in_list)
         {
            if(curr->found == false) {
               changed = true;
            }
         }
         
         return changed;
      }
      
      //NOTE: this is pretty jank-tastic but itll get cleaned up in future
      char exepath[MAX_PATH + 1];
      void Win32CommonInit(MemoryArena temp_arena) {
         __temp_arena = temp_arena;

         if(0 == GetModuleFileNameA(0, exepath, MAX_PATH + 1))
            Assert(false);
            
         exe_directory = Literal(exepath);
         for(u32 i = exe_directory.length - 1; i >= 0; i--) {
            if((exe_directory.text[i] == '\\') || (exe_directory.text[i] == '/'))
               break;

            exe_directory.length--;
         }
      }

      struct Timer {
         LARGE_INTEGER frequency;
         LARGE_INTEGER timer;
      };

      Timer InitTimer() {
         Timer result = {};
         QueryPerformanceFrequency(&result.frequency); 
         QueryPerformanceCounter(&result.timer);
         return result;
      }

      f32 GetDT(Timer *timer) {
         LARGE_INTEGER new_time;
         QueryPerformanceCounter(&new_time);
         f32 dt = (f32)(new_time.QuadPart - timer->timer.QuadPart) / (f32)timer->frequency.QuadPart;
         timer->timer = new_time;
         
         return dt;
      }

   #else
      #error "we dont support that platform yet"
   #endif
#endif
//------------------------------------------------------------------

// struct ticket_mutex {
//    volatile u32 ticket;
//    volatile u32 serving;
// };

// void BeginMutex(ticket_mutex *mutex) {
//    u32 ticket = AtomicIncrement(&mutex->ticket) - 1;
//    while(mutex->serving != ticket);
// }

// void EndMutex(ticket_mutex *mutex) {
//    AtomicIncrement(&mutex->serving);
// }

//TODO: thread_pool
