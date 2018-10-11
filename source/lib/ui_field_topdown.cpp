struct ui_field_topdown {
   v2 size_in_ft;
   rect2 bounds; 
   element *e;
};

#define FieldTopdown(...) _FieldTopdown(GEN_UI_ID, __VA_ARGS__)
ui_field_topdown _FieldTopdown(ui_id id, element *parent, texture background_image, 
                   v2 size_in_ft, f32 width) {
   width = Clamp(0, 700, width);
   v2 size = V2(width, size_in_ft.y  * (width / size_in_ft.x));
   f32 x_padding = (Size(parent->bounds).x - size.x) / 2;
   element *e = _Panel(id, parent, NULL, size, V2(x_padding, 0));
   Texture(e, background_image, e->bounds);

   ui_field_topdown result = {};
   result.size_in_ft = size_in_ft;
   result.bounds = e->bounds;
   result.e = e;
   return result;
}

f32 PixelsToFeet(ui_field_topdown *field, f32 in_px) {
   f32 px_to_ft = field->size_in_ft.x / Size(field->bounds).x;
   f32 in_ft = in_px * px_to_ft;
   return in_ft;
}

v2 PixelsToFeet(ui_field_topdown *field, v2 in_px) {
   return V2(PixelsToFeet(field, in_px.x), PixelsToFeet(field, in_px.y));
}

f32 FeetToPixels(ui_field_topdown *field, f32 in_ft) {
   f32 ft_to_px = Size(field->bounds).x / field->size_in_ft.x;
   f32 in_px = in_ft * ft_to_px;
   return in_px;
}

v2 FeetToPixels(ui_field_topdown *field, v2 in_ft) {
   return V2(FeetToPixels(field, in_ft.x), FeetToPixels(field, in_ft.y));
}

v2 GetPoint(ui_field_topdown *field, v2 p_in_ft) {
   return Center(field->bounds) + FeetToPixels(field, p_in_ft);
}

v2 GetPointOnBezier(v2 a, v2 b, v2 c, f32 t) {
   return (1-t)*(1-t)*a + 2*(1-t)*t*b + t*t*c;
}

void DrawQuadraticBezier(ui_field_topdown *field, v2 a, v2 b, v2 c) {
   u32 segments = 20;
   f32 step = (f32)1 / (f32)segments;
   for(f32 s = 0; s < segments; s++) {
      v2 p1 = GetPointOnBezier(a, b, c, s * step);
      v2 p2 = GetPointOnBezier(a, b, c, (s + 1) * step);
      Line(field->e, GetPoint(field, p1), GetPoint(field, p2), GREEN);
   }
}

void DrawBezierCurve(ui_field_topdown *field, v2 *in_points, u32 point_count) {
   Assert(point_count >= 3);

   v2 *points = PushTempArray(v2, 2 * point_count);
   Copy(in_points, point_count * sizeof(v2), points);

   u32 i;
   for (i = 0; i < point_count - 3; i += 2) {
         v2 p0 = points[i];
         v2 p1 = points[i + 1];
         v2 p2 = points[i + 2];

         v2 mid = V2((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
         
         //points.add(i + 2, mid);
         point_count++;
         for(u32 j = point_count - 1; j > (i + 2); j--) {
            points[j] = points[j - 1];
         }
         points[i + 2] = mid; 
         
         DrawQuadraticBezier(field, p0, p1, mid);
   }

   DrawQuadraticBezier(field, points[i], points[i + 1], points[i + 2]);
}