#define HorizontalSlider(...) _HorizontalSlider(GEN_UI_ID, __VA_ARGS__)
void _HorizontalSlider(ui_id id, element *parent, f32 *value, f32 min, f32 max,
                       v2 size, v2 p = V2(0, 0), v2 m = V2(0, 0))
{
   element *slider = _Panel(id + GEN_UI_ID, parent, size, Padding(p).Margin(m));
   Background(slider, V4(0.7, 0.7, 0.7, 1));
   
   f32 handle_width = 10;
   v2 handle_offset = (size.x - handle_width) * V2((*value - min) / (max - min), 0);
   rect2 handle_bounds = RectMinSize(slider->bounds.min + handle_offset, V2(handle_width, size.y));
   element *handle = _Panel(id + GEN_UI_ID, slider, handle_bounds, Captures(INTERACTION_DRAG));
   
   v4 handle_colour = V4(0.3, 0.3, 0.3, 1);
   Background(handle, IsActive(handle) ? 0.5 * handle_colour : handle_colour);
   if(IsHot(handle)) {
      Outline(handle, BLACK);
   }
   
   *value += (max - min) * (GetDrag(handle).x / size.x); 
   *value = Clamp(min, max, *value);
}