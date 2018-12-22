struct vertical_list_persistent_data {
   f32 scroll_offset;

   //TODO: not a big fan of this one frame delay
   f32 last_length;
   f32 length;
};

struct vertical_list_layout_data {
   vertical_list_persistent_data *data;
   v2 at;
};

rect2 verticalListLayout(element *e, u8 *in_layout_data, v2 element_size, v2 padding_size, v2 margin_size) {
   vertical_list_layout_data *layout_data = (vertical_list_layout_data *) in_layout_data;
   
   v2 pos = layout_data->at;
   layout_data->at.y += element_size.y + padding_size.y + margin_size.y;
   layout_data->data->length += element_size.y + padding_size.y + margin_size.y;
   return RectMinSize(e->bounds.min + pos + padding_size, element_size);
}

//TODO: make it so if you were at the bottom of the list last frame and it gets longer you move to
//      the new bottom
#define VerticalList(...) _VerticalList(GEN_UI_ID, __VA_ARGS__)
element *_VerticalList(ui_id id, element *parent, rect2 bounds) {
   UIContext *context = parent->context;
   element *base = _Panel(id + GEN_UI_ID, parent, bounds, Layout(StackLayout));
   
   vertical_list_persistent_data *data = GetOrAllocate(base, vertical_list_persistent_data);
   data->last_length = data->length;
   data->length = 0;

   element *content = _Panel(id + GEN_UI_ID, base, bounds, Captures(INTERACTION_VERTICAL_SCROLL));
   element *scroll  = _Panel(id + GEN_UI_ID, base, bounds);

   vertical_list_layout_data *layout_data = PushStruct(&context->frame_arena, vertical_list_layout_data); 
   layout_data->data = data;
   layout_data->at.y = -data->scroll_offset;
   
   content->layout_data = (u8 *) layout_data;
   content->layout_elem = verticalListLayout;
   
   rect2 scroll_column_rect = RectMinMax(V2(scroll->bounds.max.x - 20, scroll->bounds.min.y), 
                                         scroll->bounds.max);
   element *scroll_column = _Panel(id + GEN_UI_ID, scroll, scroll_column_rect);
   Outline(scroll_column, BLACK);

   f32 max_offset = data->last_length - Size(bounds).y;
   f32 scroll_handle_height = Size(bounds).y * ((data->last_length < Size(bounds).y) ? 1 : (Size(bounds).y / data->last_length));
   f32 scroll_handle_offset = (max_offset == 0) ? 0 : 
            ((Size(bounds).y - scroll_handle_height) * (data->scroll_offset / max_offset));
   rect2 scroll_handle_rect = RectMinSize(scroll_column_rect.min + V2(5, 5 + scroll_handle_offset),
                                          V2(10, scroll_handle_height - 10));
   
   element *scroll_handle = _Panel(id + GEN_UI_ID, scroll_column, scroll_handle_rect, Captures(INTERACTION_DRAG));
   Background(scroll_handle, IsActive(scroll_handle) ? RED : BLACK);

   data->scroll_offset += GetDrag(scroll_handle).y - GetVerticalScroll(content); //TODO: the scroll speed isnt 1:1, the bigger last_length the faster we should scroll
   data->scroll_offset = Clamp(0, Max(0, max_offset), data->scroll_offset);

   return content;
}

element *_VerticalList(ui_id id, element *parent) {
   return _VerticalList(id, parent, parent->bounds);
}

//TODO: implement this
/*
element *_VerticalList(ui_id id, element *parent, v2 size, 
                       v2 padding = V2(0, 0), v2 margin = V2(0, 0)) {
   element *e = _Panel(id, parent, ColumnLayout, size, padding, margin);
   return e;
}
*/