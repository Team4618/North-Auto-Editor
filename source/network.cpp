void PacketHandler_Welcome(buffer *packet_in, EditorState *state) {
   RecieveWelcomePacket(&state->profiles.current, *packet_in);
}

void PacketHandler_CurrentParameters(buffer *packet, EditorState *state) {
   RecieveCurrentParametersPacket(&state->profiles.current, *packet);
}

void PacketHandler_State(buffer *packet, EditorState *state) {
   
}

void HandlePacket(EditorState *state, PacketType::type type, buffer packet) {
   if(type == PacketType::Welcome) {
      PacketHandler_Welcome(&packet, state);
   } else if(type == PacketType::CurrentParameters) {
      PacketHandler_CurrentParameters(&packet, state);
   } else if(type == PacketType::State) {
      PacketHandler_State(&packet, state);
   }
}

void HandleDisconnect(EditorState *state) {
   if(state->profiles.current.state == RobotProfileState::Connected) {
      state->profiles.current.state = RobotProfileState::Loaded;
   }

   if(state->profiles.selected == &state->profiles.current) {
      state->profiles.selected = NULL;
   }
}