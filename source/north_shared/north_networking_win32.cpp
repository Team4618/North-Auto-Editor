SOCKET tcp_socket;
bool was_connected = false;
bool tcp_connected = false;
f32 last_recv_time = 0;

void SendPacket(buffer packet) {
   s32 sent = send(tcp_socket, (char *)packet.data, packet.offset, 0);
   // OutputDebugStringA(ToCString(Concat(Literal("Sent "), ToString(sent), Literal(" bytes\n"))));
}

void CreateSocket() {
   tcp_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
   u_long non_blocking = true;
   ioctlsocket(tcp_socket, FIONBIO, &non_blocking);
}

bool HandleConnectionStatus(char *connection_target, f32 curr_time) {
   if(recv(tcp_socket, NULL, 0, 0) == SOCKET_ERROR) {
      s32 wsa_error = WSAGetLastError();
      if(wsa_error == WSAENOTCONN) {
         tcp_connected = false;
      } else {
         //actual error or blocking
      }
   }
   
   if((curr_time - last_recv_time) > 2) {
      tcp_connected = false;
   }

   bool disconnected = false;
   if(was_connected && !tcp_connected) {
      disconnected = true;
      OutputDebugStringA(ToCString("Disconnecting, t = " + ToString(curr_time - last_recv_time) + "\n"));

      closesocket(tcp_socket);
      CreateSocket();
   }

   was_connected = tcp_connected;

   if(!tcp_connected) {
      struct sockaddr_in server_addr = {};
      server_addr.sin_family = AF_INET;
      server_addr.sin_addr.s_addr = inet_addr(connection_target); 
      server_addr.sin_port = htons(5800);

      if(connect(tcp_socket, (SOCKADDR *) &server_addr, sizeof(server_addr)) != 0) {
         // OutputDebugStringA(ToCString(Concat( ToString(WSAGetLastError()), Literal("\n") )));
      }
   } else {
      //NOTE: send to maintain connection
      PacketHeader heartbeat = {0, PacketType::Heartbeat};
      send(tcp_socket, (char *) &heartbeat, sizeof(heartbeat), 0);
   }

   return disconnected;
}

bool HasPackets(f32 curr_time, PacketHeader *header, buffer *packet) {
   u32 recv_return = recv(tcp_socket, (char *) header, sizeof(PacketHeader), MSG_PEEK);
   
   if(recv_return == SOCKET_ERROR) {
      s32 wsa_error = WSAGetLastError();

      if((wsa_error != 0) && (wsa_error != WSAEWOULDBLOCK)) {
         //an actual error happened
      }

      return false;
   } else if(recv_return == sizeof(PacketHeader)) {
      *packet = PushTempBuffer(header->size + sizeof(PacketHeader));
      recv_return = recv(tcp_socket, (char *) packet->data, packet->size, MSG_PEEK);
      if(recv_return == packet->size) {
         recv(tcp_socket, (char *) packet->data, packet->size, 0);
         
         ConsumeStruct(packet, PacketHeader);
         
         tcp_connected = true;
         last_recv_time = curr_time;
         return true;
      } else {
         return false;
      }
   }

   return false;
}