#pragma pack(push, 1)

/*
NOTE:
   Everything aside from state is sent over TCP socket 5800 
   State is sent over UPD socket 5801
*/

struct PacketHeader {
   u32 size; //NOTE: this lets packets be really big, our practical limit is much smaller than the 4 gb limit
   u8 type; 
};

namespace PacketType {
   enum type {
      Welcome = 1,
      CurrentParameters = 2,
      State = 3,
      ParameterOp = 4,
      SetState = 5,
      CurrentAutoPath = 6,
      UploadAutonomous = 7,
      //NOTE: if we change a packet just make a new type instead 
      //eg. "Welcome" becomes "Welcome_V1" & we create "Welcome_V2"
   };
};

//------------------------------------------
namespace Connect_Flags {
   enum type {
      WANTS_STATE = (1 << 0),
   };
};

struct Connect_PacketHeader {
   u8 flags;
};

//------------------------------------------

//------------------------------------------
//NOTE: everything in Welcome can only change if the robot was rebooted
//      (eg. subsystems & commands)
//      parameters are not in here because they change without a reboot
//      (parameter names dont change without reboot though, so those could be here if we wanted)

struct Welcome_SubsystemCommand {
   u8 name_length;
   u8 param_count;
   u8 type; //NOTE: North_CommandType
   //char name[name_length]
   //{ u8 length; char [length]; } [param_count]
};

struct Welcome_SubsystemDescription {
   u8 name_length;
   u8 command_count;
   //char name[name_length]
   //Welcome_SubsystemCommand [command_count]
};

//NOTE: unused
/*
namespace Welcome_Flags {
   enum type {

   };
};
*/

struct Welcome_PacketHeader {
   u8 robot_name_length;
   u8 subsystem_count;
   f32 robot_width;
   f32 robot_length;
   u32 flags;
   //char name[robot_name_length]
   //Welcome_SubsystemDescription [subsystem_count]
};
//-----------------------------------------

//-----------------------------------------
struct CurrentParameters_Parameter {
   u8 is_array;
   u8 name_length;
   u8 value_count; //Ignored if is_array is false
   //char name[name_length]
   //f32 [value_count]
};

struct CurrentParameters_SubsystemParameters {
   u8 name_length;
   u8 param_count;
   //char name[name_length]
   //Parameter [param_count]
};

struct CurrentParameters_PacketHeader {
   u8 subsystem_count;
   //SubsystemParameters [subsystem_count]
};
//----------------------------------------

//-----------------------------------------
struct State_Diagnostic {
   u8 name_length;
   f32 value;
   u8 unit; //NOTE: North_Unit
   //char name[name_length]
};

struct State_SubsystemDiagnostics {
   u8 name_length;
   u8 diagnostic_count;
   //char name[name_length]
   //State_Diagnostic [diagnostic_count]
};

struct State_Message {
   u8 type; //NOTE: North_MessageType
   u16 length;
   //char message[length]
};

struct State_Marker {
   v2 pos;
   u16 length;
   //char message[length]
};

struct State_PacketHeader {
   v2 pos;
   f32 angle;
   
   u8 mode; //NOTE: North_GameMode

   u8 subsystem_count;
   u8 message_count;
   u8 marker_count;
   f32 time;
   
   //State_SubsystemDiagnostics [subsystem_count]
   //State_Message [message_count]
   //State_Marker [marker_count]
};
//-----------------------------------------

//--------------------------------------
namespace ParameterOp_Type {
   enum type {
      SetValue = 1, //NOTE: for is_array = false
      AddValue = 2, //NOTE: for is_array = true
      RemoveValue = 3, //NOTE: for is_array = true
   };
};

struct ParameterOp_PacketHeader {
   u8 type;
   u8 subsystem_name_length;
   u8 param_name_length;

   f32 value; //NOTE: only used by SetValue & AddValue
   u32 index; //NOTE: ignored if is_array = false
   
   //char [subsystem_name_length]
   //char [param_name_length]
};
//--------------------------------------

struct SetState_PacketHeader {
   v2 pos;
   f32 angle;
};

//--------------------------------------
struct CurrentAutoPath_Path {
   v2 begin;
   v2 end;
   u8 is_conditional; //NOTE: 0 for false, else true
   u8 control_point_count;
   //v2 [control_point_count]
};

struct CurrentAutoPath_PacketHeader {
   u8 path_count;
   //Path [path_count]
};
//-------------------------------------

struct UploadAutonomous_DataPoint {
   f32 distance;
   f32 value;
};

struct UploadAutonomous_ContinuousEvent {
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 datapoint_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //UploadAutonomous_DataPoint [datapoint_count]
};

struct UploadAutonomous_DiscreteEvent {
   f32 distance;
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 parameter_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //f32 [parameter_count]
};

//TODO: change over to cubic hermite splines
//TODO: events during the initial turn?
struct UploadAutonomous_Path {
   u8 is_reverse;
   f32 accel;
   f32 deccel;
   f32 max_vel;

   //NOTE: begin & end points are parent.pos & end_node.pos
   u8 conditional_length; //NOTE: if conditional_length is 0, there is no conditional
   u8 control_point_count;

   u8 continuous_event_count;
   u8 discrete_event_count;

   //char conditional_name [conditional_length]
   //v2 [control_point_count]
   //UploadAutonomous_ContinuousEvent [continuous_event_count]
   //UploadAutonomous_DiscreteEvent [discrete_event_count]

   //UploadAutonomous_Node end_node
};

struct UploadAutonomous_Command {
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 parameter_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //f32 [parameter_count]
};

struct UploadAutonomous_Node {
   v2 pos;
   u8 command_count;
   u8 path_count;
   //UploadAutonomous_Command [command_count]
   //UploadAutonomous_Path [path_count]
};

struct UploadAutonomous_PacketHeader {
   
   //UploadAutonomous_Node begining_node
};

#pragma pack(pop)