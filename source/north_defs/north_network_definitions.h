#pragma pack(push, 1)

//NOTE: Everything is sent over TCP socket 5800 

struct PacketHeader {
   //NOTE: this lets packets be really big, our practical limit is much smaller than the 4 gb limit
   //NOTE: this size excludes the 5 byte header
   u32 size;
   u8 type; 
};

namespace PacketType {
   enum type {
      Heartbeat = 0,             // <->
      SetConnectionFlags = 1,    //  ->
      Welcome = 2,               // <-
      CurrentParameters = 3,     // <-
      State = 4,                 // <-
      ParameterOp = 5,           //  ->
      SetState = 6,              //  ->
      UploadAutonomous = 7,      //  ->
      //NOTE: if we change a packet just make a new type instead 
      //eg. "Welcome" becomes "Welcome_V1" & we create "Welcome_V2"
   };
};

//------------------------------------------
namespace SetConnectionFlags_Flags {
   enum type {
      WANTS_STATE = (1 << 0),
   };
};

struct SetConnectionFlags_PacketHeader {
   u8 flags;
};
//------------------------------------------

//------------------------------------------
//NOTE: everything in Welcome can only change if the robot was rebooted
//      (eg. commands)

struct Welcome_Command {
   u8 name_length;
   u8 param_count;
   u8 type; //NOTE: North_CommandExecutionType
   //char name[name_length]
   //{ u8 length; char [length]; } [param_count]
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
   u8 conditional_count;
   u8 command_count;
   f32 robot_width;
   f32 robot_length;
   u32 flags;
   //char name[robot_name_length]
   //{ u8 length; char [length]; } [conditional_count]
   //Welcome_Command [command_count]
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

struct CurrentParameters_Group {
   u8 name_length;
   u8 param_count;
   //char name[name_length]
   //CurrentParameters_Parameter [param_count]
};

struct CurrentParameters_PacketHeader {
   u8 group_count;
   //CurrentParameters_Group default_group
   //CurrentParameters_Group [group_count]
};
//----------------------------------------

//-----------------------------------------
struct State_Diagnostic {
   u8 name_length;
   f32 value;
   u8 unit; //NOTE: North_Unit
   //char name[name_length]
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

struct State_Path {
   u16 length;
   u8 control_point_count;
   //char message[length]
   //North_HermiteControlPoint [control_point_count]
};

struct State_Group {
   u8 name_length;
   
   u8 diagnostic_count;
   u8 message_count;
   u8 marker_count;
   u8 path_count;
   
   //char name[name_length]

   //State_Diagnostic [diagnostic_count]
   //State_Message [message_count]
   //State_Marker [marker_count]
   //State_Path [path_count]
};

struct State_PacketHeader {
   v2 pos;
   f32 angle;
   
   u8 mode; //NOTE: North_GameMode

   u8 group_count;
   f32 time;
   
   //NOTE: default_group.name_length should always be 0
   //State_Group default_group;

   //State_Group [group_count]
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
   u8 group_name_length;
   u8 param_name_length;

   f32 value; //NOTE: only used by SetValue & AddValue
   u32 index; //NOTE: ignored if is_array = false
   
   //char [group_name_length]
   //char [param_name_length]
};
//--------------------------------------

struct SetState_PacketHeader {
   v2 pos;
   f32 angle;
};

//-------------------------------------
#if 0
//NOTE: From north_file_definitions.h -------------------------
struct AutonomousProgram_ContinuousEvent {
   u8 command_name_length;
   u8 datapoint_count;
   //char [command_name_length]
   //North_PathDataPoint [datapoint_count]
};

struct AutonomousProgram_DiscreteEvent {
   f32 distance;
   u8 command_name_length;
   u8 parameter_count;
   //char [command_name_length]
   //f32 [parameter_count]
};

struct AutonomousProgram_Path {
   //NOTE: begin & end points are (parent.pos, in_tangent) & (end_node.pos, out_tangent)
   v2 in_tangent;
   v2 out_tangent;

   u8 is_reverse;

   u8 conditional_length; //NOTE: if conditional_length is 0, there is no conditional
   u8 control_point_count;

   u8 velocity_datapoint_count;
   u8 continuous_event_count;
   u8 discrete_event_count;

   //char conditional_name [conditional_length]
   //North_HermiteControlPoint [control_point_count]

   //North_PathDataPoint [velocity_datapoint_count]
   //AutonomousProgram_ContinuousEvent [continuous_event_count]
   //AutonomousProgram_DiscreteEvent [discrete_event_count]

   //AutonomousProgram_Node end_node
};

struct AutonomousProgram_CommandHeader {
   u8 type; //NOTE: North_CommandType
   //AutonomousProgram_CommandBody_[type]
};

struct AutonomousProgram_CommandBody_Generic {
   u8 command_name_length;
   u8 parameter_count;
   
   //char [command_name_length]
   //f32 [parameter_count]
};

struct AutonomousProgram_CommandBody_Wait {
   f32 duration;
};

struct AutonomousProgram_CommandBody_Pivot {
   f32 start_angle;
   f32 end_angle;
   u8 turns_clockwise;
   u8 velocity_datapoint_count;
   u8 continuous_event_count;
   u8 discrete_event_count;

   //North_PathDataPoint [velocity_datapoint_count]
   //AutonomousProgram_ContinuousEvent [continuous_event_count]
   //AutonomousProgram_DiscreteEvent [discrete_event_count]
};

struct AutonomousProgram_Node {
   v2 pos;
   u8 command_count;
   u8 path_count;
   //AutonomousProgram_CommandHeader [command_count]
   //AutonomousProgram_Path [path_count]
};
#endif

struct UploadAutonomous_PacketHeader {
   f32 starting_angle;
   //AutonomousProgram_Node begining_node
};

#pragma pack(pop)