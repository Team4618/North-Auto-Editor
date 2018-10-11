#pragma pack(push, 1)

namespace PacketType {
   enum type {
      Ping = 0,
      Connect = 1,
      Welcome = 2,
      CurrentParameters = 3,
      State = 4,
      SetParameter = 5,
      SetState = 6,
      CurrentAutoPath = 7,
      UploadAutonomous = 8,
      //NOTE: if we change a packet just make a new type instead 
      //eg. "Welcome" becomes "Welcome_V1" & we create "Welcome_V2"
      Count
   };
};

//------------------------------------------
namespace Welcome_CommandType {
   enum type {
      NonBlocking = 0,
      Blocking = 1,
      Continuous = 2,
   };
};

struct Welcome_SubsystemCommand {
   u8 name_length;
   u8 param_count;
   u8 type;
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
namespace WelcomeFlags {
   enum type {

   };
};
*/

struct Welcome_PacketHeader {
   u8 packet_type;

   u8 robot_name_length;
   u8 subsystem_count;
   f32 robot_width;
   f32 robot_height;
   u32 flags;
   //char name[robot_name_length]
   //Welcome_SubsystemDescription [subsystem_count]
};
//-----------------------------------------

//-----------------------------------------
struct Parameter {
   u8 name_length;
   f32 value;
   //char name[name_length]
};

struct SubsystemParameters {
   u8 name_length;
   u8 param_count;
   //char name[name_length]
   //Parameter [param_count]
};

struct CurrentParameters_PacketHeader {
   u8 packet_type;
   u8 subsystem_count;
   //SubsystemParameters [subsystem_count]
};
//----------------------------------------

//-----------------------------------------
struct SubsystemDiagnostics {
   u8 name_length;
   u8 diagnostic_count;
   //char name[name_length]
   //Diagnostic [diagnostic_count]
};

struct Diagnostic {
   enum unit_type {
      Unitless = 0,
      Feet = 1,
      FeetPerSecond = 2,
      Degrees = 3,
      DegreesPerSecond = 4,
      Seconds = 5,
      Percent = 6,
      Amp = 7,
      Volt = 8,
   };

   u8 name_length;
   f32 value;
   u8 unit;
   //char name[name_length]
};

struct RobotMessage {
   enum message_type {
      Message = 0,
      Warning = 1,
      Error = 2,
      SubsystemOffline = 3,
      ControlDescription = 4,
   };

   u8 type;
   u8 length;
   //char message[length]
};

/*
enum GameMode {
   GameMode_Disabled = 0,
   GameMode_Autonomous = 1,
   GameMode_Teleop = 2,
   GameMode_Test = 3,
};
*/

struct State_PacketHeader {
   u8 packet_type; 
   
   v2 pos;
   v2 vel;
   f32 angle;
   
   u8 mode;
   u8 subsystem_count;
   u8 robot_message_count;
   f32 time;
   
   //SubsystemDiagnostics [subsystem_count]
   //RobotMessage [robot_message_count]
};
//-----------------------------------------

struct SetParameter_PacketHeader {
   u8 packet_type;
   u8 subsystem_name_length;
   u8 param_name_length;
   f32 value;
   //char [subsystem_name_length]
   //char [param_name_length]
};

struct SetState_PacketHeader {
   u8 packet_type;
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
   u8 packet_type;
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