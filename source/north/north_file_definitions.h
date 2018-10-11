#pragma pack(push, 1)

struct FileHeader {
   u32 magic_number;
   u32 version_number;
};

FileHeader header(u32 magic_number, u32 version_number) {
   FileHeader result = { magic_number, version_number };
   return result;
}

//-----------------------------------------------------------
//NOTE: unused right now
/*
namespace SettingFlags {
   enum type {

   };
};
*/

struct Settings_FileHeader {
#define SETTINGS_MAGIC_NUMBER RIFF_CODE("NCSF") 
#define SETTINGS_CURR_VERSION 0
   u16 team_number;
   u8 field_name_length;
   u32 flags;
   //char field_name[field_name_length]
};
//-----------------------------------------------------------

//-----------------------------------------------------------
namespace Field_Flags {
   enum type {
      MIRRORED = (1 << 0), //eg. steamworks
      SYMMETRIC = (1 << 1), //eg. powerup
   };
};

struct Field_StartingPosition {
   v2 pos;
   f32 angle;
};

struct Field_FileHeader {
#define FIELD_MAGIC_NUMBER RIFF_CODE("NCFF") 
#define FIELD_CURR_VERSION 0
   f32 width; //in ft
   f32 height; //in ft
   u32 flags;
   u8 starting_position_count;
   u16 image_width;
   u16 image_height;
   //Field_StartingPosition [starting_position_count]
   //u32 field_image [image_width * image_height] RGBA encoded
};
//-----------------------------------------------------------

//-----------------------------------------------------------
namespace RobotProfile_CommandType {
   enum type {
      NonBlocking = 0,
      Blocking = 1,
      Continuous = 2,
   };
};

struct RobotProfile_SubsystemCommand {
   u8 name_length;
   u8 param_count;
   u8 type;
   //char name[name_length]
   //{ u8 length; char [length]; } [param_count]
};

struct RobotProfile_SubsystemDescription {
   u8 name_length;
   u8 command_count;
   //char name[name_length]
   //RobotProfile_SubsystemCommand [command_count]
};

//NOTE: unused
/*
namespace RobotProfile_Flags {
   enum type {

   };
};
*/

struct RobotProfile_FileHeader {
#define ROBOT_PROFILE_MAGIC_NUMBER RIFF_CODE("NCRP") 
#define ROBOT_PROFILE_CURR_VERSION 0
   u8 robot_name_length;
   u8 subsystem_count;
   f32 robot_width;
   f32 robot_height;
   u32 flags;
   //char robot_name[robot_name_length]
   //RobotProfile_SubsystemDescription [subsystem_count]
};
//-----------------------------------------------------------

//-----------------------------------------------------------
struct AutonomousRun_SubsystemDiagnostics {
   u8 name_length;
   u8 diagnostic_count;
   //char name[name_length]
   //AutonomousRun_Diagnostic [diagnostic_count]
};

struct AutonomousRun_Diagnostic {
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
   u8 unit;
   u32 sample_count;
   //char name[name_length]
   //AutonomousRun_DiagnosticSample [sample_count]
};

struct AutonomousRun_DiagnosticSample {
   f32 value;
   f32 time;
};

struct AutonomousRun_RobotStateSample {
   v2 pos;
   v2 vel;
   f32 angle;
   f32 time;
};

struct AutonomousRun_FileHeader {
#define AUTONOMOUS_RUN_MAGIC_NUMBER RIFF_CODE("NCAR") 
#define AUTONOMOUS_RUN_CURR_VERSION 0
   u64 timestamp;
   u8 robot_name_length;
   u8 subsystem_count;
   u32 robot_state_sample_count;
   //char [robot_name_length]
   //AutonomousRun_RobotStateSample [robot_state_sample_count]
   //AutonomousRun_SubsystemDiagnostics [subsystem_count]
};
//-----------------------------------------------------------

//-----------------------------------------------------------
struct AutonomousProgram_DataPoint {
   f32 distance;
   f32 value;
};

struct AutonomousProgram_ContinuousEvent {
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 datapoint_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //AutonomousProgram_DataPoint [datapoint_count]
};

struct AutonomousProgram_DiscreteEvent {
   f32 distance;
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 parameter_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //f32 [parameter_count]
};

struct AutonomousProgram_Value {
   u8 is_variable;
   //f32 if !is_variable
   //struct { u8 length; char [length] } if is_variable
};

//TODO: linked auto projects
struct AutonomousProgram_Path {
   u8 is_reverse;

   //NOTE: begin & end points are parent.pos & end_node.pos
   u8 conditional_length; //NOTE: if conditional_length is 0, there is no conditional
   u8 control_point_count;

   u8 continuous_event_count;
   u8 discrete_event_count;

   //AutonomousProgram_Value accel
   //AutonomousProgram_Value deccel
   //AutonomousProgram_Value max_vel

   //char conditional_name [conditional_length]
   //v2 [control_point_count]
   //AutonomousProgram_ContinuousEvent [continuous_event_count]
   //AutonomousProgram_DiscreteEvent [discrete_event_count]

   //AutonomousProgram_Node end_node
};

struct AutonomousProgram_Command {
   u8 subsystem_name_length;
   u8 command_name_length;
   u8 parameter_count;
   //char [subsystem_name_length]
   //char [command_name_length]
   //f32 [parameter_count]
};

struct AutonomousProgram_Node {
   v2 pos;
   u8 command_count;
   u8 path_count;
   //AutonomousProgram_Command [command_count]
   //AutonomousProgram_Path [path_count]
};

struct AutonomousProgram_Variable {
   u8 name_length;
   f32 value;
   //char [name_length]
};

struct AutonomousProgram_FileHeader {
#define AUTONOMOUS_PROGRAM_MAGIC_NUMBER RIFF_CODE("NCAP") 
#define AUTONOMOUS_PROGRAM_CURR_VERSION 0
   u8 variable_count;
   //AutonomousProgram_Variable [variable_count]
   //AutonomousProgram_Node begining_node
};
//-----------------------------------------------------------

#pragma pack(pop)