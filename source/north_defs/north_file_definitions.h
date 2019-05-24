#pragma pack(push, 1)

struct FileHeader {
   u32 magic_number;
   u32 version_number;
};

FileHeader header(u32 magic_number, u32 version_number) {
   FileHeader result = { magic_number, version_number };
   return result;
}

//Settings----------------------------------------------------
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

//Field------------------------------------------------------
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

//RobotProfile-----------------------------------------------
struct RobotProfile_Parameter {
   u8 is_array;
   u8 name_length; 
   u8 value_count; //Ignored if is_array is false
   //char name[name_length]
   //f32 [value_count]
};

struct RobotProfile_Group {
   u8 name_length;
   u8 parameter_count;
   //char name[name_length]
   //RobotProfile_Parameter [parameter_count]
};

struct RobotProfile_Command {
   u8 name_length;
   u8 param_count;
   u8 type; //NOTE: North_CommandExecutionType
   //char name[name_length]
   //{ u8 length; char [length]; } [param_count]
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
   u8 conditional_count;
   u8 group_count;
   u8 command_count;
   f32 robot_width;
   f32 robot_length;
   u32 flags;
   //{ u8 length; char [length]; } [conditional_count]
   //RobotProfile_Group default_group
   //RobotProfile_Group [group_count]
   //RobotProfile_Command [command_count]
};
//-----------------------------------------------------------

//RobotRecording---------------------------------------------
struct RobotRecording_Diagnostic {
   u8 name_length;
   u8 unit; //NOTE: North_Unit
   u32 sample_count;
   //char name[name_length]
   //RobotRecording_DiagnosticSample [sample_count]
};

struct RobotRecording_DiagnosticSample {
   f32 value;
   f32 time;
};

struct RobotRecording_Message {
   u8 type; //NOTE North_MessageType
   u16 length;
   f32 begin_time;
   f32 end_time;
   //char message[length]
};

struct RobotRecording_Marker {
   v2 pos;
   u16 length;
   f32 begin_time;
   f32 end_time;
   //char message[length]
};

struct RobotRecording_Path {
   u16 length;
   u8 control_point_count;
   f32 begin_time;
   f32 end_time;
   //char message[length]
   //North_HermiteControlPoint [control_point_count]
};

struct RobotRecording_Group {
   u8 name_length;
   
   u8 diagnostic_count;
   u32 message_count;
   u32 marker_count;
   u32 path_count;
   
   //char name[name_length]

   //RobotRecording_Diagnostic [diagnostic_count]
   //RobotRecording_Message [message_count]
   //RobotRecording_Marker [marker_count]
   //RobotRecording_Path [path_count]
};

struct RobotRecording_RobotStateSample {
   v2 pos;
   f32 angle;
   f32 time;
};

/*
struct RobotRecording_Chunk {
   u32 robot_state_sample_count;
   u8 group_count;

   //RobotRecording_RobotStateSample [robot_state_sample_count]
   
   //RobotRecording_Group default_group
   //RobotRecording_Group [group_count]
}

struct RobotRecording_GroupSummary {
   u64 next_summary_offset;

   u8 name_length;
   
   u8 diagnostic_count;
   u32 message_count;
   u32 marker_count;
   u32 path_count;

   //char name[name_length]
};
*/

//TODO keep track of the totals somehow, like total diagnostic samples in a group

struct RobotRecording_FileHeader {
#define ROBOT_RECORDING_MAGIC_NUMBER RIFF_CODE("NCRR") 
#define ROBOT_RECORDING_CURR_VERSION 0
   u64 timestamp;

   u8 robot_name_length;
   f32 robot_width;
   f32 robot_length;

   u32 robot_state_sample_count;
   u8 group_count;

   //char [robot_name_length]

   //RobotRecording_RobotStateSample [robot_state_sample_count]
   
   //RobotRecording_Group default_group
   //RobotRecording_Group [group_count]
};
//-----------------------------------------------------------

//AutonomousProgram------------------------------------------
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
   //AutonomousProgram_Command [command_count]
   //AutonomousProgram_Path [path_count]
};

struct AutonomousProgram_FileHeader {
#define AUTONOMOUS_PROGRAM_MAGIC_NUMBER RIFF_CODE("NCAP") 
#define AUTONOMOUS_PROGRAM_CURR_VERSION 0
   //TODO: linked auto projects
   f32 starting_angle;
   //AutonomousProgram_Node begining_node
};
//-----------------------------------------------------------

#pragma pack(pop)