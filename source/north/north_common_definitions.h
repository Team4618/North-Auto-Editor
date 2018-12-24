namespace North_GameMode {
   enum type {
      Disabled = 0,
      Autonomous = 1,
      Teleop = 2,
      Test = 3,
   };
};

namespace North_Unit {
   enum type {
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
};

namespace North_MessageType {
   enum type {
      Message = 0,
      Warning = 1,
      Error = 2,
      SubsystemOffline = 3
   };
};

namespace North_CommandExecutionType {
   enum type {
      NonBlocking = 0,
      Blocking = 1,
      Continuous = 2,
   };
};

struct North_HermiteControlPoint {
   v2 pos;
   v2 tangent; 
};

struct North_PathDataPoint {
   f32 distance;
   f32 value;
};

namespace North_CommandType {
   enum type {
      Generic = 1,
      Wait = 2,
      Pivot = 3,
   };
};