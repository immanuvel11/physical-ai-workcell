# One object in the workcell, as perceived (world frame).
string id                    # stable track id, e.g. "red_box" (assigned by the world model)
string label                 # human-readable name, e.g. "red cube" (from geometry or the VLM)
string color                 # dominant colour name
string shape                 # box | cylinder | bin | tray | unknown
string category              # part | receptacle | obstacle
geometry_msgs/Pose pose      # centre of the object's bounding box, yaw only
geometry_msgs/Vector3 size   # extents along the object's x, y, z axes (m)
float32 confidence
bool graspable               # fits the gripper (narrow side <= max opening)
