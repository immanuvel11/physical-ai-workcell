std_msgs/Header header
string instruction
string state                 # IDLE | PERCEIVING | PLANNING | EXECUTING | VERIFYING | REPLANNING | PAUSED | SUCCEEDED | FAILED | NEEDS_CLARIFICATION
string planner               # claude | rule
string plan_json             # current plan
int32 current_step
string message
