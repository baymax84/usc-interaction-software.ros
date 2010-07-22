FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg/cpp/bandit_actionlib/BanditAction.h"
  "../msg/cpp/bandit_actionlib/BanditGoal.h"
  "../msg/cpp/bandit_actionlib/BanditActionGoal.h"
  "../msg/cpp/bandit_actionlib/BanditResult.h"
  "../msg/cpp/bandit_actionlib/BanditActionResult.h"
  "../msg/cpp/bandit_actionlib/BanditFeedback.h"
  "../msg/cpp/bandit_actionlib/BanditActionFeedback.h"
  "../msg/BanditAction.msg"
  "../msg/BanditGoal.msg"
  "../msg/BanditActionGoal.msg"
  "../msg/BanditResult.msg"
  "../msg/BanditActionResult.msg"
  "../msg/BanditFeedback.msg"
  "../msg/BanditActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
