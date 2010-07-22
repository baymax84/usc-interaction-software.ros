FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
