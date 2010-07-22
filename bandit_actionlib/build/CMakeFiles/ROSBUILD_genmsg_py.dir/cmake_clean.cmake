FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/bandit_actionlib/msg/__init__.py"
  "../src/bandit_actionlib/msg/_BanditAction.py"
  "../src/bandit_actionlib/msg/_BanditGoal.py"
  "../src/bandit_actionlib/msg/_BanditActionGoal.py"
  "../src/bandit_actionlib/msg/_BanditResult.py"
  "../src/bandit_actionlib/msg/_BanditActionResult.py"
  "../src/bandit_actionlib/msg/_BanditFeedback.py"
  "../src/bandit_actionlib/msg/_BanditActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
