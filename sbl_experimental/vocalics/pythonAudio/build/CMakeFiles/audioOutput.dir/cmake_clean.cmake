FILE(REMOVE_RECURSE
  "../bin/audioOutput.pdb"
  "../bin/audioOutput"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/audioOutput.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
