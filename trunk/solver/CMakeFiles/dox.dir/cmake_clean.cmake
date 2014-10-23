FILE(REMOVE_RECURSE
  "CMakeCache.txt"
  "cmake_install.cmake"
  "../lib/libmates-util.a"
  "CMakeFiles/dox"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/dox.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
