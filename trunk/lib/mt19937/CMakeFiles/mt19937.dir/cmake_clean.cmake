FILE(REMOVE_RECURSE
  "CMakeCache.txt"
  "cmake_install.cmake"
  "CMakeFiles/mt19937.dir/mt19937ar.c.o"
  "libmt19937.pdb"
  "libmt19937.a"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang C)
  INCLUDE(CMakeFiles/mt19937.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
