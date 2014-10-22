FILE(REMOVE_RECURSE
  "CMakeCache.txt"
  "cmake_install.cmake"
  "../lib/libmates-util.a"
  "CMakeFiles/advmates-calc.dir/AppCalc.cpp.o"
  "CMakeFiles/advmates-calc.dir/mainCalc.cpp.o"
  "advmates-calc.pdb"
  "advmates-calc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/advmates-calc.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
