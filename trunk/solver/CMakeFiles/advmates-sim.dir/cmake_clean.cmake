FILE(REMOVE_RECURSE
  "CMakeCache.txt"
  "cmake_install.cmake"
  "../lib/libmates-util.a"
  "CMakeFiles/advmates-sim.dir/mainSim.cpp.o"
  "advmates-sim.pdb"
  "advmates-sim"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/advmates-sim.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
