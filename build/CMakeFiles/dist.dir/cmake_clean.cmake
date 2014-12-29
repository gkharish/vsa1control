FILE(REMOVE_RECURSE
  "doc/vsa1control.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/dist"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/dist.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
