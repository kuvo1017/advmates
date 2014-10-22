#----------------------------------------------------------------------
#
# Manuals for ADVENTURE Mates
#
#----------------------------------------------------------------------

# cmake version
cmake_minimum_required(VERSION 2.8)

# setting for UseLATEX
set(LATEX_OUTPUT_PATH build)
include(UseLATEX.cmake)

#----------------------------------------------------------------------
# compiler
set(LATEX_COMPILER  platex)
set(LATEX_COMPILER_FLAGS
  ${LATEX_COMPILER_FLAGS}
  -file-line-error -shell-escape -kanji=utf8)

set(BIBTEX_COMPILER bibtex)

set(DVIPDF_CONVERTER dvipdfmx)
set(DVIPDF_FLAGS
  ${DVIPDF_FLAGS}
  -d 5)

#----------------------------------------------------------------------
# build manuals (tex->dvi)
set(MATES_MANUAL AdvMates-0.5-manual-jp)

add_latex_document(${MATES_MANUAL}.tex
  INPUTS jp-sect1.tex
  BIBFILES jp-ref.bib)

# If you will split a LaTeX document into multiple files and use
# the LaTeX \input or \include command to put them back together,
# add the INPUTS argument to ADD_LATEX_DOCUMENT.
# All the fies have to be located together.
#
# ADD_LATEX_DOCUMENT(MyDoc.tex
# INPUTS Chapter1.tex Chapter2.tex Chapter3.tex Chapter4.tex
# BIBFILES MyDoc.bib
# IMAGE_DIRS images
# USE_INDEX)

# build manuals (dvi->pdf)
add_custom_target(dvi2pdf ALL
  COMMAND ${DVIPDF_CONVERTER} ${DVIPDF_FLAGS}
  ${LATEX_OUTPUT_PATH}/${MATES_MANUAL}.dvi
  DEPENDS ${LATEX_OUTPUT_PATH}/${MATES_MANUAL}.dvi
  )

#----------------------------------------------------------------------
# clean target
set(CLEAN_TARGET
  ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${MATES_MANUAL}.pdf
  )
# ${LATEX_OUTPUT_PATH}/* are removed automatically.

set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${CLEAN_TARGET}"
  )
