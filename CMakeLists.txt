cmake_minimum_required(VERSION 2.6) # Pour éviter un warning au lancement de CMake, on doit préciser la version minimale

# Nom du projet
project(IK_PF)

include_directories(
	/usr/include/OGRE/
)

link_directories(
	/usr/lib
)

SET(EXECUTABLE_OUTPUT_PATH ../bin/)


file(
    GLOB_RECURSE
    source_files
    src/*
    src/joint/*
)

# Déclaration de l'exécutable
add_executable(
    IK_PF
    ${source_files}
)

target_link_libraries(
    IK_PF
	OgreMain
	OIS
)


