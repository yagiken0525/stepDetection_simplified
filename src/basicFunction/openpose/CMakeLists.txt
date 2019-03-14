cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

add_library(myOpenPose STATIC myOpenPose.cpp myOpenPose.h openPosePerson.h)
target_link_libraries(myOpenPose ${OpenPose_LIBS} ${GFLAGS_LIBRARY} ${GLOG_LIBRARY})