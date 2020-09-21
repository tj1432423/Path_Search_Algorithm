HEADERS += \
    Dynamic_A_Star.h \
    #Node.h\
    Dynamic_A_Star_Node.h\
    Min_Heap_Map_Opt.h\
    #Adjacent_Node_Map.h\
    \
    ../Lib/Min_Heap_Map_Opt.h
 #   ../A_Star/A_Star_Node.h \
  #  ../A_Star/Node.h



SOURCES += \
    main.cpp \
    Dynamic_A_Star.cpp\
    Plot.cpp
    #Adjacent_Node_Map.cpp

INCLUDEPATH +=\
               /usr/local/include \
                ../Lib


LIBS += -lGL -lglut
