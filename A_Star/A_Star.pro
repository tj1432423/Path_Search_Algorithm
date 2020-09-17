HEADERS += \
    A_Star.h \
    Node.h\
    A_Star_Node.h\
    Min_Heap_Map_Opt.h\
    Adjacent_Node_Map.h


SOURCES += \
    main.cpp \
    A_Star.cpp\
    Plot.cpp
    #Adjacent_Node_Map.cpp

INCLUDEPATH +=\
               /usr/local/include \
                ../Lib


LIBS += -lGL -lglut
