HEADERS += \
    Lifelong_Planning_A_Star.h \
    Lifelong_Planning_A_Star_Node.h\
    Min_Heap_Map_Opt.h\
    \
    ../Lib/Min_Heap_Map_Opt.h

SOURCES += \
    main.cpp \
    Lifelong_Planning_A_Star.cpp\
    Plot.cpp

INCLUDEPATH +=\
               /usr/local/include \
                ../Lib


LIBS += -lGL -lglut
