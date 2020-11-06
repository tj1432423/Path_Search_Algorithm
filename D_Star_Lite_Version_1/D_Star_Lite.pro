HEADERS += \
    D_Star_Lite.h \
    D_Star_Lite_Node.h\
    Min_Heap_Map_Opt.h\
    \
    ../Lib/Min_Heap_Map_Opt.h


SOURCES += \
    main.cpp \
    D_Star_Lite.cpp\
    Plot.cpp

INCLUDEPATH +=\
               /usr/local/include \
                ../Lib


LIBS += -lGL -lglut
