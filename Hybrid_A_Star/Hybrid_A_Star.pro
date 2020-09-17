HEADERS += \
    ../A_Star/A_Star.h \
    ../A_Star/A_Star_Node.h\
    ../A_Star/Min_Heap_Map_Opt.h\
    ../A_Star/Adjacent_Node_Map.h \
            \
    #../RS_Lib/rs.h\
    ../Reedsshepp_Dubins/reeds_shepp.h\
            \
    Hybrid_A_Star.h\
    Hybrid_A_Star_Node.h

SOURCES += \
    ../A_Star/A_Star.cpp\
    #../A_Star/PNode_Map.cpp \
            \
    #../RS_Lib/rs.cpp\
    ../Reedsshepp_Dubins/reeds_shepp.cpp\
            \
    main.cpp \
    Hybrid_A_Star.cpp\
            \
    Plot.cpp


INCLUDEPATH +=\
               /usr/local/include \
                ../Lib


LIBS += -lGL -lglut
