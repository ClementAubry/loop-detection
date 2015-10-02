# -------------------------------------------------
# Project created by Zildjian 2011-09-09T11:32
# -------------------------------------------------
##############################
#
#Added : On affiche l'image résultat de LDKalman obtenu par matlab (../matlab/slam_modifClem_script2.m)
#
##############################
TARGET = LoopDetectionV10.1_Kalman
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp \
    frame.cpp \
    loopdetector.cpp \
    interval/interval.cpp \
    interval/iboolean.cpp \
    interval/box.cpp \
    interval/scalarTube.cpp
HEADERS += mainwindow.h \
    frame.h \
    loopdetector.h \
    interval/interval.h \
    interval/iboolean.h \
    interval/box.h \
    interval/scalarTube.h \
    utils.h
FORMS += mainwindow.ui

OTHER_FILES += \
    icons/zoom_reset.png \
    icons/zoom_out_512.png \
    icons/zoom_in_512.png \
    icons/newtonTest.png \
    icons/Loop.png
