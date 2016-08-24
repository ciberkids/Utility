

generate_arduino_library(UtilityLib
        BOARD ${BOARD}
        SRCS Utility.cpp
        HDRS Utility.h
        LIBS
            RF24NetworkLib
            MessageLib
            BlinkLedLib
        )

