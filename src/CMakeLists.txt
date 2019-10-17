project(Drumset_ATSAMC21)
    
add_executable(
    ${PROJECT_NAME}
)

target_sources(
    ${PROJECT_NAME} PRIVATE
        Midi.h
        DrumStateMachine.h
        main.cpp
)

target_link_libraries(
    ${PROJECT_NAME} PRIVATE
        LIBREUCPP_HAL_ATSAMC21X18
)
