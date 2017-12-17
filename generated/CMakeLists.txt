SET(QP_OASES_PATH ${PROJECT_SOURCE_DIR}/modules/acado/external_packages/qpoases)

set(QP_OASES_SRCS
        ${QP_OASES_PATH}/SRC/Bounds.cpp
        ${QP_OASES_PATH}/SRC/Constraints.cpp
        ${QP_OASES_PATH}/SRC/CyclingManager.cpp
        ${QP_OASES_PATH}/SRC/Indexlist.cpp
        ${QP_OASES_PATH}/SRC/MessageHandling.cpp
        ${QP_OASES_PATH}/SRC/QProblem.cpp
        ${QP_OASES_PATH}/SRC/QProblemB.cpp
        ${QP_OASES_PATH}/SRC/SubjectTo.cpp
        ${QP_OASES_PATH}/SRC/Utils.cpp
        ${QP_OASES_PATH}/SRC/EXTRAS/SolutionAnalysis.cpp
        )

set(QP_OASES_INCLUDES
        ${QP_OASES_PATH}
        ${QP_OASES_PATH}/INCLUDE
        ${QP_OASES_PATH}/SRC
        )


macro(MAKE_MPC_SOLVER MPC_SOLVER_NAME)

set(SOLVER_SRCS
        ${MPC_SOLVER_NAME}/acado_qpoases_interface.cpp
        ${MPC_SOLVER_NAME}/acado_integrator.c
        ${MPC_SOLVER_NAME}/acado_solver.c
        ${MPC_SOLVER_NAME}/acado_auxiliary_functions.c
        )

add_library(${MPC_SOLVER_NAME} STATIC ${QP_OASES_SRCS} ${SOLVER_SRCS})

target_include_directories(${MPC_SOLVER_NAME} PRIVATE
        ${MPC_SOLVER_NAME}
        ${QP_OASES_INCLUDES}
        )

endmacro(MAKE_MPC_SOLVER)

MAKE_MPC_SOLVER(mpc_6dof)