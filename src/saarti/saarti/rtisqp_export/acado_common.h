/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES

#include "acado_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 0
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 30
/** Number of online data values. */
#define ACADO_NOD 6
/** Number of path constraints. */
#define ACADO_NPAC 10
/** Number of control variables. */
#define ACADO_NU 4
/** Number of differential variables. */
#define ACADO_NX 7
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 11
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 7
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 120
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 5
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 2
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 31 x 7 (row major format)
 * 
 *  Matrix containing 31 differential variable vectors.
 */
real_t x[ 217 ];

/** Matrix of size: 30 x 4 (row major format)
 * 
 *  Matrix containing 30 control variable vectors.
 */
real_t u[ 120 ];

/** Matrix of size: 31 x 6 (row major format)
 * 
 *  Matrix containing 31 online data vectors.
 */
real_t od[ 186 ];

/** Column vector of size: 330
 * 
 *  Matrix containing 30 reference/measurement vectors of size 11 for first 30 nodes.
 */
real_t y[ 330 ];

/** Column vector of size: 7
 * 
 *  Reference/measurement vector for the 31. node.
 */
real_t yN[ 7 ];

/** Matrix of size: 11 x 11 (row major format) */
real_t W[ 121 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t WN[ 49 ];

/** Column vector of size: 7
 * 
 *  Current state feedback vector.
 */
real_t x0[ 7 ];

/** Column vector of size: 120
 * 
 *  Lower bounds values.
 */
real_t lbValues[ 120 ];

/** Column vector of size: 120
 * 
 *  Upper bounds values.
 */
real_t ubValues[ 120 ];

/** Column vector of size: 300
 * 
 *  Lower bounds values for affine constraints.
 */
real_t lbAValues[ 300 ];

/** Column vector of size: 300
 * 
 *  Upper bounds values for affine constraints.
 */
real_t ubAValues[ 300 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim14_swap;

/** Column vector of size: 14 */
real_t rk_dim14_bPerm[ 14 ];

/** Column vector of size: 30 */
real_t rhs_aux[ 30 ];

real_t rk_ttt;

/** Row vector of size: 17 */
real_t rk_xxx[ 17 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t rk_kkk[ 14 ];

/** Matrix of size: 14 x 14 (row major format) */
real_t rk_A[ 196 ];

/** Column vector of size: 14 */
real_t rk_b[ 14 ];

/** Row vector of size: 14 */
int rk_dim14_perm[ 14 ];

/** Column vector of size: 7 */
real_t rk_rhsTemp[ 7 ];

/** Matrix of size: 2 x 77 (row major format) */
real_t rk_diffsTemp2[ 154 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t rk_diffK[ 14 ];

/** Matrix of size: 7 x 11 (row major format) */
real_t rk_diffsPrev2[ 77 ];

/** Matrix of size: 7 x 11 (row major format) */
real_t rk_diffsNew2[ 77 ];

/** Row vector of size: 94 */
real_t state[ 94 ];

/** Column vector of size: 210 */
real_t d[ 210 ];

/** Column vector of size: 330 */
real_t Dy[ 330 ];

/** Column vector of size: 7 */
real_t DyN[ 7 ];

/** Matrix of size: 210 x 7 (row major format) */
real_t evGx[ 1470 ];

/** Matrix of size: 210 x 4 (row major format) */
real_t evGu[ 840 ];

/** Row vector of size: 17 */
real_t objValueIn[ 17 ];

/** Row vector of size: 11 */
real_t objValueOut[ 11 ];

/** Matrix of size: 210 x 7 (row major format) */
real_t Q1[ 1470 ];

/** Matrix of size: 210 x 11 (row major format) */
real_t Q2[ 2310 ];

/** Matrix of size: 120 x 4 (row major format) */
real_t R1[ 480 ];

/** Matrix of size: 120 x 11 (row major format) */
real_t R2[ 1320 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t QN1[ 49 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t QN2[ 49 ];

/** Column vector of size: 110 */
real_t conAuxVar[ 110 ];

/** Row vector of size: 17 */
real_t conValueIn[ 17 ];

/** Row vector of size: 120 */
real_t conValueOut[ 120 ];

/** Column vector of size: 300 */
real_t evH[ 300 ];

/** Matrix of size: 300 x 7 (row major format) */
real_t evHx[ 2100 ];

/** Matrix of size: 300 x 4 (row major format) */
real_t evHu[ 1200 ];

/** Column vector of size: 10 */
real_t evHxd[ 10 ];

/** Column vector of size: 7 */
real_t Dx0[ 7 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t T[ 49 ];

/** Matrix of size: 3255 x 4 (row major format) */
real_t E[ 13020 ];

/** Matrix of size: 3255 x 4 (row major format) */
real_t QE[ 13020 ];

/** Column vector of size: 210 */
real_t Qd[ 210 ];

/** Column vector of size: 217 */
real_t QDy[ 217 ];

/** Matrix of size: 120 x 7 (row major format) */
real_t H10[ 840 ];

/** Matrix of size: 300 x 7 (row major format) */
real_t A01[ 2100 ];

/** Column vector of size: 300 */
real_t pacA01Dx0[ 300 ];

/** Matrix of size: 120 x 120 (row major format) */
real_t H[ 14400 ];

/** Matrix of size: 300 x 120 (row major format) */
real_t A[ 36000 ];

/** Column vector of size: 120 */
real_t g[ 120 ];

/** Column vector of size: 120 */
real_t lb[ 120 ];

/** Column vector of size: 120 */
real_t ub[ 120 ];

/** Column vector of size: 300 */
real_t lbA[ 300 ];

/** Column vector of size: 300 */
real_t ubA[ 300 ];

/** Column vector of size: 120 */
real_t x[ 120 ];

/** Column vector of size: 420 */
real_t y[ 420 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 17 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_diffs(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 31 with xEnd. 2. Initialize node 31 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_getObjective(  );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
