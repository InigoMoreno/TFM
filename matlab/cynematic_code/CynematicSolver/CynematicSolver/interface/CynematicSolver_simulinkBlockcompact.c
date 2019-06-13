/*
CynematicSolver : A fast customized optimization solver.

Copyright (C) 2013-2019 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME CynematicSolver_simulinkBlockcompact

#include "simstruc.h"



/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


/* include FORCES functions and defs */
#include "../include/CynematicSolver.h" 

#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef CynematicSolverinterface_float CynematicSolvernmpc_float;

extern void CynematicSolver_casadi2forces(double *x, double *y, double *l, double *p, double *f, double *nabla_f, double *c, double *nabla_c, double *h, double *nabla_h, double *hess, solver_int32_default stage, solver_int32_default iteration);
CynematicSolver_extfunc pt2function_CynematicSolver = &CynematicSolver_casadi2forces;




/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		return; /* Parameter mismatch will be reported by Simulink */
    }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 3 in total */
    if (!ssSetNumInputPorts(S, 3)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 800, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 4, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 200, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 800, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	solver_int32_default i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *x0 = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *xinit = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,2);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	CynematicSolver_params params;
	CynematicSolver_output output;
	CynematicSolver_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<800; i++)
	{ 
		params.x0[i] = (double) x0[i]; 
	}

	for( i=0; i<4; i++)
	{ 
		params.xinit[i] = (double) xinit[i]; 
	}

	for( i=0; i<200; i++)
	{ 
		params.all_parameters[i] = (double) all_parameters[i]; 
	}

	

	

    #if SET_PRINTLEVEL_CynematicSolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = CynematicSolver_solve(&params, &output, &info, fp , pt2function_CynematicSolver);

	#if SET_PRINTLEVEL_CynematicSolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<8; i++)
	{ 
		outputs[i] = (real_T) output.x001[i]; 
	}

	k=8; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x002[i]; 
	}

	k=16; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x003[i]; 
	}

	k=24; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x004[i]; 
	}

	k=32; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x005[i]; 
	}

	k=40; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x006[i]; 
	}

	k=48; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x007[i]; 
	}

	k=56; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x008[i]; 
	}

	k=64; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x009[i]; 
	}

	k=72; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x010[i]; 
	}

	k=80; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x011[i]; 
	}

	k=88; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x012[i]; 
	}

	k=96; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x013[i]; 
	}

	k=104; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x014[i]; 
	}

	k=112; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x015[i]; 
	}

	k=120; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x016[i]; 
	}

	k=128; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x017[i]; 
	}

	k=136; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x018[i]; 
	}

	k=144; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x019[i]; 
	}

	k=152; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x020[i]; 
	}

	k=160; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x021[i]; 
	}

	k=168; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x022[i]; 
	}

	k=176; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x023[i]; 
	}

	k=184; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x024[i]; 
	}

	k=192; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x025[i]; 
	}

	k=200; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x026[i]; 
	}

	k=208; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x027[i]; 
	}

	k=216; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x028[i]; 
	}

	k=224; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x029[i]; 
	}

	k=232; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x030[i]; 
	}

	k=240; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x031[i]; 
	}

	k=248; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x032[i]; 
	}

	k=256; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x033[i]; 
	}

	k=264; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x034[i]; 
	}

	k=272; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x035[i]; 
	}

	k=280; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x036[i]; 
	}

	k=288; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x037[i]; 
	}

	k=296; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x038[i]; 
	}

	k=304; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x039[i]; 
	}

	k=312; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x040[i]; 
	}

	k=320; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x041[i]; 
	}

	k=328; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x042[i]; 
	}

	k=336; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x043[i]; 
	}

	k=344; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x044[i]; 
	}

	k=352; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x045[i]; 
	}

	k=360; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x046[i]; 
	}

	k=368; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x047[i]; 
	}

	k=376; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x048[i]; 
	}

	k=384; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x049[i]; 
	}

	k=392; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x050[i]; 
	}

	k=400; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x051[i]; 
	}

	k=408; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x052[i]; 
	}

	k=416; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x053[i]; 
	}

	k=424; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x054[i]; 
	}

	k=432; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x055[i]; 
	}

	k=440; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x056[i]; 
	}

	k=448; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x057[i]; 
	}

	k=456; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x058[i]; 
	}

	k=464; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x059[i]; 
	}

	k=472; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x060[i]; 
	}

	k=480; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x061[i]; 
	}

	k=488; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x062[i]; 
	}

	k=496; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x063[i]; 
	}

	k=504; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x064[i]; 
	}

	k=512; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x065[i]; 
	}

	k=520; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x066[i]; 
	}

	k=528; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x067[i]; 
	}

	k=536; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x068[i]; 
	}

	k=544; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x069[i]; 
	}

	k=552; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x070[i]; 
	}

	k=560; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x071[i]; 
	}

	k=568; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x072[i]; 
	}

	k=576; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x073[i]; 
	}

	k=584; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x074[i]; 
	}

	k=592; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x075[i]; 
	}

	k=600; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x076[i]; 
	}

	k=608; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x077[i]; 
	}

	k=616; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x078[i]; 
	}

	k=624; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x079[i]; 
	}

	k=632; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x080[i]; 
	}

	k=640; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x081[i]; 
	}

	k=648; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x082[i]; 
	}

	k=656; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x083[i]; 
	}

	k=664; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x084[i]; 
	}

	k=672; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x085[i]; 
	}

	k=680; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x086[i]; 
	}

	k=688; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x087[i]; 
	}

	k=696; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x088[i]; 
	}

	k=704; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x089[i]; 
	}

	k=712; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x090[i]; 
	}

	k=720; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x091[i]; 
	}

	k=728; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x092[i]; 
	}

	k=736; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x093[i]; 
	}

	k=744; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x094[i]; 
	}

	k=752; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x095[i]; 
	}

	k=760; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x096[i]; 
	}

	k=768; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x097[i]; 
	}

	k=776; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x098[i]; 
	}

	k=784; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x099[i]; 
	}

	k=792; 
	for( i=0; i<8; i++)
	{ 
		outputs[k++] = (real_T) output.x100[i]; 
	}

	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


