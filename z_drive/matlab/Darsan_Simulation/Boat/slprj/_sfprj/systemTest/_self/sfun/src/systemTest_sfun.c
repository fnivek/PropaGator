/* Include files */

#include "systemTest_sfun.h"
#include "systemTest_sfun_debug_macros.h"
#include "c5_systemTest.h"
#include "c6_systemTest.h"
#include "c11_systemTest.h"
#include "c12_systemTest.h"
#include "c13_systemTest.h"
#include "c14_systemTest.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _systemTestMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void systemTest_initializer(void)
{
}

void systemTest_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_systemTest_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==5) {
    c5_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==11) {
    c11_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==12) {
    c12_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==13) {
    c13_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==14) {
    c14_systemTest_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_systemTest_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1099567267U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2208162857U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3097928424U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1750735410U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3164848081U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(505625767U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(677444544U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(289157996U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 5:
        {
          extern void sf_c5_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c5_systemTest_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c6_systemTest_get_check_sum(plhs);
          break;
        }

       case 11:
        {
          extern void sf_c11_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c11_systemTest_get_check_sum(plhs);
          break;
        }

       case 12:
        {
          extern void sf_c12_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c12_systemTest_get_check_sum(plhs);
          break;
        }

       case 13:
        {
          extern void sf_c13_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c13_systemTest_get_check_sum(plhs);
          break;
        }

       case 14:
        {
          extern void sf_c14_systemTest_get_check_sum(mxArray *plhs[]);
          sf_c14_systemTest_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3846396017U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2375083232U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3891955466U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3982300213U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_systemTest_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 5:
      {
        if (strcmp(aiChksum, "OGtIbO1EJXRIr3qaWRc3iG") == 0) {
          extern mxArray *sf_c5_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c5_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "c4vh0XkbdJ0oD5uPaaxcXB") == 0) {
          extern mxArray *sf_c6_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c6_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 11:
      {
        if (strcmp(aiChksum, "BUKfB8tKjdBPY8vOhuvC5") == 0) {
          extern mxArray *sf_c11_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c11_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 12:
      {
        if (strcmp(aiChksum, "HQm503MWmILAdwyCur14cD") == 0) {
          extern mxArray *sf_c12_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c12_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 13:
      {
        if (strcmp(aiChksum, "pX95119APfO9VEIfo33zzE") == 0) {
          extern mxArray *sf_c13_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c13_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 14:
      {
        if (strcmp(aiChksum, "qqwHmPudMsVVTgCZYws4jH") == 0) {
          extern mxArray *sf_c14_systemTest_get_autoinheritance_info(void);
          plhs[0] = sf_c14_systemTest_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_systemTest_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 5:
      {
        extern const mxArray *sf_c5_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray *sf_c6_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 11:
      {
        extern const mxArray *sf_c11_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c11_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 12:
      {
        extern const mxArray *sf_c12_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c12_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 13:
      {
        extern const mxArray *sf_c13_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c13_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 14:
      {
        extern const mxArray *sf_c14_systemTest_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c14_systemTest_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_systemTest_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 5:
      {
        if (strcmp(tpChksum, "d2B9u8dKfnjgp1dAxCgiVD") == 0) {
          extern mxArray *sf_c5_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c5_systemTest_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "Z0hFTivgFri28ECNjWjbzG") == 0) {
          extern mxArray *sf_c6_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c6_systemTest_third_party_uses_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "zs8ZbmCyeMDX55sWZi0I9B") == 0) {
          extern mxArray *sf_c11_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c11_systemTest_third_party_uses_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "qnp5G443LexPt5LTK9KrUH") == 0) {
          extern mxArray *sf_c12_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c12_systemTest_third_party_uses_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "9GDaUZcGtkfhJPHFCHR0wG") == 0) {
          extern mxArray *sf_c13_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c13_systemTest_third_party_uses_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "NHD3ZtD4sO74GfViFeXQMD") == 0) {
          extern mxArray *sf_c14_systemTest_third_party_uses_info(void);
          plhs[0] = sf_c14_systemTest_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_systemTest_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 5:
      {
        if (strcmp(tpChksum, "d2B9u8dKfnjgp1dAxCgiVD") == 0) {
          extern mxArray *sf_c5_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "Z0hFTivgFri28ECNjWjbzG") == 0) {
          extern mxArray *sf_c6_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "zs8ZbmCyeMDX55sWZi0I9B") == 0) {
          extern mxArray *sf_c11_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c11_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "qnp5G443LexPt5LTK9KrUH") == 0) {
          extern mxArray *sf_c12_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c12_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "9GDaUZcGtkfhJPHFCHR0wG") == 0) {
          extern mxArray *sf_c13_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c13_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "NHD3ZtD4sO74GfViFeXQMD") == 0) {
          extern mxArray *sf_c14_systemTest_updateBuildInfo_args_info(void);
          plhs[0] = sf_c14_systemTest_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void systemTest_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _systemTestMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "systemTest","sfun",0,6,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_systemTestMachineNumber_,
    0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_systemTestMachineNumber_,0);
}

void systemTest_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_systemTest_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("systemTest",
      "systemTest");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_systemTest_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
