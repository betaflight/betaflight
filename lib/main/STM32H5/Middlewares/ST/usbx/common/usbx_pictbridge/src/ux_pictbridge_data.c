/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   PictBridge Host Class                                               */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/
#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_pictbridge.h"

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_pictbridge_data.c                                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the data items for the pictbridge class      */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/

/* Prototypes for all XML items.  */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input[];  
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_root[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_configureprintservice[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_configureprintservice[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getcapability[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getcapability[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getcapability_capability[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getcapability_capability[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getjobstatus[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getdevicestatus[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob_printinfo[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob_jobconfig[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_abortjob[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_notifyjobstatus[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_notifydevicestatus[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfileid[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfileid[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfileinfo[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfileinfo[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getthumb[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getthumb[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfile[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfile[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getpartialfile[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getpartialfile[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfilelist[];
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfilelist[];


/* Define Pictbridge XML input get File List. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfilelist[] = 
{
    { 
        "fileType", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfilelist_filetype,
    },

    { 
        "parentFileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfilelist_parentfileid,
    },

    { 
        "maxNumIDs", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfilelist_maxnumids,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get File List. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfilelist[] = 
{
    { 
        "fileIDs", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfilelist_fileids,
    },
    { 
        "numIDs", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfilelist_numids,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input get partial File. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getpartialfile[] = 
{
    { 
        "fileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getpartialfile_fileid,
    },

    { 
        "offset", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getpartialfile_offset,
    },

    { 
        "maxSize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getpartialfile_maxsize,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get Partial File. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getpartialfile[] = 
{
    { 
        "bytesRead", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getpartialfile_bytesread,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input get File. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfile[] = 
{
    { 
        "fileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfile_fileid,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get File. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfile[] = 
{
    { 
        "bytesRead", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfile_bytesread,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input get Thumb. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getthumb[] = 
{
    { 
        "fileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getthumb_fileid,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get Thumb. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getthumb[] = 
{
    { 
        "bytesRead", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getthumb_bytesread,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};
/* Define Pictbridge XML input get File Info. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfileinfo[] = 
{
    { 
        "fileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfileid_fileid,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get File ID. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfileinfo[] = 
{
    { 
        "fileType", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileinfo_filetype,
    },

    { 
        "fileSize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileinfo_filesize,
    },

    { 
        "thumbFormat", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileinfo_thumbformat,
    },

    { 
        "thumbsize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileinfo_thumbsize,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input get File ID. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getfileid[] = 
{
    { 
        "basePathID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfileid_basepathid,
    },

    { 
        "filePath", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_getfileid_filepath,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML output get File ID. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getfileid[] = 
{
    { 
        "basePathID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileid_basepathid,
    },

    { 
        "filePath", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getfileid_filepath,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML output get device status. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_notifydevicestatus[] = 
{
    { 
        "dpsPrintServiceStatus", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_dpsprintservicestatus,
    },

    { 
        "jobEndReason", 
        0,
        UX_PICTBRIDGE_XML_LEAF,                      
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_jobendreason,
    },

    { 
        "errorStatus", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_errorstatus,
    },
    { 
        "errorReason", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_errorreason,
    },
    { 
        "disconnectEnable", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_disconnectenable,
    },

    { 
        "capabilityChanged", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_capabilitychanged,
    },

    { 
        "newJobOK", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifydevicestatus_newjobok,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML input notify job.  */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_notifyjobstatus[] = 
{
    { 
        "prtPID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifyjobstatus_prtpid,
    },
    { 
        "filepath", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifyjobstatus_filepath,
    },
    { 
        "copyID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifyjobstatus_copyid,
    },
    { 
        "progress", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_notifyjobstatus_progress,
    },
    { 
        "imagesPrinted", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_output_notifyjobstatus_imagesprinted,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input abort job.  */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_abortjob[] = 
{
    { 
        "abortStyle", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_abortjob,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML input start job job config. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob_jobconfig[] = 
{
    { 
        "quality", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_quality,
    },
    { 
        "paperSize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_papersize,
    },
    { 
        "paperType", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_papertype,
    },
    { 
        "fileType", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_filetype,
    },
    { 
        "datePrint", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_dateprint,
    },
    { 
        "fileNamePrint", 
        0,
        UX_PICTBRIDGE_XML_LEAF,                      
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_filenameprint,
    },
    { 
        "imageOptimize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_imageoptimize,
    },
    { 
        "layout", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_layout,
    },
    { 
        "fixedSize", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_fixedsize,
    },
    { 
        "cropping", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_jobconfig_cropping,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML input print info. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob_printinfo[] = 
{
    { 
        "croppingArea", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_croppingarea,
    },
    { 
        "fileID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_fileid,
    },
    { 
        "fileName", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_filename,
    },
    { 
        "date", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_date,
    },
    { 
        "copies", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_copies,
    },
    { 
        "prtPID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_prtpid,
    },
    { 
        "filepath", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_filepath,
    },
    { 
        "copyID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_startjob,
        _ux_pictbridge_xml_function_input_startjob_printinfo_copyid,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML input start job. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_startjob[] = 
{
    { 
        "jobConfig", 
        0,
        _ux_pictbridge_xml_item_input_startjob_jobconfig,
        _ux_pictbridge_xml_item_input,
        UX_NULL,
    },
    { 
        "printInfo", 
        0,
        _ux_pictbridge_xml_item_input_startjob_printinfo,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_startjob_printinfo,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML output get device status. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getdevicestatus[] = 
{
    { 
        "dpsPrintServiceStatus", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_dpsprintservicestatus,
    },

    { 
        "jobEndReason", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_jobendreason,
    },

    { 
        "errorStatus", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_errorstatus,
    },
    { 
        "errorReason", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_errorreason,
    },
    { 
        "disconnectEnable", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_disconnectenable,
    },

    { 
        "capabilityChanged", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_capabilitychanged,
    },

    { 
        "newJobOK", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getdevicestatus_newjobok,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML output get job status. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getjobstatus[] = 
{
    { 
        "prtPID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getjobstatus_prtpid,
    },
    { 
        "filePath", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getjobstatus_filepath,
    },
    { 
        "copyID", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getjobstatus_copyid,
    },

    { 
        "progress", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getjobstatus_progress,
    },

    { 
        "imagesPrinted", 
        0,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_getjobstatus_imagesprinted,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML input get capability elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getcapability_capability[] = 
{
    { 
        "qualities", 
        UX_PICTBRIDGE_IR_GC_QUALITIES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_qualities,
    },
    { 
        "paperSizes", 
        UX_PICTBRIDGE_IR_GC_PAPER_SIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_papersizes,
    },
    { 
        "paperTypes", 
        UX_PICTBRIDGE_IR_GC_PAPER_TYPES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_papertypes,
    },
    { 
        "fileTypes", 
        UX_PICTBRIDGE_IR_GC_FILE_TYPES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_filetypes,
    },
    { 
        "datePrints", 
        UX_PICTBRIDGE_IR_GC_DATE_PRINTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_dateprints,
    },
    { 
        "fileNamePrints", 
        UX_PICTBRIDGE_IR_GC_FILE_NAME_PRINTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_filenameprints,
    },
    { 
        "imageOptimizes", 
        UX_PICTBRIDGE_IR_GC_IMAGE_OPTIMIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_imageoptimizes,
    },
    { 
        "layouts", 
        UX_PICTBRIDGE_IR_GC_LAYOUTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_layouts,
    },
    { 
        "fixedSizes", 
        UX_PICTBRIDGE_IR_GC_FIXED_SIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_fixedsizes,
    },
    { 
        "croppings", 
        UX_PICTBRIDGE_IR_GC_CROPPINGS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_croppings,
    },
    { 
        "charRepertoires", 
        UX_PICTBRIDGE_IR_GC_CHAR_REPERTOIRES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input_getcapability,
        _ux_pictbridge_xml_function_input_getcapability_capability_charrepertoires,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML get capability elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_getcapability[] = 
{
    { 
        "capability", 
        0,
        _ux_pictbridge_xml_item_input_getcapability_capability,
        _ux_pictbridge_xml_item_input,
        UX_NULL,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }

};

/* Define Pictbridge XML output get capability elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getcapability_capability[] = 
{
    { 
        "qualities", 
        UX_PICTBRIDGE_IR_GC_QUALITIES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_qualities,
    },
    { 
        "paperSizes", 
        UX_PICTBRIDGE_IR_GC_PAPER_SIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_papersizes,
    },
    { 
        "paperTypes", 
        UX_PICTBRIDGE_IR_GC_PAPER_TYPES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_papertypes,
    },
    { 
        "fileTypes", 
        UX_PICTBRIDGE_IR_GC_FILE_TYPES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_filetypes,
    },
    { 
        "datePrints", 
        UX_PICTBRIDGE_IR_GC_DATE_PRINTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_dateprints,
    },
    { 
        "fileNamePrints", 
        UX_PICTBRIDGE_IR_GC_FILE_NAME_PRINTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_filenameprints,
    },
    { 
        "imageOptimizes", 
        UX_PICTBRIDGE_IR_GC_IMAGE_OPTIMIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_imageoptimizes,
    },
    { 
        "layouts", 
        UX_PICTBRIDGE_IR_GC_LAYOUTS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_layouts,
    },
    { 
        "fixedSizes", 
        UX_PICTBRIDGE_IR_GC_FIXED_SIZES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_fixedsizes,
    },
    { 
        "croppings", 
        UX_PICTBRIDGE_IR_GC_CROPPINGS,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_croppings,
    },
    { 
        "charRepertoires", 
        UX_PICTBRIDGE_IR_GC_CHAR_REPERTOIRES,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_output_getcapability,
        _ux_pictbridge_xml_function_output_getcapability_capability_charrepertoires,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};

/* Define Pictbridge XML get capability elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_getcapability[] = 
{
    { 
        "capability", 
        0,
        _ux_pictbridge_xml_item_output_getcapability_capability,
        _ux_pictbridge_xml_item_output,
        UX_NULL,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }

};


/* Define Pictbridge XML configure print service elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input_configureprintservice[] = 
{
    { 
        "dpsVersions", 
        UX_PICTBRIDGE_IR_CPS_DPS_VERSIONS,
        UX_PICTBRIDGE_XML_LEAF, 
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_dpsversion,                        
    },
    { 
        "vendorName", 
        UX_PICTBRIDGE_IR_CPS_VENDOR_NAME,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_vendorname,
    },
    { 
        "vendorSpecificVersion", 
        UX_PICTBRIDGE_IR_CPS_VENDOR_SPECIFIC_VERSION,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_vendorspecificversion,
    },
    { 
        "productName", 
        UX_PICTBRIDGE_IR_CPS_PRODUCT_NAME,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_productname,
    },
    { 
        "serialNo", 
        UX_PICTBRIDGE_IR_CPS_SERIAL_NO,
        UX_PICTBRIDGE_XML_LEAF,
        _ux_pictbridge_xml_item_input,
        _ux_pictbridge_xml_function_input_serialno,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML configure print service elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output_configureprintservice[] = 
{
    { 
        "printServiceAvailable", 
        0,
        UX_NULL, 
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_printserviceavailable,                        
    },
    { 
        "dpsVersions", 
        0,
        UX_NULL, 
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_dpsversion,                        
    },
    { 
        "vendorName", 
        0,
        UX_NULL,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_vendorname,
    },
    { 
        "vendorSpecificVersion", 
        0,
        UX_NULL,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_vendorspecificversion,
    },
    { 
        "productName", 
        0,
        UX_NULL,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_productname,
    },
    { 
        "serialNo", 
        0,
        UX_NULL,
        _ux_pictbridge_xml_item_output,
        _ux_pictbridge_xml_function_output_serialno,
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }
};


/* Define Pictbridge XML input elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_input[] = 
{
    { 
        "configurePrintService", 
        UX_PICTBRIDGE_IR_CONFIGURE_PRINT_SERVICE,
        _ux_pictbridge_xml_item_input_configureprintservice, 
        _ux_pictbridge_xml_item_root,
        UX_NULL, 
    },
    { 
        "getCapability", 
        UX_PICTBRIDGE_IR_GET_CAPABILITY,
        _ux_pictbridge_xml_item_input_getcapability, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                  
    },
    { 
        "getDeviceStatus", 
        UX_PICTBRIDGE_IR_GET_DEVICE_STATUS,
        UX_NULL,                   
        _ux_pictbridge_xml_item_root,
        UX_NULL, 
    },
    { 
        "getJobStatus", 
        UX_PICTBRIDGE_IR_GET_JOB_STATUS,
        UX_NULL,                   
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_input_getjobstatus, 
    },
    { 
        "startJob", 
        UX_PICTBRIDGE_IR_START_JOB,
        _ux_pictbridge_xml_item_input_startjob, 
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_input_startjob,     
    },
    { 
        "abortJob", 
        UX_PICTBRIDGE_IR_ABORT_JOB,
        _ux_pictbridge_xml_item_input_abortjob, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                            
    },
    { 
        "continueJob", 
        UX_PICTBRIDGE_IR_CONTINUE_JOB,
        UX_NULL,
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_input_continuejob, 
    },
    { 
        "notifyJobStatus", 
        UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS,
        _ux_pictbridge_xml_item_input_notifyjobstatus,                      
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_input_notifyjobstatus, 
    },
    { 
        "notifyDeviceStatus", 
        UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS,
        _ux_pictbridge_xml_item_input_notifydevicestatus,                      
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_input_notifydevicestatus, 
    },
    { 
        "getFileID", 
        0,
        _ux_pictbridge_xml_item_input_getfileid, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFileInfo", 
        0,
        _ux_pictbridge_xml_item_input_getfileinfo, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFile", 
        0,
        _ux_pictbridge_xml_item_input_getfile, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getPartialFile", 
        0,
        _ux_pictbridge_xml_item_input_getpartialfile, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFileList", 
        0,
        _ux_pictbridge_xml_item_input_getfilelist, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getThumb", 
        0,
        _ux_pictbridge_xml_item_input_getthumb, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }

};

/* Define Pictbridge XML output elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_output[] = 
{
    { 
        "result", 
        0,
        UX_NULL,
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_result, 
    },
    { 
        "configurePrintService", 
        UX_PICTBRIDGE_IR_CONFIGURE_PRINT_SERVICE,
        _ux_pictbridge_xml_item_output_configureprintservice, 
        _ux_pictbridge_xml_item_root,
        UX_NULL, 
    },
    { 
        "getCapability", 
        UX_PICTBRIDGE_IR_GET_CAPABILITY,
        _ux_pictbridge_xml_item_output_getcapability, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                  
    },
    { 
        "getDeviceStatus", 
        UX_PICTBRIDGE_IR_GET_DEVICE_STATUS,
        _ux_pictbridge_xml_item_output_getdevicestatus, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                   
    },
    { 
        "getJobStatus", 
        UX_PICTBRIDGE_IR_GET_JOB_STATUS,
        _ux_pictbridge_xml_item_output_getjobstatus, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                   
    },
    { 
        "startJob", 
        UX_PICTBRIDGE_IR_START_JOB,
        UX_NULL, 
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_startjob,                            
    },
    { 
        "abortJob", 
        UX_PICTBRIDGE_IR_ABORT_JOB,
        UX_NULL,
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_abortjob,                            
    },
    { 
        "continueJob", 
        UX_PICTBRIDGE_IR_CONTINUE_JOB,
        UX_NULL,                      
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_continuejob, 
    },
    { 
        "notifyJobStatus", 
        0,
        UX_NULL,                      
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_notifyjobstatus, 
    },
    { 
        "notifyDeviceStatus", 
        0,
        UX_NULL,                      
        _ux_pictbridge_xml_item_root,
        _ux_pictbridge_xml_function_output_notifydevicestatus, 
    },
    { 
        "getFileID", 
        0,
        _ux_pictbridge_xml_item_output_getfileid, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFileInfo", 
        0,
        _ux_pictbridge_xml_item_output_getfileinfo, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFile", 
        0,
        _ux_pictbridge_xml_item_output_getfile, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getPartialFile", 
        0,
        _ux_pictbridge_xml_item_output_getpartialfile, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getFileList", 
        0,
        _ux_pictbridge_xml_item_output_getfilelist, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },
    { 
        "getThumb", 
        0,
        _ux_pictbridge_xml_item_output_getthumb, 
        _ux_pictbridge_xml_item_root,
        UX_NULL,                      
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }

};


/* Define Pictbridge XML Root elements. */
UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_root[] = 
{
    { 
        "xml", 
        0,
        UX_NULL, 
        UX_NULL, 
        _ux_pictbridge_xml_function_root_xml,     
    },
    
    { 
        "dps", 
        0,
        UX_NULL, 
        UX_NULL, 
        _ux_pictbridge_xml_function_root_dps,          
    },
    { 
        "input", 
        0,
        _ux_pictbridge_xml_item_input, 
        UX_NULL, 
        _ux_pictbridge_xml_function_root_input,     
    },
    { 
        "output", 
        0,
        _ux_pictbridge_xml_item_output, 
        UX_NULL, 
        _ux_pictbridge_xml_function_root_output,     
    },

    { "\0", 0, UX_NULL, UX_NULL, UX_NULL }

};


/* Define Pictbridge Discovery file names.  */
UCHAR _ux_pictbridge_ddiscovery_name[]                              =   "DDISCVRY.DPS";
UCHAR _ux_pictbridge_hdiscovery_name[]                              =   "HDISCVRY.DPS";
UCHAR _ux_pictbridge_hrequest_name[]                                =   "HREQUEST.DPS";
UCHAR _ux_pictbridge_hrsponse_name[]                                =   "HRSPONSE.DPS";
UCHAR _ux_pictbridge_drequest_name[]                                =   "DREQUEST.DPS";
UCHAR _ux_pictbridge_drsponse_name[]                                =   "DRSPONSE.DPS";

/* Define Pictbridge client volume strings.  */
UCHAR _ux_pictbridge_volume_description[]                           =   "PIMA Client Storage Volume"; 
UCHAR _ux_pictbridge_volume_label[]                                 =   "PIMA Client Storage Label"; 

/* Define Pictbridge XML tag lines or tag names.  */
UCHAR _ux_pictbridge_xml_tag_line_xmlversion[]                      =   "?xml version=\"1.0\"?" ;
UCHAR _ux_pictbridge_xml_tag_line_dpsxmlns[]                        =   "dps xmlns=\"http://www.cipa.jp/dps/schema/\""; 
UCHAR _ux_pictbridge_xml_tag_line_dps[]                             =   "dps";
UCHAR _ux_pictbridge_xml_tag_line_output[]                          =   "output";
UCHAR _ux_pictbridge_xml_tag_line_input[]                           =   "input";
UCHAR _ux_pictbridge_xml_tag_line_result[]                          =   "result";
UCHAR _ux_pictbridge_xml_tag_line_configureprintservice[]           =   "configurePrintService"; 
UCHAR _ux_pictbridge_xml_tag_line_printserviceavailable[]           =   "printServiceAvailable";
UCHAR _ux_pictbridge_xml_tag_line_dpsversions[]                     =   "dpsVersions";
UCHAR _ux_pictbridge_xml_tag_line_vendorname[]                      =   "vendorName";
UCHAR _ux_pictbridge_xml_tag_line_vendorspecificversion[]           =   "vendorSpecificVersion"; 
UCHAR _ux_pictbridge_xml_tag_line_productname[]                     =   "productName";
UCHAR _ux_pictbridge_xml_tag_line_serialno[]                        =   "serialNo";
UCHAR _ux_pictbridge_xml_tag_line_capability[]                      =   "capability";
UCHAR _ux_pictbridge_xml_tag_line_getcapability[]                   =   "getCapability";
UCHAR _ux_pictbridge_xml_tag_line_qualities[]                       =   "qualities";
UCHAR _ux_pictbridge_xml_tag_line_papersizes[]                      =   "paperSizes"; 
UCHAR _ux_pictbridge_xml_tag_line_papertypes[]                      =   "paperTypes"; 
UCHAR _ux_pictbridge_xml_tag_line_filetypes[]                       =   "fileTypes"; 
UCHAR _ux_pictbridge_xml_tag_line_dateprints[]                      =   "datePrints"; 
UCHAR _ux_pictbridge_xml_tag_line_filenameprints[]                  =   "fileNamePrints"; 
UCHAR _ux_pictbridge_xml_tag_line_imageoptimizes[]                  =   "imageOptimizes"; 
UCHAR _ux_pictbridge_xml_tag_line_layouts[]                         =   "layouts"; 
UCHAR _ux_pictbridge_xml_tag_line_fixedsizes[]                      =   "fixedSizes"; 
UCHAR _ux_pictbridge_xml_tag_line_croppings[]                       =   "croppings"; 
UCHAR _ux_pictbridge_xml_tag_line_charrepertoires[]                 =   "charRepertoires"; 
UCHAR _ux_pictbridge_xml_tag_line_getdevicestatus[]                 =   "getDeviceStatus"; 
UCHAR _ux_pictbridge_xml_tag_line_dpsprintservicestatus[]           =   "dpsPrintServiceStatus"; 
UCHAR _ux_pictbridge_xml_tag_line_jobendreason[]                    =   "jobEndReason"; 
UCHAR _ux_pictbridge_xml_tag_line_errorstatus[]                     =   "errorStatus"; 
UCHAR _ux_pictbridge_xml_tag_line_errorreason[]                     =   "errorReason"; 
UCHAR _ux_pictbridge_xml_tag_line_disconnectenable[]                =   "disconnectEnable"; 
UCHAR _ux_pictbridge_xml_tag_line_capabilitychanged[]               =   "capabilityChanged"; 
UCHAR _ux_pictbridge_xml_tag_line_newjobok[]                        =   "newJobOK"; 
UCHAR _ux_pictbridge_xml_tag_line_notifydevicestatus[]              =   "notifyDeviceStatus"; 
UCHAR _ux_pictbridge_xml_tag_line_notifyjobstatus[]                 =   "notifyJobStatus"; 
UCHAR _ux_pictbridge_xml_tag_line_progress[]                        =   "progress"; 
UCHAR _ux_pictbridge_xml_tag_line_imagesprinted[]                   =   "imagesPrinted"; 
UCHAR _ux_pictbridge_xml_tag_line_startjob[]                        =   "startJob";
UCHAR _ux_pictbridge_xml_tag_line_abortjob[]                        =   "abortJob";
UCHAR _ux_pictbridge_xml_tag_line_abortstyle[]                      =   "abortStyle";
UCHAR _ux_pictbridge_xml_tag_line_continuejob[]                     =   "continueJob";
UCHAR _ux_pictbridge_xml_tag_line_quality[]                         =   "quality";
UCHAR _ux_pictbridge_xml_tag_line_papersize[]                       =   "paperSize"; 
UCHAR _ux_pictbridge_xml_tag_line_papertype[]                       =   "paperType"; 
UCHAR _ux_pictbridge_xml_tag_line_filetype[]                        =   "fileType"; 
UCHAR _ux_pictbridge_xml_tag_line_dateprint[]                       =   "datePrint"; 
UCHAR _ux_pictbridge_xml_tag_line_filenameprint[]                   =   "fileNamePrint"; 
UCHAR _ux_pictbridge_xml_tag_line_imageoptimize[]                   =   "imageOptimize"; 
UCHAR _ux_pictbridge_xml_tag_line_layout[]                          =   "layout"; 
UCHAR _ux_pictbridge_xml_tag_line_fixedsize[]                       =   "fixedSize"; 
UCHAR _ux_pictbridge_xml_tag_line_cropping[]                        =   "cropping"; 
UCHAR _ux_pictbridge_xml_tag_line_jobconfig[]                       =   "jobConfig"; 
UCHAR _ux_pictbridge_xml_tag_line_printinfo[]                       =   "printInfo"; 
UCHAR _ux_pictbridge_xml_tag_line_fileid[]                          =   "fileID"; 
UCHAR _ux_pictbridge_xml_tag_line_filename[]                        =   "fileName"; 
UCHAR _ux_pictbridge_xml_tag_line_date[]                            =   "date"; 

/* Define Pictbridge XML variable names.  */
UCHAR _ux_pictbridge_xml_variable_papersize[]                       =   "paperSize"; 
UCHAR _ux_pictbridge_xml_variable_version[]                         =   "version"; 
UCHAR _ux_pictbridge_xml_variable_xmlns[]                           =   "xmlns"; 

/* Define Pictbridge XML string names.  */
UCHAR _ux_pictbridge_xml_string_xmlns[]                             =   "http://www.cipa.jp/dps/schema/"; 

