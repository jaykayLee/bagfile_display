#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include "gtrack.h"

//=============================================================================
#define C2V_OK  0
#define C2V_ERR_FIELD_NAME      -1
#define C2V_ERR_FIELD_SIZE      -2
#define C2V_ERR_VALUE           -3
#define C2V_ERR_PARAMETER       -4
#define C2V_ERR_FILE_OPEN       -5

//=============================================================================
typedef enum __tag_config_filed_type {
    CF_INVALID,
    // integer
    CF_U8,
    CF_I8,
    CF_U16,
    CF_I16,
    CF_U32,
    CF_I32,
    CF_U64,
    CF_I64,
    // float
    CF_FLT,
    CF_DBL,
    // byte, char
    CF_CHAR,
    // string
    CF_STR,
} config_filed_type;
char * config_field_name[]={
    "INVALID", "uint8", "int8", "uint16", "int16", "uint32", "int32", "uint64", "int64", "float", "double", "byte", "char", "str", NULL
};

typedef struct __tag_config2var {
    const char * name;
    config_filed_type  type;
    uint16_t           size;    // normal 0, array_type has value.
    uint16_t           opts;
    void * vptr;
} config2var;


#define C2V_TOKEN_MAX 100
typedef struct __tag_c2v_token {
    int count;
    char * tarr[C2V_TOKEN_MAX];
} c2v_tokens;

//=============================================================================
//=============================================================================
static GTRACK_sceneryParams l_SceneryParams = {
    {0.f,0.f,1.5f},                                                     /* sensor Position is (0,0,2) */
    {0.f, 0.f},                                                        /* Sensor orientation is (15 degrees down, 0 degrees azimuthal tilt */
    1,{{-30.0f,30.0f,0.5f,100.0f,-3.0f,8.0f}, {0.f,0.f,0.f,0.f,0.f,0.f}},     /* one boundary box {x1,x2,y1,y2,z1,z2} */
    1,{{-30.0f,30.0f,0.5f,100.0f,-3.0f,8.0f}, {0.f,0.f,0.f,0.f,0.f,0.f}}     /* one static box {x1,x2,y1,y2,z1,z2} */
};
static GTRACK_gatingParams l_GatingParams = {
    //3.f, {1.5f, 1.5f, 2.f, 0.f}    /* Gating Gain 8x, Limits are set to 2m in depth, width, 2m (if applicable) in height and no limits in doppler */
    2.f, {3.f, 2.f, 2.f, 0.f}
};
static GTRACK_stateParams l_StateParams = {
    //10U, 5U, 50U, 100U, 5U              /* det2act, det2free, act2free, stat2free, exit2free */
    3U, 3U, 10U, 40U, 5U, 1000U
};
static GTRACK_allocationParams l_AllocParams = {
    //0.f, 200.f, 0.1f, 6U, 1.5f, 2.f           /* 60 in clear, 200 obscured SNRs, 0.1m/s minimal velocity, 5 points, 1.5m in distance, 2m/s in velocity */
    100.f, 100.f, 0.5f, 5U, 1.f, 2.f
};
static GTRACK_presenceParams l_PresenceParams = {
    // 3U, 0.5f, 10U, 0,{{-3.0f,3.0f,2.0f,6.0f,0.5f,2.5f}, {0.f,0.f,0.f,0.f,0.f,0.f}}
    0, 0.0f, 0, 0, {{0.f,0.f,0.f,0.f, 0.f, 0.f},{0.f,0.f,0.f,0.f,0.f,0.f}}
};

GTRACK_advancedParameters l_adv_params = {
	.allocationParams = &l_AllocParams,
	.gatingParams = &l_GatingParams,
	.stateParams = &l_StateParams,
    .sceneryParams = &l_SceneryParams,
    .presenceParams = &l_PresenceParams
};
GTRACK_moduleConfig l_mod_config = {
	.stateVectorType = GTRACK_STATE_VECTORS_2DA,
	.verbose = GTRACK_VERBOSE_NONE,
	.deltaT = 0.05f, // 50ms frames
	.maxRadialVelocity = 5.29f, // Radial velocity from sensor is limited to +/- maxURV (in m/s)
	.radialVelocityResolution = 0.083f, // Radial velocity resolution (in m/s)
	.maxAcceleration[0] = 0.1f, // Target acceleration is not excedding 0.5 m/s2 in lateral direction
	.maxAcceleration[1] = 0.1f, // Target acceleration is not excedding 0.5 m/s2 in longitudinal direction
	.maxAcceleration[2] = 0.1f, // Target acceleration is not excedding 0.5 m/s2 in vertical direction
	.maxNumPoints = 1000,
	.maxNumTracks = 100,
	.initialRadialVelocity = 0, // Expected target radial velocity at the moment of detection, m/s
    .advParams = &l_adv_params
};

//=============================================================================
config2var g_modcfg_c2s[] = {
    // GTRACK_moduleConfig g_mod_config
    {"statVectorType"                             ,CF_I32   ,0    ,0    ,&l_mod_config.stateVectorType               },
    {"verbose"                                    ,CF_I32   ,0    ,0    ,&l_mod_config.verbose                       },
    {"maxNumPoints"                               ,CF_U16   ,0    ,0    ,&l_mod_config.maxNumPoints                  },
    {"maxNumTracks"                               ,CF_U16   ,0    ,0    ,&l_mod_config.maxNumTracks                  },
    {"initialRadialVelocity"                      ,CF_FLT   ,0    ,2    ,&l_mod_config.initialRadialVelocity         },
    {"maxRadialVelocity"                          ,CF_FLT   ,0    ,2    ,&l_mod_config.maxRadialVelocity             },
    {"radialVelocityResolution"                   ,CF_FLT   ,0    ,2    ,&l_mod_config.radialVelocityResolution      },
    {"maxAcceleration"                            ,CF_FLT   ,3    ,2    ,&l_mod_config.maxAcceleration               },
    {"deltaT"                                     ,CF_FLT   ,0    ,2    ,&l_mod_config.deltaT                        },
    // GTRACK_gatingParams advParams->gatingParams;
    {"gatingParams.gain"                          ,CF_FLT   ,0    ,3    ,&l_GatingParams.gain                        },
    {"gatingParams.limits.depth"                  ,CF_FLT   ,0    ,2    ,&l_GatingParams.gain                        },
    {"gatingParams.limits.width"                  ,CF_FLT   ,0    ,2    ,&l_GatingParams.gain                        },
    {"gatingParams.limits.height"                 ,CF_FLT   ,0    ,2    ,&l_GatingParams.gain                        },
    {"gatingParams.limits.vel"                    ,CF_FLT   ,0    ,2    ,&l_GatingParams.gain                        },
    // GTRACK_allocationParams *allocationParams
    {"allocParams.snrThre"                        ,CF_FLT   ,0    ,2    ,&l_AllocParams.snrThre                      },
    {"allocParams.snrThreObscured"                ,CF_FLT   ,0    ,2    ,&l_AllocParams.snrThreObscured              },
    {"allocParams.velocityThre"                   ,CF_FLT   ,0    ,2    ,&l_AllocParams.velocityThre                 },
    {"allocParams.pointsThre"                     ,CF_U16   ,0    ,2    ,&l_AllocParams.pointsThre                   },
    {"allocParams.maxDistanceThre"                ,CF_FLT   ,0    ,2    ,&l_AllocParams.maxDistanceThre              },
    {"allocParams.maxVelThre"                     ,CF_FLT   ,0    ,2    ,&l_AllocParams.maxVelThre                   },
    // GTRACK_stateParams *stateParams;
    {"stateParams.det2actThre"                    ,CF_U16   ,0    ,0    ,&l_StateParams.det2actThre                  },
    {"stateParams.det2freeThre"                   ,CF_U16   ,0    ,0    ,&l_StateParams.det2freeThre                 },
    {"stateParams.active2freeThre"                ,CF_U16   ,0    ,0    ,&l_StateParams.active2freeThre              },
    {"stateParams.static2freeThre"                ,CF_U16   ,0    ,0    ,&l_StateParams.static2freeThre              },
    {"stateParams.exit2freeThre"                  ,CF_U16   ,0    ,0    ,&l_StateParams.exit2freeThre                },
    {"stateParams.sleep2freeThre"                 ,CF_U16   ,0    ,0    ,&l_StateParams.sleep2freeThre               },
    // GTRACK_sceneryParams *sceneryParams;
    {"sceneryParams.sensorPosition"               ,CF_FLT   ,3    ,2    ,&l_SceneryParams.sensorPosition             },
    {"sceneryParams.sensorOrientation"            ,CF_FLT   ,2    ,2    ,&l_SceneryParams.sensorOrientation          },
    {"sceneryParams.numBoundaryBoxes"             ,CF_U8    ,1    ,0    ,&l_SceneryParams.numBoundaryBoxes           },
    {"sceneryParams.boundaryBox0"                 ,CF_FLT   ,6    ,2    ,&l_SceneryParams.boundaryBox[0]             },
    {"sceneryParams.boundaryBox1"                 ,CF_FLT   ,6    ,2    ,&l_SceneryParams.boundaryBox[1]             },
    {"sceneryParams.numStaticBoxes"               ,CF_U8    ,1    ,0    ,&l_SceneryParams.numStaticBoxes             },
    {"sceneryParams.staticBox0"                   ,CF_FLT   ,6    ,2    ,&l_SceneryParams.staticBox[0]               },
    {"sceneryParams.staticBox1"                   ,CF_FLT   ,6    ,2    ,&l_SceneryParams.staticBox[1]               },
    // GTRACK_presenceParams *presenceParams;
    {"presenceParams.pointsThre"                  ,CF_U16   ,0    ,0    ,&l_PresenceParams.pointsThre                },
    {"presenceParams.velocityThre"                ,CF_FLT   ,0    ,2    ,&l_PresenceParams.velocityThre              },
    {"presenceParams.on2offThre"                  ,CF_U16   ,0    ,0    ,&l_PresenceParams.on2offThre                },
    {"presenceParams.numOccupancyBoxes"           ,CF_U8    ,0    ,0    ,&l_PresenceParams.numOccupancyBoxes         },
    {"presenceParams.occupancyBox0"               ,CF_FLT   ,6    ,2    ,&l_PresenceParams.occupancyBox[0]           },
    {"presenceParams.occupancyBox1"               ,CF_FLT   ,6    ,2    ,&l_PresenceParams.occupancyBox[1]           },
    {NULL               ,CF_INVALID   ,0  ,0  ,NULL           },
};

//=============================================================================
char l_valstr[256];

//=============================================================================
char * get_config2var_value_str(config2var * field, char * outstr, int os_size)
{
    int i=0;
    int length = field->size == 0 ? 1 : field->size;
    char * pstr = outstr;
    switch(field->type) {
    case CF_INVALID : sprintf(outstr, "(INVALID)" );   break;
    case CF_U8 : {
        uint8_t * ptr = (uint8_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_I8 : {
        int8_t * ptr = (int8_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_U16 : {
        uint16_t * ptr = (uint16_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_I16 : {
        int16_t * ptr = (int16_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_U32 : {
        uint32_t * ptr = (uint32_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_I32 : {
        int32_t * ptr = (int32_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%d, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_U64 : {
        uint64_t * ptr = (uint64_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%lu, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_I64 : {
        int64_t * ptr = (int64_t *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%ld, ", ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_FLT : {
        float * ptr = (float *)field->vptr;
        char fmtstr[10];      sprintf(fmtstr, "%%.%uf, ", field->opts);
        for(i=0; i<length; i++)     pstr += sprintf(pstr, fmtstr, ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_DBL : {
        double * ptr = (double *)field->vptr;
        char fmtstr[10];      sprintf(fmtstr, "%%.%ug, ", field->opts);
        for(i=0; i<length; i++)     pstr += sprintf(pstr, fmtstr, ptr[i]);
        outstr[strlen(outstr)-2] = 0x00;
    } break;
    case CF_CHAR : {
        char * ptr = (char *)field->vptr;
        for(i=0; i<length; i++)     pstr += sprintf(pstr, "%c.", ptr[i]);
        outstr[strlen(outstr)-1] = 0x00;
    } break;
    case CF_STR : {
        strcpy(outstr, (char*)field->vptr );
    }    break;
    default : {
        sprintf(outstr, "(Unknown Type:%d)", field->type);
    } break;
    }
    return outstr;
}

//=============================================================================
int set_config2var_value_str(config2var * field, char * val_arr[])
{
    int i=0;
    int length = field->size == 0 ? 1 : field->size;
    switch(field->type) {
    case CF_U8 : {
        uint8_t * ptr = (uint8_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_I8 : {
        int8_t * ptr = (int8_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_U16 : {
        uint16_t * ptr = (uint16_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_I16 : {
        int16_t * ptr = (int16_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_U32 : {
        uint32_t * ptr = (uint32_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_I32 : {
        int32_t * ptr = (int32_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = atoi(val_arr[i]);
    } break;
    case CF_U64 : {
        uint64_t * ptr = (uint64_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = strtol(val_arr[i], NULL, 10);
    } break;
    case CF_I64 : {
        int64_t * ptr = (int64_t *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = strtol(val_arr[i], NULL, 10);
    } break;
    case CF_FLT : {
        float * ptr = (float *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = strtof(val_arr[i], NULL);
    } break;
    case CF_DBL : {
        double * ptr = (double *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = strtold(val_arr[i], NULL);
    } break;
    case CF_CHAR : {
        char * ptr = (char *)field->vptr;
        for(i=0; i<length; i++)     ptr[i] = val_arr[i][0];
    } break;
    case CF_STR : {
        strcpy((char *)field->vptr, val_arr[0]);
    }    break;
    default : {
        printf("(Unknown Type:%d)", field->type);
    } break;
    }
    return C2V_OK;
}

//=============================================================================
config2var * get_config2var_field_by_name(config2var * c2v, const char * field_name)
{
    int i=0;
    for(i=0; c2v[i].name!=NULL; i++) {
        if( !strcmp(field_name, c2v[i].name ))
            return &c2v[i];
    }
    return NULL;
}

//=============================================================================
int update_cmd_config2var_value(config2var * c2v, int argc, char * argv[])
{
    if( argc < 2)  // 0:name, 1:value
        return C2V_ERR_PARAMETER;
    config2var * field = get_config2var_field_by_name(c2v, argv[0]);
    if( field == NULL )
        return C2V_ERR_FIELD_NAME;
    if( field->size == 0 && argc == 2) {
        return set_config2var_value_str(field, &argv[1]);
    }
    else if( field->size == argc-1 ) {
        return set_config2var_value_str(field, &argv[1]);
    }
    else {
        return C2V_ERR_FIELD_SIZE;
    }
    return C2V_OK;
}

//=============================================================================
int read_cmd_config2var_field(config2var * c2v, char * field_name)
{
    config2var * field = get_config2var_field_by_name(c2v, field_name);
    if( field == NULL )
        return C2V_ERR_FIELD_NAME;
    char * valstr;
    valstr = get_config2var_value_str(field, l_valstr, sizeof(l_valstr));
    printf("=>%s(%s,%d) = [%s]\r\n",
            field->name, config_field_name[field->type], field->size, valstr );
    return C2V_OK;
}

//=============================================================================
void print_list_config2var(config2var * c2v, const char * name, const char * _prefix)
{
    int i=0;
    const char * prefix = (_prefix!=NULL) ? _prefix : "";
    char * valstr;
    printf("==== %s ==== \r\n", (name!=NULL ? name : "Unknown"));
    for(i=0; c2v[i].name!=NULL; i++) {
        valstr = get_config2var_value_str(&c2v[i], l_valstr, sizeof(l_valstr));
        printf("  %d %s(%s,%d) = [%s]\r\n",
                i, c2v[i].name, config_field_name[c2v[i].type], c2v[i].size, valstr );
    }
    printf("==== total : %d ==== \r\n", i);
}

//=============================================================================
char *ltrim(char *s)
{
    while(isspace(*s)) s++;
    return s;
}

char *rtrim(char *s)
{
    char* back = s + strlen(s);
    while(isspace(*--back));
    *(back+1) = '\0';
    return s;
}

char *trim(char *s)
{
    return rtrim(ltrim(s)); 
}

//=============================================================================
int parse_c2v_token(c2v_tokens * tok, char * line)
{
    char * trline = trim(line);
    char * savptr = NULL;
    //printf("    --trimed line=%s\r\n", trline);
    tok->count = 0;
    do {
        tok->tarr[tok->count] = strtok_r(trline, " \t", &savptr);
        if( tok->tarr[tok->count] == NULL )
            break;
        //printf("     token:%d=%s\r\n", tok->count, tok->tarr[tok->count]);
        tok->count ++;
        trline = 0x00;
    } while(1);
    return tok->count;
}

//=============================================================================
static c2v_tokens c2vtok;
int laodfile_config2var(config2var * c2v, const char * filepath)
{
    FILE * fp;
    char * line = NULL;
    size_t len = 0;
    int read = 0, ti = 0;

    fp = fopen(filepath, "r");
    if (fp == NULL)
        return C2V_ERR_FILE_OPEN;

    while ((read = getline(&line, &len, fp)) != -1) {
        printf("lines:%s", line);
        // in case of line first comment mark, skip all
        if(line[0] == '#') {
            printf("skipped line : [%s]\r\n", line);
            continue;
        }
        // in case of mid line, comment mark set as null. ignore after it.
        char * cmt_mark = strchr(line, '#');
        if(cmt_mark!=NULL) {
            cmt_mark[0] = 0x00;
            printf("skipped comment : [%s]\r\n", line);
        }
        // normal parse token.
        if( parse_c2v_token(&c2vtok, line) > 0 ) {
            if( !strcmp(c2vtok.tarr[0], "aat") && !strcmp(c2vtok.tarr[1], "cvupdate") ) {
                update_cmd_config2var_value(g_modcfg_c2s, c2vtok.count-2, &c2vtok.tarr[2] );
            }
        }
    }

    fclose(fp);
    if (line)
        free(line);
    return C2V_OK;
}




typedef struct __tag_c2v_set_item {
  char content_name[32];
  config2var     * c2v;
} c2v_set_item;


#define C2V_SET_MAX 5
static c2v_set_item l_c2v_set[C2V_SET_MAX];

//=============================================================================
int register_c2v_set_item(const char * content_name, config2var * arr)
{
  int i=0; 
  for(i=0; ((i<C2V_SET_MAX) && (l_c2v_set[i].content_name[0]!=0x00)); i++) {}
  printf("register index is %d\r\n", i);
  if( i >= C2V_SET_MAX )
    return -1;
  
  strcpy(l_c2v_set[i].content_name, content_name);
  l_c2v_set[i].c2v = arr;
  return 0;
}

//=============================================================================
static config2var * get_c2v_set_by_name(const char * name)
{
  int i=0; 
  for(i=0; ((i<C2V_SET_MAX) && (l_c2v_set[i].content_name[0]!=0x00)); i++) {
    if( !strcmp(l_c2v_set[i].content_name, name)) {
      return l_c2v_set[i].c2v;
    }
  }
  return NULL;
}


//=============================================================================
char * get_c2v_file_path(const char * file_name)
{
    const char * search_path[] = {
        "./",
        "/user/config/",
        NULL
    };
    char * full_path = l_valstr;
    for(int i=0; search_path[i]!=NULL; i++) {
        sprintf(full_path, "%s%s", search_path[i], file_name);
        if (access(full_path, F_OK) == 0) {
            return full_path;
        }
    }
    return NULL;
}
//=============================================================================
void main()
{
#if 0
     print_list_config2var(g_modcfg_c2s, "GTrackModuleCofig", NULL);
     read_cmd_config2var_field(g_modcfg_c2s, "stateParams.det2actThre");
     char * aa[] = {"stateParams.det2actThre", "999"};
     update_cmd_config2var_value(g_modcfg_c2s, 2, aa);
     read_cmd_config2var_field(g_modcfg_c2s, "stateParams.det2actThre");

     read_cmd_config2var_field(g_modcfg_c2s, "presenceParams.occupancyBox1");
     char * bb[] = {"presenceParams.occupancyBox1", "9.9", "8.9", "7.9", "6.9", "5.9", "4.9"};
     update_cmd_config2var_value(g_modcfg_c2s, 7, bb);
     read_cmd_config2var_field(g_modcfg_c2s, "presenceParams.occupancyBox1");

     printf("--------------------------------------\r\n");
     laodfile_config2var(g_modcfg_c2s, "/mnt/d/w1/srssim/sr_dev/srssim.gt/gtrack/c2v_test.txt");
     printf("--------------------------------------\r\n");
     print_list_config2var(g_modcfg_c2s, "GTrackModuleCofig", NULL);
#else
    char * path = get_c2v_file_path("test.c2v");
    if( path == 0x00 ) {
        printf("--file is not exist..\r\n");
    }
    else {
        printf("--file path is %s\r\n", path);
    }
#endif
}
