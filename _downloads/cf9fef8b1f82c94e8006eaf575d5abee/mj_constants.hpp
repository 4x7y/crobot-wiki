#pragma once

namespace crobot::simulation {

class MJConstants {
public:
  // clang-format off
  // const from defines
  static constexpr double MINVAL        = 1e-15;              // minimum value in any denominator
  static constexpr double PI            = 3.141592653589793;  // 
  static constexpr double MAXVAL        = 10000000000.0;      // maximum value in qpos, qvel, qacc
  static constexpr double MINMU         = 1e-05;              // minimum friction coefficient
  static constexpr double MINIMP        = 0.0001;             // minimum constraint impedance
  static constexpr double MAXIMP        = 0.9999;             // maximum constraint impedance
  static constexpr double MAXCONPAIR    = 50.0;               // maximum number of contacts per geom pair
  static constexpr double MAXVFS        = 200.0;              // maximum number of files in virtual file system
  static constexpr double MAXVFSNAME    = 100.0;              // maximum filename size in virtual file system
  static constexpr double NEQDATA       = 7.0;                // number of eq_data fields
  static constexpr double NDYN          = 10.0;               // number of actuator dynamics parameters
  static constexpr double NGAIN         = 10.0;               // number of actuator gain parameters
  static constexpr double NBIAS         = 10.0;               // number of actuator bias parameters
  static constexpr double NREF          = 2.0;                // number of solver reference parameters
  static constexpr double NIMP          = 5.0;                // number of solver impedance parameters
  static constexpr double NSOLVER       = 1000.0;             // size of mjData.solver_XXX arrays
  static constexpr double NGROUP        = 6.0;                // number of geom, site, joint groups with visflags
  static constexpr double MAXOVERLAY    = 500.0;              // maximum number of characters in overlay text
  static constexpr double MAXLINE       = 100.0;              // maximum number of lines per plot
  static constexpr double MAXLINEPNT    = 1000.0;             // maximum number points per line
  static constexpr double MAXPLANEGRID  = 200.0;              // maximum number of grid divisions for plane
  static constexpr double NAUX          = 10.0;               // number of auxiliary buffers
  static constexpr double MAXTEXTURE    = 1000.0;             // maximum number of textures
  static constexpr double MAXUISECT     = 10.0;               // maximum number of sections
  static constexpr double MAXUIITEM     = 80.0;               // maximum number of items per section
  static constexpr double MAXUITEXT     = 500.0;              // maximum number of chars in edittext and other
  static constexpr double MAXUINAME     = 40.0;               // maximum number of chars in name
  static constexpr double MAXUIMULTI    = 20.0;               // maximum number of radio/select items in group
  static constexpr double MAXUIEDIT     = 5.0;                // maximum number of elements in edit list
  static constexpr double MAXUIRECT     = 15.0;               // maximum number of rectangles
  static constexpr double KEY_ESCAPE    = 256.0;              // 
  static constexpr double KEY_ENTER     = 257.0;              // 
  static constexpr double KEY_TAB       = 258.0;              // 
  static constexpr double KEY_BACKSPACE = 259.0;              // 
  static constexpr double KEY_INSERT    = 260.0;              // 
  static constexpr double KEY_DELETE    = 261.0;              // 
  static constexpr double KEY_RIGHT     = 262.0;              // 
  static constexpr double KEY_LEFT      = 263.0;              // 
  static constexpr double KEY_DOWN      = 264.0;              // 
  static constexpr double KEY_UP        = 265.0;              // 
  static constexpr double KEY_PAGE_UP   = 266.0;              // 
  static constexpr double KEY_PAGE_DOWN = 267.0;              // 
  static constexpr double KEY_HOME      = 268.0;              // 
  static constexpr double KEY_END       = 269.0;              // 
  static constexpr double KEY_F1        = 290.0;              // 
  static constexpr double KEY_F2        = 291.0;              // 
  static constexpr double KEY_F3        = 292.0;              // 
  static constexpr double KEY_F4        = 293.0;              // 
  static constexpr double KEY_F5        = 294.0;              // 
  static constexpr double KEY_F6        = 295.0;              // 
  static constexpr double KEY_F7        = 296.0;              // 
  static constexpr double KEY_F8        = 297.0;              // 
  static constexpr double KEY_F9        = 298.0;              // 
  static constexpr double KEY_F10       = 299.0;              // 
  static constexpr double KEY_F11       = 300.0;              // 
  static constexpr double KEY_F12       = 301.0;              // 


  ///////////// const from enums ////////////////////////////////

  // mjtDisableBit
  static constexpr int DSBL_CONSTRAINT   = 1;
  static constexpr int DSBL_EQUALITY     = 2;
  static constexpr int DSBL_FRICTIONLOSS = 4;
  static constexpr int DSBL_LIMIT        = 8;
  static constexpr int DSBL_CONTACT      = 16;
  static constexpr int DSBL_PASSIVE      = 32;
  static constexpr int DSBL_GRAVITY      = 64;
  static constexpr int DSBL_CLAMPCTRL    = 128;
  static constexpr int DSBL_WARMSTART    = 256;
  static constexpr int DSBL_FILTERPARENT = 512;
  static constexpr int DSBL_ACTUATION    = 1024;
  static constexpr int DSBL_REFSAFE      = 2048;
  static constexpr int NDISABLE          = 12;

  // mjtEnableBit
  static constexpr int ENBL_OVERRIDE    = 1;
  static constexpr int ENBL_ENERGY      = 2;
  static constexpr int ENBL_FWDINV      = 4;
  static constexpr int ENBL_SENSORNOISE = 8;
  static constexpr int NENABLE          = 4;

  // mjtJoint
  static constexpr int JNT_FREE  = 0;
  static constexpr int JNT_BALL  = 1;
  static constexpr int JNT_SLIDE = 2;
  static constexpr int JNT_HINGE = 3;

  // mjtGeom
  static constexpr int GEOM_PLANE     = 0;
  static constexpr int GEOM_HFIELD    = 1;
  static constexpr int GEOM_SPHERE    = 2;
  static constexpr int GEOM_CAPSULE   = 3;
  static constexpr int GEOM_ELLIPSOID = 4;
  static constexpr int GEOM_CYLINDER  = 5;
  static constexpr int GEOM_BOX       = 6;
  static constexpr int GEOM_MESH      = 7;
  static constexpr int NGEOMTYPES     = 8;
  static constexpr int GEOM_ARROW     = 100;
  static constexpr int GEOM_ARROW1    = 101;
  static constexpr int GEOM_ARROW2    = 102;
  static constexpr int GEOM_LINE      = 103;
  static constexpr int GEOM_SKIN      = 104;
  static constexpr int GEOM_LABEL     = 105;
  static constexpr int GEOM_NONE      = 1001;

  // mjtCamLight
  static constexpr int CAMLIGHT_FIXED         = 0;
  static constexpr int CAMLIGHT_TRACK         = 1;
  static constexpr int CAMLIGHT_TRACKCOM      = 2;
  static constexpr int CAMLIGHT_TARGETBODY    = 3;
  static constexpr int CAMLIGHT_TARGETBODYCOM = 4;

  // mjtTexture
  static constexpr int TEXTURE_2D     = 0;
  static constexpr int TEXTURE_CUBE   = 1;
  static constexpr int TEXTURE_SKYBOX = 2;

  // mjtIntegrator
  static constexpr int INT_EULER = 0;
  static constexpr int INT_RK4   = 1;

  // mjtCollision
  static constexpr int COL_ALL     = 0;
  static constexpr int COL_PAIR    = 1;
  static constexpr int COL_DYNAMIC = 2;

  // mjtCone
  static constexpr int CONE_PYRAMIDAL = 0;
  static constexpr int CONE_ELLIPTIC  = 1;

  // mjtJacobian
  static constexpr int JAC_DENSE  = 0;
  static constexpr int JAC_SPARSE = 1;
  static constexpr int JAC_AUTO   = 2;

  // mjtSolver
  static constexpr int SOL_PGS    = 0;
  static constexpr int SOL_CG     = 1;
  static constexpr int SOL_NEWTON = 2;

  // mjtEq
  static constexpr int EQ_CONNECT  = 0;
  static constexpr int EQ_WELD     = 1;
  static constexpr int EQ_JOINT    = 2;
  static constexpr int EQ_TENDON   = 3;
  static constexpr int EQ_DISTANCE = 4;

  // mjtWrap
  static constexpr int WRAP_NONE     = 0;
  static constexpr int WRAP_JOINT    = 1;
  static constexpr int WRAP_PULLEY   = 2;
  static constexpr int WRAP_SITE     = 3;
  static constexpr int WRAP_SPHERE   = 4;
  static constexpr int WRAP_CYLINDER = 5;

  // mjtTrn
  static constexpr int TRN_JOINT = 0;
  static constexpr int TRN_JOINTINPARENT = 1;
  static constexpr int TRN_SLIDERCRANK = 2;
  static constexpr int TRN_TENDON = 3;
  static constexpr int TRN_SITE = 4;
  static constexpr int TRN_UNDEFINED = 1000;

  // mjtDyn
  static constexpr int DYN_NONE = 0;
  static constexpr int DYN_INTEGRATOR = 1;
  static constexpr int DYN_FILTER = 2;
  static constexpr int DYN_MUSCLE = 3;
  static constexpr int DYN_USER = 4;

  // mjtGain
  static constexpr int GAIN_FIXED = 0;
  static constexpr int GAIN_MUSCLE = 1;
  static constexpr int GAIN_USER = 2;

  // mjtBias
  static constexpr int BIAS_NONE   = 0;
  static constexpr int BIAS_AFFINE = 1;
  static constexpr int BIAS_MUSCLE = 2;
  static constexpr int BIAS_USER   = 3;

  // mjtObj
  static constexpr int OBJ_UNKNOWN  = 0;
  static constexpr int OBJ_BODY     = 1;
  static constexpr int OBJ_XBODY    = 2;
  static constexpr int OBJ_JOINT    = 3;
  static constexpr int OBJ_DOF      = 4;
  static constexpr int OBJ_GEOM     = 5;
  static constexpr int OBJ_SITE     = 6;
  static constexpr int OBJ_CAMERA   = 7;
  static constexpr int OBJ_LIGHT    = 8;
  static constexpr int OBJ_MESH     = 9;
  static constexpr int OBJ_SKIN     = 10;
  static constexpr int OBJ_HFIELD   = 11;
  static constexpr int OBJ_TEXTURE  = 12;
  static constexpr int OBJ_MATERIAL = 13;
  static constexpr int OBJ_PAIR     = 14;
  static constexpr int OBJ_EXCLUDE  = 15;
  static constexpr int OBJ_EQUALITY = 16;
  static constexpr int OBJ_TENDON   = 17;
  static constexpr int OBJ_ACTUATOR = 18;
  static constexpr int OBJ_SENSOR   = 19;
  static constexpr int OBJ_NUMERIC  = 20;
  static constexpr int OBJ_TEXT     = 21;
  static constexpr int OBJ_TUPLE    = 22;
  static constexpr int OBJ_KEY      = 23;

  // mjtConstraint
  static constexpr int CNSTR_EQUALITY             = 0;
  static constexpr int CNSTR_FRICTION_DOF         = 1;
  static constexpr int CNSTR_FRICTION_TENDON      = 2;
  static constexpr int CNSTR_LIMIT_JOINT          = 3;
  static constexpr int CNSTR_LIMIT_TENDON         = 4;
  static constexpr int CNSTR_CONTACT_FRICTIONLESS = 5;
  static constexpr int CNSTR_CONTACT_PYRAMIDAL    = 6;
  static constexpr int CNSTR_CONTACT_ELLIPTIC     = 7;

  // mjtConstraintState
  static constexpr int CNSTRSTATE_SATISFIED = 0;
  static constexpr int CNSTRSTATE_QUADRATIC = 1;
  static constexpr int CNSTRSTATE_LINEARNEG = 2;
  static constexpr int CNSTRSTATE_LINEARPOS = 3;
  static constexpr int CNSTRSTATE_CONE      = 4;

  // mjtSensor
  static constexpr int SENS_TOUCH          = 0;
  static constexpr int SENS_ACCELEROMETER  = 1;
  static constexpr int SENS_VELOCIMETER    = 2;
  static constexpr int SENS_GYRO           = 3;
  static constexpr int SENS_FORCE          = 4;
  static constexpr int SENS_TORQUE         = 5;
  static constexpr int SENS_MAGNETOMETER   = 6;
  static constexpr int SENS_RANGEFINDER    = 7;
  static constexpr int SENS_JOINTPOS       = 8;
  static constexpr int SENS_JOINTVEL       = 9;
  static constexpr int SENS_TENDONPOS      = 10;
  static constexpr int SENS_TENDONVEL      = 11;
  static constexpr int SENS_ACTUATORPOS    = 12;
  static constexpr int SENS_ACTUATORVEL    = 13;
  static constexpr int SENS_ACTUATORFRC    = 14;
  static constexpr int SENS_BALLQUAT       = 15;
  static constexpr int SENS_BALLANGVEL     = 16;
  static constexpr int SENS_JOINTLIMITPOS  = 17;
  static constexpr int SENS_JOINTLIMITVEL  = 18;
  static constexpr int SENS_JOINTLIMITFRC  = 19;
  static constexpr int SENS_TENDONLIMITPOS = 20;
  static constexpr int SENS_TENDONLIMITVEL = 21;
  static constexpr int SENS_TENDONLIMITFRC = 22;
  static constexpr int SENS_FRAMEPOS       = 23;
  static constexpr int SENS_FRAMEQUAT      = 24;
  static constexpr int SENS_FRAMEXAXIS     = 25;
  static constexpr int SENS_FRAMEYAXIS     = 26;
  static constexpr int SENS_FRAMEZAXIS     = 27;
  static constexpr int SENS_FRAMELINVEL    = 28;
  static constexpr int SENS_FRAMEANGVEL    = 29;
  static constexpr int SENS_FRAMELINACC    = 30;
  static constexpr int SENS_FRAMEANGACC    = 31;
  static constexpr int SENS_SUBTREECOM     = 32;
  static constexpr int SENS_SUBTREELINVEL  = 33;
  static constexpr int SENS_SUBTREEANGMOM  = 34;
  static constexpr int SENS_USER           = 35;

  // mjtStage
  static constexpr int STAGE_NONE = 0;
  static constexpr int STAGE_POS  = 1;
  static constexpr int STAGE_VEL  = 2;
  static constexpr int STAGE_ACC  = 3;

  // mjtDataType
  static constexpr int DATATYPE_REAL       = 0;
  static constexpr int DATATYPE_POSITIVE   = 1;
  static constexpr int DATATYPE_AXIS       = 2;
  static constexpr int DATATYPE_QUATERNION = 3;

  // mjtLRMode
  static constexpr int LRMODE_NONE       = 0;
  static constexpr int LRMODE_MUSCLE     = 1;
  static constexpr int LRMODE_MUSCLEUSER = 2;
  static constexpr int LRMODE_ALL        = 3;

  // mjtWarning
  static constexpr int WARN_INERTIA     = 0;
  static constexpr int WARN_CONTACTFULL = 1;
  static constexpr int WARN_CNSTRFULL   = 2;
  static constexpr int WARN_VGEOMFULL   = 3;
  static constexpr int WARN_BADQPOS     = 4;
  static constexpr int WARN_BADQVEL     = 5;
  static constexpr int WARN_BADQACC     = 6;
  static constexpr int WARN_BADCTRL     = 7;
  static constexpr int NWARNING         = 8;

  // mjtTimer
  static constexpr int TIMER_STEP           = 0;
  static constexpr int TIMER_FORWARD        = 1;
  static constexpr int TIMER_INVERSE        = 2;
  static constexpr int TIMER_POSITION       = 3;
  static constexpr int TIMER_VELOCITY       = 4;
  static constexpr int TIMER_ACTUATION      = 5;
  static constexpr int TIMER_ACCELERATION   = 6;
  static constexpr int TIMER_CONSTRAINT     = 7;
  static constexpr int TIMER_POS_KINEMATICS = 8;
  static constexpr int TIMER_POS_INERTIA    = 9;
  static constexpr int TIMER_POS_COLLISION  = 10;
  static constexpr int TIMER_POS_MAKE       = 11;
  static constexpr int TIMER_POS_PROJECT    = 12;
  static constexpr int NTIMER               = 13;

  // mjtCatBit
  static constexpr int CAT_STATIC  = 1;
  static constexpr int CAT_DYNAMIC = 2;
  static constexpr int CAT_DECOR   = 4;
  static constexpr int CAT_ALL     = 7;

  // mjtMouse
  static constexpr int MOUSE_NONE     = 0;
  static constexpr int MOUSE_ROTATE_V = 1;
  static constexpr int MOUSE_ROTATE_H = 2;
  static constexpr int MOUSE_MOVE_V   = 3;
  static constexpr int MOUSE_MOVE_H   = 4;
  static constexpr int MOUSE_ZOOM     = 5;
  static constexpr int MOUSE_SELECT   = 6;

  // mjtPertBit
  static constexpr int PERT_TRANSLATE = 1;
  static constexpr int PERT_ROTATE    = 2;

  // mjtCamera
  static constexpr int CAMERA_FREE     = 0;
  static constexpr int CAMERA_TRACKING = 1;
  static constexpr int CAMERA_FIXED    = 2;
  static constexpr int CAMERA_USER     = 3;

  // mjtLabel
  static constexpr int LABEL_NONE         = 0;
  static constexpr int LABEL_BODY         = 1;
  static constexpr int LABEL_JOINT        = 2;
  static constexpr int LABEL_GEOM         = 3;
  static constexpr int LABEL_SITE         = 4;
  static constexpr int LABEL_CAMERA       = 5;
  static constexpr int LABEL_LIGHT        = 6;
  static constexpr int LABEL_TENDON       = 7;
  static constexpr int LABEL_ACTUATOR     = 8;
  static constexpr int LABEL_CONSTRAINT   = 9;
  static constexpr int LABEL_SKIN         = 10;
  static constexpr int LABEL_SELECTION    = 11;
  static constexpr int LABEL_SELPNT       = 12;
  static constexpr int LABEL_CONTACTFORCE = 13;
  static constexpr int NLABEL             = 14;

  // mjtFrame
  static constexpr int FRAME_NONE   = 0;
  static constexpr int FRAME_BODY   = 1;
  static constexpr int FRAME_GEOM   = 2;
  static constexpr int FRAME_SITE   = 3;
  static constexpr int FRAME_CAMERA = 4;
  static constexpr int FRAME_LIGHT  = 5;
  static constexpr int FRAME_WORLD  = 6;
  static constexpr int NFRAME       = 7;

  // mjtVisFlag
  static constexpr int VIS_CONVEXHULL   = 0;
  static constexpr int VIS_TEXTURE      = 1;
  static constexpr int VIS_JOINT        = 2;
  static constexpr int VIS_ACTUATOR     = 3;
  static constexpr int VIS_CAMERA       = 4;
  static constexpr int VIS_LIGHT        = 5;
  static constexpr int VIS_TENDON       = 6;
  static constexpr int VIS_RANGEFINDER  = 7;
  static constexpr int VIS_CONSTRAINT   = 8;
  static constexpr int VIS_INERTIA      = 9;
  static constexpr int VIS_SCLINERTIA   = 10;
  static constexpr int VIS_PERTFORCE    = 11;
  static constexpr int VIS_PERTOBJ      = 12;
  static constexpr int VIS_CONTACTPOINT = 13;
  static constexpr int VIS_CONTACTFORCE = 14;
  static constexpr int VIS_CONTACTSPLIT = 15;
  static constexpr int VIS_TRANSPARENT  = 16;
  static constexpr int VIS_AUTOCONNECT  = 17;
  static constexpr int VIS_COM          = 18;
  static constexpr int VIS_SELECT       = 19;
  static constexpr int VIS_STATIC       = 20;
  static constexpr int VIS_SKIN         = 21;
  static constexpr int NVISFLAG         = 22;

  // mjtRndFlag
  static constexpr int RND_SHADOW     = 0;
  static constexpr int RND_WIREFRAME  = 1;
  static constexpr int RND_REFLECTION = 2;
  static constexpr int RND_ADDITIVE   = 3;
  static constexpr int RND_SKYBOX     = 4;
  static constexpr int RND_FOG        = 5;
  static constexpr int RND_HAZE       = 6;
  static constexpr int RND_SEGMENT    = 7;
  static constexpr int RND_IDCOLOR    = 8;
  static constexpr int NRNDFLAG       = 9;

  // mjtStereo
  static constexpr int STEREO_NONE         = 0;
  static constexpr int STEREO_QUADBUFFERED = 1;
  static constexpr int STEREO_SIDEBYSIDE   = 2;

  // mjtGridPos
  static constexpr int GRID_TOPLEFT     = 0;
  static constexpr int GRID_TOPRIGHT    = 1;
  static constexpr int GRID_BOTTOMLEFT  = 2;
  static constexpr int GRID_BOTTOMRIGHT = 3;

  // mjtFramebuffer
  static constexpr int FB_WINDOW    = 0;
  static constexpr int FB_OFFSCREEN = 1;

  // mjtFontScale
  static constexpr int FONTSCALE_50  = 50;
  static constexpr int FONTSCALE_100 = 100;
  static constexpr int FONTSCALE_150 = 150;
  static constexpr int FONTSCALE_200 = 200;
  static constexpr int FONTSCALE_250 = 250;
  static constexpr int FONTSCALE_300 = 300;

  // mjtFont
  static constexpr int FONT_NORMAL = 0;
  static constexpr int FONT_SHADOW = 1;
  static constexpr int FONT_BIG    = 2;

  // mjtButton
  static constexpr int BUTTON_NONE   = 0;
  static constexpr int BUTTON_LEFT   = 1;
  static constexpr int BUTTON_RIGHT  = 2;
  static constexpr int BUTTON_MIDDLE = 3;

  // mjtEvent
  static constexpr int EVENT_NONE    = 0;
  static constexpr int EVENT_MOVE    = 1;
  static constexpr int EVENT_PRESS   = 2;
  static constexpr int EVENT_RELEASE = 3;
  static constexpr int EVENT_SCROLL  = 4;
  static constexpr int EVENT_KEY     = 5;
  static constexpr int EVENT_RESIZE  = 6;

  // mjtItem
  static constexpr int ITEM_END       = -2;
  static constexpr int ITEM_SECTION   = -1;
  static constexpr int ITEM_SEPARATOR = 0;
  static constexpr int ITEM_STATIC    = 1;
  static constexpr int ITEM_BUTTON    = 2;
  static constexpr int ITEM_CHECKINT  = 3;
  static constexpr int ITEM_CHECKBYTE = 4;
  static constexpr int ITEM_RADIO     = 5;
  static constexpr int ITEM_SELECT    = 6;
  static constexpr int ITEM_SLIDERINT = 7;
  static constexpr int ITEM_SLIDERNUM = 8;
  static constexpr int ITEM_EDITINT   = 9;
  static constexpr int ITEM_EDITNUM   = 10;
  static constexpr int ITEM_EDITTXT   = 11;
  static constexpr int NITEM          = 12;

  // clang-foramt on
};

} // namespace crobot

