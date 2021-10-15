#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "sensors.h"
#include "estimator_kalman.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pm.h"
#include "stabilizer.h"
#include "ledseq.h"
#include "log.h"
#include "param.h"
#include "supervisor.h"
#include "controller.h"
#include "ledseq.h"
#include "pptraj.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"
#include "pilot.h"
#include "radiolink.h"
#include "protocol.h"

#define DEBUG_MODULE "PILOT"
#include "debug.h"



#define LED_LOCK         LED_GREEN_R
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
#define MAX_PAD_ERR 0.005
#define TAKE_OFF_HEIGHT 0.2f
#define TAKE_OFF_TIME 1.0f
#define LANDING_HEIGHT 0.12f
#define SEQUENCE_SPEED 1.0f
#define DURATION_TO_INITIAL_POSITION 2.0f

static uint32_t positionLockWriteIndex;
static float positionLockData[LOCK_LENGTH][3];
static void resetPositionLockData();
static bool hasPositionLock();

static uint32_t takeOffTime = 0;
static bool terminateTrajectoryAndLand = false;

static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t landingTimeCheckCharge = 0;

static float stabilizeEndTime;

#define NO_PROGRESS -2000.0f
static uint32_t trajectoryStartTime = 0;
static float trajectoryDurationMs = 0.0f;

static float trajecory_center_offset_x = 0.0f;
static float trajecory_center_offset_y = 0.0f;
static float trajecory_center_offset_z = 0.0f;

static uint32_t now = 0;
static uint32_t flightTime = 0;

// The nr of trajectories to fly
static uint8_t trajectoryCount = 2;
static uint8_t remainingTrajectories = 0;

// Set to 1 to be active in the swarm. If 0 the CF is still
// part of the decision network.
static uint8_t isActive = 0;
static uint8_t allActivate = 0;
static uint8_t allDeactivate = 0;
static bool hasAutoActivationTokenBeenUsed = false;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;
static logVarId_t logIdPmState;
static logVarId_t logIdPmVbat;
static logVarId_t logIdlighthouseEstBs0Rt;
static logVarId_t logIdlighthouseEstBs1Rt;

static paramVarId_t paramIdStabilizerController;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdLighthouseMethod;

//#define USE_MELLINGER

#define TRAJ_Y_OFFSET 0.35

enum State {
  // Initialization
  STATE_IDLE = 0,
  STATE_WAIT_FOR_POSITION_LOCK,

  STATE_WAIT_FOR_TAKE_OFF, // Charging
  STATE_TAKING_OFF,
  STATE_GOING_TO_INITIAL_POSITION,
  STATE_RUNNING_TRAJECTORY,
  STATE_GOING_TO_PAD,
  STATE_WAITING_AT_PAD,
  STATE_LANDING,
  STATE_CHECK_CHARGING,
  STATE_REPOSITION_ON_PAD,
  STATE_CRASHED,
};

static enum State state = STATE_IDLE;

ledseqStep_t seq_lock_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lock = {
  .sequence = seq_lock_def,
  .led = LED_LOCK,
};

const uint8_t trajectoryId = 1;

// duration, x0-x7, y0-y7, z0-z7, yaw0-yaw7
static struct poly4d sequence[] = {
  {.duration = 0.55, .p = { { 6.416979287336798e-11,-1.021113333771827e-06,0.05668589180427593,-0.0006550193702573347,-0.22695020122851842,-0.02245927705326242,0.21754862988665644,-0.0763163386851738 }, { 8.550551924012914e-11,-1.1491158055034283e-06,4.6227444146407156e-05,0.16108370241891062,0.005097163411800295,-0.2432950493424784,0.04316441972717085,0.05302700826095468 }, { 0.7935094937573058,0.3354737430678515,0.029057160591356115,-0.01266832414604316,-0.0005486333712972439,0.00014350768837690457,4.164379035236781e-06,-8.017606108453261e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { -1.1524647503337546e-10,-0.04865618192925648,-0.17599159103471598,-0.08911810954824294,0.23831074833627608,0.21924519941567547,-0.17627779094541046,-0.008157582758662189 }, { 0.01703708687483856,0.06159837907420384,-0.01475204867210726,-0.2538324733735039,-0.1745636488986106,0.16891292427241564,0.17505136687817116,-0.10478532461757861 }, { 0.9846592732952834,0.3556417614613807,0.007402285626974173,-0.013429918473541944,-0.0001397644145837933,0.0001521442105186959,1.0560984562962911e-06,-8.232913171177211e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { -0.0669872980743698,-0.11900005261511251,0.22415517756080236,0.48964784920231713,0.01631532795277162,-0.3508924809373917,-0.08978934199896278,0.11940103114016244 }, { -1.314930500635324e-10,-0.19131336231121068,-0.3399403977601118,0.12117136721085453,0.465809948774897,0.12498117985305485,-0.2945904370526364,0.04512690036505512 }, { 1.1802619153775327,0.3515733815370541,-0.014757042867951776,-0.013276286076733262,0.00027862940442694546,0.00015041191903359245,-2.123518122387068e-06,-7.890778247293665e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { 1.331380762379717e-10,0.4182485409391594,0.48071980738031506,-0.45545384753809937,-0.6618967433481032,0.03307307216986751,0.3900395467726873,-0.10319519166476154 }, { -0.146446609316142,-0.16829217031835103,0.5572530176835692,0.6920307186924336,-0.23701040003071772,-0.5110777869493257,0.0324269787310151,0.11916742197106137 }, { 1.366987423194795,0.3235458566636087,-0.03591070328290674,-0.012217896676011442,0.0006780348284016298,0.0001384300512311638,-5.159489510055758e-06,-7.004856932156779e-07 }, { -7.032049652736305e-19,2.8559933214452258,1.0578188921989469e-12,-1.0730494111436881e-11,5.356676493007769e-11,-1.41495675688421e-10,1.8919400721208388e-10,-1.0077661393087099e-10 } } },
  {.duration = 0.55, .p = { { 0.24999999985174912,0.2061155558214833,-0.9913455017052015,-0.8471890237515584,0.5681498053748525,0.6385524783191768,-0.18326875490691807,-0.10410041724314335 }, { 1.200695274301893e-10,0.7139964604177336,0.5887359357783304,-0.8909485328222915,-0.8132081410853154,0.24414642067458012,0.4561204208567855,-0.16209003295293947 }, { 1.5321107619678176,0.27346921634328714,-0.05461710861250694,-0.010326877829134709,0.0010312334197305668,0.00011701404859755804,-7.843326517779124e-06,-5.644608348066262e-07 }, { 1.5707963267948966,2.855993321445235,9.74588081275978e-13,-1.2504624207862248e-11,7.528531028453892e-11,-2.288999400804534e-10,3.4113546098087626e-10,-1.985970861280439e-10 } } },
  {.duration = 0.55, .p = { { -9.317788348400608e-11,-1.0584023887810166,-0.656627662309007,1.3979771799536898,0.9094325202544633,-0.493854565661028,-0.48832975714007343,0.21779783825613533 }, { 0.3705904772462592,0.2298926079086439,-1.4968499442134247,-0.9445489822823906,0.9545362847576447,0.724629365677623,-0.3524563689735574,-0.07522680833189324 }, { 1.6643790490384371,0.20475610085841317,-0.06960144824867719,-0.007732099331413265,0.001314155199499109,8.762361080722874e-05,-9.992480186630427e-06,-3.9006659712423786e-07 }, { -3.1415926535897936,2.8559933214452955,-1.1721404849972875e-12,1.8193195687576433e-11,-1.214954249299105e-10,3.92209327554307e-10,-6.08493662478147e-10,3.6496637176493247e-10 } } },
  {.duration = 0.55, .p = { { -0.4999999997504216,-0.23800295977348337,2.039317052835777,0.9774756740141367,-1.3698382381677419,-0.7634424513736799,0.5284599646406029,0.03451428393852941 }, { -5.429567520858986e-11,-1.4279956311687598,-0.6797682780053409,1.941986624527379,0.9440123484931213,-0.7651803099318497,-0.484472542198384,0.26652221244988 }, { 1.7547784192264733,0.12208919547549385,-0.079842564209313,-0.0046103910307376175,0.0015075194045401598,5.226198127020738e-05,-1.1460982235188534e-05,-1.889016440196309e-07 }, { -1.570796326794896,2.8559933214452906,-6.777721171174742e-13,8.41571999341113e-12,-5.1447001812870034e-11,1.601657193261176e-10,-2.4527134121419665e-10,1.4685418151406883e-10 } } },
  {.duration = 0.55, .p = { { 6.0728320509926996e-12,1.7975890188938861,0.6565807881517729,-2.485903521947362,-0.9145910674387289,1.0396332517101365,0.44481163995455636,-0.3049426706338622 }, { -0.6294095222649269,-0.22989390433931967,2.5817785905862545,0.9437251993510384,-1.7857535239814637,-0.7523466872163672,0.6992851869171793,-0.015262664208531092 }, { 1.7971483048456303,0.03110211318278792,-0.08464254136278623,-0.0011744922270002193,0.0015981487959421089,1.3338387806731703e-05,-1.2147884701613623e-05,2.4831049961939595e-08 }, { -7.032049652736305e-19,2.8559933214452258,1.0578188921989469e-12,-1.0730494111436881e-11,5.356676493007769e-11,-1.41495675688421e-10,1.8919400721208388e-10,-1.0077661393087099e-10 } } },
  {.duration = 0.55, .p = { { 0.7499999996897563,0.20611806033328717,-3.0872667001317295,-0.8455975973136185,2.1739382025069816,0.6920982320653135,-0.8532905804805064,0.07071182008899793 }, { -4.82040873963648e-11,2.141995373364969,0.5886453818591884,-2.9926608344731864,-0.8231736890624758,1.2985098774777937,0.37204987394265615,-0.33044092122041274 }, { 1.7886012682105124,-0.062004526724627494,-0.08367426920180858,0.002341446292907904,0.001579866923823775,-2.6494018132335393e-05,-1.20071691427135e-05,2.3700017024090774e-07 }, { 1.5707963267948966,2.855993321445235,9.74588081275978e-13,-1.2504624207862248e-11,7.528531028453892e-11,-2.288999400804534e-10,3.4113546098087626e-10,-1.985970861280439e-10 } } },
  {.duration = 0.55, .p = { { 1.0483682940715062e-10,-2.4377439706831407,-0.4805917448007315,3.4277238887496053,0.6759901563992652,-1.5241681722998246,-0.2711458393439101,0.3412793012091927 }, { 0.8535533902735974,0.16829571223289608,-3.5213332021628974,-0.689780101850695,2.5079381296188608,0.5868029179161517,-0.9799809312649118,0.1280544147721823 }, { 1.729719775743155,-0.15088566060301098,-0.07700373387507774,0.005697819139121337,0.0014539196184011809,-6.452058446859345e-05,-1.104863358878885e-05,4.3327149958112416e-07 }, { -3.1415926535897936,2.8559933214452955,-1.1721404849972875e-12,1.8193195687576433e-11,-1.214954249299105e-10,3.92209327554307e-10,-6.08493662478147e-10,3.6496637176493247e-10 } } },
  {.duration = 0.55, .p = { { -0.9330127015782295,-0.11900439055672117,3.8543971818934915,0.48689141778688627,-2.764991762258722,-0.4436364464607404,1.0707225013735517,-0.18338264531164744 }, { 1.599656008709793e-10,-2.664680032688923,-0.33978355377225944,3.7614438565736177,0.4830707841356907,-1.7012298967973436,-0.14897597863335033,0.3367191924899483 }, { 1.6245165038489418,-0.229484186061661,-0.0650855213384239,0.008665895012045418,0.0012288901901355187,-9.815048566881373e-05,-9.336680512287419e-06,5.997355942281935e-07 }, { -1.570796326794896,2.8559933214452906,-6.777721171174742e-13,8.41571999341113e-12,-5.1447001812870034e-11,1.601657193261176e-10,-2.4527134121419665e-10,1.4685418151406883e-10 } } },
  {.duration = 0.55, .p = { { -2.0983318998797417e-10,2.807338241786142,0.1758166542968206,-3.971078273644825,-0.25756270863916153,1.8176285869613595,0.013865966039783711,-0.31707135899458033 }, { -0.9829629128509617,-0.06160321741937426,4.063760879522907,0.250758073613977,-2.9275813199792107,-0.2723553766100829,1.119331403382848,-0.2329259845546412 }, { 1.4801608816508365,-0.2924437434807816,-0.04873183808083333,0.011043404457887842,0.0009201139233646774,-0.0001250915420548524,-6.9885668510594605e-06,7.254141315649441e-07 }, { -7.032049652736305e-19,2.8559933214452258,1.0578188921989469e-12,-1.0730494111436881e-11,5.356676493007769e-11,-1.41495675688421e-10,1.8919400721208388e-10,-1.0077661393087099e-10 } } },
  {.duration = 0.55, .p = { { 0.9999999997401849,3.987910233544425e-06,-4.135156505047333,0.002527833487451032,2.9846265931409093,0.08463223012997333,-1.1224950208116367,0.27330813572300433 }, { -2.5104201471530493e-10,2.855996676778625,-0.00013488039286297813,-4.04234090089733,-0.014833932464075071,1.865431864043328,-0.12497667379140892,-0.2836747684453799 }, { 1.3064905062426948,-0.33547374306785216,-0.029057160591243466,0.012668324142701075,0.0005486333961726315,-0.00014350776673085613,-4.164265775564083e-06,8.016978543870155e-07 }, { 1.5707963267948966,2.855993321445235,9.74588081275978e-13,-1.2504624207862248e-11,7.528531028453892e-11,-2.288999400804534e-10,3.4113546098087626e-10,-1.985970861280439e-10 } } },
  {.duration = 0.55, .p = { { 2.807828459840791e-10,-2.807339345733594,0.17608024398424846,3.970375308017605,-0.22857397923283293,-1.8413820142702892,0.2580900452427899,0.23880534280327456 }, { 0.9829629129295165,-0.061595412277325706,-4.063718564569964,0.2557052874787339,2.9322400408752176,-0.10673997136964312,-1.0799977575702353,0.301777121532787 }, { 1.1153407267047166,-0.35564176146143944,-0.0074022856248120855,0.01342991844715964,0.00013976456430208812,-0.0001521446417983823,-1.0554850265050618e-06,8.22948076696411e-07 }, { -3.1415926535897936,2.8559933214452955,-1.1721404849972875e-12,1.8193195687576433e-11,-1.214954249299105e-10,3.92209327554307e-10,-6.08493662478147e-10,3.6496637176493247e-10 } } },
  {.duration = 0.55, .p = { { -0.933012701729985,0.11899708581819672,3.854315435682374,-0.49152066331913125,-2.773991719872727,0.2887195278924881,0.9947357328685774,-0.3163928281427994 }, { 2.970290996548403e-10,-2.6646821653516586,0.3400290507104985,3.7600858312479666,-0.4560731796127023,-1.7471179948742097,0.3764026915838674,0.18552085955051698 }, { 0.9197380846224678,-0.35157338153719797,0.014757042872100732,0.013276286030137944,-0.000278629148463537,-0.00015041264917281083,2.124556409941004e-06,7.884956572283471e-07 }, { -1.570796326794896,2.8559933214452906,-6.777721171174742e-13,8.41571999341113e-12,-5.1447001812870034e-11,1.601657193261176e-10,-2.4527134121419665e-10,1.4685418151406883e-10 } } },
  {.duration = 0.55, .p = { { -2.986745047859392e-10,2.43774698672368,-0.4808084603294431,-3.425803350937227,0.6521599742855927,1.589063742542715,-0.4718518008286481,-0.12745256853880602 }, { -0.8535533904882134,0.16828920352145518,3.5212175955590097,-0.6939035328016641,-2.5206659919333774,0.4489048340344392,0.8725194119489996,-0.3161592188649125 }, { 0.7330125768052057,-0.3235458566636719,0.03591070328497667,0.01221789665061662,-0.0006780346842335766,-0.0001384304661573386,5.160079198788061e-06,7.001567151636996e-07 }, { -7.032049652736305e-19,2.8559933214452258,1.0578188921989469e-12,-1.0730494111436881e-11,5.356676493007769e-11,-1.41495675688421e-10,1.8919400721208388e-10,-1.0077661393087099e-10 } } },
  {.duration = 0.55, .p = { { 0.7499999999526065,-0.20611258902466634,-3.087125111535636,0.8490618378459894,2.189526586652342,-0.5763795255488048,-0.721677635603617,0.30109221405737663 }, { -2.8560587656687233e-10,2.1419990672451292,-0.5888245887284619,-2.9903086656386977,0.8034713719302147,1.377990394338766,-0.5379326753931835,-0.0685577269507824 }, { 0.5678892380321828,-0.2734692163433205,0.054617108613167466,0.01032687782254788,-0.001031233384309226,-0.00011701414975196907,7.843470800293175e-06,5.643796521572908e-07 }, { 1.5707963267948966,2.855993321445235,9.74588081275978e-13,-1.2504624207862248e-11,7.528531028453892e-11,-2.288999400804534e-10,3.4113546098087626e-10,-1.985970861280439e-10 } } },
  {.duration = 0.55, .p = { { 2.587142363462102e-10,-1.797593138881872,0.6567163152597161,2.4832800185027444,-0.8996957510833731,-1.1282822493754645,0.5701420116796424,0.012849921661922037 }, { 0.6294095225580959,-0.2298896411116641,-2.5816206690315875,0.9464217964189122,1.8031401070571982,-0.6624564123375953,-0.5524900223124466,0.27221860556703376 }, { 0.4356209509615622,-0.20475610085841497,0.06960144824921265,0.007732099322288083,-0.001314155138706674,-8.762380365769796e-05,9.992773242807845e-06,3.8989422933523954e-07 }, { -3.1415926535897936,2.8559933214452955,-1.1721404849972875e-12,1.8193195687576433e-11,-1.214954249299105e-10,3.92209327554307e-10,-6.08493662478147e-10,3.6496637176493247e-10 } } },
  {.duration = 0.55, .p = { { -0.5000000000539335,0.23799999297660931,2.039153560407053,-0.9793484881319394,-1.3878381537312077,0.7012694982403683,0.37648642638290836,-0.2315060810391424 }, { 2.1983208536314492e-10,-1.427999896494089,0.6798569309550538,1.9392705739389484,-0.9342755793719016,-0.8569565049701163,0.5662847965530963,-0.03587445243013377 }, { 0.3452215807735272,-0.12208919547551644,0.07984256421012907,0.00461039101898122,-0.0015075193290159292,-5.226221925459312e-05,1.1461344984408984e-05,1.8868715502750348e-07 }, { -1.570796326794896,2.8559933214452906,-6.777721171174742e-13,8.41571999341113e-12,-5.1447001812870034e-11,1.601657193261176e-10,-2.4527134121419665e-10,1.4685418151406883e-10 } } },
  {.duration = 0.55, .p = { { -1.7160929272601208e-10,1.058406508768995,-0.656669441102361,-1.39535367650893,0.9048542982592898,0.5825035633699409,-0.5266238945824033,0.07429491077908489 }, { -0.3705904775394288,0.22989093754244339,1.4966920226560663,-0.9455980134608333,-0.9719228679651201,0.6901737342209295,0.20566120391153467,-0.1817291327845235 }, { 0.3028516951543695,-0.031102113182857404,0.08464254136521183,0.0011744921974183406,-0.0015981486270726741,-1.3338878889514318e-05,1.2148590693744128e-05,-2.5229797179643945e-08 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { 0.25000000011459755,-0.20611509353629254,-0.9912039131140691,0.8474704114616869,0.5837381892373433,-0.6299252785134271,-0.05165581111733545,0.126279977326233 }, { -1.1733212493234335e-10,0.714000154297925,-0.588734034810282,-0.8885963639776358,0.813436919856226,0.3236269376698719,-0.45386212865652226,0.09979316140963199 }, { 0.31139873178948824,0.062004526724516604,0.08367426920527306,-0.002341446333445613,-0.0015798666958438464,2.6493355838210824e-05,1.2008126984933222e-05,-2.375458559854139e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { 6.069950431033365e-11,-0.41825155697966915,0.48068039774921895,0.45353330973119566,-0.6662533873605433,-0.0979686423575192,0.35295809333552086,-0.1106315409765709 }, { 0.14644660953075683,-0.16829274543601136,-0.5571374110794685,0.6916529159590967,0.24973826234372903,-0.5246299649847722,0.07503454054895237,0.06893738214691839 }, { 0.37028022425684576,0.15088566060297004,0.07700373387638232,-0.005697819155841523,-0.001453919519303095,6.452029033312363e-05,1.104906078228004e-05,-4.3351409678770295e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { -0.06698729822612587,0.11900142375990946,0.2240734313470938,-0.48876423187736406,0.0073153702057580025,0.38146349377137573,-0.1657761109856045,-0.013609151430896072 }, { 5.570835602079473e-12,-0.1913154949739155,0.339872206721429,0.11981334190008967,-0.473334015059123,0.0790930820307133,0.23078823278737132,-0.10607143235430593 }, { 0.4754834961510585,0.22948418606164786,0.06508552133959486,-0.008665895033115,-0.0012288900441894599,9.815000989805213e-05,9.337418549554038e-06,-6.001758332093377e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
  {.duration = 0.55, .p = { { 4.4296985200832773e-11,0.04865728587670713,-0.17590530724645048,0.08982107517685357,0.24782593952847343,-0.19549177208942675,-0.0956782203552369,0.08642359895583152 }, { -0.0170370869533932,0.06160025062251494,0.014709733718925573,-0.25263088771828734,0.16990492800195506,0.21018242370394344,-0.21438501268388413,0.03593418763593158 }, { 0.6198391183491642,0.2924437434807964,0.04873183808147461,-0.011043404473242202,-0.0009201138107688026,0.00012509116676211315,6.989157399975058e-06,-7.257708130434234e-07 }, { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 } } },
};


static float sequenceTime(struct poly4d sequence[], int count) {
  float totalDuration = 0.0f;

  for (int i = 0; i < count; i++) {
    totalDuration += sequence[i].duration;
  }

  return totalDuration;
}

static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }
static float getVarPX() { return logGetFloat(logIdKalmanVarPX); }
static float getVarPY() { return logGetFloat(logIdKalmanVarPY); }
static float getVarPZ() { return logGetFloat(logIdKalmanVarPZ); }
static bool isBatLow() { return logGetInt(logIdPmState) == lowPower; }
static bool isCharging() { return logGetInt(logIdPmState) == charging; }
static bool isLighthouseAvailable() { return logGetFloat(logIdlighthouseEstBs0Rt) >= 0.0f || logGetFloat(logIdlighthouseEstBs1Rt) >= 0.0f; }

#ifdef USE_MELLINGER
static void enableMellingerController() { paramSetInt(paramIdStabilizerController, ControllerTypeMellinger); }
#endif
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }
// static void useCrossingBeamPositioningMethod() { paramSetInt(paramIdLighthouseMethod, 0); }
void pilotSetActivation(const uint8_t newActive) { isActive = newActive; }
void activateAll(const uint8_t isActive) {
  P2PPacket pk;

  pk.port = 0;
  pk.size = sizeof(ActivationUpdate);

  ActivationUpdate* activationUpdate = (ActivationUpdate*)&pk.data;
  activationUpdate->msgType = MSG_TYPE_ACTIVATION_UPDATE;
  activationUpdate->isActive = isActive;

  radiolinkSendP2PPacketBroadcast(&pk);

  pilotSetActivation(isActive);
}

void pilotP2PNetworkIsActive() {
  // Automatically activate if we receive P2P traffic.
  // This functionality activates a crashed and restarted CF in a running swarm.
  if (!hasAutoActivationTokenBeenUsed) {
    hasAutoActivationTokenBeenUsed = true;
    pilotSetActivation(true);

  }
}

static void defineTrajectory() {
  const uint32_t polyCount = sizeof(sequence) / sizeof(struct poly4d);
  trajectoryDurationMs = 1000 * sequenceTime(sequence, polyCount);
  crtpCommanderHighLevelWriteTrajectory(0, sizeof(sequence), (uint8_t*)sequence);
  crtpCommanderHighLevelDefineTrajectory(trajectoryId, CRTP_CHL_TRAJECTORY_TYPE_POLY4D, 0, polyCount);
}


// The flight time for one spiral sequence, that is going up in the center and down in the spiral
int getFlightCycleTimeMs() {
  return trajectoryDurationMs;
}

// The flight time from take off to the end of the last spiral
int getFullFlightTimeMs() {
  return (TAKE_OFF_TIME + DURATION_TO_INITIAL_POSITION)* 1000 + trajectoryDurationMs;
}

bool isPilotReadyForFlight() {
  const float VBAT_ACCEPT_LEVEL = 4.00f;
  const float vbat = logGetFloat(logIdPmVbat);

  const bool batteryCharged = (vbat >= VBAT_ACCEPT_LEVEL);
  if (!batteryCharged) {
    DEBUG_PRINT("Battery: %fV", (double)vbat);
  }

  return (STATE_WAIT_FOR_TAKE_OFF == state) && batteryCharged && isActive;
}

void takeOffAt(const uint32_t time) {
  takeOffTime = time;
}

bool hasPilotLanded() {
  return (STATE_WAIT_FOR_TAKE_OFF == state) && (takeOffTime == 0);
}

bool hasCrashed() {
  return STATE_CRASHED == state;
}


static void defineLedSequence() {
  ledseqRegisterSequence(&seq_lock);
}

void initPilot() {
  DEBUG_PRINT("Pilot signing on\n");

  // Get log and param ids
  logIdStateEstimateX = logGetVarId("stateEstimate", "x");
  logIdStateEstimateY = logGetVarId("stateEstimate", "y");
  logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
  logIdKalmanVarPX = logGetVarId("kalman", "varPX");
  logIdKalmanVarPY = logGetVarId("kalman", "varPY");
  logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");
  logIdPmState = logGetVarId("pm", "state");
  logIdPmVbat = logGetVarId("pm", "vbat");
  logIdlighthouseEstBs0Rt = logGetVarId("lighthouse", "estBs0Rt");
  logIdlighthouseEstBs1Rt = logGetVarId("lighthouse", "estBs1Rt");

  paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
  paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  paramIdLighthouseMethod = paramGetVarId("lighthouse", "method");



  #ifdef USE_MELLINGER
    enableMellingerController();
  #endif

  // useCrossingBeamPositioningMethod();
  enableHighlevelCommander();
  defineTrajectory();
  defineLedSequence();
  resetPositionLockData();
}

void pilotTimerCb(xTimerHandle timer) {
  uint32_t previous = now;
  now = xTaskGetTickCount();
  uint32_t delta = now - previous;

  if(supervisorIsTumbled()) {
    state = STATE_CRASHED;
  }

  if (isBatLow()) {
    terminateTrajectoryAndLand = true;
  }

  if (allActivate) {
    allActivate = 0;
    activateAll(1);
  }

  if (allDeactivate) {
    allDeactivate = 0;
    activateAll(0);
  }

  switch(state) {
    case STATE_IDLE:
      DEBUG_PRINT("I'm ready! Waiting for position lock...\n");
      state = STATE_WAIT_FOR_POSITION_LOCK;
      break;
    case STATE_WAIT_FOR_POSITION_LOCK:
      if (hasPositionLock()) {
        DEBUG_PRINT("Position lock acquired, ready for take off..\n");
        ledseqRun(&seq_lock);
        state = STATE_WAIT_FOR_TAKE_OFF;
      }
      break;
    case STATE_WAIT_FOR_TAKE_OFF:
      trajectoryStartTime = 0;
      if (takeOffTime != 0 && takeOffTime <= now) {
        takeOffTime = 0;
        DEBUG_PRINT("Taking off!\n");

        padX = getX();
        padY = getY();
        padZ = getZ();
        DEBUG_PRINT("Pad position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ);

        terminateTrajectoryAndLand = false;
        crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, TAKE_OFF_TIME);
        state = STATE_TAKING_OFF;
      }
      break;
    case STATE_TAKING_OFF:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Going to initial position, will be there in T -%f s\n", (double)(DURATION_TO_INITIAL_POSITION));
        crtpCommanderHighLevelGoTo(sequence[0].p[0][0] + trajecory_center_offset_x, sequence[0].p[1][0] + trajecory_center_offset_y, sequence[0].p[2][0] + trajecory_center_offset_z, sequence[0].p[3][0], DURATION_TO_INITIAL_POSITION, false);
        ledseqStop(&seq_lock);
        state = STATE_GOING_TO_INITIAL_POSITION;
      }
      flightTime += delta;
      break;
    case STATE_GOING_TO_INITIAL_POSITION:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("At initial position, starting trajectory\n");
        trajectoryStartTime = now;
        crtpCommanderHighLevelStartTrajectory(trajectoryId, SEQUENCE_SPEED, true, false);
        remainingTrajectories = trajectoryCount - 1;
        state = STATE_RUNNING_TRAJECTORY;
      }
      flightTime += delta;
      break;
    case STATE_RUNNING_TRAJECTORY:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        if (terminateTrajectoryAndLand || (remainingTrajectories == 0)) {
          terminateTrajectoryAndLand = false;
          DEBUG_PRINT("Terminating trajectory, going back to pad\n");
          float timeToPadPosition = 2.0;
          crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, timeToPadPosition, false);
          state = STATE_GOING_TO_PAD;
        } else {
          if (remainingTrajectories > 0) {
            DEBUG_PRINT("Trajectory finished, running one more time\n");
            crtpCommanderHighLevelStartTrajectory(trajectoryId, SEQUENCE_SPEED, true, false);
          }
          remainingTrajectories--;
        }
      }
      flightTime += delta;
      break;
    case STATE_GOING_TO_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Over pad, stabilizing position\n");
        stabilizeEndTime = now + 5000;
        state = STATE_WAITING_AT_PAD;
      }
      flightTime += delta;
      break;
    case STATE_WAITING_AT_PAD:
      if (now > stabilizeEndTime || ((fabs(padX - getX()) < MAX_PAD_ERR) && (fabs(padY - getY()) < MAX_PAD_ERR))) {
        if (now > stabilizeEndTime) {
          DEBUG_PRINT("Warning: failed to stabilize in time!\n");
        }

        DEBUG_PRINT("Landing...\n");
        crtpCommanderHighLevelLand(padZ, 1.0);
        state = STATE_LANDING;
      }
      flightTime += delta;
      break;
    case STATE_LANDING:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Landed. Re-charge me!\n");
        crtpCommanderHighLevelStop();
        landingTimeCheckCharge = now + 3000;
        state = STATE_CHECK_CHARGING;
      }
      flightTime += delta;
      break;
    case STATE_CHECK_CHARGING:
      if (now > landingTimeCheckCharge) {
        if (isCharging()) {
          DEBUG_PRINT("Charging started");
          ledseqRun(&seq_lock);
          state = STATE_WAIT_FOR_TAKE_OFF;
        } else {
          DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
          crtpCommanderHighLevelTakeoff(padZ + LANDING_HEIGHT, 1.0);
          state = STATE_REPOSITION_ON_PAD;
        }
      }
      break;
    case STATE_REPOSITION_ON_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Over pad, stabilizing position\n");
        crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, 1.5, false);
        state = STATE_GOING_TO_PAD;
      }
      flightTime += delta;
      break;
    case STATE_CRASHED:
      crtpCommanderHighLevelStop();
      break;
    default:
      break;
  }
}


static bool hasPositionLock() {
  bool result = false;

  // Store current state
  positionLockData[positionLockWriteIndex][0] = getVarPX();
  positionLockData[positionLockWriteIndex][1] = getVarPY();
  positionLockData[positionLockWriteIndex][2] = getVarPZ();

  positionLockWriteIndex++;
  if (positionLockWriteIndex >= LOCK_LENGTH) {
    positionLockWriteIndex = 0;
  }

  // Check if we have a lock
  int count = 0;

  float lXMax = FLT_MIN;
  float lYMax = FLT_MIN;
  float lZMax = FLT_MIN;

  float lXMin = FLT_MAX;
  float lYMin = FLT_MAX;
  float lZMin = FLT_MAX;

  for (int i = 0; i < LOCK_LENGTH; i++) {
    if (positionLockData[i][0] != FLT_MAX) {
      count++;

      lXMax = fmaxf(lXMax, positionLockData[i][0]);
      lYMax = fmaxf(lYMax, positionLockData[i][1]);
      lZMax = fmaxf(lZMax, positionLockData[i][2]);

      lXMin = fminf(lXMax, positionLockData[i][0]);
      lYMin = fminf(lYMin, positionLockData[i][1]);
      lZMin = fminf(lZMin, positionLockData[i][2]);
    }
  }

  result =
    (count >= LOCK_LENGTH) &&
    ((lXMax - lXMin) < LOCK_THRESHOLD) &&
    ((lYMax - lYMin) < LOCK_THRESHOLD) &&
    ((lZMax - lZMin) < LOCK_THRESHOLD &&
    isLighthouseAvailable() &&  // Make sure we have a deck and the Lighthouses are powered
    sensorsAreCalibrated());

  return result;
}

static void resetPositionLockData() {
    positionLockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
      positionLockData[i][0] = FLT_MAX;
      positionLockData[i][1] = FLT_MAX;
      positionLockData[i][2] = FLT_MAX;
    }
}

PARAM_GROUP_START(app)
  PARAM_ADD(PARAM_UINT8, stop, &terminateTrajectoryAndLand)
  PARAM_ADD(PARAM_FLOAT, offsx, &trajecory_center_offset_x)
  PARAM_ADD(PARAM_FLOAT, offsy, &trajecory_center_offset_y)
  PARAM_ADD(PARAM_FLOAT, offsz, &trajecory_center_offset_z)
  PARAM_ADD(PARAM_UINT8, trajcount, &trajectoryCount)
  PARAM_ADD(PARAM_UINT8, active, &isActive)
  PARAM_ADD(PARAM_UINT8, activeAll, &allActivate)
  PARAM_ADD(PARAM_UINT8, deactiveAll, &allDeactivate)
PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
  LOG_ADD(LOG_UINT8, state, &state)
  LOG_ADD(LOG_UINT32, uptime, &now)
  LOG_ADD(LOG_UINT32, flighttime, &flightTime)
LOG_GROUP_STOP(app)
