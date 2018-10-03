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

#define DEBUG_MODULE "APP"
#include "debug.h"



static xTimerHandle timer;
static bool isInit = false;

static void appTimer(xTimerHandle timer);

#define LED_LOCK         LED_GREEN_R
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
static uint32_t lockWriteIndex;
static float lockData[LOCK_LENGTH][3];
static void resetLockData();
static bool hasLock();

static bool hasButtonBeenPressed = false;

// #define USE_MELLINGER

// duration, x0-x7, y0-y7, z0-z7, yaw0-yaw7
static float sequence[] = {
#ifdef USE_MELLINGER
0.5, 4.4918854870533248e-11, -7.862572669528356e-07, 0.048012950358219272, -0.00061028154723608203, -0.232594452733279, -0.025319623200247693, 0.26978046781630138, -0.10410326794288177, 5.9853862376852293e-11, -8.8481917005444088e-07, 3.915464518519511e-05, 0.1500816855437945, 0.0052239298652139183, -0.27428037693941026, 0.053527881799730948, 0.072334246434389973, 0.99350949375730446, 0.36902111737465348, 0.035159164314672905, -0.016861539426115308, -0.00080325419610844989, 0.00023112081094550424, 7.3770734048743732e-06, -1.5621745716759916e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, -8.0672522606868346e-11, -0.037465260085528661, -0.1490648776063668, -0.083031342666567837, 0.24423753665029188, 0.24716761026830833, -0.21860080170837531, -0.011127774721214546, 0.011925960812386987, 0.047430751887137544, -0.012494985225289997, -0.23649571544196019, -0.17890504684724487, 0.19042516756989772, 0.2170799221901577, -0.14293786773812614, 1.184659273295283, 0.39120593760761035, 0.0089567656061386655, -0.017875221461034173, -0.0002046292304479811, 0.00024503022074448381, 1.8702649860481025e-06, -1.6039516315724731e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, -0.046891108652058838, -0.091630040513637531, 0.18985943539403982, 0.4562049011013401, 0.016721090161261467, -0.39558109463791574, -0.11134710754371967, 0.16287518179515093, -9.2045146153288223e-11, -0.14731128897963222, -0.28792951690278484, 0.11289536282985653, 0.47739464220436728, 0.14089840796383135, -0.36531945045919367, 0.061557693675318978, 1.3802619153775311, 0.38673071969085016, -0.017856021872985642, -0.017670736735640551, 0.00040794112109616683, 0.00024224048441837595, -3.7628484838861286e-06, -1.5371309594553006e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, 9.3196625541435253e-11, 0.32205137652315841, 0.40716967685094352, -0.42434634974902125, -0.67835811536849999, 0.037285159464452371, 0.4836851945964929, -0.14076876420150797, -0.10251262652129936, -0.1295849711451369, 0.47199330597817446, 0.64476502060339025, -0.24290484866514112, -0.57616712070750808, 0.040212459680138327, 0.16255651502479632, 1.5669874231947944, 0.35590044232998502, -0.043451950973248749, -0.016262020461832671, 0.00099271070222083091, 0.00022294327230037999, -9.1408111078597939e-06, -1.3647624509535854e-06, -7.2988653843530926e-18, 3.1415926535898011, -3.5584292186135076e-13, 3.9369469248989223e-12, -2.0875861241013526e-11, 5.8578082996061578e-11, -8.4281576957666141e-11, 4.8942425456531441e-11,
0.5, 0.17499999989622428, 0.15870897798255804, -0.83966963994476795, -0.78932601342423281, 0.58227969100681498, 0.71987660638025308, -0.22727024521464242, -0.1420035841713157, 8.404861311192673e-11, 0.54977727452166036, 0.49865933760401887, -0.8300967480275262, -0.83343262757292802, 0.27524017643327503, 0.56563160412735325, -0.22110733320598089, 1.7321107619678162, 0.30081613797773615, -0.066086701424399819, -0.013745074353386807, 0.0015098286347271746, 0.00018845295589871292, -1.3895955642347523e-05, -1.0993436493291022e-06, 1.5707963267948957, 3.1415926535899223, -3.59549132938534e-12, 3.8510221939396842e-11, -2.0716244969051248e-10, 5.9749688831128023e-10, -8.8194398986660435e-10, 5.220216694389172e-10,
0.5, -6.5224394276905921e-11, -0.81496983936138201, -0.5561636299756606, 1.3024953385613647, 0.93205010704483304, -0.5567504016229623, -0.60557416694589894, 0.29709846017463593, 0.25941333407238115, 0.17701730808965674, -1.2678319027488083, -0.88003628679220069, 0.97827560215906562, 0.81691598779924834, -0.43707857022072483, -0.10261703745383875, 1.8643790490384367, 0.22523171094424313, -0.084217752381144428, -0.010291424203271751, 0.0019240545695611245, 0.00014111892279866479, -1.7702683037614866e-05, -7.598600020726879e-07, -3.1415926535897909, 3.1415926535895191, 7.9925277353156248e-12, -8.701442261542103e-11, 4.6804508862897119e-10, -1.3393681289944669e-09, 1.9510423132111082e-09, -1.1357235682098423e-09,
0.5, -0.34999999982529495, -0.18326227902554604, 1.7273015437507782, 0.91071408549230559, -1.4039061152287302, -0.86067219141397699, 0.6553393440210159, 0.047081002943382376, -3.8006916751101021e-11, -1.0995566359999356, -0.57576373147070425, 1.8093489380736572, 0.96748993559354779, -0.86263137864808492, -0.60079086294983097, 0.36356347506090031, 1.9547784192264712, 0.13429811502307962, -0.096609502694975244, -0.0061364304364700823, 0.0022071589905224703, 8.4169008996852349e-05, -2.0304752777171023e-05, -3.6752686988952259e-07, -1.5707963267948959, 3.1415926535897758, 6.3512734949524739e-13, -7.5754673277435621e-12, 4.3152022809694168e-11, -1.281118725263569e-10, 1.9041599533999558e-10, -1.1205361362800956e-10,
0.5, 4.2509447685686586e-12, 1.3841435445482908, 0.5561239275644887, -2.316116311397427, -0.93733694729083838, 1.1720378237597653, 0.55160766757103008, -0.41597289774528984, -0.44058666558544829, -0.17701830634130677, 2.1867664662274886, 0.8792687682248147, -1.8301652140649523, -0.84816330442587884, 0.86717845575125863, -0.020819830448504301, 1.9971483048456293, 0.034212324501072498, -0.10241747504984113, -0.0015632491392789688, 0.0023398495468230423, 2.1481971384447082e-05, -2.1521330028166294e-05, 4.8788050767343104e-08, -7.2988653843530926e-18, 3.1415926535898011, -3.5584292186135076e-13, 3.9369469248989223e-12, -2.0875861241013526e-11, 5.8578082996061578e-11, -8.4281576957666141e-11, 4.8942425456531441e-11,
0.5, 0.52499999978282974, 0.15871090645661717, -2.6149148950110872, -0.78784328142343629, 2.2280040456429653, 0.78024178647711218, -1.0581594196211763, 0.096458132949277034, -3.3742886573615385e-11, 1.6493364374910287, 0.49858263843455669, -2.7882620994764808, -0.84364601872066325, 1.4638841999707186, 0.46137633268334011, -0.45075511159561094, 1.9886012682105114, -0.068204979396985763, -0.10124586573781343, 0.0031164650602825499, 0.0023130828999322079, -4.2668067501205389e-05, -2.12726937964429e-05, 4.6262019672886425e-07, 1.5707963267948957, 3.1415926535899223, -3.59549132938534e-12, 3.8510221939396842e-11, -2.0716244969051248e-10, 5.9749688831128023e-10, -8.8194398986660435e-10, 5.220216694389172e-10,
0.5, 7.3385861084714823e-11, -1.8770628574260375, -0.40706120784568617, 3.193610347142831, 0.69280203161332976, -1.7182816582797982, -0.33624597593066535, 0.46553976706284095, 0.59748737319151779, 0.12958769841932274, -2.9825692222317755, -0.64266812089612912, 2.5703105509113326, 0.66153637710815272, -1.215267198961536, 0.17467927943226169, 1.9297197757431539, -0.16597422666326997, -0.09317451799085695, 0.007583797300824216, 0.0021286835487458666, -0.00010391052440403136, -1.9574152781331256e-05, 8.4483785100277247e-07, -3.1415926535897909, 3.1415926535895191, 7.9925277353156248e-12, -8.701442261542103e-11, 4.6804508862897119e-10, -1.3393681289944669e-09, 1.9510423132111082e-09, -1.1357235682098423e-09,
0.5, -0.65310889110476023, -0.091633380728702515, 3.2646744130647192, 0.45363673394086412, -2.8337571073229459, -0.50013665355628811, 1.3277951579446874, -0.25015262788448867, 1.1197596591766803e-10, -2.0518036251704803, -0.28779667004466647, 3.5045372411640527, 0.4950847545711044, -1.917893432871743, -0.18474402340535453, 0.45931931370178025, 1.8245165038489415, -0.25243260466784712, -0.078753480819093696, 0.01153430625811834, 0.0017992181362724796, -0.0001580723484557371, -1.6540502898140463e-05, 1.1687259503994741e-06, -1.5707963267948959, 3.1415926535897758, 6.3512734949524739e-13, -7.5754673277435621e-12, 4.3152022809694168e-11, -1.281118725263569e-10, 1.9041599533999558e-10, -1.1205361362800956e-10,
0.5, -1.4688340869144394e-10, 2.1616504461753259, 0.14891670618949551, -3.6998536275564167, -0.26396829318986242, 2.0491163108559363, 0.017195083372774016, -0.43251766551552723, -0.68807403899567288, -0.047434477412963381, 3.4420054649570977, 0.23363129717335912, -3.0003902673389038, -0.3070417405005369, 1.3880747024863522, -0.31773479454651143, 1.6801608816508353, -0.32168811782882922, -0.058965524079142954, 0.014698771351723712, 0.0013471386779893462, -0.00020146079522255001, -1.2381295553352363e-05, 1.4140240902995181e-06, -7.2988653843530926e-18, 3.1415926535898011, -3.5584292186135076e-13, 3.9369469248989223e-12, -2.0875861241013526e-11, 5.8578082996061578e-11, -8.4281576957666141e-11, 4.8942425456531441e-11,
0.5, 0.69999999981812921, 3.070690851441765e-06, -3.5024775597741176, 0.0023551824483679373, 3.0588542565821681, 0.095410736847131639, -1.3919978807590598, 0.3728201661490993, -1.7572918356680739e-10, 2.1991174411195558, -0.00011424369334119319, -3.7662490173587155, -0.015202852407239517, 2.1030076700808706, -0.15498266103192673, -0.38696131036086384, 1.5064905062426943, -0.36902111737467885, -0.03515916431430547, 0.016861539422113821, 0.00080325421911007708, -0.00023112087770653381, -7.3769791361063767e-06, 1.5621228510617674e-06, 1.5707963267948957, 3.1415926535899223, -3.59549132938534e-12, 3.8510221939396842e-11, -2.0716244969051248e-10, 5.9749688831128023e-10, -8.8194398986660435e-10, 5.220216694389172e-10,
0.5, 1.9654804475979627e-10, -2.1616512962148695, 0.14913996665498161, 3.6991986744747476, -0.2342586140607964, -2.0758949035797416, 0.32005558123861139, 0.32575483844482755, 0.68807403905066089, -0.047428467453543531, -3.4419696241909059, 0.23824061634725704, 3.0051648506676183, -0.12033405382289515, -1.3392973353057207, 0.41165476606030854, 1.3153407267047155, -0.39120593760755928, -0.008956765607423537, 0.017875221472410704, 0.00020462918030012309, -0.00024503010085911666, -1.8704137389287136e-06, 1.6040261796377381e-06, -3.1415926535897909, 3.1415926535895191, 7.9925277353156248e-12, -8.701442261542103e-11, 4.6804508862897119e-10, -1.3393681289944669e-09, 1.9510423132111082e-09, -1.1357235682098423e-09,
0.5, -0.65310889121098947, 0.091627756079972575, 3.2646051740241591, -0.45794980202759095, -2.8429808938737571, 0.32548998059654782, 1.2335645210684147, -0.43159208034553276, 2.0792076057500389e-10, -2.0518052673207783, 0.28800460595179256, 3.5032719689745035, -0.46741571959851291, -1.9696257013087182, 0.46677423001855683, 0.25306937004277752, 1.1197380846224672, -0.38673071969093542, 0.017856021875704408, 0.017670736702793767, -0.00040794092980466791, -0.00024224106202289477, 3.7637188113192825e-06, 1.5366138922224972e-06, -1.5707963267948959, 3.1415926535897758, 6.3512734949524739e-13, -7.5754673277435621e-12, 4.3152022809694168e-11, -1.281118725263569e-10, 1.9041599533999558e-10, -1.1205361362800956e-10,
0.5, -2.0907239318514751e-10, 1.8770651797772393, -0.40724476589914943, -3.1918209820677399, 0.66837919284741809, 1.7914421335875956, -0.5851399736554892, -0.17385829985206194, -0.59748737334174906, 0.12958268671151801, 2.9824713034387607, -0.64650992151595765, -2.5833549551207282, 0.5060760068753094, 1.082004953539226, -0.43127341339517905, 0.93301257680520511, -0.35590044233002893, 0.043451950974076878, 0.016262020453489505, -0.00099271065797029538, -0.00022294339518270573, 9.1409802102282477e-06, 1.3646716967871757e-06, -7.2988653843530926e-18, 3.1415926535898011, -3.5584292186135076e-13, 3.9369469248989223e-12, -2.0875861241013526e-11, 5.8578082996061578e-11, -8.4281576957666141e-11, 4.8942425456531441e-11,
0.5, 0.524999999966824, -0.15870669354901326, -2.6147949694702048, 0.79107091431643228, 2.2439801128863883, -0.64978549285261966, -0.89494716756270631, 0.4107204822849026, -1.9992408428974056e-10, 1.6493392817787644, -0.49873442665349693, -2.7860705837703055, 0.82345370492304315, 1.5534871170625191, -0.66708638394154296, -0.093519730343177732, 0.76788923803218212, -0.30081613797766027, 0.066086701422296334, 0.013745074377399363, -0.0015098287737359513, -0.00018845252715789839, 1.3895285422118854e-05, 1.0997605165536861e-06, 1.5707963267948957, 3.1415926535899223, -3.59549132938534e-12, 3.8510221939396842e-11, -2.0716244969051248e-10, 5.9749688831128023e-10, -8.8194398986660435e-10, 5.220216694389172e-10,
0.5, 1.810998927151834e-10, -1.3841467169390254, 0.55623871902470345, 2.3136719932412504, -0.92207118442309499, -1.2719768917815177, 0.70702894660657656, 0.017528603478194026, 0.44058666579066702, -0.17701502365597171, -2.1866327066700761, 0.88178118772751635, 1.8479842014955719, -0.74682487356931126, -0.68513884360639254, 0.37133393619385802, 0.63562095096156179, -0.22523171094425098, 0.084217752381483768, 0.010291424198616712, -0.0019240545433630682, -0.00014111899492593079, 1.7702779129162447e-05, 7.5981119855666971e-07, -3.1415926535897909, 3.1415926535895191, 7.9925277353156248e-12, -8.701442261542103e-11, 4.6804508862897119e-10, -1.3393681289944669e-09, 1.9510423132111082e-09, -1.1357235682098423e-09,
0.5, -0.35000000003775289, 0.18325999459198064, 1.7271630656648616, -0.91245898639259682, -1.4223536886174248, 0.79058107774374731, 0.46687806897348022, -0.31579790118817797, 1.5388248040809648e-10, -1.0995599203004462, 0.57583882051889601, 1.8068183937396194, -0.95751101303717545, -0.96609591454767196, 0.70224564227710407, -0.0489364111972782, 0.545221580773527, -0.1342981150230369, 0.096609502693222146, 0.0061364304587174226, -0.00220715912258864, -8.4168603105967739e-05, 2.0304127288305033e-05, 3.6790852357147667e-07, -1.5707963267948959, 3.1415926535897758, 6.3512734949524739e-13, -7.5754673277435621e-12, 4.3152022809694168e-11, -1.281118725263569e-10, 1.9041599533999558e-10, -1.1205361362800956e-10,
0.5, -1.2012652952900517e-10, 0.81497301175212267, -0.5561990166136086, -1.3000510204045479, 0.92735802466484218, 0.65668946966290087, -0.65306244727101614, 0.10134583412427306, -0.25941333427760005, 0.17701602190767388, 1.267698143189854, -0.88101366914317703, -0.99609458968187936, 0.77807219046173592, 0.25503895768658541, -0.24789706806333889, 0.50285169515436923, -0.034212324501146869, 0.10241747505185006, 0.0015632491159048431, -0.0023398494120570085, -2.1482376737650271e-05, 2.1521939616064391e-05, -4.9149192155562796e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, 0.17500000008021807, -0.15870862202293898, -0.83954971440783788, 0.78958818236161532, 0.59825575798724318, -0.71015067215854133, -0.06405799435658184, 0.17225876588908434, -8.2132487311752482e-11, 0.54978011880939881, -0.4986577274842609, -0.8279052323182835, 0.83366709605426625, 0.36484309356772421, -0.56283111254893692, 0.13612804806837539, 0.5113987317894878, 0.068204979397018875, 0.10124586573687946, -0.0031164650525906659, -0.0023130829278084851, 4.2668111646646632e-05, 2.1272676582604824e-05, -4.6263536100922093e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, 4.2489668413621593e-11, -0.32205369887434221, 0.40713629689350805, 0.4225569846775234, -0.68282310911036881, -0.11044563472102985, 0.43770075491442073, -0.15091270296502046, 0.10251262667152972, -0.12958541398572218, -0.4718953871844867, 0.64441302180094606, 0.25594925291850601, -0.59144526340846282, 0.093049785945052046, 0.094037618814137122, 0.57028022425684533, 0.16597422666329717, 0.093174517989487574, -0.0075837972856997457, -0.002128683628780556, 0.00010391075358226039, 1.9573812402153105e-05, -8.4463268176228253e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, -0.0468911087582881, 0.091631096295129735, 0.1897901963509992, -0.45538163484025679, 0.0074973034635161754, 0.43004553994508415, -0.20557774506333112, -0.018564270279785688, 3.8995856142868671e-12, -0.14731293112991461, 0.28787175909303842, 0.11163009064853219, -0.48510583201541213, 0.089166139685785156, 0.28619880271278153, -0.14469224982625098, 0.67548349615105874, 0.2524326046677921, 0.078753480821380437, -0.011534306293721406, -0.0017992178914635667, 0.0001580715173044145, 1.6541874378599997e-05, -1.1696053860538696e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.5, 3.1007890190989833e-11, 0.037466110125064254, -0.14899179523773468, 0.083686295742147629, 0.25398937064542132, -0.22038901771051569, -0.11864986260612624, 0.11789060158648455, -0.011925960867375244, 0.047432192979336221, 0.012459144459939332, -0.23537619808725951, 0.17413046356225853, 0.23695062663651342, -0.26585728921346158, 0.049017896141081575, 0.8198391183491639, 0.32168811782885293, 0.058965524079244304, -0.014698771361468084, -0.00134713858729758, 0.00020146044624436293, 1.238191287194587e-05, -1.4144383182447727e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#else
0.625, 4.4918854302262862e-11, -6.2900581347062405e-07, 0.030728288229258879, -0.00031246415217600312, -0.095270687839576249, -0.0082967341302269942, 0.070721330955232767, -0.021832037657307977, 5.9853864718468013e-11, -7.0785533609901054e-07, 2.5058972919617266e-05, 0.076841822998408818, 0.0021397216728780062, -0.089876193915769828, 0.014032013046895806, 0.015169590957620046, 0.99350949375730402, 0.29521689389968048, 0.022501865162593199, -0.0086331081977732791, -0.00032901286522727822, 7.573353886797932e-05, 1.9340103078571493e-06, -3.2768552320150972e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -8.0672522659162407e-11, -0.02997220806842299, -0.095401521668074363, -0.042512047445276042, 0.10003969501188813, 0.080991882532983783, -0.057304888563469304, -0.002333663500959629, 0.01192596081238697, 0.037944601509709588, -0.0079967905441727001, -0.12108580630639409, -0.073279507188200602, 0.062398518908469432, 0.056906199123376022, -0.029976243520517441, 1.184659273295283, 0.31296475008599217, 0.0057323299899812476, -0.0091521134052117668, -8.3816060333927956e-05, 8.0291338349992739e-05, 4.9046964018450123e-07, -3.3646190745948373e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.046891108652058797, -0.073304032410904885, 0.12151003865205912, 0.23357690936499415, 0.0068489585253275085, -0.12962401308034341, -0.029188976171974149, 0.034157401330628979, -9.2045123130454516e-11, -0.11784903118370696, -0.18427489081776516, 0.05780242576876847, 0.19554084544735589, 0.046169590320648178, -0.095766302020148728, 0.012909584040202321, 1.3802619153775311, 0.30938457575269485, -0.01142785399893502, -0.009047417207210225, 0.00016709267826866715, 7.9377371638144964e-05, -9.8641877278626522e-07, -3.2235452177453716e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 9.3196599925483228e-11, 0.25764110121852835, 0.26058859318458127, -0.21726533107134702, -0.27785548405553329, 0.012217601054668027, 0.12679517165065524, -0.029521349537445944, -0.1025126265212993, -0.10366797691610281, 0.3020757158258835, 0.3301196905501706, -0.099493826018312748, -0.18879844210243998, 0.010541455018349445, 0.034090572064950579, 1.5669874231947942, 0.28472035386391642, -0.027809248621474155, -0.0083261544876062927, 0.00040661434832335358, 7.3053955389098339e-05, -2.396103277116684e-06, -2.8625763278737563e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 0.17499999989622417, 0.12696718238603888, -0.53738856956446901, -0.4041349188747167, 0.23850176144246421, 0.23588916636590354, -0.059577531148131482, -0.029780310060709539, 8.4048571897139909e-11, 0.43982181961733285, 0.31914197606650258, -0.42500953498961896, -0.34137400425561265, 0.09019070101719652, 0.14827693122859589, -0.04636956860310755, 1.732110761967816, 0.24065291038208278, -0.042295488909266883, -0.0070374780889168155, 0.00061842589323404546, 6.1752074936658361e-05, -3.6425252935022279e-06, -2.3064695486117377e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -6.5224424522071531e-11, -0.65197587148911684, -0.35594472318421078, 0.6668776133417017, 0.38176772385296182, -0.18243597162135558, -0.15874763439837183, 0.062306062984609549, 0.25941333407238121, 0.14161384647171787, -0.81141241775905781, -0.45057857883906766, 0.40070168665001249, 0.26768703087087464, -0.11457752470128656, -0.021520352536793599, 1.8643790490384362, 0.1801853687554002, -0.053899361523855258, -0.0052692091935450129, 0.00078809275989252532, 4.6241828280847955e-05, -4.6406291141514042e-06, -1.5936366972885969e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.34999999982529473, -0.14660982322043484, 1.1054729880004359, 0.46628561177264399, -0.57503994480037657, -0.28202506367610825, 0.17179327699150249, 0.0098736019518828304, -3.8006831335178417e-11, -0.87964530879995917, -0.36848878814108499, 0.92638665629256101, 0.39628387762357636, -0.28266705016527188, -0.15749371996545747, 0.076244786879373472, 1.9547784192264717, 0.1074384920183232, -0.061830081721508917, -0.0031418524132000619, 0.00090405245513019332, 2.7580189304702177e-05, -5.3223994892060122e-06, -7.7250079815019485e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 4.2507315796081017e-12, 1.1073148356386371, 0.35591931364124879, -1.1858515514355048, -0.3839332136099744, 0.38405335408905622, 0.14460064040749124, -0.08723583944449867, -0.44058666558544812, -0.14161464507301008, 1.3995305383847718, 0.4501856093382588, -0.74963567171177203, -0.27792615152430833, 0.22732562902396772, -0.0043662348696520544, 1.9971483048456284, 0.027369859600867057, -0.065547184031854611, -0.00080038356025787134, 0.0009584023791418373, 7.0392019879220131e-06, -5.6416777180224807e-06, 1.0228740945059605e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 0.52499999978282919, 0.12696872516529237, -1.6735455328070596, -0.40337576008886361, 0.91259045709421038, 0.25566962859932219, -0.27739014290986813, 0.020228736651639153, -3.3743116591272308e-11, 1.319469149992837, 0.3190928885978605, -1.4275901949297671, -0.34555740927812378, 0.47968557467206613, 0.1209470373217177, -0.094530198362103482, 1.988601268210511, -0.054563983517639567, -0.064797354070910154, 0.0015956300992181999, 0.00094743880668467478, -1.3981588649823988e-05, -5.5763754527790772e-06, 9.6957913147023786e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 7.3385985633239077e-11, -1.5016502859408407, -0.26051917302103711, 1.635128497735207, 0.28377171215873292, -0.56304653381245073, -0.088144865076653534, 0.097630765337065023, 0.59748737319151757, 0.10367015873542322, -1.9088443022275061, -0.32904607790623203, 1.0527992016861027, 0.21677223997370446, -0.31857500451273457, 0.036632899978424185, 1.9297197757431519, -0.13277938133066394, -0.059631691512487199, 0.0038829042017404078, 0.00087190885564121783, -3.4049574710279629e-05, -5.1310421336988656e-06, 1.7708048698497331e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.65310889110475989, -0.073306704582940682, 2.0893916243608315, 0.23226200778288328, -1.1607069111809285, -0.16388477859111455, 0.34807353383476602, -0.052460808366659673, 1.119762302538093e-10, -1.641442900136391, -0.18418986882846816, 1.7943230674748429, 0.20278671547848601, -0.62845532010078209, -0.048429537247282259, 0.096326241723501543, 1.8245165038489395, -0.20194608373430886, -0.050402227723502634, 0.005905564798789808, 0.0007369597678768446, -5.1797182711628459e-05, -4.3359621071405816e-06, 2.4508984711777094e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -1.4688340171320687e-10, 1.7293203569402593, 0.095306691961308615, -1.8943250573088608, -0.10812141289254228, 0.67145443275065919, 0.0045075879187332964, -0.090705528716287454, -0.68807403899567243, -0.037947581930300366, 2.2028834975709239, 0.11961922416652265, -1.2289598535596626, -0.10061143739940563, 0.36387545466505566, -0.066633815921159883, 1.6801608816508351, -0.25735049426313522, -0.037737935409348944, 0.0075257709227945033, 0.00055178803654128237, -6.6014740935514489e-05, -3.2456139395310284e-06, 2.9651507789225116e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 0.6999999998181291, 2.4565526563390894e-06, -2.2415856382548558, 0.0012058534086316149, 1.2529067035164145, 0.031264190206271143, -0.36490389240673349, 0.078186055688315756, -1.7572936347179502e-10, 1.7592939528956626, -7.3115964061339528e-05, -1.9283194968850859, -0.006227088357228407, 0.68911355335937119, -0.040627774728035582, -0.081151668576963709, 1.5064905062426932, -0.295216893899857, -0.02250186515824935, 0.0086331081577229359, 0.0003290130456570489, -7.5733966056260759e-05, -1.9335001597892323e-06, 3.2744359646346731e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 1.9654805889082338e-10, -1.7293210369719123, 0.095449578659457496, 1.8939897213290751, -0.095952328311110696, -0.68022924202412949, 0.083900650311880251, 0.06831574108340556, 0.68807403905066078, -0.037942773962861956, -2.202860559481556, 0.12197919556443894, 1.2309155228560091, -0.039431062806421557, -0.3510887606113306, 0.086330261571199149, 1.3153407267047152, -0.31296475008611702, -0.0057323299869332855, 0.0091521133768233762, 8.3816190143572205e-05, -8.0291650169584194e-05, -4.9009241202735131e-07, 3.3628085568013069e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.65310889121098903, 0.073302204863997855, 2.0893473113750267, -0.23447029863461805, -1.1644849741449512, 0.10665655687292215, 0.32337153777655553, -0.090511419432887444, 2.0792086763181953e-10, -1.6414442138566256, 0.18432294780916944, 1.7936752481147158, -0.19145347874562776, -0.64540694981211888, 0.12236206376630525, 0.053072493544626724, 1.1197380846224674, -0.30938457575281036, 0.011427854001957076, 0.0090474171781664489, -0.00016709254403370283, -7.9377693564130033e-05, 9.8680415325808284e-07, 3.2217288736677106e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -2.0907239355689153e-10, 1.5016521438217993, -0.26063665017565585, -1.6342123428166586, 0.27376811738000351, 0.58701975836156817, -0.15339093329098955, -0.036460728105449591, -0.59748737334174906, 0.1036661493692653, 1.9087816341996011, -0.33101307980535261, -1.0581421896655052, 0.16583098604586988, 0.28364110640617896, -0.090444590081215068, 0.93301257680520466, -0.28472035386408096, 0.027809248624979667, 0.0083261544569808912, -0.00040661421443037869, -7.305426521963214e-05, 2.3964657678063833e-06, 2.8608920462363861e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 0.524999999966824, -0.12696535483922297, -1.6734687804606387, 0.40502830812759422, 0.91913425424796991, -0.21292171031842208, -0.23460503027188637, 0.086134328077516906, -1.9992419978763553e-10, 1.3194714254230102, -0.31919003305823651, -1.4264681388899634, 0.33728663753239252, 0.5090466585337291, -0.17487269305539327, -0.019612508938943198, 0.76788923803218212, -0.24065291038216868, 0.042295488911346268, 0.0070374780710066396, -0.00061842581854850787, -6.1752238502279369e-05, 3.6427055509205168e-06, 2.3056868844435503e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 1.811000827035744e-10, -1.1073173735512436, 0.35599278017624247, 1.1846000605359217, -0.37768035712359171, -0.41680138793861754, 0.18534339622948118, 0.0036760145586164665, 0.4405866657906668, -0.14161201892480935, -1.3994449322681819, 0.45147196811104628, 0.75693432895492196, -0.2447195746201295, -0.17960503696384425, 0.077874370671495829, 0.63562095096156135, -0.18018536875542662, 0.053899361524620999, 0.0052692091864285613, -0.00078809272987595146, -4.6241891642132781e-05, 4.6406934645839762e-06, 1.5933935470308871e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.35000000003775283, 0.14660799567361293, 1.1053843620249302, -0.46717900102828391, -0.58259607087726062, 0.25905760759870777, 0.12238928446311544, -0.066227619984381106, 1.5388250451345304e-10, -0.87964793624036852, 0.36853684513232871, 0.92509101759265788, -0.39219651093093999, -0.31657030930110447, 0.18408948167677419, -0.010262709275449115, 0.54522158077352656, -0.10743849201847891, 0.061830081724877625, 0.0031418523837540005, -0.00090405232665561957, -2.7580486668478083e-05, 5.322748288739285e-06, 7.708732742307279e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -1.2012658162373468e-10, 0.65197840940169915, -0.35596737063272782, -0.66562612244684116, 0.37984584690059559, 0.21518400542639846, -0.17119640218889084, 0.021253761879370073, -0.25941333427759999, 0.14161281752614827, 0.81132681164133313, -0.45107899860003092, -0.40800034393850837, 0.25495869538028909, 0.066856932513655154, -0.051987783204158214, 0.50285169515436923, -0.027369859600916979, 0.065547184033163328, 0.00080038354748782244, -0.00095840231941885185, -7.039345703157554e-06, 5.6418489657963655e-06, -1.0308400814832863e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 0.17500000008021807, -0.12696689761836677, -0.53731181722067045, 0.40426914936614672, 0.24504555848458573, -0.23270217228288881, -0.016792418837603778, 0.036125281523915038, -8.213256520628263e-11, 0.43982409504752062, -0.31914094558995276, -0.42388747894669443, 0.34147004254226304, 0.11955178490504456, -0.14754279917512608, 0.02854812083034736, 0.51139873178948747, 0.054563983517574202, 0.064797354072514954, -0.0015956301148080988, -0.0009474387331400775, 1.3981408229565554e-05, 5.5765967329643518e-06, -9.7064840026527449e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 4.2489696149839088e-11, -0.25764295909947438, 0.26056723001185467, 0.21634917615477478, -0.27968434549082949, -0.036190825587936265, 0.11474062670025328, -0.031648687687202831, 0.10251262667152972, -0.10366833118858387, -0.30201304779793375, 0.32993946716088052, 0.10483681400070624, -0.1938047839260652, 0.024392443101515275, 0.019721118030135956, 0.5702802242568451, 0.13277938133061742, 0.059631691513946532, -0.0038829042170611108, -0.00087190878280374903, 3.404940020536248e-05, 5.1312476915629961e-06, -1.7717477082791394e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, -0.046891108758288114, 0.073304877036105956, 0.12146572566458916, -0.23315539703774815, 0.003070895496534924, 0.14091732253433931, -0.053890972408150502, -0.0038932096515460037, 3.8995921993864238e-12, -0.11785034490393083, 0.18423792581952633, 0.057154606412178288, -0.19869934879390633, 0.02921796065270485, 0.075025298938343477, -0.030344164111035604, 0.6754834961510584, 0.20194608373424805, 0.050402227725423743, -0.005905564820394206, -0.00073695965626564494, 5.1796891965085726e-05, 4.336333794919538e-06, -2.4527510353217143e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.625, 3.1007889029740659e-11, 0.029972888100051272, -0.095354748952148655, 0.042847383419978478, 0.104034046216314, -0.072217073323131417, -0.03110334958347739, 0.024723451090123368, -0.011925960867375246, 0.037945754383469747, 0.007973852454344196, -0.12051261342052287, 0.071323837874394114, 0.07764398133798163, -0.069692893225717034, 0.010279797893863685, 0.81983911834916312, 0.25735049426307754, 0.037737935410902222, -0.0075257709385837439, -0.00055178795969360549, 6.601454806449519e-05, 3.2458545265361512e-06, -2.9663274941073866e-07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#endif
};

// The array of data is copy/pasted from the trajectory generator. Unfortunately it does not use the same
// order as in the firmware. Shuffle data around.
static void reorganizeData(float sequence[], int size) {
  int count = size / sizeof(float);
  const int perRow = 1 + 8 * 4;

  for (int i = 0; i < count / perRow; i++) {
    float* row = &sequence[i * perRow];
    float duration = row[0];

    for (int j = 0; j < 8 * 4; j++) {
      row[j] = row[j + 1];
    }

    row[8 * 4] = duration;
  }
}

void appInit() {
  if (isInit) {
    return;
  }

  timer = xTimerCreate("AppTimer", M2T(100), pdTRUE, NULL, appTimer);
  xTimerStart(timer, 100);

  pinMode(DECK_GPIO_IO3, INPUT_PULLUP);
  reorganizeData(sequence, sizeof(sequence));

  #ifdef USE_MELLINGER
  setControllerType(ControllerTypeMellinger);
  #endif

  commanderEnableHighLevel(true);
  crtpCommanderHighLevelDefineTrajectory(1, 0, sequence, sizeof(sequence));

  resetLockData();

  isInit = true;
}

static bool isButtonPressed() {
  return !digitalRead(DECK_GPIO_IO3);
}

enum State {
  STATE_IDLE = 0,
  STATE_WAIT_FOR_POSITION_LOCK,
  STATE_WAIT_FOR_TAKE_OFF,
  STATE_TAKING_OFF,
  STATE_GOING_TO_INITIAL_POSITION,
  STATE_RUNNING_TRAJECTORY,

  // Low battery states
  STATE_GOING_TO_PAD,
  STATE_LANDING,
  STATE_EXHAUSTED,
};

static enum State state = STATE_IDLE;

#define TAKE_OFF_HEIGHT 0.5
#define SEQUENCE_SPEED 1.0

static void appTimer(xTimerHandle timer) {
  if (isButtonPressed()) {
    if (! hasButtonBeenPressed) {
      hasButtonBeenPressed = true;
      ledseqRun(LED_LOCK, seq_armed);
    }
  }

  switch(state) {
    case STATE_IDLE:
      DEBUG_PRINT("Let's go! Waiting for position lock...\n");
      state = STATE_WAIT_FOR_POSITION_LOCK;
      break;
    case STATE_WAIT_FOR_POSITION_LOCK:
      if (hasLock()) {
        DEBUG_PRINT("Position lock acquired, ready for take off..\n");
        ledseqRun(LED_LOCK, seq_lps_lock);
        state = STATE_WAIT_FOR_TAKE_OFF;
      }
      break;
    case STATE_WAIT_FOR_TAKE_OFF:
      if (hasButtonBeenPressed) {
        DEBUG_PRINT("Taking off!\n");
        crtpCommanderHighLevelTakeOff(TAKE_OFF_HEIGHT, 1.0, 0);
        state = STATE_TAKING_OFF;
      }
      break;
    case STATE_TAKING_OFF:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Hovering, going to initial position...\n");
        ledseqStop(LED_LOCK, seq_lps_lock);
        ledseqStop(LED_LOCK, seq_armed);
        crtpCommanderHighLevelGoTo(sequence[0], sequence[8], sequence[16], sequence[24], 2.0, false, 0);
        state = STATE_GOING_TO_INITIAL_POSITION;
      }
      break;
    case STATE_GOING_TO_INITIAL_POSITION:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("At initial position, starting trajectory...\n");
        crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, false, false, 0);
        state = STATE_RUNNING_TRAJECTORY;
      }
      break;
    case STATE_RUNNING_TRAJECTORY:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        if (isBatLow()) {
          DEBUG_PRINT("Battery low, going to pad...\n");
          crtpCommanderHighLevelGoTo(-0.4, -0.5, 0.4, 0.0, 2.0, false, 0);
          state = STATE_GOING_TO_PAD;
        } else {
          DEBUG_PRINT("Trajectory finished, restarting...\n");
          crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, false, false, 0);
        }
      }
      break;
    case STATE_GOING_TO_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("At pad, landing...\n");
        crtpCommanderHighLevelLand(0.02, 1.0, 0);
        state = STATE_LANDING;
      }
      break;
    case STATE_LANDING:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Landed. Feed me!\n");
        crtpCommanderHighLevelStop();
        state = STATE_EXHAUSTED;
      }
      break;
    default:
      break;
  }
}


static bool hasLock() {
  bool result = false;

  // Store current state
  lockData[lockWriteIndex][0] = getVarPX();
  lockData[lockWriteIndex][1] = getVarPY();
  lockData[lockWriteIndex][2] = getVarPZ();

  lockWriteIndex++;
  if (lockWriteIndex >= LOCK_LENGTH) {
    lockWriteIndex = 0;
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
    if (lockData[i][0] != FLT_MAX) {
      count++;

      lXMax = fmaxf(lXMax, lockData[i][0]);
      lYMax = fmaxf(lYMax, lockData[i][1]);
      lZMax = fmaxf(lZMax, lockData[i][2]);

      lXMin = fminf(lXMax, lockData[i][0]);
      lYMin = fminf(lYMin, lockData[i][1]);
      lZMin = fminf(lZMin, lockData[i][2]);
    }
  }

  // TODO krri Check that all anchors are received?

  result = (count >= LOCK_LENGTH) && ((lXMax - lXMin) < LOCK_THRESHOLD) && ((lYMax - lYMin) < LOCK_THRESHOLD) && ((lZMax - lZMin) < LOCK_THRESHOLD && sensorsAreCalibrated());
  return result;
}

static void resetLockData() {
    lockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
      lockData[i][0] = FLT_MAX;
      lockData[i][1] = FLT_MAX;
      lockData[i][2] = FLT_MAX;
    }
}
