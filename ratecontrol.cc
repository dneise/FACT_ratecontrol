#include <valarray>

#include "Dim.h"
#include "Event.h"
#include "Shell.h"
#include "StateMachineDim.h"
#include "Connection.h"
#include "Configuration.h"
#include "Console.h"
#include "externals/PixelMap.h"

#include "tools.h"

#include "LocalControl.h"

#include "HeadersFTM.h"
#include "HeadersLid.h"
#include "HeadersDrive.h"
#include "HeadersRateScan.h"
#include "HeadersFeedback.h"
#include "HeadersRateControl.h"

namespace ba    = boost::asio;
namespace bs    = boost::system;
namespace dummy = ba::placeholders;

using namespace std;

// ------------------------------------------------------------------------

#include "DimDescriptionService.h"
#include "DimState.h"

// The threshold T vs. current I dependency is modelled as:
// T = factor * pow(I, power);
// where:
//  - I is the patch current in uA and
//  - T is the threshold needed to keed the overall camera rate stable.

struct threshold_vs_current_fit_parameter
{
    double factor;
    double power;
    double constant;
    double chisquare;
};

threshold_vs_current_fit_parameter fits_parameters[320] = {
   { 224.700546545, 0.340238565568, -81.5206065264, 813.579343977},
   { 218.503850664, 0.343274631028, -86.268318242, 816.518779409},
   { 175.267775322, 0.380404017123, -28.9355598295, 804.194234327},
   { 177.830314638, 0.373306439156, -31.2377637393, 809.116152009},
   { 159.27893043, 0.39473422593, -13.1686155188, 807.320980676},
   { 160.129257953, 0.393343976949, -12.1558454834, 802.268481296},
   { 174.479965878, 0.382011951491, -34.8461307303, 821.782027275},
   { 175.327803326, 0.377505571935, -29.5873974101, 803.401406776},
   { 194.366474558, 0.364146090422, -61.5042756438, 816.895414149},
   { 174.327840327, 0.378562481225, -25.7399963115, 804.405437866},
   { 229.554080742, 0.330312981776, -98.8133852289, 807.724620285},
   { 232.322040944, 0.326331443716, -98.2117463695, 809.439557968},
   { 229.215035585, 0.336645919471, -102.55101849, 809.365702842},
   { 183.036547151, 0.37175154161, -35.3840436515, 804.318230406},
   { 217.939621654, 0.345740575286, -80.2593866058, 817.295096568},
   { 226.603241128, 0.336060596659, -91.0126060802, 810.608456722},
   { 180.949643326, 0.37627140477, -40.4236559571, 810.166009527},
   { 177.409673164, 0.381429350326, -32.7513490662, 807.904062028},
   { 215.122880371, 0.343499023135, -80.2251581298, 823.939314301},
   { 207.831349561, 0.344398052387, -71.7492105506, 806.345883185},
   { 162.874232538, 0.395013397078, -12.5444470812, 805.736285735},
   { 162.606404834, 0.390223419415, -14.9603454886, 802.813549656},
   { 177.348955339, 0.376501440331, -32.9182294574, 812.155956306},
   { 178.475911038, 0.372346806087, -29.29269584, 808.409139421},
   { 169.695294121, 0.386614944689, -30.3722920526, 808.990498835},
   { 156.787656241, 0.395895466863, -8.41391248696, 804.179113402},
   { 152.254176491, 0.401783857663, 4.86818903389, 798.158792455},
   { 165.02953624, 0.387647320255, -13.0110944977, 804.499925258},
   { 193.535837393, 0.362853872016, -52.6611367813, 812.5248506},
   { 171.867558516, 0.381791903452, -26.2687556492, 809.844420759},
   { 151.413096076, 0.394675282115, 5.91506130168, 798.110787872},
   { 173.552991664, 0.377240064715, -21.3644532543, 802.647660713},
   { 246.68134201, 0.317600707254, -122.230771306, 806.102531264},
   { 202.579882865, 0.350529488673, -61.9380057688, 808.458639478},
   { 231.850033761, 0.325475003689, -104.346375001, 808.439819793},
   { 207.352956468, 0.344265470718, -71.6809751407, 805.787240912},
   { 175.247903797, 0.379174748638, -36.2325838847, 808.28790618},
   { 172.800984582, 0.379528598371, -29.5676071806, 797.329913029},
   { 196.437616481, 0.359172153626, -61.7816585255, 884.558233585},
   { 167.516114866, 0.386877335358, -31.086081183, 798.624340916},
   { 163.12147656, 0.387187445024, -17.7882158606, 790.242901174},
   { 250.892314264, 0.316768765245, -123.458091875, 969.107214459},
   { 205.26656702, 0.351883757804, -63.2828017211, 805.439472446},
   { 165.054883837, 0.380076332139, -9.49826721492, 786.485529079},
   { 170.266407878, 0.376740190549, -26.1220182857, 793.501022351},
   { 174.942389695, 0.369884087515, -23.1535523964, 790.091948798},
   { 177.040548863, 0.37053981064, -21.5376985068, 802.675906343},
   { 179.158451289, 0.363484071131, -25.4670471271, 798.297493614},
   { 217.644080515, 0.337427012175, -80.9250841837, 804.87451828},
   { 200.00884363, 0.351459649622, -58.0621472919, 813.0190403},
   { 177.072565629, 0.381458438248, -35.4389804072, 809.303403743},
   { 154.48355251, 0.397938392875, -5.27745729092, 803.786078714},
   { 181.180524117, 0.377052646552, -42.9491231481, 804.751143136},
   { 179.922290367, 0.377573804949, -36.6812067183, 799.808215939},
   { 156.87254314, 0.394493696173, -4.51551563065, 798.339425399},
   { 151.292079917, 0.397855490788, 9.76663939023, 789.90780499},
   { 163.204445414, 0.388821432802, -11.0773177962, 802.694667013},
   { 173.853953198, 0.374509010705, -28.7533871044, 796.112705158},
   { 166.245694851, 0.378578937271, -10.8177598644, 803.636265648},
   { 180.648849808, 0.368646481969, -28.6674912001, 805.517342498},
   { 172.875373375, 0.37282778708, -22.8498090509, 801.731208624},
   { 177.742501626, 0.37168676535, -24.4594943612, 803.988227923},
   { 175.247610101, 0.372362228114, -22.9831331711, 810.647184084},
   { 194.738996151, 0.351179350824, -48.8215311637, 795.851832332},
   { 236.75522473, 0.335046904857, -111.355680166, 833.881377696},
   { 183.895677194, 0.366889019653, -41.5260982917, 803.503341796},
   { 0.00164697937929, 3.15844171046, 260.039749203, 821.474193679},
   { 160.764658215, 0.388824357623, 1.84770489945, 792.021665655},
   { 158.395399663, 0.390307193197, -5.16199052096, 809.807026773},
   { 165.609636718, 0.383916146302, -16.7594666857, 820.708342017},
   { 167.338853741, 0.38258356738, -19.6528304733, 796.702424146},
   { 160.258005363, 0.387776695805, -9.34268260334, 799.906668387},
   { 206.114061656, 0.352985889953, -66.3734033743, 808.176080149},
   { 204.615856312, 0.350583008594, -60.9696928381, 817.644899578},
   { 186.341472936, 0.366341602407, -42.378425975, 811.389348187},
   { 173.785756, 0.37560492961, -25.2262921046, 800.424066849},
   { 155.28136441, 0.393743281067, -0.749009316965, 808.556518512},
   { 167.675597251, 0.380384342943, -22.3966545677, 795.908394475},
   { 159.737149757, 0.384338481717, -6.16624128876, 803.937663197},
   { 174.720177907, 0.370491448146, -27.2984549894, 791.830145354},
   { 160.043470944, 0.387601584284, -9.48216917094, 810.52297246},
   { 145.879085343, 0.399812062665, 5.92951964582, 810.437796734},
   { 142.014382738, 0.404587015709, 10.0459115758, 810.107566921},
   { 154.737636481, 0.389462847777, -1.58188459128, 806.509867394},
   { 155.598147085, 0.396432501648, -7.99791697257, 828.466455177},
   { 152.41756307, 0.393331379269, -3.84940607367, 818.194505023},
   { 157.548465983, 0.391733602287, -4.24501053842, 813.618645752},
   { 149.785078562, 0.401341173665, 5.40859239166, 821.466928909},
   { 161.418983767, 0.388012209126, -15.6048793639, 830.519122002},
   { 179.205104233, 0.372710535906, -27.8929994047, 846.304698824},
   { 178.267751044, 0.379144427806, -36.7026620044, 833.903011472},
   { 166.050524207, 0.385074514677, -22.2878269157, 825.927231606},
   { 153.353415955, 0.391137331623, 4.35549118695, 826.173368307},
   { 166.377131168, 0.381903358112, -16.7080969433, 841.550446748},
   { 181.3393982, 0.369747809858, -32.8528555804, 819.865128353},
   { 168.992405398, 0.376917158774, -21.7419338897, 816.025783065},
   { 158.788499577, 0.388184915553, -1.74403757468, 804.045872522},
   { 163.247126505, 0.380767242861, -7.72976713531, 799.740423323},
   { 148.132250861, 0.398557547916, 4.63534886891, 806.030290274},
   { 161.947113938, 0.381759458251, -7.71856695561, 800.283197764},
   { 158.286281054, 0.38981423054, -3.92152995049, 805.651008261},
   { 140.636686493, 0.408584582691, 10.9639247007, 809.823894394},
   { 164.491862505, 0.386343264195, -19.3236756654, 804.325762898},
   { 157.930157409, 0.387805652557, -3.116070592, 797.282063511},
   { 152.046506736, 0.408611936109, 0.261639531676, 811.641072706},
   { 134.91929655, 0.42399671126, 19.124515027, 816.864201548},
   { 134.417437203, 0.424261785969, 21.0782066121, 825.533350165},
   { 151.793015421, 0.397593722777, 3.53757691785, 801.788844529},
   { 139.678730936, 0.409207614449, 17.862662717, 805.716176302},
   { 152.133123696, 0.392228842928, 3.70779341111, 805.629705123},
   { 145.210479362, 0.400986044732, 5.79489298357, 827.286737571},
   { 145.170894911, 0.396826359526, 12.9289527022, 823.150065646},
   { 152.663437928, 0.406464436582, 0.48606282857, 823.876396768},
   { 159.521746396, 0.394585012303, -12.6463356963, 838.724759761},
   { 134.441772891, 0.423311254851, 24.3612885506, 813.830774529},
   { 154.904892904, 0.395529504135, -0.933705599406, 800.932397946},
   { 155.316148875, 0.394305570818, -1.32213291215, 801.35433082},
   { 166.434387966, 0.380322864148, -16.3706385025, 799.584265001},
   { 160.658539769, 0.382500097101, -8.41626674401, 805.07626599},
   { 171.208006748, 0.37272907825, -20.3873755316, 796.768643679},
   { 163.820982874, 0.387820013492, -19.7120486548, 813.958157598},
   { 160.153564143, 0.389188817828, -8.53437904211, 817.110370308},
   { 155.637301337, 0.394637560177, -0.332509460462, 817.007260786},
   { 161.464348596, 0.388674294559, -11.1923892963, 811.476409549},
   { 187.264000508, 0.368306304312, -43.3170706071, 833.974682987},
   { 142.779464351, 0.40859543388, 7.45237456247, 808.153540447},
   { 149.194722797, 0.400409177241, 4.21493022178, 810.008952447},
   { 142.697077082, 0.407071769294, 12.0526292255, 808.795869611},
   { 167.099952173, 0.377652846278, -18.1580229788, 809.158030826},
   { 165.525647616, 0.381282164443, -12.6451076718, 802.098646739},
   { 166.200662923, 0.380456516136, -18.1160270347, 811.152886677},
   { 173.06332689, 0.371484971428, -25.6227376761, 807.44195702},
   { 147.478200995, 0.399966819535, 5.22063574812, 798.038897184},
   { 173.778576778, 0.373989152165, -21.4264125587, 803.17324781},
   { 153.007476162, 0.391985021999, 4.59787814073, 797.341782511},
   { 157.565920149, 0.385785187714, -1.9273996856, 795.045936427},
   { 157.319832076, 0.388390052626, -2.33072130855, 805.407402362},
   { 166.698413874, 0.380413833398, -14.6678455701, 807.84063909},
   { 166.076263607, 0.379429327982, -15.3700489771, 806.406038471},
   { 156.126898136, 0.387447842692, 0.499924745779, 800.954138677},
   { 156.134435682, 0.390196886882, -3.47615399281, 809.333583191},
   { 147.630547942, 0.393205841836, 10.4497898345, 803.621154241},
   { 138.344669886, 0.409049039005, 15.3164647109, 802.106983939},
   { 153.223919755, 0.390761049286, 0.288365946058, 802.624957682},
   { 187.622689817, 0.370828861605, -49.4231200996, 813.025454252},
   { 154.009419676, 0.406519266445, -9.49281103856, 825.642679987},
   { 171.560390967, 0.386848001046, -22.1073790583, 819.742278581},
   { 154.093480513, 0.400316610965, -4.72080384707, 818.366937647},
   { 142.562978068, 0.41055940205, 9.54092782204, 812.964938269},
   { 147.151801658, 0.401786755693, -0.382848488131, 804.524453706},
   { 165.231771952, 0.387612222061, -19.5743924511, 812.12972274},
   { 159.246794166, 0.386435198613, -10.5046481113, 803.322709122},
   { 167.809423234, 0.394301431766, -28.3721057069, 821.280781892},
   { 160.746442158, 0.40344596152, -10.9490283391, 828.796175806},
   { 158.870132082, 0.398962681374, -11.3864123427, 814.003833585},
   { 161.503102006, 0.395641482352, -13.1707368446, 807.541421413},
   { 154.676361026, 0.399228510558, -1.99770723383, 820.723794993},
   { 152.041659507, 0.401293584673, -3.84557481632, 802.123059808},
   { 166.733332031, 0.383252977343, -16.5105247945, 808.993644666},
   { 160.360044242, 0.382879155148, -5.90857049873, 800.452196898},
   { 168.049474101, 0.380841906715, -24.9856069738, 822.498677517},
   { 149.898475424, 0.395895648232, 1.55586003102, 812.917480203},
   { 166.680016116, 0.380271139297, -21.2634455088, 810.625194195},
   { 157.55037888, 0.391400271097, -8.12997109616, 814.138600527},
   { 138.468394356, 0.410656062473, 15.159441096, 809.770758813},
   { 137.003984414, 0.412527238061, 14.6405002091, 815.509446791},
   { 151.995105352, 0.397076433998, -0.00665891520887, 812.828323928},
   { 147.543861444, 0.398346837698, 1.35296963386, 809.51323192},
   { 144.590962294, 0.406578789077, 10.6842408021, 815.527802954},
   { 150.447737167, 0.400701966631, 1.8114139644, 814.608480665},
   { 146.849887813, 0.405042896626, -1.48084634061, 810.276465248},
   { 149.45801802, 0.402195569403, -0.812548028587, 810.220995408},
   { 162.645534881, 0.387946581054, -19.3530276044, 808.953062171},
   { 149.550710407, 0.404880873178, 3.30576433392, 811.887822352},
   { 161.591739054, 0.390125233635, -12.6651297051, 829.371851464},
   { 153.378613916, 0.395551228697, -1.91399000481, 807.65178703},
   { 150.449687679, 0.398142519114, 1.08924795506, 805.348967884},
   { 161.842617251, 0.388512426076, -15.0574884257, 808.601127919},
   { 177.531413759, 0.376608304213, -37.9899384766, 811.225154336},
   { 191.246451787, 0.364203962905, -54.4384823476, 818.325439012},
   { 163.437162871, 0.383994628257, -9.49267054306, 813.189709913},
   { 155.560508278, 0.393825064454, -1.86095104517, 814.154268632},
   { 153.597973809, 0.389575305449, 0.388744905254, 798.431051745},
   { 166.537441772, 0.373689007119, -7.54959941068, 801.047498191},
   { 120.629875529, 0.45458310438, -14.469277551, 825.620717357},
   { 143.563016281, 0.40837546661, 13.3717172543, 814.640631405},
   { 152.964273458, 0.39531322797, -3.83639097933, 804.989934236},
   { 140.475184551, 0.403702655843, 17.509200214, 809.412275926},
   { 145.107296943, 0.400227417211, 8.39187523106, 807.975693424},
   { 144.429802523, 0.4028463135, 10.0352312493, 802.119938488},
   { 155.50544345, 0.393176583583, 9.29693962082, 813.601879127},
   { NAN, NAN, NAN, NAN},
   { 120.092486687, 0.445165562142, 53.4077973359, 822.556728991},
   { NAN, NAN, NAN, NAN},
   { 121.044767147, 0.449991364524, 37.6389298231, 836.378840129},
   { 188.051484571, 0.36418524533, -42.0676605657, 810.999243506},
   { 140.199398594, 0.415334892195, 14.1338568082, 814.015439829},
   { 159.584076696, 0.38946767251, -12.7548614812, 808.779227319},
   { 167.895884926, 0.375899775166, -12.8228237184, 804.894334691},
   { 163.17801626, 0.380484663158, -13.2889264408, 808.990753082},
   { 147.641540284, 0.39998314732, 0.378709222101, 806.54303135},
   { 149.829270494, 0.395587838574, 3.03965642017, 800.256700204},
   { 142.03214514, 0.403328888898, 9.95182777765, 800.646596177},
   { 140.813704218, 0.407210760652, 15.9307757072, 801.256406137},
   { 127.265002662, 0.422359030983, 30.0157840252, 806.37344709},
   { 150.573969619, 0.391798106073, 3.82538855059, 798.334486716},
   { 139.137967535, 0.410431442811, 26.2466021724, 812.533034377},
   { 136.913499373, 0.41184325208, 27.202720603, 805.296381536},
   { 136.182535455, 0.424788649415, 21.0585117216, 818.933813096},
   { 160.847626214, 0.39744761875, -17.0678696209, 811.330248642},
   { 171.425414137, 0.377961949298, -21.4636481689, 809.737468118},
   { 170.89030615, 0.375330078541, -20.9610937811, 803.977210684},
   { 173.076109191, 0.373982343803, -25.9969280651, 803.308958199},
   { 164.184802358, 0.385854678015, -15.6822708836, 823.803990225},
   { 162.172247896, 0.382195050135, -11.7352231587, 807.461443632},
   { 155.45575927, 0.39003126393, -2.22368376564, 801.370788671},
   { 156.908224191, 0.389676243478, -2.83305694098, 808.064832377},
   { 152.884810169, 0.393274372866, 2.25053916462, 807.631032221},
   { 155.935812159, 0.387763364067, 2.27704870729, 797.56400445},
   { 150.815047118, 0.39482411929, 8.01631945307, 800.570270113},
   { 145.972722507, 0.395844068634, 8.59411964042, 807.187612142},
   { 146.90217627, 0.397663261402, 10.4805811403, 796.064808431},
   { 151.909688592, 0.391454570307, 3.99268380982, 804.44087044},
   { 156.644099831, 0.38201237914, -0.165061157687, 811.501809948},
   { 177.743491813, 0.373774849139, -36.2227581401, 804.748977933},
   { 156.141121307, 0.395554680508, -6.78685828123, 811.701552334},
   { 148.235983331, 0.404641489426, 0.318951714849, 816.516715374},
   { 152.679261621, 0.398372027589, -4.19676355252, 804.122669634},
   { 149.71002819, 0.400996621016, 7.51892257038, 818.439194645},
   { 153.443203493, 0.396217570373, -2.99910132647, 797.979524795},
   { 147.172287386, 0.403118928098, 9.65927682058, 815.061337839},
   { 136.490072161, 0.412744087365, 22.7189382219, 805.645710932},
   { 165.408131114, 0.392479409005, -18.7310336974, 809.493629015},
   { 161.471454371, 0.404447885724, -18.6414080791, 815.044061806},
   { 190.087750472, 0.370279959561, -50.174922144, 831.959002976},
   { 183.679062209, 0.378289195372, -46.2103446042, 826.74065135},
   { 157.013236626, 0.396193784181, -5.64959969938, 821.768216627},
   { 153.292742448, 0.408901916283, -6.38536958101, 820.721438328},
   { 152.652135885, 0.397581169947, -0.598675709241, 809.356655977},
   { 163.371310892, 0.384816053641, -15.4052366974, 809.294178737},
   { 169.164863724, 0.38506817031, -27.6696255991, 828.480404782},
   { 164.105049337, 0.387529989231, -20.8862595059, 800.345732808},
   { 200.525781306, 0.356077914471, -64.988724655, 816.029796923},
   { 194.301221972, 0.360929096777, -49.1612588036, 821.61236847},
   { 169.372422313, 0.385375349344, -16.7743371366, 809.788903141},
   { 165.38795851, 0.38602134172, -20.5144991185, 803.180398855},
   { 189.423653583, 0.371151999896, -51.3334638715, 804.877620511},
   { 182.902918468, 0.371525581416, -36.3437734478, 801.384615324},
   { 216.958127664, 0.345888460672, -76.3224010986, 817.812799662},
   { 196.298132441, 0.362648906788, -55.3303794712, 816.807390928},
   { 165.377379691, 0.392083398249, -12.5395507009, 818.920571514},
   { 166.244724944, 0.393290414647, -17.9060537593, 827.102892457},
   { 205.764004074, 0.354254273892, -63.9020978838, 809.938475237},
   { 198.221670325, 0.355933314869, -57.6219751246, 800.129636939},
   { 190.127317613, 0.366941612226, -48.9911581426, 805.874499077},
   { 180.366523261, 0.376889384123, -38.9018948039, 821.195172202},
   { 190.016548133, 0.359890966195, -47.7910196739, 801.129619435},
   { 198.122926402, 0.350662808673, -58.0480194403, 795.751439546},
   { 192.522844698, 0.357907795991, -48.0808642711, 806.898527474},
   { 191.560227364, 0.355927873161, -50.4632893833, 800.451800003},
   { 183.557838812, 0.363783904429, -44.0228272882, 792.845342575},
   { 201.131741324, 0.351993735263, -60.9832360509, 806.423562894},
   { 188.781299185, 0.356919843146, -42.0435101874, 798.997897823},
   { 244.53759532, 0.327380235415, -95.5947761975, 812.113559689},
   { 164.186145489, 0.394202669877, -14.3863304396, 813.468705441},
   { 162.482428374, 0.394501986636, -11.2286929405, 815.753842604},
   { 174.241250149, 0.373338924054, -25.7966481819, 805.020307652},
   { 178.139212198, 0.375234225128, -27.9636212344, 808.817134526},
   { 200.575601291, 0.352079310948, -63.9192577712, 801.889380524},
   { 183.418843233, 0.364694191474, -39.6957757461, 801.090048971},
   { 153.800760542, 0.391006661255, 3.47386622347, 803.984802711},
   { 167.508354594, 0.376526072972, -15.0756284564, 797.570120301},
   { 159.857659547, 0.396929162704, -16.0805290675, 813.59132667},
   { 181.409183284, 0.37617210442, -47.2016225476, 806.693147482},
   { 175.81635721, 0.371594973419, -28.1766358913, 802.843600908},
   { 176.027929725, 0.375688306523, -32.0054158413, 809.403917816},
   { 178.048203239, 0.370005895829, -30.5554096334, 803.029331876},
   { 187.807324653, 0.36286007042, -44.4283619806, 802.08707083},
   { 176.916510983, 0.365537898151, -27.4032128317, 803.086311969},
   { 175.201113299, 0.367198327203, -30.2908061677, 799.5579627},
   { 234.758566716, 0.325410215768, -100.049526353, 817.620645921},
   { 190.53213497, 0.359746526485, -43.2728446385, 816.35130114},
   { 197.085299488, 0.354082456248, -54.669106351, 798.497344816},
   { 212.692352972, 0.345070320168, -72.3245921035, 813.155463354},
   { 179.325825239, 0.370815757297, -34.4639295345, 810.992726009},
   { 242.328824231, 0.324771557363, -106.412182702, 814.132991146},
   { 167.457500275, 0.380894192711, -20.2137441411, 790.699073804},
   { 180.931280767, 0.367722372538, -34.5730851843, 789.793074344},
   { 174.263810769, 0.374983581763, -21.2345268666, 812.516089339},
   { 165.457807888, 0.380333173994, -15.0881252219, 800.035160318},
   { 163.356855767, 0.380779927455, -13.3885090639, 804.18456541},
   { 162.306684293, 0.382330682243, -11.7346032505, 802.842308199},
   { 166.312025432, 0.376970665696, -12.0868612575, 811.981522752},
   { 166.35873504, 0.376433552597, -11.2502212833, 808.086605168},
   { 153.804992882, 0.391240593134, 4.94326370793, 815.795304315},
   { 165.271075764, 0.380830602818, -13.7439296321, 797.514837974},
   { 171.88103736, 0.377066696401, -17.5432694467, 813.519345723},
   { 166.59558846, 0.38125210302, -16.1715688296, 796.194266495},
   { 176.281973799, 0.371535297, -26.6823062038, 811.271837209},
   { 168.950143003, 0.372570736551, -15.2738179539, 795.713970284},
   { 173.494681084, 0.371893212418, -20.3282342134, 799.908158223},
   { 181.181705543, 0.363019184587, -32.1647995962, 793.583417981},
   { 172.197115024, 0.373023819864, -22.3041299468, 788.195667719},
   { 167.282537206, 0.376859228619, -13.2075274402, 798.491438686},
   { 180.222023879, 0.378738293779, -33.3491825485, 853.319982367},
   { 171.495288574, 0.390371104543, -27.3971967327, 839.777403243},
   { 155.143145082, 0.39835510295, -7.53293237101, 816.699888836},
   { 206.330751683, 0.363862826025, -71.8603177083, 824.675229771},
   { 176.187628315, 0.376979609635, -29.1875732523, 813.358083382},
   { 176.312082794, 0.375291563363, -32.3981478889, 811.104207093},
   { 157.763024007, 0.388813412435, -4.97233762479, 799.267426335},
   { 174.915846956, 0.37341631416, -29.1304084922, 797.074313973},
   { 158.783599159, 0.401971792129, -5.03569122544, 864.282070847},
   { 181.308056786, 0.38459534553, -38.3824684437, 824.968227417},
   { 185.100965361, 0.369054173615, -48.3616260838, 814.606755092},
   { 153.858920065, 0.408470143554, -3.60735756547, 815.853176611},
   { 190.609476398, 0.357058818365, -48.8349527588, 793.838963486},
   { 179.08022453, 0.370606255, -33.7041482404, 804.018889644},
   { 182.63692134, 0.367748955915, -40.6269629177, 805.925731104},
   { 162.221889097, 0.386318063272, -14.8499219131, 797.189423786}
};

// ------------------------------------------------------------------------

class StateMachineRateControl : public StateMachineDim//, public DimInfoHandler
{
private:
    struct config
    {
        uint16_t fCalibrationType;
        uint16_t fTargetRate;
        uint16_t fMinThreshold;
        uint16_t fAverageTime;
        uint16_t fRequiredEvents;
    };

    map<string, config> fRunTypes;

    PixelMap fMap;

    struct state_control_t{
        bool fPhysTriggerEnabled;
        bool fTriggerOn;
    };
    state_control_t fStateControl;


    DimVersion fDim;

    vector<bool> has_this_FTU_been_mofified_this_time;
    vector<bool> should_this_FTU_be_ommited_next_time;


    DimDescribedState fDimFTM;
    DimDescribedState fDimRS;


    DimDescribedService fDimThreshold;

    float  fTargetRate;
    float  fTriggerRate;

    uint16_t fThresholdMin;
    uint16_t fThresholdReference;

    uint16_t fAverageTime;
    uint16_t fRequiredEvents;

    bool fVerbose;

    uint64_t fCounter;

    Time fCalibrationTimeStart;

    double self_patch_rate_median;
    double self_patch_rate_std;

    double self_board_rate_median;
    double self_board_rate_std;

    vector<uint32_t> fLastThresholdsReadFromFTM;
    vector<uint32_t> fLastThresholdsSetByUs;


    vector<double> fBiasPatchCurrents;  // currents

    bool CheckEventSize(const EventImp &evt, size_t size)
    {
        if (size_t(evt.GetSize())==size)
            return true;

        if (evt.GetSize()==0)
            return false;

        ostringstream msg;
        msg << evt.GetName() << " - Received event has " << evt.GetSize() << " bytes, but expected " << size << ".";
        Fatal(msg);
        return false;
    }


    int HandleStaticData(const EventImp &evt) {
        if (!CheckEventSize(evt, sizeof(FTM::DimStaticData))) return GetCurrentState();

        const FTM::DimStaticData &sdata = *static_cast<const FTM::DimStaticData*>(evt.GetData());

        fStateControl.fPhysTriggerEnabled = sdata.HasTrigger();
        fStateControl.fTriggerOn = (evt.GetQoS()&FTM::kFtmStates)==FTM::kFtmRunning;
        fLastThresholdsReadFromFTM.assign(sdata.fThreshold, sdata.fThreshold+160);

        return GetCurrentState();
    }

    int HandleTriggerRates(const EventImp &evt) {
        //const FTM::DimTriggerRates &sdata = *static_cast<const FTM::DimTriggerRates*>(evt.GetData());
        return GetCurrentState();
    }

    int HandleCalibratedCurrents(const EventImp &evt) {
        if (!CheckEventSize(evt, sizeof(Feedback::CalibratedCurrentsData))) return GetCurrentState();

        const Feedback::CalibratedCurrentsData &calibrated_currents = (
            *static_cast<const Feedback::CalibratedCurrentsData*>(evt.GetData()) );

        fBiasPatchCurrents = GetCalibratedCurrentsFromCalibratedCurrentsData(calibrated_currents);
        auto foo = CalcThresholdsFromCurrents(fBiasPatchCurrents);
        fLastThresholdsSetByUs = CombineThresholds(foo);

        if (GetCurrentState() == RateControl::State::kInProgress){
            SetThresholds(fLastThresholdsSetByUs);
        }

        fThresholds.assign(sdata.fThreshold, sdata.fThreshold+160);
        return GetCurrentState();
    }

    vector<double>
    GetCalibratedCurrentsFromCalibratedCurrentsData(const Feedback::CalibratedCurrentsData& currents){
        vector<double> tmp(currents.I, currents.I + BIAS::kNumChannels);
        return move(tmp);
    }

    vector<uint32_t>
    CalcThresholdsFromCurrents(const vector<double> currents){
        // b : BiasPatch ID
        vector<uint32_t> bias_patch_thresholds(320, 0);
        for(int b=0; b<currents.size(); b++){
            threshold_vs_current_fit_parameter fit = fits_parameters[b];
            bias_patch_thresholds[b] = uint32_t(fit.constant + fit.factor * pow(currents[b], fit.power));
        }
        return move(bias_patch_thresholds);
    }


    vector<uint32_t>
    CombineThresholds(const vector<uint32_t> bias_patch_thresholds){
        // t : TriggerPatch ID
        // b_4 : BiasPatch ID of associated bias patch with 4 pixel
        // b_5 : BiasPatch ID of associated bias patch with 5 pixel
        const int pixel_per_patch = 9;
        vector<uint32_t> trigger_patch_thresholds(160, 0);
        for(int t=0; t < trigger_patch_thresholds.size(); t++){
            const int b_4 = fMap.hw(t*pixel_per_patch).hv();
            const int b_5 = fMap.hw(t*pixel_per_patch + pixel_per_patch/2).hv();
            trigger_patch_thresholds[t] = max(b_4, b_5);
        }
        // exceptions for broken patches:
        // ---------------------------------------------------------------
        // We have 4 bias patches, which are broken, i.e. the current vs threshold
        // dependency was not fittable.
        // Luckily the neighboring bias patch works and can be used for setting
        // the threshold. In case there is a star in the broken patch, this patch
        // will fire like crazy since the threshold is derived from the neighboring
        // patch, which has a much lower current.
        trigger_patch_thresholds[19] = bias_patch_thresholds[39];  // 38 is dead
        trigger_patch_thresholds[33] = bias_patch_thresholds[67];  // 66 is crazy
        trigger_patch_thresholds[95] = bias_patch_thresholds[190];  // 191 is crazy
        trigger_patch_thresholds[96] = bias_patch_thresholds[192];  // 193 is crazy
        return move(trigger_patch_thresholds);
    }

    void SetThresholds(vector<uint32_t>& thresholds){
        if (fStateControl.fTriggerOn){
            Dim::SendCommandNB("FTM_CONTROL/SET_SELECTED_THRESHOLDS", thresholds);
        } else {
            Dim::SendCommandNB("FTM_CONTROL/SET_ALL_THRESHOLDS", thresholds);
        }
    }


    int CalibrateRun(const EventImp &evt)
    {
        Info("Starting to control thresholds.");
        return RateControl::State::kInProgress;
    }

    int StopRC()
    {
        Info("Stop received.");
        return RateControl::State::kConnected;
    }

    int SetMinThreshold(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 4))
            return kSM_FatalError;
        fThresholdReference = evt.GetUShort();
        return GetCurrentState();
    }

    int SetTargetRate(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 4))
            return kSM_FatalError;
        fTargetRate = evt.GetFloat();
        return GetCurrentState();
    }

    int Print() const
    {
        Out() << fDim << endl;
        Out() << fDimFTM << endl;
        Out() << fDimRS << endl;
        return GetCurrentState();
    }

    void PrintThresholds(vector<uint32_t> thresholds) {
        if (thresholds.empty() || thresholds.size() != 160)
            return;

        for (int j=0; j<10; j++) {
            for (int k=0; k<4; k++) {
                for (int i=0; i<4; i++) {
                    const int p = i + k*4 + j*16;
                        Out() << setw(3) << thresholds[p] << " ";
                }
                Out() << "   ";
            }
            Out() << endl;
        }
        Out() << endl;
    }


    int SetVerbosity(const EventImp &evt)
    {
        if (!CheckEventSize(evt, 1))
            return kSM_FatalError;
        fVerbose = evt.GetBool();
        return GetCurrentState();
    }

    int Execute()
    {
        if (!fDim.online())
            return RateControl::State::kDimNetworkNA;

        // All subsystems are not connected
        if ( fDimFTM.state() < FTM::State::kConnected )
        {
            return RateControl::State::kDisconnected;
        }

        // Do not allow any action while a ratescan is configured or in progress
        if (fDimRS.state()>=RateScan::State::kConfiguring)
            return RateControl::State::kConnected;

       return RateControl::State::kConnected;
    }

public:
    StateMachineRateControl(ostream &out=cout) : StateMachineDim(out, "RATE_CONTROL"),
        has_this_FTU_been_mofified_this_time(40),
        should_this_FTU_be_ommited_next_time(40),
        fDimFTM("FTM_CONTROL"),
        fDimRS("RATE_SCAN"),
        fDimThreshold("RATE_CONTROL/THRESHOLD", "S:1;D:1;D:1",
                      "Resulting threshold after calibration"
                      "|threshold[dac]:Resulting threshold from calibration"
                      "|begin[mjd]:Start time of calibration"
                      "|end[mjd]:End time of calibration")
    {
        fStateControl.fPhysTriggerEnabled = false;
        fStateControl.fTriggerOn = false;

        fDim.Subscribe(*this);
        fDimFTM.Subscribe(*this);
        fDimRS.Subscribe(*this);

        Subscribe("FTM_CONTROL/TRIGGER_RATES")
            (bind(&StateMachineRateControl::HandleTriggerRates, this, placeholders::_1));
        Subscribe("FTM_CONTROL/STATIC_DATA")
            (bind(&StateMachineRateControl::HandleStaticData,   this, placeholders::_1));
        Subscribe("FEEDBACK/CALIBRATED_CURRENTS")
            (bind(&StateMachineRateControl::HandleCalibratedCurrents, this, placeholders::_1));

        // State names
        AddStateName(RateControl::State::kDimNetworkNA, "DimNetworkNotAvailable",
                     "The Dim DNS is not reachable.");

        AddStateName(RateControl::State::kDisconnected, "Disconnected",
                     "The Dim DNS is reachable, but the required subsystems are not available.");

        AddStateName(RateControl::State::kConnected, "Connected",
                     "All needed subsystems are connected to their hardware, no action is performed.");

        AddStateName(RateControl::State::kSettingGlobalThreshold, "Calibrating",
                     "A global minimum threshold is currently determined.");

        AddStateName(RateControl::State::kGlobalThresholdSet, "GlobalThresholdSet",
                     "A global threshold has ben set, waiting for the trigger to be switched on.");

        AddStateName(RateControl::State::kInProgress, "InProgress",
                     "Rate control in progress.");

        AddEvent("CALIBRATE_RUN", "C")
            (bind(&StateMachineRateControl::CalibrateRun, this, placeholders::_1))
            ("Start a threshold calibration as defined in the setup for this run-type, state change to InProgress is delayed until trigger enabled");

        AddEvent("STOP", RateControl::State::kSettingGlobalThreshold, RateControl::State::kGlobalThresholdSet, RateControl::State::kInProgress)
            (bind(&StateMachineRateControl::StopRC, this))
            ("Stop a calibration or ratescan in progress");

        AddEvent("SET_MIN_THRESHOLD", "I:1")
            (bind(&StateMachineRateControl::SetMinThreshold, this, placeholders::_1))
            ("Set a minimum threshold at which th rate control starts calibrating");

        AddEvent("SET_TARGET_RATE", "F:1")
            (bind(&StateMachineRateControl::SetTargetRate, this, placeholders::_1))
            ("Set a target trigger rate for the calibration");

        AddEvent("PRINT")
            (bind(&StateMachineRateControl::Print, this))
            ("Print current status");

        AddEvent("SET_VERBOSE", "B")
            (bind(&StateMachineRateControl::SetVerbosity, this, placeholders::_1))
            ("set verbosity state"
             "|verbosity[bool]:disable or enable verbosity for received data (yes/no), except dynamic data");

    }

    bool GetConfig(Configuration &conf, const string &name, const string &sub, uint16_t &rc)
    {
        if (conf.HasDef(name, sub))
        {
            rc = conf.GetDef<uint16_t>(name, sub);
            return true;
        }

        Error("Neither "+name+"default nor "+name+sub+" found.");
        return false;
    }

    int EvalOptions(Configuration &conf)
    {
        fVerbose = !conf.Get<bool>("quiet");

        if (!fMap.Read(conf.Get<string>("pixel-map-file")))
        {
            Error("Reading mapping table from "+conf.Get<string>("pixel-map-file")+" failed.");
            return 1;
        }

        fThresholdReference = 300;
        fThresholdMin       = 300;
        fTargetRate         =  75;

        fAverageTime        =  10;
        fRequiredEvents     =   8;

        // ---------- Setup run types ---------
        const vector<string> types = conf.Vec<string>("run-type");
        if (types.empty())
            Warn("No run-types defined.");
        else
            Message("Defining run-types");

        for (auto it=types.begin(); it!=types.end(); it++)
        {
            Message(" -> "+ *it);

            if (fRunTypes.count(*it)>0)
            {
                Error("Run-type "+*it+" defined twice.");
                return 1;
            }

            config &c = fRunTypes[*it];
            if (!GetConfig(conf, "calibration-type.", *it, c.fCalibrationType) ||
                !GetConfig(conf, "target-rate.",      *it, c.fTargetRate)      ||
                !GetConfig(conf, "min-threshold.",    *it, c.fMinThreshold)    ||
                !GetConfig(conf, "average-time.",     *it, c.fAverageTime)     ||
                !GetConfig(conf, "required-events.",  *it, c.fRequiredEvents))
                return 2;
        }

        return -1;
    }
};

// ------------------------------------------------------------------------

#include "Main.h"

template<class T>
int RunShell(Configuration &conf)
{
    return Main::execute<T, StateMachineRateControl>(conf);
}

void SetupConfiguration(Configuration &conf)
{
    po::options_description control("Rate control options");
    control.add_options()
        ("quiet,q", po_bool(),  "Disable printing more informations during rate control.")
        ("pixel-map-file", var<string>()->required(), "Pixel mapping file. Used here to get the default reference voltage.")
        ;

    conf.AddOptions(control);

    po::options_description runtype("Run type configuration");
    runtype.add_options()
        ("run-type",           vars<string>(),  "Name of run-types (replace the * in the following configuration by the case-sensitive names defined here)")
        ("calibration-type.*", var<uint16_t>(), "Calibration type (0: none, 1: by rate, 2: by current)")
        ("target-rate.*",      var<uint16_t>(), "Target rate for calibration by rate")
        ("min-threshold.*",    var<uint16_t>(), "Minimum threshold which can be applied in a calibration")
        ("average-time.*",     var<uint16_t>(), "Time in seconds to average the currents for a calibration by current.")
        ("required-events.*",  var<uint16_t>(), "Number of required current events to start a calibration by current.");
    ;

    conf.AddOptions(runtype);
}

void PrintUsage() {
    cout <<
        "The ratecontrol program is a keep the rate reasonable low.\n"
        "\n"
        "Usage: ratecontrol [-c type] [OPTIONS]\n"
        "  or:  ratecontrol [OPTIONS]\n";
    cout << endl;
}

void PrintHelp() {
    Main::PrintHelp<StateMachineRateControl>();
}

int main(int argc, const char* argv[])
{
    Configuration conf(argv[0]);
    conf.SetPrintUsage(PrintUsage);
    Main::SetupConfiguration(conf);
    SetupConfiguration(conf);

    if (!conf.DoParse(argc, argv, PrintHelp))
        return 127;

    if (!conf.Has("console"))
        return RunShell<LocalStream>(conf);

    if (conf.Get<int>("console")==0)
        return RunShell<LocalShell>(conf);
    else
        return RunShell<LocalConsole>(conf);

    return 0;
}


