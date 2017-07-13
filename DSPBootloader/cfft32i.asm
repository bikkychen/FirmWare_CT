;============================================================================
;
; File Name     : cfft_izc.asm
; 
; Originator    : Advanced Embeeded Control 
;                 Texas Instruments 
; 
; Description   : This file contains source code to zero the imaginary parts of 
;                 the complex input to the FFT module.
;               
; Date          : 25/2/2002 (dd/mm/yyyy)
;===========================================================================    
; Routine Type  : C Callable        
;
; Description   :
; void FFTC_izc(FFTxxxC_handle) 
; This function zeros the imaginary part of the complex input, in the case of 
; Complex FFT modules.
; 
;====================================================================== 
; COMPLEX FFT MODULES
;----------------------------------------------------------------------
;typedef struct {   
;        long *ipcbptr;
;        long *tfptr               
;        int size;
;        int nrstage;             
;        int *magptr;
;        int peakmag;
;        int peakfrq;
;        int normflag;
;        int *winptr; 
;        void (*init)(void);
;        void (*izero)(void *);
;        void (*calc)(void *);
;        void (*mag)(void *);
;        void (*win)(void *);
;        }CFFT32;
;======================================================================

            .include "sel_q.asm"
           
            .def   _CFFT32_init  
            
_CFFT32_init:            
; Twiddle factor Initialisation
            MOVL    XAR5,#TF_1024P_CFFT     ; 
            MOVL    *+XAR4[2],XAR5          ; tfptr->TF_1024P_CFFT
            LRETR
 

        .sect   "FFTtf" 

        .if(TF_QFMAT==Q31)
TF_1024P_CFFT:   
        .long   0,13176712,26352928,39528151,52701887,65873638,79042909,92209205,105372028,118530885
        .long   131685278,144834714,157978697,171116733,184248325,197372981,210490206,223599506,236700388,249792358
        .long   262874923,275947592,289009871,302061269,315101295,328129457,341145265,354148230,367137861,380113669
        .long   393075166,406021865,418953276,431868915,444768294,457650927,470516330,483364019,496193509,509004318
        .long   521795963,534567963,547319836,560051104,572761285,585449903,598116479,610760536,623381598,635979190
        .long   648552838,661102068,673626408,686125387,698598533,711045377,723465451,735858287,748223418,760560380
        .long   772868706,785147934,797397602,809617249,821806413,833964638,846091463,858186435,870249095,882278992
        .long   894275671,906238681,918167572,930061894,941921200,953745043,965532978,977284562,988999351,1000676905
        .long   1012316784,1023918550,1035481766,1047005996,1058490808,1069935768,1081340445,1092704411,1104027237,1115308496
        .long   1126547765,1137744621,1148898640,1160009405,1171076495,1182099496,1193077991,1204011567,1214899813,1225742318
        .long   1236538675,1247288478,1257991320,1268646800,1279254516,1289814068,1300325060,1310787095,1321199781,1331562723
        .long   1341875533,1352137822,1362349204,1372509294,1382617710,1392674072,1402678000,1412629117,1422527051,1432371426
        .long   1442161874,1451898025,1461579514,1471205974,1480777044,1490292364,1499751576,1509154322,1518500250,1527789007
        .long   1537020244,1546193612,1555308768,1564365367,1573363068,1582301533,1591180426,1599999411,1608758157,1617456335
        .long   1626093616,1634669676,1643184191,1651636841,1660027308,1668355276,1676620432,1684822463,1692961062,1701035922
        .long   1709046739,1716993211,1724875040,1732691928,1740443581,1748129707,1755750017,1763304224,1770792044,1778213194
        .long   1785567396,1792854372,1800073849,1807225553,1814309216,1821324572,1828271356,1835149306,1841958164,1848697674
        .long   1855367581,1861967634,1868497586,1874957189,1881346202,1887664383,1893911494,1900087301,1906191570,1912224073
        .long   1918184581,1924072871,1929888720,1935631910,1941302225,1946899451,1952423377,1957873796,1963250501,1968553292
        .long   1973781967,1978936331,1984016189,1989021350,1993951625,1998806829,2003586779,2008291295,2012920201,2017473321
        .long   2021950484,2026351522,2030676269,2034924562,2039096241,2043191150,2047209133,2051150040,2055013723,2058800036
        .long   2062508835,2066139983,2069693342,2073168777,2076566160,2079885360,2083126254,2086288720,2089372638,2092377892
        .long   2095304370,2098151960,2100920556,2103610054,2106220352,2108751352,2111202959,2113575080,2115867626,2118080511
        .long   2120213651,2122266967,2124240380,2126133817,2127947206,2129680480,2131333572,2132906420,2134398966,2135811153
        .long   2137142927,2138394240,2139565043,2140655293,2141664948,2142593971,2143442326,2144209982,2144896910,2145503083
        .long   2146028480,2146473080,2146836866,2147119825,2147321946,2147443222,2147483647,2147443222,2147321946,2147119825
        .long   2146836866,2146473080,2146028480,2145503083,2144896910,2144209982,2143442326,2142593971,2141664948,2140655293
        .long   2139565043,2138394240,2137142927,2135811153,2134398966,2132906420,2131333572,2129680480,2127947206,2126133817
        .long   2124240380,2122266967,2120213651,2118080511,2115867626,2113575080,2111202959,2108751352,2106220352,2103610054
        .long   2100920556,2098151960,2095304370,2092377892,2089372638,2086288720,2083126254,2079885360,2076566160,2073168777
        .long   2069693342,2066139983,2062508835,2058800036,2055013723,2051150040,2047209133,2043191150,2039096241,2034924562
        .long   2030676269,2026351522,2021950484,2017473321,2012920201,2008291295,2003586779,1998806829,1993951625,1989021350
        .long   1984016189,1978936331,1973781967,1968553292,1963250501,1957873796,1952423377,1946899451,1941302225,1935631910
        .long   1929888720,1924072871,1918184581,1912224073,1906191570,1900087301,1893911494,1887664383,1881346202,1874957189
        .long   1868497586,1861967634,1855367581,1848697674,1841958164,1835149306,1828271356,1821324572,1814309216,1807225553
        .long   1800073849,1792854372,1785567396,1778213194,1770792044,1763304224,1755750017,1748129707,1740443581,1732691928
        .long   1724875040,1716993211,1709046739,1701035922,1692961062,1684822463,1676620432,1668355276,1660027308,1651636841
        .long   1643184191,1634669676,1626093616,1617456335,1608758157,1599999411,1591180426,1582301533,1573363068,1564365367
        .long   1555308768,1546193612,1537020244,1527789007,1518500250,1509154322,1499751576,1490292364,1480777044,1471205974
        .long   1461579514,1451898025,1442161874,1432371426,1422527051,1412629117,1402678000,1392674072,1382617710,1372509294
        .long   1362349204,1352137822,1341875533,1331562723,1321199781,1310787095,1300325060,1289814068,1279254516,1268646800
        .long   1257991320,1247288478,1236538675,1225742318,1214899813,1204011567,1193077991,1182099496,1171076495,1160009405
        .long   1148898640,1137744621,1126547765,1115308496,1104027237,1092704411,1081340445,1069935768,1058490808,1047005996
        .long   1035481766,1023918550,1012316784,1000676905,988999351,977284562,965532978,953745043,941921200,930061894
        .long   918167572,906238681,894275671,882278992,870249095,858186435,846091463,833964638,821806413,809617249
        .long   797397602,785147934,772868706,760560380,748223418,735858287,723465451,711045377,698598533,686125387
        .long   673626408,661102068,648552838,635979190,623381598,610760536,598116479,585449903,572761285,560051104
        .long   547319836,534567963,521795963,509004318,496193509,483364019,470516330,457650927,444768294,431868915
        .long   418953276,406021865,393075166,380113669,367137861,354148230,341145265,328129457,315101295,302061269
        .long   289009871,275947592,262874923,249792358,236700388,223599506,210490206,197372981,184248325,171116733
        .long   157978697,144834714,131685278,118530885,105372028,92209205,79042909,65873638,52701887,39528151
        .long   26352928,13176712,0,-13176712,-26352928,-39528151,-52701887,-65873638,-79042909,-92209205
        .long   -105372028,-118530885,-131685278,-144834714,-157978697,-171116733,-184248325,-197372981,-210490206,-223599506
        .long   -236700388,-249792358,-262874923,-275947592,-289009871,-302061269,-315101295,-328129457,-341145265,-354148230
        .long   -367137861,-380113669,-393075166,-406021865,-418953276,-431868915,-444768294,-457650927,-470516330,-483364019
        .long   -496193509,-509004318,-521795963,-534567963,-547319836,-560051104,-572761285,-585449903,-598116479,-610760536
        .long   -623381598,-635979190,-648552838,-661102068,-673626408,-686125387,-698598533,-711045377,-723465451,-735858287
        .long   -748223418,-760560380,-772868706,-785147934,-797397602,-809617249,-821806413,-833964638,-846091463,-858186435
        .long   -870249095,-882278992,-894275671,-906238681,-918167572,-930061894,-941921200,-953745043,-965532978,-977284562
        .long   -988999351,-1000676905,-1012316784,-1023918550,-1035481766,-1047005996,-1058490808,-1069935768,-1081340445,-1092704411
        .long   -1104027237,-1115308496,-1126547765,-1137744621,-1148898640,-1160009405,-1171076495,-1182099496,-1193077991,-1204011567
        .long   -1214899813,-1225742318,-1236538675,-1247288478,-1257991320,-1268646800,-1279254516,-1289814068,-1300325060,-1310787095
        .long   -1321199781,-1331562723,-1341875533,-1352137822,-1362349204,-1372509294,-1382617710,-1392674072,-1402678000,-1412629117
        .long   -1422527051,-1432371426,-1442161874,-1451898025,-1461579514,-1471205974,-1480777044,-1490292364,-1499751576,-1509154322
        .long   -1518500250,-1527789007,-1537020244,-1546193612,-1555308768,-1564365367,-1573363068,-1582301533,-1591180426,-1599999411
        .long   -1608758157,-1617456335,-1626093616,-1634669676,-1643184191,-1651636841,-1660027308,-1668355276,-1676620432,-1684822463
        .long   -1692961062,-1701035922,-1709046739,-1716993211,-1724875040,-1732691928,-1740443581,-1748129707,-1755750017,-1763304224
        .long   -1770792044,-1778213194,-1785567396,-1792854372,-1800073849,-1807225553,-1814309216,-1821324572,-1828271356,-1835149306
        .long   -1841958164,-1848697674,-1855367581,-1861967634,-1868497586,-1874957189,-1881346202,-1887664383,-1893911494,-1900087301
        .long   -1906191570,-1912224073,-1918184581,-1924072871,-1929888720,-1935631910,-1941302225,-1946899451,-1952423377,-1957873796
        .long   -1963250501,-1968553292,-1973781967,-1978936331,-1984016189,-1989021350,-1993951625,-1998806829,-2003586779,-2008291295
        .long   -2012920201,-2017473321,-2021950484,-2026351522,-2030676269,-2034924562,-2039096241,-2043191150,-2047209133,-2051150040
        .long   -2055013723,-2058800036,-2062508835,-2066139983,-2069693342,-2073168777,-2076566160,-2079885360,-2083126254,-2086288720
        .long   -2089372638,-2092377892,-2095304370,-2098151960,-2100920556,-2103610054,-2106220352,-2108751352,-2111202959,-2113575080
        .long   -2115867626,-2118080511,-2120213651,-2122266967,-2124240380,-2126133817,-2127947206,-2129680480,-2131333572,-2132906420
        .long   -2134398966,-2135811153,-2137142927,-2138394240,-2139565043,-2140655293,-2141664948,-2142593971,-2143442326,-2144209982
        .long   -2144896910,-2145503083,-2146028480,-2146473080,-2146836866,-2147119825,-2147321946,-2147443222

        .endif
    
        .if(TF_QFMAT==Q30)
TF_1024P_CFFT
        .long   0,6588356,13176464,19764076,26350943,32936819,39521455,46104602,52686014,59265442
        .long   65842639,72417357,78989349,85558366,92124163,98686491,105245103,111799753,118350194,124896179
        .long   131437462,137973796,144504935,151030634,157550647,164064728,170572633,177074115,183568930,190056834
        .long   196537583,203010932,209476638,215934457,222384147,228825464,235258165,241682010,248096755,254502159
        .long   260897982,267283981,273659918,280025552,286380643,292724951,299058239,305380268,311690799,317989595
        .long   324276419,330551034,336813204,343062693,349299266,355522689,361732726,367929144,374111709,380280190
        .long   386434353,392573967,398698801,404808624,410903207,416982319,423045732,429093217,435124548,441139496
        .long   447137835,453119340,459083786,465030947,470960600,476872522,482766489,488642281,494499676,500338453
        .long   506158392,511959275,517740883,523502998,529245404,534967884,540670223,546352205,552013618,557654248
        .long   563273883,568872310,574449320,580004702,585538248,591049748,596538995,602005783,607449906,612871159
        .long   618269338,623644239,628995660,634323400,639627258,644907034,650162530,655393548,660599890,665781362
        .long   670937767,676068911,681174602,686254647,691308855,696337036,701339000,706314559,711263525,716185713
        .long   721080937,725949013,730789757,735602987,740388522,745146182,749875788,754577161,759250125,763894504
        .long   768510122,773096806,777654384,782182683,786681534,791150767,795590213,799999706,804379079,808728167
        .long   813046808,817334838,821592095,825818421,830013654,834177638,838310216,842411232,846480531,850517961
        .long   854523370,858496606,862437520,866345964,870221790,874064853,877875009,881652112,885396022,889106597
        .long   892783698,896427186,900036924,903612776,907154608,910662286,914135678,917574653,920979082,924348837
        .long   927683790,930983817,934248793,937478595,940673101,943832191,946955747,950043650,953095785,956112036
        .long   959092290,962036435,964944360,967815955,970651112,973449725,976211688,978936898,981625251,984276646
        .long   986890984,989468165,992008094,994510675,996975812,999403415,1001793390,1004145648,1006460100,1008736660
        .long   1010975242,1013175761,1015338134,1017462281,1019548121,1021595575,1023604567,1025575020,1027506862,1029400018
        .long   1031254418,1033069992,1034846671,1036584389,1038283080,1039942680,1041563127,1043144360,1044686319,1046188946
        .long   1047652185,1049075980,1050460278,1051805027,1053110176,1054375676,1055601479,1056787540,1057933813,1059040255
        .long   1060106826,1061133483,1062120190,1063066909,1063973603,1064840240,1065666786,1066453210,1067199483,1067905576
        .long   1068571464,1069197120,1069782521,1070327646,1070832474,1071296985,1071721163,1072104991,1072448455,1072751542
        .long   1073014240,1073236540,1073418433,1073559913,1073660973,1073721611,1073741824,1073721611,1073660973,1073559913
        .long   1073418433,1073236540,1073014240,1072751542,1072448455,1072104991,1071721163,1071296985,1070832474,1070327646
        .long   1069782521,1069197120,1068571464,1067905576,1067199483,1066453210,1065666786,1064840240,1063973603,1063066909
        .long   1062120190,1061133483,1060106826,1059040255,1057933813,1056787540,1055601479,1054375676,1053110176,1051805027
        .long   1050460278,1049075980,1047652185,1046188946,1044686319,1043144360,1041563127,1039942680,1038283080,1036584389
        .long   1034846671,1033069992,1031254418,1029400018,1027506862,1025575020,1023604567,1021595575,1019548121,1017462281
        .long   1015338134,1013175761,1010975242,1008736660,1006460100,1004145648,1001793390,999403415,996975812,994510675
        .long   992008094,989468165,986890984,984276646,981625251,978936898,976211688,973449725,970651112,967815955
        .long   964944360,962036435,959092290,956112036,953095785,950043650,946955747,943832191,940673101,937478595
        .long   934248793,930983817,927683790,924348837,920979082,917574653,914135678,910662286,907154608,903612776
        .long   900036924,896427186,892783698,889106597,885396022,881652112,877875009,874064853,870221790,866345964
        .long   862437520,858496606,854523370,850517961,846480531,842411232,838310216,834177638,830013654,825818421
        .long   821592095,817334838,813046808,808728167,804379079,799999706,795590213,791150767,786681534,782182683
        .long   777654384,773096806,768510122,763894504,759250125,754577161,749875788,745146182,740388522,735602987
        .long   730789757,725949013,721080937,716185713,711263525,706314559,701339000,696337036,691308855,686254647
        .long   681174602,676068911,670937767,665781362,660599890,655393548,650162530,644907034,639627258,634323400
        .long   628995660,623644239,618269338,612871159,607449906,602005783,596538995,591049748,585538248,580004702
        .long   574449320,568872310,563273883,557654248,552013618,546352205,540670223,534967884,529245404,523502998
        .long   517740883,511959275,506158392,500338453,494499676,488642281,482766489,476872522,470960600,465030947
        .long   459083786,453119340,447137835,441139496,435124548,429093217,423045732,416982319,410903207,404808624
        .long   398698801,392573967,386434353,380280190,374111709,367929144,361732726,355522689,349299266,343062693
        .long   336813204,330551034,324276419,317989595,311690799,305380268,299058239,292724951,286380643,280025552
        .long   273659918,267283981,260897982,254502159,248096755,241682010,235258165,228825464,222384147,215934457
        .long   209476638,203010932,196537583,190056834,183568930,177074115,170572633,164064728,157550647,151030634
        .long   144504935,137973796,131437462,124896179,118350194,111799753,105245103,98686491,92124163,85558366
        .long   78989349,72417357,65842639,59265442,52686014,46104602,39521455,32936819,26350943,19764076
        .long   13176464,6588356,0,-6588356,-13176464,-19764076,-26350943,-32936819,-39521455,-46104602
        .long   -52686014,-59265442,-65842639,-72417357,-78989349,-85558366,-92124163,-98686491,-105245103,-111799753
        .long   -118350194,-124896179,-131437462,-137973796,-144504935,-151030634,-157550647,-164064728,-170572633,-177074115
        .long   -183568930,-190056834,-196537583,-203010932,-209476638,-215934457,-222384147,-228825464,-235258165,-241682010
        .long   -248096755,-254502159,-260897982,-267283981,-273659918,-280025552,-286380643,-292724951,-299058239,-305380268
        .long   -311690799,-317989595,-324276419,-330551034,-336813204,-343062693,-349299266,-355522689,-361732726,-367929144
        .long   -374111709,-380280190,-386434353,-392573967,-398698801,-404808624,-410903207,-416982319,-423045732,-429093217
        .long   -435124548,-441139496,-447137835,-453119340,-459083786,-465030947,-470960600,-476872522,-482766489,-488642281
        .long   -494499676,-500338453,-506158392,-511959275,-517740883,-523502998,-529245404,-534967884,-540670223,-546352205
        .long   -552013618,-557654248,-563273883,-568872310,-574449320,-580004702,-585538248,-591049748,-596538995,-602005783
        .long   -607449906,-612871159,-618269338,-623644239,-628995660,-634323400,-639627258,-644907034,-650162530,-655393548
        .long   -660599890,-665781362,-670937767,-676068911,-681174602,-686254647,-691308855,-696337036,-701339000,-706314559
        .long   -711263525,-716185713,-721080937,-725949013,-730789757,-735602987,-740388522,-745146182,-749875788,-754577161
        .long   -759250125,-763894504,-768510122,-773096806,-777654384,-782182683,-786681534,-791150767,-795590213,-799999706
        .long   -804379079,-808728167,-813046808,-817334838,-821592095,-825818421,-830013654,-834177638,-838310216,-842411232
        .long   -846480531,-850517961,-854523370,-858496606,-862437520,-866345964,-870221790,-874064853,-877875009,-881652112
        .long   -885396022,-889106597,-892783698,-896427186,-900036924,-903612776,-907154608,-910662286,-914135678,-917574653
        .long   -920979082,-924348837,-927683790,-930983817,-934248793,-937478595,-940673101,-943832191,-946955747,-950043650
        .long   -953095785,-956112036,-959092290,-962036435,-964944360,-967815955,-970651112,-973449725,-976211688,-978936898
        .long   -981625251,-984276646,-986890984,-989468165,-992008094,-994510675,-996975812,-999403415,-1001793390,-1004145648
        .long   -1006460100,-1008736660,-1010975242,-1013175761,-1015338134,-1017462281,-1019548121,-1021595575,-1023604567,-1025575020
        .long   -1027506862,-1029400018,-1031254418,-1033069992,-1034846671,-1036584389,-1038283080,-1039942680,-1041563127,-1043144360
        .long   -1044686319,-1046188946,-1047652185,-1049075980,-1050460278,-1051805027,-1053110176,-1054375676,-1055601479,-1056787540
        .long   -1057933813,-1059040255,-1060106826,-1061133483,-1062120190,-1063066909,-1063973603,-1064840240,-1065666786,-1066453210
        .long   -1067199483,-1067905576,-1068571464,-1069197120,-1069782521,-1070327646,-1070832474,-1071296985,-1071721163,-1072104991
        .long   -1072448455,-1072751542,-1073014240,-1073236540,-1073418433,-1073559913,-1073660973,-1073721611

        .endif   