#define MIC_INPUT                   A2
#define RXLED                       17
#define TXLED                       30

#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.3
#define BASIS_DIM                   3

#define EUCLIDEAN_THRESHOLD         0.06
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/
float pca_vec1[80] = {-0.025578816848609586, -0.014281665153853651, -0.016973967225044517, -0.0020997384243730405, -0.029712880420177952, -0.20846717938228987, -0.30810499050182827, -0.27153323112702665, -0.3134817276434077, -0.2899924104938623, -0.3085839488296305, -0.2828035150902611, -0.22500910050270445, -0.22578143584365257, -0.17253920057489133, -0.11736753162905182, -0.05866399416692939, -0.003613239244397594, 0.058335524891591335, 0.10862511797273205, 0.13203055597581403, 0.13848043241261948, 0.14320874482271126, 0.15342705427444708, 0.156206103240067, 0.1374389355414044, 0.13782271683192346, 0.14199408101421032, 0.14124524677916053, 0.13945173674012953, 0.12844191296914592, 0.09473221513549095, 0.09028195534369067, 0.07271167122078212, 0.03107820533846081, 0.05120404391639553, 0.03176089195642469, 0.014548556250054617, 0.03024447673574052, 0.04168678686607375, 0.03218961530145304, 0.03153495890077865, 0.03161428270696649, 0.03695081770060045, 0.030896858508194302, 0.026964959287474048, 0.022316114305200013, 0.029070340488088126, 0.028015989116509, 0.020140203127547728, 0.01768318309072683, 0.012507374726117145, 0.013651387141274665, 0.007916597351345898, 0.007701069869823278, 0.008525985848184786, 0.005565371786084096, 0.012587107404399785, 0.009863613633235074, 0.006557473693315192, 0.014458932418857455, 0.024649806063113823, 0.011361644038863588, 0.02150071456640594, 0.023886389942905182, 0.022161841870179163, 0.026393858724979857, 0.01604646947059446, 0.013561570260231554, 0.006366504226776401, 0.025519513250202616, 0.02795595359741774, 0.005640373476372381, 0.00827837046097796, 0.012966337911864209, 0.011990079877771774, 0.009492103493661464, 0.010364281154448547, 0.009608234003451453, 0.005175324076528232};
float pca_vec2[80] = {0.034088978848992854, 0.04652719123076571, 0.008919134554541963, -0.002286656563517564, 0.01625806979509037, -0.021835744784651195, 0.1181783019224156, -0.031526808204862165, -0.013113235811261425, -0.03837769090319141, -0.028594145648908883, -0.06493346316121107, -0.10465251187882993, -0.1483225441312259, -0.16552194487042482, -0.1945051507807446, -0.23707584796408587, -0.2677301785248376, -0.3062444935959666, -0.2732530087794183, -0.26298657921140317, -0.21644532448774692, -0.16476538125947635, -0.11589384476136418, -0.1311915220426278, -0.10520243923793125, -0.09321086591299461, -0.10559878090409976, -0.08438885296524191, -0.07105632978262384, -0.004629113050640753, 0.06856397701459949, 0.09194906850925824, 0.08293917741171243, 0.12926012192135247, 0.19879523565214474, 0.19819606916342355, 0.23674908118209334, 0.20103635897546765, 0.17951428459758956, 0.15924991140696113, 0.1332455978549719, 0.11267205795421657, 0.08440455328822846, 0.07440687319714166, 0.05121442862645935, 0.044079176099194865, 0.04815633568060872, 0.057108961224643685, 0.04491564586505395, 0.039704497698736886, 0.0144262404373756, 0.01620366227029658, 0.011521865604150677, 0.011327929200434102, 0.019139945803638734, 0.019099585369979435, 0.015747975838051285, 0.01535858701148735, 0.015436788826694673, 0.02396810525131431, 0.06074843552769783, 0.026576951799997372, 0.056086842059337796, 0.05477240620929492, 0.05424949762502316, 0.07743809804021894, 0.04282739839415474, 0.02552536003231331, 0.009151223559870386, 0.06908969469610297, 0.07609798175747505, 0.01329554349233791, 0.005488872437314398, 0.019964721634258453, 0.012311778243989529, 0.010206491742714882, 0.00734229686946068, 0.006586621896645607, 0.00321846791199232};
float pca_vec3[80] = {0.01571722005788899, 0.027905094502950478, 0.032168352860412974, 0.012870092170820625, -0.016698689169654668, 0.045507459079610235, -0.15718527072265287, -0.19768333979778469, -0.20305710371335375, -0.16773033496121392, -0.12280731792143487, -0.02579731011287454, 0.04149294420944043, 0.008376983820982136, 0.04283115857263143, 0.1015846520620258, 0.11261735705428313, 0.14071233914004083, 0.0956943900777178, 0.03381791600966318, -0.002239886100024414, -0.041063106027202964, -0.07018243099226804, -0.05749429257049257, -0.13673998986696917, -0.1320465137204487, -0.15425949378141685, -0.1661087791581324, -0.1547633527157254, -0.18508990236798833, -0.2668418191030839, -0.31783070011580716, -0.27367600402542125, -0.24552047546765396, -0.2072109929233155, -0.15555057179759385, -0.05954002819683106, -0.042758534454310715, 0.03513094211284595, 0.06661806594807389, 0.08030602251916136, 0.08173821043298413, 0.08305912894574755, 0.06791003361034746, 0.06880731097882618, 0.05969288969386457, 0.061369403288185395, 0.04379948136101741, 0.05371113591255618, 0.059900387130579866, 0.04766200599144166, 0.045544881128211756, 0.04616084537039221, 0.036939423395038945, 0.03752644441812598, 0.031900514792795195, 0.03439876049140117, 0.05572788887816354, 0.05350021260841104, 0.05459308600071612, 0.06912993973742532, 0.1476767588770827, 0.05341061659091068, 0.14539973482690155, 0.14500063408139047, 0.13680187188115162, 0.10940410629258616, 0.058270732114212624, 0.05141164069268324, 0.03201211325636658, 0.13250486441395884, 0.1962442000199886, 0.03456590781826153, 0.037475534200525754, 0.07125002892725397, 0.05136680143111445, 0.04192208541600266, 0.04484466555266437, 0.046644285149527315, 0.037246683876291875};
float projected_mean_vec[3] = {-0.060391625268089374, -0.04651228035632966, -0.036081080757431425};
float centroid1[3] = {-0.05742803317191374, 0.022178334100538537, -0.006965791786171419};
float centroid2[3] = {-0.02824142660044888, -0.0052901993809622485, 0.026597024099954774};
float centroid3[3] = {0.040565649786755684, 0.003404009994011979, -0.01709168490459057};
float centroid4[3] = {0.04510380998560693, -0.020292144713588228, -0.0025395474091928455};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};
volatile int re_pointer = 0;

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
int write_arr = 0;

// this function allows us to get the correct array to save data to
// we are using a concept known as "pointer juggling" to save memory
int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  pinMode(MIC_INPUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(1000);

  re_pointer = 0;
      
  cli();
 
  //set timer1 interrupt at 1Hz * SAMPLING INTERVAL / 1000
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();

  Serial.begin(38400);
  delay(1000);
}


void loop(void) {
  if (re_pointer%AVG_SIZE == 0 && re_pointer <= SIZE){
    write_arr = !write_arr;
    envelope(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);
    
    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if(envelope(out, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: the principal components are unit norm
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += result[i]*pca_vec1[i];
        proj2 += result[i]*pca_vec2[i];
        proj3 += result[i]*pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];

      // Classification
      // Use the function 'l2_norm3' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;

      for(int i = 0; i < 4; i++) {
       float cur_val = l2_norm3(proj1, proj2, proj3, centroids[i]);
       if(cur_val < best_dist){
          best_dist = cur_val;
          best_index = i;
       }
      }


      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      String arg[4] = {"duck",  "cat", "mangp", "cow"};
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        Serial.println(best_index);
        Serial.println(arg[best_index]);
      } else {
        Serial.println("Above EUCLID Thresh");
        Serial.println(best_dist);
        
      }
      

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
   } else {
     Serial.println("Below LOUDNESS_THRESHOLD.");
   }

    delay(2000);
    re_pointer = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(RXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, LOW);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, LOW);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(TXLED, HIGH);
}

void envelope(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
  
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
  
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    Serial.println(maximum);
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres && block < SIZE_AFTER_FILTER);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}
/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/


ISR(TIMER1_COMPA_vect){//timer1 interrupt 8Khz toggles pin 13 (LED)
  if (re_pointer < SIZE) {
    digitalWrite(RXLED, LOW);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}
