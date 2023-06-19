///-----LIGHT SENSOR PINS | START-----///
#define F1 32  //A4
#define F2 35  //A5
#define F3 34  //A3
#define L 33  //soldered connection
#define R 39  //soldered connection
///-----LIGHT SENSOR PINS | END-----///

// Light filter variables
const int NUM_LIGHT_SAMPLES = 20;
int filter_iteration = 0;
// Light filter coefficients
double light_coefs[NUM_LIGHT_SAMPLES] = {-0.000594869140482306, -0.00213602857014467, -0.00403924830390829,
-0.00348986060828215, 0.00454211822285059, 0.0252122313979722, 0.0600918217747266, 0.104084101808472,
0.145614590831723, 0.171002965691906, 0.171002965691906, 0.145614590831723, 0.104084101808472,
0.0600918217747266, 0.0252122313979722, 0.00454211822285059, -0.00348986060828215, -0.00403924830390829,
-0.00213602857014467, -0.000594869140482306};
// Light filter samples
int fl_samples[NUM_LIGHT_SAMPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int fm_samples[NUM_LIGHT_SAMPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int fr_samples[NUM_LIGHT_SAMPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int l_samples[NUM_LIGHT_SAMPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int r_samples[NUM_LIGHT_SAMPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void add_sample(double sample, int sample_list[NUM_LIGHT_SAMPLES]) {
  if (filter_iteration < NUM_LIGHT_SAMPLES) {
    sample_list[filter_iteration] = sample;
  } else {
    for (int i = 1; i < NUM_LIGHT_SAMPLES; i++) {
      sample_list[i - 1] = sample_list[i];
    }
    sample_list[NUM_LIGHT_SAMPLES - 1] = sample;
  }
}

double filter(int sample_list[NUM_LIGHT_SAMPLES]) {
  double val = 0;
  for (int i = 0; i < NUM_LIGHT_SAMPLES; i++) {
    val += sample_list[i] * light_coefs[i];
  }
  return val;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {

  // Add light samples to memory.
  add_sample(analogRead(F1), fl_samples);
  add_sample(analogRead(F2), fm_samples);
  add_sample(analogRead(F3), fr_samples);
  add_sample(analogRead(L), l_samples);
  add_sample(analogRead(R), r_samples);

  // Filter and print light readings.
  Serial.print("F1: ");
  Serial.print(filter(fl_samples));
  Serial.print(" F2: ");
  Serial.print(filter(fm_samples));
  Serial.print(" F3: ");
  Serial.print(filter(fr_samples));
  Serial.print(" L: ");
  Serial.print(filter(l_samples));
  Serial.print(" R: ");
  Serial.println(filter(r_samples));
  
  if (filter_iteration < NUM_LIGHT_SAMPLES) {
    filter_iteration++;
  }
  delay(100);
}