#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <arduinoFFT.h>

#define TFT_CS     8
#define TFT_RST    21
#define TFT_DC     9
#define MIC_PIN    A3

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Объявления функций (прототипы)
void drawStaticFramework();
void drawFrequencyLabel(int band);
void sampleAudio();
void computeFFT();
void calculateBands();
void visualizeData();
uint16_t calculateColor(int height, int maxHeight);
void findPeakFrequency();

// Параметры FFT
const uint16_t samples = 128;
const double samplingFrequency = 22050;
const byte bands = 8;

double vReal[samples];
double vImag[samples];
int bandHeight[bands] = {0};
int prevBandHeight[bands] = {0};
float peakFrequency = 0;
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, samples, samplingFrequency);

int bandBounds[] = {2, 4, 8, 16, 24, 32, 40, 50, 60};
String freqLabels[bands];
const float dbScale[] = {50, 60, 70, 80, 90};
const int updateInterval = 10;

void setup() {
  Serial.begin(115200);
  tft.init(240, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  drawStaticFramework();
  
  for(int i=0; i<bands; i++){
    float centerFreq = ((bandBounds[i] + bandBounds[i+1])/2.0) * (samplingFrequency/2)/samples;
    freqLabels[i] = (centerFreq < 1000) ? 
      String(centerFreq, 0) + "Hz" : 
      String(centerFreq/1000, 1) + "kHz";
    drawFrequencyLabel(i);
  }
}

void loop() {
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate > updateInterval) {
    sampleAudio();
    computeFFT();
    calculateBands();
    visualizeData();
    
    Serial.print("Peak Frequency: ");
    Serial.print(peakFrequency);
    Serial.println(" Hz");
    
    lastUpdate = millis();
  }
}

void drawStaticFramework() {
  tft.drawRect(0, 0, tft.width(), tft.height()-25, ST77XX_WHITE);
  for(int i=0; i<5; i++){
    int yPos = map(dbScale[i], 50, 90, tft.height()-30, 10);
    tft.setCursor(5, yPos-6);
    tft.print(String(dbScale[i]) + "dB");
  }
}

void drawFrequencyLabel(int band) {
  int barWidth = tft.width() / bands;
  int xPos = band * barWidth + 4;
  tft.fillRect(xPos, tft.height()-20, barWidth-8, 16, ST77XX_BLACK);
  tft.setCursor(xPos+2, tft.height()-18);
  tft.print(freqLabels[band]);
}

void sampleAudio() {
  for(int i=0; i<samples; i++) {
    vReal[i] = analogRead(MIC_PIN) - 512;
    vImag[i] = 0;
    delayMicroseconds(1000000/samplingFrequency);
  }
}

void computeFFT() {
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  findPeakFrequency();
}

void calculateBands() {
  for(int i=0; i<bands; i++) {
    double sum = 0;
    int count = bandBounds[i+1] - bandBounds[i];
    
    for(int j=bandBounds[i]; j<bandBounds[i+1]; j++) {
      sum += vReal[j];
    }
    
    float avg = sum / count;
    float dbValue = 20 * log10(avg/2.0 + 1);
    
    int targetHeight = map(constrain(dbValue, 50, 90), 50, 90, 0, tft.height()-35);
    bandHeight[i] = 0.3 * targetHeight + 0.7 * prevBandHeight[i];
    prevBandHeight[i] = bandHeight[i];
  }
}

void visualizeData() {
  int barWidth = tft.width() / bands;
  for(int i = 0; i < bands; i++) {
    int xPos = i * barWidth + 4;
    int currentHeight = constrain(bandHeight[i], 0, tft.height()-35);

    drawFrequencyLabel(i);
    tft.fillRect(xPos, 10, barWidth-8, tft.height()-45, ST77XX_BLACK);

    for(int y = 0; y < currentHeight; y++) {
      uint16_t color = calculateColor(y, currentHeight);
      tft.drawFastHLine(xPos, tft.height()-35 - y, barWidth-8, color);
    }
  }
}


uint16_t calculateColor(int height, int maxHeight) {
  float ratio = (float)height / maxHeight;
  uint8_t green = 255 * ratio;
  uint8_t blue = 255 * (1 - ratio);
  return tft.color565(0, green, blue);
}

void findPeakFrequency() {
  double maxValue = 50;
  int peakBin = 0;
  
  for(int i = 1; i < samples/2; i++) {
    if(vReal[i] > maxValue) {
      maxValue = vReal[i];
      peakBin = i;
    }
  }
  
  peakFrequency = (maxValue > 50) ? (peakBin * samplingFrequency) / samples : 0;
}