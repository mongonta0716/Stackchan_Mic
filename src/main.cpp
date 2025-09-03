// Copyright(c) 2023 Takao Akaki


#include <M5Unified.h>
#include <Avatar.h>
#include "fft.hpp"
#include <cinttypes>
#include <Stackchan_system_config.h>
#include <Stackchan_servo.h>
#include <SD.h>


#define BATTERT_CHK_INTERVAL 10000 // バッテリーチェックを行う間隔(msec)
uint32_t last_battery_chk_time = 0;
bool battery_chk_flag = false; // バッテリー表示をするかどうか
uint8_t move_mode = 0;   // 動作モード: 0:音がないときも動く。1:音があるときだけ動く。2:回転あり

#define USE_MIC

#ifdef USE_MIC
  // ---------- Mic sampling ----------

  #define READ_LEN    (2 * 256)
  #define LIPSYNC_LEVEL_MAX 10.0f

  int16_t *adcBuffer = NULL;
  static fft_t fft;
  static constexpr size_t WAVE_SIZE = 256 * 2;

  static constexpr const size_t record_samplerate = 16000; // M5StickCPlus2だと48KHzじゃないと動かなかったが、M5Unified0.1.12で修正されたのとM5AtomS2+PDFUnitで不具合が出たので戻した。。
  static int16_t *rec_data;
  
  // setupの最初の方の機種判別で書き換えている場合があります。そちらもチェックしてください。（マイクの性能が異なるため）
  uint8_t lipsync_shift_level = 11; // リップシンクのデータをどのくらい小さくするか設定。口の開き方が変わります。
  float lipsync_max =LIPSYNC_LEVEL_MAX;  // リップシンクの単位ここを増減すると口の開き方が変わります。

#endif

// M5GoBottomのLEDを使わない場合は下記の1行をコメントアウトしてください。
#define USE_LED

#ifdef USE_LED
  #include <Adafruit_NeoPixel.h>
  #define NUM_LEDS 10
  #define NUM_LEDS_HEX 55 
  #define LED_BRIGHTNESS 15
#if defined(ARDUINO_M5STACK_FIRE) || defined(ARDUINO_M5Stack_Core_ESP32)
  // M5Core1 + M5GoBottom1の組み合わせ
  #define LED_PIN 15
  #define LED_PIN_HEX 26
  uint8_t gHue = 0;
#else
  // M5Core2 + M5GoBottom2の組み合わせ
  #define LED_PIN 25
  #define LED_PIN_HEX 26
  uint8_t gHue = 0;
#endif
  // NeoPixel strips
  Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel strip_hex(NUM_LEDS_HEX, LED_PIN_HEX, NEO_GRB + NEO_KHZ800);
  uint8_t hue[5] = {0, 95, 160, 210, 45};

  static inline uint32_t colorHSV8(Adafruit_NeoPixel &s, uint8_t h, uint8_t s8 = 255, uint8_t v8 = 255) {
    // Map 0-255 HSV to NeoPixel's 0-65535, apply gamma correction
    return s.gamma32(Adafruit_NeoPixel::ColorHSV((uint16_t)h * 257, (uint16_t)s8 * 257, (uint16_t)v8 * 257));
  }

  static void fillRainbow(Adafruit_NeoPixel &s, uint8_t startHue, uint8_t delta = 7) {
    for (uint16_t i = 0; i < s.numPixels(); ++i) {
      uint8_t h = startHue + (uint8_t)(i * delta);
      s.setPixelColor(i, colorHSV8(s, h));
    }
  }

  void turn_off_led() {
    // Turn all LEDs off
    strip.clear();
    strip.show();
  }

  void clear_led_buff() {
    // Clear buffer for inner strip
    strip.clear();
  }

  void level_led(int level1, int level2) {  
  if(level1 > 5) level1 = 5;
  if(level2 > 5) level2 = 5;
    
    clear_led_buff();
    int showLevel = (level2 > 0) ? level2 : level1;
    if (showLevel > 0) {
      fillRainbow(strip, hue[showLevel - 1]);
    }
    strip.show();
  }
#endif


using namespace m5avatar;

Avatar avatar;
ColorPalette *cps[6];
uint8_t palette_index = 0;

float mouth_ratio = 0.0f;
uint32_t last_rotation_msec = 0;
uint32_t last_lipsync_max_msec = 0;

StackchanSERVO servo;
StackchanSystemConfig system_config;

void lipsync(void *args) {
  
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  size_t bytesread;
  uint64_t level = 0;
#ifndef SDL_h_
  while (avatar->isDrawing()) {
    level = 0;
    if ( M5.Mic.record(rec_data, WAVE_SIZE, record_samplerate)) {
      fft.exec(rec_data);
      for (size_t bx=5;bx<=60;++bx) {
        int32_t f = fft.get(bx);
        level += abs(f);
      }
    }
    uint32_t temp_level = level >> lipsync_shift_level;
    //M5_LOGI("level:%" PRId64 "\n", level) ;         // lipsync_maxを調整するときはこの行をコメントアウトしてください。
    //M5_LOGI("temp_level:%d\n", temp_level) ;         // lipsync_maxを調整するときはこの行をコメントアウトしてください。
    mouth_ratio = (float)(temp_level / lipsync_max);
    //M5_LOGI("ratio:%f\n", mouth_ratio);
    if (mouth_ratio <= 0.01f) {
      mouth_ratio = 0.0f;
      if ((lgfx::v1::millis() - last_lipsync_max_msec) > 500) {
        // 0.5秒以上無音の場合リップシンク上限をリセット
        last_lipsync_max_msec = lgfx::v1::millis();
        lipsync_max = LIPSYNC_LEVEL_MAX;
      }
    } else {
      if (mouth_ratio > 1.3f) {
        if (mouth_ratio > 1.5f) {
          // リップシンク上限を大幅に超えた場合、上限を上げていく。
          lipsync_max += 10.0f;
        }
        mouth_ratio = 1.3f;
      }
      last_lipsync_max_msec = lgfx::v1::millis(); // 無音でない場合は更新
    }

    if ((lgfx::v1::millis() - last_rotation_msec) > 350) {
      int direction = random(-2, 2);
      avatar->setRotation(direction * 10 * mouth_ratio);
      last_rotation_msec = lgfx::v1::millis();
    }
  #else
    float ratio = 0.0f;
  #endif
    avatar->setMouthOpenRatio(mouth_ratio);
#ifdef USE_LED
    fillRainbow(strip, gHue, 7);
    strip.show();
    fillRainbow(strip_hex, gHue, 7);
    strip_hex.show();
    {
      static uint32_t lastHueMs = 0;
      uint32_t now = lgfx::v1::millis();
      if (now - lastHueMs >= 20) { gHue = gHue + 10; lastHueMs = now; }
    }
#endif
    vTaskDelay(3/portTICK_PERIOD_MS);
    
  }  
}
void servoLoop(void *args) {
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  long move_time = 0;
  long interval_time = 0;
  long move_x = 0;
  long move_y = 0;
  float gaze_x = 0.0f;
  float gaze_y = 0.0f;
  bool sing_mode = false;
  while (avatar->isDrawing()) {
    if (mouth_ratio == 0.0f) {
      // 待機時の動き
      interval_time = random(system_config.getServoInterval(AvatarMode::NORMAL)->interval_min
                           , system_config.getServoInterval(AvatarMode::NORMAL)->interval_max);
      move_time = random(system_config.getServoInterval(AvatarMode::NORMAL)->move_min
                       , system_config.getServoInterval(AvatarMode::NORMAL)->move_max);
      sing_mode = false;

    } else {
      // 歌うモードの動き
      interval_time = random(system_config.getServoInterval(AvatarMode::SINGING)->interval_min
                           , system_config.getServoInterval(AvatarMode::SINGING)->interval_max);
      move_time = random(system_config.getServoInterval(AvatarMode::SINGING)->move_min
                       , system_config.getServoInterval(AvatarMode::SINGING)->move_max);
      sing_mode = true;
    } 
    avatar->getGaze(&gaze_y, &gaze_x);
    
//    Serial.printf("x:%f:y:%f\n", gaze_x, gaze_y);
    // X軸は90°から+-で左右にスイング
      switch(move_mode) {
      case 0:
        // 音がなっている時しか動かない。
        if (gaze_x < 0) {
          move_x = system_config.getServoInfo(AXIS_X)->start_degree - mouth_ratio * random(20,50);
        } else {
          move_x = system_config.getServoInfo(AXIS_X)->start_degree + mouth_ratio * random(20,50);
        }
        // Y軸は90°から上にスイング（最大35°）
        move_y = system_config.getServoInfo(AXIS_Y)->start_degree - mouth_ratio * 10 - abs(25.0 * gaze_y);
        servo.moveXY(move_x, move_y, move_time);
        break;
      case 1:
        // 音がないときも動く。
        if (gaze_x < 0) {
          move_x = system_config.getServoInfo(AXIS_X)->start_degree - mouth_ratio * 30 + (int)(15.0 * gaze_x);
        } else {
          move_x = system_config.getServoInfo(AXIS_X)->start_degree + mouth_ratio * 30 + (int)(15.0 * gaze_x);
        }
        move_y = system_config.getServoInfo(AXIS_Y)->start_degree - mouth_ratio * 10 - abs(25.0 * gaze_y);
        servo.moveXY(move_x, move_y, move_time);
        break;
      case 2:
        // 回転あり
        M5_LOGI("turn_mode");
        // 回転して動くモード
        // 音がなっている時しか動かない。
        move_x = abs(gaze_x * 500);
        //M5_LOGI("gaze_x: %2.2f\n", gaze_x);
        //M5_LOGI("move_x: %d\n", move_x);
        bool cw = true;
        if (gaze_x >= 0.0f) {
          cw = true;
        } else {
          cw = false;
        }
        // Y軸は90°から上にスイング（最大35°）
        move_y = system_config.getServoInfo(AXIS_Y)->start_degree - mouth_ratio * 10 - abs(25.0 * gaze_y);
        servo.turnX(move_x, cw, move_time);
        vTaskDelay(move_time/portTICK_PERIOD_MS);
        servo.moveY(move_y, move_time);
        vTaskDelay(move_time/portTICK_PERIOD_MS);
        break;
    }

    vTaskDelay(interval_time/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void setup()
{
  auto cfg = M5.config();
  cfg.internal_mic = true;
  cfg.output_power = true;
  M5.begin(cfg);
  M5.setTouchButtonHeight(40);
  M5.Log.setLogLevel(m5::log_target_display, ESP_LOG_NONE);
  M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_INFO);
  M5.Log.setEnableColor(m5::log_target_serial, false);
  M5_LOGI("Avatar Start");
  M5.Log.printf("M5.Log avatar Start\n");
  float scale = 0.0f;
  int8_t position_top = 0;
  int8_t position_left = 0;
  uint8_t display_rotation = 1; // ディスプレイの向き(0〜3)
  uint8_t first_cps = 0;
  auto mic_cfg = M5.Mic.config();
  switch (M5.getBoard()) {
    case m5::board_t::board_M5AtomS3:
      first_cps = 4;
      scale = 0.55f;
      position_top =  -60;
      position_left = -95;
      display_rotation = 2;
      // M5AtomS3は外部マイク(PDMUnit)なので設定を行う。
      mic_cfg.sample_rate = 16000;
      //mic_cfg.dma_buf_len = 256;
      //mic_cfg.dma_buf_count = 3;
      mic_cfg.pin_ws = 1;
      mic_cfg.pin_data_in = 2;
      M5.Mic.config(mic_cfg);
      break;

    case m5::board_t::board_M5StickC:
      first_cps = 1;
      scale = 0.6f;
      position_top = -80;
      position_left = -80;
      display_rotation = 3;
      break;

    case m5::board_t::board_M5StickCPlus:
      first_cps = 1;
      scale = 0.85f;
      position_top = -55;
      position_left = -35;
      display_rotation = 3;
      break;

    case m5::board_t::board_M5StickCPlus2:
      first_cps = 2;
      scale = 0.85f;
      position_top = -55;
      position_left = -35;
      display_rotation = 3;
      break;
   
     case m5::board_t::board_M5StackCore2:
      first_cps = 0;
      scale = 1.0f;
      position_top = 0;
      position_left = 0;
      display_rotation = 1;
      break;

    case m5::board_t::board_M5StackCoreS3:
    case m5::board_t::board_M5StackCoreS3SE:
      scale = 1.0f;
      position_top = 0;
      position_left = 0;
      display_rotation = 1;
      break;

    case m5::board_t::board_M5Stack:
      scale = 1.0f;
      position_top = 0;
      position_left = 0;
      display_rotation = 1;
      break;

    case m5::board_t::board_M5Dial:
      first_cps = 5;
      scale = 0.8f;
      position_top =  0;
      position_left = -40;
      display_rotation = 0;
      // M5ADial(StampS3)は外部マイク(PDMUnit)なので設定を行う。(Port.A)
      mic_cfg.pin_ws = 15;
      mic_cfg.pin_data_in = 13;
      M5.Mic.config(mic_cfg);
      break;

      
    defalut:
      M5.Log.println("Invalid board.");
      break;
  }
#ifndef SDL_h_
  rec_data = (typeof(rec_data))heap_caps_malloc(WAVE_SIZE * sizeof(int16_t), MALLOC_CAP_8BIT);
  memset(rec_data, 0 , WAVE_SIZE * sizeof(int16_t));
#endif
  M5.Speaker.end();
  if(!SD.begin(GPIO_NUM_4, SPI, 25000000)) {
    M5.Log.println("SD Card Mount Failed");
    delay(2000);
  };
  system_config.loadConfig(SD, "/yaml/SC_BasicConfig.yaml");
 
  if ((system_config.getServoInfo(AXIS_X)->pin == 21)
     || (system_config.getServoInfo(AXIS_X)->pin == 22)) {
    // Port.Aを利用する場合は、I2Cが使えないのでアイコンは表示しない。
    avatar.setBatteryIcon(false);
    battery_chk_flag = false;
    if (M5.getBoard() == m5::board_t::board_M5Stack) {
      M5.In_I2C.release();
    }
  } else {
    avatar.setBatteryIcon(true);
    avatar.setBatteryStatus(M5.Power.isCharging(), M5.Power.getBatteryLevel());
    last_battery_chk_time = millis();
    battery_chk_flag = true;
  }
 
  // servo
  servo.begin(system_config.getServoInfo(AXIS_X)->pin, system_config.getServoInfo(AXIS_X)->start_degree,
              system_config.getServoInfo(AXIS_X)->offset,
              system_config.getServoInfo(AXIS_Y)->pin, system_config.getServoInfo(AXIS_Y)->start_degree,
              system_config.getServoInfo(AXIS_Y)->offset,
              (ServoType)system_config.getServoType());
  delay(2000);
  M5.Power.setExtOutput(!system_config.getUseTakaoBase());
  
  servo_interval_s* servo_interval = system_config.getServoInterval(AvatarMode::NORMAL); // ノーマルモード時のサーボインターバル情報を取得
  servo_interval_s* servo_interval_sing = system_config.getServoInterval(AvatarMode::SINGING); // 歌っているときのサーボインターバル情報を取得

  M5.Display.setRotation(display_rotation);
  avatar.setScale(scale);
  avatar.setPosition(position_top, position_left);
  avatar.init(1); // start drawing
  cps[0] = new ColorPalette();
  cps[0]->set(COLOR_PRIMARY, TFT_BLACK);
  cps[0]->set(COLOR_BACKGROUND, TFT_YELLOW);
  cps[1] = new ColorPalette();
  cps[1]->set(COLOR_PRIMARY, TFT_BLACK);
  cps[1]->set(COLOR_BACKGROUND, TFT_ORANGE);
  cps[2] = new ColorPalette();
  cps[2]->set(COLOR_PRIMARY, (uint16_t)0x00ff00);
  cps[2]->set(COLOR_BACKGROUND, (uint16_t)0x303303);
  cps[3] = new ColorPalette();
  cps[3]->set(COLOR_PRIMARY, TFT_WHITE);
  cps[3]->set(COLOR_BACKGROUND, TFT_BLACK);
  cps[4] = new ColorPalette();
  cps[4]->set(COLOR_PRIMARY, TFT_BLACK);
  cps[4]->set(COLOR_BACKGROUND, TFT_WHITE);
  cps[5] = new ColorPalette();
  cps[5]->set(COLOR_PRIMARY, (uint16_t)0x000000);
  cps[5]->set(COLOR_BACKGROUND, 0x1a2bbd);//TFT_BLUE);
  avatar.setColorPalette(*cps[first_cps]);
#ifdef USE_LED
  strip.begin();
  strip_hex.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip_hex.setBrightness(LED_BRIGHTNESS);
  level_led(5, 5);
  delay(1000);
  turn_off_led();
#endif

  avatar.addTask(lipsync, "lipsync");
  avatar.addTask(servoLoop, "ServoLoop");
  last_rotation_msec = lgfx::v1::millis();
  
  // Initialize Serial2 for pins 13 (RX) and 14 (TX)
  Serial1.begin(115200, SERIAL_8N1, 13, 14);
  
  M5_LOGI("setup end");
}

uint32_t count = 0;

void loop()
{
  M5.update();

  if (M5.BtnA.wasPressed()) {
    M5_LOGI("Push BtnA");
    palette_index++;
    if (palette_index > 5) {
      palette_index = 0;
    }
    avatar.setColorPalette(*cps[palette_index]);
  }
  if (M5.BtnA.wasDoubleClicked()) {
    M5.Display.setRotation(3);
  }
  if (M5.BtnB.wasPressed()) {
    move_mode++;
    switch (move_mode) {
      case 0:
        avatar.setSpeechText("SilentMode");
        break;
      case 1:
        avatar.setSpeechText("HardMode");
        break;
      case 2:
        avatar.setSpeechText("Rotation");
        break;
      default:
        move_mode = 0;
        avatar.setSpeechText("SilentMode");
    }
    M5_LOGI("move_mode %d\n", move_mode);
  }
    if (M5.BtnC.wasPressed()) {
    M5_LOGI("Push BtnC");
    // Send character 'A' followed by newline over Serial2
    Serial1.print("A\n");
    Serial1.flush();
  }
  if (M5.BtnC.wasDoubleClicked()) {
    M5.Power.setExtOutput(!M5.Power.getExtOutput());
    if (M5.Power.getExtOutput()) {
      avatar.setSpeechText("ExtOutput");
    } else {
      avatar.setSpeechText("Battery  ");
    }
  }
  if (battery_chk_flag && ((millis() - last_battery_chk_time) > BATTERT_CHK_INTERVAL)) {
    // 一定時間ごとにバッテリー表示を更新する。
    avatar.setBatteryStatus(M5.Power.isCharging(), M5.Power.getBatteryLevel());
    last_battery_chk_time = millis();
    avatar.setSpeechText("");
   // PowerStatus status = checkTakaoBasePowerStatus(&M5.Power, 3200);
   // M5_LOGI("PowerStatus: %d\n", status);
  }
  lgfx::v1::delay(1);

}
