#include <M5Unified.h>

#include <esp_log.h>
#include <SD.h>
#include <SPIFFS.h>

#include <gob_unifiedButton.hpp>
#include "cmdvox.h"

#define NameOf(x) #x

constexpr char TAG[] = "Main";
constexpr int kSampleRate = 16000;
constexpr int kSampleNum = 3;

enum OpeMode
{
    NON_OPE,
    LOAD_CMD,
    COMP_CMD,
};

OpeMode mode_ = NON_OPE;
gob::UnifiedButton m5Button_;
cmdvox::MfccCommander commander_;
std::string rootPath_;
std::string cmdName_;
std::string cmdPath_;
int16_t* raw_buffer_;
int sample_length_;

void abort()
{
    ESP_LOGE(TAG, "aborted");
    while(true) { vTaskDelay(500 / portTICK_PERIOD_MS); }
}

bool initMicBuffer(int length)
{
    sample_length_ = length;
    raw_buffer_ = (int16_t*)heap_caps_malloc(kSampleNum * sizeof(*raw_buffer_) * sample_length_, MALLOC_CAP_8BIT);
    return (raw_buffer_ != nullptr);
}

int16_t* rxMic()
{
    static int sample_index = 0;
    M5.Mic.record(&raw_buffer_[sample_length_ * sample_index++], sample_length_);
    if (sample_index >= kSampleNum) { sample_index = 0; }
    return &raw_buffer_[sample_length_ * sample_index];
}

void mountFs(bool is_sdcard)
{
    if (is_sdcard)
    {
        ESP_LOGI(TAG, "Mount /sd");
        rootPath_ = "/sd";
        if (!SD.begin(GPIO_NUM_4, SPI, 25000000, rootPath_.c_str()))
        {
            ESP_LOGE(TAG, "Failed to mount sd");
            abort();
        }
    }
    else
    {
        ESP_LOGI(TAG, "Mount /spiffs");
        rootPath_ = "/spiffs";
        if (!SPIFFS.begin(true, rootPath_.c_str()))
        {
            ESP_LOGE(TAG, "Failed to mount spiffs");
            abort();
        }
    }
}

void changeMode(OpeMode nextMode)
{
    switch (nextMode)
    {
    case LOAD_CMD:
        M5.Display.drawString(NameOf(LOAD_CMD), 0, 0);
        break;
    case COMP_CMD:
        M5.Display.drawString(NameOf(COMP_CMD), 0, 0);
        break;
    case NON_OPE:
        M5.Display.drawString("NON_OPE ", 0, 0);
        break;
    }
    mode_ = nextMode;
}

void updateMode()
{
    if (mode_ == NON_OPE || mode_ == COMP_CMD)
    {
        if (M5.BtnB.wasHold())
        {
            changeMode(LOAD_CMD);
        }
        else if (M5.BtnC.wasHold())
        {
            changeMode((mode_ == NON_OPE) ? COMP_CMD : NON_OPE);
        }
    }
}

void setup()
{
    auto micConfig = M5.Mic.config();
    cmdvox::CommanderConfig cmdConfig;
    micConfig.sample_rate
    = cmdConfig.vad_config.sample_rate
    = cmdConfig.mfcc_config.sample_rate
    = kSampleRate;

    if (!commander_.init(cmdConfig)) { abort(); }
    if (!initMicBuffer(commander_.feed_length())) { abort(); }

    M5.Mic.config(micConfig);
    M5.begin();
    M5.Mic.begin();
    m5Button_.begin(&M5.Display);

    mountFs(true);
}

void loop()
{
    m5Button_.update();
    m5Button_.draw();
    M5.update();

    updateMode();
    if (mode_ == NON_OPE)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else if (mode_ == LOAD_CMD)
    {
        commander_.loadSettings(rootPath_ + "/cmd_settings.json");
        changeMode(NON_OPE);
    }
    else if (mode_ == COMP_CMD)
    {
        auto data = rxMic();
        cmdvox::DetectResult result;
        if (commander_.detect(data, &result))
        {
            std::string str = result.command_name + ": " + std::to_string(result.score) + " ";
            M5.Display.drawString(str.c_str(), 0, 30);
        }
    }
}
