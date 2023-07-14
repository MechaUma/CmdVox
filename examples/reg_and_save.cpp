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
    REG_CMD1,
    REG_CMD2,
    REG_CMD3,
    SAVE_CMD,
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
    case REG_CMD1:
        M5.Display.drawString(NameOf(REG_CMD1), 0, 0);
        cmdName_ = "CMD1";
        cmdPath_ = rootPath_ + "/command1.bin";
        break;
    case REG_CMD2:
        M5.Display.drawString(NameOf(REG_CMD2), 0, 0);
        cmdName_ = "CMD2";
        cmdPath_ = rootPath_ + "/command2.bin";
        break;
    case REG_CMD3:
        M5.Display.drawString(NameOf(REG_CMD3), 0, 0);
        cmdName_ = "CMD3";
        cmdPath_ = rootPath_ + "/command3.bin";
        break;
    case SAVE_CMD:
        M5.Display.drawString(NameOf(SAVE_CMD), 0, 0);
        break;
    case NON_OPE:
        M5.Display.drawString("NON_OPE ", 0, 0);
        break;
    }
    mode_ = nextMode;
}

void updateMode()
{
    if (mode_ == NON_OPE)
    {
        if (M5.BtnA.wasClicked())
        {
            changeMode(REG_CMD1);
        }
        else if (M5.BtnB.wasClicked())
        {
            changeMode(REG_CMD2);
        }
        else if (M5.BtnC.wasClicked())
        {
            changeMode(REG_CMD3);
        }
        else if (M5.BtnA.wasHold())
        {
            changeMode(SAVE_CMD);
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
    else if (mode_ == SAVE_CMD)
    {
        commander_.saveSettings(rootPath_ + "/cmd_settings.json");
        changeMode(NON_OPE);
    }
    else
    {
        auto data = rxMic();
        if (commander_.feedSample(data).can_fetch)
        {
            auto result = commander_.fetchFeature();
            commander_.saveFeature(cmdPath_.c_str(), *result.feature);
            commander_.add(cmdvox::MfccCommand{
                .info{
                    .name = cmdName_,
                    .id = 0,
                    .threshold = 180,
                    .path = cmdPath_
                },
                .feature = std::move(result.feature)
            });
            changeMode(NON_OPE);
        }
    }
}
