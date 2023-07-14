#include <M5Unified.h>

#ifdef __cplusplus
extern "C" {
#include <esp_agc.h>
}
#endif
#include <esp_ns.h>
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
    LOAD_CMD,
    COMP_CMD,
};

OpeMode mode_ = NON_OPE;
gob::UnifiedButton m5Button_;
cmdvox::MfccCommander commander_;
ns_handle_t ns_handle_;
using agc_handle_t = void*;
agc_handle_t agc_handle_;
std::string rootPath_;
std::string cmdName_;
std::string cmdPath_;
int16_t* raw_buffer_;
int sample_length_ = 0;

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
        M5.Display.clear();
        M5.Display.drawString(NameOf(REG_CMD1), 0, 0);
        cmdName_ = "CMD1";
        cmdPath_ = rootPath_ + "/command1.bin";
        break;
    case REG_CMD2:
        M5.Display.clear();
        M5.Display.drawString(NameOf(REG_CMD2), 0, 0);
        cmdName_ = "CMD2";
        cmdPath_ = rootPath_ + "/command2.bin";
        break;
    case REG_CMD3:
        M5.Display.clear();
        M5.Display.drawString(NameOf(REG_CMD3), 0, 0);
        cmdName_ = "CMD3";
        cmdPath_ = rootPath_ + "/command3.bin";
        break;
    case SAVE_CMD:
        M5.Display.drawString(NameOf(SAVE_CMD), 0, 0);
        break;
    case LOAD_CMD:
        M5.Display.drawString(NameOf(LOAD_CMD), 0, 0);
        break;
    case COMP_CMD:
        M5.Display.clear();
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
        else if (M5.BtnB.wasHold())
        {
            changeMode(LOAD_CMD);
        }
        else if (M5.BtnC.wasHold())
        {
            changeMode((mode_ == NON_OPE) ? COMP_CMD: NON_OPE);
        }
    }

}

/*
    Tips (1)
    As needed, apply pre-processing.
    As an example,
    Noise suppression is used to remove noise from the audio source,
    and Automatic gain control is used to adjust the input level.
*/
void preProcess(int16_t* data)
{
    ns_process(ns_handle_, data, data);
    esp_agc_process(agc_handle_, data, data, commander_.feed_length(), kSampleRate);
}

void setup()
{
    auto micConfig = M5.Mic.config();
    cmdvox::CommanderConfig cmdConfig;
    micConfig.sample_rate
    = cmdConfig.vad_config.sample_rate
    = cmdConfig.mfcc_config.sample_rate
    = kSampleRate;

    /*
        Tips (2)
        If you want to detect consecutive commands,
        it may be beneficial to adjust parameters like "hangover." 
    */
    // cmdConfig.vad_config.hangover_ms = 100;
    /*
        Tips (3)
        If there are many false positives in the voice segment,
        try adjusting vad_mode.
    */
    // cmdConfig.vad_config.vad_mode = simplevox::VadMode::Aggression_LV1;

    ns_handle_ = ns_pro_create(10, 1, kSampleRate);
    agc_handle_ = esp_agc_open(3, kSampleRate);

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
    M5.update();
    updateMode();
    m5Button_.draw();

    if (mode_ == NON_OPE)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else if (mode_ == SAVE_CMD)
    {
        commander_.saveSettings(rootPath_ + "/cmd_settings.json");
        changeMode(NON_OPE);
    }
    else if (mode_ == LOAD_CMD)
    {
        commander_.loadSettings(rootPath_ + "/cmd_settings.json");
        changeMode(NON_OPE);
    }
    else if (mode_ == COMP_CMD)
    {
        /*
            This is an example of displaying all commands
            that detected a voice section within 1000 (100 x 10) ms
        */
        static int compLife = -1;
        static std::vector<cmdvox::DetectResult> results;

        auto data = rxMic();
        preProcess(data);
        cmdvox::DetectResult result;
        if (commander_.detect(data, &result))
        {
            results.push_back(result);
            compLife = 100;
            std::string compStr;
            for (const auto& result: results)
            {
                compStr += result.command_name
                + "(" + std::to_string(result.score) + "), ";
            }
            M5.Display.drawString("                                            ", 0, 30);
            M5.Display.drawString(compStr.c_str(), 0, 30);
        }

        if (compLife > 0 && commander_.vad_state() < simplevox::VadState::Speech)
        {
            compLife--;
        }

        if (compLife == 0)
        {
            compLife = -1;
            results.clear();
        }
    }
    else
    {
        static int rec_count = 0;
        static std::unique_ptr<simplevox::MfccFeature> features[3];
        auto data = rxMic();
        preProcess(data);
        if (commander_.feedSample(data).can_fetch)
        {
            auto result = commander_.fetchFeature();
            features[rec_count++] = std::move(result.feature);
            M5.Mic.end();
            M5.Speaker.tone(800, 100);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            M5.Speaker.end();
            M5.Mic.begin();
        }

        /*
            The feature with the best (lowest) score among the three features is selected as the feature of the command.
            Threshold is also either calculated from averages or heuristic value.
        */
        if (rec_count == 3)
        {
            const auto dtw01 = simplevox::calcDTW(*features[0], *features[1]);
            const auto dtw02 = simplevox::calcDTW(*features[0], *features[2]);
            const auto dtw12 = simplevox::calcDTW(*features[1], *features[2]);
            uint32_t score[3] = { dtw01 + dtw02, dtw01 + dtw12, dtw02 + dtw12 };
            int index = (score[0] <= score[1] && score[0] <= score[2]) ? 0
                        : (score[1] <= score[0] && score[1] <= score[2]) ? 1
                        : 2;
            const auto mean = score[index] / 2;
            const auto threshold = std::max(static_cast<uint32_t>(mean * 1.2), 180u);
            
            commander_.saveFeature(cmdPath_.c_str(), *features[index]);
            commander_.add(cmdvox::MfccCommand{
                .info{
                    .name = cmdName_,
                    .id = 0,
                    .threshold = threshold,
                    .path = cmdPath_
                },
                .feature = std::move(features[index])
            });

            std::string regStr = cmdName_ + "(" + std::to_string(threshold) + ")";
            regStr += "[" + std::to_string(dtw01) + ", " + std::to_string(dtw02) + ", " + std::to_string(dtw12) + "]";
            M5.Display.drawString(regStr.c_str(), 0, 30);
            rec_count = 0;
            changeMode(NON_OPE);
        }
    }
}
