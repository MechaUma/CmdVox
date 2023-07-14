/*!
 * CmdVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#include "cmdvox.h"

#include <algorithm>
#include <stdio.h>
#include <sys/stat.h>

#include <esp_heap_caps.h>
#include <esp_log.h>

#include <ArduinoJson.h>
#include <simplevox.h>

#define NameOf(x) #x

constexpr char TAG[] = "CMDVOX";

namespace
{

/**
 * @brief 除算を行い演算結果を切り上げます（正の整数）
 * @param[in] dividend  被除数
 * @param[in]  divisor   除数
 * @return 切り上げた整数(3/2 -> 2)
 */
constexpr int divCeil(int dividend, int divisor)
{
  return (dividend + divisor - 1) / divisor;
}

template<typename T>
void arr_push_back(const T* src, int n, T* dest, int* length)
{
    std::copy_n(src, n, &dest[*length]);
    *length += n;
}

template<typename T>
void arr_pop_front(T* arr, int n, int* length)
{
    if (*length < n) { return; }

    std::copy_n(&arr[n], *length - n, arr);
    *length -= n;
}

}


namespace cmdvox
{

bool MfccCommander::init(const CommanderConfig &config)
{
    const auto& vad_config = config.vad_config;
    const auto& mfcc_config = config.mfcc_config;

    if (vad_config.sample_rate != mfcc_config.sample_rate)
    {
        return false;
    }

    if (!vad_engine_.init(vad_config))
    {
        return false;
    }

    if (!mfcc_engine_.init(mfcc_config))
    {
        vad_engine_.deinit();
        return false;
    }

    const int max_length = config.limit_time_ms * vad_config.sample_rate / 1000;
    max_frame_num_ = (max_length - (mfcc_config.frame_length() - mfcc_config.hop_length())) / mfcc_config.hop_length();
    const int pre_length =
                vad_config.frame_length() *
                ( divCeil(vad_config.before_length(), vad_config.frame_length())
                + divCeil(vad_config.decision_length(), vad_config.frame_length()));
    pre_frame_num_ = (pre_length - (mfcc_config.frame_length() - mfcc_config.hop_length())) / mfcc_config.hop_length();

    raw_mfcc_ = (float*)heap_caps_malloc(sizeof(*raw_mfcc_) * max_frame_num_ * mfcc_config.coef_num, MALLOC_CAP_8BIT);
    raw_max_length_ = std::max(vad_config.frame_length(), mfcc_config.frame_length()) * 2;
    raw_queue_ = (int16_t*)heap_caps_malloc(sizeof(*raw_queue_) * raw_max_length_, MALLOC_CAP_8BIT);
    
    if (raw_mfcc_ == nullptr || raw_queue_ == nullptr)
    {
        if (raw_queue_ != nullptr)
        {
            heap_caps_free(raw_queue_);
            raw_queue_ = nullptr;
        }
        if (raw_mfcc_ != nullptr)
        {
            heap_caps_free(raw_mfcc_);
            raw_mfcc_ = nullptr;
        }
        mfcc_engine_.deinit();
        vad_engine_.deinit();
        return false;
    }

    frame_length_ = vad_config.frame_length();
    config_ = config;
    reset();
    return true;
}

void MfccCommander::deinit()
{
    if (raw_queue_ != nullptr)
    {
        heap_caps_free(raw_queue_);
        raw_queue_ = nullptr;
    }
    if (raw_mfcc_ != nullptr)
    {
        heap_caps_free(raw_mfcc_);
        raw_mfcc_ = nullptr;
    }
    mfcc_engine_.deinit();
    vad_engine_.deinit();
}

void MfccCommander::reset()
{
    raw_length_ = 0;
    frame_count_ = 0;
    vad_engine_.reset();
    vad_state_ = simplevox::VadState::Warmup;
}

void MfccCommander::add(MfccCommand &&command)
{
    for (auto& cmd: commands)
    {
        if (cmd.info.id == command.info.id
            && cmd.info.name.compare(command.info.name) == 0)
        {
            ESP_LOGI(TAG, "Swap and Add command: %s", command.info.name.c_str());
            std::swap(cmd, command);
            return;
        }
    }

    ESP_LOGI(TAG, "Add command: %s", command.info.name.c_str());
    commands.push_back(std::move(command));
}

void MfccCommander::remove(const std::string &name, int id)
{
    for (auto it = commands.begin(); it != commands.end();)
    {
        if ((*it).info.name.compare(name))
        {
            if (id < 0 || id == (*it).info.id)
            {
                it = commands.erase(it);
            }
        }
        else
        {
            ++it;
        }
    }
}

void MfccCommander::modifyInfo(const std::string &name, int id, const CommandInfo &info)
{
    for (auto& command: commands)
    {
        if (command.info.name == name && command.info.id == id)
        {
            command.info = info;
            break;
        }
    }
}

void MfccCommander::clear()
{
    commands.clear();
}

void MfccCommander::saveSettings(const std::string &path)
{
    FILE* file = fopen(path.c_str(), "w");
    if (file == NULL) { return; }

    fprintf(file, "{\"%s\":[", NameOf(commands));
    for(int i = 0; i < commands.size(); i++)
    {
        const auto& command = commands[i];
        if(i > 0) { fprintf(file, ",\n"); }
        fprintf(
            file,
            "{\"name\":\"%s\", \"id\":%d, \"threshold\":%lu, \"path\":\"%s\"}",
            command.info.name.c_str(),
            command.info.id,
            command.info.threshold,
            command.info.path.c_str()
        );
    }
    fprintf(file, "]}");
    fclose(file);
}

void MfccCommander::loadSettings(const std::string &path)
{
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
        ESP_LOGE(TAG, "stat() failed: %d", errno);
        return;
    }

    const auto file_size = info.st_size;
    std::unique_ptr<char[]> file_str(new char[file_size + 1]);
    FILE* file = fopen(path.c_str(), "r");
    const auto length = fread(file_str.get(), sizeof(*file_str.get()), file_size, file);
    file_str[length] = '\0';

    DynamicJsonDocument doc(2 * file_size);
    auto error = deserializeJson(doc, file_str.get());
    if (error)
    {
        ESP_LOGE(TAG, "deserializeJson failed: %s", error.c_str());
    }
    else
    {
        auto arr = doc[NameOf(commands)].as<JsonArray>();
        for (const auto& value : arr)
        {
            MfccCommand command {
                .info {
                    .name = value["name"],
                    .id = value["id"],
                    .threshold = value["threshold"],
                    .path = value["path"]
                }
            };
            ESP_LOGI(TAG, "Add command: %s", command.info.name.c_str());

            command.feature = std::unique_ptr<simplevox::MfccFeature>(loadFeature(command.info.path.c_str()));
            add(std::move(command));
        }
    }

    fclose(file);
}

FeedResult MfccCommander::feedSample(const int16_t *data)
{
    FeedResult result {
        .can_fetch = can_fetch()
    };

    if (can_fetch()) { return  result; }

    const int vad_frame_length = config_.vad_config.frame_length();
    const int mfcc_frame_length = config_.mfcc_config.frame_length();
    const int mfcc_hop_length = config_.mfcc_config.hop_length();
    const int mfcc_coef_num = config_.mfcc_config.coef_num;

    const auto state = vad_state_ = vad_engine_.process(data);
    if (state >= simplevox::VadState::Silence)
    {
        arr_push_back(data, vad_frame_length, raw_queue_, &raw_length_);
    }

    while (raw_length_ >= mfcc_frame_length)
    {
        if (frame_count_ < max_frame_num_)
        {
            mfcc_engine_.calculate(raw_queue_, &raw_mfcc_[frame_count_ * mfcc_coef_num]);
            frame_count_++;
        }
        arr_pop_front(raw_queue_, mfcc_hop_length, &raw_length_);
    }

    if (state < simplevox::VadState::Speech && frame_count_ > pre_frame_num_)
    {
        const int over_count = frame_count_ - pre_frame_num_;
        const int over_length = over_count * mfcc_coef_num;
        int length = frame_count_ * mfcc_coef_num;
        arr_pop_front(raw_mfcc_, over_length, &length);
        frame_count_ -= over_count;
    }

    result.can_fetch = can_fetch();
    return result;
}

FetchResult MfccCommander::fetchFeature()
{
    FetchResult result;
    if (can_fetch())
    {
        result.feature = std::unique_ptr<simplevox::MfccFeature>(mfcc_engine_.create(raw_mfcc_, frame_count_, mfcc_engine_.config().coef_num));
        reset();
        return result;
    }
    else
    {
        return result;
    }
}

bool MfccCommander::detect(const int16_t *data, DetectResult *result)
{
    const auto feed_result = feedSample(data);
    if (feed_result.can_fetch)
    {
        int index = -1;
        uint32_t min_dtw = UINT32_MAX;
        auto fetch_result = fetchFeature();
        for(int i = 0; i < commands.size(); i++)
        {
            const auto& command = commands[i];
            const auto dtw = simplevox::calcDTW(*fetch_result.feature, *command.feature);
            ESP_LOGI(TAG, "command[%d]: %lu", i, dtw);
            if (dtw < min_dtw && dtw < command.info.threshold)
            {
                min_dtw = dtw;
                index = i;
            }
        }

        const bool is_detected = (index >= 0);
        if (is_detected)
        {
            const auto& command = commands[index];
            result->command_name = command.info.name;
            result->id = command.info.id;
            result->score = min_dtw;
        }
        return is_detected;
    }
    return false;
}

} // namespace cmdvox
