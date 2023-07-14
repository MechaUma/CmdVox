/*!
 * CmdVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef CMDVOX_H_
#define CMDVOC_H_

#include <memory>
#include <vector>
#include <stdint.h>

#include <simplevox.h>

namespace cmdvox
{

struct CommanderConfig
{
    simplevox::VadConfig vad_config;
    simplevox::MfccConfig mfcc_config;
    int limit_time_ms = 3000;
};

/**
 * @brief information of command
 * @note Identified by the combination of name and id.
 * 
 */
struct CommandInfo
{
    std::string name;
    int id;
    uint32_t threshold;
    std::string path;
};

struct MfccCommand
{
    CommandInfo info;
    std::unique_ptr<simplevox::MfccFeature> feature;
};

struct FeedResult
{
    bool can_fetch;
};

struct FetchResult
{
    std::unique_ptr<simplevox::MfccFeature> feature;
};

struct DetectResult
{
    std::string command_name;
    int id;
    uint32_t score;
};

class MfccCommander
{
public:
    bool init(const CommanderConfig& config);
    void deinit();
    void reset();

    void add(MfccCommand&& command);
    void remove(const std::string& name, int id = -1);
    void modifyInfo(const std::string& name, int id, const CommandInfo& info);
    void clear();

    void saveSettings(const std::string& path);
    void loadSettings(const std::string& path);

    FeedResult feedSample(const int16_t* data);
    FetchResult fetchFeature();
    bool detect(const int16_t* data, DetectResult* result);

    int feed_length() { return frame_length_; }
    simplevox::VadState vad_state() { return vad_state_; }

    // delegation
    int detectVoice(int16_t* dest, int length, const int16_t* data) { return vad_engine_.detect(dest, length, data); }
    void calcFeature(const int16_t* frame, float* mfcc) { mfcc_engine_.calculate(frame, mfcc); }
    void normFeature(const float* src, int frame_num, int coef_num, int16_t* dest) { mfcc_engine_.normalize(src, frame_num, coef_num, dest); }
    static bool saveFeature(const char* path, const simplevox::MfccFeature& mfcc) { return simplevox::MfccEngine::saveFile(path, mfcc); }
    static simplevox::MfccFeature* loadFeature(const char* path) { return simplevox::MfccEngine::loadFile(path); }
    simplevox::MfccFeature* createFeature(const int16_t* raw_audio, int length) { return mfcc_engine_.create(raw_audio, length); }
    simplevox::MfccFeature* createFeature(const float* mfccs, int frame_num, int coef_num) { return mfcc_engine_.create(mfccs, frame_num, coef_num); }
private:
    CommanderConfig config_;
    simplevox::VadEngine vad_engine_;
    simplevox::MfccEngine mfcc_engine_;
    std::vector<MfccCommand> commands;
    int frame_length_;

    int16_t* raw_queue_ = nullptr;
    int raw_max_length_;
    int raw_length_;
    float* raw_mfcc_ = nullptr;
    int max_frame_num_;
    int pre_frame_num_;
    int frame_count_;
    simplevox::VadState vad_state_;
    bool can_fetch() { return vad_state_ == simplevox::VadState::Detected || (vad_state_ >= simplevox::VadState::Speech && max_frame_num_ <= frame_count_); }
};

} // namespace cmdvox


#endif // CMDVOX_H_