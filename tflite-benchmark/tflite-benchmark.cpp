#include <chrono>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <htwk_vision_config.h>
#include <tfliteexecuter.h>

using namespace htwk;
using namespace std::chrono;
using namespace std;
namespace bpo = boost::program_options;
namespace bf = boost::filesystem;

void handleProgramArguments(bpo::variables_map &varmap, int argc, char **argv) {
    bpo::options_description options_all("");
    auto tmp = options_all.add_options();
    tmp = tmp("help,h", "Print this help");
    tmp = tmp("warmup-rounds,w", bpo::value<int>()->default_value(10), "How many rounds on prewarm should be used for every benchmark.");
    tmp = tmp("benchmark-time,t", bpo::value<int>()->default_value(5), "How many seconds will we measure per benchmark.");

    bpo::store(bpo::parse_command_line(argc, argv, options_all), varmap);  // parse and store

    bpo::notify(varmap);  // update the varmap

    if (varmap.count("help")) {
        std::cout << options_all;
        exit(0);
    }
}

struct BenchmarkParts {
    std::string name;
    std::string model_file;
    std::vector<int> input_dim;
    int num_cpu = 1;
};

int main(int argc, char *argv[])
{
    bpo::variables_map varmap;
    handleProgramArguments(varmap, argc, argv);

    const float benchmark_time = varmap["benchmark-time"].as<int>();
    const int warmup_rounds = varmap["warmup-rounds"].as<int>();

    HtwkVisionConfig config;

    std::vector<BenchmarkParts> benchmarks {
        {"U_RobotDetector", config.tflitePath + "/uc-robot-detector.tflite", {1, 80, 60, 3}, 1},
        {"U_GoalPostDetector", config.tflitePath + "/uc-goalpost-detector.tflite", {32, 15, 10, 3}, 2},
        {"U_BallHypGenerator", config.tflitePath + "/uc-ball-hyp-generator.tflite", {16, 30, 40, 3}, 4},
        {"U_FieldBorder", config.tflitePath + "/uc-field-border-classifier.tflite", {1, config.lcObjectDetectorConfig.scaledImageHeight, config.lcObjectDetectorConfig.scaledImageWidth, 3}},
        {"U_PenatlyDetector", config.tflitePath + "/uc-penaltyspot-classifier.tflite", {config.hypothesisGeneratorMaxHypothesisCount, config.ucPenaltySpotClassifierConfig.patchSize, config.ucPenaltySpotClassifierConfig.patchSize, 1}},
        {"U_BallPreClassifier", config.tflitePath + "/uc-ball-small-classifier.tflite", {config.hypothesisGeneratorMaxHypothesisCount, config.ballDetectorPatchSize, config.ballDetectorPatchSize, 1}},
        {"U_BallPostClassifier", config.tflitePath + "/uc-ball-large-classifier.tflite", {config.ucBallLargeClassifierConfig.camHypothesesCount, config.ucBallLargeClassifierConfig.patchSize, config.ucBallLargeClassifierConfig.patchSize, 3}},
        {"L_ObstacleDetection", config.tflitePath + "/lc-obstacle-classifier.tflite", {1, config.obstacleDetectionInputHeight, config.obstacleDetectionInputWidth, 3}},
        {"L_BallHypGenerator", config.tflitePath + "/" + config.lcObjectDetectorConfig.hypGenModelBall, {1, config.lcObjectDetectorConfig.scaledImageHeight, config.lcObjectDetectorConfig.scaledImageWidth, 3}},
        {"L_PenaltyHypGenerator", config.tflitePath + "/" + config.lcObjectDetectorConfig.hypGenModelPenatlySpot, {1, config.lcObjectDetectorConfig.scaledImageHeight, config.lcObjectDetectorConfig.scaledImageWidth, 3}},
        {"L_ObjectDetector", config.tflitePath + "/lc-object-classifier.tflite", {2, config.lcObjectDetectorConfig.patchSize, config.lcObjectDetectorConfig.patchSize, 3}},
    };

    for(auto& b : benchmarks) {
        TFLiteExecuter exec;
        exec.loadModelFromFile(b.model_file, b.input_dim, b.num_cpu);

        for(int i = 0; i < warmup_rounds; i++) {
            exec.execute();
        }

        int counter = 0;
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<float> time_span;

        do {
            exec.execute();
            counter++;
            t2 = high_resolution_clock::now();
            time_span = duration_cast<duration<float>>(t2 - t1);
        } while(time_span.count() < benchmark_time);

        printf("%s; %.2f ;cycles/s\n", b.name.c_str(), counter / time_span.count());
        fflush(stdout);
    }
    return 0;
}
