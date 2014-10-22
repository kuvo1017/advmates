#include "AppMates.h"
#include "AmuConverter.h"
#include "GVInitializer.h"
#include "GVManager.h"
#include "Random.h"
#include "Simulator.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <unistd.h>
#ifndef USE_MINGW
#include <getopt.h>
#endif
#ifdef _OPENMP
#include <omp.h>
#endif

#define MATES_NDEBUG

using namespace std;

//======================================================================
AppMates::AppMates() :  _simulator(), _dataPath(), _key()
{
    //_dataPathと_keyのデフォルト値を設定
    _dataPath = "./";

#ifdef MATES_NDEBUG
    _key = time(NULL);
#else
    _key = 2;
#endif

    _isVerbose = true;

    _inputMap     = true;
    _inputSignal  = true;
    _inputVehicle = true;

    _generateRandomVehicle = true;
}

//======================================================================
void AppMates::init(int argc, char** argv, bool output)
{
    // オプションの処理
    AppMates::parseArgument(argc, argv);

    cout << "random seed   : " << _key << endl;
    cout << "data directory: " << _dataPath << endl;

    // 乱数の準備
    srand(_key);
    Random::setSeed(_key);

    // パスの設定
    initPath();

    // グローバル変数を読み込む(_initPathよりも後ろでなければならない)
    GVManager& gvm = GVManager::instance();
    string fileName;
    if (gvm.getVariable("GV_INIT_FILE", &fileName))
    {
        gvm.setVariablesFromFile(fileName);
    }
    if (_isVerbose)
    {
        gvm.print();
    }

#ifdef _OPENMP
    // スレッド数設定
    int thread = (int)gvm.getNumeric("THREAD_NUM");
    if (thread <= 0)
    {
        thread = omp_get_num_procs();
    }
    cout << "Thread number: " << thread;
    omp_set_num_threads(thread);

    // 並列ストック最大数設定
    int randomStock = (int)gvm.getNumeric("RANDOM_MULTI_STOCK");
    if (randomStock == 0 && thread > 1)
    {
        Random::multiStockSetMax(1);
        cout << ", Random stock: 1 (auto)" << endl;
    }
    else if (randomStock > 0)
    {
        Random::multiStockSetMax(randomStock);
        cout << ", Random stock: " << randomStock << endl;
    }
    else
        cout << ", Random stock: none" << endl;
#endif

    // シミュレータの準備
    // RoadMapの作成
    bool isReady = getReadySimulator(output);
    if (!isReady)
    {
        cerr << "Error: failed to get ready for running simulator." << endl;
    }
}

//======================================================================
Simulator* AppMates::simulator()
{
    return _simulator;
}

//======================================================================
int AppMates::optionIndex;
string AppMates::shortOptions = "HhD:d:R:r:T:t:SsLlMmGgQq";
struct option AppMates::longOptions[] =
{
    {"help", 0, 0, 'h'},
    {"time", 1, 0, 't'},
    {"no-input",         0, 0, 30},
    {"no-input-map",     0, 0, 31},
    {"no-input-signal",  0, 0, 32},
    {"no-input-vehicle", 0, 0, 33},
    {"no-output-timeline",   0, 0, 's'},
    {"no-output-tripinfo",   0, 0, 'l'},
    {"no-output-generate",   0, 0, 'g'},
    {"no-output-instrument", 0, 0, 'm'},
    {"no-verbose", 0, 0, 'q'},
    {"no-output-monitor-d", 0, 0, 20},
    {"no-output-monitor-s", 0, 0, 21},
    {"no-generate-random-vehicle", 0, 0, 40},
    {0, 0, 0, 0}
};

//======================================================================
void AppMates::parseArgument(int argc, char** argv)
{
    int opt;
#ifdef USE_MINGW
    while ((opt = getopt(argc, argv,
                         AppMates::shortOptions.c_str())) != -1)
#else //USE_MINGW
    while ((opt = getopt_long(argc, argv,
                              AppMates::shortOptions.c_str(),
                              AppMates::longOptions,
                              &AppMates::optionIndex)) != -1)
#endif //USE_MINGW
    {
        switch (opt)
        {
        case 'H':
        case 'h': // ヘルプを出力する
            printUsage();
            break;
        case 'D':
        case 'd': // データディレクトリを指定する
            _initDataPath(optarg);
            break;
        case 'R':
        case 'r': // 乱数の種を指定する
            _initRandomSeed(optarg);
            break;
        case 'Q': // 情報表示をonに（デフォルトでon）
            _isVerbose = true;
            break;
        case 'q': // 情報表示をoffに
            _isVerbose = false;
            break;
#ifndef USE_MINGW
        case 30:  // 入力をoffに
            _inputMap     = false;
            _inputSignal  = false;
            _inputVehicle = false;
            break;
        case 31:  // 地図データ入力をoffに
            _inputMap     = false;
            break;
        case 32:  // 信号データ入力をoffに
            _inputSignal  = false;
            break;
        case 33:
            _inputVehicle = false;
            break;
        case 40:
            _generateRandomVehicle = false;
            break;
#endif //USE_MINGW
        default:
            break;
        }
    }

    // 派生クラスのparseArgumentを呼ぶ
    optind = 1;
    parseArgument(argc, argv);
}

//======================================================================
void AppMates::printUsage()
{
    cout <<
        "Options:\n"
        " -d <DataDir>      : set root path of input and output directory.\n"
        "                     (default: current directory)\n"
        " -r <Number>       : set random seed.\n"
        "                     (default: variable number on account of current time)\n"
#ifndef USE_MINGW
        " --no-verbose      : hide detail information.\n"
        " --no-input        : do not read any input files.\n"
        " --no-input-map    : do not read network.txt and mapPosition.txt.\n"
        " --no-input-signal : do not read signal input files.\n"
        "                     All signals show blue sign.\n"
        " --no-generate-random-vehicle\n"
        "                   : do not generate vehicles without input data."
#endif //USE_MINGW
         << endl;
}

//======================================================================
void AppMates::initPath()
{
    // データ関連の設定をする
    // RoadMap関連クラスからも使用する
    GVInitializer::init(_dataPath);
}

//======================================================================
bool AppMates::getReadySimulator(bool isWrite)
{
    bool result = true;

    //_simulatorの新規作成
    if (_simulator != NULL)
    {
        _simulator = NULL;
    }
    _simulator = new Simulator();

    // 入力フラグを設定する
    _simulator->setInputState(_inputMap,
                              _inputSignal,
                              _inputVehicle);

    if (isWrite)
    {
        // 出力フラグを設定する
        /*
         * デフォルトでは 時系列データtrue，路側器データtrue，
         * 車両発生データtrue
         */
        _simulator->setOutputState(_outputTimeline,
                                   _outputInstrumentD,
                                   _outputInstrumentS,
                                   _outputGenCounter,
                                   _outputTripInfo);
    }

    // 詳細メッセージフラグを設定する
    _simulator->setVerbose(_isVerbose);

    // エージェントフラグを設定する
    _simulator->setAgentState(_generateRandomVehicle);

    // 地図を準備する
    if (_inputMap)
    {
        result = _simulator->getReadyRoadMap();

        // 路側器に関するファイルを読み込む
        // 自動車以外を計測する場合もあるかもしれないので，
        // 下のifndefの外にした
        result = _simulator->getReadyRoadsideUnit();
        if (!result)
        {
            cerr << "Cannot Load RoadsideUnit Files." << endl;
            return result;
        }
#ifndef EXCLUDE_VEHICLES
        // 車両に関するファイルを読み込む
        result = _simulator->getReadyVehicles();
        if (!result)
        {
            cerr << "Cannot Load Vehicle Files." << endl;
            return result;
        }
#endif //EXCLUDE_VEHICLES
    }
    else
    {
        // 地図を読み込まない場合->サンプルシナリオ
        result = _simulator->getReadySampleScenario(-500, 500,
                                                    -500, 500,
                                                    200, 200,
                                                    100, 10);
    }
    if (!result)
    {
        cerr << "Cannot Create RoadMap." << endl;
        return result;
    }

    // レーンチェック
    _simulator->checkLane();
    return result;
}

//======================================================================
bool AppMates::_initDataPath(string arg)
{
    if (!arg.empty())
    {
        if (arg[arg.length()-1] != '/')
        {
            arg += '/';
        }
        _dataPath = arg;
    }
    return true;
}

//======================================================================
bool AppMates::_initRandomSeed(string arg)
{
    if (!arg.empty())
    {
        _key = static_cast<unsigned int>(AmuConverter::strtoul(arg));
        return true;
    }
    else
    {
        return false;
    }
}
