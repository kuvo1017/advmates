#include "AppCalc.h"
#include "Simulator.h"
#include "TimeManager.h"
#include "GVManager.h"
#include "VehicleIO.h"
#include <iostream>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#ifndef USE_MINGW
#include <getopt.h>
#endif

using namespace std;

//======================================================================
AppCalc::AppCalc()
{
    _outputTimeline    = true;
    _outputInstrumentD = true;
    _outputInstrumentS = true;
    _outputGenCounter  = true;
    _outputTripInfo    = true;

    _maxTime = DEFAULT_MAX_TIME;
}

//======================================================================
AppCalc::~AppCalc()
{
    if (_simulator)
    {
        delete _simulator;
    }
}

//======================================================================
void AppCalc::init(int argc, char** argv, bool output)
{
    // オプションの処理とグローバル変数の準備，simulatorの作成
    AppMates::init(argc, argv, output);

    if (output && AppMates::_isVerbose)
    {
        cout << "time to stop calculation    : " << _maxTime << endl;
        cout << "output timeline data        : "
             << (_outputTimeline?"true":"false")
             << "\noutput monitoring detail data      : "
             << (_outputInstrumentD?"true":"false")
             << "\noutput monitoring statistic data   : "
             << (_outputInstrumentS?"true":"false")
             << "\noutput generate counter data : "
             << (_outputGenCounter?"true":"false")
             << "\noutput trip distance and trip time : "
             << (_outputTripInfo?"true":"false") << endl;
    }
}

//======================================================================
void AppCalc::parseArgument(int argc, char** argv)
{
    int opt;
#ifdef USE_MINGW
    while ((opt = getopt(argc, argv,
                         App::shortOptions.c_str())) != -1)
#else //USE_MINGW
    while ((opt = getopt_long(argc, argv,
                              AppMates::shortOptions.c_str(),
                              AppMates::longOptions,
                              &AppMates::optionIndex)) != -1)
#endif //USE_MINGW
    {
        switch (opt)
        {
        case 'T':
        case 't':
            _maxTime = static_cast<unsigned long>(atoi(optarg));
            break;
        case 'S': // 時系列データ出力をonに(calcデフォルトでon)
            _outputTimeline = true;
            break;
        case 's': // 時系列データ出力をoffに
            _outputTimeline = false;
            break;
        case 'L': // 走行距離，時間の出力をonに(calcデフォルトでon)
            _outputTripInfo = true;
            break;
        case 'l': // 走行距離，時間の出力をoffに
            _outputTripInfo = false;
            break;
        case 'M': // 路側器データ出力をonに(calcデフォルトでon)
            _outputInstrumentD = true;
            _outputInstrumentS = true;
            break;
        case 'm': // 路側器データ出力をoffに
            _outputInstrumentD = false;
            _outputInstrumentS = false;
            break;
#ifndef USE_MINGW
        case 20: // 路側器データのうち詳細データの出力をoffに
            _outputInstrumentD = false;
            break;
        case 21: // 路側器データのうち統計データの出力をoffに
            _outputInstrumentS = false;
            break;
#endif //USE_MINGW
        case 'G': // 発生データ出力をonに(matesデフォルトでon)
            _outputGenCounter = true;
            break;
        case 'g': // 発生データ出力をoffに
            _outputGenCounter = false;
            break;
        default:
            break;
        }
    }
}

//======================================================================
void AppCalc::printUsage()
{
    cout <<
        "Usage  : ./advmates-calc [Option list] \n"
         << endl;
    AppMates::printUsage();
    cout <<
        " -t                : time to stop calculation\n"
        "                     This must be multiple number of time step.\n"
        "                     (Time step default is 100 mili second.)\n"
        "                     If this isn't given, simulator will use default value.\n"
        "                     (default: "
         << DEFAULT_MAX_TIME
         <<
        "[mili second]).\n"
        " -s                : do not output timeline data\n"
        " -m                : do not output monitoring data\n"
#ifndef USE_MINGW
        " --no-output-monitor-d\n"
        "                   : do not output monitoring data(detail)\n"
        " --no-output-monitor-s\n"
        "                   : do not output monitoring data(statistic)\n"
#endif //USE_MINGW
        " -g                : do not output generate counter data\n"
        " -l                : do not output trip info\n"
         << endl;
    exit(EXIT_SUCCESS);
}

//======================================================================
int AppCalc::batchRun()
{
    ulint maxTime = _maxTime;
    ulint mt = GVManager::instance().getMaxTime();
    if (mt>100)
        maxTime = mt;

    cout << "*** Run Mates: time=" << maxTime
         << " (dt=" << TimeManager::unit() << ") ***" << endl;
    _simulator->run(maxTime);

    if (_outputTripInfo)
    {
        VehicleIO::instance().writeAllVehiclesDistanceData();
    }

    return EXIT_SUCCESS;
}
