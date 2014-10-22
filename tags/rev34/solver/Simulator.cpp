#include "Simulator.h"
#include "GVManager.h"
#include "GVInitializer.h"
#include "RoadMapBuilder.h"
#include "ObjManager.h"
#include "FileManager.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "Vehicle.h"
#include "VehicleFamily.h"
#include "VehicleIO.h"
#include "SignalIO.h"
#include "Router.h"
#include "Random.h"
#include "DetectorIO.h"
#include "DetectorUnit.h"
#include "GenerateVehicleIO.h"
#include "VehicleFamilyManager.h"
#include "VehicleFamilyIO.h"
#include "AmuStringOperator.h"
#include "AmuConverter.h"
#include "Conf.h"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <cmath>
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;

//======================================================================
Simulator::Simulator()
{
    _roadMap = 0;
    _checkLaneError = false;
    _isVerbose   = false;

    _isMapInput     = false;
    _isSignalInput  = false;
    _isVehicleInput = false;

    _isWrite     = false;
    _isMonitorD  = false;
    _isMonitorS  = false;
    _isGenCount  = false;

    _isRandomVehicleGenerate = false;

    _vehicleIO = &VehicleIO::instance();
    _signalIO = &SignalIO::instance();

#ifdef _OPENMP
    _rerouteGenMax = _rerouteRunMax = _genIntensive = _genIntensiveStep = 0;
#endif
}

//======================================================================
Simulator::~Simulator()
{
#ifdef _OPENMP
    cout << "\nReroute generate max: " << _rerouteGenMax <<
        " run max: " << _rerouteRunMax << endl;
#endif

    TimeManager::printAllClockers();

    // Managerで管理するオブジェクトの開放
    TimeManager::deleteAllClockers();
    FileManager::deleteAllOFStreams();
    ObjManager::deleteAll();
    if (_roadMap!=NULL)
    {
        delete _roadMap;
    }
}

//======================================================================
bool Simulator::hasInit() const
{
    if (_roadMap)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool Simulator::getReadyRoadMap()
{
    RoadMapBuilder builder;

    // 道路ネットワークの作成
    builder.buildRoadMap();

    // 制限速度の設定
    builder.setSpeedLimit();

    if (_isSignalInput)
    {
        // 信号の作成
        builder.buildSignals();
    }
    else
    {
        // 全青信号の作成
        builder.buildSignalsAllBlue();
    }

    _roadMap = builder.roadMap();
    _signalIO->setRoadMap(_roadMap);

    // mapInfo.txtの作成
    if (_roadMap)
    {
        _roadMap->writeMapInfo();
    }

    // コンソールへ地図情報を表示する
    if (_isVerbose)
    {
        _roadMap->dispIntersections();
    }

    // ODノードのレベル分け
    vector<ODNode*> vec = _roadMap->odNodes();
    for (int i=0; i<static_cast<signed int>(vec.size()); i++)
    {
        if (0<=_odNodeStartLevel(vec[i])
            && _odNodeStartLevel(vec[i])<3)
        {
            _startLevel[_odNodeStartLevel(vec[i])].push_back(vec[i]);
        }
        if (0<=_odNodeGoalLevel(vec[i])
            && _odNodeGoalLevel(vec[i])<3)
        {
            _goalLevel[_odNodeGoalLevel(vec[i])].push_back(vec[i]);
        }
    }

    // global_static.txtの出力
    GVManager& gvm = GVManager::instance();
    string fGlobalStatic;
    gvm.getVariable("RESULT_GLOBAL_STATIC_FILE", &fGlobalStatic);
    ofstream ofs(fGlobalStatic.c_str(), ios::out);

    map<string, Signal*, less<string> >::const_iterator itl;
    // 信号の総数 (信号ID=ノードIDの数とは一致しないので注意)
    int totalNumberOfSignals=0;
    for (itl=_roadMap->signals()->begin();
         itl!=_roadMap->signals()->end();
         itl++)
    {
        totalNumberOfSignals += (*itl).second->numDirections();
    } 

    ofs << _roadMap->intersections()->size() << "\n" // 交差点の総数
        << totalNumberOfSignals;		     // 信号機の総数

    return _roadMap;
}

//======================================================================
bool Simulator::getReadySampleScenario(double xmin, double xmax,
				       double ymin, double ymax,
				       double xsp,  double ysp,
				       unsigned int numVehicles,
				       double headway)
{
    RoadMapBuilder builder;

    // 道路ネットワークの作成
    builder.buildGridRoadMap(xmin, xmax, ymin, ymax, xsp, ysp);

    // 全青信号の作成
    builder.buildSignalsAllBlue();

    _roadMap = builder.roadMap();
    _signalIO->setRoadMap(_roadMap);


    // ODノードのレベル分け
    vector<ODNode*> vec = _roadMap->odNodes();
    for (int i=0; i<static_cast<signed int>(vec.size()); i++)
    {
        if (0<=_odNodeStartLevel(vec[i]) && _odNodeStartLevel(vec[i])<3)
        {
            _startLevel[_odNodeStartLevel(vec[i])].push_back(vec[i]);
        }
        if (0<=_odNodeGoalLevel(vec[i]) && _odNodeGoalLevel(vec[i])<3)
        {
            _goalLevel[_odNodeGoalLevel(vec[i])].push_back(vec[i]);
        }
    }

    // 車両発生に関する設定
    getReadyVehicles();

    // 車両配置
    if (numVehicles)
    {
        generateSampleVehicles(numVehicles, headway);
    }

    return _roadMap;
}

//======================================================================
void Simulator::startSampleScenario(double xmin, double xmax,
				    double ymin, double ymax,
				    double xsp,  double ysp,
				    unsigned int numVehicles,
				    double headway)
{
    GVInitializer::init("./");
    getReadySampleScenario(xmin, xmax, ymin, ymax, xsp, ysp,
                           numVehicles, headway);
}

//======================================================================
bool Simulator::getReadyRoadsideUnit()
{
    assert(_roadMap);

    // 車両感知器設定ファイルの読み込み
    DetectorIO::getReadyDetectors(_roadMap);

    // 感知器データ出力ファイルの準備
    vector<DetectorUnit*>* detectorUnits = ObjManager::detectorUnits();
    DetectorIO::getReadyOutputFiles(detectorUnits);
    if (_isVerbose)
    {
        DetectorIO::print();
    }

    // 車両発生カウンタの設定ファイル読み込みと準備
    GenerateVehicleIO::getReadyCounters(_roadMap, _isVerbose);

    return true;
}

//======================================================================
bool Simulator::getReadyVehicles()
{
    assert(_roadMap);
    GVManager& gvm = GVManager::instance();

    // 標準的な交通量(単路ごとの台/時)
    // 基本交通容量の10%
    // "レーンごと"ではない
    _defaultTrafficVolume[0] = (int)gvm.getNumeric("DEFAULT_TRAFFIC_VOLUME_WIDE");
    _defaultTrafficVolume[1] = (int)gvm.getNumeric("DEFAULT_TRAFFIC_VOLUME_NORMAL");
    _defaultTrafficVolume[2] = (int)gvm.getNumeric("DEFAULT_TRAFFIC_VOLUME_NARROW");

    // 車両発生定義ファイルの読み込み
    if (_isVehicleInput)
    {
        string fGenerateTable, fDefaultGenerateTable;
        gvm.getVariable("GENERATE_TABLE", &fGenerateTable);
        gvm.getVariable("DEFAULT_GENERATE_TABLE", &fDefaultGenerateTable);

        if (!fGenerateTable.empty())
        {
            _table.init(fGenerateTable);
        } 
        if (!fDefaultGenerateTable.empty())
        {
            _defaultTable.init(fDefaultGenerateTable);
        }
    }

    // 車種情報の設定
    // デフォルトで20(普通車), 50(大型車)を作成する
    double length, width, height;
    double weight = 0.0;
    double accel, decel;
    double r, g, b;

    length = gvm.getNumeric("VEHICLE_LENGTH_PASSENGER");
    width  = gvm.getNumeric("VEHICLE_WIDTH_PASSENGER");
    height = gvm.getNumeric("VEHICLE_HEIGHT_PASSENGER");
    accel  = gvm.getNumeric("MAX_ACCELERATION_PASSENGER");
    decel  = gvm.getNumeric("MAX_DECELERATION_PASSENGER");
    r = 1.0;
    g = 0.0;
    b = 0.0;
    VFAttribute passenger(VehicleFamily::passenger(),
                          length, width, height, weight,
                          accel, decel, r, g, b);
    VehicleFamilyManager::addVehicleFamily(passenger);

    length = gvm.getNumeric("VEHICLE_LENGTH_TRUCK");
    width  = gvm.getNumeric("VEHICLE_WIDTH_TRUCK");
    height = gvm.getNumeric("VEHICLE_HEIGHT_TRUCK");
    accel  = gvm.getNumeric("MAX_ACCELERATION_TRUCK");
    decel  = gvm.getNumeric("MAX_DECELERATION_TRUCK");
    r = 0.3;
    g = 0.7;
    b = 1.0;
    VFAttribute truck(VehicleFamily::truck(),
                      length, width, height, weight,
                      accel, decel, r, g, b);
    VehicleFamilyManager::addVehicleFamily(truck);

    // 車両情報をファイルから読み込む
    VehicleFamilyIO::getReadyVehicleFamily(_isVerbose);
    VehicleFamilyIO::print();

    // 経路探索用のパラメータの設定
    readRouteParameter();

    return true;
}

//======================================================================
void Simulator::readRouteParameter()
{
    /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
     * 経路探索用のパラメータの設定
     * 0番目は距離に関するコスト（これが大きいと距離が短い経路が高効用）
     * 1番目は時間に関するコスト（時間が短い経路が高効用）
     * 2番目は交差点での直進に関するコスト（直進が少ない経路が高効用）
     * 3番目は交差点での左折に関するコスト（左折が少ない経路が高効用）
     * 4番目は交差点での右折に関するコスト（右折が少ない経路が高効用）
     * 5番目は道路の広さに関するコスト（道路幅が広い経路が高効用）
     *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
     */

    _vehicleRoutingParams.resize(1);
    _vehicleRoutingParams[0].resize(VEHICLE_ROUTING_PARAMETER_SIZE);

    _vehicleRoutingParams[0][0] = 1;
    _vehicleRoutingParams[0][1] = 0;
    _vehicleRoutingParams[0][2] = 0;
    _vehicleRoutingParams[0][3] = 0;
    _vehicleRoutingParams[0][4] = 0;
    _vehicleRoutingParams[0][5] = 0;

    string fParamFile;
    GVManager::instance().getVariable("VEHICLE_ROUTE_PARAM_FILE",
                                      &fParamFile);

    ifstream inParamFile(fParamFile.c_str(), ios::in);
    if (inParamFile.good())
    {
        string str;
        int index = 0;
        while (inParamFile.good())
        {
            getline(inParamFile, str);
            AmuStringOperator::getAdjustString(&str);
            if (!str.empty())
            {
                vector<string> tokens;
                AmuStringOperator::getTokens(&tokens, str, ',');
                if (tokens.size()==VEHICLE_ROUTING_PARAMETER_SIZE)
                {
                    // パラメータ指定が有効な行
                    _vehicleRoutingParams.resize(index+1);
                    _vehicleRoutingParams[index]
                        .resize(VEHICLE_ROUTING_PARAMETER_SIZE);

                    for (unsigned int i=0; i<tokens.size(); i++)
                    {
                        _vehicleRoutingParams[index][i]
                            = AmuConverter::strtod(tokens[i]);
                    }
                    index++;
                }
            }
        }
    }
    else
    {
        if (_isVehicleInput)
        {
            // 入力ファイルが存在しない場合
            cout << "no vehicle routing parameter file: "
                 << fParamFile << endl;
        }
        _vehicleRoutingParams.resize(3);
        for (unsigned int i=1; i<_vehicleRoutingParams.size(); i++)
        {
            _vehicleRoutingParams[i]
                .resize(VEHICLE_ROUTING_PARAMETER_SIZE);
        }

        _vehicleRoutingParams[1][0] = 0;
        _vehicleRoutingParams[1][1] = 1;
        _vehicleRoutingParams[1][2] = 0;
        _vehicleRoutingParams[1][3] = 0;
        _vehicleRoutingParams[1][4] = 0;
        _vehicleRoutingParams[1][5] = 0;

        _vehicleRoutingParams[2][0] = 1;
        _vehicleRoutingParams[2][1] = 1;
        _vehicleRoutingParams[2][2] = 0;
        _vehicleRoutingParams[2][3] = 0;
        _vehicleRoutingParams[2][4] = 0;
        _vehicleRoutingParams[2][5] = 0;
    }

    if (_isVerbose)
    {
        cout << endl << "*** Vehicle Routing Parameters ***" << endl;
        cout << "NumParams: " << _vehicleRoutingParams.size() << ", "
             << _vehicleRoutingParams[0].size() << endl;
        for (unsigned int i=0; i<_vehicleRoutingParams.size(); i++)
        {
            cout << i << ":";
            for (unsigned int j=0; j<_vehicleRoutingParams[i].size(); j++)
            {
                cout << _vehicleRoutingParams[i][j] << ",";
            }
            cout << endl;
        }
        cout << endl;
    }
}

//======================================================================
void Simulator::checkLane()
{
    // レーンチェック、エラー時は表示確認のため run のみ止める
    _checkLaneError = !_roadMap->checkIntersectionLane();
}

//======================================================================
bool Simulator::checkLaneError()
{
    return _checkLaneError;
}

//======================================================================
bool Simulator::run(ulint time)
{
    // レーンチェックエラー、表示確認のため run のみ止める
    if (_checkLaneError)
    {
        return false;
    }
    if (time>TimeManager::time())
    {
        TimeManager::startClock("TOTALRUN");
        while (time>TimeManager::time())
        {
            timeIncrement();
        }
        TimeManager::stopClock("TOTALRUN");
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool Simulator::timeIncrement()
{
#ifdef _OPENMP
    if (_genIntensive == 0)
    {
        GVManager& gvm = GVManager::instance();
        _genIntensive = (int)gvm.getNumeric("GENERATE_INTENSIVE");
        if (_genIntensive < 1)
        {
            _genIntensive = 1;
        }
        _genIntensiveStep = 0;
    }
#endif

    // 時刻の更新
    TimeManager::increment();
    if (_isVerbose)
    {
        if (TimeManager::time()%1000==0)
        {
            cout << "Time: "
                 << TimeManager::time()/1000 << "[sec]" << endl;
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // エージェント列の更新
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // レーン上のエージェントを更新
    TimeManager::startClock("RENEW");
    _roadMap->renewAgentLine();
    TimeManager::stopClock("RENEW");

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // モニタリング
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if (_isMonitorD || _isMonitorS)
    {
        vector<DetectorUnit*>* detectorUnits = ObjManager::detectorUnits();
        for_each(detectorUnits->begin(),
                 detectorUnits->end(),
                 mem_fun(&DetectorUnit::monitorLanes));
        DetectorIO::writeTrafficData(detectorUnits,
                                     _isMonitorD, _isMonitorS);
    }

    // エージェントの消去
    _roadMap->deleteArrivedAgents(_isTripInfo);

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // エージェントの発生
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 車両の発生
    TimeManager::startClock("GENERATE");
    generateVehicle();
    TimeManager::stopClock("GENERATE");

#ifdef USE_ADDIN
    // アドインエージェント発生後
    aim.afterGenerate();
#endif

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 認知
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    vector<Vehicle*>* vehicles = ObjManager::vehicles();
    TimeManager::startClock("RECOGNIZE");
#ifndef _OPENMP
    for_each(vehicles->begin(), vehicles->end(),
             mem_fun(&Vehicle::recognize));
#else
    int vehiclesSize = vehicles->size();
    // 並列時は車線変更等の前処理を非並列実行後、並列で認知処理
    for_each(vehicles->begin(), vehicles->end(),
             mem_fun(&Vehicle::preRecognize));
    Random::multiStockReady(vehiclesSize);
#pragma omp parallel for schedule (dynamic)
    for (int i = 0; i < vehiclesSize; i++)
    {
        Random::multiStockBeginMulti(i);
        (*vehicles)[i]->recognize();
    }
    Random::multiStockEndMulti();
#endif
    TimeManager::stopClock("RECOGNIZE");

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 意志決定
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    TimeManager::startClock("READYTORUN");
#ifndef _OPENMP
    for_each(vehicles->begin(), vehicles->end(),
             mem_fun(&Vehicle::readyToRun));
#else
    Random::multiStockReady(vehiclesSize);
#pragma omp parallel for schedule (dynamic)
    for (int i = 0; i < vehiclesSize; i++)
    {
        Random::multiStockBeginMulti(i);
        (*vehicles)[i]->readyToRun();
    }
    Random::multiStockEndMulti();
#endif
    TimeManager::stopClock("READYTORUN");

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 行動
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    TimeManager::startClock("RUN");
#ifndef _OPENMP
    for_each(vehicles->begin(), vehicles->end(),
             mem_fun(&Vehicle::run));
#else
    // 並列時は並列で走行処理後、非並列で移動後処理
    Random::multiStockReady(vehiclesSize);
#pragma omp parallel for schedule (dynamic)
    for (int i = 0; i < vehiclesSize; i++)
    {
        Random::multiStockBeginMulti(i);
        (*vehicles)[i]->clearRerouteCnt();
        (*vehicles)[i]->run();
    }
    Random::multiStockEndMulti();
    for_each(vehicles->begin(), vehicles->end(),
             mem_fun(&Vehicle::postRun));
    _rerouteCnt = 0;
    for (int i = 0; i < vehiclesSize; i++)
    {
        _rerouteCnt += (*vehicles)[i]->rerouteCnt();
    }
    if (_rerouteCnt > _rerouteRunMax)
    {
        _rerouteRunMax = _rerouteCnt;
    }
#endif
    TimeManager::stopClock("RUN");

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 時系列データ出力
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    writeRunInfo();
    if (_isWrite)
    {
        writeResult();
    }

#ifdef USE_ADDIN
    // アドインステップ終了
    aim.stepEnd();
#endif

    return true;
}

//======================================================================
void Simulator::generateVehicle()
{
    // 車両をに確実に発生させる（バス等）場合はここに処理を追加する

    // 確率的に車両の発生
    _generateVehicleRandom();
}

//======================================================================
void Simulator::generateVehicleManual(const string& startId,
				      const string& goalId,
				      vector<string> stopPoints,
				      VehicleType vehicleType,
				      vector<double> params)
{
    ODNode* start = dynamic_cast<ODNode*>(_roadMap->intersection(startId));
    if (start==NULL)
    {
        cout << "start:" << startId << " is not a OD node." << endl;
        return;
    }

    ODNode* goal = NULL;
    if (goalId!="******")
    {
        goal = dynamic_cast<ODNode*>(_roadMap->intersection(goalId));
    }
    if (goal==NULL)
    {
        goal = _decideGoalRandomly(start);
    }
    if (goal==NULL)
    {
        cout << "cannot decide goal." << endl;
        return;
    }
  
    assert(start && goal);

    OD od;
    od.setValue(start->id(), goal->id(), stopPoints);
    Vehicle* tmpVehicle
        = _createVehicle(start,
                         goal,
                         start,
                         start->nextSection(0),
                         &od,
                         vehicleType,
                         params);

    // startの_waitingVehiclesの先頭に加える
    /*
     * 登場は_generateVehicleRandomの中で一括して行う
     */
    start->addWaitingVehicleFront(tmpVehicle);

    tmpVehicle->route()->print(cout);
}

//======================================================================
void Simulator::generateSampleVehicles(unsigned int numVehicles,
				       double headway)
{
    assert(_roadMap);

    // ODノード
    vector<ODNode*> nodes = _roadMap->odNodes();

    // レーン1本あたりの車両台数を求める
    /*
     * 単路には上下1本ずつのレーンが含まれ、
     * ODノードに直接接続するレーンには車両を配置しないため、
     * 車両を配置すべきレーン総数は
     * (セクション総数)x2-（ODノード総数）
     */
    int ave = (int)ceil(static_cast<double>(numVehicles)
                        / (_roadMap->sections()->size()*2-nodes.size()));
    int id = 0;
  
    CITRMAPS its = _roadMap->sections()->begin();
    while (its != _roadMap->sections()->end())
    {
        map<string, Lane*, less<string> >::const_iterator itl
            = (*its).second->lanes()->begin();
        while (itl != (*its).second->lanes()->end())
        {
            double length = (*itl).second->length();

            // レーン上流にある交差点
            Intersection* prev
                = (*its).second->intersection
                (!((*its).second->isUp((*itl).second)));
            // レーン下流にある交差点
            Intersection* next
                = (*its).second->intersection
                ((*its).second->isUp((*itl).second));

            // 下流がODノードであるレーンには発生させない
            if (dynamic_cast<ODNode*>(next))
            {
                itl++;
                continue;
            }

            // ここまでで車両を配置すべき単路とレーンが決定された
            for (int i=0;
                 i<ave && static_cast<unsigned int>(id)<numVehicles;
                 i++)
            {
                // 車両生成処理を行う
	
                // 起点(ダミーの場合もある)の指定
                ODNode* start = NULL;
                if (dynamic_cast<ODNode*>(prev))
                {
                    start = dynamic_cast<ODNode*>(prev);
                }
                else
                {
                    start = nodes[Random::uniform(0,nodes.size())];
                }

                // 終点(ランダム)の指定
                ODNode* goal = _decideGoalRandomly(start);

                // 経由地情報の設定
                OD od;
                if (dynamic_cast<ODNode*>(prev))
                {
                    od.setValue(start->id(), goal->id());
                }
                else
                {
                    vector<string> stopPoints;
                    stopPoints.push_back(prev->id());
                    stopPoints.push_back(next->id());
                    od.setValue(start->id(), goal->id(),stopPoints);
                    od.setLastPassedStopPoint(prev->id());
                }

                // 車両生成
                Vehicle* tmpVehicle
                    = _createVehicle(start,
                                     goal,
                                     prev,
                                     (*its).second,
                                     &od,
                                     VehicleFamily::passenger());

                // 車両の配置
                length -= tmpVehicle->bodyLength()*0.5;

                tmpVehicle->addToSection(_roadMap, (*its).second, (*itl).second,
                                         (*itl).second->length()
                                         -(headway+tmpVehicle->bodyLength())*i
                                         -tmpVehicle->bodyLength()*0.5);
                const_cast<Route*>(tmpVehicle->route())->setLastPassedIntersection(prev);
                bool result = ObjManager::addVehicleToReal(tmpVehicle);
                assert(result);

                length -= (headway + tmpVehicle->bodyLength()*0.5); 
                if (length<tmpVehicle->bodyLength())
                    break;

                id++;
            }
            itl++;
        }
        its++;
    }
}

//======================================================================
void Simulator::_generateVehicleRandom()
{
    // 車両発生テーブルに現在時刻を通知する
    _table.setTime(TimeManager::time());
    _defaultTable.setTime(TimeManager::time());

    vector<ODNode*> odNodes = _roadMap->odNodes();

#ifndef _OPENMP
    for (int i=0; i<static_cast<signed int>(odNodes.size()); i++)
#else
    int nodesSize = static_cast<signed int>(odNodes.size());
    _rerouteCnt = 0;
    ulint step = TimeManager::step();
    // 車両集中発生、車両の登場は毎回行う、この関数の末尾
    if (_genIntensiveStep == 0)
    {
        Random::multiStockReady(nodesSize);
#pragma omp parallel for schedule (dynamic)
        for (int i=0; i < nodesSize; i++)
        {
            Random::multiStockBeginMulti(i);
            for (int j = 0; j < _genIntensive; j++)
#endif
    {
        // 各ODノードの処理
        // 車両発生はGenerate Table, Default Generate Tableで指定された
        // 1つのCellごとに，1[veh./step]まで発生させることができる．
        ODNode* start = odNodes[i];
        ODNode* goal = NULL;
        
        if (_odNodeStartLevel(start)<0)
        {
            // ODノードからの流出点がない = Destinationにしかなり得ない
            continue;
        }
        
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // ODの決定と車両の生成・登録
        // isEnoughSpaceはAddToRealの段階で判断
        
        // ファイルにより交通量が指定されているか
        // 指定されていない場合にはdefaultTrafficVolumeに従ってランダム生成
        bool hasActiveGTCells = false;
        
        OD od;
        int volume;
        VehicleType vehicleType;
        vector<const GTCell*> cells;
        // Generate Tableに基づく車両発生
        _table.getActiveGTCells(start->id(), &cells);
        if (cells.size()!=0)
        {
            hasActiveGTCells = true;
            
            for (unsigned int i=0; i<cells.size(); i++)
            {
                od = *cells[i]->od();
                volume = cells[i]->volume();
                vehicleType = cells[i]->vehicleType();
                if (volume!=0)
                {
                    if (Random::uniform()
                        <volume/60.0/60.0/1000.0*TimeManager::unit()){
                        goal
                            = dynamic_cast<ODNode*>(_roadMap->intersection(od.goal()));
                        assert(goal);
                        od = *cells[i]->od();
                        Vehicle* tmpVehicle
                            = _createVehicle(start,
                                             goal,
                                             start,
                                             start->nextSection(0),
                                             &od,
                                             vehicleType);
#ifdef _OPENMP
                        tmpVehicle->setGenIntensiveStep(step + j);
#endif
                        start->addWaitingVehicle(tmpVehicle);
                    }
                }
            }
        }
                
        // Default Generate Tableに基づく車両発生
        _defaultTable.getActiveGTCells(start->id(), &cells);
        if (cells.size()!=0)
        {
            hasActiveGTCells = true;
                    
            for (unsigned int i=0; i<cells.size(); i++)
            {
                volume = cells[i]->volume();
                vehicleType = cells[i]->vehicleType();
                if (volume!=0)
                {
                    if (Random::uniform()
                        <volume/60.0/60.0/1000.0*TimeManager::unit())
                    {
                        goal = _decideGoalRandomly(start);
                        assert(goal);
                        const OD* pOD = cells[i]->od();
                        od.setValue(pOD->start(), goal->id(), *pOD->stopPoints());
                        Vehicle* tmpVehicle
                            = _createVehicle(start,
                                             goal,
                                             start,
                                             start->nextSection(0),
                                             &od,
                                             vehicleType);
#ifdef _OPENMP
                        tmpVehicle->setGenIntensiveStep(step + j);
#endif
                        start->addWaitingVehicle(tmpVehicle);
                    }
                }
            }
        }

        // Generate Table, Default Generate Tableの両方で指定されていない場合
        // 用意されたdefaultTrafficVolumeの情報を用いる
        /*
         * Generate Table, Default Generate Tableにおいて
         * volume=0が指定されている場合には車両は発生しない
         * 逆にファイルで指定しない場合にはdefaultの交通量に従って車両が発生するため
         * 車両を発生させたくないODノードがある場合には注意．
         */
        if (hasActiveGTCells==false
            && _isRandomVehicleGenerate)
        {
            if (Random::uniform()
                <_defaultTrafficVolume[_odNodeStartLevel(start)]
                /60.0/60.0/1000.0*TimeManager::unit())
            {
                goal = _decideGoalRandomly(start);
                if(Random::uniform() < 0.7)
                {
                    vehicleType = VehicleFamily::passenger();
                }
                else
                {
                    vehicleType = VehicleFamily::truck();
                }
                assert(goal);
                od.setValue(start->id(), goal->id());
                Vehicle* tmpVehicle
                    = _createVehicle(start,
                                     goal,
                                     start,
                                     start->nextSection(0),
                                     &od,
                                     vehicleType);
#ifdef _OPENMP
                tmpVehicle->setGenIntensiveStep(step + j);
#endif
                start->addWaitingVehicle(tmpVehicle);
            }
        }
        
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // 車両の登場
        /*
         * 渋滞などの理由により生成した車両が即座に登場できない場合があるので
         * ODNodeの_waitingVehiclesが空でないステップでは登場を試みる
         * 並列時は後で非並列でまとめて行う
         */
#ifndef _OPENMP
        if (start->hasWaitingVehicles())
        {
            start->pushVehicleToReal(_roadMap, _isWrite);
        }
#endif
    }
#ifdef _OPENMP
        }  // end parallel
        Random::multiStockEndMulti();
    }
#endif

#ifdef _OPENMP
    if (_rerouteCnt > _rerouteGenMax)
    {
        _rerouteGenMax = _rerouteCnt;
    }
    for (int i=0; i < nodesSize; i++)
    {
        ODNode* start = odNodes[i];
        if (_odNodeStartLevel(start)<0)
        {
            continue;
        }
        if (start->hasWaitingVehicles())
        {
            start->pushVehicleToReal(_roadMap, _isWrite, step);
        }
    }
    _genIntensiveStep++;
    if (_genIntensiveStep >= _genIntensive)
    {
        _genIntensiveStep = 0;
    }
#endif
}


//======================================================================
Vehicle* Simulator::_createVehicle(ODNode* start,
                                   ODNode* goal,
                                   Intersection* past,
                                   Section* section,
                                   OD* od,
                                   VehicleType vehicleType){
    assert(start!=NULL && goal!=NULL);

    // 車両の生成 
    Vehicle* tmpVehicle = ObjManager::createVehicle();

    //車両属性の設定
    tmpVehicle->setType(vehicleType);
    _setVehicleStatus(tmpVehicle);
  
    // 経路選択用のパラメータの設定
    tmpVehicle->router()->setTrip(start, goal, od, _roadMap);
    tmpVehicle->router()
        ->setParam(_vehicleRoutingParams
                   [Random::uniform(0,_vehicleRoutingParams.size())]);


    // 経路選択は関数の外に出す
    tmpVehicle->reroute(section, past);

#ifdef _OPENMP
#pragma omp critical (Simulator)
    {
        _rerouteCnt++;
    }
#endif

    return tmpVehicle;
}

//======================================================================
Vehicle* Simulator::_createVehicle(ODNode* start,
                                   ODNode* goal,
                                   Intersection* past,
                                   Section* section,
                                   OD* od,
                                   VehicleType vehicleType,
                                   vector<double> params){
    assert(start!=NULL && goal!=NULL);

    // 車両の生成 
    Vehicle* tmpVehicle = ObjManager::createVehicle();

    //車両属性の設定
    tmpVehicle->setType(vehicleType);
    _setVehicleStatus(tmpVehicle);
  
    // 経路選択用のパラメータの設定
    vector<double> p;
    tmpVehicle->router()->setTrip(start, goal, od, _roadMap);
    for (int i=0; i<VEHICLE_ROUTING_PARAMETER_SIZE; i++)
    {
        p.push_back(_vehicleRoutingParams[0][i]);
    }
    for (unsigned int i=0; i<params.size()&&i<p.size(); i++)
    {
        p[i] = params[i];
    }
    tmpVehicle->router()->setParam(p);

    // 経路選択
    tmpVehicle->reroute(section, past);

    return tmpVehicle;
}

//======================================================================
void Simulator::_setVehicleStatus(Vehicle* vehicle)
{
    double length, width, height;
    double accel, decel;
    double r, g, b;

    // 車種に対応付けられた属性を取得する
    /*
     * ファイルから読み込んだ属性を適用するように変更
     * 2014/6/8 by H.Fujii
     */
    VFAttribute* vfa
        = VehicleFamilyManager::vehicleFamilyAttribute(vehicle->type());
    if (!vfa)
    {
        if (VehicleFamily::isTruck(vehicle->type()))
        {
            vfa = VehicleFamilyManager::vehicleFamilyAttribute
                (VehicleFamily::truck());
        }
        else
        {
            vfa = VehicleFamilyManager::vehicleFamilyAttribute
                (VehicleFamily::passenger());
        }
    }
    vfa->getSize(&length, &width, &height);
    vfa->getPerformance(&accel, &decel);
    vfa->getBodyColor(&r, &g, &b);

    vehicle->setBodySize(length, width, height);
    vehicle->setPerformance(accel, decel);
    vehicle->setBodyColor(r, g, b);
}

//======================================================================
ODNode* Simulator::_decideGoalRandomly(ODNode* start)
{
    ODNode* result = NULL;

    // start以外のODノードを_goalLevelからコピーする
    // 複数ネットワークを許す場合には，startから到達可能かどうかもチェックする
    vector<ODNode*> goals[3];
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<static_cast<signed int>(_goalLevel[i].size()); j++)
        {
#ifdef UNIQUE_NETWORK
            if (start!=_goalLevel[i][j])
            {
                goals[i].push_back(_goalLevel[i][j]);
            }
#else  //UNIQUE_NETWORK
            if (start!=_goalLevel[i][j]
                && start->isNetworked(start,_goalLevel[i][j]))
            {
                goals[i].push_back(_goalLevel[i][j]);
            }
#endif //UNIQUE_NETWORK
        }
    }
    int level[3];
    for(int i=0; i<3; i++)
    {
        level[i] = _defaultTrafficVolume[i]; 
        if (goals[i].size()==0)
        {
            level[i] = 0;
        }
    }

    int total = level[0] + level[1] + level[2];
    int r = Random::uniform(RAND_MAX) % total;
    for (int i=0; i<3; i++)
    {
        if (r<level[i])
        {
            result = goals[i][Random::uniform(goals[i].size())];
        } 
        else
        {
            r -= level[i];
        }
    }
    assert(result);
    return result;
}

//======================================================================
int Simulator::_odNodeStartLevel(ODNode* node) const
{
    int result = -1;
    // ODノードから見たnumOut
    if (node->border(0)->numOut()>=3)
    {
        result = 0;
    } else if (node->border(0)->numOut()==2)
    {
        result = 1;
    } else if (node->border(0)->numOut()==1)
    {
        result = 2;
    }
    return result;
}

//======================================================================
int Simulator::_odNodeGoalLevel(ODNode* node) const
{
    // ODノードから見たnumIn
    int result = -1;
    if (node->border(0)->numIn()>=3)
    {
        result = 0;
    }
    else if (node->border(0)->numIn()==2)
    {
        result = 1;
    }
    else if (node->border(0)->numIn()==1)
    {
        result = 2;
    }
    return result;
}

//======================================================================
void Simulator::setVerbose(const bool isVerbose)
{
    _isVerbose = isVerbose;
}

//======================================================================
void Simulator::setWriteState(const bool isWrite)
{
    _isWrite = isWrite;
}

//======================================================================
void Simulator::setMonitorState(const bool isMonitorD,
                                const bool isMonitorS)
{
    _isMonitorD = isMonitorD;
    _isMonitorS = isMonitorS;
}

//======================================================================
void Simulator::setGenCountState(const bool isGenCount)
{
    _isGenCount = isGenCount;
}

//======================================================================
void Simulator::setInputState(const bool isMapInput,
                              const bool isSignalInput,
                              const bool isVehicleInput)
{
    _isMapInput     = isMapInput;
    _isSignalInput  = isSignalInput;
    _isVehicleInput = isVehicleInput;
}

//======================================================================
void Simulator::setOutputState(const bool isWrite,
                               const bool isMonitorD,
                               const bool isMonitorS,
                               const bool isGenCount,
                               const bool isTripInfo)
{
    _isWrite = isWrite;
    _isMonitorD = isMonitorD;
    _isMonitorS = isMonitorS;
    _isGenCount = isGenCount;
    _isTripInfo = isTripInfo;
}

//======================================================================
void Simulator::setAgentState(const bool isRandomVehicleGenerate)
{
    _isRandomVehicleGenerate = isRandomVehicleGenerate;
}

//======================================================================
void Simulator::writeResult() const
{
    const map<string, Signal*, less<string> >* signals
        = _roadMap->signals();
    _signalIO
        ->writeSignalsDynamicData(TimeManager::time(), signals);

    vector<Vehicle*>* vehicles = ObjManager::vehicles();
    _vehicleIO
        ->writeVehiclesDynamicData(TimeManager::time(), vehicles);
}

//======================================================================
void Simulator::writeRunInfo() const
{
    GVManager& gvm = GVManager::instance();
    string fRunInfo;
    gvm.getVariable("RESULT_RUN_INFO_FILE", &fRunInfo);
    ofstream ofs(fRunInfo.c_str(), ios::trunc);
  
    if (!ofs.fail())
    {
        ofs << TimeManager::step() << "\n"
            << TimeManager::unit();
        ofs.close();
    }
}

//======================================================================
RoadMap* Simulator::roadMap()
{
    return _roadMap;
}

