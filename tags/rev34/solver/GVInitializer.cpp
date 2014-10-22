#include "GVInitializer.h"
#include "GVManager.h"
#include <iostream>

using namespace std;

//======================================================================
void GVInitializer::init(const string& dataPath)
{
    GVManager& gvm = GVManager::instance();

    gvm.setNewVariable("DATA_DIRECTORY",
                       dataPath);

    // グローバル変数設定ファイル
    gvm.setNewVariable("GV_INIT_FILE",
                       dataPath + "init.txt");

    // 地図に関連するファイル
    gvm.setNewVariable("MAP_POSITION_FILE",
                       dataPath + "mapPosition.txt");
    gvm.setNewVariable("MAP_NETWORK_FILE",
                       dataPath + "network.txt");
    gvm.setNewVariable("SPEED_LIMIT_FILE",
                       dataPath + "speedLimit.txt");

    // 車両発生に関するファイル
    gvm.setNewVariable("GENERATE_TABLE", 
                       dataPath + "generateTable.txt");
    gvm.setNewVariable("DEFAULT_GENERATE_TABLE", 
                       dataPath + "defaultGenerateTable.txt");

    // 車種に関するファイル
    gvm.setNewVariable("VEHICLE_FAMILY_FILE",
                       dataPath + "vehicleFamily.txt");

    // 自動車エージェントの経路に関するファイル
    gvm.setNewVariable("VEHICLE_ROUTE_PARAM_FILE",
                       dataPath + "vehicleRoutingParam.txt");

    // 路側器に関するファイル
    gvm.setNewVariable("GENCOUNTER_FILE",
                       dataPath + "genCounter.txt");
    gvm.setNewVariable("DETECTOR_FILE",
                       dataPath + "detector.txt");
 
    // 信号に関するディレクトリorファイルor拡張子
    gvm.setNewVariable("SIGNAL_CONTROL_DIRECTORY",
                       dataPath + "signals/");
    gvm.setNewVariable("SIGNAL_CONTROL_FILE_DEFAULT",
                       "default");
    gvm.setNewVariable("SIGNAL_ASPECT_FILE_DEFAULT_PREFIX",
                       "defaultInter");
    gvm.setNewVariable("CONTROL_FILE_EXTENSION",
                       ".msf");
    gvm.setNewVariable("ASPECT_FILE_EXTENSION",
                       ".msa");

    // 交差点属性指定ファイル
    gvm.setNewVariable("INTERSECTION_ATTRIBUTE_DIRECTORY",
                       dataPath+"intersection/");

    // 道路形状に関するファイル
    gvm.setNewVariable("INTERSECTION_STRUCT_FILE",
                       dataPath + "intersectionStruct.txt");
    gvm.setNewVariable("SECTION_STRUCT_FILE",
                       dataPath + "sectionStruct.txt");

    // 出力先
    string resultPath = dataPath + "result/";
    gvm.setNewVariable("RESULT_OUTPUT_DIRECTORY",
                       resultPath);
    gvm.setNewVariable("RESULT_TIMELINE_DIRECTORY",
                       resultPath + "timeline/");
    gvm.setNewVariable("RESULT_IMG_DIRECTORY",
                       resultPath + "img/");
    gvm.setNewVariable("RESULT_INSTRUMENT_DIRECTORY",
                       resultPath + "inst/");
    gvm.setNewVariable("RESULT_DETECTORD_PREFIX",
                       "detD");
    gvm.setNewVariable("RESULT_DETECTORS_PREFIX",
                       "detS");
    gvm.setNewVariable("RESULT_GENCOUNTER_PREFIX",
                       "gen");
    gvm.setNewVariable("RESULT_RUN_INFO_FILE",
                       resultPath + "run_info.txt");
    gvm.setNewVariable("RESULT_NODE_INFO_FILE",
                       resultPath + "node_info.txt");
    gvm.setNewVariable("RESULT_LINK_INFO_FILE",
                       resultPath + "link_info.txt");
    gvm.setNewVariable("RESULT_ROAD_ENTITY_INFO_FILE",
                       resultPath + "road_entity_info.txt");

    gvm.setNewVariable("RESULT_GLOBAL_STATIC_FILE",
                       resultPath + "global_static.txt");
    gvm.setNewVariable("RESULT_GLOBAL_DYNAMIC_FILE",
                       resultPath + "global_dynamic.txt");

    gvm.setNewVariable("RESULT_VEHICLE_STATIC_FILE",
                       "vehicle_static.txt");
    gvm.setNewVariable("RESULT_VEHICLE_DISTANCE_FILE",
                       "vehicle_distance.txt");

#ifdef USE_ADDIN
    // アドイン
    gvm.setNewVariable("ADDIN_LIB_NAME",
                       ""); // lib<name>.so
#endif

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 定数の定義
    /*
     * 後でGVManager::setVariablesFromFile()によって上書きされる．
     */

    /* シミュレーションの実行に関するもの */

    // 出力ファイルに各種コメントを付して出力する
    gvm.setNewVariable("OUTPUT_COMMENT_IN_FILE", 0);

    // 計時の有無、0 以外ならあり
    gvm.setNewVariable("CHECK_TIME", 0);

    /* エージェントに関するもの */

    // 自動車の反応遅れ時間[sec]
    gvm.setNewVariable("REACTION_TIME_VEHICLE", 0.74);

    // 自動車が交差点で右折する場合などの
    // 対向車とのギャップアクセプタンス[sec]
    gvm.setNewVariable("GAP_ACCEPTANCE_VEHICLE_CROSS", 3.0);

    // 普通車(VehicleFamily::passenger())の最大加速度，減速度[m/(sec^2)]
    gvm.setNewVariable("MAX_ACCELERATION_PASSENGER", 3.0);
    gvm.setNewVariable("MAX_DECELERATION_PASSENGER", -5.0);

    // バス(VehicleFamily::bus())の最大加速度，減速度[m/(sec^2)]
    gvm.setNewVariable("MAX_ACCELERATION_BUS", 3.0);
    gvm.setNewVariable("MAX_DECELERATION_BUS", -5.0);

    // 大型車(VehicleFamily::truck())の最大加速度，減速度[m/(sec^2)]
    gvm.setNewVariable("MAX_ACCELERATION_TRUCK", 3.0);
    gvm.setNewVariable("MAX_DECELERATION_TRUCK", -5.0);

    // 車線変更時に与える横向きの速度[km/h]
    gvm.setNewVariable("ERROR_VELOCITY", 7.5);

    // 発生端点からL[m]以内の車両は出力しない
    gvm.setNewVariable("NO_OUTPUT_LENGTH_FROM_ORIGIN_NODE", 0);

    // 車両ログの追加情報を出力する
    gvm.setNewVariable("OUTPUT_VEHICLE_EXTENSION", 0);

    // 普通車(VehicleFamily::passenger())のサイズ[m]
    gvm.setNewVariable("VEHICLE_LENGTH_PASSENGER", 4.400);
    gvm.setNewVariable("VEHICLE_WIDTH_PASSENGER",  1.830);
    gvm.setNewVariable("VEHICLE_HEIGHT_PASSENGER", 1.315);

    // バス(VehicleFamily::bus())のサイズ[m]
    gvm.setNewVariable("VEHICLE_LENGTH_BUS", 8.465);
    gvm.setNewVariable("VEHICLE_WIDTH_BUS",  2.230);
    gvm.setNewVariable("VEHICLE_HEIGHT_BUS", 3.420);

    // 大型車(VehicleFamily::truck())のサイズ[m]
    gvm.setNewVariable("VEHICLE_LENGTH_TRUCK", 8.465);
    gvm.setNewVariable("VEHICLE_WIDTH_TRUCK",  2.230);
    gvm.setNewVariable("VEHICLE_HEIGHT_TRUCK", 3.420);

    // 車線変更中に先のレーンを見る
    gvm.setNewVariable("LOOKUP_SHIFT_LANE", 1);

    // 交錯を厳密に評価
    gvm.setNewVariable("STRICT_COLLISION_CHECK", 1);

    /* 道路に関するもの */

    // 右折専用レーンの標準長さ[m]
    gvm.setNewVariable("RIGHT_TURN_LANE_LENGTH", 30);

    // 標準制限速度[km/h]
    /* ただしSPEED_LIMIT_INTERSECTIONが用いられることはほとんど無く，
     * 右左折時は下のVELOCITY_AT〜が使われ，
     * 直進時は次の単路でのSPEED_LIMITが参照される．*/
    gvm.setNewVariable("SPEED_LIMIT_SECTION", 60);
    gvm.setNewVariable("SPEED_LIMIT_INTERSECTION", 60);

    // 徐行速度[km/h]
    gvm.setNewVariable("VELOCITY_CRAWL", 10);

    // 右左折時の速度[km/h]
    gvm.setNewVariable("VELOCITY_AT_TURNING_RIGHT", 40);
    gvm.setNewVariable("VELOCITY_AT_TURNING_LEFT",  40);

    // 車両発生時の制限速度[km/h]、負ならなし
    gvm.setNewVariable("GENERATE_VELOCITY_LIMIT", -1);

    // 右左折時の最小ヘッドウェイ[秒]
    gvm.setNewVariable("MIN_HEADWAY_AT_TURNING", 1.7);

    // 車両発生量既定値[台/h]、入口レーン 3 以上 / 2 / 1、基本交通容量の10%
    gvm.setNewVariable("DEFAULT_TRAFFIC_VOLUME_WIDE",   660);
    gvm.setNewVariable("DEFAULT_TRAFFIC_VOLUME_NORMAL", 440);
    gvm.setNewVariable("DEFAULT_TRAFFIC_VOLUME_NARROW", 125);

    // 標準のレーン幅、歩道幅、横断歩道幅、路側幅
    gvm.setNewVariable("DEFAULT_LANE_WIDTH",      3.5);
    gvm.setNewVariable("DEFAULT_SIDEWALK_WIDTH",  5.0);
    gvm.setNewVariable("DEFAULT_CROSSWALK_WIDTH", 5.0);
    gvm.setNewVariable("DEFAULT_ROADSIDE_WIDTH",  1.0);

    // 単路の歩道を自動設定する際のレーン数、-1 なら自動設定なし
    /* 自動設定ありの場合、単路の全レーン数がこれ以上なら歩道を設定する
     * これ以下なら車道通行可能にする */
    gvm.setNewVariable("AUTO_SIDEWALK_SECTION_LANE", -1);

    // 道路エンティティの厳密な内部判定、1 ならあり
    /* 凹型の道路エンティティでも判定可能になる、ただし処理速度は遅い */
    gvm.setNewVariable("ROAD_ENTITY_STRICT_JUDGE_INSIDE", 1);

    // 交差点サイズ制限、中心からの距離[m]
    gvm.setNewVariable("INTERSECTION_SIZE_LIMIT", 20);

#ifdef USE_ADDIN
    /* アドインに関するもの */

    // アドイン経路変更禁止長さ
    gvm.setNewVariable("ADDIN_ROUTE_BAN_LENGTH", 10);

    // アドイン歩行者長期停止時間[ms]
    gvm.setNewVariable("ADDIN_WALKER_SLEEP_TIME", 5000);
#endif

#ifdef _OPENMP
    /* マルチスレッドに関するもの */

    // スレッド数，0ならomp_get_num_procs()が返すプロセッサ数
    gvm.setNewVariable("THREAD_NUM", 0);

    // 集中車両発生
    gvm.setNewVariable("GENERATE_INTENSIVE", 1);

    // 経路検索最大ステップ
    gvm.setNewVariable("ROUTER_SEARCH_STEP", 10000);

    // 乱数並列ストック、0 なら自動、並列実行時のみストック
    gvm.setNewVariable("RANDOM_MULTI_STOCK", 0);
#endif
}
