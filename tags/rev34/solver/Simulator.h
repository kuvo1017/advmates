#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "RoadMap.h"
#include "TimeManager.h"
#include "GeneratingTable.h"
#include "VehicleFamily.h"
#include "VehicleIO.h"
#include "SignalIO.h"

class ODNode;
class Vehicle;

/**
 * @addtogroup Running
 * @brief シミュレーションの実行
 * @ingroup Procedure
 */

/// シミュレータ本体
/**
 * @ingroup Initialization Running
 */
class Simulator
{
public:
    Simulator();
    ~Simulator();

    //====================================================================
    /** @name シミュレーションの準備 */
    //@{

    /// 初期化は適切に完了しているか
    bool hasInit() const;

    /// ファイルを読み込んでRoadMapを作成する
    /**
     * @return 作成に成功したかどうか
     */
    bool getReadyRoadMap();

    /// RoadsideUnitに関する初期設定を行う
    /**
     * @return 設定に成功したかどうか
     */
    bool getReadyRoadsideUnit();

    /// Vehicleに関する初期設定を行う
    /**
     * @return 設定に成功したかどうか
     */
    bool getReadyVehicles();

    /// Router用パラメータの設定
    void readRouteParameter();

    /// ファイルを"読み込まずに"RoadMapを作成する
    /**
     * (@p xmin, @p ymin) - (@p xmax, @p ymax)の地域に
     * x方向の間隔 @p xsp，y方向の間隔 @p yspの格子状道路を生成し，
     * @p numVehicles台の車両を間隔@p headwayで配置する（単位は[m]）．
     */
    bool getReadySampleScenario(double xmin, double xmax,
                                double ymin, double ymax,
                                double xsp,  double ysp,
                                unsigned int numVehicles,
                                double headway);

    /// サンプルシナリオをスタートする
    /**
     * 外部からこの関数を呼び出すことを想定している
     */
    void startSampleScenario(double xmin, double xmax,
                             double ymin, double ymax,
                             double xsp,  double ysp,
                             unsigned int numVehicles,
                             double headway);
  
    /// レーンチェック、エラー時は表示確認のため run のみ止める
    void checkLane();

    /// レーンチェックエラー取得
    bool checkLaneError();

    //@}

    //====================================================================
    /** @name シミュレーションの実行 */
    //@{

    /// @p time までシミュレータを動かす
    bool run(ulint time);

    /// 1ステップ分シミュレータを動かす
    bool timeIncrement();

    //@}

    //======================================================================
    /** @name エージェントの発生と消去 */
    //@{

    /// 車両を発生させる
    void generateVehicle();

    /// 手動で車両を発生させる
    void generateVehicleManual(const std::string& startId,
                               const std::string& goalId,
                               std::vector<std::string> stopPoints,
                               VehicleType vehicleType,
                               std::vector<double> params);

    /// サンプルシナリオにおいて車両の初期配置を行う
    void generateSampleVehicles(unsigned int numVehicles,
                                double headway);
    //@}

private:
    //======================================================================
    /** @name エージェントの発生と消去に用いる */
    //@{

    /// 確率的に車両を発生させる
    void _generateVehicleRandom();

    /// 車両を生成し，経路選択する
    /**
     * 乱数によって車両生成フラグが立った後の処理．
     * ObjManager::createVehicle()で車両を生成し，経路選択．
     */
    Vehicle* _createVehicle(ODNode* start,
                            ODNode* goal,
                            Intersection* past,
                            Section* section,
                            OD* od,
                            VehicleType vehicleType);

    /// 車両を生成し，経路選択する
    /**
     * 引数として経路選択パラメータを与える．
     */
    Vehicle* _createVehicle(ODNode* start,
                            ODNode* goal,
                            Intersection* past,
                            Section* section,
                            OD* od,
                            VehicleType vehicleType,
                            std::vector<double> params);

    /// 車両の属性を設定
    ///
    void _setVehicleStatus(Vehicle* vehicle);

    /// ゴールをランダムに決定する
    ODNode* _decideGoalRandomly(ODNode* start);

    /// ODノードのスタートレベルを返す
    /**
     * 単路への流入点でstartLevelを決定する
     * "単路の"流入点、流出点は"ODノードの"流入点、流出点と反対
     */
    int _odNodeStartLevel(ODNode* node) const;

    /// ODノードのゴールレベルを返す
    /**
     * 単路からの流出点でgoalLevelを決定する
     * "単路の"流入点、流出点は"ODノードの"流入点、流出点と反対
     */
    int _odNodeGoalLevel(ODNode* node) const;

    //@}

public:
    //====================================================================
    /** @name ファイル入出力とフラグの管理 */
    //@{

    /// 画面に詳細な情報を出力するかどうかセットする
    void setVerbose(const bool isVerbose);

    /// 入力フラグを変更する
    void setInputState(const bool isMapInput,
                       const bool isSignalInput,
                       const bool isVehicleInput);

    /// 出力フラグを変更する
    void setWriteState(const bool isWrite);
    /// 出力フラグを変更する
    void setMonitorState(const bool isMonitorD, const bool isMonitorS);
    /// 出力フラグを変更する
    void setGenCountState(const bool isGenCount);
    /// 出力フラグを変更する
    void setOutputState(const bool isWrite,
                        const bool isMonitorD,
                        const bool isMonitorS,
                        const bool isGenCount,
                        const bool isTripInfo);

    /// エージェント発生フラグを変更する
    void setAgentState(const bool isRandomVehicleGenerate);

    /// 時系列データを出力する
    void writeResult() const;

    /// run_infoを出力する
    /**
     * @note
     * 本来はシミュレーション終了時に出力すればよいはずだが，
     * 実行時エラーが発生したbりCtrl-Cで強制終了した場合に対応するため
     * 各タイムステップの処理が終わるたびに更新することにする．
     */
    void writeRunInfo() const;

    //@}

    //====================================================================
    /** @name 内部の地図やエージェントへのアクセッサ */
    //@{

    /// 地図を返す
    RoadMap* roadMap();

    //@}

private:
    /// 地図オブジェクト
    RoadMap* _roadMap;

    /** @name 車両発生定義テーブル */
    //@{
    GeneratingTable _table;        //< 発生地点目的地点の双方を設定
    GeneratingTable _defaultTable; //< 発生点のみ設定
    //@}

    /// レベル分けされたOriginノード
    std::vector<ODNode*> _startLevel[3];

    /// レベル分けされたDestinationノード
    std::vector<ODNode*> _goalLevel[3];

    /// デフォルトのレベル別交通量
    int _defaultTrafficVolume[3];

    /// Router用パラメータセット
    std::vector<std::vector<double> > _vehicleRoutingParams;

    /// レーンチェックエラー、表示確認のため run のみ止める
    bool _checkLaneError;

    //====================================================================
    /** @name 入出力用オブジェクトとフラグ*/
    /// @{

    /// 画面に詳細情報を出力するか
    bool _isVerbose;

    /// 地図データを入力するか
    bool _isMapInput;

    /// 信号データを入力するか
    bool _isSignalInput;

    /// 車両データを入力するか
    bool _isVehicleInput;

    /// 時系列データを出力するか
    bool _isWrite;

    /// 車両発生情報が指定されていない交差点から車両を発生させるか
    bool _isRandomVehicleGenerate;

    /// 車両情報の入出力を担当するオブジェクト
    VehicleIO* _vehicleIO;

    /// 信号情報の入出力を担当するオブジェクト
    SignalIO* _signalIO;

    /// 路側機によってデータを観測し，結果を出力するか
    bool _isMonitorD;
    bool _isMonitorS;

    /// 車両発生カウンタのデータを出力するか
    bool _isGenCount;

    /// 走行距離と旅行時間を出力するか
    bool _isTripInfo;

#ifdef _OPENMP
    ///負荷確認のための reroute カウンタと最大値
    int _rerouteCnt;
    int _rerouteGenMax;
    int _rerouteRunMax;

    /// 車両集中発生数
    int _genIntensive;
    /// 車両集中発生ステップ数
    int _genIntensiveStep;
#endif
    //@}
};

#endif //__SIMULATOR_H__
