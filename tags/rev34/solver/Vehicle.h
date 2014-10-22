#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include <string>
#include "RoadOccupant.h"
#include "AmuVector.h"
#include "LocalLaneRouter.h"
#include "LocalLaneRoute.h"
#include "Blinker.h"
#include "VehicleFamily.h"
#include "VehicleShift.h"
#include "RelativeDirection.h"
#include "TimeManager.h"

class RoadMap;
class LaneBundle;
class Intersection;
class Section;
class Lane;
class ARouter;
class Route;
class VirtualLeader;

/**
 * @addtogroup Vehicle
 * @brief 自動車エージェント
 * @ingroup Agent
 */

/// 自動車エージェントの基底クラス
/**
 * @ingroup Vehicle
 */
class Vehicle : public RoadOccupant
{
public:
    Vehicle();
    virtual ~Vehicle();

    /// 認識前処理、車線変更等、非並列
    void preRecognize();

    /// 周囲の状況を認識する
    virtual void recognize();

    /// 移動のための準備を行う
    virtual void readyToRun();

    /// 移動する
    virtual void run();

#ifdef _OPENMP
    /// 移動後処理、経過時間変更等、非並列
    void postRun();
#endif

    //====================================================================
    /** @name 識別番号と車体の大きさに関する関数 */
    /// @{

    /// 識別番号を返す
    const std::string& id() const;
    // 識別番号を設定する
    void setId(const std::string& id);
    /// 車種を返す
    VehicleType type() const;
    /// 車種を設定する
    void setType(VehicleType type);
    /// 車体の幅を返す
    double bodyWidth() const;
    /// 車体の長さを返す
    double bodyLength() const;
    /// 車体の高さを返す
    double bodyHeight() const;
    /// 車体の大きさを設定する
    void setBodySize(double length, double width, double height);
    /// 性能を設定する
    void setPerformance(double accel, double brake);
    /// 車体色を設定する
    void setBodyColor(double r, double g, double b);
    /// 車体色を返す
    void getBodyColor(double* result_r,
                      double* result_g,
                      double* result_b) const;

    /// @}

    //====================================================================
    /** @name 道路上の位置に関する関数 */
    /// @{

    /// 単路@p sectionのレーン@p lane上の距離@p lengthに登場する
    /**
     * @p roadMapの登録も兼ねる(発生時のみ呼び出される)
     * バスクラスは独自処理をする
     */
    virtual bool addToSection(RoadMap* roadMap, Section* section,
		              Lane* lane, double length);

    /// レーンの始点からの距離を返す
    double length() const;
    /// 前タイムステップでの距離を返す
    double oldLength() const;
    /// 交差点や単路の始点からの距離を返す
    double totalLength() const;
    /// トリップ長を返す
    double tripLength() const;
    /// x座標を返す
    double x() const;
    /// y座標を返す
    double y() const;
    /// z座標を返す
    double z() const;

    /// 現在所属するレーン束を返す
    LaneBundle* laneBundle() const;

    /// 現在所属する単路を返す
    Section* section() const;

    /// 現在所属する交差点を返す
    Intersection* intersection() const;

    /// 現在所属するレーンを返す
    Lane* lane() const;

    /// 発生点から離れていて，結果に反映される自動車であるか
    /**
     * 発生時は速度が0であり，これは非現実的な設定である（本来は速度を持っているはず）．
     * これにより排気排出量に影響が生じる恐れがある．
     * 従って，発生点に近すぎる自動車は出力から除外する必要がある．
     *
     * 結果として出力しないだけであり，速度計算は通常通り行う．
     * なおその距離はNO_OUTPUT_LENGTH_FROM_ORIGIN_NODEというキーで
     * GVManagerの中で管理されている（初期値は0）．
     */
    bool isAwayFromOriginNode() const;

    /// @p interのレーン@p laneに自身をセットする
    bool setLane(Intersection* inter, Lane* lane, double length);
    /// @p sectioinのレーン@p laneに自身をセットする
    bool setLane(Section* section, Lane* lane, double length);

    /// @}
public:
    //====================================================================
    /** @name 動きに関する関数 */
    /// @{

    /// 速度を返す
    double velocity() const;

    /// 加速度を返す
    double accel() const;
    /// 速度の方向ベクトルを返す
    const AmuVector directionVector() const;

    /// 現在所属するレーン束オブジェクトに車線変更中であることを通知する
    void notify();
    /// 現在所属するレーン束オブジェクトに車線変更が終わったことを通知する
    void unnotify();

    /// ウィンカーを返す
    Blinker blinker() const;

    /// 次の交差点に流入する境界番号を返す
    int directionFrom() const;
    /// 次の交差点から流出する境界番号を返す
    int directionTo() const;

    /// 車線変更挙動定義オブジェクトを返す
    VehicleShift& shiftLane();

    /// 休止状態かどうか
    bool isSleep() const;

    /// 発生時刻を設定する
    void setStartTime(ulint startTime);

    /// 発生時刻を取得する
    ulint startTime() const;

    /// 仮想先行エージェントの集合を返す
    const std::vector<VirtualLeader*>* virtualLeaders() const;

protected:
    /// 相手を「見る」ことができるか
    /**
     * ここで言う「見る」とは、適用するアプリケーションによって
     * 目視によるものか、通信によるものか意味合いが異なる
     */
    bool _isVisible(Vehicle* other) const;


    /// 交差点への侵入を相手に譲るか
    bool _isYielding(Intersection* inter,
                     int thisDir,
                     int thatDir,
                     RelativeDirection turning,
                     Vehicle* other);

    /// 次に進むべきレーンを決める(run()で使用)
    void _decideNextLane(Intersection* inter, Lane* lane);

    /// 次に進むべきレーンを決める(run()で使用)
    void _decideNextLane(Section* section, Lane* lane);

    /// 交差点内でレーンを移る
    void _runIntersection2Intersection();

    /// 交差点から単路に移る
    void _runIntersection2Section();

    /// 単路内でレーンを移る
    void _runSection2Section();

    /// 単路から交差点に移る
    void _runSection2Intersection();

    /// @}
public:
    //======================================================================
    /** @name 経路探索に関する関数 */
    /// @{

    /// 経路を返す
    const Route* route() const;
    /// 経路探索オブジェクトを返す
    ARouter* router();
    /// 経路を探索して自身の_routeに設定する
    /**
     * @param start 探索を開始する交差点。
     * 初めて道路地図上に登場したときに呼び出される
     * @note 希望した経路で渋滞に巻き込まれたときにもRerouteする？
     */
    bool reroute(const Intersection* start);

    /// 経路を再探索して自身の_routeに設定する
    /**
     * @param section
     * @param start
     * 希望した経路を逸脱したときに呼び出される
     * sectionとstartを持たないと「車両がどちらの方向に向かっているか」分からないので追加
     */
    bool reroute(const Section* section, const Intersection* start);

    /// 交差点で通過するレーンの集合を返す
    const std::vector<Lane*>* lanesInIntersection() const;

#ifdef USE_ADDIN
    /// @}
public:
    //======================================================================
    /** @name アドインに関する関数 */
    /// @{

    /// 希望走行速度を返す
    double vMax() const;

    /// アドイン経路設定チェック、経路到着可能チェック、エラーなら false
    bool checkSetAddinRoute(std::vector<std::string>* idList);

    /// アドイン経路設定
    void setAddinRoute(std::vector<std::string>* idList, bool newRoute);

    /// アドイン任意データ作成、NULL ならエラー
    void* createAddinData(unsigned int size);

    /// アドイン任意データ取得、NULL ならなし
    void* getAddinData();
#endif

#ifdef _OPENMP
    /// @}
public:
    //======================================================================
    /** @name マルチスレッドに関する関数 */
    /// @{

    /// 集中発生ステップ数を返す
    ulint genIntensiveStep() const;
    /// 集中発生ステップ数を設定する
    void setGenIntensiveStep(ulint genIntensiveStep);

    /// reroute 回数を返す（負荷確認）
    int rerouteCnt() const;
    /// reroute 回数を消去する
    void clearRerouteCnt();
#endif

    /// @}
public:
    //======================================================================
    /** @name その他 */
    /// @{

    /// 情報を出力する
    void print() const;

    /// @}
protected:
    //====================================================================
    /** @name 識別番号と車体の大きさに関する変数 */
    /// @{

    /// 識別番号
    std::string _id;
    /// 車種
    /** 中身はint型．VehicleFamily.hを参照． */
    VehicleType _type;
    /// 車体の幅
    double _bodyWidth;
    /// 車体の長さ
    double _bodyLength;
    /// 車体の高さ
    double _bodyHeight;

    /// 車体色の赤成分
    double _bodyColorR;
    /// 車体色の緑成分
    double _bodyColorG;
    /// 車体色の青成分
    double _bodyColorB;

    /// @}
    //====================================================================
    /** @name 位置に関する変数 */
    /// @{

    /// 現在自分が存在する道路ネットワーク
    RoadMap* _roadMap;
    /// 自分がいる交差点
    Intersection* _intersection;
    /// 自分がいた交差点
    Intersection* _prevIntersection;
    /// 自分がいる単路
    Section* _section;
    /// 自分がいるレーン
    Lane* _lane;
    /// 次に進むレーン
    Lane* _nextLane;
    /// レーン始点からの長さ[m]
    double _length;
    /// 前タイムステップでのレーン始点からの距離[m]
    /**
     * 特定のポイントを通過したタイムステップを感知する際に用いる．
     */
    double _oldLength;
    /// 単路始点からの長さ[m]
    /**
     * runSection2**で0にする
     */
    double _totalLength;
    /// トリップ長[m]
    /**
     * 発生してから消滅するまでの総走行距離
     */
    double _tripLength;

    /// レーン中心線からのずれ[m]
    double _error;

    /// 発生時刻
    ulint _startTime;

    /// @}
    //====================================================================
    /** @name 先行エージェントに関する変数 */
    /// @{
  
    /// 仮想先行エージェントの集合
    std::vector<VirtualLeader*> _leaders;

    /// @}
    //====================================================================
    /** @name 動きに関する変数 */
    /// @{

    /// 速度[m/msec]
    double _velocity;
    /// error方向の速度[m/msec]
    double _errorVelocity;

    /// 加速度[m/msec^2]
    double _accel;

    /// 最大加速度[m/msec^2]
    double _maxAcceleration;
    /// 最大減速度[m/msec^2]
    double _maxDeceleration;

    /// そのレーンにおける希望走行速度[m/msec]
    double _vMax;

    /// ウィンカー
    Blinker _blinker;

    /// 車線変更挙動定義オブジェクト
    friend class VehicleShift;
    VehicleShift _shiftLane;

    /// 車線変更中に先のレーンを見る
    bool _lookupShiftLane;

    /// 交錯を厳密に評価
    bool _strictCollisionCheck;

    /// 現在のセクションに入った時刻
    /**
     * _runIntersection2Sectionでリセットされる
     */
    ulint _entryTime;

    /// 現在特殊な行動（車線変更）をとっているかどうか
    bool _isNotifying;

#ifdef _OPENMP
    /// @}
    //====================================================================
    /** @name マルチスレッドに関する変数 */
    /// @{

    /// 集中発生ステップ数
    ulint _genIntensiveStep;

    /// 移動処理中
    bool _running;

    /// 経過時間変更交差点、NULL ならなし、非並列
    Intersection* _passTimeIntersection;
    /// 経過時間移動元
    int _passTimeFrom;
    /// 経過時間移動先
    int _passTimeTo;
    /// 経過時間
    ulint _passTime;

    /// 注目（車線変更）車両レーン変更、非並列
    enum ChangeWatchedLaneType
    {
        CWLT_NONE,
        CWLT_ERASE_SECTION,
        CWLT_ERASE_INTERSECTION,
        CWLT_SECTION_TO_INTERSECTION,
        CWLT_INTERSECTION_TO_SECTION
    };
    ChangeWatchedLaneType _changeWatchedLane;
    /// 注目（車線変更）車両レーン変更交差点
    Intersection* _changeWatchedLaneIntersection;
    /// 注目（車線変更）車両レーン変更単路
    Section* _changeWatchedLaneSection;

    /// reroute 回数（負荷確認）
    int _rerouteCnt;

    // 経路検索最大ステップ
    int _routerSearchStep;
#endif

    /// 交差点に侵入するときに一旦停止したかどうか
    bool _hasPaused;

    /// 休止時間[msec]
    /**
     * 休止状態の車両は全ての通行優先権を失う
     * 逆に言うと他車両は休止状態の車両を判断の材料としない
     */
    int _sleepTime;

    /// @}
    //====================================================================
    /** @name 経路に関する変数 */
    /// @{

    /// 大局的経路を生成するオブジェクト
    ARouter* _router;
    /// 大局的経路オブジェクト(Intersectionのvector)
    Route* _route;

    /// 局所的経路を生成するオブジェクト
    LocalLaneRouter _localRouter;
    /// 局所的経路オブジェクト（Laneのvector）
    LocalLaneRoute _localRoute;

#ifdef USE_ADDIN
    /// @}
    //======================================================================
    /** @name アドインに関する変数 */
    /// @{

    /// アドイン任意データ
    void* _addinData;
#endif

    /// @}
};

#endif //__VEHICLE_H__
