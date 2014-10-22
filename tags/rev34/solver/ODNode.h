#ifndef __OD_NODE_H__
#define __OD_NODE_H__

#include "Intersection.h"
#include "TimeManager.h"
#include <string>
#include <deque>
#include <vector>

class RoadMap;
class Vehicle;

/// ODノードクラス
/*
 * Intersectionの派生クラス
 *
 * @ingroup roadNetwork
 */
class ODNode : public Intersection{
public:
    ODNode(const std::string& id, const std::string& type);
    ~ODNode();

    //====================================================================
    /** @name 車両の発生と消去にかかわる関数 */
    /// @{

    /// _waitingVehiclesに車両がセットされているか
    bool hasWaitingVehicles() const;

    /// _waitingVehiclesに車両を追加する
    void addWaitingVehicle(Vehicle* vehicle);

    /// _waitingVehiclesの先頭に車両を追加する
    /**
     * マニュアルで追加するときやバスなど
     */
    void addWaitingVehicleFront(Vehicle* vehicle);

    /// _waitingVehicleから車両をポップしシミュレーションに登場させる
#ifndef _OPENMP
    void pushVehicleToReal(RoadMap* roadMap, bool _isGenCount);
#else
    void pushVehicleToReal(RoadMap* roadMap, bool _isGenCount, int genIntensiveStep);
#endif

    /// レーン上にいるエージェントを消去する
    void deleteAgent(bool isWrite);

    /// @}

    /// 発生車両データを格納する構造体
    struct GeneratedVehicleData
    {
        Vehicle* vehicle;
        Lane* lane;
        ulint headway;
    };

    /// 発生車両データを格納する構造体を初期化する
    void clearGVD(GeneratedVehicleData* gvd);

    /// 計測フラグが立っているか
     bool isCount() const;

    /// 計測フラグを変更する
    void setCountState(const bool isCount);

protected:
    /// シミュレーションへの登場を待つ車両【メインコンテナ】
    std::deque<Vehicle*> _waitingVehicles;

    /// 直前の車両を発生させた時刻
    ulint _lastGenTime;

    /// 発生車両データ
    std::vector<GeneratedVehicleData> _nodeGvd;

    /// 発生車両を計測するか
    bool _isCount;

    //====================================================================
public:
    /** 
     * @name ODノードの詳細構造を作成する関数群
     * @note 長いので別ファイルで定義する
     * @sa IntersectionBuilder.cpp
     */
    /// @{

    /// 車道頂点の作成
    bool createDefaultStreetVertices();
    /// 頂点の作成
    bool createVertices();
    /// 境界の作成
    bool createDefaultBorders();
    /// サブセクションの作成
    bool createDefaultSubsections();
    /// レーンの作成
    bool createDefaultLanes();
    /// 属性指定ファイルからレーンの作成
    bool createLanesFromFile();

    /// @}
 
};
#endif //__OD_NODE_H__
