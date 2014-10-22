#ifndef __LANE_H__
#define __LANE_H__

#include <string>
#include <vector>
#include "Connector.h"
#include "AmuLineSegment.h"
#include "TimeManager.h"

class RoadOccupant;
class Vehicle;

/// 仮想走行レーンを表すクラス
/**
 * シミュレーション空間上のある点からある点へと向かう線分で定義される。
 *
 * 仮想走行レーンとは車が本来移動すべき道筋を表す。
 * すなわち車エージェントはレーンの位置・形状に関わりなく、
 * 自身のレーン上の一次元座標 @c length [m] を増加させることで移動する。
 * 終点にたどり着いた時点で次の隣接するレーンへ移動する。
 *
 * 単路の場合は車線にほぼ対応する。
 *
 * 交差点では単路の接続状況とその車線数にあわせて
 * 右折・直進･左折に対応するレーンが配置される。
 *
 * @ingroup RoadNetwork
 */
class Lane
{
public:
    Lane(const std::string& id, const Connector* ptBegin, const Connector* ptEnd);
    ~Lane();

    /// 識別番号を設定する
    void setId(const std::string& id);
    /// 識別番号を返す
    const std::string& id() const;

    //====================================================================
    /** @name レーンに関するもの */
    /// @{

    /// 始点を返す
    const Connector* beginConnector() const;
    /// 終点を返す
    const Connector* endConnector() const;

    /// レーンの長さを返す
    double length() const;
    /// 線分を返す
    const AmuLineSegment lineSegment() const;
    /// 方向ベクトルを返す
    const AmuVector directionVector() const;

    /// 制限速度を返す
    double speedLimit() const;
    /// 制限速度をセットする
    void setSpeedLimit(double limit);

    /// 最新の車両到達時刻を返す
    ulint lastArrivalTime() const;
    /// 最新の車両到達時刻をセットする
    void setLastArrivalTime(ulint arrivalTime);

    /// 交点を返す
    /** @sa LineSegment::createIntersectionPoint() */
    bool createIntersectionPoint(const AmuLineSegment& crSLine,
                                 AmuPoint* result_point) const;
    /// 内分点を返す
    /** @sa LineSegment::createInteriorPoint() */
    AmuPoint createInteriorPoint(double d0, double d1) const;

    /// 点@p pointに最も近いレーン上の点を返す
    const AmuPoint calcNearestPoint(AmuPoint point) const;

    /// @}

    //====================================================================
    /** @name レーン上のエージェントに関するもの */
    /// @{

    /// エージェントの集合を返す
    std::vector<RoadOccupant*>* agents();

    /// このレーンの先頭のエージェントを返す
    RoadOccupant* headAgent();
    RoadOccupant* headAgent(Vehicle* sightVehicle);
    Vehicle* headVehicle();

    /// このレーンの最後尾のエージェントを返す
    RoadOccupant* tailAgent();
    RoadOccupant* tailAgent(Vehicle* sightVehicle);
    Vehicle* tailVehicle();
    RoadOccupant* tailAgentStrict(Vehicle* sightVehicle);

    /// @p agentの一つ前のエージェントを返す
    RoadOccupant* frontAgent(RoadOccupant* agent);
    RoadOccupant* frontAgent(RoadOccupant* agent, Vehicle* sightVehicle);
    Vehicle* frontVehicle(RoadOccupant* agent);

    /// 位置@p lengthの点の一つ前のエージェントを返す
    RoadOccupant* frontAgent(double length);
    RoadOccupant* frontAgent(double length, Vehicle* sightVehicle);
    Vehicle* frontVehicle(double length);
    RoadOccupant* frontAgentStrict(double length, Vehicle* sightVehicle);

    /// @p agentの一つ後ろのエージェントを返す
    RoadOccupant* followingAgent(RoadOccupant* agent);
    Vehicle* followingVehicle(RoadOccupant* agent);

    /// 位置@p lengthの点の一つ後ろのエージェントを返す
    RoadOccupant* followingAgent(double length);
    RoadOccupant* followingAgent(double length, Vehicle* sightVehicle);
    Vehicle* followingVehicle(double length);

    /// @p agentを含めた前方にいるエージェントを厳密に距離計算して返す
    RoadOccupant* foreAgentStrict(RoadOccupant* agent);

    /// @p vehicleをレーンに登録する
    bool putAgent(RoadOccupant* agent);

    /// エージェントの順序列を更新する
    void renewAgentLine();

    /// @p agentをレーンから消去する
    /**
     * おもに車線変更で用いる．
     * 現在の_agentsから@p agentを削除し，_tmpAgentsを構成する
     */
    void extractAfterEraseAgent(RoadOccupant* agent);

    /// @p agentを@p frontの後ろに追加する
    /**
     * おもに車線変更で用いる．
     * 現在の_agentsの@p frontの後ろに@p agentを追加する
     * frontがNULLの場合は先頭に追加する
     */
    void extractAfterAddAgent(RoadOccupant* agent, RoadOccupant* front);

    /// 道路勾配を返す
    double gradient() const;

    /// レーン上の車両エージェントの平均速度を返す
    double averageVel() const;

    /// @}

protected:
    /// 識別番号
    /**
     * レーンを含むオブジェクト内で一意になるように設定される。
     */
    std::string _id;

    /// 制限速度[km/h]
    double _speedLimit;
    /// 最新の車両到達時刻[msec]
    ulint _lastArrivalTime;

    /// 始点
    const Connector* _beginConnector;
    /// 終点
    const Connector* _endConnector;
    /// レーンを表す線分
    const AmuLineSegment _lineSegment;

    /// このレーンに配置されているRoadOccupant
    /**
     * エージェントが行動した後で_tmpAgentsに登録し、
     * renewAgentLineで_tmpAgentsから_agentにコピーする
     */
    std::vector<RoadOccupant*> _agents;
    std::vector<RoadOccupant*> _tmpAgents;

    /// 道路勾配
    double _gradient;
};

#endif //__LANE_H__
