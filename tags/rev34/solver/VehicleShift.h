#ifndef __VEHICLESHIFT_H__
#define __VEHICLESHIFT_H__

class Lane;
class Vehicle;
class RoadOccupant;

/// 車線変更関連機能を提供するクラス
/**
 * 車線変更機能のみでなく、車線変更されている側の車の速度制御も担当する。
 *
 * @bug
 * 車線変更を行っている最中に交差点に入ると実行時エラーが起こる。
 * 現在は交差点から一定以上の距離がない場合は車線変更を始めないようにして
 * 対処している。
 *
 * @note
 * 車線変更先の車の取得が beginShift() 時のみだけなので
 * 車線変更中にレーンから出て行かれると見えなくなる？ by Y.Kato
 *
 * @ingroup Vehicle
 */
class VehicleShift
{
public:
    VehicleShift();
    VehicleShift(Vehicle* vehicle);
    ~VehicleShift();
  
    void setVehicle(Vehicle* vehicle);

    /// 車線変更を始められるか
    bool canShift(Lane* laneTo, double lengthTo) const;

    /// 現在車線変更を行っているか
    bool isActive() const;

    /// 車線変更動作を始める
    void beginShift(Lane* laneTo, double lengthTo);

    /// 車線変更動作を終える
    void endShift();

    /// 車線変更動作を進める
    /** 
     * @return 動作が終了したか
     */
    bool proceedShift();

    /// 車線変更を中断する
    /**
     * 一瞬で元のレーンの中心に戻る．
     * 車線変更を行うと問題がある部分を切り抜けるために作成．
     */
    void abortShift();
  
    /// 車線変更を行う側のerror方向の速度を返す
    double activeErrorVelocity() const;

    /// 車線変更を受ける側の処理
    /**
     * 車線変更中の車をVirtualLeaderに登録する
     */
    void searchInterruption() const;
  
    Lane* laneTo() const;

    double lengthTo() const;

    double errorTo() const;

    const RoadOccupant* agentToNext() const;
  
private:
    Lane* _laneTo;
    double _lengthTo;
    double _errorTo;

    Vehicle* _vehicle;

    RoadOccupant* _agentToNext;
    RoadOccupant* _agentToPrev;

    bool _isActive;
};

#endif //__VEHICLESHIFT_H__
