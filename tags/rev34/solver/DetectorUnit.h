#ifndef __DETECTOR_UNIT_H__
#define __DETECTOR_UNIT_H__

#include <vector>
#include <string>
#include "TimeManager.h"
#include "RoadsideUnit.h"
#include "Detector.h"

class Section;

/// 車両感知器群クラス
/**
 * 一つの単路に設置される車両感知器の「グループ」に相当する．
 * 各レーンを担当する個々の感知器を管理する．
 * 出力ファイルはユニット単位で作成する．
 *
 * @ingroup roadsideUnit
 */
class DetectorUnit : public RoadsideUnit{
public:
    DetectorUnit(const std::string& id);
    ~DetectorUnit();

    /// 識別番号を返す
    const std::string& id() const;
    /// _sectionを返す
    Section* section() const;
    /// _isUpを返す
    bool isUp() const;
    /// _lengthを返す
    double length() const;
    /// _intervarを返す
    ulint interval() const;

    /// ユニットを配置し，個々の感知器を作成する
    void setPosition(Section* section, bool isUp, double length, ulint interval);

    /// _detectorsを返す
    const std::vector<Detector*>* detectors() const;

    /// 単路を観測する
    void monitorLanes();

    /// 観測データを返す
    bool getPassedVehicleData
    (std::vector<Detector::PassedVehicleData>* result) const;

    /// 統計データを格納する構造体
    struct StatVehicleData
    {
        ulint beginTime;
        int totalAllPassengers;
        int totalAllTrucks;
        int sumPassengers;
        int sumTrucks;
        std::vector<std::string> laneIds;
        std::vector<int> numPassengers;
        std::vector<int> numTrucks;
    };

    /// 統計データを初期化する
    void clearStatVehicleData(StatVehicleData svd)
    {
        svd.beginTime = TimeManager::time();
        svd.sumPassengers = 0;
        for (unsigned int i=0; i<svd.numPassengers.size(); i++)
        {
            svd.numPassengers[i] = 0;
        }
        svd.sumTrucks = 0;
        for (unsigned int i=0; i<svd.numTrucks.size(); i++)
        {
            svd.numTrucks[i]=0;
        }
    };

    /// 統計データを返す
    StatVehicleData statVehicleData();

    /// 情報を表示する
    void print() const;

private:
    /// 識別番号
    std::string _id;

    /// 設置されている単路
    Section* _section;

    /// 単路の上り方向かどうか
    bool _isUp;

    /// 始点交差点からの距離
    double _length;

    /// 統計データを出力する時間間隔
    ulint _interval;

    /// ユニットを構成する感知器
    std::vector<Detector*> _detectors;

    /// 観測データ
    std::vector<Detector::PassedVehicleData> _unitPvd;

    /// 統計データ
    StatVehicleData _unitSvd;
};

#endif //__DETECTOR_UNIT_H__
