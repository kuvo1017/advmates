#ifndef __ROADSIDE_UNIT_H__
#define __ROADSIDE_UNIT_H__

/** @addtogroup roadsideUnit */

/// 路側に配置される信号機以外の機器の純粋抽象クラス
/**
 * DetectorUnitなどに継承させる
 *
 * @ingroup roadsideUnit
 */
class RoadsideUnit{
 public:
  virtual ~RoadsideUnit(){};

  /// 機器の識別番号を返す
  virtual const std::string& id() const = 0;
};

#endif //__ROADSIDE_UNIT_H__
