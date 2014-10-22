#ifndef __GENERATE_VEHICLE_IO_H__
#define __GENERATE_VEHICLE_IO_H__

#include "ODNode.h"

/// 発生した車両の情報を出力するクラス
/**
 * インスタンスを生成しない．
 * @ingroup IO
 */
class GenerateVehicleIO
{
private:
    GenerateVehicleIO(){};
    ~GenerateVehicleIO(){};

public:
    /// ファイルで指定されたODノードを計測フラグを立て，出力ファイルを作成する
    static void getReadyCounters(RoadMap* roadMap,
                                 bool isVerbose);

    /// 発生した車両の情報をファイル出力する
    static void writeGeneratedVehicleData
    (ODNode* node, std::vector<ODNode::GeneratedVehicleData>* gvd);

    /// 情報を画面に出力する
    static void print(std::vector<ODNode*>* nodes);
};

#endif //__GENERATE_VEHICLE_IO_H__
