// OSM ファイルアクセス

#ifndef OSMFILE_H
#define OSMFILE_H

#include <string>
#include <map>
#include <vector>

using namespace std;

class Intsec;

//　OSM ファイルノード
class osmFileNode
{
    Intsec*         _intsec;                                //交差点
    double          _lat, _lon;                             //緯度経度
    bool            _border;                                //境界
public:
    osmFileNode(double lat, double lon, bool border);       //コンストラクタ
    Intsec* createIntsec();                                 //交差点作成
    void setIntsecMapPos();                                 //交差点位置設定
    double lat()                                            //データ取得
    {
        return _lat;
    }
    double lon()
    {
        return _lon;
    }
};

//　OSM ファイル
class osmFile
{
    string          _fileName;                              //ファイル名
    map<string, osmFileNode*>
                    _nodes;                                 //全ノード
    vector<string>  _ndRefs;                                //道路内ノード参照ID
    string          _highway;                               //道路種別
    int             _lanes;                                 //レーン数、指定なしなら -1
    bool            _oneway;                                //一方通行指定
    double          _latMin, _latMax, _lonMin, _lonMax;     //緯度経度最大／最小値

public:
    osmFile();                                              //コンストラクタ
    ~osmFile();                                             //デストラクタ
    void prevRead();                                        //読み込み前処理
    bool read();                                            //読み込み

private:
    bool readNodes();                                       //全ノード読み込み
    bool readWays();                                        //全道路読み込み
    void setWay();                                          //道路設定
    void setLatLonMinMax(double lat, double lon);           //緯度経度最大／最小値設定
};

#endif // OSMFILE_H
