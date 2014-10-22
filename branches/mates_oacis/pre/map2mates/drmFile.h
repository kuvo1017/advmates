#ifndef DRMFILE_H
#define DRMFILE_H

#include <QFile>
#include <vector>
#include <string>
#include <map>

using namespace std;

class Intsec;

//レコードサイズと最大継続数
#define DRM_RECORD_SIZE       256
#define DRM_RECORD_CONT_MAX   256

// DRM ファイルノード
class drmFileNode
{
    Intsec*         _intsec;                                //交差点
    drmFileNode*    _conFileNode;                           //隣接ファイルノード
    double          _lat, _lon;                             //緯度経度
public:
    drmFileNode(double lat, double lon);                    //コンストラクタ
    void setConFileNode(drmFileNode* conFileNode);          //隣接ファイルノード設定
    Intsec* createIntsec();                                 //交差点作成
    void setIntsecMapPos();                                 //交差点位置設定
};

// DRM ファイル
class drmFile
{
    vector<string>  _allFileNames;                          //全ファイル名
    string          _fileName;                              //ファイル名
    QFile*          _file;                                  //ファイル
    map<string, drmFileNode*>
                    _nodes;                                 //全ノード
    string          _code2nd;                               //ファイルごとの2次メッシュコード
    double          _fileLatMinJapan, _fileLonMinJapan;     //ファイルごとの緯度経度最小値、日本測地系
    unsigned char   _record[DRM_RECORD_CONT_MAX][DRM_RECORD_SIZE];
                                                            //レコード
    int             _recordCnt;                             //レコードカウンタ
    double          _latMin, _latMax, _lonMin, _lonMax;     //緯度経度最大／最小値

public:
    drmFile();                                              //コンストラクタ
    ~drmFile();                                             //デストラクタ
    void prevRead();                                        //読み込み前処理
    bool read();                                            //読み込み

private:
    bool getNode();                                         //ノード取得
    bool getLink(int contNum);                              //リンク取得
    unsigned char getCode(int cont, int pos);               //レコードから文字コード取得
    int getDigit(int cont, int pos, int num);               //レコードから数値取得、エラーならメッセージを出して -1
    void getString(int cont, int pos, int num, string* str);//レコードから文字列取得、エラーチェックなし
    void setLatLonMinMax(double lat, double lon);           //緯度経度最大／最小値設定
    void xyToLatLon(int x, int y, double* lat, double* lon);// xy 座標から緯度経度変換
};

#endif // DRMFILE_H
