// MATES ファイルアクセス

#ifndef MATESFILE_H
#define MATESFILE_H

#include <Qt>
#include <string>
#include <sstream>

#define QT_FATAL_ASSERT

using namespace std;

// MATES ファイルアクセス
class MatesFile
{
    string      _mapPosFile;                            //マップ位置ファイル
    string      _networkFile;                           //ネットワークファイル
    string      _errorFile;                             //エラーファイル
    string      _intsecDir;                             //交差点ディレクトリ
public:
    MatesFile();                                        //コンストラクタ
    void        prevRead();                             //読み込み前処理
    void        prevWrite();                            //書き込み前処理
    bool        read();                                 //読み込み
    bool        write();                                //書き込み
private:
    bool        checkFile(const char* file);            //ファイルチェック
    bool        readMapPos();                           //マップ位置ファイル読み込み
    bool        readNetwork();                          //ネットワークファイル読み込み
    int         intsecId(const char* id);               //交差点ID取得とチェック、エラーなら MAP_NOID
    double      pos(const char* posStr, bool* ok);      //位置取得とチェック、エラーなら Ok が false、データを補正
    int         lane(const char lane);                  //車線数種得、エラーなら -1
    bool        writeMapPos();                          //マップ位置ファイル書き込み
    bool        writeNetwork();                         //ネットワークファイル書き込み
    bool        writeError();                           //エラーファイル書き込み
};

#endif // FILE_H
