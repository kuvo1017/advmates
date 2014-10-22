#ifndef __APPMATES_H__
#define __APPMATES_H__

#include <string>
#include <getopt.h>

class Simulator;

/**
 * @addtogroup Procedure
 * @brief シミュレーション手順を定義するモジュール
 */

/**
 * @addtogroup Initialization
 * @brief シミュレーションの初期化と開始
 * @ingroup Procedure
 */

/// アプリケーションの基底クラス
/**
 * データファイルや乱数の種などを管理する
 *
 * @ingroup Initialization
 */
class AppMates
{
public:
    AppMates();
    virtual ~AppMates(){};

    /// 初期化とコマンドライン引数の処理
    virtual void init(int argc, char** argv, bool output);

    /// シミュレータを返す
    virtual Simulator* simulator();

protected:
    /// コマンドライン引数を処理する
    virtual void parseArgument(int argc, char** argv);

    /// 説明を出力する
    virtual void printUsage();

    /// データディレクトリのパスを設定する．
    /**
     * _dataPathを元にFilePathManagerへの登録を行う
     */
    virtual void initPath();

    /// シミュレータの準備
    /**
     * @param isWrite 時系列データを出力するか
     */
    virtual bool getReadySimulator(bool isWrite);

private:
    /// データディレクトリへのパスを指定する
    /**
     * @param arg ディレクトリ名
     */
    bool _initDataPath(std::string arg);

    /// 乱数の種を指定する
    /**
     * @param arg 指定する種(文字列で与える)
     */
    bool _initRandomSeed(std::string arg);

protected:
    Simulator* _simulator;

    /// データディレクトリへのパス
    std::string _dataPath; 

    /// 乱数の種
    unsigned int _key;

    /** @name コマンドラインオプションの解析に用いる変数 **/
    //@{
    static int optionIndex;
    static std::string shortOptions;
    static struct option longOptions[];
    //@}

    /// 情報を画面出力するか否か
    bool _isVerbose;

    /** @name 入力フラグ */
    //@{

    /// 地図情報の入力があるか
    bool _inputMap;

    /// 信号情報の入力があるか
    bool _inputSignal;

    /// 車両情報の入力があるか
    bool _inputVehicle;

    /// 車両発生情報が指定されていない交差点から車両を発生させるか
    bool _generateRandomVehicle;

    //@}

    /** @name 出力フラグ **/
    //@{

    /// 時系列データを出力するか
    bool _outputTimeline;

    /// 計測機器の詳細データを出力するか
    bool _outputInstrumentD;

    /// 計測機器の統計データを出力するか
    bool _outputInstrumentS;

    /// エージェント発生データを出力するか
    bool _outputGenCounter;

    /// エージェントの走行距離、旅行時間を出力するか
    bool _outputTripInfo;

    //@}
};

#endif //__APPMATES_H__
