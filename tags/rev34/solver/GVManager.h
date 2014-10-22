#ifndef __GV_MANAGER_H__
#define __GV_MANAGER_H__

#include <string>
#include <map>

/**
 * @addtogroup Utility
 * @brief シミュレータ全般から利用される機能を提供するモジュール
 */

/// シミュレーション内で大域的に利用する定数を管理するクラス
/**
 * 変数を登録・管理する
 *
 * @ingroup Utility
 */
class GVManager
{
protected:
    GVManager();
    ~GVManager();

    /// コピーコンストラクタ
    GVManager(const GVManager& rhs);

    /// 代入演算子
    GVManager& operator=(const GVManager& rhs);

public:
    /// 唯一のインスタンスを返す
    static GVManager& instance();

    /// 指定のシンボルに関連付けられた文字列を返す
    bool getVariable(const std::string& key, std::string* value);
    /// 指定のシンボルに関連付けられた数値を返す
    bool getVariable(const std::string& key, double* value);
    /// 指定のシンボルに関連付けられた数値を返す
    double getNumeric(const std::string& key);
    /// _maxTimeを返す
    unsigned long getMaxTime() const;

    /// 指定のシンボルに文字列を設定する
    bool setNewVariable(const std::string& key, const std::string& value);
    /// 指定のシンボルに数値を設定する
    bool setNewVariable(const std::string& key, const double value);

    /// 指定のシンボルに文字列を設定し直す
    bool resetVariable(const std::string& key, const std::string& value);
    /// 指定のシンボルに数値を設定し直す
    bool resetVariable(const std::string& key, const double value);

    /// ファイルから変数を読み込む
    /**
     * 1行につき1変数．形式は symbol=value とする．
     */
    bool setVariablesFromFile(const std::string& fileName);

    /// 全ての変数を出力する
    void print() const;

protected:
    /// 文字列として保持される変数
    std::map<std::string, std::string> _strings;

    /// 数値として保持される変数
    std::map<std::string, double> _numerics;

    /// _maxTimeのみ特別に持つ
    unsigned long _maxTime;
};

#endif //__GV_MANAGER_H__
