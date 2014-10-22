//メイン

// 2010/1/7 会議で作業の流れを考慮した仕様が出ていた、そちらを重視
//交差点新規作成→連続→交差点接続→連続→車線数指定→連続→位置補正→連続
//交差点モード
//　交差点クリック：交差点選択＋編集画面（削除）
//　交差点ドラッグ：交差点位置移動
//　単路クリック　：単路選択＋単路モード
//　他クリック　　：交差点新規作成
//単路モード
//　交差点クリック：交差点選択＋交差点モード
//　交差点ドラッグ：単路新規作成
//　単路クリック　：単路選択＋単路編集画面（削除）
//　他クリック　　：なし
//単路付け替えモード
//　交差点クリック：交差点選択＋交差点モード
//　交差点ドラッグ：単路付け替え（選択単路のみ）
//　単路クリック　：単路選択＋単路編集画面（選択時、削除）
//　他クリック　　：なし

//地図の X/Y 縮尺は同じとみなす
//表示位置は左下が基準、表示倍率は（表示／マップ）、背景倍率は（マップ／背景）
// GoogleMap で取った地図の倍率は 4 位、1-4 位が普通か？

//構造的問題は入力時にはじく、それ以外のエラーは画面やセーブ時に警告
//エラーの深刻度は開発状況により変わるのでわからない、とりあえず同じに扱う
// MATES 本体はあまりエラーチェックしてないのでここでやらないと理由がわからない
//他にもありそうだがとりあえず仕様にある所まで、仕様になくても明らかなエラーはチェック

//ファイル名日本語不可、Qt 自体があちこち動かない
// Qt のエディタはタブ位置が一定しないのでスペースの方がいい
//一般的でないフォント指定だと存在しない場合がある、Sans Serif なら大丈夫だろう
// 2000 個でもそこそこ動く
//現状ではマップ位置ファイル等は設定ファイルと同一フォルダ、背景画像ファイルはフルパス／相対パス指定
// MATES 本体は init.txt でマップ位置ファイル等を指定可能だが、現状では対応していない
// MATES 本体は位置が double になっているが、説明書やサンプルを見る限り int なので int にしとく
//一応 double での読み込みは可能

//改版履歴
//  2010.08.10 ver1.1  ファイル読み込み時に先頭接続先を正しく読めない問題を修正
//                     ダイアログ文字表示部に余裕を入れてフォントを変えても切れないよう修正
//  2011.03.04 ver1.2  テストデータ作成追加、Mates 並列実行テスト用
//                     Windows Qt で動くよう修正
//                     設定画面のタイトル修正、リリース時に不要なファイル削除
//                     2/4車線変更ツールボックスダブルクリックで選択が消えるバグ修正
//  2013.01.31 ver2.0  設定エディタ対応、選択モード追加
//  2013.05.24 ver2.1  広域データ対応、簡易表示追加、交差点指定ジャンプ追加
//                     MATES本体の交差点形状修正対応

#include <QTranslator>
#include <QApplication>
#include <QMessageBox>
#include <QTextCodec>
#include <sstream>
#define MAIN_CPP
#include "app.h"
#include "mainwindow.h"
#include "mapframe.h"
#include "map.h"
#include <iostream>

//メイン
int main(int argc, char *argv[])
{
#ifdef USE_QT4
    QTextCodec::setCodecForTr(QTextCodec::codecForLocale());
    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
#endif // USE_QT4
    QApplication a(argc, argv);
    MainWindow mw;
    MapFrame mf;
    Map m;

    const char* path = NULL;
    stringstream ss;
    ss << "Usage: map_editor [-p <パス>] (パスは終端/なし)\n\n";
    ss << "コマンドライン不正:\nmap_editor ";
    for (int i = 1; i < argc; i++)
        ss << argv[i] << " ";
    if (argc == 3 && strcmp(argv[1], "-p") == 0)
        path = argv[2];
    else if (argc != 1)
        QMessageBox::warning(NULL, mw.tr("マップエディタ"), mw.tr(ss.str().c_str()));

    app.init(&mw, &mf, &m);
    mw.setCentralWidget(&mf);
    mw.init();
    m.init();
    mw.show();
    if (path != NULL)
        mw.fileOpen(path);

    return a.exec();
}
