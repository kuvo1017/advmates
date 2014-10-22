//メインウィンドウ

//非ネイティブファイルダイアログは英語だが、そうでないとまともにフィルタが動かない
//これが標準？static 関数でないのを使うとこれがでる、マニュアルもこれ？

#include <QtGui>
#include <QFileDialog>
#include <QInputDialog>
#include <sstream>
#include <string>
#include "mainwindow.h"
//app.h を最初に置くとQtフォームスロット編集が誤動作する
#include "app.h"
#include "ui_mainwindow.h"
#include "mapframe.h"
#include "map.h"
#include "intsec.h"
#include "road.h"
#include "setupDialog.h"
#include "testDataDialog.h"
#include "fileAccess.h"

using namespace std;

#define MAINTITLE     "MATESマップファイルエディタ"         //メインウィンドウタイトル
#define TITLE         "マップファイルエディタ"              //ダイアログタイトル
#define ABOUT         "MATESマップファイルエディタ 2.1 (C) 2010-2013 東京大学吉村研究室 by AA&S"

//コンストラクタ
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), _ui(new Ui::MainWindow)
{
    _ui->setupUi(this);
}

//デストラクタ
MainWindow::~MainWindow()
{
    delete _ui;
}

//初期処理
void MainWindow::init()
{
    _lane2 = true;
    _ui->actionLane2->setChecked(true);
    setEditMode(select);
    setTitle();
}

//変換イベント
void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        _ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//クローズイベント
void MainWindow::closeEvent(QCloseEvent *e)
{
    if (!app.getMap()->isNoData())
    {
        if (QMessageBox::question(this, tr(TITLE), tr("現在の内容をファイルに保存しますか？"),
                                  QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
            fileSave(false);
    }
    e->accept();
}

//ファイルオープン
void MainWindow::fileOpen(const char* path)
{
    string          setupFile, backFile;
    stringstream    filter;
    QFileInfo       fileInfo;
    QString         newFile;
    bool            noSetupFile;
    FileAccess      fileAccess;
    bool            backFileOk, cancel;

    //データ削除チェック
    if (!app.getMap()->isNoData())
    {
        if (QMessageBox::question(this, tr(TITLE), tr("編集内容を破棄してよろしいですか？"),
                                  QMessageBox::Yes, QMessageBox::No) != QMessageBox::Yes)
            return;
    }

    //ファイル名取得
    app.getSetupFile(&setupFile);
    if (setupFile == "") {
        if (path == NULL)
            setupFile = QCoreApplication::applicationDirPath().toStdString();
        else
            setupFile = path;
        setupFile += "/";
        setupFile += APP_DEFAULTFILE;
    }
    filter << "マップファイルエディタ設定ファイル (" << APP_DEFAULTFILE_ALL <<");;"
          << "マップ位置ファイル (" << APP_MAPPOSFILE << ");;"
          << "すべてのファイル (*.*)";
    newFile = QFileDialog::getOpenFileName(this, tr("設定ファイルオープン"), setupFile.c_str(),
                                           tr(filter.str().c_str()), 0,
                                           QFileDialog::DontUseNativeDialog);
    if (newFile == "")
        return;
    fileInfo.setFile(newFile);
    if (fileInfo.fileName() == APP_MAPPOSFILE)
    {
        noSetupFile = true;
        setupFile = fileInfo.absolutePath().toStdString() + "/" + APP_DEFAULTFILE;
    }
    else
    {
        noSetupFile = false;
        setupFile = newFile.toStdString();
    }

    //ファイル読み込み失敗、キャンセル以外は全クリア
    fileAccess.init(this, setupFile.c_str());
    if (!fileAccess.read(&cancel, noSetupFile))
    {
        if (!cancel)
        {
            app.getMap()->deleteAll();
            app.setDefault();
            app.getMapFrame()->repaintAll(false, true);
            setTitle();
        }
    }

    //ファイル読み込み成功、再描画
    else
    {
        if (_editMode != large && app.getMap()->getIntsecNum() >= 1000) {
            app.getMapFrame()->setVisible(false);
            stringstream message;
            message << "交差点数が " << app.getMap()->getIntsecNum()
                    << " 個ありますので広域表示モードにします。";
            QMessageBox::information(this, tr(TITLE), tr(message.str().c_str()),
                                     QMessageBox::Ok);
            setEditMode(large);
            app.getMapFrame()->setVisible(true);
        }

        if (app.getBackImage())
        {
            app.getBackFile(&backFile);
            if (!app.getMapFrame()->loadBackImage(backFile.c_str(), setupFile.c_str()))
            {
                backFileOk = false;
                QMessageBox::critical(this, tr(TITLE), tr("画像ファイルをロードできません"));
                if (app.backFileDialog(this, &backFile))
                {
                    if (!app.getMapFrame()->loadBackImage(backFile.c_str(), setupFile.c_str()))
                        QMessageBox::critical(this, tr(TITLE), tr("画像ファイルをロードできません"));
                    else
                        backFileOk = true;
                }
                if (backFileOk)
                    app.setBackFile(backFile.c_str());
                else
                    app.setBackImage(false);
            }
        }
        app.setSetupFile(setupFile.c_str());
        app.getMapFrame()->repaintAll(true, true);
        setTitle();
    }
}

//編集モード設定
void  MainWindow::setEditMode(EditMode em)
{
    _editMode = em;
    _ui->actionSelect->setChecked(_editMode == select);
    _ui->actionLarge->setChecked(_editMode == large);
    _ui->actionIntsec->setChecked(_editMode == intsec);
    _ui->actionRoad->setChecked(_editMode == road);
    _ui->actionRoadRepl->setChecked(_editMode == roadRepl);
    _ui->actionLane2->setEnabled(_editMode == road);
    _ui->actionLane4->setEnabled(_editMode == road);
    app.getMapFrame()->changeEditMode();
}

//右クリックメニューイベント
void MainWindow::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.addAction(_ui->actionSelect);
    menu.addSeparator();
    menu.addAction(_ui->actionIntsec);
    menu.addSeparator();
    menu.addAction(_ui->actionRoad);
    menu.addAction(_ui->actionRoadRepl);
    menu.addSeparator();
    menu.addAction(_ui->actionLane2);
    menu.addAction(_ui->actionLane4);
    menu.addSeparator();
    menu.addAction(_ui->actionFileOption);
    menu.addSeparator();
    menu.addAction(_ui->actionDirJump);

    int intsecId;
    app.shMem().get(&intsecId);
    stringstream ssj, sss;
    if (intsecId >= 0)
    {
        ssj << "交差点ジャンプ <" << intsecId << "> (&J)";
        _ui->actionJump->setText(tr(ssj.str().c_str()));
        menu.addAction(_ui->actionJump);
        menu.addSeparator();
        sss << "設定エディタ起動 <" << intsecId << "> (&E)";
        _ui->actionConfigEditor->setText(tr(sss.str().c_str()));
    }
    menu.addSeparator();
    menu.addAction(_ui->actionConfigEditor);

    menu.exec(event->globalPos());

    //他でも使うので戻す、他で使うタイミングでは ID 不明
    _ui->actionJump->setText(tr("交差点ジャンプ(&J)"));
    _ui->actionConfigEditor->setText(tr("設定エディタ起動(&E)"));
}

//ファイル新規
void MainWindow::on_actionFileNew_triggered()
{
    if (!app.getMap()->isNoData())
    {
        if (QMessageBox::question(this, tr(TITLE), tr("編集内容を破棄してよろしいですか？"),
                                  QMessageBox::Yes, QMessageBox::No) != QMessageBox::Yes)
            return;
    }
    app.getMap()->deleteAll();
    app.setDefault();
    app.getMapFrame()->repaintAll(false, true);
    setTitle();

    //テストデータ作成
//  createTestData();
}

//ファイルオープン
void MainWindow::on_actionFileOpen_triggered()
{
    fileOpen();
}

//ファイル保存
void MainWindow::on_actionFileSave_triggered()
{
    fileSave(false);
}

//ファイル別名保存
void MainWindow::on_actionFileSaveAs_triggered()
{
    fileSave(true);
}

//ファイル設定
void MainWindow::on_actionFileOption_triggered()
{
    SetupDialog sd(this);
    sd.init();
    if (sd.exec() == QDialog::Accepted)
        app.getMapFrame()->repaintAll();
}

//ファイルテストデータ
void MainWindow::on_actionFileTestData_triggered()
{
    if (!app.getMap()->isNoData())
    {
        if (QMessageBox::question(this, tr(TITLE), tr("編集内容を破棄してよろしいですか？"),
                                  QMessageBox::Yes, QMessageBox::No) != QMessageBox::Yes)
            return;
    }
    TestDataDialog tdd(this);
    tdd.init();
    if (tdd.exec() != QDialog::Accepted)
        return;
    setTitle();
}

//終了
void MainWindow::on_actionExit_triggered()
{
    close();
}

//マップファイルエディタについて
void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("MATESマップファイルエディタについて"), tr(ABOUT));
}

//選択
void MainWindow::on_actionSelect_triggered()
{
    setEditMode(select);
    app.getMapFrame()->repaintAll();
}

//広域表示
void MainWindow::on_actionLarge_triggered()
{
    setEditMode(large);
    app.getMapFrame()->repaintAll();
}

//交差点
void MainWindow::on_actionIntsec_triggered()
{
    setEditMode(intsec);
    app.getMapFrame()->repaintAll();
}

//単路
void MainWindow::on_actionRoad_triggered()
{
    setEditMode(road);
    app.getMapFrame()->repaintAll();
}

//単路付け替え
void MainWindow::on_actionRoadRepl_triggered()
{
    setEditMode(roadRepl);
    app.getMapFrame()->repaintAll();
}

//往復2車線
void MainWindow::on_actionLane2_triggered()
{
    _lane2 = true;
    _ui->actionLane2->setChecked(true);
    _ui->actionLane4->setChecked(false);
}

//往復4車線
void MainWindow::on_actionLane4_triggered()
{
    _lane2 = false;
    _ui->actionLane4->setChecked(true);
    _ui->actionLane2->setChecked(false);
}

//拡大
void MainWindow::on_actionZoomIn_triggered()
{
    app.getMapFrame()->zoomInOut(1);
}

//縮小
void MainWindow::on_actionZoomOut_triggered()
{
    app.getMapFrame()->zoomInOut(-1);
}

//ジャンプ
void MainWindow::on_actionJump_triggered()
{
    //ここでメモリから出す
    int intsecId;
    app.shMem().get(&intsecId);
    if (intsecId >= 0) {
        MapFrame* mapFrame = app.getMapFrame();
        Intsec* intsec = app.getMap()->getIntsec(intsecId);
        if (intsec != NULL) {
            mapFrame->changeShowSelect(intsecId, MAP_NOID, MAP_NOID);
            int viewX = mapFrame->getViewX(intsec->getMapPosX()) -
                        mapFrame->width() / 2;
            int viewY = mapFrame->getViewY(intsec->getMapPosY()) +
                        mapFrame->height() / 2;
            app.setViewMap(mapFrame->getMapX(viewX), mapFrame->getMapY(viewY));
            app.getMapFrame()->repaintAll();
        }
    }
}

//指定ジャンプ
void MainWindow::on_actionDirJump_triggered()
{
    bool ok;
    int id = QInputDialog::getInt(this, tr("交差点指定ジャンプ"),
                                  tr("交差点ID"), 0, 0, 999999, 6, &ok);
    if (!ok)
        return;

    //ここでメモリに入れる
    app.shMem().set(id);
    on_actionJump_triggered();
}

//設定エディタ
void MainWindow::on_actionConfigEditor_triggered()
{
    //ここでメモリから出す
    int intsecId;
    app.shMem().get(&intsecId);
    app.configEditor(intsecId);
}

//全体情報
void MainWindow::on_actionAllInfo_triggered()
{
    int                 intsecAll, intsecOD, errorIntsec, roadAll, errorRoad;
    Map*                pMap = app.getMap();
    stringstream        ss;
    MAP_INTSECMAP_IT    imi;
    Intsec*             pIntsec;
    MAP_ROADMAP_IT      rmi;
    Road*               pRoad;

    intsecAll = intsecOD = errorIntsec = roadAll = errorRoad = 0;

    pIntsec = pMap->nextIntsec(&imi, true);
    while (pIntsec != NULL)
    {
        if (pIntsec->checkError())
            errorIntsec++;
        if (pIntsec->getRoadNum() == 1)
            intsecOD++;
        intsecAll++;
        pIntsec = pMap->nextIntsec(&imi);
    }
    pRoad = pMap->nextRoad(&rmi, true);
    while (pRoad != NULL)
    {
        if (pRoad->checkError())
            errorRoad++;
        roadAll++;
        pRoad = pMap->nextRoad(&rmi);
    }

    ss << "全交差点数 " << intsecAll << " OD交差点数 " << intsecOD
       << " エラー交差点数 " << errorIntsec << endl
       << "全単路数 " << roadAll << " エラー単路数 " << errorRoad;
    QMessageBox::information(this, tr(TITLE), tr(ss.str().c_str()));
}

//タイトル設定
void MainWindow::setTitle()
{
    string          setupFile;
    stringstream    title;

    app.getSetupFile(&setupFile);
    if (setupFile == "")
        setWindowTitle(tr(MAINTITLE));
    else
    {
        title << MAINTITLE << " - " << setupFile;
        setWindowTitle(tr(title.str().c_str()));
    }
}

//ファイル保存（共通）
void MainWindow::fileSave(bool saveAs)
{
    string          setupFile;
    stringstream    filter;
    QString         newFile;
    FileAccess      fileAccess;

    //データなし
    if (app.getMap()->isNoData())
    {
        if (QMessageBox::question(this, tr(TITLE), tr("データがありませんが保存しますか？"),
                                  QMessageBox::Yes, QMessageBox::No) != QMessageBox::Yes)
            return;
    }

    //ファイル名取得
    app.getSetupFile(&setupFile);
    if (setupFile == "")
    {
        saveAs = true;
        setupFile = QCoreApplication::applicationDirPath().toStdString() + "/" + APP_DEFAULTFILE;
    }
    if (saveAs)
    {
        filter << "マップファイルエディタ設定ファイル (" << APP_DEFAULTFILE_ALL <<");;"
              << "すべてのファイル (*.*)";
        newFile = QFileDialog::getSaveFileName(this, tr("設定ファイル保存"), setupFile.c_str(),
                                               tr(filter.str().c_str()), 0,
                                               QFileDialog::DontUseNativeDialog |
                                               QFileDialog::DontConfirmOverwrite);
        if (newFile == "")
            return;
        setupFile = newFile.toStdString();
    }
//  qDebug("MainWindow::fileSave '%s'", setupFile.c_str());

    //ファイル書き込み
    fileAccess.init(this, setupFile.c_str());
    if (fileAccess.write())
    {
        app.setSetupFile(setupFile.c_str());
        setTitle();
    }
}

//テストデータ作成
void MainWindow::createTestData()
{
    if (QMessageBox::question(this, tr("テスト"), tr("テストデータ？"),
                              QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
    {
        app.setViewMagni(0.25);
        app.setBackImage(true);
        QString backFile = QCoreApplication::applicationDirPath();
        backFile += "/images/grid.png";
        app.setBackFile(backFile.toStdString().c_str());
        if (!app.getMapFrame()->loadBackImage(backFile.toStdString().c_str(), ""))
            qDebug("MainWindow::createTestData main load back image error!");
        app.setBackMagni(4);

        if (QMessageBox::question(this, tr("テスト"), tr("大データ？"),
                                  QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
        {
            Intsec*	pIntsec;
            int		i, x, y, col, row;
            bool		bAlready;
            if (QMessageBox::question(this, tr("テスト"), tr("巨大データ？"),
                                      QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
            {
                if (QMessageBox::question(this, tr("テスト"), tr("超巨大データ？"),
                                          QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
                {
                    col = 50;
                    row = 50;
                }
                else
                {
                    col = 30;
                    row = 30;
                }
            }
            else
            {
                col = 10;
                row = 15;
            }
            for (i = 0; i < col * row; i++)
            {
                pIntsec = app.getMap()->createIntsec(i);
                pIntsec->setMapPos((i % col) * 160 + 40, (i / col) * 80 + 40, 0);
            }
            for (x = 0; x < col - 1; x++)
            {
                for (y = 0; y < row; y++)
                    app.getMap()->createRoad(app.getMap()->getIntsec(x + y * col),
                                             app.getMap()->getIntsec(x + 1 + y * col), &bAlready);
            }
            for (x = 1; x < col - 1; x++)
            {
                for (y = 0; y < row - 1; y++)
                    app.getMap()->createRoad(app.getMap()->getIntsec(x + y * col),
                                             app.getMap()->getIntsec(x + ( y + 1 ) * col), &bAlready);
            }
        }
        app.getMapFrame()->repaintAll(true, true);
    }
}
