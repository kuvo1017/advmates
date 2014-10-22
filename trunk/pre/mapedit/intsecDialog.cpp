//交差点編集画面

#include <sstream>
#include "ui_intsecDialog.h"
#include "app.h"
#include "map.h"
#include "intsec.h"
#include "intsecDialog.h"

#define TITLE	"交差点編集画面"

//コンストラクタ
IntsecDialog::IntsecDialog(QWidget *parent) : QDialog(parent), _ui(new Ui::IntsecDialog)
{
    _ui->setupUi(this);
}

//デストラクタ
IntsecDialog::~IntsecDialog()
{
    delete _ui;
}

//初期処理
void IntsecDialog::init(Intsec* pIntsec)
{
    INTSEC_ANGLEMAP_IT  iami;
    Road*               pRoad;
    int                 index, intsecIdCon;
    stringstream        ss;

    _pIntsec = pIntsec;
    _intsecId = pIntsec->getIntsecId();
    _center = _delete = false;
    _ui->spinBoxId->setValue(_intsecId);
    _ui->spinBoxX->setValue(_pIntsec->getMapPosX());
    _ui->spinBoxY->setValue(_pIntsec->getMapPosY());
    _ui->spinBoxZ->setValue(_pIntsec->getMapPosZ());

    //右揃え不能、仕方ないか
    index = 0;
    pRoad = _pIntsec->nextRoadByAngle(&iami, true, &intsecIdCon);
    while (pRoad != NULL)
    {
        ss.str("");
        ss << intsecIdCon;
        _ui->comboFirstIntsec->insertItem(index, QString(ss.str().c_str()), QVariant(intsecIdCon));
        if (_pIntsec->getFirstIntsecIdCon() == intsecIdCon)
            _ui->comboFirstIntsec->setCurrentIndex(index);
        pRoad = _pIntsec->nextRoadByAngle(&iami, false, &intsecIdCon);
        index++;
    }
}

//変換イベント
void IntsecDialog::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        _ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

//中心座標ボタン、表示データ取得はやらない
void IntsecDialog::on_buttonCenter_clicked()
{
    int     intsecIdSave;

    if (QMessageBox::question(this, tr(TITLE), tr("この交差点を座標の中心にしてよろしいですか？"),
                              QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
    {
        intsecIdSave = _intsecId;
        getViewData();
        if (_intsecId != _pIntsec->getIntsecId() && app.getMap()->getIntsec(_intsecId) != NULL)
        {
            QMessageBox::warning(this, tr(TITLE), tr("この交差点IDは使用済みです。"));
            _intsecId = intsecIdSave;
            return;
        }
        _center = true;
        done(QDialog::Accepted);
    }
}

//削除ボタン
void IntsecDialog::on_buttonDelete_clicked()
{
    if (QMessageBox::question(this, tr(TITLE), tr("この交差点を削除してよろしいですか？"),
                              QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
    {
        _delete = true;
        done(QDialog::Accepted);
    }
}

//検証ボタン
void IntsecDialog::on_buttonVerify_clicked()
{
    Intsec  intsecSave(_pIntsec->getIntsecId());
    string  message;

    intsecSave.copyMapPos(_pIntsec);
    intsecSave.setFirstIntsecIdCon(_pIntsec->getFirstIntsecIdCon());
    getViewData(false);
    if (!_pIntsec->checkError())
        QMessageBox::information(this, tr(TITLE), tr("問題はありません。"));
    else
    {
        _pIntsec->getErrorMessage(&message);
        QMessageBox::warning(this, tr(TITLE), tr(message.c_str()));
    }
    _pIntsec->copyMapPos(&intsecSave);
    _pIntsec->setFirstIntsecIdCon(intsecSave.getFirstIntsecIdCon());
}

//設定エディタボタン
void IntsecDialog::on_buttonConfig_clicked()
{
    app.configEditor(_pIntsec->getIntsecId());
}

//OKボタン
void IntsecDialog::on_buttonOk_clicked()
{
    int     intsecIdSave;

    intsecIdSave = _intsecId;
    getViewData();
    if (_intsecId != _pIntsec->getIntsecId() && app.getMap()->getIntsec(_intsecId) != NULL)
    {
        QMessageBox::warning(this, tr(TITLE), tr("この交差点IDは使用済みです。"));
        _intsecId = intsecIdSave;
        return;
    }
    done(QDialog::Accepted);
}

//キャンセルボタン
void IntsecDialog::on_buttonCancel_clicked()
{
    done(QDialog::Rejected);
}

//表示データ取得
void IntsecDialog::getViewData(bool getIntsecId)
{
    int	index;

    if (getIntsecId)
       _intsecId = _ui->spinBoxId->value();
    _pIntsec->setMapPos(_ui->spinBoxX->value(), _ui->spinBoxY->value(), _ui->spinBoxZ->value());
    index = _ui->comboFirstIntsec->currentIndex();
    if (index != -1)
        _pIntsec->setFirstIntsecIdCon(_ui->comboFirstIntsec->itemData(index).toInt());
}
