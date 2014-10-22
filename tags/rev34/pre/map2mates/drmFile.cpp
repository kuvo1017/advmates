// DRM ファイルアクセス

#include <Qt>
#include <QFileInfo>
#include <QDir>
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include "app.h"
#include "map.h"
#include "intsec.h"
#include "road.h"
#include "drmFile.h"

using namespace std;

// 2次メッシュコードの一区画のサイズ（緯度経度換算）
#define LAT2ND_SIZE  (5.0 / 60)
#define LON2ND_SIZE  (7.5 / 60)

drmFileNode::drmFileNode(double lat, double lon)
{
    _intsec = NULL;
    _conFileNode = NULL;
    _lat = lat;
    _lon = lon;
}

void drmFileNode::setConFileNode(drmFileNode* conFileNode)
{
    _conFileNode = conFileNode;
//  cout << "con file node lat " << _lat << " lon " << _lon << endl;
}

Intsec* drmFileNode::createIntsec()
{
    if (_intsec == NULL) {
        if (_conFileNode != NULL)
            _intsec = _conFileNode->createIntsec();
        else {
            _intsec = app.map()->createIntsec();
            setIntsecMapPos();
        }
    }

    return _intsec;
}

void drmFileNode::setIntsecMapPos()
{
    if (_intsec != NULL)
        _intsec->setMapPos(app.lonToX(_lon), app.latToY(_lat), 0);
}

drmFile::drmFile()
{
    _fileLatMinJapan = _fileLonMinJapan = APP_LATLON_MAX;
    _latMin = _lonMin =  APP_LATLON_MAX;
    _latMax = _lonMax = -APP_LATLON_MAX;
    _recordCnt = 0;
}

drmFile::~drmFile()
{
    map<string, drmFileNode*>::iterator itn = _nodes.begin();
    while (itn != _nodes.end()) {
        delete (*itn).second;
        itn++;
    }
}

void drmFile::prevRead()
{
    for (int i = 0; i < (int)app.inputDRM_Files()->size(); i++)
        cout << "read drm files \"" << app.inputDRM_Files()->at(i) << "\"" << endl;
}

bool drmFile::read()
{
//  int roadCnt = 0;

    for (int i = 0; i < (int)app.inputDRM_Files()->size(); i++) {
        QFileInfo fi(app.inputDRM_Files()->at(i).c_str());
        QStringList files = fi.absoluteDir().entryList(QStringList(fi.fileName()),
                                                       QDir::Files | QDir::NoSymLinks);
        for (int j = 0; j < (int)files.size(); j++) {
//          cout << files.at(j).toStdString() << endl;
            stringstream ss;
            ss << fi.absolutePath().toStdString() << "/" << files.at(j).toStdString();
            _allFileNames.push_back(ss.str());
        }
    }
    if (_allFileNames.size() == 0) {
        cerr << "no drm file error" << endl;
        return false;
    }

    for (int i = 0; i < (int)_allFileNames.size(); i++) {
        _fileName = _allFileNames.at(i);

        cout << "read drm \"" << _fileName << "\" records..." << endl;

        _file = new QFile(_fileName.c_str());
        if (!_file->open(QIODevice::ReadOnly)) {
            cerr << "drm file open error" << endl << _fileName << endl;
            return false;
        }

        _recordCnt = 1;
        bool inLatLon = false;
        int cont = 0;
        while (!_file->atEnd()) {

            // 1レコード取得、SJIS なら改行分飛ばす、念のため最大継続数チェック
            if (cont >= DRM_RECORD_CONT_MAX) {
                cerr << "drm file too much continue record " << _recordCnt
                     << endl << _fileName << endl;
                return false;
            }
            if (_file->read((char*)_record[cont], DRM_RECORD_SIZE) != DRM_RECORD_SIZE) {
                cerr << "drm file read error record " << _recordCnt
                     << endl << _fileName << endl;
                return false;
            }
            char dummy[16];
            if (app.drmCharCode() == APP_DRM_CC_SJIS)
                _file->read(dummy, 2);

            // ID 取得
            int id = getDigit(cont, 0, 2);
            if (id < 0)
                return false;
            else if (_recordCnt == 1 && id != 11) {
                cerr << "drm file first record error" << endl << _fileName << endl;
                return false;
            }

            //管理データ（その１）、2次メッシュコード、緯度経度判定
            if (id == 11) {
                int lat1, lat2, lon1, lon2;
                getString(0, 2, 6, &_code2nd);
                lat1 = getDigit(0, 2, 2);
                lon1 = getDigit(0, 4, 2);
                lat2 = getDigit(0, 6, 1);
                lon2 = getDigit(0, 7, 1);
                if (lat1 < 0 || lat2 < 0 || lon1 < 0 || lon2 < 0) {
                    cerr << "drm file 2nd mesh code error" << endl << _fileName << endl;
                    return false;
                }
                double lat1stConv = (double)lat1 * 2.0 / 3.0;
                double lon1stConv = (double)lon1 + 100.0;
                double fileLatMin, fileLonMin, fileLatMax, fileLonMax;
                fileLatMin = lat1stConv +  lat2      * LAT2ND_SIZE;
                fileLatMax = lat1stConv + (lat2 + 1) * LAT2ND_SIZE;
                fileLonMin = lon1stConv +  lon2      * LON2ND_SIZE;
                fileLonMax = lon1stConv + (lon2 + 1) * LON2ND_SIZE;
                _fileLatMinJapan = fileLatMin;
                _fileLonMinJapan = fileLonMin;
                app.latLonJapanToWorld(&fileLatMin, &fileLonMin);
                app.latLonJapanToWorld(&fileLatMax, &fileLonMax);
                if (app.inLatLonMinMaxArea(fileLatMin, fileLatMax,
                                           fileLonMin, fileLonMax)) {
                    inLatLon = true;
//                  cout << "in lat lon" << endl;
                }
            }

            //緯度経度内なら全道路ノードデータ、全道路リンクデータ、リンクのみ継続あり
            if (inLatLon) {
                if (id == 31) {
                    if (!getNode())
                        return false;
                }
                else if (id == 32) {
                    string contCode;
                    getString(cont, 255, 1, &contCode);
//                  cout << "id " << id << " contCode " << contCode
//                       << " cont " << cont << endl;
                    if (contCode == "1")
                        cont++;
                    else {
                        if (!getLink(cont + 1))
                            return false;
                        cont = 0;
//                      roadCnt++;
//                      if (roadCnt > 10)
//                          return true;
                    }
                }
            }

            _recordCnt++;
        }

        _file->close();
    }

    app.setLatLonMid((_latMin + _latMax) / 2, (_lonMin + _lonMax) / 2);
    map<string, drmFileNode*>::iterator itn = _nodes.begin();
    while (itn != _nodes.end()) {
        (*itn).second->setIntsecMapPos();
        itn++;
    }

    return true;
}

bool drmFile::getNode()
{
    string nodeCnt, conCode2nd, conNodeCnt;

    getString(0, 2, 5, &nodeCnt);
    int x = getDigit(0, 7,  5);
    int y = getDigit(0, 12, 5);
    if (x < 0 || y < 0)
        return false;
    double lat, lon;
    xyToLatLon(x, y, &lat, &lon);
    getString(0, 18, 6, &conCode2nd);
    getString(0, 24, 5, &conNodeCnt);
//  cout << "node " << _code2nd << nodeCnt << " x " << x << " y " << y << endl;
//  cout << "node lat " << lat << " lon " << lon << endl;

    bool border;
    if (app.inLatLonMinMax(lat, lon, &border)) {
        setLatLonMinMax(lat, lon);
        drmFileNode* node = new drmFileNode(lat, lon);
        string id = _code2nd + nodeCnt;
        _nodes.insert(make_pair(id, node));
        string conId = conCode2nd + conNodeCnt;
        map<string, drmFileNode*>::iterator itn = _nodes.find(conId);
        if (itn != _nodes.end())
            node->setConFileNode(itn->second);
        Intsec* intsec = node->createIntsec();
        intsec->setBorder(border);
//      cout << "node id "  << id << " lat " << lat << " lon " << lon << endl;
    }

    return true;
}

bool drmFile::getLink(int contNum)
{
    string nodeCnt1, nodeCnt2, nodeId1, nodeId2;
    int laneForward, laneBackward;

    getString(0, 2, 5, &nodeCnt1);
    getString(0, 7, 5, &nodeCnt2);
    int type    = getDigit(0, 15, 1);
    int laneNum = getDigit(0, 27, 1);
    int control = getDigit(0, 28, 1);
    if (type == 0)
        type = 10;
    if (type < app.drmRT_Min() || type > app.drmRT_Max())
        return true;
    if (control == 2 || control == 3)  //通行禁止、実際はないが念のため
        return true;
    nodeId1 = _code2nd + nodeCnt1;
    nodeId2 = _code2nd + nodeCnt2;
    map<string, drmFileNode*>::iterator itn1 = _nodes.find(nodeId1);
    map<string, drmFileNode*>::iterator itn2 = _nodes.find(nodeId2);
    if (itn1 != _nodes.end() && itn2 != _nodes.end()) {

        //一方通行、正方向
        if      (control == 4 || control == 6 || control == 8) {
            if (laneNum == 0)
                laneForward = 1;
            else
                laneForward = laneNum / 2;
            laneBackward = 0;
        }

        //一方通行、逆方向
        else if (control == 5 || control == 7) {
            laneForward = 0;
            if (laneNum == 0)
                laneBackward = 1;
            else
                laneBackward = laneNum / 2;
        }

        //両方向
        else {
            if (laneNum == 0)
                laneForward = laneBackward = 1;
            else {
                laneForward = (laneNum + 1) / 2;
                laneBackward = laneNum - laneForward;
            }
        }

        Intsec* intsec1 = (*itn1).second->createIntsec();
        Intsec* intsec2 = (*itn2).second->createIntsec();
        if (!app.drmInterOp())
            app.map()->createRoadPackage(intsec1, intsec2,
                                         laneForward, laneBackward);
        else {
            //補間点処理
            int interOpNum = getDigit(0, 38, 3);
            if (interOpNum < 2) {
                cerr << "drm file invalid interpolation error" << endl
                     << _fileName << endl;
                return false;
            }
            Intsec* intsecInterOp1 = intsec1;
//          cout << "road " << (*itn1).first << " " << (*itn2).first
//               << " inter " << interOpNum << endl;
            for (int i = 1; i < interOpNum - 1; i++) {
                int cont = i / 21;
                int j    = i % 21;
//              cout << "inter op " << interOpNum << " cont num " << contNum << endl;
//              cout << "cont " << cont << " i " << i << " j " << j << endl;
                if (cont >= contNum) {
                    cerr << "drm file link continue error" << endl << _fileName << endl;
                    return false;
                }
                int x = getDigit(cont, 41 + j * 10, 5);
                int y = getDigit(cont, 46 + j * 10, 5);
//              cout << "road " << (*itn1).first << " " << (*itn2).first
//                   << " inter " << interOpNum << " x " << x << " y " << y << endl;
                if (x < 0 || y < 0)
                    return false;
                double lat, lon;
                xyToLatLon(x, y, &lat, &lon);
                bool border;
                //完全に道路を切る、実際は境界に入るので意味ないと思われるが念のため
                if (!app.inLatLonMinMax(lat, lon, &border)) {
                    intsecInterOp1 = NULL;
                    continue;
                }
                drmFileNode* node = new drmFileNode(lat, lon);
                char buff[32];
                sprintf(buff, "%04d", i);
                string id = _code2nd + nodeCnt1 + nodeCnt2 + buff;
//              cout << "inter op id " << id << endl;
                _nodes.insert(make_pair(id, node));
                Intsec* intsecInterOp2 = node->createIntsec();
                intsecInterOp2->setInterOp(true);
                intsecInterOp2->setBorder(border);
                if (intsecInterOp1 != NULL)
                    app.map()->createRoadPackage(intsecInterOp1, intsecInterOp2,
                                                 laneForward, laneBackward);
                intsecInterOp1 = intsecInterOp2;
            }
            if (intsecInterOp1 != NULL)
                app.map()->createRoadPackage(intsecInterOp1, intsec2,
                                             laneForward, laneBackward);
        }

/*
        string base;
        getString(30, 8, &base);
        cout << "lane num " << laneNum << " type " << type << " " << " base " << base << endl;
*/
    }

    return true;
}

unsigned char drmFile::getCode(int cont, int pos)
{
    if (app.drmCharCode() == APP_DRM_CC_EBCDIC)
        return _record[cont][pos] - 0xC0;
    else
        return _record[cont][pos];
}

int drmFile::getDigit(int cont, int pos, int num)
{
    int val, valAll;

    valAll = 0;
    for (int i = pos; i < pos + num; i++) {
        val = (int)getCode(cont, i) - (int)'0';
        if (val < 0 || val > 9) {
            cerr << "drm file digit error record " << _recordCnt
                 << endl << _fileName << endl;
            return -1;
        }
        valAll = val + valAll * 10;
    }

    return valAll;
}

void drmFile::getString(int cont, int pos, int num, string* str)
{
    (*str) = "";
    for (int i = pos; i < pos + num; i++)
        str->push_back(getCode(cont, i));
}

void drmFile::setLatLonMinMax(double lat, double lon)
{
    if (_latMin > lat)
        _latMin = lat;
    if (_lonMin > lon)
        _lonMin = lon;
    if (_latMax < lat)
        _latMax = lat;
    if (_lonMax < lon)
        _lonMax = lon;
}

void drmFile::xyToLatLon(int x, int y, double* lat, double* lon)
{
    *lat = (double)y * LAT2ND_SIZE / 10000.0 + _fileLatMinJapan;
    *lon = (double)x * LON2ND_SIZE / 10000.0 + _fileLonMinJapan;
    app.latLonJapanToWorld(lat, lon);
}
