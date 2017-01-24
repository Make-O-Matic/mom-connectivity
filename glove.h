#ifndef GLOVE_H
#define GLOVE_H

#include <QHash>
#include <QSharedPointer>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include "extern-plugininfo.h"

class Glove : public QObject
{
    Q_OBJECT
public:
    explicit Glove(const QBluetoothDeviceInfo &leftInfo, const QBluetoothDeviceInfo &rightInfo, QObject *parent = 0);
    void connect();

private:
    QHash<QBluetoothDeviceInfo, QSharedPointer<QBluetoothSocket>> m_sockets;
};

#endif // GLOVE_H
