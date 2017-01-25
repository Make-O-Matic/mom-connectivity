#include <QSharedPointer>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include <QBluetoothUuid>
#include "extern-plugininfo.h"
#include "glove.h"

Glove::Glove(const QBluetoothDeviceInfo &leftInfo, const QBluetoothDeviceInfo &rightInfo, QObject *parent) :
    QObject(parent)
{
    m_sockets.insert(leftInfo, QSharedPointer<QBluetoothSocket>());
    m_sockets.insert(rightInfo, QSharedPointer<QBluetoothSocket>());
}

void Glove::connect()
{
    foreach (const auto deviceInfo, m_sockets.keys()) {
        QSharedPointer<QBluetoothSocket> socket{new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol)};
        m_sockets[deviceInfo] = socket;
    }
    for (auto it = m_sockets.begin(); it != m_sockets.end(); it++) {
        it.value()->connectToService(it.key().address(), QBluetoothUuid(QBluetoothUuid::SerialPort));
        //connect(it.value(), &QBluetoothSocket::connected, this, ());
    }
}

inline uint qHash(const QBluetoothDeviceInfo &info) {
    return qHash(info.address().toUInt64());
}
