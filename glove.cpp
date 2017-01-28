#include <functional>
#include <QSharedPointer>
#include <QVariant>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include <QBluetoothUuid>
//#include "extern-plugininfo.h"
#include "glove.h"
//#include <iostream>
Glove::Glove(const QString &name, const QString &leftMAC, const QString &rightMAC,
             const std::function<bool()> &isRecording, QObject *parent) :
    QObject(parent), m_isRecording(isRecording)
{   
    auto leftAddress = QBluetoothAddress(leftMAC);
    auto rightAddress = QBluetoothAddress(rightMAC);
    auto leftInfo = QBluetoothDeviceInfo(leftAddress, name, 0);
    auto rightInfo = QBluetoothDeviceInfo(rightAddress, name, 0);
    m_connections.insert(leftInfo, QSharedPointer<QBluetoothSocket>());
    m_connections.insert(rightInfo, QSharedPointer<QBluetoothSocket>());
}

void Glove::connectDevice()
{
    foreach (const auto deviceInfo, m_connections.keys()) {
        QSharedPointer<QBluetoothSocket> socket{new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol)};
        connect(socket.data(), &QBluetoothSocket::connected, this, &Glove::emitConnectionChanged);
        connect(socket.data(), &QBluetoothSocket::disconnected, this, &Glove::emitConnectionChanged);
        connect(socket.data(), &QBluetoothSocket::readyRead, this, [deviceInfo, this](){ read(deviceInfo); });

        m_connections[deviceInfo] = socket;
    }

    for (auto device = m_connections.begin(); device != m_connections.end(); device++) {
        device.value()->connectToService(device.key().address(), QBluetoothUuid(QBluetoothUuid::SerialPort));
    }
}

void Glove::read(const QBluetoothDeviceInfo deviceInfo)
{
    /*
    struct message {
        accelerationX;
        rotationX;
        rfid;
        grasp;
        userInputButton;
        handIsinGlove;
        additionalButton;
        isSameRFIDTag;
    } dataPoint;
*/
    if (!m_isRecording()) {
        m_connections[deviceInfo]->readAll();
        return;
    }
    //std::cerr << "a" << std::endl;
    char c;
    auto &packet = m_packets[deviceInfo];
    while (m_connections[deviceInfo]->getChar(&c)) {
        if (c == '\0') {
            //cobs_decode(packet.data(), packet.size(), &dataPoint);
            //db
            packet.clear();
        } else {
            packet += c;
        }
    }
     //   std::cerr.write(&c, 1);
    //std::cerr << std::endl;
    //emit, continue
}

void Glove::emitConnectionChanged() {
    Connected state = Connected::none;
    if (m_connections.begin().value()->state() == QBluetoothSocket::ConnectedState)
        state = Connected::left;
    if ((++m_connections.begin()).value()->state() == QBluetoothSocket::ConnectedState)
        state = static_cast<Connected>(state & Connected::right);
    emit connectionChanged(state);
}

inline uint qHash(const QBluetoothDeviceInfo &info) {
    return qHash(info.address().toUInt64());
}
