#include <functional>
#include <QSharedPointer>
#include <QVariant>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include <QBluetoothUuid>
extern "C" {
    #define restrict
    #include "cobs/cobs.h"
    #undef restrict
}
#include "extern-plugininfo.h"
#include "packet.h"
#include "glove.h"

Glove::Glove(const QString &name, const QString &leftMAC, const QString &rightMAC,
             const std::function<bool()> &isRecording, QObject *parent) :
    QObject(parent), m_isRecording(isRecording)
{
    const auto leftAddress = QBluetoothAddress(leftMAC);
    const auto rightAddress = QBluetoothAddress(rightMAC);
    const auto leftInfo = QBluetoothDeviceInfo(leftAddress, name, 0);
    const auto rightInfo = QBluetoothDeviceInfo(rightAddress, name, 0);
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
        connect(socket.data(), static_cast<void(QBluetoothSocket::*)(QBluetoothSocket::SocketError)>(&QBluetoothSocket::error),
                [socket](QBluetoothSocket::SocketError error){ qCDebug(dcMakeOMatic) << error << socket->errorString(); });

        m_connections[deviceInfo] = socket;
    }

    for (auto device = m_connections.begin(); device != m_connections.end(); device++) {
        device.value()->connectToService(device.key().address(), QBluetoothUuid::SerialPort);
    }
}

void Glove::read(const QBluetoothDeviceInfo deviceInfo)
{
    /*
    struct message {
        accelerationX; ax
        rotationX; ex
        rfid;
        grasp; myo
        userInputButton; key char
        handIsinGlove; capsens
        additionalButton; sw
        isSameRFIDTag; lastnr
    } dataPoint;
*/
    if (!m_isRecording()) {
        m_connections[deviceInfo]->readAll();
        return;
    }

    char c;
    auto &packet = m_packets[deviceInfo];
    auto &packedData = m_data[deviceInfo];
    while (m_connections[deviceInfo]->getChar(&c)) {
        if (c == '\0') {
            packedData.reserve(packet.size());
            cobs_decode(reinterpret_cast<uint8_t*>(packet.data()), packet.size(), packedData.data());
            //auto data =
                    reinterpret_cast<Packet*>(packedData.data());

            packet.clear();
        } else {
            packet += c;
        }
    }
    //emit, continue
}

void Glove::emitConnectionChanged() {
    Connected state = Connected::none;
    if (m_connections.begin().value()->state() == QBluetoothSocket::ConnectedState)
        state = Connected::left;
    if ((++m_connections.begin()).value()->state() == QBluetoothSocket::ConnectedState)
        state = static_cast<Connected>(state | Connected::right);
    QVariant stateString;
    stateString.setValue(state);
    emit connectionChanged(stateString);
}

inline uint qHash(const QBluetoothDeviceInfo &info) {
    return qHash(info.address().toUInt64());
}
