#ifndef GLOVE_H
#define GLOVE_H

#include <functional>
#include <QHash>
#include <QSharedPointer>
#include <QVariant>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include <mongocxx/client.hpp>
#include <mongocxx/uri.hpp>
//#include "extern-plugininfo.h"

class Glove : public QObject
{
    Q_OBJECT
public:
    explicit Glove(const QString &name, const QString &leftMAC, const QString &rightMAC,
                   const std::function<bool()> &isRecording, QObject *parent = 0);
    void connectDevice();

    enum Connected { none, left, right, both };
    Q_ENUM(Connected)

signals:
    void connectionChanged(QVariant state);

private:
    void read(const QBluetoothDeviceInfo deviceInfo);

    mongocxx::client m_dbConnection{mongocxx::uri{}};
    mongocxx::database m_db{m_dbConnection["makeomatic"]};
    mongocxx::collection  m_trainsets{m_db["trainsets"]};

    std::function<bool()> m_isRecording;

    QHash<QBluetoothDeviceInfo, QSharedPointer<QBluetoothSocket>> m_connections;
    QHash<QBluetoothDeviceInfo, QByteArray> m_packets;
    QHash<QBluetoothDeviceInfo, QVector<uint8_t>> m_data;

private slots:
    void emitConnectionChanged();
};

#endif // GLOVE_H
