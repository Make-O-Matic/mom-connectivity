#ifndef GLOVE_H
#define GLOVE_H
#include <limits.h>
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <glibmm.h>
#include <giomm.h>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>

class Glove
{
public:
    explicit Glove(const std::string &leftMAC, const std::string &rightMAC,
                   const std::function<bool()> &isRecording,
                   const std::string &leftUUID, const std::string &rightUUID);
    ~Glove();
    void connect();

    enum Connected { none, left, right, both };

private:
    void read(const std::string &device, const boost::system::error_code&, std::size_t length);

    std::chrono::time_point<std::chrono::high_resolution_clock> m_connectionTime;

    const mongocxx::instance &m_dbDriver{mongocxx::instance::current()};
    //const mongocxx::client m_dbConnection{mongocxx::uri{}};
    //const mongocxx::database m_db{m_dbConnection["makeomatic"]};
    //mongocxx::collection  m_trainsets{m_db["trainsets"]};

    std::function<bool()> m_isRecording;

    struct Connections {
        std::string MAC;
        std::string uuid;
        boost::asio::io_service ioService;
        std::unique_ptr<boost::asio::generic::stream_protocol::socket> socket;
        boost::asio::streambuf buffer;
        std::vector<uint8_t> unpackedBuffer;
        mongocxx::client db;
        mongocxx::collection collection;
        std::unique_ptr<std::thread> thread;
    };
    std::unordered_map<std::string, Connections> m_dataConnections;

	Glib::RefPtr<Glib::MainLoop> m_glib;
	Glib::RefPtr<Gio::DBus::Connection> m_dbusConn;
    std::unique_ptr<std::thread> m_dbus;
    std::unique_ptr<std::thread> m_bt;

    void on_method_call(const Glib::RefPtr<Gio::DBus::Connection>& /* connection */,
                               const Glib::ustring& /* sender */,
                               const Glib::ustring& /* object_path */,
                               const Glib::ustring& /* interface_name */,
                               const Glib::ustring& method_name,
                               const Glib::VariantContainerBase& parameters,
                               const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation);
    const Gio::DBus::InterfaceVTable m_interfaceVtable{sigc::mem_fun(*this, &Glove::on_method_call)};
    void on_bus_acquired(const Glib::RefPtr<Gio::DBus::Connection>& connection, const Glib::ustring&);
    void onConnectionChanged();
};

#endif // GLOVE_H
