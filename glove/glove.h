#ifndef GLOVE_H
#define GLOVE_H

#include <functional>
#include <vector>
#include <unordered_map>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <glibmm.h>
#include <giomm.h>
#include <mongocxx/client.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>

class Glove
{
public:
    explicit Glove(const std::string &leftMAC, const std::string &rightMAC,
                   const std::function<bool()> &isRecording);
    void connectDevice();

    enum Connected { none, left, right, both };

private:
    void read(const std::string &device, const boost::system::error_code&, std::size_t length);

    mongocxx::client m_dbConnection{mongocxx::uri{}};
    mongocxx::database m_db{m_dbConnection["makeomatic"]};
    mongocxx::collection  m_trainsets{m_db["trainsets"]};

    std::function<bool()> m_isRecording;

    boost::asio::io_service m_ioService;
    struct DataConnection {
        std::string MAC;
        std::unique_ptr<boost::asio::posix::stream_descriptor> stream;//{m_ioService};
        boost::asio::streambuf buffer;
        std::vector<uint8_t> unpackedBuffer;
        std::unique_ptr<std::thread> thread;
    };
    std::unordered_map<std::string, DataConnection> m_dataConnections;

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
