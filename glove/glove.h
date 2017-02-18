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

using namespace std::placeholders;

class Glove
{
public:
    explicit Glove(const std::string &leftMAC, const std::string &rightMAC,
                   const std::function<void(int)> &setConnected,
                   const std::function<bool()> &isRecording,
                   const std::string &leftUUID, const std::string &rightUUID);
    ~Glove() noexcept;
    
    void connect();
    void disconnect();
    
    std::string now() const;

    enum Connected { none, left, right, both };
    void setTrainset(const std::string &trainset);
    std::string m_trainsetId;

private:
    void read(const std::string &device, const boost::system::error_code&, std::size_t length);

    std::chrono::time_point<std::chrono::high_resolution_clock> m_connectionTime;

    const mongocxx::instance &m_dbDriver{mongocxx::instance::current()};

    std::function<void(Connected)> m_setConnected;
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

	Glib::RefPtr<Glib::MainLoop> m_gLoop;
    std::unique_ptr<std::thread> m_gThread;
    int m_profileId;

    void on_method_call(const Glib::RefPtr<Gio::DBus::Connection>& /* connection */,
                               const std::string& /* sender */,
                               const std::string& /* object_path */,
                               const std::string& /* interface_name */,
                               const std::string& method_name,
                               const Glib::VariantContainerBase& parameters,
                               const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation);
    const Gio::DBus::InterfaceVTable m_interfaceVtable{std::bind(&Glove::on_method_call, this, _1, _2, _3 , _4, _5, _6, _7)};
    void updateConnected();
};

#endif // GLOVE_H
