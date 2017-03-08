#ifndef GLOVE_H
#define GLOVE_H
#include <memory>
#include <functional>
#include <chrono>
#include <string>
#include <vector>
#include <unordered_map>
#include <future>
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
                   const std::string &leftID, const std::string &rightID);
    ~Glove() noexcept;
    
    void connect();
    void disconnect();
    
    std::string now() const;

    enum Connected { none, left, right, both };
    void setTrainsetExercise(const std::string &trainset,
        const int step, const std::string &mutation, const std::string &mutationIndex);

private:
    void ingestData(const std::string &device, const boost::system::error_code&, std::size_t length);

    std::chrono::time_point<std::chrono::high_resolution_clock> m_connectionTime;

    const mongocxx::instance &m_dbDriver{mongocxx::instance::current()};

    std::function<void(Connected)> m_setConnected;
    std::function<bool()> m_isRecording;

    struct Connections {
        std::string MAC;
        std::string id;
        boost::asio::io_service ioService;
        std::unique_ptr<boost::asio::generic::stream_protocol::socket> socket;
        boost::asio::streambuf buffer;
        std::vector<uint8_t> unpackedBuffer;
        mongocxx::client db;
        mongocxx::collection collection;
        std::future<void> run;
    };
    std::unordered_map<std::string, Connections> m_dataConnections;
	std::unique_ptr<Gio::DBus::Connection> m_dbus;
	int m_profileId;
	Glib::RefPtr<Glib::MainLoop> m_gLoop;
    std::future<void> m_runGLoop;

    void execute(const Glib::RefPtr<Gio::DBus::Connection>& /* connection */,
                               const std::string& /* sender */,
                               const std::string& /* object_path */,
                               const std::string& /* interface_name */,
                               const std::string& method,
                               const Glib::VariantContainerBase& parameters,
                               const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation);
    const Gio::DBus::InterfaceVTable m_profileInterface{std::bind(&Glove::execute, this, _1, _2, _3 , _4, _5, _6, _7)};
    void updateConnected();
    
    int m_step;
	std::string m_mutation;
	std::string m_mutationIndex;
};

#endif // GLOVE_H
