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
#include "MOM-Glove/MOM-Glove/Packet.h"

using namespace std::placeholders;

class Glove
{
public:
    Glove(const std::string &leftMAC, const std::string &rightMAC,
          const std::function<void(int)> &setConnected,
          const std::function<bool()> &isRecording,
          const std::string &leftID, const std::string &rightID);
    ~Glove() noexcept;
    
    void connect();
    void disconnect();
    
    void setTrainsetExercise(const std::string &trainset,
        const int step, const std::string &mutation, const std::string &mutationIndex);
    std::string now() const;
    std::function<void(const std::string&,bool)> processRFID;
    void setIODone(const std::function<void()> &f) { for (auto &connections : m_dataConnections) connections.second.handleIODone = f; }
        
    enum Connected { none, left, right, both };
private:
    class ConnectionsBuffer;
    void processData(ConnectionsBuffer &connections,
                     const boost::system::error_code &error, std::size_t length);
    void execute(const Glib::RefPtr<Gio::DBus::Connection>&,
        const std::string&, const std::string&, const std::string&,
        const std::string& method,
        const Glib::VariantContainerBase& parameters,
        const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation);
    void updateConnected();

    const Gio::DBus::InterfaceVTable m_profileInterface{
        std::bind(&Glove::execute, this, _1, _2, _3 , _4, _5, _6, _7)};
    const std::function<void(Connected)> m_setConnected;
    const std::function<bool()> m_isRecording;
    
    const mongocxx::instance &m_dbDriver{mongocxx::instance::current()};

    class ConnectionsBuffer : boost::asio::streambuf {
    public:
        using boost::asio::streambuf::consume;
        const bool running() const { return (m_run.valid() && m_run.wait_for(std::chrono::seconds(0)) != std::future_status::ready); }
        void start(const int socket);
        void stop();
        void setTrainsetAndWait(const std::string &trainset, const std::shared_future<void> &message);
        void requestRead();
        Packet get(const int length);

        bool left;
        std::string id;
        std::function<void(const boost::system::error_code&,std::size_t)> handleRead;
        mongocxx::collection collection;
        bool sent_rfid{false};
        std::function<void()> handleIODone;
    private:
        boost::asio::io_service m_ioService;
        std::unique_ptr<boost::asio::generic::stream_protocol::socket> m_socket;
        std::future<void> m_run;
        const mongocxx::client m_db{mongocxx::uri{}};
        Packet m_packet[2];
    };
    std::unordered_map<std::string, ConnectionsBuffer> m_dataConnections;

    Glib::RefPtr<Glib::MainLoop> m_gLoop;
    Glib::RefPtr<Gio::DBus::Connection> m_dbus;
    Gio::SlotAsyncReady m_null;
    int m_profile;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_connectionTime;
    std::future<void> m_runGLoop;
        
    int m_step;
    std::string m_mutation;
    std::string m_mutationIndex;
};

#endif // GLOVE_H
