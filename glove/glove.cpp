#include <stdexcept>
#include <memory>
#include <utility>
#include <functional>
#include <chrono>
#include <cctype>
#include <string>
#include <map>
#include <iostream>
#include <future>
#include <boost/asio.hpp>
#include <glibmm.h>
#include <giomm.h>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>
//#include <bluetooth/bluetooth.h>
extern "C" {
#define restrict
#include "cobs/cobs.h"
#undef restrict
}
#include "MOM-Glove/MOM-Glove/Packet.h"
#include "glove.h"

using namespace std::placeholders;
using Glib::ustring;
template <typename T>
using Var = Glib::Variant<T>;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
constexpr auto const& hashStart = bsoncxx::builder::stream::open_document;
constexpr auto const& hashStop = bsoncxx::builder::stream::close_document;

Glove::Glove(const std::string &leftMAC, const std::string &rightMAC,
             const std::function<void(int)> &setConnected,
             const std::function<bool()> &isRecording,
             const std::string &leftID, const std::string &rightID) :
    m_setConnected(setConnected), m_isRecording(isRecording)
{
        for (auto MAC : {leftMAC, rightMAC}) {
            auto device{MAC};
            for (char &c : device) {
                c = std::toupper(c);
                if (c == ':')
                    c = '_';
            }
            device.insert(0, "/org/bluez/hci0/dev_");
            auto &connections{m_dataConnections[device]};
            connections.id = (MAC == leftMAC ? leftID : rightID);
            connections.readHandler = std::bind(&Glove::ingestData, this,
                std::ref(connections), _1, _2);
        }

        Gio::init();
        Glib::init();
        m_gLoop = Glib::MainLoop::create();
        m_dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);

        const auto profile{Gio::DBus::NodeInfo::create_for_xml(
                    "<node>"
                    "  <interface name='org.bluez.Profile1'>"
                    "    <method name='Release' />"
                    "    <method name='NewConnection'>"
                    "      <arg type='o' name='device' direction='in' />"
                    "      <arg type='h' name='fd' direction='in' />"
                    "      <arg type='a{sv}' name='fd_properties' direction='in' />"
                    "    </method>"
                    "    <method name='RequestDisconnection'>"
                    "      <arg type='o' name='device' direction='in' />"
                    "    </method>"
                    "  </interface>"
                    "</node>"
                    )};
        m_profileId = m_dbus->register_object("/io/makeomatic/glove",
                                              profile->lookup_interface(),
                                              m_profileInterface);
        const auto profileManager{Gio::DBus::Proxy::create_sync(m_dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1")};

        const auto options{
                Glib::Variant<std::map<Glib::ustring, Glib::VariantBase>>::create(
                    std::map<Glib::ustring, Glib::VariantBase>(
                        {
                            { "Name", Var<ustring>::create("MOM Glove Client") },
                            { "Service", Var<ustring>::create("spp") },
                            { "Channel", Var<guint16>::create(0) },
                            //{ "RequireAuthentication", Var<bool>::create(false) },
                            { "Role", Var<ustring>::create("client") },
                            //{ "Version", Glib::Variant<guint16>::create(1) },
                            { "AutoConnect", Var<bool>::create(false) }
                            //  { "Service", Var<ustring>::create("00001101-0000-1000-8000-00805f9b34fb") },
                            // { "AutoConnect", Var<bool>::create(true) }
                        }))};
        Var<ustring> path;
        Var<ustring>::create_object_path(path, "/io/makeomatic/glove");
        const auto parameters{Glib::VariantContainerBase::create_tuple(
                    std::vector<Glib::VariantBase>({ path,
                                                     Var<ustring>::create("1101"),
                                                     options }))};
        profileManager->call_sync("RegisterProfile", parameters);
}

Glove::~Glove() {
    try {
        disconnect();
    } catch (const std::runtime_error &error)   {
    }
    const auto profileManager = Gio::DBus::Proxy::create_sync(m_dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1");

        Var<ustring> path;
        Var<ustring>::create_object_path(path, "/io/makeomatic/glove");
        const auto parameters{Glib::VariantContainerBase::create_tuple(path)};
        try {
            profileManager->call_sync("UnregisterProfile", parameters);
            m_dbus->unregister_object(m_profileId);
        } catch (const Gio::Error &error)   {
        }
}

void Glove::connect()
{
        m_connectionTime = std::chrono::high_resolution_clock::now();

        m_runGLoop = std::async(std::launch::async, [this](){ m_gLoop->run(); });
            for (auto &connections : m_dataConnections) {
                try {
                    m_dbus->call_sync(connections.first, "org.bluez.Device1",
                                  "ConnectProfile", Glib::VariantContainerBase::create_tuple(
                                      Var<ustring>::create("1101")),
                                  "org.bluez");
                } catch (const Glib::Error &error)   {
                    throw std::runtime_error(error.what() + connections.first);
                }
            }
}

void Glove::disconnect() {
    for (auto &connections : m_dataConnections) {
        if (!connections.second.running())
            continue;
        m_dbus->call_sync(connections.first, "org.bluez.Device1",
                              "DisconnectProfile", Glib::VariantContainerBase::create_tuple(
                                  Var<ustring>::create("1101")),
                              "org.bluez");
    }
    if (m_runGLoop.valid()) {
        m_gLoop->quit();
        m_runGLoop.get();
    }
}

void Glove::setTrainsetExercise(const std::string &trainset,
    const int step, const std::string &mutation, const std::string &mutationIndex) {
    std::promise<void> done;
    std::shared_future<void> doneMessage{done.get_future()};
    for (auto &connections : m_dataConnections) {
        connections.second.setTrainsetAndWait(trainset, doneMessage);
    }

    if (trainset.empty()) {
        m_step = step;
        m_mutation = mutation;
        m_mutationIndex = mutationIndex;
    }
        
    done.set_value();
    /*
                     connections.db["makeomatic"].run_command( document{} <<
                                                      "collMod" << connections.second.db["makeomatic"][trainset] <<
                                                       "usePowerOf2Sizes" << true <<
                                                   finalize );
                                                   */
}

std::string Glove::now() const {
    const auto now{std::chrono::high_resolution_clock::now()};
    const auto time{std::chrono::duration_cast<std::chrono::microseconds>(
        now - m_connectionTime)};
    const auto timeString{std::to_string(time.count())};
    return timeString;
}


void Glove::ingestData(ConnectionsBuffer& connections,
    const boost::system::error_code& error, std::size_t length)
{
        if (!error) {
            if (!connections.running())//socket)
                return;
            const auto now{Glove::now()};

            if (!m_isRecording()) {
                connections.consume(length);
                connections.requestRead();
                return;
            }

            const auto &data{connections.get(length)};
            mongocxx::options::update options;
            options.upsert(true);
            std::string rfid{(char*)(data.rfid),(char*)(data.rfid)+ID_LENGTH};
            if (rfid == std::string(ID_LENGTH, '\0'))
                rfid = std::string(ID_LENGTH, '0');
            auto doc{
                        //    connections.collection.update_one(
                        //               document{} << "_id" << 1 << finalize,
                        //                    document{} << "$push" << hashStart << "collection" << hashStart <<
                        document{} << //"$set" << hashStart << str << hashStart <<
                                      "trainset" << connections.collection.name() <<
                                      "collector" << hashStart <<
                                          "id" << connections.id <<
                                      hashStop <<
                                      "step" << m_step <<
                                      "mutation" << hashStart <<
                                          "id" << m_mutation <<
                                          "index" << m_mutationIndex <<
                                      hashStop <<
                                      "data" << hashStart <<
                                          "stamp" << hashStart <<
                                              "microSeconds" << now <<
                                          hashStop <<
                                          "rfid" << rfid  <<
                                          "grasp" << hashStart <<
                                              "sensorA" << data.graspa <<
                                              "sensorB" << data.graspb <<
                                              "sensorC" << data.graspc <<
                                          hashStop <<
                                          "acceleration" << hashStart <<
                                              "x" << data.ax <<
                                              "y" << data.ay <<
                                              "z" << data.az <<
                                          hashStop <<
                                          "rotation" << hashStart <<
                                              "x" << data.ex <<
                                              "y" << data.ey <<
                                              "z" << data.ez <<
                                          hashStop <<
                                          "interface" << hashStart <<
                                              "userInputButton" << static_cast<bool>(data.key) <<
                                              "handIsInGlove" << static_cast<bool>(data.wear) <<
                                      //         hashStop <<
                                      //   hashStop <<
                                      hashStop << hashStop << finalize};//, options);
            connections.collection.insert_one(//update
                                                  //                    document{} << "_id" << 1 << finalize,
                                                  std::move(doc));//, options);

            connections.requestRead();
        } else {
            std::cerr << error.message() << std::endl;
        }
}

void Glove::execute(const Glib::RefPtr<Gio::DBus::Connection>&,
    const std::string&, const std::string&, const std::string&,
    const std::string& method,
    const Glib::VariantContainerBase& parameters,
    const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation) {

    Var<ustring> device;
    if(method == "NewConnection") {
        parameters.get_child(device, 0);
        const auto socket{
            invocation->get_message().release()->get_unix_fd_list().release()->get(0)};
        auto &connections{m_dataConnections.at(device.get())};
        connections.start(socket);
    } else if(method == "RequestDisconnection") {
        parameters.get_child(device, 0);
        auto &connections{m_dataConnections.at(device.get())};
        connections.stop();
    } else if(method == "Release") {
    } else {
        Gio::DBus::Error error(Gio::DBus::Error::UNKNOWN_METHOD,
                               "Method does not exist.");
        invocation->return_error(error);
        return;
    }
    invocation->return_value(Glib::VariantContainerBase(nullptr));
    updateConnected();
}

void Glove::updateConnected() {
    Connected state{Connected::none};
    if (m_dataConnections.begin()->second.running())
        state = Connected::left;
    if ((++(m_dataConnections.begin()))->second.running())
        state = static_cast<Connected>(state | Connected::right);
    m_setConnected(state);
}

void Glove::ConnectionsBuffer::start(const int socket) {
    m_socket.reset(new boost::asio::generic::stream_protocol::socket{m_ioService});
    m_socket->assign(boost::asio::generic::stream_protocol(AF_BLUETOOTH,3), socket);
    requestRead();
    m_run = std::async(std::launch::async, [this](){ m_ioService.run(); });
}

void Glove::ConnectionsBuffer::stop() {
    m_ioService.post([this](){
        boost::system::error_code error;
        m_socket->shutdown(boost::asio::generic::stream_protocol::socket::shutdown_both,
            error);
        m_socket->close(error);
        m_socket.reset(nullptr);
        m_ioService.stop();
    });
    m_run.get();
}

void Glove::ConnectionsBuffer::setTrainsetAndWait(
    const std::string &trainset, const std::shared_future<void> &message) {
    std::promise<void> ready;
    auto setTrainsetAndWait{std::make_shared<std::packaged_task<void()>>(
       [&trainset, &ready, message, this](){
            if (!trainset.empty())
                collection = m_db["makeomatic"][trainset];
            ready.set_value();
            message.wait();
        })};
    m_ioService.post([setTrainsetAndWait](){ setTrainsetAndWait->operator()(); });
    ready.get_future().wait();
}

void Glove::ConnectionsBuffer::requestRead() {
    boost::asio::async_read_until(*m_socket, *this, '\0', readHandler);
}

const Packet& Glove::ConnectionsBuffer::get(const int length) {
    m_unpackedBuffer.reserve(length - 1);
    cobs_decode(reinterpret_cast<uint8_t*>(gptr()), length - 1, m_unpackedBuffer.data());
    consume(length);
    return *reinterpret_cast<Packet*>(m_unpackedBuffer.data());
}
