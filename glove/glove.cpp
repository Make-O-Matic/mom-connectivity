#include <stdexcept>
#include <memory>
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
#include "packet.h"
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
            auto path = MAC;
            for (char &c : path) {
                c = std::toupper(c);
                if (c == ':')
                    c = '_';
            }
            path.insert(0, "/org/bluez/hci0/dev_");
            auto &connections = m_dataConnections[path];
            connections.MAC = MAC;
            connections.id = (MAC == leftMAC ? leftID : rightID);
            connections.db = mongocxx::client{mongocxx::uri{}};
        }
        Gio::init();
        Glib::init();
        m_gLoop = Glib::MainLoop::create();
        auto dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);
        const auto dbusProxy = Gio::DBus::Proxy::create_sync(dbus, "org.freedesktop.DBus", "/org/freedesktop/DBus", "org.freedesktop.DBus");
        dbusProxy->call_sync("RequestName", Glib::VariantContainerBase::create_tuple(
                                 std::vector<Glib::VariantBase>({ Var<ustring>::create("org.makeomatic"),
                                                                  Var<guint32>::create(0)})));
        
        auto profile = Gio::DBus::NodeInfo::create_for_xml(
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
                    );
      
        m_profileId = dbus->register_object("/org/makeomatic/glove",
                                              profile->lookup_interface(),
                                              m_profileInterface);
        const auto profileManager = Gio::DBus::Proxy::create_sync(dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1");

        auto options =
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
                        }));
        Var<ustring> path;
        Var<ustring>::create_object_path(path, "/org/makeomatic/glove");
        const auto parameters = Glib::VariantContainerBase::create_tuple(
                    std::vector<Glib::VariantBase>({ path,
                                                     Var<ustring>::create("1101"),
                                                     options }));
        profileManager->call_sync("RegisterProfile", parameters);
}

Glove::~Glove() {
    try {
    disconnect();
        } catch (const std::runtime_error &error)   {
    }
        auto dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);
    const auto profileManager = Gio::DBus::Proxy::create_sync(dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1");
		
		
        Var<ustring> path;
        Var<ustring>::create_object_path(path, "/org/makeomatic/glove");
        const auto parameters = Glib::VariantContainerBase::create_tuple(path);
        try {
        profileManager->call_sync("UnregisterProfile", parameters);
        dbus->unregister_object(m_profileId);
        const auto dbusProxy = Gio::DBus::Proxy::create_sync(dbus, "org.freedesktop.DBus", "/org/freedesktop/DBus", "org.freedesktop.DBus");
        dbusProxy->call_sync("ReleaseName", Glib::VariantContainerBase::create_tuple(
                                 Var<ustring>::create("org.makeomatic")));
                                         } catch (const Gio::Error &error)   {
    }
}

void Glove::connect()
{
        m_connectionTime = std::chrono::high_resolution_clock::now();

        m_runGLoop = std::async(std::launch::async, [this](){ m_gLoop->run(); });
        
        auto dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);
            for (auto &connections : m_dataConnections) {
				try {
                dbus->call_sync(connections.first, "org.bluez.Device1",
                                  "ConnectProfile", Glib::VariantContainerBase::create_tuple(
                                      Var<ustring>::create("1101")),
                                  "org.bluez");//, G_MAXINT);
    } catch (const Glib::Error &error)   {
        throw std::runtime_error(error.what());
    }
            }
                m_dbus.reset(dbus.release());
}

void Glove::setTrainsetExercise(const std::string &trainset,
    const int step, const std::string &mutation, const std::string &mutationIndex) {
    std::promise<void> donePromise;
    std::shared_future<void> done{donePromise.get_future()};
    for (auto &connections : m_dataConnections) {
        std::promise<void> ready;
		auto setTrainsetAndWait = std::make_shared<std::packaged_task<void()>>([&connections, &trainset, &ready, done](){ 
            if (!trainset.empty())
                connections.second.collection = connections.second.db["makeomatic"][trainset];
            ready.set_value();
            done.wait();
		});
		connections.second.ioService.post([setTrainsetAndWait](){ setTrainsetAndWait->operator()(); });
        ready.get_future().wait();
    }

    if (trainset.empty()) {
        m_step = step;
        m_mutation = mutation;
        m_mutationIndex = mutationIndex;
    }
        
    donePromise.set_value();
    /*
                     connections.db["makeomatic"].run_command( document{} <<
                                                      "collMod" << connections.second.db["makeomatic"][trainset] <<
                                                       "usePowerOf2Sizes" << true <<
                                                   finalize );
                                                   */
}

void Glove::disconnect() {

		        auto dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);
        for (auto &connections : m_dataConnections) {
    if (!connections.second.run.valid())
        continue;
            dbus->call_sync(connections.first, "org.bluez.Device1",
                              "DisconnectProfile", Glib::VariantContainerBase::create_tuple(
                                  Var<ustring>::create("1101")),
                              "org.bluez");//, G_MAXINT);
connections.second.run.get();
}
    if (m_runGLoop.valid()) {
        m_gLoop->quit();
        m_runGLoop.get();
    }
}

std::string Glove::now() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(now - m_connectionTime);
    auto timeString = std::to_string(time.count());
    return timeString;
}


void Glove::ingestData(const std::string &device, const boost::system::error_code& error, std::size_t length)
{
        if (!error) {
            if (!m_dataConnections.at(device).socket) 
                return;
			auto now = Glove::now();

            auto &connections = m_dataConnections.at(device);
            if (!m_isRecording()) {
                connections.buffer.consume(length);
                boost::asio::async_read_until(*(connections.socket), connections.buffer, '\0',
                                              std::bind(&Glove::ingestData, this, device, _1, _2));
                return;
            }

            if (length) {
                connections.unpackedBuffer.reserve(length - 1);
                cobs_decode(reinterpret_cast<uint8_t*>(connections.buffer.gptr()), length - 1,
                            connections.unpackedBuffer.data());
                connections.buffer.consume(length);
                auto data = reinterpret_cast<Packet*>(connections.unpackedBuffer.data());
                mongocxx::options::update options;
                options.upsert(true);
                std::string rfid{(char*)(data->rfid),(char*)(data->rfid)+ID_LENGTH};
                if (rfid == std::string(ID_LENGTH, '\0'))
                    rfid = std::string(ID_LENGTH, '0');
                auto doc =
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
                                              "sensorA" << data->graspa <<
                                              "sensorB" << data->graspb <<
                                              "sensorC" << data->graspc <<
                                          hashStop <<
                                          "acceleration" << hashStart <<
                                              "x" << data->ax <<
                                              "y" << data->ay <<
                                              "z" << data->az <<
                                          hashStop <<
                                          "rotation" << hashStart <<
                                              "x" << data->ex <<
                                              "y" << data->ey <<
                                              "z" << data->ez <<
                                          hashStop <<
                                          "interface" << hashStart <<
                                              "userInputButton" << (bool)data->key <<
                                              "handIsInGlove" << (bool)data->wear <<
                                      //         hashStop <<
                                      //   hashStop <<
                                      hashStop << hashStop << finalize;//, options);
                connections.collection.insert_one(//update
                                                  //                    document{} << "_id" << 1 << finalize,
                                                  std::move(doc));//, options);
                                                  
                boost::asio::async_read_until(*(connections.socket), connections.buffer, '\0',
                                              std::bind(&Glove::ingestData, this, device, _1, _2));

            }
        } else {
            std::cerr << error.message() << std::endl;
        }
}

void Glove::execute(const Glib::RefPtr<Gio::DBus::Connection>& /* connection */,
                           const std::string& /* sender */,
                           const std::string& /* object_path */,
                           const std::string& /* interface_name */,
                           const std::string& method,
                           const Glib::VariantContainerBase& parameters,
                           const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation) {

    if(method == "NewConnection") {
        Var<ustring> gDevice;
        parameters.get_child(gDevice, 0);
        std::string device = gDevice.get();
        const auto fd = 
            invocation->get_message().release()->get_unix_fd_list().release()->get(0);
        auto &connections = m_dataConnections.at(device);
        connections.socket.reset(
                    new boost::asio::generic::stream_protocol::socket{connections.ioService});
        connections.socket->assign(boost::asio::generic::stream_protocol(AF_BLUETOOTH,3),
                                   fd);
        connections.run = std::async(std::launch::async, [&connections, device, this](){
                                                     boost::asio::async_read_until(*connections.socket, connections.buffer, '\0',
                                                     std::bind(&Glove::ingestData, this, device, _1, _2));
                                                     updateConnected();
                                                     connections.ioService.run();
                                                 });
        invocation->return_value(Glib::VariantContainerBase(nullptr));
    } else if(method == "RequestDisconnection") {
            Var<ustring> gDevice;
            parameters.get_child(gDevice, 0);
            auto &connections = m_dataConnections.at(gDevice.get());
            connections.ioService.post([&connections,this](){ 
            boost::system::error_code error;
            connections.socket->shutdown(boost::asio::generic::stream_protocol::socket::shutdown_both,
                                         error);
            connections.socket->close(error);
            connections.socket.reset(nullptr);
            connections.ioService.stop();
         });    
            invocation->return_value(Glib::VariantContainerBase(nullptr));
    } else if(method == "Release") {
        invocation->return_value(Glib::VariantContainerBase(nullptr));
    } else {
        Gio::DBus::Error error(Gio::DBus::Error::UNKNOWN_METHOD,
                               "Method does not exist.");
        invocation->return_error(error);
    }
}

void Glove::updateConnected() {
    Connected state = Connected::none;
    if (m_dataConnections.begin()->second.socket && m_dataConnections.begin()->second.socket->is_open())
        state = Connected::left;
    if ((++(m_dataConnections.begin()))->second.socket && (++(m_dataConnections.begin()))->second.socket->is_open())
        state = static_cast<Connected>(state | Connected::right);
    m_setConnected(state);
}

