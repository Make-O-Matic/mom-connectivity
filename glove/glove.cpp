#include <cctype>
#include <clocale>
#include <functional>
#include <unordered_map>
#include <thread>
#include <boost/asio.hpp>
#include <glibmm.h>
#include <giomm.h>
//#include <bluetooth/bluetooth.h>
extern "C" {
#define restrict
#include "cobs/cobs.h"
#undef restrict
}
#include "packet.h"
#include "glove.h"
#include <iostream>
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
             const std::string &leftUUID, const std::string &rightUUID) :
    m_setConnected(setConnected), m_isRecording(isRecording)
{
    try {
        for (auto MAC : {leftMAC, rightMAC}) {
            auto path = MAC;
            for (char &c : path) {
                c = std::toupper(c);
                if (c == ':')
                    c = '_';
            }
            path.insert(0, "/org/bluez/hci0/dev_");
            m_collectors[path] = (MAC == leftMAC ? leftUUID : rightUUID);
            auto &connections = m_dataConnections[path];
            connections.MAC = MAC;
            connections.db = mongocxx::client{mongocxx::uri{}};
        }

        Gio::init();
        Glib::init();
        m_gLoop = Glib::MainLoop::create();
        m_dbus = Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM);
        const auto dbusProxy = Gio::DBus::Proxy::create_sync(m_dbus, "org.freedesktop.DBus", "/org/freedesktop/DBus", "org.freedesktop.DBus");
        dbusProxy->call_sync("RequestName", Glib::VariantContainerBase::create_tuple(
                                 std::vector<Glib::VariantBase>({ Var<ustring>::create("org.makeomatic"),
                                                                  Var<guint32>::create(0)})));

        auto introspectionData = Gio::DBus::NodeInfo::create_for_xml(
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
        m_profileId = m_dbus->register_object("/org/makeomatic/glove",
                                              introspectionData->lookup_interface(),
                                              m_interfaceVtable);
        const auto profileManager = Gio::DBus::Proxy::create_sync(m_dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1");

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
        Glib::VariantContainerBase parameters = Glib::VariantContainerBase::create_tuple(
                    std::vector<Glib::VariantBase>({ path,
                                                     Var<ustring>::create("1101"),
                                                     options }));
        profileManager->call_sync("RegisterProfile", parameters);
    } catch  (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << std::endl;
        throw std::exception();
    }

}

Glove::~Glove() {
    disconnect();

    const auto profileManager = Gio::DBus::Proxy::create_sync(m_dbus, "org.bluez", "/org/bluez", "org.bluez.ProfileManager1");

    try {
        Var<ustring> path;
        Var<ustring>::create_object_path(path, "/org/makeomatic/glove");
        Glib::VariantContainerBase parameters = Glib::VariantContainerBase::create_tuple(
                    path );
        profileManager->call_sync("UnregisterProfile", parameters);
        m_dbus->unregister_object(m_profileId);
        const auto dbusProxy = Gio::DBus::Proxy::create_sync(m_dbus, "org.freedesktop.DBus", "/org/freedesktop/DBus", "org.freedesktop.DBus");
        dbusProxy->call_sync("ReleaseName", Glib::VariantContainerBase::create_tuple(
                                 Var<ustring>::create("org.makeomatic")));


    } catch  (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << std::endl;
    }
}

void Glove::connect()
{
    try  {
        m_connectionTime = std::chrono::high_resolution_clock::now();

        m_gThread.reset(new std::thread{[this](){
                                            try {
                                                m_gLoop->run();
                                            } catch (const Glib::Error& error)   {
                                                std::cerr << "Got an error: '" << error.what() << "'." << std::endl;
                                            }
                                        }});
        try {

            for (auto &connections : m_dataConnections) {
                m_dbus->call_sync(connections.first, "org.bluez.Device1",
                                  "ConnectProfile", Glib::VariantContainerBase::create_tuple(
                                      Var<ustring>::create("00001101-0000-1000-8000-00805f9b34fb")),
                                  "org.bluez");//, G_MAXINT);

            }
        } catch (const Glib::Error& error)   {
            std::cerr << "Got an error: '" << error.what() << "'." << std::endl;
            throw std::exception();
        }

    } catch (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << error.code() << std::endl;
        throw std::exception();
    }
}

void Glove::setTrainset(const std::string &trainset) {
    for (auto &connections : m_dataConnections) {
        connections.second.collection = connections.second.db["makeomatic"][trainset];
    }
    /*
                     connections.db["makeomatic"].run_command( document{} <<
                                                      "collMod" << connections.second.db["makeomatic"][trainset] <<
                                                       "usePowerOf2Sizes" << true <<
                                                   finalize );
                                                   */
}

void Glove::disconnect() {
    m_bt->join();
    try {
        for (auto &connections : m_dataConnections) {
            m_dbus->call_sync(connections.first, "org.bluez.Device1",
                              "DisconnectProfile", Glib::VariantContainerBase::create_tuple(
                                  Var<ustring>::create("00001101-0000-1000-8000-00805f9b34fb")),
                              "org.bluez");//, G_MAXINT);
        }
    } catch (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << "'." << std::endl;
    }

    m_gLoop->quit();
    m_gThread->join();
}

void Glove::read(const std::string &device, const boost::system::error_code& error, std::size_t length)
{
    try {
        if (!error) {
            auto now = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(now - m_connectionTime);

            auto &connections = m_dataConnections.at(device);
            if (!m_isRecording()) {
                connections.buffer.consume(length);
                boost::asio::async_read_until(*(connections.socket), connections.buffer, '\0',
                                              std::bind(&Glove::read, this, device, _1, _2));
                return;
            }

            if (length) {
                connections.unpackedBuffer.reserve(length - 1);
                cobs_decode(reinterpret_cast<uint8_t*>(connections.buffer.gptr()), length - 1,
                            connections.unpackedBuffer.data());
                connections.buffer.consume(length);
                boost::asio::async_read_until(*(connections.socket), connections.buffer, '\0',
                                              std::bind(&Glove::read, this, device, _1, _2));
                auto data = reinterpret_cast<Packet*>(connections.unpackedBuffer.data());
                std::ostringstream strs; strs << time.count(); std::string str = strs.str();
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
                                      "id" << m_collectors[device] <<
                                      hashStop <<
                                      "step" << m_step <<
                                      "mutation" << hashStart <<
                                      "id" << m_mutation <<
                                      "index" << m_mutationIndex <<
                                      hashStop <<
                                      "data" << hashStart <<
                                      "stamp" << hashStart <<
                                      "microSeconds" << time.count() <<
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
                                      //        "calculated" << hashStart <<
                                      //             "isSameRFIDTag" << (char)data->lastnr <<
                                      //         hashStop <<
                                      //   hashStop <<
                                      hashStop << hashStop << finalize;//, options);
                connections.collection.insert_one(//update
                                                  //                    document{} << "_id" << 1 << finalize,
                                                  std::move(doc));//, options);

            }
        } else {
            std::cerr << error.message() << std::endl;
        }
    } catch  (const std::exception& error)   {
        std::cerr << "Got an error2: '" << error.what() << std::endl;
    }
}

void Glove::on_method_call(const Glib::RefPtr<Gio::DBus::Connection>& /* connection */,
                           const ustring& /* sender */,
                           const ustring& /* object_path */,
                           const ustring& /* interface_name */,
                           const ustring& method_name,
                           const Glib::VariantContainerBase& parameters,
                           const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation) {
    if(method_name == "NewConnection") {
        Var<ustring> gDevice;
        parameters.get_child(gDevice, 0);
        std::string device = gDevice.get();
        const auto fd = invocation->get_message()->get_unix_fd_list()->get(0);
        auto &connections = m_dataConnections.at(device);
        connections.socket.reset(
                    new boost::asio::generic::stream_protocol::socket{connections.ioService});
        connections.socket->assign(boost::asio::generic::stream_protocol(AF_UNIX,3),
                                   fd);
        connections.thread.reset(new std::thread{[&connections,device,this](){
                                                     boost::asio::async_read_until(*connections.socket, connections.buffer, '\0',
                                                     std::bind(&Glove::read, this, device, _1, _2));
                                                     updateConnected();
                                                     connections.ioService.run();
                                                 }});
        invocation->return_value(Glib::VariantContainerBase(nullptr));
    } else if(method_name == "RequestDisconnection") {
        try {
            Var<ustring> gDevice;
            parameters.get_child(gDevice, 0);
            auto &connections = m_dataConnections.at(gDevice.get());
            //connections.ioService.post([&connections,this](){ connections.socket->close(); });
            boost::system::error_code error;
            connections.socket->shutdown(boost::asio::generic::stream_protocol::socket::shutdown_both,
                                         error);
            connections.socket->close();
            connections.socket.reset(nullptr);
            connections.ioService.stop();
            connections.thread->join();
            invocation->return_value(Glib::VariantContainerBase(nullptr));
        } catch  (const std::exception& error)   {
            std::cerr << "Got an error: '" << error.what() << std::endl;
        }
    } else if(method_name == "Release") {
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

