#include <cctype>
#include <clocale>
#include <functional>
#include <unordered_map>
#include <thread>
#include <boost/asio.hpp>
#include <glibmm.h>
#include <giomm.h>
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

Glove::Glove(const std::string &leftMAC, const std::string &rightMAC,
             const std::function<bool()> &isRecording) :
    m_isRecording(isRecording)
{
    for (auto MAC : {leftMAC, rightMAC}) {
        auto path = MAC;
        for (char &c : path) {
            c = std::toupper(c);
            if (c == ':')
                c = '_';
        }
        path.insert(0, "/org/bluez/hci0/dev_");
        m_dataConnections[path].MAC = MAC;
        m_dataConnections[path].stream.reset(
                    new boost::asio::posix::stream_descriptor{m_ioService});
    }
}

//static Glib::RefPtr<Gio::DBus::NodeInfo> introspection_data;

void Glove::connectDevice()
{
    try  {
        Gio::init();
        Glib::init();

        Gio::DBus::own_name(Gio::DBus::BUS_TYPE_SYSTEM, "org.bluez.Profile1",
                            sigc::mem_fun(*this, &Glove::on_bus_acquired));

        m_dbus.reset(new std::thread{[](){
                                         try {
                                             auto loop = Glib::MainLoop::create();
                                             loop->run();
                                         } catch (const Glib::Error& error)   {
                                             std::cerr << "Got an error: '" << error.what() << "'." << std::endl;
                                         }
                                     }});
    } catch (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << error.code() << std::endl;
    }
}

void Glove::read(const std::string &device, const boost::system::error_code& /*error*/, std::size_t length)
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

    if (!m_isRecording()) {
        m_connections[deviceInfo]->readAll();
        return;
    }
*/
    auto &connection = m_dataConnections.at(device);
    if (length) {
        connection.unpackedBuffer.reserve(length - 1);//buffer.gptr()
        cobs_decode(reinterpret_cast<uint8_t*>(connection.buffer.gptr()), length - 1,
                    connection.unpackedBuffer.data());
        connection.buffer.consume(length);
        auto data = reinterpret_cast<Packet*>(connection.unpackedBuffer.data());
        std::cerr << data->ex << " ";
    }
    boost::asio::async_read_until(*(connection.stream), connection.buffer, '\0',
                                  std::bind(&Glove::read, this, device, _1, _2));
    //mutation

    //emit, continue
}
void Glove::on_bus_acquired(const Glib::RefPtr<Gio::DBus::Connection>& dbus, const Glib::ustring&) {
    try {
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
        dbus->register_object("/org/bluez/glove",
                              introspectionData->lookup_interface(),
                              m_interfaceVtable);
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
        Var<ustring>::create_object_path(path, "/org/bluez/glove");
        Glib::VariantContainerBase parameters = Glib::VariantContainerBase::create_tuple(
                    std::vector<Glib::VariantBase>({ path,
                                                     Var<ustring>::create("1101"),
                                                     options }));
        profileManager->call_sync("RegisterProfile", parameters);
        m_bt.reset(new std::thread{[dbus, this](){
                                       try {
                                           for (auto &connection : m_dataConnections) {
                                               dbus->call_sync(connection.first, "org.bluez.Device1",
                                               "ConnectProfile", Glib::VariantContainerBase::create_tuple(
                                               Var<ustring>::create("00001101-0000-1000-8000-00805f9b34fb")),
                                               "org.bluez", G_MAXINT);
                                           }
                                       } catch (const Glib::Error& error)   {
                                           std::cerr << "Got an error: '" << error.what() << "'." << std::endl;
                                       }
                                   }});
    } catch (const Glib::Error& error)   {
        std::cerr << "Got an error: '" << error.what() << error.code() << std::endl;
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
        Var<ustring> device;
        parameters.get_child(device, 0);
        const auto fd = invocation->get_message()->get_unix_fd_list()->get(0);
        auto &connection = m_dataConnections.at(device.get());
        connection.stream->assign(::dup(fd));
        boost::asio::async_read_until(*connection.stream, connection.buffer, '\0',
                                      std::bind(&Glove::read, this, device.get(), _1, _2));
        connection.thread.reset(new std::thread{[this](){ m_ioService.run(); }});
        invocation->return_value(Glib::VariantContainerBase());
    }
    Gio::DBus::Error error(Gio::DBus::Error::UNKNOWN_METHOD,
                           "Method does not exist.");
    invocation->return_error(error);
}
/*
void Glove::onConnectionChanged() {
    Connected state = Connected::none;
    if (m_connections.begin().value()->state() == QBluetoothSocket::ConnectedState)
        state = Connected::left;
    if ((++m_connections.begin()).value()->state() == QBluetoothSocket::ConnectedState)
        state = static_cast<Connected>(state | Connected::right);
}
*/
