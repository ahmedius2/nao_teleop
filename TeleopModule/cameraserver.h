#ifndef CAMERASERVER_H
#define CAMERASERVER_H


#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/altexttospeechproxy.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/system/error_code.hpp>
#include <alcommon/albroker.h>
#include <string>


class CameraServer
{
public:
    CameraServer(const std::string &gvmName, const int &resolution,
                 const int &colorSpace, const int &maxfps,
                 const unsigned short &port,
                 boost::shared_ptr<AL::ALBroker> parentBroker);

    virtual ~CameraServer();

private:
    void replyCamFrameRequest(const boost::system::error_code& error,
                               std::size_t);
    void handleAccept(const boost::system::error_code& error);

    boost::thread io_thread;
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::acceptor connAcceptor;
    boost::asio::ip::tcp::socket camServerTCPSocket;
    boost::array<char, 1> recv_buffer;
    AL::ALVideoDeviceProxy camProxy;
    std::string clientName;

    AL::ALTextToSpeechProxy ttsProxy; // for debugging
};

#endif // CAMERASERVER_H
