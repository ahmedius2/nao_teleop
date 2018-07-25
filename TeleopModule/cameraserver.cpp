
#include <alvision/alimage.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <qi/log.hpp>
#include <bitset>

#include "cameraserver.h"

using namespace boost::asio::ip;

CameraServer::CameraServer(const std::string &gvmName, const int &resolution,
                           const int &colorSpace, const int &maxfps,
                           const unsigned short &port,
                           boost::shared_ptr<AL::ALBroker> parentBroker)
    : connAcceptor(io_service, tcp::endpoint(tcp::v4(),port)),
      camServerTCPSocket(io_service),
      camProxy(parentBroker),
      maxFPS(maxfps)
{
    connAcceptor.async_accept(camServerTCPSocket,
            boost::bind(&CameraServer::handleAccept, this,
              boost::asio::placeholders::error));

    clientName = camProxy.subscribeCamera(gvmName,AL::kBottomCamera,
                                          resolution, colorSpace, maxFPS);
    io_thread = boost::thread(boost::bind(
                      &boost::asio::io_service::run,
                      boost::ref(io_service)));
}

CameraServer::~CameraServer()
{
    camServerTCPSocket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    camServerTCPSocket.close();
    io_service.stop();
    io_thread.join();
    camProxy.unsubscribe(clientName);
    qiLogDebug("Camera server destructed");
}


void CameraServer::replyCamFrameRequest(const boost::system::error_code& error,
                                        std::size_t size)
{
    if ((boost::asio::error::eof == error) ||
        (boost::asio::error::connection_reset == error)){
        handleDisconnect();
        return;
    }

    boost::array<char,12> datagram;
#ifdef TELEOPMODULE_IS_REMOTE
    const AL::ALValue imageVal = camProxy.getImageRemote(clientName);
    AL::ALImage *image = AL::ALImage::fromALValue(imageVal);
#else
    const AL::ALImage *image = (AL::ALImage*)camProxy.getImageLocal(clientName);
#endif
    for(int i=3; i>=0; --i){
        datagram[3-i]  = (char)((image->getWidth() >> (i*8)) & 0xFF);
        datagram[7-i]  = (char)((image->getHeight() >> (i*8)) & 0xFF);
        datagram[11-i] = (char)((image->getNbLayers() >> (i*8)) & 0xFF);
    }

    try{
        int ret = boost::asio::write(camServerTCPSocket,
                                     boost::asio::buffer(datagram, 12));
        if (ret < 12)
            qiLogDebug("Camera socket error 12");
        ret = boost::asio::write(camServerTCPSocket,
                   boost::asio::buffer(image->getData(),image->getAllocatedSize()));
        if (ret < image->getAllocatedSize())
            qiLogDebug("Camera socket error image data");
    }
    catch(boost::system::system_error e){
        if((boost::asio::error::eof == e.code()) ||
           (boost::asio::error::connection_reset == e.code()))
        {
            camProxy.releaseImage(clientName);
            handleDisconnect();
            return;
        }
    }

    camProxy.releaseImage(clientName);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / maxFPS));

    camServerTCPSocket.async_read_some(boost::asio::buffer(recv_buffer),
            boost::bind(&CameraServer::replyCamFrameRequest, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred));
}

void CameraServer::handleDisconnect(){
    // handle disconnect
    camServerTCPSocket.close();
    // The acceptor will open the socket when a new connection comes
    connAcceptor.async_accept(camServerTCPSocket,
            boost::bind(&CameraServer::handleAccept, this,
              boost::asio::placeholders::error));
    qiLogDebug("Camera client disconnected, waiting for new connection.");
}

void CameraServer::handleAccept(const boost::system::error_code& error)
{
    camServerTCPSocket.async_read_some(boost::asio::buffer(recv_buffer),
            boost::bind(&CameraServer::replyCamFrameRequest, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred));
  }
