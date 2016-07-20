#include "hppserverprocess.hh"

#include <hpp/corbaserver/rbprm/server.hh>
#include <hpp/corbaserver/rbprm/common.hh>

namespace hpp {
  namespace rbprm {
    HppServerProcess::HppServerProcess(hpp::corbaServer::Server *server)
      : server_ (server)
    {}

    HppServerProcess::~HppServerProcess()
    {
      delete server_;
    }

    void HppServerProcess::init()
    {
      server_->startCorbaServer ();
      emit done ();
      gepetto::gui::ServerProcess::init();
    }

    void HppServerProcess::processRequest(bool loop)
    {
      server_->processRequest (loop);
      emit done();
    }
  } // namespace rbprm
} // namespace hpp
