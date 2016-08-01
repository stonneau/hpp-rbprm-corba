#include "hppserverprocess.hh"

#include <hpp/corbaserver/rbprm/server.hh>
#include <hpp/corbaserver/rbprm/common.hh>

namespace hpp {
  namespace rbprm {
    HppServerProcess::HppServerProcess(hpp::corbaServer::Server* basic, 
            hpp::rbprm::Server* rbprm, hpp::affordance::Server* aff)
      : basic_ (basic), rbprm_ (rbprm), aff_ (aff)
    {}

    HppServerProcess::~HppServerProcess()
    {
      delete basic_;
      delete rbprm_;
      delete aff_;
    }

    void HppServerProcess::init()
    {
      basic_->startCorbaServer ();
	  	aff_->startCorbaServer ("hpp", "corbaserver", "affordanceCorba", "affordance");
      rbprm_->startCorbaServer ("hpp", "corbaserver", "rbprm");
      emit done ();
      gepetto::gui::ServerProcess::init();
    }

    void HppServerProcess::processRequest(bool loop)
    {
      basic_->processRequest (loop);
      emit done();
    }
  } // namespace rbprm
} // namespace hpp
