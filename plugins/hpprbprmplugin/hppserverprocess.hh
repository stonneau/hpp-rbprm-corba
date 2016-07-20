#ifndef HPP_RBPRM_HPPSERVERPROCESS_HH
#define HPP_RBPRM_HPPSERVERPROCESS_HH

#include <hpp/corbaserver/fwd.hh>

#include <gepetto/gui/omniorb/omniorbthread.hh>

namespace hpp {
  namespace rbprm {
    class HppServerProcess : public gepetto::gui::ServerProcess
    {
      Q_OBJECT

      public:
        HppServerProcess (hpp::corbaServer::Server* server_);

        ~HppServerProcess ();

        public slots:
          void init ();
        void processRequest (bool loop);

      private:
        hpp::corbaServer::Server* server_;
    };
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_HPPSERVERPROCESS_HH
