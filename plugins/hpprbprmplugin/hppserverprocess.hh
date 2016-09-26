#ifndef HPP_RBPRM_HPPSERVERPROCESS_HH
#define HPP_RBPRM_HPPSERVERPROCESS_HH

#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/rbprm/server.hh>
#include <hpp/corbaserver/affordance/server.hh>

#include <gepetto/gui/omniorb/omniorbthread.hh>

namespace hpp {
  namespace rbprm {
    class HppServerProcess : public gepetto::gui::ServerProcess
    {
      Q_OBJECT

      public:
        HppServerProcess (hpp::corbaServer::Server* basic,
                hpp::rbprm::Server* rbprm, hpp::affordanceCorba::Server* aff);

        ~HppServerProcess ();

        public slots:
          void init ();
        void processRequest (bool loop);

      private:
        hpp::corbaServer::Server* basic_;
        hpp::rbprm::Server* rbprm_;
        hpp::affordanceCorba::Server* aff_;
    };
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_HPPSERVERPROCESS_HH
