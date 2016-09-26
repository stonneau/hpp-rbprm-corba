#include "hpprbprmplugin.hh"

#include <hpp/core/problem-solver.hh>
#include <hppserverprocess.hh>

/// Plugin to emulate a corbaserver for hpp-core

/// \namespace hpp
/// namespace that encapsulate all the softwares of humanoid-path-planner
namespace hpp {
  namespace rbprm {
    HppRbprmPlugin::HppRbprmPlugin() :
      server_ (NULL)
    {
    }

    HppRbprmPlugin::~HppRbprmPlugin()
    {
      if (server_) {
        server_->wait();
        delete server_;
      }
    }

    /// \fn void HppCorbaserver::init()
    /// Init the plugin
    void HppRbprmPlugin::init()
    {
      hpp::core::ProblemSolverPtr_t ps = hpp::core::ProblemSolver::create ();

      hpp::corbaServer::Server* basic = new hpp::corbaServer::Server (ps, 0, NULL, true);
      hpp::rbprm::Server* rbprm =
          new hpp::rbprm::Server (0, NULL, true, "rbprmChild");
      rbprm->setProblemSolver (ps);
      hpp::affordanceCorba::Server* aff = new hpp::affordanceCorba::Server (0, NULL, true);
      aff->setProblemSolver (ps);

      server_ = new gepetto::gui::CorbaServer(new HppServerProcess(basic, rbprm, aff));
      server_->start();
      server_->waitForInitDone();
    }

    /// \fn QString HppCorbaserverPlugin::name() const
    /// Return the name of the plugin
    /// \return name of the plugin
    QString HppRbprmPlugin::name() const
    {
      return QString ("hpp-rbprm plugin");
    }

    Q_EXPORT_PLUGIN2 (hpprbprmplugin, HppRbprmPlugin)
  } // namespace rbprm
} // namespace hpp
