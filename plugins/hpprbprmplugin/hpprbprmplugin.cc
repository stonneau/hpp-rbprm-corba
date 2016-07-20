#include "hpprbprmplugin.hh"

#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/rbprm/server.hh>
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
      hpp::rbprm::impl::Server rbprmServer =
          new hpp::rbprm::impl::Server (0, NULL, true, "rbprmChild");
      rbprmServer.setProblemSolver (ps);
      server_ = new gepetto::gui::CorbaServer(new HppServerProcess(rbprmServer));
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
