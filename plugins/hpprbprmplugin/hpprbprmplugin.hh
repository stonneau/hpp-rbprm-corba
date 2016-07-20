#ifndef HPP_GUI_HPPRBPRMPLUGIN_HH
#define HPP_GUI_HPPRBPRMPLUGIN_HH

#include <gepetto/gui/plugin-interface.hh>
#include <gepetto/gui/omniorb/omniorbthread.hh>


/// namespace that encapsulate all the softwares of humanoid-path-planner
namespace hpp {
  /// namespace that encapsulate the hpp's plugin for gepetto-gui
  namespace rbprm {
    /// HppCorbaserverPlugin allows to launch a corbaserver when gui is launch
    class HppRbprmPlugin : public QObject,
    public gepetto::gui::PluginInterface
    {
      Q_OBJECT
      Q_INTERFACES (gepetto::gui::PluginInterface)

      public:
        explicit HppRbprmPlugin ();

        virtual ~HppRbprmPlugin ();

signals:

        public slots:

          // PluginInterface interface
      public:
      /// Initializes the plugin
          void init();
      /// Returns the plugin's name
      /// \return name of the plugin
          QString name() const;

      private:
          gepetto::gui::CorbaServer* server_;
    };
  } // namespace gui
} // namespace hpp

#endif // HPP_GUI_HPPRBPRMPLUGIN_HH
