#ifndef RW_BAT_PLUGIN_H
#define RW_BAT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <vector>
#include <string>
#include <tuple>

namespace gazebo
{
  class RwBattPlugin : public ModelPlugin
  {
    public:
      RwBattPlugin();
      virtual ~RwBattPlugin();
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
      void OnUpdate();

      // Helper functions
      std::vector<double> linspace(double start, double end, int num);
      std::vector<double> Interpolate1D(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& x_new);
      double findMin(const std::vector<double>& vec);
      double findMax(const std::vector<double>& vec);
      std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double>>> gerar_superficie(
          const std::vector<double>& Descarga, const std::vector<double>& Taxa, const std::vector<std::vector<double>>& Z);
      double encontrarValorMaximo(const std::vector<std::vector<double>>& matriz);
      size_t encontrar_indice_mais_proximo(const std::vector<double>& vetor, double valor_procurado, double epsilon = 0.05);
      std::tuple<double, double, double, double> bateria(double i, double dT, double& ji, double& Con_bat_real, double& C, double& V,
          const std::vector<std::vector<double>>& Tensao, const std::vector<double>& Taxa, const std::vector<double>& Descarga, double Cap_nom, double E_bat);
      void imprimir_matriz(const std::vector<std::vector<double>>& matriz);
      std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double>>, double, double> processar_dados(const std::string& nome_bateria);

      // Plugin variables
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;

      // Battery variables
      double S; // Series
      double P; // Parallel
      std::vector<double> Descarga;
      std::vector<double> Taxa;
      std::vector<std::vector<double>> Tensao;
      double Cap_nom;
      double E_bat;

      // Simulation variables
      double taxa;
      double tensao;
      double ji;
      double Energia;
      double corrente;
      double dt;
      std::string nome_bateria;
  };

  GZ_REGISTER_MODEL_PLUGIN(RwBattPlugin)
}

#endif // RW_BATT_PLUGIN_H
