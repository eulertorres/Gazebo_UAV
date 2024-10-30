#include "rw_bat_plugin.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace gazebo{

RwBattPlugin::RwBattPlugin(){
    // Initialize variables
    taxa = 0.0;
    tensao = 0.0;
    ji = 0.0;
    Energia = 0.0;
    Cap_nom = 0.0;
    E_bat = 0.0;
    corrente = 50.0; // Example current
    dt = 1.0; // Time step
}

RwBattPlugin::~RwBattPlugin(){
}

void RwBattPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    // Store the model pointer
    this->model = _parent;

    // Leitura dos parâmetros do SDF
    if (_sdf->HasElement("nome_bateria"))
        nome_bateria = _sdf->Get<std::string>("nome_bateria");
    else
        nome_bateria = "Amprius";

    if (_sdf->HasElement("S"))
        S = _sdf->Get<int>("S");
    else
        S = 14;

    if (_sdf->HasElement("P"))
        P = _sdf->Get<int>("P");
    else
        P = 5;

    if (_sdf->HasElement("nomeLink"))
        link_name_ = _sdf->Get<std::string>("nomeLink");
    else
        link_name_ = "battery_link";

	// Definir o tópico de corrente (pode ser parametrizado via SDF)
	if (_sdf->HasElement("TopicoCorrente"))
		current_pub_topic_ = _sdf->Get<std::string>("TopicoCorrente");
	else
		current_pub_topic_ = "/battery/Total_current";

	// Definir o tópico de tensão (pode ser parametrizado via SDF)
	if (_sdf->HasElement("TopicoTensao"))
		voltage_pub_topic_ = _sdf->Get<std::string>("TopicoTensao");
	else
		voltage_pub_topic_ = "/battery/voltage";

	if (_sdf->HasElement("numRotores"))
		numrotors_ = _sdf->Get<int>("numRotores");
	else
		numrotors_ = 4;

    // Process battery data
    std::tie(Descarga, Taxa, Tensao, Cap_nom, E_bat) = processar_dados(nome_bateria);

    // Initialize voltage
    tensao = encontrarValorMaximo(Tensao);

	std::cout << "Tensão atual da bateria: " << tensao << std::endl;

    // Connect to the world update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RwBattPlugin::OnUpdate, this));
}

void RwBattPlugin::OnUpdate(){

	std::tie(Energia, tensao, taxa, ji) = bateria(corrente, dt, ji, Energia, taxa, tensao, Tensao, Taxa, Descarga, Cap_nom, E_bat);

    // Print current voltage
    std::cout << "Tensão atual da bateria: " << tensao << std::endl;

    // Check voltage threshold
    if (tensao < 3.0){
      std::cout << "A tensão caiu abaixo de 3V. Carregaaaaaaa." << std::endl;
      std::cout << "Autonomia [min]: " << ji / 60 << std::endl;
    }
}

// Função para gerar um vetor discretizado (equivalente a np.linspace)
std::vector<double> RwBattPlugin::linspace(double start, double end, int num){
    std::vector<double> values;
    double step = (end - start) / (num - 1); // Calcula o passo
    for (int i = 0; i < num; ++i) {
        values.push_back(start + i * step); // Adiciona os valores ao vetor
    }
    return values;
}

  // Função de interpolação linear em 1D
std::vector<double> RwBattPlugin::Interpolate1D(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& x_new){
    std::vector<double> y_new;
    for (double x_value : x_new) {
        auto it = std::lower_bound(x.begin(), x.end(), x_value);

        if (it == x.end()) {
            y_new.push_back(y.back());
        } else if (it == x.begin()) {
            y_new.push_back(y.front());
        } else {
            size_t idx = std::distance(x.begin(), it);
            double x0 = x[idx - 1], x1 = x[idx];
            double y0 = y[idx - 1], y1 = y[idx];
            double slope = (y1 - y0) / (x1 - x0);
            y_new.push_back(y0 + slope * (x_value - x0));
        }
    }
    return y_new;
}

// Função para encontrar o valor mínimo em um vetor
double RwBattPlugin::findMin(const std::vector<double>& vec){
    return *std::min_element(vec.begin(), vec.end());
}

// Função para encontrar o valor máximo em um vetor
double RwBattPlugin::findMax(const std::vector<double>& vec){
    return *std::max_element(vec.begin(), vec.end());
}

// Função para gerar uma superfície interpolada
std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double>>> RwBattPlugin::gerar_superficie(
      const std::vector<double>& Descarga, const std::vector<double>& Taxa, const std::vector<std::vector<double>>& Z){
    // Passo 1: Interpolação em x (Descarga) - já foi discretizado
    std::vector<double> xi = Descarga;  // Mantemos xi igual a Descarga (já discretizado)

    // Passo 2: Discretizar yi (Taxa) com 1000 pontos
    std::vector<double> yi = linspace(findMin(Taxa), findMax(Taxa), 1000);

    std::vector<std::vector<double>> zi; // Matriz final após interpolação em y

    // Para cada valor em xi (cada coluna de Z), fazemos a interpolação em relação a yi
    for (size_t i = 0; i < xi.size(); ++i) {
        std::vector<double> coluna_Z;

        // Para cada linha da matriz Z, coletamos a coluna correspondente
        for (const auto& linha : Z) {
            coluna_Z.push_back(linha[i]);
        }

        // Interpolação 1D em y (Taxa) para cada coluna
        std::vector<double> zi_coluna = Interpolate1D(Taxa, coluna_Z, yi);
        zi.push_back(zi_coluna);
    }

    // Retorna a matriz interpolada em ambos os eixos
    return {xi, yi, zi};
}

// Função para encontrar o valor máximo em uma matriz
double RwBattPlugin::encontrarValorMaximo(const std::vector<std::vector<double>>& matriz){
    double maxValor = matriz[0][0]; // Inicia com o primeiro valor da matriz
    for (const auto& linha : matriz) {
        for (const auto& valor : linha) {
            if (valor > maxValor) {
                maxValor = valor; // Atualiza o máximo se encontrar um valor maior
            }
        }
    }
    return maxValor;
}

// Função para encontrar o índice mais próximo
size_t RwBattPlugin::encontrar_indice_mais_proximo(const std::vector<double>& vetor, double valor_procurado, double epsilon){
    auto it = std::lower_bound(vetor.begin(), vetor.end(), valor_procurado);
    if (it != vetor.end() && std::abs(*it - valor_procurado) < epsilon) {
        return std::distance(vetor.begin(), it);
    } else {
        if (it == vetor.begin()) return 0;  // Retorna 0 se for o menor valor
        if (it == vetor.end()) return vetor.size() - 1;  // Retorna o último índice
        size_t idx = std::distance(vetor.begin(), it);
        return (std::abs(vetor[idx - 1] - valor_procurado) < std::abs(vetor[idx] - valor_procurado)) ? idx - 1 : idx;
    }
}

// Função da bateria
std::tuple<double, double, double, double> RwBattPlugin::bateria(double i, double dT, double& ji, double& Con_bat_real, double& C, double& V,
      const std::vector<std::vector<double>>& Tensao, const std::vector<double>& Taxa, const std::vector<double>& Descarga, double Cap_nom, double E_bat){
    // Dados da bateria
    double E_bat_tot = E_bat * S * P;  // Energia total da bateria

    if (ji == 0)  // Primeira iteração
    {
      V = V;
      double Pot_req = i * V * S;
      C = (i / P) / Cap_nom;
      ji += dT;
    }
    else
    {
      // Atualizar o consumo de energia real
      double Energia = Con_bat_real;
      double Pot_req = i * V * S;
      double Con_bat = Pot_req * dT / 3600;  // Converte para Wh
      Con_bat_real = Energia + Con_bat;

      // Verificar se o consumo excede a energia total da bateria
      if (Con_bat_real > E_bat_tot)
      {
        Con_bat_real = E_bat_tot;
        std::cout << "A bateria atingiu o limite de energia." << std::endl;
        return std::make_tuple(Con_bat_real, V - 0.0001, 0, ji);
      }
      else
      {
        // Calcular a descarga e encontrar os índices correspondentes na matriz de tensão
        double Desc = (Con_bat_real / E_bat_tot) * 100;  // Descarga em porcentagem
        size_t coluna = encontrar_indice_mais_proximo(Taxa, C);
        size_t linha = encontrar_indice_mais_proximo(Descarga, Desc);
        std::cout << "Desc: " << Desc << ", C: " << C << std::endl;
        std::cout << "Coluna: " << coluna << ", Linha: " << linha << std::endl;

        // Obter a tensão correspondente
        if (Tensao.empty() || Tensao[0].empty())
        {
          std::cerr << "A matriz Tensao está vazia." << std::endl;
        }
        else
        {
          V = Tensao[linha][coluna]; // Acesso à matriz se não estiver vazia
        }

        C = (i / P) / Cap_nom;
        std::cout << "Tensão: " << V << std::endl;
        ji += dT;
      }
    }
    return std::make_tuple(Con_bat_real, V, C, ji);
}

// Função para processar os dados com base no nome da bateria
std::tuple<std::vector<double>, std::vector<double>, std::vector<std::vector<double>>, double, double> RwBattPlugin::processar_dados(const std::string& nome_bateria){
    std::vector<double> Descarga, Taxa;
    std::vector<std::vector<double>> Tensao;
    double Cap_nom = 0.0;
    double E_bat = 0.0;

    if (nome_bateria == "Welion")
    {
      // Implementação específica para a bateria Welion
      // ...
      // Aqui você pode inserir os dados específicos da bateria Welion
    }
    else if (nome_bateria == "Amprius")
    {
      // Dados da bateria Amprius
      // (Os dados originais foram omitidos por brevidade. Insira aqui os dados reais.)

      // Exemplo de dados
      std::vector<std::pair<double, double>> dados1C = {{0, 4.2}, {100, 3.0}};
      std::vector<double> Descarga1C;
      std::vector<double> tensao1C;

      for (const auto& par : dados1C) {
        Descarga1C.push_back(par.first);
        tensao1C.push_back(par.second);
      }

      auto Descarga_discretizado1C = linspace(findMin(Descarga1C), findMax(Descarga1C), 1000);
      std::vector<double> tensao_interpolada1C = Interpolate1D(Descarga1C, tensao1C, Descarga_discretizado1C);

      // Vetor de taxas C
      std::vector<double> C_values = {1};

      // Matriz Z com as tensões interpoladas
      std::vector<std::vector<double>> Z = { tensao_interpolada1C };

      // Definindo os valores para a bateria "Amprius"
      double m_cel = 0.314;
      E_bat = 353 * m_cel;
      Cap_nom = 30.75;

      // Gerar a superfície
      std::tie(Descarga, Taxa, Tensao) = gerar_superficie(Descarga_discretizado1C, C_values, Z);
    }
    else
    {
      // Implementação para outras baterias
      // ...
    }

    return {Descarga, Taxa, Tensao, Cap_nom, E_bat};
}

} // namespace gazebo

