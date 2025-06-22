// ==============================================
// INCLUDES E BIBLIOTECAS NECESSÁRIAS
// ==============================================
#include <ESP8266WiFi.h>          // Biblioteca WiFi para ESP8266
#include <PubSubClient.h>         // Cliente MQTT
#include <ArduinoJson.h>          // Manipulação de JSON
#include <ModbusMaster.h>         // Comunicação Modbus
#include <SoftwareSerial.h>       // Serial por software para RS485
#include <WiFiManager.h>          // Gerenciador de configuração WiFi
#include <FS.h>                   // Sistema de arquivos SPIFFS
#include <ESP8266WebServer.h>     // Servidor web HTTP

// ==============================================
// PROTÓTIPOS DE FUNÇÕES (DECLARAÇÕES ANTECIPADAS)
// ==============================================
void saveCustomConfig();                    // Salvar configurações MQTT
void loadCustomConfig();                    // Carregar configurações MQTT
void saveConfigCallback();                  // Callback para salvar configuração
void configModeCallback(WiFiManager *myWiFiManager); // Callback modo configuração
void checkConfigButton();                   // Verificar botão de configuração
void setupWiFiManager();                    // Configurar WiFiManager
void preTransmission();                     // Callback pré-transmissão Modbus
void postTransmission();                    // Callback pós-transmissão Modbus
float readFloatRegister(uint16_t reg);      // Ler registrador float Modbus
void printAlarmDebug(String alarmType, bool alarmState, bool ledState, unsigned long timeToAction, bool pending);
void checkPhaseAlarm();                     // Verificar alarmes de fase
void checkVoltageAlarm();                   // Verificar alarmes de tensão
void checkSystemInitialization();           // Verificar inicialização do sistema
void readAllParameters();                   // Ler todos os parâmetros do medidor
void connectMQTT();                         // Conectar ao broker MQTT
void publishMQTTData();                     // Publicar dados via MQTT
String generateCSS();                       // Gerar CSS para página web
String generateJavaScript();                // Gerar JavaScript para página web
String generateMainPage();                  // Gerar página HTML principal
void handleRoot();                          // Tratar requisição da página principal
void handleDataAPI();                       // Tratar requisição da API de dados
void handleConfigPortal();                  // Tratar abertura do portal de configuração
void handleReset();                         // Tratar reset de configurações
void handleNotFound();                      // Tratar páginas não encontradas
void setupWebServer();                      // Configurar servidor web

// ==============================================
// CONFIGURAÇÕES GLOBAIS - CARREGADAS DO ARQUIVO
// ==============================================
char mqtt_server[40] = "broker.emqx.io";     // Buffer para servidor MQTT
char mqtt_user[20] = "emqx";                 // Buffer para usuário MQTT  
char mqtt_password[20] = "public";           // Buffer para senha MQTT
int mqtt_port = 1883;                        // Porta MQTT

// Flag para indicar se deve salvar configuração
bool shouldSaveConfig = false;

// ==============================================
// DEFINIÇÕES DE PINOS (CONFIRA SUAS CONEXÕES)
// ==============================================
#define RX_PIN 4       // GPIO4 (D2) - Recepção RS485
#define TX_PIN 5       // GPIO5 (D1) - Transmissão RS485
#define MAX485_DE_RE_PIN 12  // GPIO12 (D6) - Controle DE/RE do MAX485
#define FASE_ALARM_PIN 14    // GPIO14 (D5) - LED alarme fase (LOW ativo)
#define VOLT_ALARM_PIN 13    // GPIO13 (D7) - LED alarme tensão (LOW ativo)
#define RS485_STATE_PIN 15   // GPIO15 (D8) - LED status RS485 (LOW = OK)
#define CONFIG_BUTTON_PIN 0  // GPIO0 (D3) - Botão para reset WiFi

// ==============================================
// REGISTRADORES MODBUS (AJUSTE PARA SEU MEDIDOR)
// ==============================================
#define REG_UA_VOLTAGE 0      // Registrador tensão Fase A
#define REG_UB_VOLTAGE 2      // Registrador tensão Fase B
#define REG_UC_VOLTAGE 4      // Registrador tensão Fase C
#define REG_IA_CURRENT 12     // Registrador corrente Fase A
#define REG_IB_CURRENT 14     // Registrador corrente Fase B
#define REG_IC_CURRENT 16     // Registrador corrente Fase C
#define REG_PA_ACTIVE_POWER 18 // Registrador potência ativa Fase A
#define REG_P_TOTAL_ACTIVE_POWER 24 // Registrador potência ativa total
#define REG_FREQ_PHASE_A 60   // Registrador frequência

// ==============================================
// PARÂMETROS DO SISTEMA (AJUSTE CONFORME NECESSÁRIO)
// ==============================================
const float TENSAO_MINIMA = 105.0;           // Tensão mínima permitida (V)
const float TENSAO_MAXIMA = 135.0;           // Tensão máxima permitida (V)
const float LIMITE_FALHA_FASE = 50.0;        // Limite para detectar falha de fase (V)
const unsigned long TEMPO_CONFIRMACAO_ALARME = 3000;    // 3 segundos para ativar alarme
const unsigned long TEMPO_NORMALIZACAO = 15000;         // 15 segundos para normalizar
const unsigned long INTERVALO_LEITURA = 2000;           // Intervalo entre leituras (ms)
const unsigned long INTERVALO_MQTT_RECONNECT = 5000;    // Intervalo para tentar reconectar MQTT (ms)

// ==============================================
// VARIÁVEIS GLOBAIS DO SISTEMA
// ==============================================
// Instâncias dos objetos principais
SoftwareSerial rs485Serial(RX_PIN, TX_PIN);  // Comunicação RS485
ModbusMaster node;                            // Instância Modbus
WiFiClient espClient;                         // Cliente WiFi
PubSubClient mqttClient(espClient);           // Cliente MQTT
WiFiManager wifiManager;                      // Gerenciador WiFi
ESP8266WebServer webServer(80);               // Servidor web na porta 80

// Variáveis de leitura dos sensores
float uaVoltage = 0.0, ubVoltage = 0.0, ucVoltage = 0.0;        // Tensões das fases A, B, C
float iaCurrent = 0.0, ibCurrent = 0.0, icCurrent = 0.0;        // Correntes das fases A, B, C
float totalActivePower = 0.0;                       // Potência ativa total
float frequency = 60.0;                              // Frequência da rede

// Estados do sistema de alarmes
bool faseAlarmActive = false;          // Estado lógico do alarme de fase (para MQTT)
bool voltAlarmActive = false;          // Estado lógico do alarme de tensão (para MQTT)
bool faseAlarmLedActive = false;       // Estado físico do LED de alarme de fase
bool voltAlarmLedActive = false;       // Estado físico do LED de alarme de tensão
bool rs485CommOK = false;              // Status da comunicação RS485
bool sistemaInicializado = false;     // Flag de controle de inicialização
bool wifiConfigurado = false;          // Flag de configuração WiFi
bool mqttConnected = false;            // Status de conexão MQTT

// Controle de tempo do sistema
unsigned long lastReadTime = 0;              // Última leitura dos sensores
unsigned long lastSuccessfulComm = 0;        // Última comunicação RS485 bem-sucedida
unsigned long sistemaIniciadoEm = 0;         // Momento de inicialização do sistema
unsigned long lastConfigButtonCheck = 0;     // Controle do botão de configuração
unsigned long lastMqttAttempt = 0;           // Último tentativa de conexão MQTT

// Controle de alarmes - Fase
unsigned long faseAlarmDetectedTime = 0;     // Tempo de detecção do alarme de fase
unsigned long faseNormalDetectedTime = 0;    // Tempo de normalização da fase
bool faseAlarmPending = false;               // Alarme de fase pendente
bool faseNormalPending = false;              // Normalização de fase pendente

// Controle de alarmes - Tensão
unsigned long voltAlarmDetectedTime = 0;     // Tempo de detecção do alarme de tensão
unsigned long voltNormalDetectedTime = 0;    // Tempo de normalização da tensão
bool voltAlarmPending = false;               // Alarme de tensão pendente
bool voltNormalPending = false;              // Normalização de tensão pendente

// ==============================================
// FUNÇÕES DE CONFIGURAÇÃO PERSONALIZADA
// ==============================================

// Função para salvar configurações MQTT no arquivo JSON
void saveCustomConfig() {
  Serial.println("[CONFIG] Salvando configuração customizada...");
  
  // Criar documento JSON com as configurações atuais
  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_password"] = mqtt_password;
  
  // Tentar abrir arquivo para escrita
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("[CONFIG] Erro ao abrir arquivo de configuração para escrita!");
    return;
  }
  
  // Serializar JSON e salvar no arquivo
  serializeJson(json, configFile);
  configFile.close();
  Serial.println("[CONFIG] Configuração salva com sucesso!");
}

// Função para carregar configurações MQTT do arquivo JSON
void loadCustomConfig() {
  Serial.println("[CONFIG] Carregando configuração customizada...");
  
  // Verificar se o arquivo de configuração existe
  if (SPIFFS.exists("/config.json")) {
    Serial.println("[CONFIG] Arquivo de configuração encontrado");
    
    // Tentar abrir arquivo para leitura
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile) {
      Serial.println("[CONFIG] Lendo arquivo de configuração...");
      
      // Criar documento JSON e fazer parse do arquivo
      DynamicJsonDocument json(1024);
      DeserializationError error = deserializeJson(json, configFile);
      
      // Verificar se o parse foi bem-sucedido
      if (!error) {
        Serial.println("[CONFIG] JSON parseado com sucesso");
        
        // Carregar configurações do JSON com valores padrão como fallback
        if (json.containsKey("mqtt_server")) {
          String temp_server = json["mqtt_server"].as<String>();
          if (temp_server.length() > 0 && temp_server.length() < sizeof(mqtt_server)) {
            temp_server.toCharArray(mqtt_server, sizeof(mqtt_server));
          } else {
            strlcpy(mqtt_server, "broker.emqx.io", sizeof(mqtt_server));
          }
        } else {
          strlcpy(mqtt_server, "broker.emqx.io", sizeof(mqtt_server));
        }
        
        // Carregar porta MQTT
        if (json.containsKey("mqtt_port")) {
          mqtt_port = json["mqtt_port"].as<int>();
          // Validar porta (deve estar entre 1 e 65535)
          if (mqtt_port <= 0 || mqtt_port > 65535) {
            mqtt_port = 1883;
          }
        } else {
          mqtt_port = 1883;
        }
        
        // Carregar usuário MQTT
        if (json.containsKey("mqtt_user")) {
          String temp_user = json["mqtt_user"].as<String>();
          if (temp_user.length() > 0 && temp_user.length() < sizeof(mqtt_user)) {
            temp_user.toCharArray(mqtt_user, sizeof(mqtt_user));
          } else {
            strlcpy(mqtt_user, "emqx", sizeof(mqtt_user));
          }
        } else {
          strlcpy(mqtt_user, "emqx", sizeof(mqtt_user));
        }
        
        // Carregar senha MQTT
        if (json.containsKey("mqtt_password")) {
          String temp_password = json["mqtt_password"].as<String>();
          if (temp_password.length() > 0 && temp_password.length() < sizeof(mqtt_password)) {
            temp_password.toCharArray(mqtt_password, sizeof(mqtt_password));
          } else {
            strlcpy(mqtt_password, "public", sizeof(mqtt_password));
          }
        } else {
          strlcpy(mqtt_password, "public", sizeof(mqtt_password));
        }
        
        // Exibir configurações carregadas (sem mostrar senha)
        Serial.println("[CONFIG] Configuração carregada com sucesso:");
        Serial.println("MQTT Server: " + String(mqtt_server));
        Serial.println("MQTT Port: " + String(mqtt_port));
        Serial.println("MQTT User: " + String(mqtt_user));
        Serial.println("MQTT Password: [OCULTA]");
        
      } else {
        Serial.println("[CONFIG] Erro ao fazer parse do JSON - usando valores padrão");
        // Definir valores padrão em caso de erro de parse
        strlcpy(mqtt_server, "broker.emqx.io", sizeof(mqtt_server));
        mqtt_port = 1883;
        strlcpy(mqtt_user, "emqx", sizeof(mqtt_user));
        strlcpy(mqtt_password, "public", sizeof(mqtt_password));
      }
      
      configFile.close();
    } else {
      Serial.println("[CONFIG] Erro ao abrir arquivo de configuração para leitura!");
      // Usar valores padrão se não conseguir abrir o arquivo
      strlcpy(mqtt_server, "broker.emqx.io", sizeof(mqtt_server));
      mqtt_port = 1883;
      strlcpy(mqtt_user, "emqx", sizeof(mqtt_user));
      strlcpy(mqtt_password, "public", sizeof(mqtt_password));
    }
  } else {
    Serial.println("[CONFIG] Arquivo de configuração não encontrado - usando valores padrão");
    // Definir valores padrão se arquivo não existir
    strlcpy(mqtt_server, "broker.emqx.io", sizeof(mqtt_server));
    mqtt_port = 1883;
    strlcpy(mqtt_user, "emqx", sizeof(mqtt_user));
    strlcpy(mqtt_password, "public", sizeof(mqtt_password));
  }
}

// Callback chamado quando configurações devem ser salvas
void saveConfigCallback() {
  Serial.println("[WIFI] Configuração deve ser salva");
  shouldSaveConfig = true;
}

// ==============================================
// FUNÇÕES DE CONEXÃO MQTT - CORRIGIDAS
// ==============================================

// Função para conectar ao broker MQTT com retry e debug detalhado
void connectMQTT() {
  // Verificar se WiFi está conectado
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MQTT] WiFi não conectado - abortando conexão MQTT");
    mqttConnected = false;
    return;
  }
  
  // Verificar se já está conectado
  if (mqttClient.connected()) {
    mqttConnected = true;
    return;
  }
  
  // Verificar se já passou tempo suficiente desde última tentativa
  if (millis() - lastMqttAttempt < INTERVALO_MQTT_RECONNECT) {
    return;
  }
  
  lastMqttAttempt = millis();
  
  Serial.println("[MQTT] Tentando conectar ao broker MQTT...");
  Serial.println("[MQTT] Servidor: " + String(mqtt_server));
  Serial.println("[MQTT] Porta: " + String(mqtt_port));
  Serial.println("[MQTT] Usuário: " + String(mqtt_user));
  Serial.println("[MQTT] Senha: [OCULTA]");
  
  // Gerar ID único do cliente baseado no MAC address
  String clientId = "MedidorIoT-" + WiFi.macAddress();
  clientId.replace(":", ""); // Remove dois pontos do MAC
  
  Serial.println("[MQTT] Client ID: " + clientId);
  
  // Configurar servidor e porta
  mqttClient.setServer(mqtt_server, mqtt_port);
  
  // Configurar buffer maior para mensagens grandes
  mqttClient.setBufferSize(1024);
  
  // Configurar timeout para conexão
  mqttClient.setSocketTimeout(15);
  
  // Tentar conectar com diferentes métodos baseado nas credenciais
  bool connected = false;
  
  // Método 1: Conectar com usuário e senha
  if (strlen(mqtt_user) > 0 && strlen(mqtt_password) > 0) {
    Serial.println("[MQTT] Tentativa 1: Conectando com usuário e senha...");
    connected = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password);
  }
  
  // Método 2: Se falhou, tentar conectar apenas com client ID (sem credenciais)
  if (!connected) {
    Serial.println("[MQTT] Tentativa 2: Conectando sem credenciais...");
    connected = mqttClient.connect(clientId.c_str());
  }
  
  // Verificar resultado da conexão
  if (connected) {
    mqttConnected = true;
    Serial.println("[MQTT] ✅ Conectado ao broker MQTT com sucesso!");
    Serial.println("[MQTT] Publicando mensagem de inicialização...");
    
    // Publicar mensagem de status inicial
    StaticJsonDocument<256> statusDoc;
    statusDoc["device"] = "MedidorEnergia";
    statusDoc["status"] = "online";
    statusDoc["ip"] = WiFi.localIP().toString();
    statusDoc["mac"] = WiFi.macAddress();
    statusDoc["uptime"] = millis() / 1000;
    statusDoc["timestamp"] = millis();
    
    char statusBuffer[256];
    serializeJson(statusDoc, statusBuffer);
    
    // Publicar em tópicos de status
    mqttClient.publish("medidor/status", statusBuffer, true); // Retained message
    mqttClient.publish("medidor/online", "true", true);       // Retained message
    
    Serial.println("[MQTT] Mensagem de status inicial publicada");
    
  } else {
    mqttConnected = false;
    // Debug detalhado do erro
    int state = mqttClient.state();
    Serial.print("[MQTT] ❌ Falha na conexão. Estado: ");
    
    switch (state) {
      case MQTT_CONNECTION_TIMEOUT:
        Serial.println("TIMEOUT - Verifique a conectividade de rede");
        break;
      case MQTT_CONNECTION_LOST:
        Serial.println("CONNECTION_LOST - Conexão perdida");
        break;
      case MQTT_CONNECT_FAILED:
        Serial.println("CONNECT_FAILED - Falha na conexão");
        break;
      case MQTT_DISCONNECTED:
        Serial.println("DISCONNECTED - Desconectado");
        break;
      case MQTT_CONNECT_BAD_PROTOCOL:
        Serial.println("BAD_PROTOCOL - Protocolo incorreto");
        break;
      case MQTT_CONNECT_BAD_CLIENT_ID:
        Serial.println("BAD_CLIENT_ID - ID do cliente inválido");
        break;
      case MQTT_CONNECT_UNAVAILABLE:
        Serial.println("UNAVAILABLE - Servidor indisponível");
        break;
      case MQTT_CONNECT_BAD_CREDENTIALS:
        Serial.println("BAD_CREDENTIALS - Credenciais inválidas");
        break;
      case MQTT_CONNECT_UNAUTHORIZED:
        Serial.println("UNAUTHORIZED - Não autorizado");
        break;
      default:
        Serial.println("ERRO_DESCONHECIDO: " + String(state));
        break;
    }
    
    // Sugestões de debug
    Serial.println("[MQTT] 💡 Dicas para debug:");
    Serial.println("   - Verifique se o broker está acessível");
    Serial.println("   - Teste conexão: telnet " + String(mqtt_server) + " " + String(mqtt_port));
    Serial.println("   - Verifique credenciais no portal de configuração");
    Serial.println("   - Tente com broker público: test.mosquitto.org:1883");
  }
}

// Função para publicar dados via MQTT com controle de erro
void publishMQTTData() {
  // Verificar se MQTT está conectado
  if (!mqttConnected || !mqttClient.connected()) {
    Serial.println("[MQTT] Não conectado - tentando reconectar...");
    connectMQTT();
    return;
  }

  Serial.println("[MQTT] Publicando dados...");

  // Criar documento JSON com todos os dados
  StaticJsonDocument<1024> doc;
  doc["timestamp"] = millis();
  doc["device_id"] = WiFi.macAddress();
  
  // Dados das três fases
  JsonObject faseA = doc.createNestedObject("faseA");
  faseA["tensao"] = round(uaVoltage * 10) / 10.0;  // Arredondar para 1 casa decimal
  faseA["corrente"] = round(iaCurrent * 1000) / 1000.0;  // Arredondar para 3 casas decimais
  
  JsonObject faseB = doc.createNestedObject("faseB");
  faseB["tensao"] = round(ubVoltage * 10) / 10.0;
  faseB["corrente"] = round(ibCurrent * 1000) / 1000.0;
  
  JsonObject faseC = doc.createNestedObject("faseC");
  faseC["tensao"] = round(ucVoltage * 10) / 10.0;
  faseC["corrente"] = round(icCurrent * 1000) / 1000.0;
  
  // Dados gerais do sistema
  doc["potenciaTotal"] = round(totalActivePower * 10) / 10.0;
  doc["frequencia"] = round(frequency * 100) / 100.0;
  
  // Estados lógicos dos alarmes (atualizados imediatamente)
  doc["alarmeFase"] = faseAlarmActive;
  doc["alarmeTensao"] = voltAlarmActive;
  doc["rs485Status"] = rs485CommOK;
  
  // Estados físicos dos LEDs (com debounce)
  doc["ledFaseAlarm"] = faseAlarmLedActive;
  doc["ledTensaoAlarm"] = voltAlarmLedActive;
  doc["sistemaInicializado"] = sistemaInicializado;
  
  // Informações de conectividade
  JsonObject connectivity = doc.createNestedObject("conectividade");
  connectivity["wifiSSID"] = WiFi.SSID();
  connectivity["wifiRSSI"] = WiFi.RSSI();
  connectivity["wifiIP"] = WiFi.localIP().toString();
  connectivity["mqttConnected"] = mqttConnected;
  connectivity["uptime"] = millis() / 1000;

  // Serializar JSON
  char buffer[1024];
  size_t jsonSize = serializeJson(doc, buffer);
  
  Serial.println("[MQTT] JSON gerado (" + String(jsonSize) + " bytes):");
  Serial.println(buffer);
  
  // Publicar dados principais
  bool published = mqttClient.publish("medidor/dados", buffer);
  
  if (published) {
    Serial.println("[MQTT] ✅ Dados publicados com sucesso no tópico 'medidor/dados'");
    
    // Publicar dados individuais em tópicos separados para facilitar monitoramento
    mqttClient.publish("medidor/tensao/faseA", String(uaVoltage, 1).c_str());
    mqttClient.publish("medidor/tensao/faseB", String(ubVoltage, 1).c_str());
    mqttClient.publish("medidor/tensao/faseC", String(ucVoltage, 1).c_str());
    
    mqttClient.publish("medidor/corrente/faseA", String(iaCurrent, 3).c_str());
    mqttClient.publish("medidor/corrente/faseB", String(ibCurrent, 3).c_str());
    mqttClient.publish("medidor/corrente/faseC", String(icCurrent, 3).c_str());
    
    mqttClient.publish("medidor/potencia", String(totalActivePower, 1).c_str());
    mqttClient.publish("medidor/frequencia", String(frequency, 2).c_str());
    
    mqttClient.publish("medidor/alarmes/fase", faseAlarmActive ? "ATIVO" : "NORMAL");
    mqttClient.publish("medidor/alarmes/tensao", voltAlarmActive ? "ATIVO" : "NORMAL");
    mqttClient.publish("medidor/comunicacao/rs485", rs485CommOK ? "OK" : "ERRO");
    
    Serial.println("[MQTT] Dados individuais publicados em tópicos separados");
    
  } else {
    Serial.println("[MQTT] ❌ Falha ao publicar dados - verificando conexão...");
    mqttConnected = false;
    
    // Tentar reconectar imediatamente
    connectMQTT();
  }
}

// ==============================================
// CALLBACKS DO WIFIMANAGER
// ==============================================

// Callback chamado quando entra no modo de configuração
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("[WIFI] Entrando no modo de configuração");
  Serial.print("[WIFI] IP do portal de configuração: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[WIFI] Nome da rede: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  
  // Piscar LEDs para indicar modo de configuração
  for(int i = 0; i < 10; i++) {
    digitalWrite(FASE_ALARM_PIN, !digitalRead(FASE_ALARM_PIN));
    digitalWrite(VOLT_ALARM_PIN, !digitalRead(VOLT_ALARM_PIN));
    delay(200);
  }
}

// ==============================================
// FUNÇÃO PARA VERIFICAR BOTÃO DE RESET WIFI
// ==============================================

// Verifica se botão foi pressionado por mais de 3 segundos para reset
void checkConfigButton() {
  // Verificar se o botão está pressionado (LOW)
  if (digitalRead(CONFIG_BUTTON_PIN) == LOW) {
    if (lastConfigButtonCheck == 0) {
      lastConfigButtonCheck = millis();
    } else if (millis() - lastConfigButtonCheck > 3000) {
      Serial.println("[WIFI] Botão de reset pressionado - resetando configurações WiFi");
      
      // Feedback visual: piscar LEDs para confirmar reset
      for(int i = 0; i < 5; i++) {
        digitalWrite(FASE_ALARM_PIN, LOW);
        digitalWrite(VOLT_ALARM_PIN, LOW);
        delay(200);
        digitalWrite(FASE_ALARM_PIN, HIGH);
        digitalWrite(VOLT_ALARM_PIN, HIGH);
        delay(200);
      }
      
      // Resetar configurações WiFi e MQTT, depois reiniciar
      wifiManager.resetSettings();
      SPIFFS.remove("/config.json");
      Serial.println("[WIFI] Configurações resetadas - reiniciando...");
      ESP.restart();
    }
  } else {
    lastConfigButtonCheck = 0;  // Reset do contador se botão não estiver pressionado
  }
}

// ==============================================
// CONFIGURAÇÃO DO WIFIMANAGER
// ==============================================

// Configura o WiFiManager com portal personalizado e campos MQTT
void setupWiFiManager() {
  Serial.println("[WIFI] Configurando WiFiManager personalizado...");
  
  // CSS personalizado para o portal de configuração
  String customCSS = 
    "<style>"
    ":root {"
    "  --primary-color: #2c3e50;"
    "  --secondary-color: #34495e;"
    "  --accent-color: #3498db;"
    "  --success-color: #27ae60;"
    "  --text-color: #ecf0f1;"
    "  --bg-color: #1a1a1a;"
    "}"
    "body {"
    "  background: var(--bg-color);"
    "  color: var(--text-color);"
    "  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;"
    "  margin: 0;"
    "  padding: 20px;"
    "}"
    ".container {"
    "  max-width: 500px;"
    "  margin: 0 auto;"
    "  background: var(--primary-color);"
    "  border-radius: 15px;"
    "  box-shadow: 0 10px 30px rgba(0,0,0,0.3);"
    "  overflow: hidden;"
    "}"
    ".header {"
    "  background: linear-gradient(135deg, var(--accent-color), var(--secondary-color));"
    "  padding: 30px 20px;"
    "  text-align: center;"
    "}"
    ".header h1 {"
    "  margin: 0;"
    "  font-size: 2.5em;"
    "  text-shadow: 2px 2px 4px rgba(0,0,0,0.3);"
    "}"
    "input[type=text], input[type=password], select {"
    "  width: 100%;"
    "  padding: 15px;"
    "  margin: 10px 0;"
    "  border: none;"
    "  border-radius: 8px;"
    "  background: var(--secondary-color);"
    "  color: var(--text-color);"
    "  box-sizing: border-box;"
    "  font-size: 16px;"
    "}"
    "input[type=submit] {"
    "  background: var(--success-color);"
    "  color: white;"
    "  padding: 15px 30px;"
    "  border: none;"
    "  border-radius: 8px;"
    "  cursor: pointer;"
    "  width: 100%;"
    "  font-size: 18px;"
    "  margin-top: 20px;"
    "  transition: all 0.3s ease;"
    "}"
    "input[type=submit]:hover {"
    "  background: #229954;"
    "  transform: translateY(-2px);"
    "  box-shadow: 0 5px 15px rgba(0,0,0,0.2);"
    "}"
    "</style>";
  
  // Aplicar CSS customizado ao portal
  wifiManager.setCustomHeadElement(customCSS.c_str());
  
  // Criar campos customizados para configuração MQTT
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Broker", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", String(mqtt_port).c_str(), 6);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqtt_password, 20, "type='password'");
  
  // Adicionar campos personalizados ao WiFiManager
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  
  // Configurar callbacks do WiFiManager
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // Configurar timeouts e parâmetros do portal
  wifiManager.setConfigPortalTimeout(600); // 10 minutos de timeout
  wifiManager.setConnectTimeout(20);       // 20 segundos para conectar
  wifiManager.setDebugOutput(true);        // Habilitar debug
  
  // Definir nome da rede do portal baseado no chip ID
  String apName = "Medidor-" + String(ESP.getChipId(), HEX);
  
  Serial.print("[WIFI] Nome da rede de configuração: ");
  Serial.println(apName);
  
  // Tentar conectar automaticamente, se falhar abrir portal
  if (!wifiManager.autoConnect(apName.c_str(), "12345678")) {
    Serial.println("[WIFI] Falha na conexão - reiniciando...");
    delay(3000);
    ESP.restart();
  }
  
  // Verificar se novas configurações foram salvas
  if (shouldSaveConfig) {
    Serial.println("[WIFI] Salvando novos parâmetros MQTT...");
    
    // Obter valores dos campos customizados e validar
    String newServer = custom_mqtt_server.getValue();
    String newPort = custom_mqtt_port.getValue();
    String newUser = custom_mqtt_user.getValue();
    String newPassword = custom_mqtt_pass.getValue();
    
    // Copiar novos valores para as variáveis globais
    if (newServer.length() > 0 && newServer.length() < sizeof(mqtt_server)) {
      newServer.toCharArray(mqtt_server, sizeof(mqtt_server));
    }
    
    int portValue = newPort.toInt();
    if (portValue > 0 && portValue <= 65535) {
      mqtt_port = portValue;
    }
    
    if (newUser.length() > 0 && newUser.length() < sizeof(mqtt_user)) {
      newUser.toCharArray(mqtt_user, sizeof(mqtt_user));
    }
    
    if (newPassword.length() > 0 && newPassword.length() < sizeof(mqtt_password)) {
      newPassword.toCharArray(mqtt_password, sizeof(mqtt_password));
    }
    
    // Salvar configurações no arquivo
    saveCustomConfig();
    
    // Reiniciar para aplicar novas configurações
    Serial.println("[WIFI] Reiniciando para aplicar novas configurações...");
    delay(2000);
    ESP.restart();
  }
  
  // Conexão WiFi estabelecida com sucesso
  wifiConfigurado = true;
  Serial.println("[WIFI] Conectado com sucesso!");
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WIFI] SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("[WIFI] Força do sinal: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

// ==============================================
// FUNÇÕES DE COMUNICAÇÃO MODBUS
// ==============================================

// Função chamada antes da transmissão Modbus (habilita modo transmissão)
void preTransmission() {
  digitalWrite(MAX485_DE_RE_PIN, HIGH);  // Habilitar transmissão
  delayMicroseconds(50);                 // Aguardar estabilização
}

// Função chamada após a transmissão Modbus (habilita modo recepção)
void postTransmission() {
  delayMicroseconds(50);                 // Aguardar fim da transmissão
  digitalWrite(MAX485_DE_RE_PIN, LOW);   // Habilitar recepção
}

// Lê um registrador float (32 bits) do dispositivo Modbus
float readFloatRegister(uint16_t reg) {
  // Tentar ler 2 registradores (32 bits total)
  uint8_t result = node.readHoldingRegisters(reg, 2);
  delay(50);  // Delay para estabilização da comunicação
  
  if (result == node.ku8MBSuccess) {
    // Combinar os dois registradores em um valor de 32 bits
    uint32_t temp = ((uint32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
    
    // Converter para float usando memcpy para evitar problemas de alinhamento
    float value;
    memcpy(&value, &temp, sizeof(value));
    
    // Atualizar status de comunicação
    lastSuccessfulComm = millis();
    rs485CommOK = true;
    digitalWrite(RS485_STATE_PIN, LOW);  // LED indica comunicação OK
    
    return value;
  } else {
    // Erro na comunicação
    rs485CommOK = false;
    digitalWrite(RS485_STATE_PIN, HIGH); // LED indica erro de comunicação
    return NAN;  // Retorna Not-A-Number em caso de erro
  }
}

// ==============================================
// FUNÇÕES DE DEBUG
// ==============================================

// Imprime informações detalhadas de debug dos alarmes
void printAlarmDebug(String alarmType, bool alarmState, bool ledState, 
                    unsigned long timeToAction, bool pending) {
  Serial.print("[DEBUG] ");
  Serial.print(alarmType);
  Serial.print(" - Estado: ");
  Serial.print(alarmState ? "ATIVO" : "NORMAL");
  Serial.print(" | LED: ");
  Serial.print(ledState ? "ALARM" : "OK");
  
  if (pending) {
    Serial.print(" | Tempo restante: ");
    Serial.print((timeToAction - millis()) / 1000.0, 1);
    Serial.println("s");
  } else {
    Serial.println(" | Estável");
  }
}

// ==============================================
// FUNÇÕES DE CONTROLE DE ALARMES
// ==============================================

// Verifica e controla alarmes de falha de fase
void checkPhaseAlarm() {
  // Verificar se alguma fase está com tensão muito baixa (falha de fase)
  bool faseAusente = (uaVoltage < LIMITE_FALHA_FASE) || 
                    (ubVoltage < LIMITE_FALHA_FASE) || 
                    (ucVoltage < LIMITE_FALHA_FASE);

  // Atualizar estado lógico imediatamente (usado no MQTT)
  faseAlarmActive = faseAusente;

  // Controlar LED físico com sistema de debounce
  if (faseAusente && !faseAlarmLedActive) {
    // Condição de alarme detectada - iniciar contagem
    if (!faseAlarmPending) {
      faseAlarmPending = true;
      faseAlarmDetectedTime = millis() + TEMPO_CONFIRMACAO_ALARME;
      faseNormalPending = false; // Cancelar normalização pendente
      Serial.println("[ALARM] Falha de fase detectada - iniciando contagem de 3s");
    }
    
    // Verificar se tempo de confirmação passou
    if (millis() >= faseAlarmDetectedTime) {
      faseAlarmLedActive = true;
      faseAlarmPending = false;
      digitalWrite(FASE_ALARM_PIN, LOW); // Ativar LED de alarme (LOW ativo)
      Serial.println("[ALARM] LED de falha de fase ATIVADO");
    }
  }
  else if (!faseAusente && faseAlarmLedActive) {
    // Condição normal detectada - iniciar contagem de normalização
    if (!faseNormalPending) {
      faseNormalPending = true;
      faseNormalDetectedTime = millis() + TEMPO_NORMALIZACAO;
      faseAlarmPending = false; // Cancelar alarme pendente
      Serial.println("[NORMAL] Fases normalizadas - iniciando contagem de 15s");
    }
    
    // Verificar se tempo de normalização passou
    if (millis() >= faseNormalDetectedTime) {
      faseAlarmLedActive = false;
      faseNormalPending = false;
      digitalWrite(FASE_ALARM_PIN, HIGH); // Desativar LED de alarme (HIGH normal)
      Serial.println("[NORMAL] LED de falha de fase DESATIVADO");
    }
  }

  // Mostrar debug detalhado quando há mudanças pendentes
  if (faseAlarmPending) {
    printAlarmDebug("FASE", faseAlarmActive, faseAlarmLedActive, faseAlarmDetectedTime, true);
  } else if (faseNormalPending) {
    printAlarmDebug("FASE", faseAlarmActive, faseAlarmLedActive, faseNormalDetectedTime, true);
  }
}

// Verifica e controla alarmes de tensão fora da faixa
void checkVoltageAlarm() {
  // Verificar se alguma fase está com tensão fora da faixa permitida
  bool tensaoForaFaixa = (uaVoltage < TENSAO_MINIMA || uaVoltage > TENSAO_MAXIMA) ||
                        (ubVoltage < TENSAO_MINIMA || ubVoltage > TENSAO_MAXIMA) ||
                        (ucVoltage < TENSAO_MINIMA || ucVoltage > TENSAO_MAXIMA);

  // Atualizar estado lógico imediatamente (usado no MQTT)
  voltAlarmActive = tensaoForaFaixa;

  // Controlar LED físico com sistema de debounce
  if (tensaoForaFaixa && !voltAlarmLedActive) {
    // Condição de alarme detectada - iniciar contagem
    if (!voltAlarmPending) {
      voltAlarmPending = true;
      voltAlarmDetectedTime = millis() + TEMPO_CONFIRMACAO_ALARME;
      voltNormalPending = false; // Cancelar normalização pendente
      Serial.println("[ALARM] Tensão fora da faixa detectada - iniciando contagem de 3s");
    }
    
    // Verificar se tempo de confirmação passou
    if (millis() >= voltAlarmDetectedTime) {
      voltAlarmLedActive = true;
      voltAlarmPending = false;
      digitalWrite(VOLT_ALARM_PIN, LOW); // Ativar LED de alarme (LOW ativo)
      Serial.println("[ALARM] LED de tensão fora da faixa ATIVADO");
    }
  }
  else if (!tensaoForaFaixa && voltAlarmLedActive) {
    // Condição normal detectada - iniciar contagem de normalização
    if (!voltNormalPending) {
      voltNormalPending = true;
      voltNormalDetectedTime = millis() + TEMPO_NORMALIZACAO;
      voltAlarmPending = false; // Cancelar alarme pendente
      Serial.println("[NORMAL] Tensões normalizadas - iniciando contagem de 15s");
    }
    
    // Verificar se tempo de normalização passou
    if (millis() >= voltNormalDetectedTime) {
      voltAlarmLedActive = false;
      voltNormalPending = false;
      digitalWrite(VOLT_ALARM_PIN, HIGH); // Desativar LED de alarme (HIGH normal)
      Serial.println("[NORMAL] LED de tensão fora da faixa DESATIVADO");
    }
  }

  // Mostrar debug detalhado quando há mudanças pendentes
  if (voltAlarmPending) {
    printAlarmDebug("TENSAO", voltAlarmActive, voltAlarmLedActive, voltAlarmDetectedTime, true);
  } else if (voltNormalPending) {
    printAlarmDebug("TENSAO", voltAlarmActive, voltAlarmLedActive, voltNormalDetectedTime, true);
  }
}

// Controla a inicialização do sistema (15 segundos iniciais)
void checkSystemInitialization() {
  // Verificar se passou o tempo de inicialização
  if (!sistemaInicializado && (millis() - sistemaIniciadoEm >= TEMPO_NORMALIZACAO)) {
    sistemaInicializado = true;
    Serial.println("[INIT] Sistema inicializado - verificando condições para normalizar LEDs");
    
    // Se não há alarmes ativos, normalizar os LEDs
    if (!faseAlarmActive) {
      digitalWrite(FASE_ALARM_PIN, HIGH);
      Serial.println("[INIT] LED de fase normalizado");
    }
    
    if (!voltAlarmActive) {
      digitalWrite(VOLT_ALARM_PIN, HIGH);
      Serial.println("[INIT] LED de tensão normalizado");
    }
  }
  
  // Mostrar debug de inicialização a cada 5 segundos
  if (!sistemaInicializado) {
    static unsigned long lastInitDebug = 0;
    if (millis() - lastInitDebug >= 5000) {
      lastInitDebug = millis();
      unsigned long tempoRestante = (TEMPO_NORMALIZACAO - (millis() - sistemaIniciadoEm)) / 1000;
      Serial.print("[INIT] Tempo restante para inicialização: ");
      Serial.print(tempoRestante);
      Serial.println("s");
    }
  }
}

// ==============================================
// FUNÇÕES DE LEITURA E PUBLICAÇÃO
// ==============================================

// Lê todos os parâmetros do medidor via Modbus
void readAllParameters() {
  Serial.println("\n=== Lendo dados do medidor ===");
  
  // Ler tensões das três fases
  uaVoltage = readFloatRegister(REG_UA_VOLTAGE);
  ubVoltage = readFloatRegister(REG_UB_VOLTAGE);
  ucVoltage = readFloatRegister(REG_UC_VOLTAGE);
  
  // Ler correntes das três fases
  iaCurrent = readFloatRegister(REG_IA_CURRENT);
  ibCurrent = readFloatRegister(REG_IB_CURRENT);
  icCurrent = readFloatRegister(REG_IC_CURRENT);
  
  // Ler potência ativa total e frequência
  totalActivePower = readFloatRegister(REG_P_TOTAL_ACTIVE_POWER);
  frequency = readFloatRegister(REG_FREQ_PHASE_A);

  // Exibir leituras no Serial Monitor
  Serial.print("Tensões (V): A="); Serial.print(uaVoltage,1);
  Serial.print(" B="); Serial.print(ubVoltage,1);
  Serial.print(" C="); Serial.println(ucVoltage,1);
  
  Serial.print("Correntes (A): A="); Serial.print(iaCurrent,3);
  Serial.print(" B="); Serial.print(ibCurrent,3);
  Serial.print(" C="); Serial.println(icCurrent,3);
  
  Serial.print("Potência Ativa Total: "); Serial.print(totalActivePower,1); Serial.println(" W");
  Serial.print("Frequência: "); Serial.print(frequency,2); Serial.println(" Hz");
  
  // Mostrar status dos alarmes
  Serial.print("Status Alarmes - Fase: ");
  Serial.print(faseAlarmActive ? "ATIVO" : "NORMAL");
  Serial.print(" (LED: ");
  Serial.print(faseAlarmLedActive ? "ALARM" : "OK");
  Serial.print(") | Tensão: ");
  Serial.print(voltAlarmActive ? "ATIVO" : "NORMAL");
  Serial.print(" (LED: ");
  Serial.print(voltAlarmLedActive ? "ALARM" : "OK");
  Serial.print(") | MQTT: ");
  Serial.println(mqttConnected ? "CONECTADO" : "DESCONECTADO");
}

// ==============================================
// FUNÇÕES DO SERVIDOR WEB
// ==============================================

// Função para gerar CSS personalizado para a página web
String generateCSS() {
  return R"(
<style>
  :root {
    --primary-color: #2c3e50;
    --secondary-color: #34495e;
    --accent-color: #3498db;
    --success-color: #27ae60;
    --warning-color: #f39c12;
    --danger-color: #e74c3c;
    --text-color: #2c3e50;
    --bg-color: #ecf0f1;
    --card-bg: #ffffff;
  }
  
  * {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
  }
  
  body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: var(--bg-color);
    color: var(--text-color);
    line-height: 1.6;
  }
  
  .container {
    max-width: 1200px;
    margin: 0 auto;
    padding: 20px;
  }
  
  .header {
    background: linear-gradient(135deg, var(--primary-color), var(--secondary-color));
    color: white;
    padding: 30px;
    border-radius: 15px;
    text-align: center;
    margin-bottom: 30px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.1);
  }
  
  .header h1 {
    font-size: 2.5em;
    margin-bottom: 10px;
    text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
  }
  
  .header p {
    font-size: 1.2em;
    opacity: 0.9;
  }
  
  .stats-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 20px;
    margin-bottom: 30px;
  }
  
  .card {
    background: var(--card-bg);
    border-radius: 15px;
    padding: 25px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.1);
    transition: transform 0.3s ease;
  }
  
  .card:hover {
    transform: translateY(-5px);
  }
  
  .card h3 {
    color: var(--primary-color);
    margin-bottom: 15px;
    font-size: 1.4em;
    display: flex;
    align-items: center;
    gap: 10px;
  }
  
  .card-icon {
    font-size: 1.5em;
  }
  
  .phase-data {
    display: flex;
    justify-content: space-between;
    margin-bottom: 10px;
    padding: 10px;
    background: #f8f9fa;
    border-radius: 8px;
  }
  
  .phase-label {
    font-weight: bold;
    color: var(--secondary-color);
  }
  
  .phase-value {
    font-weight: bold;
  }
  
  .voltage { color: var(--accent-color); }
  .current { color: var(--success-color); }
  .power { color: var(--warning-color); }
  .frequency { color: var(--primary-color); }
  
  .status-card {
    background: var(--card-bg);
    border-radius: 15px;
    padding: 25px;
    margin-bottom: 20px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.1);
  }
  
  .status-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 15px;
  }
  
  .status-item {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 15px;
    border-radius: 10px;
    background: #f8f9fa;
  }
  
  .status-ok { border-left: 4px solid var(--success-color); }
  .status-warning { border-left: 4px solid var(--warning-color); }
  .status-error { border-left: 4px solid var(--danger-color); }
  
  .status-dot {
    width: 12px;
    height: 12px;
    border-radius: 50%;
    animation: pulse 2s infinite;
  }
  
  .dot-green { background: var(--success-color); }
  .dot-yellow { background: var(--warning-color); }
  .dot-red { background: var(--danger-color); }
  
  @keyframes pulse {
    0% { opacity: 1; }
    50% { opacity: 0.5; }
    100% { opacity: 1; }
  }
  
  .config-section {
    background: var(--card-bg);
    border-radius: 15px;
    padding: 25px;
    margin-bottom: 20px;
    box-shadow: 0 5px 20px rgba(0,0,0,0.1);
  }
  
  .btn {
    display: inline-block;
    padding: 12px 24px;
    border: none;
    border-radius: 8px;
    font-size: 16px;
    font-weight: bold;
    text-decoration: none;
    cursor: pointer;
    transition: all 0.3s ease;
    margin: 5px;
  }
  
  .btn-primary {
    background: var(--accent-color);
    color: white;
  }
  
  .btn-primary:hover {
    background: #2980b9;
    transform: translateY(-2px);
  }
  
  .btn-danger {
    background: var(--danger-color);
    color: white;
  }
  
  .btn-danger:hover {
    background: #c0392b;
    transform: translateY(-2px);
  }
  
  .footer {
    text-align: center;
    margin-top: 40px;
    padding: 20px;
    color: #7f8c8d;
    border-top: 1px solid #ddd;
  }
  
  .refresh-info {
    background: #e8f4f8;
    padding: 15px;
    border-radius: 10px;
    margin-bottom: 20px;
    text-align: center;
    border-left: 4px solid var(--accent-color);
  }
  
  @media (max-width: 768px) {
    .container {
      padding: 10px;
    }
    
    .header h1 {
      font-size: 2em;
    }
    
    .stats-grid {
      grid-template-columns: 1fr;
    }
    
    .status-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
)";
}

// Função para gerar JavaScript para atualização automática
String generateJavaScript() {
  return R"(
<script>
  // Função para atualizar os dados automaticamente
  function updateData() {
    fetch('/api/data')
      .then(response => response.json())
      .then(data => {
        // Atualizar tensões
        document.getElementById('voltage-a').textContent = data.faseA.tensao.toFixed(1) + ' V';
        document.getElementById('voltage-b').textContent = data.faseB.tensao.toFixed(1) + ' V';
        document.getElementById('voltage-c').textContent = data.faseC.tensao.toFixed(1) + ' V';
        
        // Atualizar correntes
        document.getElementById('current-a').textContent = data.faseA.corrente.toFixed(3) + ' A';
        document.getElementById('current-b').textContent = data.faseB.corrente.toFixed(3) + ' A';
        document.getElementById('current-c').textContent = data.faseC.corrente.toFixed(3) + ' A';
        
        // Atualizar potência e frequência
        document.getElementById('power-total').textContent = data.potenciaTotal.toFixed(1) + ' W';
        document.getElementById('frequency').textContent = data.frequencia.toFixed(2) + ' Hz';
        
        // Atualizar status dos alarmes
        updateAlarmStatus('alarm-fase', data.alarmeFase, data.ledFaseAlarm);
        updateAlarmStatus('alarm-tensao', data.alarmeTensao, data.ledTensaoAlarm);
        updateAlarmStatus('comm-rs485', !data.rs485Status, false);
        updateAlarmStatus('mqtt-status', !data.conectividade.mqttConnected, false);
        
        // Atualizar timestamp
        document.getElementById('last-update').textContent = new Date().toLocaleString();
      })
      .catch(error => {
        console.error('Erro ao atualizar dados:', error);
        document.getElementById('last-update').textContent = 'Erro na atualização: ' + new Date().toLocaleString();
      });
  }
  
  // Função para atualizar status visual dos alarmes
  function updateAlarmStatus(elementId, isAlarm, isLedActive) {
    const element = document.getElementById(elementId);
    if (!element) return; // Elemento não existe
    
    const dot = element.querySelector('.status-dot');
    const text = element.querySelector('.status-text');
    
    // Remover classes antigas
    element.classList.remove('status-ok', 'status-warning', 'status-error');
    dot.classList.remove('dot-green', 'dot-yellow', 'dot-red');
    
    if (isAlarm) {
      if (isLedActive) {
        // Alarme ativo e LED ligado
        element.classList.add('status-error');
        dot.classList.add('dot-red');
        text.textContent = 'ALARME ATIVO';
      } else {
        // Alarme detectado mas LED ainda não ativou (debounce)
        element.classList.add('status-warning');
        dot.classList.add('dot-yellow');
        text.textContent = 'ALARME PENDENTE';
      }
    } else {
      // Tudo normal
      element.classList.add('status-ok');
      dot.classList.add('dot-green');
      text.textContent = 'NORMAL';
    }
  }
  
  // Atualizar dados imediatamente e depois a cada 2 segundos
  updateData();
  setInterval(updateData, 2000);
  
  // Função para abrir portal de configuração
  function openConfigPortal() {
    if (confirm('Deseja abrir o portal de configuração? Isso criará uma rede WiFi temporária.')) {
      fetch('/config', { method: 'POST' })
        .then(response => response.text())
        .then(data => {
          alert('Portal de configuração iniciado! Conecte-se à rede WiFi que será criada.');
        })
        .catch(error => {
          alert('Erro ao abrir portal: ' + error);
        });
    }
  }
  
  // Função para resetar configurações
  function resetConfig() {
    if (confirm('ATENÇÃO: Isso irá resetar todas as configurações WiFi e MQTT. Confirma?')) {
      if (confirm('Tem certeza? O dispositivo será reiniciado.')) {
        fetch('/reset', { method: 'POST' })
          .then(response => response.text())
          .then(data => {
            alert('Configurações resetadas! O dispositivo será reiniciado.');
          })
          .catch(error => {
            alert('Erro ao resetar: ' + error);
          });
      }
    }
  }
</script>
)";
}

// Função para gerar a página principal HTML
String generateMainPage() {
  String html = "<!DOCTYPE html><html lang='pt-BR'><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Medidor de Energia - Monitoramento</title>";
  html += generateCSS();
  html += "</head><body>";
  
  html += "<div class='container'>";
  
  // Cabeçalho
  html += "<div class='header'>";
  html += "<h1>⚡ Medidor de Energia IoT</h1>";
  html += "<p>Sistema de Monitoramento Trifásico em Tempo Real</p>";
  html += "</div>";
  
  // Informação de atualização
  html += "<div class='refresh-info'>";
  html += "<strong>📊 Dados atualizados automaticamente a cada 2 segundos</strong><br>";
  html += "Última atualização: <span id='last-update'>Carregando...</span>";
  html += "</div>";
  
  // Grid de dados das fases
  html += "<div class='stats-grid'>";
  
  // Card de Tensões
  html += "<div class='card'>";
  html += "<h3><span class='card-icon'>⚡</span>Tensões por Fase</h3>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase A:</span>";
  html += "<span class='phase-value voltage' id='voltage-a'>-- V</span>";
  html += "</div>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase B:</span>";
  html += "<span class='phase-value voltage' id='voltage-b'>-- V</span>";
  html += "</div>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase C:</span>";
  html += "<span class='phase-value voltage' id='voltage-c'>-- V</span>";
  html += "</div>";
  html += "</div>";
  
  // Card de Correntes
  html += "<div class='card'>";
  html += "<h3><span class='card-icon'>🔌</span>Correntes por Fase</h3>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase A:</span>";
  html += "<span class='phase-value current' id='current-a'>-- A</span>";
  html += "</div>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase B:</span>";
  html += "<span class='phase-value current' id='current-b'>-- A</span>";
  html += "</div>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Fase C:</span>";
  html += "<span class='phase-value current' id='current-c'>-- A</span>";
  html += "</div>";
  html += "</div>";
  
  // Card de Potência e Frequência
  html += "<div class='card'>";
  html += "<h3><span class='card-icon'>📈</span>Potência e Frequência</h3>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Potência Total:</span>";
  html += "<span class='phase-value power' id='power-total'>-- W</span>";
  html += "</div>";
  html += "<div class='phase-data'>";
  html += "<span class='phase-label'>Frequência:</span>";
  html += "<span class='phase-value frequency' id='frequency'>-- Hz</span>";
  html += "</div>";
  html += "</div>";
  
  html += "</div>";
  
  // Status dos Alarmes
  html += "<div class='status-card'>";
  html += "<h3><span class='card-icon'>🚨</span>Status dos Alarmes</h3>";
  html += "<div class='status-grid'>";
  
  html += "<div class='status-item status-ok' id='alarm-fase'>";
  html += "<div class='status-dot dot-green'></div>";
  html += "<div><strong>Alarme de Fase</strong><br><span class='status-text'>NORMAL</span></div>";
  html += "</div>";
  
  html += "<div class='status-item status-ok' id='alarm-tensao'>";
  html += "<div class='status-dot dot-green'></div>";
  html += "<div><strong>Alarme de Tensão</strong><br><span class='status-text'>NORMAL</span></div>";
  html += "</div>";
  
  html += "<div class='status-item status-ok' id='comm-rs485'>";
  html += "<div class='status-dot dot-green'></div>";
  html += "<div><strong>Comunicação RS485</strong><br><span class='status-text'>NORMAL</span></div>";
  html += "</div>";
  
  html += "<div class='status-item status-ok' id='mqtt-status'>";
  html += "<div class='status-dot dot-green'></div>";
  html += "<div><strong>MQTT</strong><br><span class='status-text'>CONECTADO</span></div>";
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  
  // Seção de Configuração
  html += "<div class='config-section'>";
  html += "<h3><span class='card-icon'>⚙️</span>Configurações</h3>";
  html += "<p>Gerencie as configurações do sistema:</p>";
  html += "<div style='margin-top: 15px;'>";
  html += "<button class='btn btn-primary' onclick='openConfigPortal()'>🔧 Abrir Portal de Configuração</button>";
  html += "<button class='btn btn-danger' onclick='resetConfig()'>🔄 Resetar Configurações</button>";
  html += "</div>";
  html += "</div>";
  
  // Informações do Sistema
  html += "<div class='config-section'>";
  html += "<h3><span class='card-icon'>📊</span>Informações do Sistema</h3>";
  html += "<div class='status-grid'>";
  html += "<div class='status-item'><strong>IP:</strong> " + WiFi.localIP().toString() + "</div>";
  html += "<div class='status-item'><strong>SSID:</strong> " + WiFi.SSID() + "</div>";
  html += "<div class='status-item'><strong>RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</div>";
  html += "<div class='status-item'><strong>MAC:</strong> " + WiFi.macAddress() + "</div>";
  html += "<div class='status-item'><strong>Uptime:</strong> " + String(millis()/1000) + " segundos</div>";
  html += "<div class='status-item'><strong>Memória Livre:</strong> " + String(ESP.getFreeHeap()) + " bytes</div>";
  html += "</div>";
  html += "</div>";
  
  // Informações MQTT
  html += "<div class='config-section'>";
  html += "<h3><span class='card-icon'>📡</span>Status MQTT</h3>";
  html += "<div class='status-grid'>";
  html += "<div class='status-item'><strong>Broker:</strong> " + String(mqtt_server) + "</div>";
  html += "<div class='status-item'><strong>Porta:</strong> " + String(mqtt_port) + "</div>";
  html += "<div class='status-item'><strong>Usuário:</strong> " + String(mqtt_user) + "</div>";
  html += "<div class='status-item'><strong>Status:</strong> " + String(mqttConnected ? "CONECTADO" : "DESCONECTADO") + "</div>";
  html += "<div class='status-item'><strong>Tópico Principal:</strong> medidor/dados</div>";
  html += "<div class='status-item'><strong>Última Tentativa:</strong> " + String((millis() - lastMqttAttempt)/1000) + "s atrás</div>";
  html += "</div>";
  html += "</div>";
  
  html += "</div>";
  
  // Rodapé
  html += "<div class='footer'>";
  html += "<p>Medidor de Energia IoT - Desenvolvido por ClaudioSeda</p>";
  html += "<p>Versão 2.0 - " + String(__DATE__) + "</p>";
  html += "</div>";
  
  html += generateJavaScript();
  html += "</body></html>";
  
  return html;
}

// ==============================================
// FUNÇÕES DE TRATAMENTO DAS REQUISIÇÕES WEB
// ==============================================

// Função para lidar com requisições da página principal
void handleRoot() {
  webServer.send(200, "text/html", generateMainPage());
}

// Função para fornecer dados em JSON via API
void handleDataAPI() {
  // Criar JSON com dados atuais
  StaticJsonDocument<1024> doc;
  doc["timestamp"] = millis();
  
  // Dados das fases
  JsonObject faseA = doc.createNestedObject("faseA");
  faseA["tensao"] = uaVoltage;
  faseA["corrente"] = iaCurrent;
  
  JsonObject faseB = doc.createNestedObject("faseB");
  faseB["tensao"] = ubVoltage;
  faseB["corrente"] = ibCurrent;
  
  JsonObject faseC = doc.createNestedObject("faseC");
  faseC["tensao"] = ucVoltage;
  faseC["corrente"] = icCurrent;
  
  // Dados gerais
  doc["potenciaTotal"] = totalActivePower;
  doc["frequencia"] = frequency;
  
  // Status dos alarmes
  doc["alarmeFase"] = faseAlarmActive;
  doc["alarmeTensao"] = voltAlarmActive;
  doc["rs485Status"] = rs485CommOK;
  doc["ledFaseAlarm"] = faseAlarmLedActive;
  doc["ledTensaoAlarm"] = voltAlarmLedActive;
  doc["sistemaInicializado"] = sistemaInicializado;
  
  // Informações de conectividade
  JsonObject connectivity = doc.createNestedObject("conectividade");
  connectivity["wifiSSID"] = WiFi.SSID();
  connectivity["wifiRSSI"] = WiFi.RSSI();
  connectivity["wifiIP"] = WiFi.localIP().toString();
  connectivity["mqttConnected"] = mqttConnected;
  connectivity["uptime"] = millis() / 1000;
  
  // Converter para string e enviar
  String jsonString;
  serializeJson(doc, jsonString);
  webServer.send(200, "application/json", jsonString);
}

// Função para abrir portal de configuração via web
void handleConfigPortal() {
  webServer.send(200, "text/plain", "Abrindo portal de configuração...");
  
  // Configurar e abrir portal
  delay(1000);
  wifiManager.setConfigPortalTimeout(300);
  String apName = "Medidor-" + String(ESP.getChipId(), HEX);
  
  // Adicionar campos MQTT
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Broker", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", String(mqtt_port).c_str(), 6);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqtt_password, 20, "type='password'");
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  
  if (wifiManager.startConfigPortal(apName.c_str(), "12345678")) {
    // Verificar se configurações mudaram
    String newServer = custom_mqtt_server.getValue();
    String newPort = custom_mqtt_port.getValue();
    String newUser = custom_mqtt_user.getValue();
    String newPassword = custom_mqtt_pass.getValue();
    
    bool configChanged = false;
    
    if (newServer != String(mqtt_server)) {
      newServer.toCharArray(mqtt_server, sizeof(mqtt_server));
      configChanged = true;
    }
    
    int portValue = newPort.toInt();
    if (portValue != mqtt_port && portValue > 0 && portValue <= 65535) {
      mqtt_port = portValue;
      configChanged = true;
    }
    
    if (newUser != String(mqtt_user)) {
      newUser.toCharArray(mqtt_user, sizeof(mqtt_user));
      configChanged = true;
    }
    
    if (newPassword != String(mqtt_password)) {
      newPassword.toCharArray(mqtt_password, sizeof(mqtt_password));
      configChanged = true;
    }
    
    if (configChanged) {
      saveCustomConfig();
      ESP.restart();
    }
  }
}

// Função para resetar configurações via web
void handleReset() {
  webServer.send(200, "text/plain", "Resetando configurações...");
  
  delay(1000);
  wifiManager.resetSettings();
  SPIFFS.remove("/config.json");
  ESP.restart();
}

// Função para lidar com páginas não encontradas (404)
void handleNotFound() {
  String message = "Página não encontrada\n\n";
  message += "URI: " + webServer.uri() + "\n";
  message += "Método: ";
  message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
  message += "\n";
  message += "Argumentos: " + String(webServer.args()) + "\n";
  
  // Adicionar informações dos argumentos recebidos
  for (uint8_t i = 0; i < webServer.args(); i++) {
    message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
  }
  
  webServer.send(404, "text/plain", message);
}

// ==============================================
// CONFIGURAÇÃO DO SERVIDOR WEB
// ==============================================

// Função para configurar todas as rotas do servidor web
void setupWebServer() {
  Serial.println("[WEB] Configurando servidor web...");
  
  // Rota principal - página de monitoramento
  webServer.on("/", handleRoot);
  
  // API para dados em JSON - atualização automática
  webServer.on("/api/data", handleDataAPI);
  
  // Rota para abrir portal de configuração via POST
  webServer.on("/config", HTTP_POST, handleConfigPortal);
  
  // Rota para resetar configurações via POST
  webServer.on("/reset", HTTP_POST, handleReset);
  
  // Rota para página de erro 404 - páginas não encontradas
  webServer.onNotFound(handleNotFound);
  
  // Iniciar servidor web na porta 80
  webServer.begin();
  Serial.println("[WEB] Servidor web iniciado na porta 80");
  Serial.println("[WEB] Acesse: http://" + WiFi.localIP().toString());
  Serial.println("[WEB] API JSON: http://" + WiFi.localIP().toString() + "/api/data");
}

// ==============================================
// SETUP (INICIALIZAÇÃO DO SISTEMA)
// ==============================================
void setup() {
  // Inicializar comunicação serial para debug
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("=== SISTEMA DE MONITORAMENTO INICIANDO ===");
  
  // Inicializar sistema de arquivos SPIFFS
  Serial.println("[SPIFFS] Inicializando sistema de arquivos...");
  if (!SPIFFS.begin()) {
    Serial.println("[SPIFFS] Erro ao inicializar SPIFFS!");
    Serial.println("[SPIFFS] Formatando SPIFFS...");
    SPIFFS.format();
    if (!SPIFFS.begin()) {
      Serial.println("[SPIFFS] Erro crítico no SPIFFS!");
    } else {
      Serial.println("[SPIFFS] SPIFFS formatado e inicializado com sucesso!");
    }
  } else {
    Serial.println("[SPIFFS] SPIFFS inicializado com sucesso!");
  }
  
  // Carregar configurações salvas do arquivo
  loadCustomConfig();
  
  // Configurar pinos de entrada e saída
  pinMode(FASE_ALARM_PIN, OUTPUT);         // LED de alarme de fase
  pinMode(VOLT_ALARM_PIN, OUTPUT);         // LED de alarme de tensão
  pinMode(RS485_STATE_PIN, OUTPUT);        // LED de status RS485
  pinMode(MAX485_DE_RE_PIN, OUTPUT);       // Controle DE/RE do MAX485
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP); // Botão de reset WiFi
  
  // Estado inicial: LEDs em alarme por 15 segundos
  digitalWrite(FASE_ALARM_PIN, LOW);    // LED alarme ativo (LOW)
  digitalWrite(VOLT_ALARM_PIN, LOW);    // LED alarme ativo (LOW)
  digitalWrite(RS485_STATE_PIN, HIGH);  // LED erro RS485 ativo (HIGH)
  digitalWrite(MAX485_DE_RE_PIN, LOW);  // MAX485 em modo recepção
  
  // Marcar momento de inicialização
  sistemaIniciadoEm = millis();
  
  Serial.println("[INIT] LEDs iniciados em estado de alarme");
  Serial.println("[INIT] Aguardando 15 segundos antes de normalizar...");

  // Inicializar comunicação RS485 e Modbus
  rs485Serial.begin(9600);
  node.begin(1, rs485Serial);                    // Endereço Modbus = 1
  node.preTransmission(preTransmission);         // Callback antes da transmissão
  node.postTransmission(postTransmission);       // Callback após a transmissão

  // Configurar WiFi usando WiFiManager
  setupWiFiManager();

  // Configurar cliente MQTT com configurações carregadas
  mqttClient.setServer(mqtt_server, mqtt_port);
  
  // Configurar e iniciar servidor web
  setupWebServer();
  
  // Tentar conectar ao MQTT pela primeira vez
  Serial.println("[SETUP] Iniciando primeira conexão MQTT...");
  connectMQTT();
  
  Serial.println("=== SETUP CONCLUÍDO ===");
  Serial.println("Configurações MQTT ativas:");
  Serial.println("Server: " + String(mqtt_server));
  Serial.println("Port: " + String(mqtt_port));
  Serial.println("User: " + String(mqtt_user));
  Serial.println("\nAcesso ao sistema:");
  Serial.println("- Página web: http://" + WiFi.localIP().toString());
  Serial.println("- API JSON: http://" + WiFi.localIP().toString() + "/api/data");
  Serial.println("\nTópicos MQTT:");
  Serial.println("- medidor/dados (JSON completo)");
  Serial.println("- medidor/tensao/faseA, faseB, faseC");
  Serial.println("- medidor/corrente/faseA, faseB, faseC");
  Serial.println("- medidor/potencia");
  Serial.println("- medidor/frequencia");
  Serial.println("- medidor/alarmes/fase, tensao");
  Serial.println("- medidor/status");
  Serial.println("\nInstruções para configuração:");
  Serial.println("- Pressione botão FLASH por 3s+ para resetar WiFi");
  Serial.println("- Use a página web para configurar MQTT");
  Serial.println("- Portal de configuração: rede 'Medidor-XXXX' (senha: 12345678)\n");
}

// ==============================================
// LOOP PRINCIPAL (EXECUÇÃO CONTÍNUA)
// ==============================================
void loop() {
  // Processar requisições do servidor web
  webServer.handleClient();
  
  // Manter conexão MQTT ativa e processar mensagens
  mqttClient.loop();
  
  // Verificar se botão de reset WiFi foi pressionado
  checkConfigButton();
  
  // Monitorar status da conexão WiFi
  if (WiFi.status() != WL_CONNECTED && wifiConfigurado) {
    Serial.println("[WIFI] Conexão perdida - tentando reconectar...");
    mqttConnected = false; // Marcar MQTT como desconectado também
    // Tentar reconectar automaticamente
    if (!wifiManager.autoConnect()) {
      Serial.println("[WIFI] Falha na reconexão - reiniciando...");
      ESP.restart();
    }
  }

  // Tentar conectar/reconectar MQTT se necessário
  if (wifiConfigurado && !mqttConnected) {
    connectMQTT();
  }

  // Verificar se sistema ainda está em inicialização
  checkSystemInitialization();

  // Executar leituras periódicas dos sensores
  if (millis() - lastReadTime >= INTERVALO_LEITURA) {
    lastReadTime = millis();
    
    // Ler todos os parâmetros do medidor
    readAllParameters();
    
    // Só verificar alarmes após inicialização completa
    if (sistemaInicializado) {
      checkPhaseAlarm();    // Verificar alarmes de fase
      checkVoltageAlarm();  // Verificar alarmes de tensão
    }
    
    // Publicar dados se WiFi estiver configurado e MQTT conectado
    if (wifiConfigurado && mqttConnected) {
      publishMQTTData();
    }
  }

  delay(100);  // Pequeno delay para não sobrecarregar o processador
}