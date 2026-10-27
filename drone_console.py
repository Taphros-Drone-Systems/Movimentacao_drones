#!/usr/bin/env python3
"""
drone_console.py - Console interativo para controle de drone em tempo real

Comandos disponíveis:
  help, ?                    - Mostrar ajuda
  connect                    - Conectar ao drone
  disconnect                 - Desconectar
  status, st                 - Mostrar status do drone
  arm                        - Armar motores
  disarm                     - Desarmar motores
  takeoff <altura>           - Decolar para altura (metros)
  land                       - Pousar
  mode <modo>                - Mudar modo (GUIDED, AUTO, RTL, LAND, LOITER)
  goto <lat> <lon> <alt>     - Ir para coordenada
  gotorelat <n> <e> <u>      - Mover relativo (norte, leste, cima em metros)
  velocity <vx> <vy> <vz>    - Controle por velocidade (m/s)
  velocity_body <vx> <vy> <vz> - Velocidade no referencial do corpo
  yaw <angulo>               - Girar para ângulo (graus)
  yaw_rel <angulo>           - Girar ângulo relativo
  yaw_rate <taxa> [duracao]  - Rotação contínua (graus/segundo)
  stop                       - Parar navegação (loiter)
  rtl                        - Return to Launch
  param_get <nome>           - Ler parâmetro
  param_set <nome> <valor>   - Escrever parâmetro
  param_list [padrao]        - Listar parâmetros (opcional filtrar)
  mission_show               - Mostrar missão local
  mission_clear              - Limpar missão local
  mission_add_wp <lat> <lon> <alt> - Adicionar waypoint à missão
  mission_add_loiter <seg> [raio] - Adicionar loiter à missão
  mission_upload             - Enviar missão para o drone
  mission_run                - Executar missão (muda para AUTO e arma)
  reboot                     - Reiniciar autopilot
  autostat on/off            - Ligar/desligar status automático
  exit, quit                 - Sair

Exemplos:
  > connect
  > arm
  > takeoff 10
  > goto -35.363261 149.165230 30
  > land
  > disconnect
  > exit
"""

import sys
import time
import threading
import readline  # Para histórico de comandos
from vehicle import Vehicle
from commands import CommandResult
from navigation import NavResult
from parameters import ParamResult
from mission import MissionResult


class DroneConsole:
    """Console interativo para controle do drone"""
    
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.drone = None
        self.running = True
        self.auto_status = False
        self.status_thread = None
        self.status_lock = threading.Lock()
        self.last_status_line = ""
        
    def start(self):
        """Inicia o console"""
        print("\n" + "=" * 70)
        print("  DRONE CONSOLE - Controle em Tempo Real")
        print("=" * 70)
        print(f"\n  Conexão padrão: {self.connection_string}")
        print("  Digite 'connect' para conectar ao drone")
        print("  Digite 'help' para lista de comandos")
        print("=" * 70)
        
        while self.running:
            try:
                # Usar input normal - o status automático não vai interferir
                cmd_input = input("\n> ").strip()
                if not cmd_input:
                    continue
                
                self.execute_command(cmd_input)
                
            except KeyboardInterrupt:
                print("\n\n  Use 'exit' ou 'quit' para sair")
            except EOFError:
                break
            except Exception as e:
                print(f"  Erro: {e}")
        
        self.cleanup()
    
    def _safe_print(self, message, newline=True):
        """Imprime mensagem sem interferir no input atual"""
        # Esta versão simples apenas imprime - como o input já foi processado,
        # não há problema em imprimir diretamente
        if newline:
            print(message)
        else:
            print(message, end="", flush=True)
    
    def execute_command(self, cmd_input):
        """Executa um comando"""
        parts = cmd_input.split()
        cmd = parts[0].lower()
        args = parts[1:]
        
        # Comandos de ajuda
        if cmd in ['help', '?', 'h']:
            self.show_help()
        
        # Comandos de conexão
        elif cmd == 'connect':
            self.cmd_connect()
        elif cmd == 'disconnect':
            self.cmd_disconnect()
        
        # Comandos de status
        elif cmd in ['status', 'st']:
            self.cmd_status()
        elif cmd == 'autostat':
            self.cmd_autostat(args)
        
        # Comandos de controle
        elif cmd == 'arm':
            self.cmd_arm()
        elif cmd == 'disarm':
            self.cmd_disarm()
        elif cmd == 'takeoff':
            self.cmd_takeoff(args)
        elif cmd == 'land':
            self.cmd_land()
        elif cmd == 'mode':
            self.cmd_mode(args)
        elif cmd == 'rtl':
            self.cmd_rtl()
        elif cmd == 'stop':
            self.cmd_stop()
        
        # Comandos de navegação
        elif cmd == 'goto':
            self.cmd_goto(args)
        elif cmd == 'gotorelat':
            self.cmd_goto_relat(args)
        elif cmd == 'velocity':
            self.cmd_velocity(args)
        elif cmd == 'velocity_body':
            self.cmd_velocity_body(args)
        elif cmd == 'yaw':
            self.cmd_yaw(args)
        elif cmd == 'yaw_rel':
            self.cmd_yaw_rel(args)
        elif cmd == 'yaw_rate':
            self.cmd_yaw_rate(args)
        
        # Comandos de parâmetros
        elif cmd == 'param_get':
            self.cmd_param_get(args)
        elif cmd == 'param_set':
            self.cmd_param_set(args)
        elif cmd == 'param_list':
            self.cmd_param_list(args)
        
        # Comandos de missão
        elif cmd == 'mission_show':
            self.cmd_mission_show()
        elif cmd == 'mission_clear':
            self.cmd_mission_clear()
        elif cmd == 'mission_add_wp':
            self.cmd_mission_add_wp(args)
        elif cmd == 'mission_add_loiter':
            self.cmd_mission_add_loiter(args)
        elif cmd == 'mission_upload':
            self.cmd_mission_upload()
        elif cmd == 'mission_run':
            self.cmd_mission_run()
        
        # Comandos do sistema
        elif cmd == 'reboot':
            self.cmd_reboot()
        elif cmd in ['exit', 'quit', 'q']:
            self.cmd_exit()
        
        else:
            print(f"  Comando desconhecido: {cmd}")
            print("  Digite 'help' para lista de comandos")
    
    def show_help(self):
        """Mostra ajuda"""
        print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                          COMANDOS DISPONÍVEIS                                 ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║ CONEXÃO                                                                       ║
║   connect                    Conectar ao drone                                ║
║   disconnect                 Desconectar                                      ║
║                                                                               ║
║ STATUS                                                                        ║
║   status, st                 Mostrar status do drone                          ║
║   autostat on/off            Ligar/desligar status automático                 ║
║                                                                               ║
║ CONTROLE BÁSICO                                                               ║
║   arm                        Armar motores                                    ║
║   disarm                     Desarmar motores                                 ║
║   takeoff <altura>           Decolar (metros)                                 ║
║   land                       Pousar                                           ║
║   mode <modo>                Mudar modo (GUIDED, AUTO, RTL, LAND, LOITER)     ║
║   rtl                        Return to Launch                                 ║
║   stop                       Parar navegação (loiter)                         ║
║   reboot                     Reiniciar autopilot                              ║
║                                                                               ║
║ NAVEGAÇÃO                                                                     ║
║   goto <lat> <lon> <alt>     Ir para coordenada                               ║
║   gotorelat <n> <e> <u>      Mover relativo (norte, leste, cima em metros)    ║
║   velocity <vx> <vy> <vz>    Velocidade (norte, leste, baixo m/s)             ║
║   velocity_body <vx> <vy> <vz> Velocidade (frente, direita, baixo m/s)        ║
║   yaw <angulo>               Girar para ângulo absoluto (graus)               ║
║   yaw_rel <angulo>           Girar ângulo relativo (graus)                    ║
║   yaw_rate <taxa> [duracao]  Rotação contínua (graus/segundo)                 ║
║                                                                               ║
║ PARÂMETROS                                                                    ║
║   param_get <nome>           Ler parâmetro                                    ║
║   param_set <nome> <valor>   Escrever parâmetro                               ║
║   param_list [padrao]        Listar parâmetros (opcional filtrar)             ║
║                                                                               ║
║ MISSÃO                                                                        ║
║   mission_show               Mostrar missão local                             ║
║   mission_clear              Limpar missão local                              ║
║   mission_add_wp <lat> <lon> <alt> Adicionar waypoint                         ║
║   mission_add_loiter <seg> [raio] Adicionar loiter                            ║
║   mission_upload             Enviar missão para o drone                       ║
║   mission_run                Executar missão (AUTO + ARM)                     ║
║                                                                               ║
║ SISTEMA                                                                       ║
║   help, ?                    Mostrar esta ajuda                               ║
║   exit, quit                 Sair                                             ║
╚═══════════════════════════════════════════════════════════════════════════════╝
        """)
    
    def cmd_autostat(self, args):
        """Liga/desliga status automático"""
        if len(args) < 1:
            print("  Uso: autostat on/off")
            print(f"  Status atual: {'ON' if self.auto_status else 'OFF'}")
            return
        
        if args[0].lower() == 'on':
            self.start_auto_status()
        elif args[0].lower() == 'off':
            self.stop_auto_status()
        else:
            print(f"  Opção inválida: {args[0]}. Use 'on' ou 'off'")
    
    def cmd_connect(self):
        """Conecta ao drone"""
        if self.drone and self.drone.is_connected:
            print("  Já conectado!")
            return
        
        print(f"  Conectando a {self.connection_string}...")
        self.drone = Vehicle(self.connection_string)
        
        if not self.drone.connect():
            print("  ❌ Falha na conexão!")
            self.drone = None
            return
        
        print("  ✅ Conectado!")
        print("  Aguardando telemetria...")
        
        if self.drone.wait_ready(timeout=10):
            print("  ✅ Telemetria recebida!")
            self.cmd_status()
        else:
            print("  ⚠️ Timeout aguardando telemetria")
    
    def cmd_disconnect(self):
        """Desconecta do drone"""
        if self.drone:
            self.stop_auto_status()
            self.drone.disconnect()
            self.drone = None
            print("  ✅ Desconectado!")
        else:
            print("  Não conectado")
    
    def cmd_status(self):
        """Mostra status do drone"""
        if not self._check_connected():
            return
        
        try:
            status = self.drone.get_status_dict()
            link_stats = self.drone.get_link_stats()
            
            print("\n" + "=" * 60)
            print("  STATUS DO DRONE")
            print("=" * 60)
            
            # Sistema
            print(f"\n  📡 SISTEMA")
            print(f"    Conexão: {'✅' if status['connected'] else '❌'}")
            print(f"    Modo: {status['mode']}")
            print(f"    Armado: {'✅ SIM' if status['armed'] else '❌ NÃO'}")
            
            # Posição
            print(f"\n  📍 POSIÇÃO")
            if status['position']['latitude']:
                print(f"    Latitude: {status['position']['latitude']:.6f}")
                print(f"    Longitude: {status['position']['longitude']:.6f}")
                print(f"    Altitude: {status['position']['altitude_rel']:.1f}m")
                print(f"    Heading: {status['position']['heading']:.1f}°")
            else:
                print("    Aguardando dados...")
            
            # Atitude
            print(f"\n  🎯 ATITUDE")
            print(f"    Roll: {status['attitude']['roll']:.1f}°")
            print(f"    Pitch: {status['attitude']['pitch']:.1f}°")
            print(f"    Yaw: {status['attitude']['yaw']:.1f}°")
            
            # Bateria
            print(f"\n  🔋 BATERIA")
            if status['battery']['voltage']:
                print(f"    Voltagem: {status['battery']['voltage']:.1f}V")
                print(f"    Corrente: {status['battery']['current']:.1f}A")
                print(f"    Carga: {status['battery']['remaining']}%")
            else:
                print("    Aguardando dados...")
            
            # GPS
            print(f"\n  🛰️ GPS")
            if status['gps']['fix_type'] is not None:
                fix_names = {0: "Sem GPS", 1: "Sem Fix", 2: "2D", 3: "3D", 
                            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
                print(f"    Fix: {fix_names.get(status['gps']['fix_type'], 'Desconhecido')}")
                print(f"    Satélites: {status['gps']['satellites']}")
            else:
                print("    Aguardando dados...")
            
            # Link
            print(f"\n  📶 LINK")
            print(f"    Estado: {link_stats['state']}")
            print(f"    Packet Loss: {link_stats['packet_loss_rate']}")
            print(f"    Failsafe: {'⚠️ ATIVO' if status['link']['failsafe_state'] != 'NORMAL' else '✅ NORMAL'}")
            
            # Saúde
            print(f"\n  ❤️ SAÚDE")
            print(f"    Problemas ativos: {status['health']['active_issues']}")
            
            print("=" * 60)
            
        except Exception as e:
            print(f"  Erro ao obter status: {e}")
    
    def cmd_arm(self):
        """Armar motores"""
        if not self._check_connected():
            return
        
        print("  Armando motores...")
        result = self.drone.arm(timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print("  ✅ Motores armados!")
        elif result == CommandResult.TIMEOUT:
            print("  ❌ Timeout - drone não respondeu")
        elif result == CommandResult.DENIED:
            print("  ❌ Negado - verifique:")
            print("     - GPS fix adequado?")
            print("     - Safety switch pressionado?")
            print("     - Pré-arm checks passaram?")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_disarm(self):
        """Desarmar motores"""
        if not self._check_connected():
            return
        
        print("  Desarmando motores...")
        result = self.drone.disarm(timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print("  ✅ Motores desarmados!")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_takeoff(self, args):
        """Decolar"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: takeoff <altura>")
            print("  Exemplo: takeoff 10")
            return
        
        try:
            altitude = float(args[0])
        except ValueError:
            print(f"  ❌ Altura inválida: {args[0]}")
            return
        
        # Verificar modo
        if self.drone.state.flight_mode() != "GUIDED":
            print("  Mudando para modo GUIDED...")
            self.drone.set_mode("GUIDED")
            time.sleep(0.5)
        
        # Verificar se está armado
        if not self.drone.state.is_armed():
            print("  Armando primeiro...")
            result = self.drone.arm()
            if result != CommandResult.SUCCESS:
                print(f"  ❌ Não foi possível armar: {result}")
                return
        
        print(f"  Decolando para {altitude}m...")
        result = self.drone.takeoff_guided(altitude, timeout=20.0)
        
        if result == NavResult.SUCCESS:
            print(f"  ✅ Decolagem concluída!")
        else:
            print(f"  ❌ Falha na decolagem: {result}")
    
    def cmd_land(self):
        """Pousar"""
        if not self._check_connected():
            return
        
        print("  Iniciando pouso...")
        self.drone.land()
        print("  ✅ Comando de pouso enviado")
    
    def cmd_mode(self, args):
        """Mudar modo"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: mode <modo>")
            print("  Modos disponíveis: GUIDED, AUTO, RTL, LAND, LOITER, STABILIZE")
            return
        
        mode = args[0].upper()
        print(f"  Mudando para modo {mode}...")
        result = self.drone.set_mode(mode, timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print(f"  ✅ Modo alterado para {mode}")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_rtl(self):
        """Return to Launch"""
        if not self._check_connected():
            return
        
        print("  Ativando Return to Launch...")
        result = self.drone.set_mode("RTL", timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print("  ✅ RTL ativado!")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_stop(self):
        """Parar navegação"""
        if not self._check_connected():
            return
        
        self.drone.stop_navigation()
        print("  ✅ Navegação parada (loiter)")
    
    def cmd_goto(self, args):
        """Ir para coordenada"""
        if not self._check_connected():
            return
        
        if len(args) < 3:
            print("  Uso: goto <latitude> <longitude> <altitude> [yaw]")
            print("  Exemplo: goto -35.363261 149.165230 30")
            return
        
        try:
            lat = float(args[0])
            lon = float(args[1])
            alt = float(args[2])
            yaw = float(args[3]) if len(args) > 3 else None
        except ValueError:
            print("  ❌ Coordenadas inválidas")
            return
        
        # Verificar modo
        if self.drone.state.flight_mode() != "GUIDED":
            print("  Mudando para modo GUIDED...")
            self.drone.set_mode("GUIDED")
            time.sleep(0.5)
        
        # Verificar se está armado
        if not self.drone.state.is_armed():
            print("  ❌ Drone não está armado. Use 'arm' primeiro")
            return
        
        print(f"  Navegando para: lat={lat}, lon={lon}, alt={alt}m")
        result = self.drone.goto(lat, lon, alt, yaw=yaw, timeout=30.0)
        
        if result == NavResult.SUCCESS:
            print("  ✅ Chegou ao destino!")
        elif result == NavResult.TIMEOUT:
            print("  ⚠️ Timeout - não chegou a tempo")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_goto_relat(self, args):
        """Mover relativo"""
        if not self._check_connected():
            return
        
        if len(args) < 3:
            print("  Uso: gotorelat <norte> <leste> <cima>")
            print("  Exemplo: gotorelat 10 5 0 (10m norte, 5m leste)")
            return
        
        try:
            north = float(args[0])
            east = float(args[1])
            up = float(args[2])
        except ValueError:
            print("  ❌ Valores inválidos")
            return
        
        print(f"  Movendo: norte={north}m, leste={east}m, cima={up}m")
        result = self.drone.goto_relative(north, east, up, timeout=30.0)
        
        if result == NavResult.SUCCESS:
            print("  ✅ Movimento concluído!")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_velocity(self, args):
        """Controle por velocidade"""
        if not self._check_connected():
            return
        
        if len(args) < 3:
            print("  Uso: velocity <vx> <vy> <vz> [duracao]")
            print("  vx: norte (+), vy: leste (+), vz: baixo (+)")
            print("  Exemplo: velocity 5 0 0 3 (5m/s norte por 3s)")
            return
        
        try:
            vx = float(args[0])
            vy = float(args[1])
            vz = float(args[2])
            duration = float(args[3]) if len(args) > 3 else None
        except ValueError:
            print("  ❌ Valores inválidos")
            return
        
        print(f"  Velocidade: vx={vx}, vy={vy}, vz={vz}")
        result = self.drone.set_velocity(vx, vy, vz, duration=duration)
        
        if result == NavResult.SUCCESS:
            print("  ✅ Comando enviado")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_velocity_body(self, args):
        """Velocidade no referencial do corpo"""
        if not self._check_connected():
            return
        
        if len(args) < 3:
            print("  Uso: velocity_body <frente> <direita> <baixo> [duracao]")
            print("  Exemplo: velocity_body 5 0 0 3 (5m/s frente por 3s)")
            return
        
        try:
            vx = float(args[0])
            vy = float(args[1])
            vz = float(args[2])
            duration = float(args[3]) if len(args) > 3 else None
        except ValueError:
            print("  ❌ Valores inválidos")
            return
        
        print(f"  Velocidade body: frente={vx}, direita={vy}, baixo={vz}")
        result = self.drone.set_velocity_body(vx, vy, vz, duration=duration)
        
        if result == NavResult.SUCCESS:
            print("  ✅ Comando enviado")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_yaw(self, args):
        """Yaw absoluto"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: yaw <angulo>")
            print("  Exemplo: yaw 90 (apontar para leste)")
            return
        
        try:
            angle = float(args[0])
        except ValueError:
            print("  ❌ Ângulo inválido")
            return
        
        self.drone.set_yaw(angle, is_relative=False)
        print(f"  ✅ Apontando para {angle}°")
    
    def cmd_yaw_rel(self, args):
        """Yaw relativo"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: yaw_rel <angulo>")
            print("  Exemplo: yaw_rel 90 (girar 90° horário)")
            return
        
        try:
            angle = float(args[0])
        except ValueError:
            print("  ❌ Ângulo inválido")
            return
        
        self.drone.set_yaw(angle, is_relative=True)
        print(f"  ✅ Girando {angle}°")
    
    def cmd_yaw_rate(self, args):
        """Taxa de yaw"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: yaw_rate <taxa> [duracao]")
            print("  Exemplo: yaw_rate 30 3 (30°/s por 3 segundos)")
            return
        
        try:
            rate = float(args[0])
            duration = float(args[1]) if len(args) > 1 else None
        except ValueError:
            print("  ❌ Taxa inválida")
            return
        
        self.drone.set_yaw_rate(rate, duration=duration)
        print(f"  ✅ Rotação: {rate}°/s")
    
    def cmd_param_get(self, args):
        """Ler parâmetro"""
        if not self._check_connected():
            return
        
        if len(args) < 1:
            print("  Uso: param_get <nome>")
            print("  Exemplo: param_get SYSID_THISMAV")
            return
        
        param_name = args[0].upper()
        result, param = self.drone.get_param(param_name, timeout=3.0)
        
        if result == ParamResult.SUCCESS:
            print(f"  {param_name} = {param.value}")
        elif result == ParamResult.NOT_FOUND:
            print(f"  ❌ Parâmetro não encontrado: {param_name}")
        else:
            print(f"  ❌ Erro: {result}")
    
    def cmd_param_set(self, args):
        """Escrever parâmetro"""
        if not self._check_connected():
            return
        
        if len(args) < 2:
            print("  Uso: param_set <nome> <valor>")
            print("  Exemplo: param_set MIS_TAKEOFF_ALT 15")
            return
        
        param_name = args[0].upper()
        try:
            value = float(args[1])
        except ValueError:
            print(f"  ❌ Valor inválido: {args[1]}")
            return
        
        result = self.drone.set_param(param_name, value, timeout=3.0)
        
        if result == ParamResult.SUCCESS:
            print(f"  ✅ {param_name} = {value}")
        else:
            print(f"  ❌ Falha: {result}")
    
    def cmd_param_list(self, args):
        """Listar parâmetros"""
        if not self._check_connected():
            return
        
        pattern = args[0] if args else None
        
        if pattern:
            params = self.drone.find_params(pattern)
        else:
            params = self.drone.get_all_params()
        
        if not params:
            print("  Nenhum parâmetro encontrado. Use 'download_params' primeiro?")
            print("  (Os parâmetros são baixados automaticamente em background)")
            return
        
        print(f"\n  Parâmetros encontrados: {len(params)}")
        print("  " + "-" * 50)
        
        # Ordenar e mostrar
        for i, (name, param) in enumerate(sorted(params.items())):
            print(f"    {name:20} = {param.value}")
            if i >= 30 and len(params) > 40:
                print(f"    ... e mais {len(params) - 30} parâmetros")
                break
    
    def cmd_mission_show(self):
        """Mostrar missão local"""
        items = self.drone.get_mission()
        
        if not items:
            print("  Nenhum item na missão local")
            return
        
        print(f"\n  Missão local ({len(items)} itens):")
        print("  " + "-" * 50)
        
        for item in items:
            cmd_name = self._get_command_name(item.command)
            if item.x != 0 or item.y != 0:
                print(f"    [{item.seq:2d}] {cmd_name}: lat={item.x:.6f}, lon={item.y:.6f}, alt={item.z:.1f}m")
            else:
                print(f"    [{item.seq:2d}] {cmd_name}: param1={item.param1}, param2={item.param2}")
    
    def cmd_mission_clear(self):
        """Limpar missão local"""
        self.drone.clear_local_mission()
        print("  ✅ Missão local limpa")
    
    def cmd_mission_add_wp(self, args):
        """Adicionar waypoint à missão"""
        if len(args) < 3:
            print("  Uso: mission_add_wp <lat> <lon> <alt>")
            print("  Exemplo: mission_add_wp -35.363261 149.165230 30")
            return
        
        try:
            lat = float(args[0])
            lon = float(args[1])
            alt = float(args[2])
        except ValueError:
            print("  ❌ Coordenadas inválidas")
            return
        
        self.drone.add_waypoint(lat, lon, alt)
        print(f"  ✅ Waypoint adicionado: ({lat}, {lon}, {alt}m)")
    
    def cmd_mission_add_loiter(self, args):
        """Adicionar loiter à missão"""
        if len(args) < 1:
            print("  Uso: mission_add_loiter <segundos> [raio]")
            print("  Exemplo: mission_add_loiter 5 10")
            return
        
        try:
            seconds = float(args[0])
            radius = float(args[1]) if len(args) > 1 else 10.0
        except ValueError:
            print("  ❌ Valores inválidos")
            return
        
        self.drone.add_loiter(seconds, radius)
        print(f"  ✅ Loiter adicionado: {seconds}s, raio={radius}m")
    
    def cmd_mission_upload(self):
        """Enviar missão para o drone"""
        if not self._check_connected():
            return
        
        items = self.drone.get_mission()
        if not items:
            print("  ❌ Nenhum item na missão local. Adicione waypoints primeiro.")
            return
        
        print(f"  Enviando missão com {len(items)} itens...")
        result = self.drone.upload_mission(timeout=30.0)
        
        if result == MissionResult.SUCCESS:
            print("  ✅ Missão enviada com sucesso!")
        else:
            print(f"  ❌ Falha no upload: {result}")
    
    def cmd_mission_run(self):
        """Executar missão"""
        if not self._check_connected():
            return
        
        # Verificar se tem missão
        items = self.drone.get_mission()
        if not items:
            print("  ❌ Nenhum item na missão local. Adicione waypoints primeiro.")
            return
        
        print("  Preparando para executar missão...")
        
        # Enviar missão
        print("  Enviando missão...")
        result = self.drone.upload_mission(timeout=30.0)
        if result != MissionResult.SUCCESS:
            print(f"  ❌ Falha no upload: {result}")
            return
        
        # Mudar para AUTO
        print("  Mudando para modo AUTO...")
        self.drone.set_mode("AUTO")
        time.sleep(0.5)
        
        # Armar (inicia a missão)
        print("  Armando e iniciando missão...")
        result = self.drone.arm(timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print("  ✅ Missão iniciada!")
            print("  Monitorando missão (pressione Ctrl+C para interromper)...")
            
            try:
                while self.drone.state.is_armed():
                    alt = self.drone.state.position.relative_altitude or 0
                    # Usar print com \r mas sem interferir - como é uma linha separada
                    print(f"\r    Altitude: {alt:.1f}m | Modo: {self.drone.state.flight_mode()}", end="", flush=True)
                    time.sleep(1)
                print("\n  ✅ Missão concluída!")
            except KeyboardInterrupt:
                print("\n  ⚠️ Missão interrompida!")
                self.drone.set_mode("RTL")
        else:
            print(f"  ❌ Falha ao armar: {result}")
    
    def cmd_reboot(self):
        """Reiniciar autopilot"""
        if not self._check_connected():
            return
        
        print("  ⚠️ ATENÇÃO: Reiniciando autopilot!")
        confirm = input("  Confirmar? (s/N): ").lower().strip()
        
        if confirm == 's':
            result = self.drone.reboot(autopilot_only=True)
            if result == CommandResult.SUCCESS:
                print("  ✅ Comando de reboot enviado")
                print("  O drone será reiniciado em alguns segundos...")
                self.drone = None
            else:
                print(f"  ❌ Falha: {result}")
        else:
            print("  Reboot cancelado")
    
    def cmd_exit(self):
        """Sair do programa"""
        print("\n  Encerrando...")
        self.running = False
    
    def start_auto_status(self):
        """Inicia thread de status automático"""
        if self.auto_status:
            print("  Status automático já está ativo")
            return
        
        self.auto_status = True
        self.status_thread = threading.Thread(target=self._auto_status_loop, daemon=True)
        self.status_thread.start()
        print("  ✅ Status automático ativado (a cada 10 segundos)")
    
    def stop_auto_status(self):
        """Para thread de status automático"""
        if not self.auto_status:
            print("  Status automático já está inativo")
            return
        
        self.auto_status = False
        if self.status_thread:
            self.status_thread.join(timeout=2)
        print("  ✅ Status automático desativado")
    
    def _auto_status_loop(self):
        """Loop de status automático - NÃO interfere no input"""
        while self.auto_status and self.drone and self.drone.is_connected:
            time.sleep(10)  # A cada 10 segundos (menos frequente)
            if self.auto_status and self.drone and self.drone.is_connected:
                try:
                    self._print_mini_status()
                except:
                    pass
    
    def _print_mini_status(self):
        """Imprime status resumido em uma linha separada"""
        if not self.drone or not self.drone.is_connected:
            return
        
        try:
            mode = self.drone.state.flight_mode()
            armed = "🔴 ARMADO" if self.drone.state.is_armed() else "⚪ DESARMADO"
            alt = self.drone.state.position.relative_altitude or 0
            bat = self.drone.state.battery.voltage or 0
            fix = self.drone.state.gps.fix_type or 0
            
            # Imprime uma linha separada de status (não interfere no input)
            print(f"\n  [STATUS] {armed} | Modo: {mode} | Alt: {alt:.1f}m | Bat: {bat:.1f}V | GPS Fix: {fix}")
            print("> ", end="", flush=True)  # Re-exibe o prompt
        except:
            pass
    
    def _check_connected(self):
        """Verifica se está conectado"""
        if not self.drone or not self.drone.is_connected:
            print("  ❌ Não conectado. Use 'connect' primeiro")
            return False
        return True
    
    def _get_command_name(self, command_id):
        """Obtém nome do comando MAV_CMD"""
        cmd_names = {
            22: "NAV_TAKEOFF",
            16: "NAV_WAYPOINT",
            21: "NAV_LAND",
            20: "NAV_RTL",
            17: "NAV_LOITER_TIME",
            18: "NAV_LOITER_UNLIM",
            200: "DO_DELAY",
            183: "DO_SET_SERVO",
        }
        return cmd_names.get(command_id, f"CMD_{command_id}")
    
    def cleanup(self):
        """Limpeza final"""
        self.stop_auto_status()
        if self.drone:
            if self.drone.state.is_armed():
                print("\n  Desarmando...")
                self.drone.disarm()
            self.drone.disconnect()
        print("\n  Programa encerrado.")


def main():
    """Função principal"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Console interativo para controle de drone')
    parser.add_argument('-c', '--connection', default='udp:127.0.0.1:14550',
                        help='String de conexão (padrão: udp:127.0.0.1:14550)')
    
    args = parser.parse_args()
    
    console = DroneConsole(args.connection)
    console.start()


if __name__ == "__main__":
    main()
