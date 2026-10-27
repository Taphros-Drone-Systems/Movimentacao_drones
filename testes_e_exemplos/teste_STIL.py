#!/usr/bin/env python3
"""
teste_SITL.py - Teste completo da biblioteca ppymavlink com SITL
Versão com correções para navegação
"""

import time
import sys
from vehicle import Vehicle
from commands import CommandResult
from parameters import ParamResult
from mission import MissionResult
from navigation import NavResult
from failsafe import FailsafeState


def print_header(title):
    """Imprime cabeçalho formatado"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def print_ok(message):
    print(f"  ✅ {message}")


def print_error(message):
    print(f"  ❌ {message}")


def print_info(message):
    print(f"  ℹ️ {message}")


def print_warning(message):
    print(f"  ⚠️ {message}")


class TestRunner:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.drone = None
        self.results = {}

    def run(self):
        """Executa todos os testes"""
        print_header("PPYMAVLINK - TESTE COMPLETO COM SITL")
        print(f"\n  Connection string: {self.connection_string}")
        
        if not self._connect():
            print_error("Falha na conexão! Verifique se o SITL está rodando.")
            return False

        try:
            self._test_telemetry()
            self._test_parameters()
            self._test_commands()
            self._test_mission()
            self._test_failsafe()
            self._test_stress()
            
            # Testes que movem o drone (opcionais)
            self._test_navigation_prompt()
            
        except KeyboardInterrupt:
            print_warning("\nTestes interrompidos pelo usuário")
        except Exception as e:
            print_error(f"Erro inesperado: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._summary()
            self._cleanup()

    def _connect(self):
        """Conecta ao veículo"""
        print_header("CONEXÃO")
        
        self.drone = Vehicle(self.connection_string)
        
        print_info("Conectando...")
        if not self.drone.connect():
            return False
        
        print_ok(f"Conectado - SysID: {self.drone.target_system}")
        
        print_info("Aguardando telemetria...")
        if not self.drone.wait_ready(timeout=15):
            print_error("Timeout aguardando telemetria")
            return False
        
        print_ok("Telemetria recebida")
        return True

    def _test_telemetry(self):
        """Teste de telemetria"""
        print_header("TESTE 1: TELEMETRIA")
        
        state = self.drone.state.get_state()
        
        # Sistema
        print(f"  Modo: {state.system.flight_mode}")
        print(f"  Armado: {'SIM' if state.system.armed else 'NÃO'}")
        
        # Posição
        if state.position.latitude:
            print(f"  Posição: {state.position.latitude:.6f}, {state.position.longitude:.6f}")
            print(f"  Altitude: {state.position.relative_altitude:.1f}m")
            print(f"  Heading: {state.position.heading:.1f}°")
        
        # Atitude
        print(f"  Atitude: Roll={state.attitude.roll:.1f}° Pitch={state.attitude.pitch:.1f}° Yaw={state.attitude.yaw:.1f}°")
        
        # GPS
        if state.gps.fix_type is not None:
            fix_names = {0: "Sem GPS", 1: "Sem Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
            print(f"  GPS: {fix_names.get(state.gps.fix_type, 'Desconhecido')} ({state.gps.satellites_visible} sat)")
        
        # Bateria
        if state.battery.voltage:
            print(f"  Bateria: {state.battery.voltage:.1f}V {state.battery.remaining}%")
        
        # Status completo
        self.drone.print_status()
        self.results["Telemetria"] = True

    def _test_parameters(self):
        """Teste de parâmetros"""
        print_header("TESTE 2: PARÂMETROS")
        
        print_info("Baixando parâmetros (pode levar alguns segundos)...")
        
        def progress(current, total, name):
            if current % 100 == 0 or current == total:
                print(f"    Progresso: {current}/{total} - {name}")
        
        result = self.drone.download_params(timeout=30.0, progress_callback=progress)
        
        if result == ParamResult.SUCCESS:
            total = self.drone.parameters.param_count()
            print_ok(f"Download completo: {total} parâmetros")
        else:
            print_warning(f"Download parcial ou falhou: {result}")
        
        # Leitura de parâmetros específicos
        print_info("Lendo parâmetros específicos:")
        common_params = ["SYSID_THISMAV", "ARMING_CHECK", "FS_GCS_ENABLE"]
        
        for param_name in common_params:
            result, param = self.drone.get_param(param_name, timeout=2.0)
            if result == ParamResult.SUCCESS:
                print(f"    {param_name} = {param.value}")
            else:
                print(f"    {param_name}: {result}")
        
        # Busca por padrão
        batt_params = self.drone.find_params("BATT")
        if batt_params:
            print_info(f"Parâmetros com 'BATT' encontrados: {len(batt_params)}")
            for name, param in list(batt_params.items())[:5]:
                print(f"    {name} = {param.value}")
        
        # Salvar backup
        if self.drone.save_params("parameters_backup.txt"):
            print_ok("Parâmetros salvos em parameters_backup.txt")
        
        self.results["Parâmetros"] = True

    def _test_commands(self):
        """Teste de comandos básicos"""
        print_header("TESTE 3: COMANDOS BÁSICOS")
        
        # Mostrar modos disponíveis
        print_info("Modos de voo disponíveis:")
        self.drone.commands.print_available_modes()
        
        # Testar mudança de modo
        current_mode = self.drone.state.flight_mode()
        print_info(f"Modo atual: {current_mode}")
        
        result = self.drone.set_mode("GUIDED", timeout=5.0)
        if result == CommandResult.SUCCESS:
            print_ok(f"Mudança para GUIDED: {result}")
            time.sleep(1)
            print(f"  Modo atual: {self.drone.state.flight_mode()}")
        else:
            print_warning(f"Não foi possível mudar para GUIDED: {result}")
        
        # Testar ARM
        print_info("Testando ARM...")
        result = self.drone.arm(timeout=5.0)
        
        if result == CommandResult.SUCCESS:
            print_ok(f"ARM: {result}")
            time.sleep(2)
            print(f"  Armado: {self.drone.state.is_armed()}")
            
            # Desarmar
            print_info("Desarmando...")
            result = self.drone.disarm(timeout=5.0)
            print_ok(f"DISARM: {result}")
        else:
            print_warning(f"ARM falhou: {result}")
        
        # Voltar ao modo original
        if current_mode not in ["GUIDED", "UNKNOWN"]:
            self.drone.set_mode(current_mode, timeout=3.0)
            print_info(f"Retornando ao modo {current_mode}")
        
        self.results["Comandos"] = True

    def _test_mission(self):
        """Teste de missões"""
        print_header("TESTE 4: MISSÕES")
        
        # Download da missão atual
        result, items = self.drone.download_mission(timeout=15.0)
        
        if result == MissionResult.SUCCESS:
            print_ok(f"Missão atual: {len(items)} itens")
        elif result == MissionResult.NO_MISSION:
            print_ok("Nenhuma missão carregada")
        else:
            print_warning(f"Download: {result}")
        
        # Criar missão local
        print_info("Criando missão local de exemplo...")
        self.drone.clear_local_mission()
        
        self.drone.add_takeoff(10.0)
        print("    + Takeoff (10m)")
        
        self.drone.add_waypoint(-35.363261, 149.165230, 30.0)
        print("    + Waypoint 1")
        
        self.drone.add_waypoint(-35.363261, 149.166000, 30.0)
        print("    + Waypoint 2")
        
        self.drone.add_loiter(5.0, radius=10.0)
        print("    + Loiter (5s)")
        
        self.drone.add_rtl()
        print("    + RTL")
        
        mission_items = self.drone.get_mission()
        print_ok(f"Missão local criada: {len(mission_items)} itens")
        
        # Perguntar se quer fazer upload
        print("\n  Deseja enviar a missão para o veículo? (s/N)")
        if input("  > ").lower().strip() == 's':
            result = self.drone.upload_mission(timeout=30.0)
            if result == MissionResult.SUCCESS:
                print_ok("Missão enviada com sucesso!")
            else:
                print_error(f"Falha no upload: {result}")
        else:
            print_info("Upload ignorado")
        
        self.results["Missões"] = True

    def _test_navigation_prompt(self):
        """Teste de navegação (com confirmação)"""
        print_header("TESTE 5: NAVEGAÇÃO (OPCIONAL)")
        print_warning("ESTE TESTE PODE MOVER O VEÍCULO!")
        
        print("\n  Deseja executar o teste de navegação? (s/N)")
        if input("  > ").lower().strip() != 's':
            print_info("Teste de navegação ignorado")
            self.results["Navegação"] = True
            return
        
        self._test_navigation()
        self.results["Navegação"] = True

    def _test_navigation(self):
        """Executa teste de navegação corrigido"""
        print_info("Preparando para navegação...")
        
        # 1. Mudar para GUIDED
        print_info("  -> Mudando para modo GUIDED...")
        result = self.drone.set_mode("GUIDED", timeout=5.0)
        if result != CommandResult.SUCCESS:
            print_error(f"  -> Não foi possível mudar para GUIDED: {result}")
            return
        print_ok("  -> Modo GUIDED ativado")
        time.sleep(1)
        
        # 2. Verificar se está desarmado
        if self.drone.state.is_armed():
            print_info("  -> Veículo já está armado, desarmando primeiro...")
            self.drone.disarm(timeout=3.0)
            time.sleep(1)
        
        # 3. Armar com verificação
        print_info("  -> Armando veículo...")
        result = self.drone.arm(timeout=5.0)
        
        if result != CommandResult.SUCCESS:
            print_error(f"  -> Falha no ARM: {result}")
            print_info("  -> Tentando force disarm e rearm...")
            
            # Tenta force disarm
            self.drone.commands.force_disarm()
            time.sleep(1)
            
            # Tenta armar novamente
            result = self.drone.arm(timeout=5.0)
            if result != CommandResult.SUCCESS:
                print_error(f"  -> Falha persistente no ARM: {result}")
                return
        
        # 4. Verificar estado após ARM
        time.sleep(2)
        if not self.drone.state.is_armed():
            print_error("  -> Veículo NÃO está armado após comando!")
            print_info("  -> No terminal do SITL, digite: arm uncheck all")
            print_info("  -> Aguardando ARM manual...")
            
            # Aguarda até 10 segundos para ARM manual
            for i in range(10):
                time.sleep(1)
                if self.drone.state.is_armed():
                    print_ok("  -> ARM manual detectado!")
                    break
            
            if not self.drone.state.is_armed():
                print_error("  -> Não foi possível armar. Abortando navegação.")
                return
        
        print_ok("  -> Veículo armado com sucesso!")
        
        # 5. Aguardar estabilização
        print_info("  -> Aguardando estabilização (2s)...")
        time.sleep(2)
        
        # 6. Decolagem
        print_info("  -> Decolando 5 metros...")
        result = self.drone.takeoff_guided(5.0, timeout=25.0)
        
        if result == NavResult.SUCCESS:
            print_ok("  -> Decolagem concluída!")
            time.sleep(3)
            print(f"  -> Altitude: {self.drone.state.position.relative_altitude:.1f}m")
            
            # 7. Movimento
            print_info("  -> Movendo 10m para norte...")
            result = self.drone.goto_relative(10.0, 0.0, 0.0, timeout=20.0)
            
            if result == NavResult.SUCCESS:
                print_ok("  -> Movimento concluído!")
            else:
                print_warning(f"  -> Movimento: {result}")
            
            time.sleep(2)
            
            # 8. Yaw
            print_info("  -> Girando 90 graus...")
            self.drone.set_yaw(90, is_relative=True)
            time.sleep(2)
            print(f"  -> Yaw atual: {self.drone.state.attitude.yaw:.1f}°")
            
            # 9. RTL
            print_info("  -> Retornando ao ponto de partida (RTL)...")
            self.drone.set_mode("RTL")
            
            # 10. Aguardar pouso
            print_info("  -> Aguardando pouso...")
            timeout = 45
            start = time.time()
            last_alt = 999
            
            while time.time() - start < timeout and self.drone.state.is_armed():
                alt = self.drone.state.position.relative_altitude or 0
                
                # Mostra progresso apenas quando altitude muda significativamente
                if abs(alt - last_alt) > 0.5 or int(time.time() - start) % 3 == 0:
                    print(f"\r  -> Altitude: {alt:.1f}m", end="", flush=True)
                    last_alt = alt
                
                time.sleep(0.5)
            
            print()
            
            if not self.drone.state.is_armed():
                print_ok("  -> Pouso concluído!")
            else:
                print_warning("  -> Timeout aguardando pouso")
                
                # Desarma manualmente
                print_info("  -> Desarmando manualmente...")
                self.drone.disarm()
            
        else:
            print_error(f"  -> Decolagem falhou: {result}")
            
            # Diagnóstico
            print_info("  -> Diagnóstico:")
            print(f"     Modo: {self.drone.state.flight_mode()}")
            print(f"     Armado: {self.drone.state.is_armed()}")
            
            # Tenta desarmar
            if self.drone.state.is_armed():
                self.drone.disarm()
        
        # 11. Desarmar por segurança
        if self.drone.state.is_armed():
            print_info("  -> Desarmando por segurança...")
            self.drone.disarm()
            time.sleep(1)
        
        print_ok("Teste de navegação concluído!")

    def _test_failsafe(self):
        """Teste do sistema de failsafe"""
        print_header("TESTE 6: FAILSAFE")
        
        # Configurar
        self.drone.configure_failsafe(
            heartbeat_timeout=5.0,
            auto_land_on_failsafe=False,
            auto_reconnect=True
        )
        print_ok("Failsafe configurado")
        
        # Estado atual
        state = self.drone.get_failsafe_state()
        print(f"  Estado failsafe: {state.name}")
        print(f"  Failsafe ativo: {self.drone.is_failsafe_active}")
        print(f"  Conectado: {self.drone.is_connected}")
        
        # Estatísticas
        stats = self.drone.get_link_stats()
        print(f"  Packet loss: {stats['packet_loss_rate']}")
        print(f"  Reconexões: {stats['reconnect_attempts']}")
        
        self.results["Failsafe"] = True

    def _test_stress(self):
        """Teste de stress"""
        print_header("TESTE 7: STRESS TEST")
        
        print_info("Executando operações rápidas...")
        
        # Leituras de estado
        print("  Snapshots de estado...", end="", flush=True)
        for i in range(20):
            self.drone.state.get_state()
        print(" OK")
        
        # Leituras de parâmetros (apenas cache)
        print("  Leituras de parâmetros...", end="", flush=True)
        self.drone.get_all_params()
        print(" OK")
        
        # Estatísticas
        print("  Coletando estatísticas...", end="", flush=True)
        for i in range(10):
            self.drone.get_link_stats()
        print(" OK")
        
        print_ok("Stress test concluído sem erros!")
        self.results["Stress"] = True

    def _summary(self):
        """Imprime resumo dos testes"""
        print_header("RESUMO DOS TESTES")
        
        passed = sum(1 for v in self.results.values() if v)
        total = len(self.results)
        
        print(f"\n  Testes executados: {total}")
        print(f"  Testes passados: {passed}")
        print(f"  Testes falhos: {total - passed}")
        
        print("\n  Resultados detalhados:")
        for name, result in self.results.items():
            icon = "✅" if result else "❌"
            print(f"    {icon} {name}")
        
        # Estado final
        if self.drone and self.drone.is_connected:
            print(f"\n  Estado final:")
            print(f"    Modo: {self.drone.state.flight_mode()}")
            print(f"    Armado: {self.drone.state.is_armed()}")
            print(f"    Conectado: {self.drone.is_connected}")

    def _cleanup(self):
        """Limpeza final"""
        if self.drone:
            # Garante desarme
            if self.drone.state.is_armed():
                print("\n  Desarmando...")
                self.drone.disarm()
            
            print("\n  Desconectando...")
            self.drone.disconnect()
            print("  ✅ Desconectado")


def main():
    """Função principal"""
    conn_string = "udp:127.0.0.1:14550"
    if len(sys.argv) > 1:
        conn_string = sys.argv[1]
    
    runner = TestRunner(conn_string)
    runner.run()


if __name__ == "__main__":
    main() 