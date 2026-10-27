"""
main.py

Teste completo da biblioteca ppymavlink.
Demonstra todas as funcionalidades das Fases 1-5.
"""

import time
import sys
import threading
from vehicle import Vehicle, MissionItem
from commands import CommandResult
from parameters import ParamResult
from mission import MissionResult
from navigation import NavResult
from failsafe import FailsafeState


# ===================================================================
# UTILITÁRIOS DE EXIBIÇÃO
# ===================================================================

def print_separator(title=""):
    """Imprime separador formatado."""
    print("\n" + "=" * 70)
    if title:
        print(f"  {title}")
        print("=" * 70)


def print_subtitle(text):
    """Imprime subtítulo."""
    print(f"\n  ▸ {text}")


def print_result(success, message):
    """Imprime resultado de teste."""
    icon = "✓" if success else "✗"
    print(f"    {icon} {message}")


def print_state(vehicle_state):
    """Imprime estado completo do veículo."""
    print_separator("ESTADO DO VEÍCULO")

    system = vehicle_state.system
    print(f"  Modo: {system.flight_mode}")
    print(f"  Armado: {'SIM' if system.armed else 'NÃO'}")
    print(f"  Status: {system.system_status}")

    att = vehicle_state.attitude
    print(f"  Atitude: Roll={att.roll:.1f}° Pitch={att.pitch:.1f}° Yaw={att.yaw:.1f}°")

    pos = vehicle_state.position
    if pos.latitude is not None:
        print(f"  Posição: Lat={pos.latitude:.6f} Lon={pos.longitude:.6f}")
        print(f"  Altitude Rel: {pos.relative_altitude:.2f}m Abs: {pos.absolute_altitude:.2f}m")
        print(f"  Velocidade: vx={pos.vx:.2f} vy={pos.vy:.2f} vz={pos.vz:.2f} m/s")
        print(f"  Heading: {pos.heading:.1f}°")

    gps = vehicle_state.gps
    if gps.fix_type is not None:
        fix_names = {
            0: "Sem GPS", 1: "Sem Fix", 2: "2D Fix", 
            3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
        }
        fix_str = fix_names.get(gps.fix_type, f"Tipo {gps.fix_type}")
        print(f"  GPS: {fix_str} ({gps.satellites_visible} satélites)")

    bat = vehicle_state.battery
    if bat.voltage is not None:
        print(f"  Bateria: {bat.voltage:.1f}V {bat.current:.1f}A {bat.remaining}%")


def print_link_stats(stats):
    """Imprime estatísticas do link."""
    print_separator("ESTATÍSTICAS DO LINK")
    print(f"  Estado: {stats['state']}")
    print(f"  Pacotes Enviados: {stats['packets_sent']}")
    print(f"  Pacotes Recebidos: {stats['packets_received']}")
    print(f"  Pacotes Perdidos: {stats['packets_lost']}")
    print(f"  Taxa de Perda: {stats['packet_loss_rate']}")
    print(f"  Tentativas de Reconexão: {stats['reconnect_attempts']}")
    print(f"  Failsafe Ativo: {stats['is_failsafe_active']}")


# ===================================================================
# TESTE 1: CONEXÃO E TELEMETRIA (Fases 1-2)
# ===================================================================

def test_connection_and_telemetry(connection_string="udp:127.0.0.1:14550"):
    """
    Testa conexão básica e telemetria.
    """
    print_separator("TESTE 1: CONEXÃO E TELEMETRIA (Fases 1-2)")
    
    drone = Vehicle(connection_string)
    
    # Conecta
    print_subtitle("Conectando ao veículo...")
    if not drone.connect():
        print_result(False, "Falha na conexão")
        return None
    
    print_result(True, f"Conectado - SysID: {drone.target_system}")
    
    # Aguarda telemetria
    print_subtitle("Aguardando telemetria inicial...")
    if not drone.wait_ready(timeout=15):
        print_result(False, "Timeout aguardando telemetria")
        drone.disconnect()
        return None
    
    print_result(True, "Telemetria recebida")
    
    # Exibe estado
    state = drone.state.get_state()
    print_state(state)
    
    # Verifica se temos dados básicos
    has_position = state.position.latitude is not None
    has_attitude = state.attitude.roll is not None or state.attitude.pitch is not None
    has_mode = state.system.flight_mode != "UNKNOWN"
    
    print_subtitle("Verificações:")
    print_result(has_position, f"Posição: {'OK' if has_position else 'SEM DADOS'}")
    print_result(has_attitude, f"Atitude: {'OK' if has_attitude else 'SEM DADOS'}")
    print_result(has_mode, f"Modo: {state.system.flight_mode}")
    
    # Exibe status formatado
    drone.print_status()
    
    return drone


# ===================================================================
# TESTE 2: COMANDOS BÁSICOS (Fase 3)
# ===================================================================

def test_basic_commands(drone: Vehicle):
    """
    Testa comandos básicos de controle.
    """
    print_separator("TESTE 2: COMANDOS BÁSICOS (Fase 3)")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    # 1. Teste de modos
    print_subtitle("1. Teste de mudança de modo")
    
    current_mode = drone.state.flight_mode()
    print(f"    Modo atual: {current_mode}")
    
    # Tenta modo GUIDED
    result = drone.set_mode("GUIDED", timeout=5.0)
    print_result(
        result == CommandResult.SUCCESS,
        f"set_mode('GUIDED'): {result}"
    )
    
    time.sleep(1)
    new_mode = drone.state.flight_mode()
    mode_ok = new_mode == "GUIDED"
    print_result(mode_ok, f"Modo após mudança: {new_mode}")
    
    if not mode_ok:
        print("    Modo GUIDED não disponível, testes continuarão com modo atual")
    
    # 2. Teste ARM/DISARM
    print_subtitle("2. Teste ARM/DISARM")
    
    result = drone.arm(timeout=5.0)
    armed_ok = result == CommandResult.SUCCESS
    print_result(armed_ok, f"arm(): {result}")
    
    if armed_ok:
        time.sleep(2)
        is_armed = drone.state.is_armed()
        print_result(is_armed, f"Confirmado armado: {is_armed}")
        
        if is_armed:
            print_subtitle("3. Desarmando...")
            result = drone.disarm(timeout=5.0)
            print_result(
                result == CommandResult.SUCCESS,
                f"disarm(): {result}"
            )
            time.sleep(1)
            print_result(not drone.state.is_armed(), "Confirmado desarmado")
    else:
        print("    ARM não disponível (simulador SITL pode exigir GPS fix)")
    
    # 3. Retorna ao modo original
    if current_mode != "GUIDED" and current_mode != "UNKNOWN":
        print_subtitle("4. Retornando ao modo original...")
        result = drone.set_mode(current_mode, timeout=5.0)
        print_result(
            result == CommandResult.SUCCESS,
            f"set_mode('{current_mode}'): {result}"
        )
    
    return True


# ===================================================================
# TESTE 3: PARÂMETROS (Fase 4)
# ===================================================================

def test_parameters(drone: Vehicle):
    """
    Testa gerenciamento de parâmetros.
    """
    print_separator("TESTE 3: PARÂMETROS (Fase 4)")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    # 1. Download de todos os parâmetros
    print_subtitle("1. Download de parâmetros...")
    
    download_done = threading.Event()
    progress_data = {'current': 0, 'total': 0, 'last_name': ''}
    
    def progress_callback(current, total, name):
        progress_data['current'] = current
        progress_data['total'] = total
        progress_data['last_name'] = name
        
        if current == 1:
            print(f"    Iniciando download...")
        elif current % 50 == 0 or current == total:
            print(f"    Progresso: {current}/{total} - {name}")
        
        if current >= total:
            download_done.set()
    
    result = drone.download_params(timeout=60.0, progress_callback=progress_callback)
    
    params_ok = result == ParamResult.SUCCESS
    print_result(params_ok, f"Download: {result}")
    
    if params_ok:
        total = drone.parameters.param_count()
        print_result(True, f"Total de parâmetros: {total}")
    
    # 2. Leitura de parâmetro específico
    print_subtitle("2. Leitura de parâmetros específicos")
    
    # Tenta alguns parâmetros comuns
    common_params = [
        "SYSID_THISMAV",
        "MIS_TAKEOFF_ALT",
        "BATT_MONITOR",
        "ARMING_CHECK",
        "FS_GCS_ENABLE"
    ]
    
    for param_name in common_params:
        result, param = drone.get_param(param_name, timeout=3.0)
        if result == ParamResult.SUCCESS:
            print(f"    {param_name} = {param.value}")
        else:
            print(f"    {param_name}: {result}")
    
    # 3. Busca por padrão
    print_subtitle("3. Busca de parâmetros (padrão 'BATT_')")
    
    batt_params = drone.find_params("BATT_")
    if batt_params:
        for name, param in list(batt_params.items())[:5]:
            print(f"    {name} = {param.value}")
        if len(batt_params) > 5:
            print(f"    ... e mais {len(batt_params) - 5} parâmetros")
    else:
        print("    Nenhum parâmetro 'BATT_' encontrado no cache")
    
    # 4. Exportar parâmetros
    print_subtitle("4. Exportando parâmetros para arquivo...")
    export_ok = drone.save_params("parameters_backup.txt")
    print_result(export_ok, f"Exportação: {'OK' if export_ok else 'FALHA'}")
    
    return True


# ===================================================================
# TESTE 4: MISSÕES (Fase 4)
# ===================================================================

def test_mission(drone: Vehicle):
    """
    Testa gerenciamento de missões.
    """
    print_separator("TESTE 4: MISSÕES (Fase 4)")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    # 1. Download da missão atual
    print_subtitle("1. Download da missão atual...")
    
    result, items = drone.download_mission(timeout=15.0)
    
    if result == MissionResult.SUCCESS:
        print_result(True, f"Missão baixada: {len(items)} itens")
        
        for item in items[:5]:
            print(f"    Item {item.seq}: {item}")
        if len(items) > 5:
            print(f"    ... e mais {len(items) - 5} itens")
    elif result == MissionResult.NO_MISSION:
        print_result(True, "Nenhuma missão no veículo (OK)")
    else:
        print_result(False, f"Download: {result}")
    
    # 2. Cria missão local
    print_subtitle("2. Criando missão local...")
    
    # Limpa missão local
    drone.clear_local_mission()
    
    # Adiciona itens
    drone.add_takeoff(10.0)
    print("    + Takeoff (10m)")
    
    # Waypoints de exemplo (SITL default location)
    drone.add_waypoint(-35.363261, 149.165230, 30.0)
    print("    + Waypoint 1 (-35.363, 149.165, 30m)")
    
    drone.add_waypoint(-35.363261, 149.166000, 30.0)
    print("    + Waypoint 2 (-35.363, 149.166, 30m)")
    
    drone.add_waypoint(-35.363800, 149.166000, 30.0)
    print("    + Waypoint 3 (-35.364, 149.166, 30m)")
    
    drone.add_loiter(5.0, radius=10.0)
    print("    + Loiter (5s, raio 10m)")
    
    drone.add_rtl()
    print("    + RTL (Return to Launch)")
    
    mission_items = drone.get_mission()
    print_result(
        len(mission_items) == 6,
        f"Total de itens: {len(mission_items)}"
    )
    
    # 3. Lista missão
    print_subtitle("3. Missão criada:")
    for item in mission_items:
        print(f"    Seq {item.seq}: {item}")
    
    # 4. Exporta missão
    print_subtitle("4. Exportando missão...")
    export_ok = drone.export_mission("mission_backup.waypoints")
    print_result(export_ok, f"Exportação: {'OK' if export_ok else 'FALHA'}")
    
    # 5. Upload da missão
    print_subtitle("5. Upload da missão...")
    
    upload_input = input("    Deseja enviar missão para o veículo? (s/N): ").lower().strip()
    
    if upload_input == 's':
        result = drone.upload_mission(timeout=30.0)
        print_result(
            result == MissionResult.SUCCESS,
            f"Upload: {result}"
        )
        
        if result == MissionResult.SUCCESS:
            # Verifica upload
            time.sleep(2)
            result2, items2 = drone.download_mission(timeout=15.0)
            if result2 == MissionResult.SUCCESS:
                print_result(
                    len(items2) == len(mission_items),
                    f"Verificação: {len(items2)} itens recebidos"
                )
    else:
        print("    Upload ignorado")
    
    # 6. Limpa missão do veículo (opcional)
    clear_input = input("\n    Deseja limpar missão do veículo? (s/N): ").lower().strip()
    
    if clear_input == 's':
        result = drone.clear_mission(timeout=10.0)
        print_result(
            result == MissionResult.SUCCESS,
            f"Clear: {result}"
        )
    else:
        print("    Limpeza ignorada")
    
    return True


# ===================================================================
# TESTE 5: NAVEGAÇÃO GUIADA (Fase 4)
# ===================================================================

def test_navigation(drone: Vehicle):
    """
    Testa navegação guiada.
    CUIDADO: Este teste requer modo GUIDED e ARM.
    """
    print_separator("TESTE 5: NAVEGAÇÃO GUIADA (Fase 4)")
    print("  ATENÇÃO: Este teste pode mover o veículo!")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    # Verifica se deve executar
    nav_input = input("\n  Executar teste de navegação? (s/N): ").lower().strip()
    if nav_input != 's':
        print("  Teste ignorado")
        return True
    
    # 1. Verifica pré-condições
    print_subtitle("1. Verificando pré-condições...")
    
    # Modo GUIDED
    current_mode = drone.state.flight_mode()
    if current_mode != "GUIDED":
        print(f"    Mudando para GUIDED (atual: {current_mode})...")
        result = drone.set_mode("GUIDED", timeout=5.0)
        if result != CommandResult.SUCCESS:
            print_result(False, f"Não foi possível mudar para GUIDED: {result}")
            return False
        time.sleep(1)
    
    print_result(True, f"Modo: {drone.state.flight_mode()}")
    
    # ARM
    if not drone.state.is_armed():
        print("    Armando...")
        result = drone.arm(timeout=5.0)
        if result != CommandResult.SUCCESS:
            print_result(False, f"Não foi possível armar: {result}")
            return False
        time.sleep(2)
    
    print_result(drone.state.is_armed(), f"Armado: {drone.state.is_armed()}")
    
    # 2. Decolagem
    print_subtitle("2. Decolagem (3 metros)...")
    
    takeoff_alt = 3.0
    result = drone.takeoff_guided(takeoff_alt, timeout=15.0)
    
    if result == NavResult.SUCCESS:
        print_result(True, f"Decolagem concluída")
        time.sleep(2)
        
        current_alt = drone.state.position.relative_altitude
        print(f"    Altitude atual: {current_alt:.1f}m")
    
    # 3. Teste de Yaw
    print_subtitle("3. Teste de controle de Yaw...")
    
    print("    Girando 90° horário...")
    drone.set_yaw(90, is_relative=True, direction=1)
    time.sleep(3)
    
    current_yaw = drone.state.attitude.yaw
    print(f"    Yaw atual: {current_yaw:.1f}°")
    
    # 4. Movimento relativo
    print_subtitle("4. Movimento relativo (5m norte)...")
    
    result = drone.goto_relative(5.0, 0.0, 0.0, timeout=15.0)
    print_result(
        result == NavResult.SUCCESS,
        f"Movimento: {result}"
    )
    
    # 5. Controle por velocidade
    print_subtitle("5. Controle por velocidade (2s leste)...")
    
    drone.set_velocity(0, 2.0, 0, duration=2.0)
    print_result(True, "Velocidade aplicada")
    
    time.sleep(1)
    
    # 6. Loiter
    print_subtitle("6. Parando (loiter)...")
    drone.loiter()
    print_result(True, "Loiter ativado")
    
    time.sleep(2)
    
    # 7. Pouso
    print_subtitle("7. Iniciando pouso...")
    
    land_input = input("    Deseja pousar? (s/N): ").lower().strip()
    if land_input == 's':
        result = drone.land(timeout=10.0)
        print_result(
            result == CommandResult.SUCCESS,
            f"Comando LAND: {result}"
        )
        
        # Aguarda pouso
        print("    Aguardando pouso completo...")
        start = time.time()
        while time.time() - start < 30:
            if not drone.state.is_armed():
                print_result(True, "Pouso detectado (desarmado)")
                break
            time.sleep(1)
        
        # Desarma se ainda estiver armado
        if drone.state.is_armed():
            print("    Desarmando...")
            drone.disarm(timeout=5.0)
    else:
        print("    Pouso ignorado - lembre-se de pousar manualmente!")
    
    return True


# ===================================================================
# TESTE 6: FAILSAFE (Fase 5)
# ===================================================================

def test_failsafe(drone: Vehicle):
    """
    Testa sistema de failsafe.
    """
    print_separator("TESTE 6: FAILSAFE (Fase 5)")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    # 1. Configura failsafe
    print_subtitle("1. Configurando failsafe...")
    
    drone.configure_failsafe(
        heartbeat_timeout=3.0,
        max_reconnect_attempts=3,
        reconnect_interval=2.0,
        reconnect_backoff=True,
        packet_loss_warning=0.3,
        packet_loss_critical=0.5,
        auto_land_on_failsafe=False,  # Não pousar durante teste
        auto_rtl_on_failsafe=False,
        failsafe_action_delay=5.0,
        auto_reconnect=True,
        auto_arm_after_reconnect=False
    )
    print_result(True, "Configuração aplicada")
    
    # 2. Registra callbacks
    print_subtitle("2. Registrando callbacks...")
    
    def on_failsafe_activated(state):
        print(f"\n  [CALLBACK] Failsafe ativado! Estado: {state}")
    
    def on_failsafe_recovered():
        print(f"\n  [CALLBACK] Link recuperado!")
    
    def on_state_changed(old_state, new_state):
        print(f"  [CALLBACK] Mudança de estado: {old_state.name} -> {new_state.name}")
    
    def on_packet_loss(loss_rate):
        print(f"  [CALLBACK] Packet loss: {loss_rate:.1%}")
    
    def on_disconnected():
        print(f"\n  [CALLBACK] Desconectado!")
    
    def on_reconnected():
        print(f"\n  [CALLBACK] Reconectado!")
    
    drone.failsafe.on_failsafe(on_failsafe_activated)
    drone.failsafe.on_recover(on_failsafe_recovered)
    drone.failsafe.on_state_change(on_state_changed)
    drone.failsafe.on_packet_loss(on_packet_loss)
    drone.failsafe.on_disconnect(on_disconnected)
    drone.failsafe.on_reconnect(on_reconnected)
    
    print_result(True, "Callbacks registrados")
    
    # 3. Verifica estado atual
    print_subtitle("3. Estado atual do failsafe...")
    
    state = drone.get_failsafe_state()
    print(f"    Estado: {state.name}")
    print(f"    Failsafe ativo: {drone.is_failsafe_active()}")
    print(f"    Conectado: {drone.is_connected}")
    
    # 4. Estatísticas do link
    print_subtitle("4. Estatísticas do link...")
    
    stats = drone.get_link_stats()
    print_link_stats(stats)
    
    # 5. Monitoramento contínuo
    print_subtitle("5. Monitoramento contínuo (30 segundos)...")
    print("    O sistema continuará monitorando o link.")
    print("    Pressione Ctrl+C para interromper.")
    
    monitor_input = input("\n    Iniciar monitoramento? (s/N): ").lower().strip()
    
    if monitor_input == 's':
        try:
            start_time = time.time()
            count = 0
            
            while time.time() - start_time < 30:
                time.sleep(2)
                count += 1
                
                vs = drone.state.get_state()
                stats = drone.get_link_stats()
                
                alt = vs.position.relative_altitude or 0
                bat = vs.battery.voltage or 0
                loss = stats['packet_loss_rate']
                
                status = (
                    f"[{count:3d}] "
                    f"Modo: {vs.system.flight_mode:<10} "
                    f"Alt: {alt:6.1f}m "
                    f"Bat: {bat:4.1f}V "
                    f"Link: {stats['state']:<12} "
                    f"Perda: {loss}"
                )
                print(status)
                
                # Se failsafe ativar, mostra alerta
                if drone.is_failsafe_active():
                    print(f"  ALERTA: Failsafe ativo!")
                
                # A cada 10 iterações, imprime estado detalhado
                if count % 10 == 0:
                    drone.print_status()
        
        except KeyboardInterrupt:
            print("\n  Monitoramento interrompido")
    else:
        print("  Monitoramento ignorado")
    
    # 6. Estatísticas finais
    print_subtitle("6. Estatísticas finais...")
    drone.print_link_stats()
    
    return True


# ===================================================================
# TESTE 7: INTEGRAÇÃO COMPLETA
# ===================================================================

def test_full_integration(drone: Vehicle):
    """
    Teste integrado usando múltiplos subsistemas.
    """
    print_separator("TESTE 7: INTEGRAÇÃO COMPLETA")
    print("  ATENÇÃO: Este teste executa uma sequência completa!")
    print("  Inclui: modo -> arm -> takeoff -> navegação -> land")
    
    if not drone.is_connected:
        print_result(False, "Drone não conectado")
        return False
    
    full_input = input("\n  Executar teste completo? (s/N): ").lower().strip()
    if full_input != 's':
        print("  Teste ignorado")
        return True
    
    print_subtitle("Sequência: GUIDED → ARM → TAKEOFF → GOTO → RTL → LAND")
    
    # 1. Modo GUIDED
    print("\n  1. Mudando para GUIDED...")
    result = drone.set_mode("GUIDED")
    print(f"     Resultado: {result}")
    
    # 2. Arm
    print("  2. Armando...")
    result = drone.arm()
    print(f"     Resultado: {result}")
    
    if result != CommandResult.SUCCESS:
        print("     Falha ao armar, abortando")
        return False
    
    time.sleep(2)
    
    # 3. Takeoff
    print("  3. Decolando 5 metros...")
    result = drone.takeoff_guided(5.0, timeout=20.0)
    print(f"     Resultado: {result}")
    
    if result == NavResult.SUCCESS:
        time.sleep(2)
        
        # 4. Navegação
        print("  4. Navegando para waypoint...")
        
        pos = drone.state.position
        if pos.latitude and pos.longitude:
            # Move 10m norte
            result = drone.goto_relative(10.0, 0.0, 0.0, timeout=20.0)
            print(f"     Resultado: {result}")
            
            time.sleep(2)
            
            # 5. RTL
            print("  5. Retornando para launch (RTL)...")
            result = drone.set_mode("RTL")
            print(f"     Resultado: {result}")
            
            # Aguarda pouso
            print("     Aguardando pouso...")
            start = time.time()
            while time.time() - start < 60:
                if not drone.state.is_armed():
                    print("     Pouso detectado!")
                    break
                time.sleep(1)
    
    # 6. Disarm (segurança)
    if drone.state.is_armed():
        print("  6. Desarmando...")
        drone.disarm()
    
    print("\n  Sequência concluída!")
    drone.print_status()
    
    return True


# ===================================================================
# TESTE 8: STRESS TEST (OPCIONAL)
# ===================================================================

def test_stress(drone: Vehicle):
    """
    Teste de stress do sistema.
    """
    print_separator("TESTE 8: STRESS TEST")
    print("  Testa múltiplas operações em sequência rápida")
    
    stress_input = input("\n  Executar stress test? (s/N): ").lower().strip()
    if stress_input != 's':
        print("  Teste ignorado")
        return True
    
    print_subtitle("Executando operações em sequência...")
    
    # Sequência de leituras rápidas
    print("  1. Leituras rápidas de parâmetros...")
    for i in range(10):
        drone.get_param("SYSID_THISMAV", timeout=1.0)
    print("     OK")
    
    # Sequência de leituras de estado
    print("  2. Snapshots de estado...")
    for i in range(50):
        state = drone.state.get_state()
    print(f"     OK - Último modo: {state.system.flight_mode}")
    
    # Sequência de stats
    print("  3. Estatísticas...")
    for i in range(20):
        stats = drone.get_link_stats()
    print(f"     OK - Perda: {stats['packet_loss_rate']}")
    
    # Sequência de status dict
    print("  4. Status dict...")
    for i in range(20):
        status = drone.get_status_dict()
    print(f"     OK - Modo: {status['mode']}")
    
    print("\n  Stress test concluído sem erros!")
    
    return True


# ===================================================================
# RESUMO FINAL
# ===================================================================

def print_final_summary(drone: Vehicle, test_results: dict):
    """
    Imprime resumo final dos testes.
    """
    print_separator("RESUMO FINAL DOS TESTES")
    
    passed = sum(1 for v in test_results.values() if v)
    total = len(test_results)
    
    print(f"\n  Testes executados: {total}")
    print(f"  Testes passados: {passed}")
    print(f"  Testes falhos: {total - passed}")
    
    print("\n  Resultados:")
    for test_name, result in test_results.items():
        icon = "✓" if result else "✗"
        print(f"    {icon} {test_name}")
    
    print(f"\n  Estado final do veículo:")
    print(f"    Conectado: {drone.is_connected}")
    print(f"    Modo: {drone.state.flight_mode()}")
    print(f"    Armado: {drone.state.is_armed()}")
    print(f"    Parâmetros em cache: {drone.parameters.param_count()}")
    print(f"    Itens na missão local: {drone.mission_count()}")
    
    stats = drone.get_link_stats()
    print(f"    Failsafe: {stats['state']}")
    print(f"    Packet Loss: {stats['packet_loss_rate']}")
    
    print("\n" + "=" * 70)
    print("  TESTES CONCLUÍDOS")
    print("=" * 70)


# ===================================================================
# MAIN
# ===================================================================

def main():
    """
    Função principal de teste.
    """
    print("=" * 70)
    print("  PPYMAVLINK - TESTE COMPLETO DA BIBLIOTECA")
    print("  Fases 1-5: Connection, State, Commands, Parameters,")
    print("  Mission, Navigation, Failsafe")
    print("=" * 70)
    
    # String de conexão (ajuste conforme necessário)
    connection_string = "udp:127.0.0.1:14550"
    
    # Pode ser passada como argumento
    if len(sys.argv) > 1:
        connection_string = sys.argv[1]
    
    print(f"\n  Connection string: {connection_string}")
    print("  Para usar outra: python main.py <connection_string>")
    
    # Confirmação
    input("\n  Pressione ENTER para iniciar os testes...")
    
    # Resultados dos testes
    test_results = {}
    
    # ===================================================================
    # TESTE 1: Conexão e Telemetria
    # ===================================================================
    drone = test_connection_and_telemetry(connection_string)
    
    if drone is None:
        test_results["Conexão e Telemetria"] = False
        print("\n Teste 1 falhou - verifique a conexão com o veículo/SITL")
        print_final_summary(None, test_results)
        return
    else:
        test_results["Conexão e Telemetria"] = True
    
    try:
        # ===================================================================
        # TESTE 2: Comandos Básicos
        # ===================================================================
        result = test_basic_commands(drone)
        test_results["Comandos Básicos"] = result
        
        # ===================================================================
        # TESTE 3: Parâmetros
        # ===================================================================
        result = test_parameters(drone)
        test_results["Parâmetros"] = result
        
        # ===================================================================
        # TESTE 4: Missões
        # ===================================================================
        result = test_mission(drone)
        test_results["Missões"] = result
        
        # ===================================================================
        # TESTE 5: Navegação (OPCIONAL - move o veículo)
        # ===================================================================
        if drone.state.is_armed():
            print("\n  Veículo armado - pulando teste de navegação por segurança")
            print("    Desarme o veículo antes de executar este teste")
            test_results["Navegação"] = False
        else:
            result = test_navigation(drone)
            test_results["Navegação"] = result
        
        # ===================================================================
        # TESTE 6: Failsafe
        # ===================================================================
        result = test_failsafe(drone)
        test_results["Failsafe"] = result
        
        # ===================================================================
        # TESTE 7: Integração (OPCIONAL - move o veículo)
        # ===================================================================
        if drone.state.is_armed():
            print("\n  Veículo armado - pulando teste de integração por segurança")
            test_results["Integração"] = False
        else:
            result = test_full_integration(drone)
            test_results["Integração"] = result
        
        # ===================================================================
        # TESTE 8: Stress Test (OPCIONAL)
        # ===================================================================
        result = test_stress(drone)
        test_results["Stress Test"] = result
        
    except KeyboardInterrupt:
        print("\n\n  Testes interrompidos pelo usuário")
        test_results["Interrupção"] = False
    
    except Exception as e:
        print(f"\n\n Erro durante os testes: {e}")
        import traceback
        traceback.print_exc()
        test_results["Erro"] = False
    
    finally:
        # ===================================================================
        # RESUMO FINAL
        # ===================================================================
        print_final_summary(drone, test_results)
        
        # ===================================================================
        # LIMPEZA
        # ===================================================================
        print("\n  Desconectando...")
        drone.disconnect()
        print("  Biblioteca encerrada com sucesso!")


if __name__ == "__main__":
    main()