Materiais

--------pixhawk 2.4.8--------
Controladora de voo antiga, vai ser utilizada
para inicio do projeto.

Como software para controladora de voo sera utilizado o
ArduPilot em conjunto com o Mission Planner

--------Raspberry Pi 5--------

--------GPS M8N--------

--------SITL--------
Foi usado o ArduPilot para fazer um software in the loop
para entender essa etapa seguir o tutorial em https://ardupilot.org/dev/docs/building-setup-linux.html com isso podemos instalar o ArduPilot e criar um setup que 
simule qualquer placa.

para iniciar a simulação ir até a pasta do ardupilot e digitar:

./Tools/autotest/sim_vehicle.py -v ArduCopter --map --console

Com o ambiente pronto, para inicialo é só abrir um terminal na pasta do ardupilot e digitar "cd ArduCopter" e depois "sim_vehicle.py -v ArduCopter --console --map" para iniciar o SITL o console e o mapa respectivamente


--------Estrutura da Biblioteca--------
# Fase 1 — Base da comunicação

1. `connection.py`

   * connect
   * disconnect
   * recv
   * send
   * wait_heartbeat

2. `reader.py`

   * loop contínuo
   * thread de leitura
   * recv_match()

3. `dispatcher.py`

   * register()
   * unregister()
   * dispatch()
   * callbacks por mensagem

---

# Fase 2 — Estado interno do veículo

4. `state.py`

   * attitude
   * gps
   * battery
   * mode
   * armed

5. `telemetry.py`

   * handlers automáticos
   * atualização do state cache
   * parsing de mensagens

6. `heartbeat.py`

   * enviar heartbeat GCS
   * monitorar timeout
   * detectar perda de link

---

# Fase 3 — Controle do veículo

7. `commands.py`

   * arm
   * disarm
   * takeoff
   * land
   * reboot
   * set_mode

8. `transaction.py`

   * ACK handling
   * retries
   * timeout
   * request/response matching

9. `vehicle.py`

   * API de alto nível
   * integração geral

---

# Fase 4 — Recursos MAVLink avançados

10. `parameters.py`

* get_param
* set_param
* download_all

11. `mission.py`

* upload mission
* download mission
* clear mission

12. `navigation.py`

* goto
* velocity control
* yaw control

---

# Fase 5 — Robustez

13. `failsafe.py`

* reconnect
* heartbeat timeout
* packet loss

14. `queue.py`

* filas assíncronas
* buffers

15. `logging.py`

* logs MAVLink
* debug
* telemetry recording

---

# Fase 6 — Recursos avançados futuros

16. MAVLink 2
17. Signing
18. Multi-vehicle
19. Swarm
20. ROS2 bridge
21. Video streaming
22. Companion computer integration
23. AI/autonomy layer

---