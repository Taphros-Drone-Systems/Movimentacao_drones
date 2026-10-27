from collections import defaultdict
from threading import RLock
from typing import Callable, Dict, List, Optional, Any


class MAVLinkDispatcher:
    """
    Event Bus central da aplicação.

    Suporta:
    - mensagens MAVLink
    - eventos internos do sistema
    - callbacks "once" (executam apenas uma vez)
    - wildcard "*" para todos os eventos
    - estatísticas de eventos
    """

    def __init__(self):

        self._handlers: Dict[str, List[Callable]] = defaultdict(list)
        self._once_handlers: Dict[str, List[Callable]] = defaultdict(list)
        self._lock = RLock()
        
        # Estatísticas
        self._handler_stats: Dict[str, int] = defaultdict(int)
        self._total_events: int = 0
        self._start_time: Optional[float] = None

    # =====================================================
    # REGISTER
    # =====================================================

    def register(self, event_name: str, callback: Callable):
        """
        Registra um callback para um evento específico.
        
        Args:
            event_name: Nome do evento (use "*" para todos os eventos)
            callback: Função a ser chamada
        """
        with self._lock:

            if callback not in self._handlers[event_name]:
                self._handlers[event_name].append(callback)

    def once(self, event_name: str, callback: Callable):
        """
        Registra um callback que será executado apenas uma vez.
        
        Args:
            event_name: Nome do evento
            callback: Função a ser chamada (será removida após execução)
        """
        with self._lock:

            if callback not in self._once_handlers[event_name]:
                self._once_handlers[event_name].append(callback)

    # =====================================================
    # UNREGISTER
    # =====================================================

    def unregister(self, event_name: str, callback: Callable):
        """
        Remove um callback registrado.
        
        Args:
            event_name: Nome do evento
            callback: Função a ser removida
        """
        with self._lock:

            if callback in self._handlers[event_name]:
                self._handlers[event_name].remove(callback)

            if callback in self._once_handlers[event_name]:
                self._once_handlers[event_name].remove(callback)

            if not self._handlers[event_name]:
                del self._handlers[event_name]

    def unregister_all(self, event_name: Optional[str] = None):
        """
        Remove todos os callbacks de um evento ou todos os eventos.
        
        Args:
            event_name: Nome do evento (None = remove todos)
        """
        with self._lock:
            if event_name is None:
                self._handlers.clear()
                self._once_handlers.clear()
            else:
                self._handlers.pop(event_name, None)
                self._once_handlers.pop(event_name, None)

    # =====================================================
    # DISPATCH
    # =====================================================

    def dispatch(self, event_name: str, *args, **kwargs):
        """
        Dispara um evento para todos os callbacks registrados.
        
        Args:
            event_name: Nome do evento
            *args, **kwargs: Argumentos passados para os callbacks
        """
        # Atualiza estatísticas
        with self._lock:
            self._handler_stats[event_name] += 1
            self._total_events += 1
            if self._start_time is None:
                self._start_time = __import__('time').time()
            
            # Copia handlers para evitar problemas de concorrência
            handlers = list(self._handlers.get(event_name, []))
            once_handlers = list(self._once_handlers.get(event_name, []))
            wildcard_handlers = list(self._handlers.get("*", []))
            
            # Remove handlers "once" após copiar
            if once_handlers:
                del self._once_handlers[event_name]

        # Executa handlers específicos primeiro
        for handler in handlers:
            try:
                handler(*args, **kwargs)
            except Exception as e:
                print(f"[DISPATCH ERROR] {event_name} -> {handler.__name__}: {e}")

        # Executa handlers "once"
        for handler in once_handlers:
            try:
                handler(*args, **kwargs)
            except Exception as e:
                print(f"[DISPATCH ERROR] {event_name} (once) -> {handler.__name__}: {e}")

        # Executa handlers wildcard (para todos os eventos)
        for handler in wildcard_handlers:
            try:
                # Passa o nome do evento como primeiro argumento para wildcard
                handler(event_name, *args, **kwargs)
            except TypeError:
                # Se o handler não aceitar event_name, tenta sem
                try:
                    handler(*args, **kwargs)
                except Exception as e:
                    print(f"[DISPATCH ERROR] * -> {handler.__name__}: {e}")
            except Exception as e:
                print(f"[DISPATCH ERROR] * -> {handler.__name__}: {e}")

    # =====================================================
    # STATISTICS
    # =====================================================

    def get_stats(self) -> dict:
        """
        Retorna estatísticas do dispatcher.
        
        Returns:
            Dict com estatísticas de eventos
        """
        with self._lock:
            return {
                'total_events': self._total_events,
                'events_by_type': dict(self._handler_stats),
                'active_handlers': {
                    name: len(handlers) 
                    for name, handlers in self._handlers.items()
                },
                'once_handlers': {
                    name: len(handlers) 
                    for name, handlers in self._once_handlers.items()
                },
                'total_handlers': sum(len(h) for h in self._handlers.values()),
                'total_once_handlers': sum(len(h) for h in self._once_handlers.values()),
                'uptime_seconds': (__import__('time').time() - self._start_time) if self._start_time else 0
            }

    def print_stats(self):
        """
        Imprime estatísticas do dispatcher no console.
        """
        stats = self.get_stats()
        
        print("\n" + "=" * 50)
        print("DISPATCHER STATISTICS")
        print("=" * 50)
        print(f"Total de eventos: {stats['total_events']}")
        print(f"Uptime: {stats['uptime_seconds']:.1f}s")
        print(f"Handlers ativos: {stats['total_handlers']}")
        print(f"Handlers once: {stats['total_once_handlers']}")
        
        print("\nEventos por tipo (top 10):")
        events_sorted = sorted(
            stats['events_by_type'].items(), 
            key=lambda x: x[1], 
            reverse=True
        )[:10]
        for event_name, count in events_sorted:
            print(f"  {event_name}: {count}")
        
        print("=" * 50)

    def reset_stats(self):
        """
        Reseta estatísticas do dispatcher.
        """
        with self._lock:
            self._handler_stats.clear()
            self._total_events = 0
            self._start_time = None

    # =====================================================
    # UTILITY METHODS
    # =====================================================

    def has_handler(self, event_name: str, callback: Optional[Callable] = None) -> bool:
        """
        Verifica se existe handler para um evento.
        
        Args:
            event_name: Nome do evento
            callback: Se fornecido, verifica se este callback específico está registrado
        
        Returns:
            bool indicando se o handler existe
        """
        with self._lock:
            if callback is None:
                return event_name in self._handlers or event_name in self._once_handlers
            
            return (callback in self._handlers.get(event_name, []) or 
                   callback in self._once_handlers.get(event_name, []))

    def get_handler_count(self, event_name: str) -> int:
        """
        Retorna número de handlers para um evento.
        
        Args:
            event_name: Nome do evento
        
        Returns:
            Número de handlers
        """
        with self._lock:
            return (len(self._handlers.get(event_name, [])) + 
                   len(self._once_handlers.get(event_name, [])))

    def list_events(self) -> List[str]:
        """
        Lista todos os eventos com handlers registrados.
        
        Returns:
            Lista de nomes de eventos
        """
        with self._lock:
            events = set(self._handlers.keys()) | set(self._once_handlers.keys())
            return sorted(events)

    # =====================================================
    # CLEANUP
    # =====================================================

    def clear(self):
        """
        Limpa todos os handlers e estatísticas.
        """
        self.unregister_all()
        self.reset_stats()