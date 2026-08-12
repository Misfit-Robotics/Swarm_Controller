import time
import threading
import rclpy
from swarm_control_pkg.swarm_core import SwarmControllerCore


def print_fleet_status(core: SwarmControllerCore) -> None:
    """Print current fleet status."""
    fleet = core.get_fleet_snapshot()
    if not fleet:
        print('\nFleet: no drones known\n')
        return
    print('\n--- Fleet Status ---')
    now = time.monotonic()
    for agent_id in sorted(fleet.keys()):
        entry = fleet[agent_id]
        d = entry['data']
        state = 'ONLINE' if entry['online'] else 'OFFLINE'
        age = now - entry['last_seen']
        print(f"  {agent_id}: {state} | "
              f"lat={d.get('lat', 0):.6f} lon={d.get('lon', 0):.6f} | "
              f"hdg={d.get('heading', 0):.1f}° vel={d.get('velocity', 0):.1f} mph | "
              f"age {age:.1f}s")
    print()


def cli_loop(core: SwarmControllerCore) -> None:
    """Interactive CLI loop."""
    print('\n=== Swarm Controller CLI ===')
    print('Commands: status, help, exit\n')
    while True:
        try:
            cmd = input('> ').strip().lower()
            if cmd in ('exit', 'quit', 'q'):
                print('Exiting...')
                break
            elif cmd in ('status', 's'):
                print_fleet_status(core)
            elif cmd in ('help', 'h', '?'):
                print('  status (s) — print fleet status')
                print('  help (h)   — show this help')
                print('  exit (q)   — quit')
            elif cmd:
                print(f'Unknown command: {cmd}')
        except KeyboardInterrupt:
            print('\nExiting...')
            break
        except EOFError:
            break


def main(args=None):
    rclpy.init(args=args)
    core = SwarmControllerCore()
    
    # Spin ROS2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(core,), daemon=True)
    spin_thread.start()
    
    # Give ROS2 a moment to initialize
    time.sleep(0.5)
    
    try:
        cli_loop(core)
    finally:
        core.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
