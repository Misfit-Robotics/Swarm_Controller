import time
import threading
from flask import Flask, jsonify, render_template, request
import rclpy
from swarm_control_pkg.swarm_core import SwarmControllerCore

_FLASK_PORT = 5000
_FLASK_HOST = '0.0.0.0'

app = Flask(__name__, template_folder='templates')
_core: SwarmControllerCore | None = None


def init_app(core: SwarmControllerCore) -> None:
    global _core
    _core = core


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok'})


@app.route('/fleet', methods=['GET'])
def fleet_status():
    if _core is None:
        return jsonify({'error': 'core not initialized'}), 500

    snapshot = _core.get_fleet_snapshot()
    now = time.monotonic()

    fleet = {}
    for agent_id, entry in snapshot.items():
        fleet[agent_id] = {
            'online': entry['online'],
            'age_s': now - entry['last_seen'],
            'lat': entry['data'].get('lat', 0),
            'lon': entry['data'].get('lon', 0),
            'heading': entry['data'].get('heading', 0),
            'velocity': entry['data'].get('velocity', 0),
            'fuel_range': entry['data'].get('fuel_range', 0),
            'bingo': entry['data'].get('bingo', 0),
        }

    return jsonify({
        'timestamp': time.time(),
        'fleet': fleet,
        'drone_count': len(fleet),
        'online_count': sum(1 for e in fleet.values() if e['online']),
    })


@app.route('/fleet/<agent_id>', methods=['GET'])
def drone_status(agent_id: str):
    if _core is None:
        return jsonify({'error': 'core not initialized'}), 500

    snapshot = _core.get_fleet_snapshot()
    if agent_id not in snapshot:
        return jsonify({'error': f'drone {agent_id} not found'}), 404

    entry = snapshot[agent_id]
    now = time.monotonic()
    return jsonify({
        'agent_id': agent_id,
        'online': entry['online'],
        'age_s': now - entry['last_seen'],
        'data': entry['data'],
    })


@app.route('/task', methods=['POST'])
def send_task():
    if _core is None:
        return jsonify({'error': 'core not initialized'}), 500

    body = request.get_json(force=True, silent=True)
    if not body:
        return jsonify({'error': 'invalid JSON body'}), 400

    nai_name = body.get('nai_name', 'Unknown NAI')
    priority = body.get('priority', 'Medium')
    coordinates = body.get('coordinates', [])

    if not coordinates:
        return jsonify({'error': 'coordinates are required'}), 400

    task_id = _core.publish_task(nai_name, priority, coordinates)
    return jsonify({'status': 'sent', 'task_id': task_id}), 200


def main(args=None):
    rclpy.init(args=args)
    core = SwarmControllerCore()
    init_app(core)

    spin_thread = threading.Thread(target=rclpy.spin, args=(core,), daemon=True)
    spin_thread.start()

    time.sleep(0.5)

    print(f'\n=== Swarm Controller Web Dashboard ===')
    print(f'Open your browser: http://localhost:{_FLASK_PORT}')
    print(f'REST API:')
    print(f'  GET /health        — health check')
    print(f'  GET /fleet         — all drones')
    print(f'  GET /fleet/<id>    — single drone\n')

    try:
        app.run(host=_FLASK_HOST, port=_FLASK_PORT, debug=False, threaded=True)
    finally:
        core.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
