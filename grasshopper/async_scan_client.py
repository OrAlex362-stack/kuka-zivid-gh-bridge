"""Asynchronous Grasshopper client for /scan/* commands.

GhPython inputs:
    base_url       e.g. http://127.0.0.1:8765
    start          rising edge -> POST /scan/start
    capture        rising edge -> POST /scan/capture
    merge          rising edge -> POST /scan/merge
    finish         rising edge -> POST /scan/finish
    reset          rising edge -> POST /scan/reset
    clear          optional rising edge clears displayed state while idle

GhPython outputs:
    busy, ok, action, errorCode, httpStatus, response, details, previewPath, captureCount
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

try:
    import scriptcontext as sc
except Exception:
    sc = None

_ACTIONS = [
    ('start', '/scan/start'),
    ('capture', '/scan/capture'),
    ('merge', '/scan/merge'),
    ('finish', '/scan/finish'),
    ('reset', '/scan/reset'),
]


def parse_error(payload):
    if not isinstance(payload, dict):
        return 'UNKNOWN_ERROR'
    for key in ('error_code', 'error', 'code'):
        value = payload.get(key)
        if value:
            return str(value)
    details = payload.get('details')
    if isinstance(details, dict):
        stationary = details.get('stationary_during_capture')
        if isinstance(stationary, dict) and stationary.get('reason'):
            return str(stationary.get('reason'))
    return 'UNKNOWN_ERROR'


def _post_json(url):
    request = urllib.request.Request(url, data=b'{}', headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(request, timeout=None) as handle:
            raw = handle.read().decode('utf-8')
            payload = json.loads(raw) if raw else {}
            return int(handle.getcode()), payload
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode('utf-8')
        try:
            payload = json.loads(raw) if raw else {}
        except Exception:
            payload = {'error': raw}
        return int(exc.code), payload


def _initial_state():
    state = {
        'busy': False,
        'ok': False,
        'action': None,
        'errorCode': None,
        'httpStatus': None,
        'response': None,
        'details': None,
        'previewPath': None,
        'captureCount': 0,
        'thread': None,
        'previous_clear': False,
    }
    for name, _endpoint in _ACTIONS:
        state['previous_' + name] = False
    return state


def _sticky_key():
    if 'ghenv' in globals():
        return 'kuka_zivid_async_scan_' + str(ghenv.Component.InstanceGuid)
    return 'kuka_zivid_async_scan_standalone'


def _preview_from_payload(payload):
    if not isinstance(payload, dict):
        return None
    files = payload.get('files')
    if not isinstance(files, dict):
        return None
    return files.get('preview_xyzrgb') or files.get('preview_xyz')


def _worker(state, base_url, action_name, endpoint):
    try:
        status, payload = _post_json(base_url.rstrip('/') + endpoint)
        state['httpStatus'] = status
        state['response'] = payload
        state['details'] = payload.get('details') if isinstance(payload, dict) else None
        state['ok'] = bool(isinstance(payload, dict) and payload.get('ok') is True and status < 400)
        state['errorCode'] = None if state['ok'] else parse_error(payload)
        state['previewPath'] = _preview_from_payload(payload)
        if isinstance(payload, dict) and payload.get('capture_count') is not None:
            state['captureCount'] = int(payload.get('capture_count'))
    except Exception as exc:
        state['httpStatus'] = None
        state['response'] = None
        state['details'] = {'cause': str(exc)}
        state['ok'] = False
        state['errorCode'] = 'HTTP_WORKER_FAILED'
        state['previewPath'] = None
    finally:
        state['busy'] = False
        state['thread'] = None


state = _initial_state()
if sc is not None:
    key = _sticky_key()
    state = sc.sticky.get(key) or _initial_state()
    sc.sticky[key] = state

clear_now = bool(globals().get('clear', False))
if clear_now and not state.get('previous_clear', False) and not state.get('busy', False):
    preserved = {k: state.get(k, False) for k in state if k.startswith('previous_')}
    state.clear()
    state.update(_initial_state())
    state.update(preserved)

if not state.get('busy', False):
    selected = None
    for name, endpoint in _ACTIONS:
        now = bool(globals().get(name, False))
        if now and not state.get('previous_' + name, False):
            selected = (name, endpoint)
            break
    if selected is not None:
        name, endpoint = selected
        state['busy'] = True
        state['ok'] = False
        state['action'] = name
        state['errorCode'] = None
        state['httpStatus'] = None
        state['response'] = None
        state['details'] = None
        state['previewPath'] = None
        thread = threading.Thread(target=_worker, args=(state, globals().get('base_url', 'http://127.0.0.1:8765'), name, endpoint))
        thread.daemon = True
        state['thread'] = thread
        thread.start()

for name, _endpoint in _ACTIONS:
    state['previous_' + name] = bool(globals().get(name, False))
state['previous_clear'] = clear_now

busy = bool(state.get('busy'))
ok = bool(state.get('ok'))
action = state.get('action')
errorCode = state.get('errorCode')
httpStatus = state.get('httpStatus')
response = state.get('response')
details = state.get('details')
previewPath = state.get('previewPath')
captureCount = int(state.get('captureCount') or 0)