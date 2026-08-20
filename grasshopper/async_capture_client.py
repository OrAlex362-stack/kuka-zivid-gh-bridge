"""Asynchronous Grasshopper client for POST /capture.

GhPython inputs:
    base_url  e.g. http://127.0.0.1:8765
    capture   button/toggle; rising edge starts one background request
    reset     optional bool; clears displayed state on rising edge

GhPython outputs:
    busy, ok, errorCode, httpStatus, response, details, previewPath
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
import uuid

try:
    import scriptcontext as sc
except Exception:
    sc = None


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


def _post_json(url, body):
    data = json.dumps(body or {}).encode('utf-8')
    request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'}, method='POST')
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
    return {
        'busy': False,
        'ok': False,
        'errorCode': None,
        'httpStatus': None,
        'response': None,
        'details': None,
        'previewPath': None,
        'thread': None,
        'previous_capture': False,
        'previous_reset': False,
    }


def _sticky_key():
    if 'ghenv' in globals():
        return 'kuka_zivid_async_capture_' + str(ghenv.Component.InstanceGuid)
    return 'kuka_zivid_async_capture_standalone'


def _worker(state, url, request_id):
    try:
        status, payload = _post_json(url.rstrip('/') + '/capture', {'request_id': request_id})
        state['httpStatus'] = status
        state['response'] = payload
        state['details'] = payload.get('details') if isinstance(payload, dict) else None
        state['ok'] = bool(isinstance(payload, dict) and payload.get('ok') is True and status < 400)
        state['errorCode'] = None if state['ok'] else parse_error(payload)
        files = payload.get('files') if isinstance(payload, dict) else None
        state['previewPath'] = None if not isinstance(files, dict) else (files.get('preview_xyzrgb') or files.get('preview_xyz'))
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

capture_now = bool(globals().get('capture', False))
reset_now = bool(globals().get('reset', False))

if reset_now and not state.get('previous_reset', False) and not state.get('busy', False):
    preserved = {'previous_capture': state.get('previous_capture', False), 'previous_reset': True}
    state.clear()
    state.update(_initial_state())
    state.update(preserved)

if capture_now and not state.get('previous_capture', False) and not state.get('busy', False):
    state['busy'] = True
    state['ok'] = False
    state['errorCode'] = None
    state['httpStatus'] = None
    state['response'] = None
    state['details'] = None
    state['previewPath'] = None
    request_id = 'gh-capture-' + str(uuid.uuid4())
    thread = threading.Thread(target=_worker, args=(state, globals().get('base_url', 'http://127.0.0.1:8765'), request_id))
    thread.daemon = True
    state['thread'] = thread
    thread.start()

state['previous_capture'] = capture_now
state['previous_reset'] = reset_now

busy = bool(state.get('busy'))
ok = bool(state.get('ok'))
errorCode = state.get('errorCode')
httpStatus = state.get('httpStatus')
response = state.get('response')
details = state.get('details')
previewPath = state.get('previewPath')