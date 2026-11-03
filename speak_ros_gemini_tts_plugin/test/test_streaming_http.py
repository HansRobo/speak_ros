# Copyright 2026 HansRobo
# SPDX-License-Identifier: Apache-2.0
"""Exercise Gemini SSE audio against a local server with dummy credentials."""
import base64
import contextlib
import http.server
import json
import socket
import subprocess
import sys
import threading
import time
import unittest

CLIENT = sys.argv.pop(1)
PCM = b'\x34\x12' * 6000


def event(kind, **fields):
    return {'event_type': kind, **fields}


def audio(data, **metadata):
    return {'type': 'audio', 'data': base64.b64encode(data).decode(), **metadata}


def sse(value, crlf=False, multiline=False):
    ending = '\r\n' if crlf else '\n'
    payload = json.dumps(value, indent=2) if multiline else json.dumps(value)
    lines = [f'event: {value["event_type"]}']
    lines.extend('data: ' + line for line in payload.splitlines())
    return (ending.join(lines) + ending * 2).encode()


COMPLETE = event('interaction.completed', interaction={'status': 'completed'})


def response_events(scenario):
    prefix = sse(event('interaction.created', interaction={'status': 'in_progress'}))
    first = audio(PCM[:6001], mime_type='audio/l16;rate=24000', sample_rate=24000, channels=1)
    if scenario == 'metadata_start':
        prefix += sse(event('step.start', step={'type': 'model_output'}))
    if scenario == 'failed_start':
        return sse(event('step.start', step={'type': 'model_output', 'error': {'code': 13}})), sse(COMPLETE)
    if scenario == 'uri_audio':
        first.pop('data')
        first['uri'] = 'https://example.invalid/audio'
    if scenario == 'empty':
        return prefix, sse(COMPLETE)
    if scenario == 'bad_json':
        return b'data: {broken}\n\n', b''
    if scenario == 'bad_event_type':
        return b'event: step.start\ndata: {"event_type":"step.stop"}\n\n', b''
    if scenario == 'oversize':
        return b'data: ' + b'x' * (4 * 1024 * 1024) + b'\n\n', b''
    if scenario.startswith('base64_'):
        first['data'] = {'base64_character': 'AA?=', 'base64_padding': 'A===',
                         'base64_length': 'A', 'base64_bits': 'AB=='}[scenario]
    if scenario == 'fractional_rate':
        first['sample_rate'] = 24000.5
    if scenario in ('bad_rate', 'bad_channels', 'bad_mime', 'bad_codec'):
        first.update({'bad_rate': {'sample_rate': 16000}, 'bad_channels': {'channels': 2},
                      'bad_mime': {'mime_type': 'audio/wav'},
                      'bad_codec': {'mime_type': 'audio/l16;codec=opus'}}[scenario])
    if scenario == 'odd':
        first = audio(PCM[:-1])
        remaining = b''
    else:
        remaining = sse(event('step.delta', delta=audio(PCM[6001:])))
    start = event('step.start', step={'type': 'model_output', 'content': [first]})
    prefix += sse(start, crlf=scenario == 'fragmented', multiline=scenario == 'fragmented')
    prefix += b': keepalive\n' + b':' + b' ' * 5000 + b'\n\n'
    if scenario == 'missing_completion':
        return prefix, remaining
    if scenario == 'partial_event':
        return prefix, remaining + b'data: {"event_type":"interaction.completed"}'
    if scenario == 'failed':
        return prefix, remaining + sse(event('interaction.failed'))
    if scenario == 'failed_status':
        return prefix, remaining + sse(event('interaction.status_update', status='failed'))
    if scenario == 'incomplete':
        return prefix, remaining + sse(event('interaction.completed', interaction={'status': 'incomplete'}))
    if scenario == 'error':
        return prefix, sse(event('error', error={'message': 'mock failure'}))
    if scenario == 'audio_after_completion':
        return prefix, sse(COMPLETE) + remaining
    if scenario == 'duplicate_completion':
        return prefix, remaining + sse(COMPLETE) + sse(COMPLETE)
    return prefix, remaining + sse(COMPLETE)


@contextlib.contextmanager
def endpoint(scenario):
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def do_POST(self):
            request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            requests.append((self.path, dict(self.headers), request))
            try:
                if scenario == 'headers_wait':
                    time.sleep(3)
                status = 302 if scenario == 'redirect' else 429 if scenario == 'http_error' else 200
                self.send_response(status)
                content_type = ('text/event-streaming' if scenario == 'bad_content_type' else
                                'Text/Event-Stream; charset=UTF-8' if scenario == 'content_type_case' else
                                'text/event-stream')
                self.send_header('Content-Type', content_type)
                if scenario == 'redirect':
                    self.send_header('Location', f'http://127.0.0.1:{self.server.server_port}/redirected')
                first, last = response_events(scenario)
                chunked = scenario != 'content_length'
                if chunked:
                    self.send_header('Transfer-Encoding', 'chunked')
                else:
                    self.send_header('Content-Length', str(len(first) + len(last)))
                self.end_headers()
                if scenario == 'body_wait':
                    time.sleep(3)
                    return

                def send(data):
                    if not data:
                        return
                    if chunked:
                        self.wfile.write(f'{len(data):x}\r\n'.encode() + data + b'\r\n')
                    else:
                        self.wfile.write(data)
                    self.wfile.flush()

                if scenario == 'fragmented':
                    for offset in range(0, len(first), 13):
                        send(first[offset:offset + 13])
                else:
                    send(first)
                if scenario in ('early', 'abort'):
                    time.sleep(1.2)
                if scenario == 'truncated':
                    self.close_connection = True
                    self.connection.shutdown(socket.SHUT_WR)
                    return
                send(last)
                if chunked:
                    self.wfile.write(b'0\r\n\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port, requests
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


class StreamingHttpTest(unittest.TestCase):
    def run_case(self, scenario, mode='success'):
        with endpoint(scenario) as (port, requests):
            result = subprocess.run([CLIENT, str(port), mode], capture_output=True,
                                    text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(len(requests), 1)
            path, headers, body = requests[0]
            self.assertEqual(path, '/v1beta/interactions')
            self.assertEqual(headers.get('x-goog-api-key'), 'mock-key')
            self.assertEqual(headers.get('Accept'), 'text/event-stream')
            self.assertEqual(body, {
                'model': 'gemini-3.8-flash-tts',
                'input': [{'type': 'user_input', 'content': [{
                    'type': 'text', 'text': 'streaming test',
                    'annotations': [{'type': 'speech_metadata', 'style': 'calm'}]}]}],
                'response_format': {'type': 'audio', 'mime_type': 'audio/l16', 'sample_rate': 24000},
                'generation_config': {'speech_config': [{'voice': 'Kore'}]},
                'stream': True, 'store': False})

    def test_delivery_before_completion(self):
        self.run_case('early', 'early')

    def test_fragmented_multiline_crlf_and_content_length(self):
        for scenario in ('normal', 'fragmented', 'content_length', 'metadata_start', 'content_type_case'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario)

    def test_cancel_interrupts_header_and_body_waits(self):
        for scenario in ('headers_wait', 'body_wait'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'cancel')

    def test_callback_false_interrupts_body(self):
        self.run_case('abort', 'abort')

    def test_malformed_events_and_audio(self):
        for scenario in ('bad_json', 'bad_event_type', 'base64_character', 'base64_padding',
                         'base64_length', 'base64_bits', 'bad_rate', 'bad_channels',
                         'bad_mime', 'bad_codec', 'odd', 'oversize', 'uri_audio', 'fractional_rate'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'error')

    def test_protocol_failures_do_not_complete_or_retry(self):
        for scenario in ('missing_completion', 'partial_event', 'failed', 'failed_status',
                         'incomplete', 'error', 'audio_after_completion', 'duplicate_completion',
                         'truncated', 'http_error', 'redirect', 'failed_start', 'bad_content_type'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'error')

    def test_invalid_endpoint_origins(self):
        result = subprocess.run([CLIENT, '0', 'invalid_url'], capture_output=True,
                                text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_empty_audio_completes(self):
        self.run_case('empty', 'empty')


if __name__ == '__main__':
    unittest.main()
