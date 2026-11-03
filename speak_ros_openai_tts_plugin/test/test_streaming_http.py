# Copyright 2026 HansRobo
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real OpenAI plugin without credentials or external requests."""
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


@contextlib.contextmanager
def endpoint(scenario):
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def do_POST(self):
            request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            requests.append((self.path, self.headers.get('Authorization'), request))
            try:
                if scenario == 'headers_wait':
                    time.sleep(3)
                status = 302 if scenario == 'redirect' else 429 if scenario == 'http_error' else 200
                self.send_response(status)
                if scenario == 'redirect':
                    self.send_header('Location', f'http://127.0.0.1:{self.server.server_port}/redirected')
                data = b'' if scenario == 'empty' else PCM
                if scenario == 'odd':
                    data = data[:-1]
                chunked = scenario in ('chunked', 'chunked_broken')
                self.send_header('Content-Type', 'application/octet-stream')
                if chunked:
                    self.send_header('Transfer-Encoding', 'chunked')
                else:
                    self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                if scenario == 'body_wait':
                    time.sleep(3)
                    return
                if scenario == 'http_error':
                    self.wfile.write(data)
                elif scenario in ('early', 'abort'):
                    self.wfile.write(data[:8193])
                    self.wfile.flush()
                    time.sleep(1.2)
                    self.wfile.write(data[8193:])
                elif scenario == 'truncated':
                    self.wfile.write(data[:-10])
                    self.wfile.flush()
                    self.close_connection = True
                    self.connection.shutdown(socket.SHUT_WR)
                elif chunked:
                    for part in (data[:1], data[1:4097], data[4097:]):
                        self.wfile.write(f'{len(part):x}\r\n'.encode() + part + b'\r\n')
                        self.wfile.flush()
                    if scenario == 'chunked_broken':
                        self.close_connection = True
                        self.connection.shutdown(socket.SHUT_WR)
                    else:
                        self.wfile.write(b'0\r\n\r\n')
                else:
                    self.wfile.write(data)
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
            path, auth, body = requests[0]
            self.assertEqual(path, '/v1/audio/speech')
            self.assertEqual(auth, 'Bearer mock-key')
            self.assertEqual(body, {'model': 'tts-1', 'input': 'streaming test',
                                   'voice': 'nova', 'response_format': 'pcm',
                                   'speed': 1.0, 'instructions': 'calm'})

    def test_delivery_before_response_completion(self):
        self.run_case('early', 'early')

    def test_content_length_and_chunked_frames(self):
        for scenario in ('normal', 'chunked'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario)

    def test_cancel_interrupts_header_and_body_waits(self):
        for scenario in ('headers_wait', 'body_wait'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'cancel')

    def test_callback_false_interrupts_body(self):
        self.run_case('abort', 'abort')

    def test_errors_do_not_emit_completion_or_retry(self):
        for scenario in ('http_error', 'redirect', 'odd', 'truncated', 'chunked_broken'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'error')

    def test_invalid_endpoint_origins_are_rejected(self):
        result = subprocess.run([CLIENT, '0', 'invalid_url'], capture_output=True,
                                text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_empty_pcm_completes(self):
        self.run_case('empty', 'empty')


if __name__ == '__main__':
    unittest.main()
