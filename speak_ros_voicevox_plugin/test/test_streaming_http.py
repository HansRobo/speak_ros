# Copyright 2026 HansRobo
# SPDX-License-Identifier: Apache-2.0
"""Run the real C++ plugin against a controllably delayed HTTP engine."""
import contextlib
import http.server
import io
import json
import socket
import struct
import subprocess
import sys
import threading
import time
import unittest
import urllib.parse
import wave

CLIENT = sys.argv.pop(1)
PCM = b'\x34\x12' * 6000
out = io.BytesIO()
with wave.open(out, 'wb') as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(16000)
    wav.writeframes(PCM)
WAV = out.getvalue()


@contextlib.contextmanager
def engine(scenario):
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def reply(self, data, status=200, content_type='application/json'):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            self.wfile.flush()

        def do_GET(self):
            requests.append(self.path)
            if scenario == 'manifest404':
                self.reply(b'{}', 404)
            elif scenario == 'manifest500':
                self.reply(b'{}', 500)
            elif scenario == 'manifest_invalid':
                self.reply(b'{broken')
            elif scenario == 'manifest_invalid_root':
                self.reply(b'[]')
            elif scenario == 'manifest_invalid_features':
                self.reply(b'{"supported_features": []}')
            else:
                features = {}
                if scenario != 'missing':
                    features['streaming_synthesis'] = {
                        'type': 'bool', 'value': scenario != 'disabled'}
                self.reply(json.dumps({'supported_features': features}).encode())

        def do_POST(self):
            route = urllib.parse.urlsplit(self.path)
            requests.append(route.path)
            body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
            if route.path == '/audio_query':
                self.reply(b'{}')
                return
            query = json.loads(body)
            assert query['outputSamplingRate'] == 16000
            assert query['outputStereo'] is False
            if route.path == '/streaming_synthesis':
                assert urllib.parse.parse_qs(route.query)['segment_length'] == ['0.3']
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'audio/wav')
                self.send_header('Content-Length', str(len(WAV)))
                self.end_headers()
                if scenario == 'cancel':
                    time.sleep(3)
                    return
                if scenario == 'truncated':
                    self.wfile.write(WAV[:-10])
                    self.wfile.flush()
                    self.close_connection = True
                    self.connection.shutdown(socket.SHUT_WR)
                    return
                if scenario == 'invalid_wav_pending':
                    self.wfile.write(b'NOPE' + WAV[4:9000])
                    self.wfile.flush()
                    time.sleep(1.2)
                    self.wfile.write(WAV[9000:])
                    self.wfile.flush()
                    return
                if scenario == 'invalid_wav':
                    self.wfile.write(b'NOPE' + WAV[4:])
                    self.wfile.flush()
                    return
                if scenario in ('riff_short', 'riff_long'):
                    size = len(WAV) - 8 + (-2 if scenario == 'riff_short' else 2)
                    self.wfile.write(WAV[:4] + struct.pack('<I', size) + WAV[8:])
                    self.wfile.flush()
                    return
                if scenario in ('early', 'abort'):
                    # Fragment the header independently from PCM. Keep the final
                    # bytes unsent until after the first callback's deadline.
                    for byte in WAV[:44]:
                        self.wfile.write(bytes([byte]))
                        self.wfile.flush()
                    self.wfile.write(WAV[44:9000])
                    self.wfile.flush()
                    time.sleep(1.2)
                    self.wfile.write(WAV[9000:])
                else:
                    self.wfile.write(WAV)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass  # Expected when the callback or cancellation aborts HTTP.

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
    def run_case(self, scenario, mode='success', endpoint=None):
        with engine(scenario) as (port, requests):
            result = subprocess.run([CLIENT, str(port), mode], capture_output=True,
                                    text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            if endpoint:
                self.assertIn(endpoint, requests)
                other = '/synthesis' if endpoint == '/streaming_synthesis' else '/streaming_synthesis'
                self.assertNotIn(other, requests)
            return requests

    def test_pcm_arrives_before_response_finishes(self):
        self.run_case('early', 'early', '/streaming_synthesis')

    def test_supported_engine(self):
        self.run_case('supported', endpoint='/streaming_synthesis')

    def test_legacy_engines(self):
        for scenario in ('missing', 'disabled', 'manifest404'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, endpoint='/synthesis')

    def test_manifest_failures_are_not_legacy(self):
        for scenario in ('manifest500', 'manifest_invalid',
                         'manifest_invalid_root', 'manifest_invalid_features'):
            with self.subTest(scenario=scenario):
                requests = self.run_case(scenario, 'error')
                self.assertNotIn('/synthesis', requests)
                self.assertNotIn('/streaming_synthesis', requests)

    def test_truncation_and_invalid_wav_fail_without_retry(self):
        for scenario in ('truncated', 'invalid_wav', 'riff_short', 'riff_long'):
            with self.subTest(scenario=scenario):
                self.run_case(scenario, 'error', '/streaming_synthesis')

    def test_decoder_error_interrupts_pending_body(self):
        self.run_case('invalid_wav_pending', 'early_error', '/streaming_synthesis')

    def test_cancel_interrupts_body_wait(self):
        self.run_case('cancel', 'cancel', '/streaming_synthesis')

    def test_callback_false_stops_before_response_end(self):
        self.run_case('abort', 'abort', '/streaming_synthesis')


if __name__ == '__main__':
    unittest.main()
