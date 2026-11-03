# speak_ros_http

HTTP/PCM utilities shared by network synthesis plugins. The `speak_ros` core
keeps no cpprestsdk dependency; plugins opt into this package.

- `HttpStream` owns one synthesis operation's HTTP client and cancellation watcher.
  It issues caller-built requests and incrementally consumes response bodies.
  A false return from `consume` means cancellation or consumer rejection; only
  a true return permits the provider decoder to signal normal completion.
- `streamingConfig` sets a 4096-byte HTTP read size. Providers retain control of
  redirects, certificate validation, authentication and endpoint policy.
- `PcmStream` converts arbitrary byte fragments to frame-aligned PCM callbacks,
  retaining only an incomplete frame. `finish` rejects incomplete frames and
  sends the normal completion marker, including for empty audio.

JSON request/response schemas, HTTP status handling, WAV/container validation,
model settings and termination rules remain provider-owned. These helpers are
not a provider interface and do not interpret protocol events.

PCM staging and decoder buffers are bounded locally; cpprestsdk's internal
response buffering is not covered by that guarantee. Existing provider HTTP
integration suites validate early delivery, transport failures and cancellation;
the package's PCM contract test additionally checks mono/stereo alignment across
arbitrary fragment sizes and consumer rejection.
