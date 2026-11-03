# speak_ros_interfaces

### PCM stream contract

`AudioChunk` carries frame-aligned raw PCM, independent of the synthesis backend
or transport. `stream_id` identifies one utterance and stays constant across
arbitrary delays. `is_final` marks normal completion and may accompany empty data.
A failed or cancelled stream need not emit a normal completion marker. Consumers
must also handle a new ID and explicit stop commands without waiting indefinitely
for a final marker. Topic output completion means publication, not speaker playback.

The added fields require rebuilding all publishers and subscribers together.
