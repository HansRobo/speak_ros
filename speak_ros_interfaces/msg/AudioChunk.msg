builtin_interfaces/Time timestamp
uint8[] data
uint32 sample_rate
uint8 channels
string encoding

# One identifier per utterance, independent of the synthesis backend.
string stream_id
# Normal end of stream; data may be empty.
bool is_final
