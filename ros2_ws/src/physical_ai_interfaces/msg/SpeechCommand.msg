# header.stamp is publication time; timestamp_ns preserves the microphone/ASR
# event time used for traceability through the speech-to-task pipeline.
std_msgs/Header header
string transcript
string detected_language
float32 confidence
uint64 timestamp_ns
