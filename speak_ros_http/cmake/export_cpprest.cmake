# Retain imported targets and the dependencies used by cpprest's public headers.
list(APPEND speak_ros_http_LIBRARIES cpprestsdk::cpprest OpenSSL::SSL OpenSSL::Crypto)
