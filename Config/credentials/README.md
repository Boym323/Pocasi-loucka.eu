# Credentials

1. Copy `credentials.example.h` to `credentials.h`.
2. Fill in Wi-Fi, MQTT and painlessMesh credentials.
3. Keep `credentials.h` private; it is ignored by the repository.

The main unit protects `/data`, `/reboot` and the OTA page `/update` with
`WEB_USERNAME` / `WEB_PASSWORD`. For backward compatibility, builds that do
not define these macros use the existing MQTT username and password.
